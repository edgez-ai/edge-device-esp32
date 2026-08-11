#include "usb_control_transport.h"

#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#include "ble_control.h"
#include "driver/uart.h"
#include "edgez_frame_protocol.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "halow_sync_bridge.h"
#include "log_stream_protocol.h"

#define USB_RX_BUFFER_SIZE (EDGEZ_FRAME_MAX_LEN * 64)
#define USB_IO_TIMEOUT_MS 1000
#define USB_HOST_ACTIVITY_TIMEOUT_MS (25U * 1000U)
#define USB_GAP_MIN_MS 1
#define USB_GAP_INITIAL_MS 3
#define USB_GAP_MAX_MS 10
#define USB_PROTOCOL_HEADER_LEN 3
#define USB_LEGACY_HEADER_LEN 8
#define USB_LEGACY_MAX_PAYLOAD 256
#define USB_LEGACY_VERSION 1
#define USB_LEGACY_ECHO_REQUEST 1
#define USB_LEGACY_ECHO_RESPONSE 2
#define USB_LEGACY_TX_ACK 3
#define USB_LEGACY_FLOW_CONTROL 4
#define USB_LEGACY_EXIT_CONTROL 8
#define USB_LEGACY_EXIT_CONTROL_RESPONSE 9
#define USB_HANDSHAKE_NONCE_LEN 16
#define USB_HANDSHAKE_LOG_LEVEL_LEN 1
#define USB_HANDSHAKE_PAYLOAD_LEN \
    (USB_HANDSHAKE_NONCE_LEN + USB_HANDSHAKE_LOG_LEVEL_LEN)
#define USB_UART_PORT UART_NUM_0
#define USB_UART_BAUD_RATE 921600
#define USB_UART_RX_BUFFER_SIZE 65536
#define USB_UART_TX_BUFFER_SIZE 32768
#define USB_RX_TASK_STACK_SIZE 16384
#define USB_QUEUE_TASK_STACK_SIZE 16384
#define USB_CONTROL_QUEUE_FRAMES 64
#define USB_REALTIME_QUEUE_FRAMES 256
#define USB_LOG_LINE_MAX_LEN USB_LEGACY_MAX_PAYLOAD
#define USB_STREAM_MAGIC_0 0x94
#define USB_STREAM_MAGIC_1 0xC3
#define USB_STREAM_HEADER_LEN 4

typedef enum {
    USB_MODE_LOG = 0,
    USB_MODE_ENTERING_CONTROL = 1,
    USB_MODE_CONTROL = 2,
} usb_transport_mode_t;

static const uint8_t s_voice_magic[USB_PROTOCOL_HEADER_LEN] = {'V', 'C', 2};
static const uint8_t s_speed_magic[USB_PROTOCOL_HEADER_LEN] = {'S', 'T', 2};
static const char *TAG = "edgez_usb";

static void usb_apply_log_exclusions(void)
{
    /* Per-notification NimBLE GATT traces are transport implementation noise,
     * not device diagnostics. Keep its warnings/errors at every global level. */
    esp_log_level_set("NimBLE", ESP_LOG_WARN);
}
typedef struct {
    uint16_t len;
    uint8_t payload[EDGEZ_FRAME_MAX_PAYLOAD];
} usb_queued_frame_t;

typedef struct {
    usb_queued_frame_t *frames;
    size_t capacity;
    size_t head;
    size_t tail;
    SemaphoreHandle_t lock;
    SemaphoreHandle_t items;
    SemaphoreHandle_t spaces;
} usb_frame_queue_t;

static SemaphoreHandle_t s_tx_lock;
static uint8_t *s_rx_buffer;
static size_t s_rx_len;
static bool s_uart_initialized;
static atomic_int s_transport_mode = ATOMIC_VAR_INIT(USB_MODE_LOG);
static atomic_uint_fast32_t s_last_host_activity_ms;
static vprintf_like_t s_console_vprintf;
static uint8_t s_inter_frame_gap_ms = USB_GAP_INITIAL_MS;
static uint16_t s_low_pressure_frames;
static usb_frame_queue_t s_mobile_to_halow_control_queue;
static usb_frame_queue_t s_mobile_to_halow_realtime_queue;
static usb_frame_queue_t s_halow_to_mobile_control_queue;
static usb_frame_queue_t s_halow_to_mobile_realtime_queue;
static SemaphoreHandle_t s_mobile_to_halow_work;
static SemaphoreHandle_t s_halow_to_mobile_work;
static TaskHandle_t s_mobile_to_halow_task;
static TaskHandle_t s_halow_to_mobile_task;

static bool usb_frame_queue_init(usb_frame_queue_t *queue, size_t capacity)
{
    if (!queue || capacity == 0) return false;
    memset(queue, 0, sizeof(*queue));
    queue->frames = (usb_queued_frame_t *)heap_caps_calloc(
        capacity, sizeof(usb_queued_frame_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!queue->frames) return false;
    queue->capacity = capacity;
    queue->lock = xSemaphoreCreateMutex();
    queue->items = xSemaphoreCreateCounting((UBaseType_t)capacity, 0);
    queue->spaces = xSemaphoreCreateCounting((UBaseType_t)capacity,
                                             (UBaseType_t)capacity);
    if (!queue->lock || !queue->items || !queue->spaces) {
        if (queue->lock) vSemaphoreDelete(queue->lock);
        if (queue->items) vSemaphoreDelete(queue->items);
        if (queue->spaces) vSemaphoreDelete(queue->spaces);
        heap_caps_free(queue->frames);
        memset(queue, 0, sizeof(*queue));
        return false;
    }
    return true;
}

static void usb_frame_queue_deinit(usb_frame_queue_t *queue)
{
    if (!queue) return;
    if (queue->lock) vSemaphoreDelete(queue->lock);
    if (queue->items) vSemaphoreDelete(queue->items);
    if (queue->spaces) vSemaphoreDelete(queue->spaces);
    if (queue->frames) heap_caps_free(queue->frames);
    memset(queue, 0, sizeof(*queue));
}

static bool usb_frame_queue_push(usb_frame_queue_t *queue,
                                 const uint8_t *payload,
                                 uint16_t payload_len,
                                 TickType_t wait)
{
    if (!queue || !queue->frames || !payload || payload_len == 0 ||
        payload_len > EDGEZ_FRAME_MAX_PAYLOAD) {
        return false;
    }
    if (xSemaphoreTake(queue->spaces, wait) != pdTRUE) return false;
    if (xSemaphoreTake(queue->lock, wait) != pdTRUE) {
        xSemaphoreGive(queue->spaces);
        return false;
    }
    usb_queued_frame_t *slot = &queue->frames[queue->tail];
    slot->len = payload_len;
    memcpy(slot->payload, payload, payload_len);
    queue->tail = (queue->tail + 1) % queue->capacity;
    xSemaphoreGive(queue->lock);
    xSemaphoreGive(queue->items);
    return true;
}

static bool usb_frame_queue_pop(usb_frame_queue_t *queue,
                                usb_queued_frame_t *out,
                                TickType_t wait)
{
    if (!queue || !out || !queue->frames) return false;
    if (xSemaphoreTake(queue->items, wait) != pdTRUE) return false;
    if (xSemaphoreTake(queue->lock, portMAX_DELAY) != pdTRUE) {
        xSemaphoreGive(queue->items);
        return false;
    }
    *out = queue->frames[queue->head];
    queue->head = (queue->head + 1) % queue->capacity;
    xSemaphoreGive(queue->lock);
    xSemaphoreGive(queue->spaces);
    return true;
}

static bool usb_priority_queue_push(usb_frame_queue_t *queue,
                                    SemaphoreHandle_t work,
                                    const uint8_t *payload,
                                    uint16_t payload_len,
                                    TickType_t wait)
{
    if (!work || !usb_frame_queue_push(queue, payload, payload_len, wait)) {
        return false;
    }
    xSemaphoreGive(work);
    return true;
}

static void usb_transport_queues_deinit(void)
{
    usb_frame_queue_deinit(&s_mobile_to_halow_control_queue);
    usb_frame_queue_deinit(&s_mobile_to_halow_realtime_queue);
    usb_frame_queue_deinit(&s_halow_to_mobile_control_queue);
    usb_frame_queue_deinit(&s_halow_to_mobile_realtime_queue);
    if (s_mobile_to_halow_work) {
        vSemaphoreDelete(s_mobile_to_halow_work);
        s_mobile_to_halow_work = NULL;
    }
    if (s_halow_to_mobile_work) {
        vSemaphoreDelete(s_halow_to_mobile_work);
        s_halow_to_mobile_work = NULL;
    }
}

static uint16_t read_le16(const uint8_t *value)
{
    return (uint16_t)value[0] | ((uint16_t)value[1] << 8);
}

static void write_le16(uint8_t *value, uint16_t data)
{
    value[0] = (uint8_t)data;
    value[1] = (uint8_t)(data >> 8);
}

bool usb_control_transport_is_connected(void)
{
    return s_uart_initialized &&
        atomic_load_explicit(&s_transport_mode, memory_order_acquire) ==
            USB_MODE_CONTROL;
}

static uint32_t usb_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void usb_control_transport_exit(void)
{
    if (!s_uart_initialized) return;
    bool disconnected = false;
    if (s_tx_lock &&
        xSemaphoreTake(s_tx_lock, pdMS_TO_TICKS(USB_IO_TIMEOUT_MS)) == pdTRUE) {
        (void)uart_wait_tx_done(USB_UART_PORT,
                               pdMS_TO_TICKS(USB_IO_TIMEOUT_MS));
        atomic_store_explicit(&s_transport_mode, USB_MODE_LOG,
                              memory_order_release);
        xSemaphoreGive(s_tx_lock);
        disconnected = true;
    }
    if (disconnected) {
        if (ble_control_is_connected()) {
            ble_control_cap_log_level_for_ble();
        }
        halow_sync_bridge_note_usb_connected(false);
    }
}

static bool usb_write_raw_locked(const uint8_t *data, size_t len)
{
    if (!data || len == 0 || !s_uart_initialized) {
        return false;
    }
    size_t written = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(USB_IO_TIMEOUT_MS);
    while (written < len && xTaskGetTickCount() < deadline) {
        int count = uart_write_bytes(USB_UART_PORT, data + written, len - written);
        if (count <= 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            written += count;
        }
    }
    if (written == len) {
        TickType_t now = xTaskGetTickCount();
        TickType_t remaining = now < deadline ? deadline - now : 0;
        if (uart_wait_tx_done(USB_UART_PORT, remaining) != ESP_OK) {
            written = 0;
        } else {
            vTaskDelay(pdMS_TO_TICKS(s_inter_frame_gap_ms));
        }
    }
    return written == len;
}

static bool usb_write_raw(const uint8_t *data, size_t len)
{
    if (!data || len == 0 || !usb_control_transport_is_connected() || !s_tx_lock) {
        return false;
    }
    if (xSemaphoreTake(s_tx_lock, pdMS_TO_TICKS(USB_IO_TIMEOUT_MS)) != pdTRUE) {
        return false;
    }
    bool ok = usb_write_raw_locked(data, len);
    xSemaphoreGive(s_tx_lock);
    return ok;
}

static bool usb_write_stream_payload_locked(const uint8_t *payload,
                                            uint16_t payload_len)
{
    if (!payload || payload_len == 0 || payload_len > EDGEZ_FRAME_MAX_PAYLOAD) {
        return false;
    }
    uint8_t frame[USB_STREAM_HEADER_LEN + EDGEZ_FRAME_MAX_PAYLOAD];
    frame[0] = USB_STREAM_MAGIC_0;
    frame[1] = USB_STREAM_MAGIC_1;
    frame[2] = (uint8_t)(payload_len >> 8);
    frame[3] = (uint8_t)payload_len;
    memcpy(frame + USB_STREAM_HEADER_LEN, payload, payload_len);
    return usb_write_raw_locked(frame, USB_STREAM_HEADER_LEN + payload_len);
}

static bool usb_write_stream_payload(const uint8_t *payload, uint16_t payload_len)
{
    if (!payload || payload_len == 0 || payload_len > EDGEZ_FRAME_MAX_PAYLOAD) {
        return false;
    }
    uint8_t frame[USB_STREAM_HEADER_LEN + EDGEZ_FRAME_MAX_PAYLOAD];
    frame[0] = USB_STREAM_MAGIC_0;
    frame[1] = USB_STREAM_MAGIC_1;
    frame[2] = (uint8_t)(payload_len >> 8);
    frame[3] = (uint8_t)payload_len;
    memcpy(frame + USB_STREAM_HEADER_LEN, payload, payload_len);
    return usb_write_raw(frame, USB_STREAM_HEADER_LEN + payload_len);
}

static void usb_send_payload(const uint8_t *prefix, size_t prefix_len,
                             const uint8_t *payload, uint16_t payload_len,
                             TickType_t wait)
{
    if (!usb_control_transport_is_connected()) return;
    size_t framed_payload_len = prefix_len + payload_len;
    if ((!payload && payload_len > 0) || framed_payload_len > EDGEZ_FRAME_MAX_PAYLOAD) {
        return;
    }
    uint8_t frame[EDGEZ_FRAME_MAX_PAYLOAD];
    if (prefix_len > 0) memcpy(frame, prefix, prefix_len);
    if (payload_len > 0) {
        memcpy(frame + prefix_len, payload, payload_len);
    }
    usb_frame_queue_t *queue = prefix_len == 0
        ? &s_halow_to_mobile_control_queue
        : &s_halow_to_mobile_realtime_queue;
    if (!usb_priority_queue_push(queue, s_halow_to_mobile_work, frame,
                                 (uint16_t)framed_payload_len, wait)) {
        ESP_LOGW(TAG, "HaLow->mobile PSRAM queue full len=%u",
                 (unsigned)framed_payload_len);
    }
}

void usb_control_transport_send_frame(const uint8_t *payload, uint16_t payload_len)
{
    usb_send_payload(NULL, 0, payload, payload_len, portMAX_DELAY);
}

void usb_control_transport_send_voice_frame(const uint8_t *payload, uint16_t payload_len)
{
    /* Control, voice, and speed share the same ordered HaLow->mobile FIFO and
     * the same back-pressure policy. Do not turn local UART congestion into
     * apparent radio packet loss for any stream. */
    usb_send_payload(s_voice_magic, sizeof(s_voice_magic), payload, payload_len,
                     portMAX_DELAY);
}

void usb_control_transport_send_log_frame(const uint8_t *payload,
                                          uint16_t payload_len)
{
    if (!usb_control_transport_is_connected() || !payload || payload_len == 0 ||
        payload_len > EDGEZ_FRAME_MAX_PAYLOAD) {
        return;
    }
    /* The shared mobile RX queue already decoupled the ESP log callback.
     * Keep this final USB enqueue non-blocking and below protobuf/control. */
    (void)usb_priority_queue_push(&s_halow_to_mobile_realtime_queue,
                                  s_halow_to_mobile_work,
                                  payload, payload_len, 0);
}

static void usb_send_legacy_echo(uint8_t type, uint16_t sequence,
                                 const uint8_t *payload, uint16_t payload_len)
{
    if (payload_len > USB_LEGACY_MAX_PAYLOAD) return;
    uint8_t frame[USB_LEGACY_HEADER_LEN + USB_LEGACY_MAX_PAYLOAD] = {
        EDGEZ_FRAME_MAGIC_0, EDGEZ_FRAME_MAGIC_1, USB_LEGACY_VERSION, type,
    };
    write_le16(&frame[4], sequence);
    write_le16(&frame[6], payload_len);
    if (payload_len > 0 && payload) memcpy(&frame[USB_LEGACY_HEADER_LEN], payload, payload_len);
    usb_write_stream_payload(frame, USB_LEGACY_HEADER_LEN + payload_len);
}

static bool usb_queue_log_record(const uint8_t *text,
                                 uint16_t text_len)
{
    if ((!text && text_len > 0) || text_len > USB_LOG_LINE_MAX_LEN) return false;
    uint8_t frame[EDGEZ_LOG_STREAM_HEADER_LEN + USB_LOG_LINE_MAX_LEN] = {
        EDGEZ_LOG_STREAM_MAGIC_0, EDGEZ_LOG_STREAM_MAGIC_1,
        EDGEZ_LOG_STREAM_VERSION, EDGEZ_LOG_STREAM_RECORD, 0,
    };
    if (text_len > 0) memcpy(&frame[EDGEZ_LOG_STREAM_HEADER_LEN], text, text_len);
    return halow_sync_bridge_queue_log_frame(
        frame, (uint16_t)(EDGEZ_LOG_STREAM_HEADER_LEN + text_len));
}

static bool usb_enter_control_mode_with_pong(uint16_t sequence,
                                             const uint8_t *nonce,
                                             uint16_t nonce_len)
{
    if (!s_uart_initialized || !s_tx_lock || !nonce ||
        nonce_len != USB_HANDSHAKE_PAYLOAD_LEN ||
        nonce[USB_HANDSHAKE_NONCE_LEN] > ESP_LOG_VERBOSE) {
        return false;
    }
    if (usb_control_transport_is_connected()) {
        usb_send_legacy_echo(USB_LEGACY_ECHO_RESPONSE,
                             sequence, nonce, nonce_len);
        return true;
    }

    int expected = USB_MODE_LOG;
    if (!atomic_compare_exchange_strong_explicit(
            &s_transport_mode, &expected, USB_MODE_ENTERING_CONTROL,
            memory_order_acq_rel, memory_order_acquire)) {
        return false;
    }

    /* The host carries its persisted log threshold with the random nonce, so
     * framed device and app logging start at the same level. */
    esp_log_level_set("*", (esp_log_level_t)nonce[USB_HANDSHAKE_NONCE_LEN]);
    usb_apply_log_exclusions();

    if (xSemaphoreTake(s_tx_lock, pdMS_TO_TICKS(USB_IO_TIMEOUT_MS)) != pdTRUE) {
        atomic_store_explicit(&s_transport_mode, USB_MODE_LOG,
                              memory_order_release);
        return false;
    }

    /* No new plain-text ESP logs are emitted in ENTERING_CONTROL. Drain text
     * already queued by the UART console, then make the nonce-bearing pong the
     * first framed record in the control session. */
    bool ok = uart_wait_tx_done(USB_UART_PORT,
                               pdMS_TO_TICKS(USB_IO_TIMEOUT_MS)) == ESP_OK;
    if (ok) {
        uint8_t pong[USB_LEGACY_HEADER_LEN + USB_LEGACY_MAX_PAYLOAD] = {
            EDGEZ_FRAME_MAGIC_0, EDGEZ_FRAME_MAGIC_1,
            USB_LEGACY_VERSION, USB_LEGACY_ECHO_RESPONSE,
        };
        write_le16(&pong[4], sequence);
        write_le16(&pong[6], nonce_len);
        if (nonce_len > 0 && nonce) {
            memcpy(&pong[USB_LEGACY_HEADER_LEN], nonce, nonce_len);
        }
        ok = usb_write_stream_payload_locked(
            pong, (uint16_t)(USB_LEGACY_HEADER_LEN + nonce_len));
    }
    atomic_store_explicit(&s_transport_mode,
                          ok ? USB_MODE_CONTROL : USB_MODE_LOG,
                          memory_order_release);
    xSemaphoreGive(s_tx_lock);
    if (ok) {
        atomic_store_explicit(&s_last_host_activity_ms, usb_now_ms(),
                              memory_order_release);
        halow_sync_bridge_note_usb_connected(true);
        halow_sync_bridge_set_log_stream_enabled(true);
        halow_sync_bridge_request_log_level_test();
    }
    return ok;
}

static int usb_log_vprintf(const char *format, va_list args)
{
    /* Submitting an LG2 BLE notification produces synchronous NimBLE INFO
     * diagnostics. Re-enqueuing those diagnostics would create an unbounded
     * notify -> log -> LG2 -> notify feedback loop and starve the GATT link. */
    if (halow_sync_bridge_log_delivery_active()) return 0;

    int mode = atomic_load_explicit(&s_transport_mode, memory_order_acquire);
    bool ble_connected = ble_control_is_connected();
    if (mode == USB_MODE_LOG && !ble_connected) {
        return s_console_vprintf ? s_console_vprintf(format, args) : 0;
    }
    if (mode == USB_MODE_ENTERING_CONTROL) return 0;

    uint8_t text[USB_LOG_LINE_MAX_LEN + 1];

    va_list copy;
    va_copy(copy, args);
    int formatted_len = vsnprintf((char *)text, sizeof(text), format, copy);
    va_end(copy);
    if (formatted_len < 0) return formatted_len;

    size_t text_len = (size_t)formatted_len;
    if (text_len > USB_LOG_LINE_MAX_LEN) text_len = USB_LOG_LINE_MAX_LEN;

    /* Logging must never block the task which produced the log. Control and
     * realtime traffic have priority; log records are dropped if their PSRAM
     * queue is full. Do not log this failure, since that would recurse. */
    bool queued = usb_queue_log_record(text, (uint16_t)text_len);
    if (mode == USB_MODE_LOG) {
        int console_len = s_console_vprintf
            ? s_console_vprintf(format, args)
            : formatted_len;
        return queued ? console_len : 0;
    }
    return queued ? formatted_len : 0;
}

static void usb_set_log_level(uint8_t requested_level,
                              const uint8_t *tag_data,
                              uint16_t tag_len)
{
    if (requested_level > ESP_LOG_VERBOSE || tag_len >= 64) {
        const uint8_t response[EDGEZ_LOG_STREAM_HEADER_LEN] = {
            EDGEZ_LOG_STREAM_MAGIC_0, EDGEZ_LOG_STREAM_MAGIC_1,
            EDGEZ_LOG_STREAM_VERSION, EDGEZ_LOG_STREAM_LEVEL_RESPONSE,
            EDGEZ_LOG_STREAM_LEVEL_ERROR,
        };
        (void)halow_sync_bridge_queue_log_frame(response, sizeof(response));
        return;
    }

    char tag[64] = "*";
    if (tag_len > 0 && tag_data) {
        memcpy(tag, tag_data, tag_len);
        tag[tag_len] = '\0';
    }
    uint16_t effective_level = requested_level;
    if (effective_level > CONFIG_LOG_MAXIMUM_LEVEL) {
        effective_level = CONFIG_LOG_MAXIMUM_LEVEL;
    }
    esp_log_level_set(tag, (esp_log_level_t)effective_level);
    usb_apply_log_exclusions();
    halow_sync_bridge_set_log_stream_enabled(true);
    uint16_t response_tag_len = (uint16_t)strlen(tag);
    uint8_t response[EDGEZ_LOG_STREAM_HEADER_LEN + sizeof(tag)] = {
        EDGEZ_LOG_STREAM_MAGIC_0, EDGEZ_LOG_STREAM_MAGIC_1,
        EDGEZ_LOG_STREAM_VERSION, EDGEZ_LOG_STREAM_LEVEL_RESPONSE,
        (uint8_t)effective_level,
    };
    memcpy(&response[EDGEZ_LOG_STREAM_HEADER_LEN], tag, response_tag_len);
    (void)halow_sync_bridge_queue_log_frame(
        response, (uint16_t)(EDGEZ_LOG_STREAM_HEADER_LEN + response_tag_len));
    halow_sync_bridge_request_log_level_test();
}

static void usb_update_mobile_gap_from_pressure(void)
{
    UBaseType_t depth = uxSemaphoreGetCount(s_mobile_to_halow_realtime_queue.items);
    uint8_t next = s_inter_frame_gap_ms;
    if (depth >= (USB_REALTIME_QUEUE_FRAMES * 3U) / 4U) {
        next = next < USB_GAP_MAX_MS ? next + 1 : USB_GAP_MAX_MS;
        s_low_pressure_frames = 0;
    } else if (depth <= USB_REALTIME_QUEUE_FRAMES / 4U) {
        if (++s_low_pressure_frames >= 64) {
            next = next > USB_GAP_MIN_MS ? next - 1 : USB_GAP_MIN_MS;
            s_low_pressure_frames = 0;
        }
    } else {
        s_low_pressure_frames = 0;
    }
    if (next != s_inter_frame_gap_ms) {
        s_inter_frame_gap_ms = next;
        usb_send_legacy_echo(USB_LEGACY_FLOW_CONTROL, next, NULL, 0);
    }
}

static void usb_send_response(void *ctx, const uint8_t *payload, uint16_t payload_len)
{
    (void)ctx;
    usb_control_transport_send_frame(payload, payload_len);
}

static void usb_process_payload(const uint8_t *payload, uint16_t payload_len)
{
    if (payload_len > USB_PROTOCOL_HEADER_LEN &&
        memcmp(payload, s_voice_magic, USB_PROTOCOL_HEADER_LEN) == 0) {
        esp_err_t err = halow_sync_bridge_handle_voice_to_radio(
            payload + USB_PROTOCOL_HEADER_LEN, payload_len - USB_PROTOCOL_HEADER_LEN);
        if (err == ESP_OK) {
            usb_send_legacy_echo(USB_LEGACY_TX_ACK, 0, NULL, 0);
        }
        return;
    }
    if (payload_len > USB_PROTOCOL_HEADER_LEN &&
        memcmp(payload, s_speed_magic, USB_PROTOCOL_HEADER_LEN) == 0) {
        (void)halow_sync_bridge_handle_speed_to_radio(
            payload + USB_PROTOCOL_HEADER_LEN, payload_len - USB_PROTOCOL_HEADER_LEN);
        usb_update_mobile_gap_from_pressure();
        return;
    }
    halow_sync_bridge_note_active_interface(HALOW_SYNC_ACTIVE_INTERFACE_USB);
    uint8_t frame[EDGEZ_FRAME_MAX_LEN];
    frame[0] = EDGEZ_FRAME_MAGIC_0;
    frame[1] = EDGEZ_FRAME_MAGIC_1;
    frame[2] = (uint8_t)payload_len;
    frame[3] = (uint8_t)(payload_len >> 8);
    memcpy(frame + EDGEZ_FRAME_HEADER_LEN, payload, payload_len);
    edgez_frame_protocol_handle_frame(frame, EDGEZ_FRAME_HEADER_LEN + payload_len,
                                      usb_send_response, NULL, false);
    /* Match BLE write-completion semantics: once a complete application frame
     * has been accepted by the protocol layer, tell the host it may send the
     * next frame. Peer/radio delivery is acknowledged separately by the
     * conversation protocol. */
    usb_send_legacy_echo(USB_LEGACY_TX_ACK, 0, NULL, 0);
}

static void usb_mobile_to_halow_task(void *arg)
{
    (void)arg;
    usb_queued_frame_t frame;
    for (;;) {
        if (xSemaphoreTake(s_mobile_to_halow_work, portMAX_DELAY) != pdTRUE) continue;
        /* Protobuf/control always wins when both queues contain work. */
        if (!usb_frame_queue_pop(&s_mobile_to_halow_control_queue, &frame, 0) &&
            !usb_frame_queue_pop(&s_mobile_to_halow_realtime_queue, &frame, 0)) continue;
        usb_process_payload(frame.payload, frame.len);
    }
}

static void usb_halow_to_mobile_task(void *arg)
{
    (void)arg;
    usb_queued_frame_t frame;
    for (;;) {
        if (xSemaphoreTake(s_halow_to_mobile_work, portMAX_DELAY) != pdTRUE) continue;
        /* Protobuf/control always wins when both queues contain work. */
        if (!usb_frame_queue_pop(&s_halow_to_mobile_control_queue, &frame, 0) &&
            !usb_frame_queue_pop(&s_halow_to_mobile_realtime_queue, &frame, 0)) continue;
        if (!usb_control_transport_is_connected()) continue;
        bool is_log = frame.len >= EDGEZ_LOG_STREAM_HEADER_LEN &&
                      frame.payload[0] == EDGEZ_LOG_STREAM_MAGIC_0 &&
                      frame.payload[1] == EDGEZ_LOG_STREAM_MAGIC_1 &&
                      frame.payload[2] == EDGEZ_LOG_STREAM_VERSION;
        if (!usb_write_stream_payload(frame.payload, frame.len) && !is_log) {
            ESP_LOGW(TAG, "HaLow->mobile UART write failed len=%u",
                     (unsigned)frame.len);
        }
    }
}

static void usb_consume_rx(void)
{
    while (s_rx_len >= 2) {
        if (s_rx_buffer[0] != USB_STREAM_MAGIC_0 ||
            s_rx_buffer[1] != USB_STREAM_MAGIC_1) {
            memmove(s_rx_buffer, s_rx_buffer + 1, --s_rx_len);
            continue;
        }
        if (s_rx_len < USB_STREAM_HEADER_LEN) return;
        uint16_t payload_len = ((uint16_t)s_rx_buffer[2] << 8) | s_rx_buffer[3];
        if (payload_len == 0 || payload_len > EDGEZ_FRAME_MAX_PAYLOAD) {
            memmove(s_rx_buffer, s_rx_buffer + 1, --s_rx_len);
            continue;
        }
        uint16_t frame_len = USB_STREAM_HEADER_LEN + payload_len;
        if (s_rx_len < frame_len) return;
        const uint8_t *payload = s_rx_buffer + USB_STREAM_HEADER_LEN;
        atomic_store_explicit(&s_last_host_activity_ms, usb_now_ms(),
                              memory_order_release);
        if (payload_len >= EDGEZ_LOG_STREAM_HEADER_LEN &&
            payload[0] == EDGEZ_LOG_STREAM_MAGIC_0 &&
            payload[1] == EDGEZ_LOG_STREAM_MAGIC_1 &&
            payload[2] == EDGEZ_LOG_STREAM_VERSION &&
            payload[3] == EDGEZ_LOG_STREAM_SET_LEVEL) {
            if (usb_control_transport_is_connected()) {
                usb_set_log_level(
                    payload[4],
                    &payload[EDGEZ_LOG_STREAM_HEADER_LEN],
                    payload_len - EDGEZ_LOG_STREAM_HEADER_LEN);
            }
        } else if (payload_len >= USB_LEGACY_HEADER_LEN &&
            payload[0] == EDGEZ_FRAME_MAGIC_0 &&
            payload[1] == EDGEZ_FRAME_MAGIC_1 &&
            payload[2] == USB_LEGACY_VERSION &&
            (payload[3] == USB_LEGACY_ECHO_REQUEST ||
             payload[3] == USB_LEGACY_ECHO_RESPONSE ||
             payload[3] == USB_LEGACY_FLOW_CONTROL ||
             payload[3] == USB_LEGACY_EXIT_CONTROL)) {
            uint16_t echo_len = read_le16(payload + 6);
            if (echo_len <= USB_LEGACY_MAX_PAYLOAD &&
                payload_len == USB_LEGACY_HEADER_LEN + echo_len) {
                /* A valid frame received from the host is the UART bridge's
                 * connection signal. UART driver installation alone is not. */
                if (payload[3] == USB_LEGACY_ECHO_REQUEST) {
                    if (usb_enter_control_mode_with_pong(
                            read_le16(payload + 4),
                            payload + USB_LEGACY_HEADER_LEN,
                            echo_len)) {
                        halow_sync_bridge_note_active_interface(
                            HALOW_SYNC_ACTIVE_INTERFACE_USB);
                    }
                } else if (payload[3] == USB_LEGACY_FLOW_CONTROL) {
                    if (!usb_control_transport_is_connected()) {
                        goto consume_frame;
                    }
                    uint16_t requested = read_le16(payload + 4);
                    if (requested >= USB_GAP_MIN_MS && requested <= USB_GAP_MAX_MS) {
                        s_inter_frame_gap_ms = (uint8_t)requested;
                    }
                } else if (payload[3] == USB_LEGACY_EXIT_CONTROL) {
                    if (usb_control_transport_is_connected()) {
                        usb_send_legacy_echo(USB_LEGACY_EXIT_CONTROL_RESPONSE,
                                             read_le16(payload + 4), NULL, 0);
                        usb_control_transport_exit();
                    }
                }
            }
        } else {
            /* Only the nonce-bearing ping/pong handshake may claim UART TX.
             * Ignore protobuf and realtime frames received in log mode. */
            if (!usb_control_transport_is_connected()) goto consume_frame;
            halow_sync_bridge_note_active_interface(
                HALOW_SYNC_ACTIVE_INTERFACE_USB);
            const bool realtime = payload_len > USB_PROTOCOL_HEADER_LEN &&
                (memcmp(payload, s_voice_magic, USB_PROTOCOL_HEADER_LEN) == 0 ||
                 memcmp(payload, s_speed_magic, USB_PROTOCOL_HEADER_LEN) == 0);
            usb_frame_queue_t *queue = realtime
                ? &s_mobile_to_halow_realtime_queue
                : &s_mobile_to_halow_control_queue;
            if (!usb_priority_queue_push(queue, s_mobile_to_halow_work,
                                         payload, payload_len, portMAX_DELAY)) {
                ESP_LOGW(TAG, "Mobile->HaLow PSRAM queue full len=%u",
                         (unsigned)payload_len);
            }
        }
consume_frame:
        s_rx_len -= frame_len;
        if (s_rx_len > 0) memmove(s_rx_buffer, s_rx_buffer + frame_len, s_rx_len);
    }
}

static void usb_rx_task(void *arg)
{
    (void)arg;
    uint8_t chunk[2048];
    for (;;) {
        int count = uart_read_bytes(USB_UART_PORT, chunk, sizeof(chunk),
                                    pdMS_TO_TICKS(10));
        if (count > 0) {
            if (s_rx_len + count > USB_RX_BUFFER_SIZE) s_rx_len = 0;
            memcpy(s_rx_buffer + s_rx_len, chunk, count);
            s_rx_len += count;
            usb_consume_rx();
        } else if (usb_control_transport_is_connected()) {
            uint32_t last_activity = atomic_load_explicit(
                &s_last_host_activity_ms, memory_order_acquire);
            if ((uint32_t)(usb_now_ms() - last_activity) >=
                USB_HOST_ACTIVITY_TIMEOUT_MS) {
                ESP_LOGW(TAG,
                         "USB host activity timed out; starting mobile disconnect grace period");
                usb_control_transport_exit();
            }
        }
    }
}

esp_err_t usb_control_transport_init(void)
{
    atomic_store_explicit(&s_transport_mode, USB_MODE_LOG,
                          memory_order_release);
    atomic_store_explicit(&s_last_host_activity_ms, 0, memory_order_release);
    s_inter_frame_gap_ms = USB_GAP_INITIAL_MS;
    s_low_pressure_frames = 0;
    s_tx_lock = xSemaphoreCreateMutex();
    if (!s_tx_lock) return ESP_ERR_NO_MEM;
    s_rx_buffer = (uint8_t *)heap_caps_malloc(
        USB_RX_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_rx_buffer) {
        vSemaphoreDelete(s_tx_lock);
        s_tx_lock = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_mobile_to_halow_work = xSemaphoreCreateCounting(
        USB_CONTROL_QUEUE_FRAMES + USB_REALTIME_QUEUE_FRAMES, 0);
    s_halow_to_mobile_work = xSemaphoreCreateCounting(
        USB_CONTROL_QUEUE_FRAMES + USB_REALTIME_QUEUE_FRAMES, 0);
    if (!s_mobile_to_halow_work || !s_halow_to_mobile_work ||
        !usb_frame_queue_init(&s_mobile_to_halow_control_queue, USB_CONTROL_QUEUE_FRAMES) ||
        !usb_frame_queue_init(&s_mobile_to_halow_realtime_queue, USB_REALTIME_QUEUE_FRAMES) ||
        !usb_frame_queue_init(&s_halow_to_mobile_control_queue, USB_CONTROL_QUEUE_FRAMES) ||
        !usb_frame_queue_init(&s_halow_to_mobile_realtime_queue, USB_REALTIME_QUEUE_FRAMES)) {
        usb_transport_queues_deinit();
        heap_caps_free(s_rx_buffer);
        s_rx_buffer = NULL;
        vSemaphoreDelete(s_tx_lock);
        s_tx_lock = NULL;
        return ESP_ERR_NO_MEM;
    }

    const uart_config_t uart_config = {
        .baud_rate = USB_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_param_config(USB_UART_PORT, &uart_config);
    if (err == ESP_OK) {
        err = uart_set_pin(USB_UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err == ESP_OK && !uart_is_driver_installed(USB_UART_PORT)) {
        err = uart_driver_install(USB_UART_PORT, USB_UART_RX_BUFFER_SIZE,
                                  USB_UART_TX_BUFFER_SIZE, 0, NULL, 0);
    }
    if (err != ESP_OK) {
        usb_transport_queues_deinit();
        heap_caps_free(s_rx_buffer);
        s_rx_buffer = NULL;
        vSemaphoreDelete(s_tx_lock);
        s_tx_lock = NULL;
        return err;
    }
    s_uart_initialized = true;
    s_console_vprintf = esp_log_set_vprintf(usb_log_vprintf);
    if (xTaskCreate(usb_mobile_to_halow_task, "edgez_usb_to_halow",
                    USB_QUEUE_TASK_STACK_SIZE, NULL, 8,
                    &s_mobile_to_halow_task) != pdPASS ||
        xTaskCreate(usb_halow_to_mobile_task, "edgez_halow_to_usb",
                    USB_QUEUE_TASK_STACK_SIZE, NULL, 8,
                    &s_halow_to_mobile_task) != pdPASS) {
        s_uart_initialized = false;
        esp_log_set_vprintf(s_console_vprintf);
        s_console_vprintf = NULL;
        if (s_mobile_to_halow_task) {
            vTaskDelete(s_mobile_to_halow_task);
            s_mobile_to_halow_task = NULL;
        }
        if (s_halow_to_mobile_task) {
            vTaskDelete(s_halow_to_mobile_task);
            s_halow_to_mobile_task = NULL;
        }
        usb_transport_queues_deinit();
        heap_caps_free(s_rx_buffer);
        s_rx_buffer = NULL;
        vSemaphoreDelete(s_tx_lock);
        s_tx_lock = NULL;
        return ESP_ERR_NO_MEM;
    }
    /* Echo handling is shallow, but EdgeZ protobuf dispatch enters the full
     * mesh bridge and its nanopb/status paths. A 4 KiB stack survives ping/pong
     * then overflows on the first init packet, rebooting before authorization
     * can be retained. Match the mesh task stack class for protocol traffic. */
    if (xTaskCreate(usb_rx_task, "edgez_usb_rx", USB_RX_TASK_STACK_SIZE,
                    NULL, 8, NULL) != pdPASS) {
        s_uart_initialized = false;
        esp_log_set_vprintf(s_console_vprintf);
        s_console_vprintf = NULL;
        vTaskDelete(s_mobile_to_halow_task);
        vTaskDelete(s_halow_to_mobile_task);
        s_mobile_to_halow_task = NULL;
        s_halow_to_mobile_task = NULL;
        usb_transport_queues_deinit();
        heap_caps_free(s_rx_buffer);
        s_rx_buffer = NULL;
        vSemaphoreDelete(s_tx_lock);
        s_tx_lock = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
