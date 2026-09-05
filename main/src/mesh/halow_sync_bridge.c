#include "halow_sync_bridge.h"

#include <math.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#include "edgez_platform.h"
#include "edgez_frame_protocol.h"
#include "usb_control_transport.h"
#include "esp_app_desc.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_sleep.h"
#include "factory_data.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#ifdef CONFIG_EDGEZ_SENSOR_SCRIPTING

#endif
#include "mbedtls/gcm.h"
#include "mbedtls/base64.h"
#include "mbedtls/sha256.h"
#include "nvs.h"
#include "pb_decode.h"
#include "pb_encode.h"




#ifdef CONFIG_ENABLE_MM_HALOW
#include "mmwlan.h"
#endif

static const char *TAG = "halow_mesh_api";
static const char *SPEED_TAG = "speed_test";

#ifdef CONFIG_MM_MESH_DEBUG_LOG
#define MESH_DEBUG_LOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define MESH_DEBUG_LOGI(...) do {} while (0)
#endif
enum {
    EDGEZ_ROUTE_MESSAGE_ID_LEN = 16,
    EDGEZ_ROUTE_SEQUENCE_LEN = 4,
    EDGEZ_ROUTE_MAC_LEN = 6,
    EDGEZ_ROUTE_SEQUENCE_OFFSET = EDGEZ_ROUTE_MESSAGE_ID_LEN,
    EDGEZ_ROUTE_FROM_OFFSET = EDGEZ_ROUTE_SEQUENCE_OFFSET + EDGEZ_ROUTE_SEQUENCE_LEN,
    EDGEZ_ROUTE_TO_OFFSET = EDGEZ_ROUTE_FROM_OFFSET + 8,
    EDGEZ_ROUTE_MAX_HOP_OFFSET = EDGEZ_ROUTE_TO_OFFSET + 8,
    EDGEZ_ROUTE_HOP_OFFSET = EDGEZ_ROUTE_MAX_HOP_OFFSET + 1,
    EDGEZ_ROUTE_PREFIX_LEN = EDGEZ_ROUTE_HOP_OFFSET + 1,
    EDGEZ_GLOBAL_BUFFER_TARGET_TABLE_SIZE = 16,
    EDGEZ_SEEN_MESSAGE_TABLE_SIZE = 32,
    EDGEZ_DELIVERY_DEDUPE_TABLE_SIZE = 32,
    EDGEZ_DELIVERY_DEDUPE_WINDOW_MS = 5000,
    EDGEZ_GLOBAL_BUFFER_CHUNK_SIZE = 220,
    EDGEZ_CONVERSATION_CHUNK_MAGIC_SIZE = 3,
    EDGEZ_CONVERSATION_CHUNK_HEADER_SIZE = EDGEZ_CONVERSATION_CHUNK_MAGIC_SIZE + 8 + 4 + 2 + 2 + 1,
    EDGEZ_GLOBAL_BUFFER_CHUNK_PAYLOAD_SIZE = EDGEZ_GLOBAL_BUFFER_CHUNK_SIZE + EDGEZ_CONVERSATION_CHUNK_HEADER_SIZE,
    EDGEZ_RELIABLE_PENDING_TABLE_SIZE = 24,
    EDGEZ_RELIABLE_ACK_TIMEOUT_MS = 240,
    EDGEZ_RELIABLE_MAX_ATTEMPTS = 3,
    DEVICE_RELIABLE_TASK_STACK_SIZE = 8192,
    /* Voice and speed-test traffic are best-effort. Keep enough frames to
     * cover short scheduler stalls, but never build a stale realtime
     * backlog when BATMAN or the radio is congested. */
    EDGEZ_REALTIME_TX_QUEUE_DEPTH = 32,
    EDGEZ_VOICE_TX_MAX_ATTEMPTS = 3,
    EDGEZ_VOICE_TX_RETRY_DELAY_MS = 15,
    /* A BLE write-without-response is dispatched on NimBLE's host task. It
     * must never wait for radio capacity: blocking here prevents received ACL
     * mbufs from being returned to the controller and eventually stalls the
     * complete BLE link. The mobile producer provides bulk pacing; this queue
     * only absorbs short scheduler and Morse DMA-return bursts. */
    EDGEZ_SPEED_TX_MAX_ATTEMPTS = 8,
    EDGEZ_SPEED_TX_RETRY_BASE_MS = 10,
    EDGEZ_SPEED_TX_RETRY_MAX_MS = 50,
    EDGEZ_REALTIME_PATH_CACHE_SIZE = 4,
    EDGEZ_REALTIME_PATH_CACHE_TTL_MS = 250,
    EDGEZ_TOPOLOGY_ROUTE_SNAPSHOT_MAX = 16,
    DEVICE_VOICE_TX_TASK_STACK_SIZE = 8192,
    EDGEZ_BEACON_TEXT_MAX_LEN = 480,
    EDGEZ_BEACON_AES_GCM_NONCE_SIZE = 12,
    EDGEZ_BEACON_AES_GCM_TAG_SIZE = 16,
    EDGEZ_BEACON_ENCRYPTED_MAGIC_SIZE = 4,
    EDGEZ_VOICE_RAW_MAGIC_SIZE = 4,
    EDGEZ_SPEED_RAW_MAGIC_SIZE = 4,
    EDGEZ_SPEED_FRAME_HEADER_SIZE = 26,
    EDGEZ_SPEED_ROUTE_PIN_TABLE_SIZE = 24,
    EDGEZ_SPEED_ROUTE_PIN_TTL_MS = 90U * 1000U,
    EDGEZ_GLOBAL_BUFFER_BLE_MAGIC_SIZE = 4,
    EDGEZ_VOICE_BLE_ROUTE_SIZE = 6 + 1 + 4,
    EDGEZ_VOICE_NONCE_SIZE = 12,
    EDGEZ_GLOBAL_BUFFER_REQUEST_SIZE = 8,
    EDGEZ_GLOBAL_BUFFER_CHUNK_REQUEST_SIZE = 14,
    EDGEZ_GLOBAL_BUFFER_TX_SPACING_MS = 30,
    EDGEZ_GLOBAL_BUFFER_RESPONSE_SIZE = 16,
    EDGEZ_GLOBAL_BUFFER_BUSY_RETRY_MS = 2000,
    DEVICE_BEACON_TASK_STACK_SIZE = 16384,
};
#define EDGEZ_APP_TASK_STACK_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#define EDGEZ_SLEEP_TASK_STACK_CAPS (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
/* A one-megabyte queue retained roughly 1,600 maximum-size frames, far more
 * than a realtime mobile link can drain before they become stale. Keep a
 * bounded burst cushion in PSRAM without permanently reserving 1 MiB. */
#define EDGEZ_MOBILE_RX_QUEUE_BYTES (256U * 1024U)
static const uint8_t EDGEZ_CONVERSATION_CHUNK_MAGIC[EDGEZ_CONVERSATION_CHUNK_MAGIC_SIZE] = {'E', 'V', '2'};
static const uint8_t EDGEZ_BEACON_ENCRYPTED_MAGIC[EDGEZ_BEACON_ENCRYPTED_MAGIC_SIZE] = {'E', 'Z', 'B', 1};
static const uint8_t EDGEZ_VOICE_RAW_MAGIC[EDGEZ_VOICE_RAW_MAGIC_SIZE] = {'V', 'C', 'R', 2};
static const uint8_t EDGEZ_SPEED_RAW_MAGIC[EDGEZ_SPEED_RAW_MAGIC_SIZE] = {'E', 'Z', 'S', 'T'};
static const uint8_t EDGEZ_GLOBAL_BUFFER_BLE_MAGIC[EDGEZ_GLOBAL_BUFFER_BLE_MAGIC_SIZE] = {'G', 'B', 'D', 1};
static const uint8_t EDGEZ_GLOBAL_BUFFER_REQUEST_MAGIC[4] = {'G', 'B', 'R', 1};
static const uint8_t EDGEZ_GLOBAL_BUFFER_CHUNK_REQUEST_MAGIC[4] = {'G', 'B', 'R', 2};
static const uint8_t EDGEZ_GLOBAL_BUFFER_RESPONSE_MAGIC[4] = {'G', 'B', 'S', 1};
enum {
    EDGEZ_GLOBAL_BUFFER_STATUS_ACCEPTED = 0,
    EDGEZ_GLOBAL_BUFFER_STATUS_BUSY = 1,
    EDGEZ_GLOBAL_BUFFER_STATUS_NOT_FOUND = 2,
    EDGEZ_GLOBAL_BUFFER_STATUS_ERROR = 3,
};
static uint16_t s_from_radio_seq;
static volatile uint8_t s_ble_profile_reports_remaining;
static TaskHandle_t s_status_report_task;
static TaskHandle_t s_device_beacon_task;
static volatile bool s_device_beacon_force_once;
static volatile uint32_t s_device_beacon_last_refresh_ms;
static volatile uint32_t s_topology_report_last_broadcast_ms;
static TaskHandle_t s_ble_shutdown_task;
static TaskHandle_t s_reliable_tx_task;
static TaskHandle_t s_voice_tx_task;
static TaskHandle_t s_mobile_rx_task;
static QueueHandle_t s_voice_tx_queue;
static QueueHandle_t s_mobile_rx_queue;
static StaticQueue_t s_voice_tx_queue_control;
static StaticQueue_t s_mobile_rx_queue_control;
static uint8_t *s_voice_tx_queue_storage;
static uint8_t *s_mobile_rx_queue_storage;
static atomic_bool s_mobile_log_stream_enabled = ATOMIC_VAR_INIT(false);
static atomic_bool s_mobile_log_delivery_active = ATOMIC_VAR_INIT(false);
static atomic_uint_fast32_t s_realtime_tx_queue_drops = ATOMIC_VAR_INIT(0);
static atomic_uint_fast32_t s_realtime_tx_send_failures = ATOMIC_VAR_INIT(0);
static atomic_uint_fast32_t s_public_channel_mask =
    ATOMIC_VAR_INIT(EDGEZ_PUBLIC_CHANNEL_ALL_MASK);
static halow_sync_active_interface_t s_active_interface = HALOW_SYNC_ACTIVE_INTERFACE_NONE;
static portMUX_TYPE s_device_settings_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_ble_shutdown_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_ble_shutdown_deadline_ms;
static bool s_ble_shutdown_pending;
static uint32_t s_mode_sleep_deadline_ms;
static bool s_mode_sleep_pending;
static bool s_user_halow_shutdown_pending;

static ai_edgez_halow_DeviceSettings s_device_settings = ai_edgez_halow_DeviceSettings_init_zero;

#ifdef CONFIG_EDGEZ_SENSOR_SCRIPTING
typedef struct {
    bool active;
    uint32_t script_id;
    uint32_t total_size;
    uint32_t received;
    ai_edgez_halow_ScriptConfig meta;
    uint8_t *buffer;
} script_config_upload_t;

static script_config_upload_t s_script_config_upload = {0};
#endif

typedef struct {
    bool active;
    uint64_t target;
    uint64_t next_hop;
    uint64_t message_id_high;
    uint64_t message_id_low;
    uint32_t chunk_sequence;
    uint32_t next_offset;
    uint32_t end_offset;
    uint64_t chunk_group_id;
} edgez_global_buffer_target_t;

static edgez_global_buffer_target_t
    s_global_buffer_targets[EDGEZ_GLOBAL_BUFFER_TARGET_TABLE_SIZE];
static const uint8_t *s_global_buffer_data = NULL;
static uint32_t s_global_buffer_length = 0;
static uint8_t s_global_buffer_target_count;
static uint8_t s_global_buffer_target_index;
static bool s_global_buffer_tx_active;
static bool s_global_buffer_serving_acquired;
static uint64_t s_global_buffer_requester;
static uint64_t s_global_buffer_request_message_id_high;
static uint64_t s_global_buffer_request_message_id_low;
static uint8_t s_global_buffer_targets_completed;
static bool s_global_buffer_send_pending;
static portMUX_TYPE s_global_buffer_tx_lock = portMUX_INITIALIZER_UNLOCKED;

static const char *DEVICE_SETTINGS_NVS_NAMESPACE = "edgez_dev";
static const char *DEVICE_SETTINGS_NVS_MODE = "mode";
static const char *DEVICE_SETTINGS_NVS_TYPE = "type";
static const char *DEVICE_SETTINGS_NVS_MESH_ID = "mesh";
static const char *DEVICE_SETTINGS_NVS_PASSPHRASE = "pass";
static const char *DEVICE_SETTINGS_NVS_SHARE_LOC = "share_loc";
static const char *DEVICE_SETTINGS_NVS_USER_NAME = "name";
static const char *DEVICE_SETTINGS_NVS_MARKER = "marker";
static const char *DEVICE_SETTINGS_NVS_INTERVAL = "interval";
static const char *DEVICE_SETTINGS_NVS_USER_HIGH = "uid_hi";
static const char *DEVICE_SETTINGS_NVS_USER_LOW = "uid_lo";
static const char *DEVICE_SETTINGS_NVS_PUBLIC_KEY = "pub_key";
static const char *DEVICE_SETTINGS_NVS_PRIVATE_KEY = "priv_key";
static const char *DEVICE_SETTINGS_NVS_LATITUDE = "lat";
static const char *DEVICE_SETTINGS_NVS_LONGITUDE = "lon";
static const char *DEVICE_SETTINGS_NVS_MAX_HOP = "max_hop";
static const char *DEVICE_SETTINGS_NVS_HAS_GEO_FENCE = "has_geo";
static const char *DEVICE_SETTINGS_NVS_GEO_FENCE = "geo";
static const char *DEVICE_SETTINGS_NVS_UART_I2C_SENSOR_TYPE = "u2c_type";
static const char *DEVICE_SETTINGS_NVS_RS485_SENSOR_TYPE = "rs_type";
static const char *DEVICE_SETTINGS_NVS_GEO_INDEX = "geo_idx";
static const char *DEVICE_SETTINGS_NVS_UPSTREAM_WIFI_SSID = "up_wifi";
static const char *DEVICE_SETTINGS_NVS_UPSTREAM_WIFI_PASSPHRASE = "up_pass";
static const char *DEVICE_SETTINGS_NVS_BEACON_UNICAST = "bcn_to";
static const char *DEVICE_SETTINGS_NVS_SLEEP_ENABLED = "sleep_en";
static const char *DEVICE_SETTINGS_NVS_DEVICE_GPS = "gps_en";
static const char *DEVICE_SETTINGS_NVS_MESH_FREQUENCY = "mesh_freq";
static const char *DEVICE_SETTINGS_NVS_MESH_BANDWIDTH = "mesh_bw";

static void device_settings_get_snapshot(ai_edgez_halow_DeviceSettings *out);
static void device_settings_apply_snapshot(const ai_edgez_halow_DeviceSettings *settings);
static esp_err_t device_settings_save_to_nvs(const ai_edgez_halow_DeviceSettings *settings);
static esp_err_t device_settings_persist_script_sensor_selection(const ai_edgez_halow_ScriptConfig *config);
static bool sensor_selector_is_empty(const char *selector);
static uint32_t default_network_packet_max_hop(void);
static void global_buffer_tx_reset_state(void);
static void global_buffer_tx_send_current_chunk(void);
static void global_buffer_tx_schedule_send(void);
static esp_err_t global_buffer_tx_start_for_requester(uint64_t requester,
                                                      uint64_t request_message_id_high,
                                                      uint64_t request_message_id_low,
                                                      uint32_t expected_length,
                                                      uint64_t requested_group_id,
                                                      int requested_chunk_index);
static esp_err_t global_buffer_send_status(uint64_t requester,
                                           uint8_t status,
                                           uint32_t retry_after_ms,
                                           uint32_t available_length);
static void send_device_settings_frame(uint16_t seq,
                                       const ai_edgez_halow_NetworkPacket *request);

enum {
    BLE_DISCONNECTED_WINDOW_MS = 2U * 60U * 1000U,
    DEVICE_DEEP_SLEEP_WINDOW_MS = 3U * 60U * 1000U,
    USER_DISCONNECT_DEEP_SLEEP_WINDOW_MS = 1U * 60U * 1000U,
};

typedef struct {
    bool valid;
    uint64_t high;
    uint64_t low;
    uint32_t sequence;
    uint8_t best_hop;
} edgez_seen_message_t;

static edgez_seen_message_t s_seen_messages[EDGEZ_SEEN_MESSAGE_TABLE_SIZE];
static uint8_t s_seen_message_replace_index;
static portMUX_TYPE s_seen_message_lock = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    bool valid;
    uint64_t checksum;
    uint32_t last_seen_ms;
} edgez_delivery_dedupe_entry_t;

static edgez_delivery_dedupe_entry_t s_delivery_dedupe[EDGEZ_DELIVERY_DEDUPE_TABLE_SIZE];
static portMUX_TYPE s_delivery_dedupe_lock = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    bool valid;
    uint64_t message_id_high;
    uint64_t message_id_low;
    uint32_t sequence;
    uint64_t from;
    uint64_t to;
    uint64_t next_hop;
    uint32_t max_hop;
    uint8_t attempts;
    uint32_t last_send_ms;
    uint32_t ack_timeout_ms;
    size_t payload_len;
    uint8_t payload[ai_edgez_halow_NetworkPacket_size];
} edgez_reliable_pending_t;

static EXT_RAM_BSS_ATTR edgez_reliable_pending_t
    s_reliable_pending[EDGEZ_RELIABLE_PENDING_TABLE_SIZE];
static portMUX_TYPE s_reliable_pending_lock = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    uint8_t payload[ai_edgez_halow_NetworkPacket_size];
    uint16_t payload_len;
    uint64_t from;
    uint64_t to;
    uint64_t next_hop;
    uint64_t message_id_high;
    uint64_t message_id_low;
    uint32_t max_hop;
    uint32_t sequence;
    bool forward_from_mobile;
    uint32_t mobile_initial_hop;
} edgez_voice_tx_item_t;

typedef enum {
    EDGEZ_MOBILE_RX_REALTIME = 0,
    EDGEZ_MOBILE_RX_LOG,
    EDGEZ_MOBILE_RX_CONTROL,
} edgez_mobile_rx_kind_t;

typedef struct {
    edgez_mobile_rx_kind_t kind;
    uint16_t payload_len;
    uint8_t payload[ai_edgez_halow_NetworkPacket_size];
} edgez_mobile_rx_item_t;

#define EDGEZ_TOPOLOGY_MAX_PEERS 6
#define EDGEZ_TOPOLOGY_RECENT_MS (5U * 60U * 1000U)
#define EDGEZ_TOPOLOGY_CONNECTED_MS (90U * 1000U)
#define EDGEZ_TOPOLOGY_RSSI_OFFSET 1000

typedef struct {
    bool valid;
    bool heard_beacon;
    bool connected;
    uint64_t peer_id;
    int32_t encoded_rssi;
    uint32_t last_seen_ms;
    uint32_t beacon_last_seen_ms;
    uint32_t connected_last_seen_ms;
    pb_size_t sensor_data_count;
    ai_edgez_halow_SensorData sensor_data[7];
} edgez_topology_peer_t;

static edgez_topology_peer_t s_topology_peers[EDGEZ_TOPOLOGY_MAX_PEERS];
static portMUX_TYPE s_topology_lock = portMUX_INITIALIZER_UNLOCKED;

static void bridge_send_frame(const uint8_t *payload,
                             uint16_t payload_len,
                             bool use_forward_channel)
{
    bool sent = false;
    const bool ble_connected = edgez_platform_get()->ble_is_connected();

    if (ble_connected) {
        if (use_forward_channel) {
            edgez_platform_get()->ble_send_forward_frame(payload, payload_len);
        } else {
            edgez_platform_get()->ble_send_frame(payload, payload_len);
        }
        sent = true;
    } else if (usb_control_transport_is_connected()) {
        usb_control_transport_send_frame(payload, payload_len);
        sent = true;
    }

    if (!sent) {
        ESP_LOGD(TAG, "No mobile interface connected for protobuf frame len=%u", payload_len);
    }
}

static bool bridge_has_connected_interface(void)
{
    return edgez_platform_get()->ble_is_connected() ||
           usb_control_transport_is_connected();
}

static void bridge_send_voice_frame_now(const uint8_t *payload,
                                        uint16_t payload_len)
{
    if (edgez_platform_get()->ble_is_connected()) {
        edgez_platform_get()->ble_send_voice_frame(payload, payload_len);
    } else if (usb_control_transport_is_connected()) {
        usb_control_transport_send_voice_frame(payload, payload_len);
    }
}

static void bridge_send_log_frame_now(const uint8_t *payload,
                                      uint16_t payload_len)
{
    if (edgez_platform_get()->ble_is_connected()) {
        edgez_platform_get()->ble_send_voice_frame(payload, payload_len);
    } else if (usb_control_transport_is_connected()) {
        usb_control_transport_send_log_frame(payload, payload_len);
    }
}

static bool bridge_queue_mobile_rx_frame(const uint8_t *payload,
                                         uint16_t payload_len,
                                         edgez_mobile_rx_kind_t kind,
                                         TickType_t wait)
{
    if (!payload || payload_len == 0 ||
        payload_len > sizeof(((edgez_mobile_rx_item_t *)0)->payload) ||
        !s_mobile_rx_queue) {
        return false;
    }
    edgez_mobile_rx_item_t item = {
        .kind = kind,
        .payload_len = payload_len,
    };
    memcpy(item.payload, payload, payload_len);
    return xQueueSend(s_mobile_rx_queue, &item, wait) == pdPASS;
}

static void bridge_send_voice_frame(const uint8_t *payload, uint16_t payload_len)
{
    if (!bridge_queue_mobile_rx_frame(payload, payload_len,
                                      EDGEZ_MOBILE_RX_REALTIME,
                                      portMAX_DELAY)) {
        ESP_LOGW(TAG, "HaLow->mobile RX queue enqueue failed len=%u",
                 (unsigned)payload_len);
    }
}

void halow_sync_bridge_send_realtime_frame(const uint8_t *payload,
                                            uint16_t payload_len)
{
    bridge_send_voice_frame(payload, payload_len);
}

bool halow_sync_bridge_queue_log_frame(const uint8_t *payload,
                                       uint16_t payload_len)
{
    if (!atomic_load_explicit(&s_mobile_log_stream_enabled,
                              memory_order_acquire)) {
        return false;
    }
    return bridge_queue_mobile_rx_frame(payload, payload_len,
                                        EDGEZ_MOBILE_RX_LOG, 0);
}

bool halow_sync_bridge_log_delivery_active(void)
{
    return atomic_load_explicit(&s_mobile_log_delivery_active,
                                memory_order_acquire);
}

void halow_sync_bridge_set_log_stream_enabled(bool enabled)
{
    atomic_store_explicit(&s_mobile_log_stream_enabled, enabled,
                          memory_order_release);
}

void halow_sync_bridge_request_log_level_test(void)
{
    if (!atomic_load_explicit(&s_mobile_log_stream_enabled,
                              memory_order_acquire)) {
        return;
    }
    ESP_LOGE(TAG, "Log level test ERROR: error output enabled");
    ESP_LOGW(TAG, "Log level test WARN: warning output enabled");
    ESP_LOGI(TAG, "Log level test INFO: informational output enabled");
    ESP_LOGD(TAG, "Log level test DEBUG: debug output enabled");
    ESP_LOGV(TAG, "Log level test VERBOSE: verbose output enabled");
}

static uint64_t mac_to_u64(const uint8_t mac[6])
{
    return ((uint64_t)mac[0] << 40) |
           ((uint64_t)mac[1] << 32) |
           ((uint64_t)mac[2] << 24) |
           ((uint64_t)mac[3] << 16) |
           ((uint64_t)mac[4] << 8) |
           (uint64_t)mac[5];
}

bool halow_sync_public_channel_enabled(uint64_t id)
{
    if (!halow_sync_is_public_channel(id)) {
        return false;
    }
    uint32_t index = (uint32_t)((id - EDGEZ_PUBLIC_CHANNEL_FIRST_ID) /
                                EDGEZ_PUBLIC_CHANNEL_ID_STEP);
    uint32_t mask = (uint32_t)atomic_load_explicit(
        &s_public_channel_mask, memory_order_acquire);
    return (mask & (1U << index)) != 0;
}

static void public_channel_mask_set(uint32_t mask)
{
    mask &= EDGEZ_PUBLIC_CHANNEL_ALL_MASK;
    atomic_store_explicit(&s_public_channel_mask, mask, memory_order_release);
    ESP_LOGI(TAG, "Public channel subscriptions updated mask=0x%02lx",
             (unsigned long)mask);
}

static void u64_to_mac_bytes(uint64_t value, uint8_t mac[6])
{
    value &= 0xffffffffffffULL;
    mac[0] = (uint8_t)(value >> 40);
    mac[1] = (uint8_t)(value >> 32);
    mac[2] = (uint8_t)(value >> 24);
    mac[3] = (uint8_t)(value >> 16);
    mac[4] = (uint8_t)(value >> 8);
    mac[5] = (uint8_t)value;
}

static bool mac_is_broadcast_u64(uint64_t mac)
{
    return (mac & 0xffffffffffffULL) == 0xffffffffffffULL;
}

static bool mac_is_zero_u64(uint64_t mac)
{
    return (mac & 0xffffffffffffULL) == 0;
}

static bool mac_is_zero_bytes(const uint8_t mac[6])
{
    return mac && mac[0] == 0 && mac[1] == 0 && mac[2] == 0 &&
           mac[3] == 0 && mac[4] == 0 && mac[5] == 0;
}

static bool batman_route_lookup(uint64_t target, uint64_t *next_hop_out);
static uint64_t read_u64_be(const uint8_t *in);

static uint32_t route_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

typedef enum {
    SPEED_TRACE_MOBILE_IN = 0,
    SPEED_TRACE_RELAY_IN,
    SPEED_TRACE_RADIO_TX,
    SPEED_TRACE_DEST_OUT,
    SPEED_TRACE_STAGE_COUNT,
} speed_trace_stage_t;

typedef struct {
    uint64_t transfer_id;
    uint32_t started_ms;
    uint32_t interval_started_ms;
    uint32_t interval_frames;
    uint64_t interval_bytes;
    uint32_t total_frames;
    uint64_t total_bytes;
    uint32_t errors;
} speed_trace_stats_t;

static speed_trace_stats_t s_speed_trace[SPEED_TRACE_STAGE_COUNT];
static portMUX_TYPE s_speed_trace_lock = portMUX_INITIALIZER_UNLOCKED;

static void speed_trace_record(speed_trace_stage_t stage,
                               const uint8_t *frame,
                               size_t frame_len,
                               uint32_t sequence,
                               uint8_t hop,
                               uint8_t max_hop,
                               UBaseType_t queue_depth,
                               esp_err_t result)
{
    if (stage >= SPEED_TRACE_STAGE_COUNT || !frame ||
        frame_len < EDGEZ_SPEED_FRAME_HEADER_SIZE) {
        return;
    }

    static const char *const stage_names[SPEED_TRACE_STAGE_COUNT] = {
        "mobile_in", "relay_in", "radio_tx", "dest_out",
    };
    const uint64_t transfer_id = read_u64_be(frame + 6);
    const uint8_t frame_type = frame[5];
    const uint32_t now = route_now_ms();
    bool emit = false;
    speed_trace_stats_t snapshot = {0};
    uint32_t interval_ms = 0;

    portENTER_CRITICAL(&s_speed_trace_lock);
    speed_trace_stats_t *stats = &s_speed_trace[stage];
    if (stats->transfer_id != transfer_id) {
        memset(stats, 0, sizeof(*stats));
        stats->transfer_id = transfer_id;
        stats->started_ms = now;
        stats->interval_started_ms = now;
    }
    stats->interval_frames++;
    stats->interval_bytes += frame_len;
    stats->total_frames++;
    stats->total_bytes += frame_len;
    if (result != ESP_OK) stats->errors++;
    interval_ms = now - stats->interval_started_ms;
    emit = frame_type == 1 || frame_type == 3 || interval_ms >= 1000U;
    if (emit) {
        snapshot = *stats;
        stats->interval_frames = 0;
        stats->interval_bytes = 0;
        stats->interval_started_ms = now;
    }
    portEXIT_CRITICAL(&s_speed_trace_lock);

    if (!emit) return;
    if (interval_ms == 0) interval_ms = 1;
    uint32_t elapsed_ms = now - snapshot.started_ms;
    if (elapsed_ms == 0) elapsed_ms = 1;
    uint64_t interval_kbps =
        snapshot.interval_bytes * 8ULL / interval_ms;
    uint64_t average_kbps = snapshot.total_bytes * 8ULL / elapsed_ms;
    esp_log_level_set(SPEED_TAG, ESP_LOG_INFO);
    ESP_LOGI(
        SPEED_TAG,
        "SPEED_METRIC stage=%s transfer=%016llx type=%u seq=%lu hop=%u/%u interval_ms=%lu frames=%lu bytes=%llu kbps=%llu avg_kbps=%llu total_frames=%lu total_bytes=%llu queue=%u errors=%lu result=%s",
        stage_names[stage],
        (unsigned long long)transfer_id,
        (unsigned)frame_type,
        (unsigned long)sequence,
        (unsigned)hop,
        (unsigned)max_hop,
        (unsigned long)interval_ms,
        (unsigned long)snapshot.interval_frames,
        (unsigned long long)snapshot.interval_bytes,
        (unsigned long long)interval_kbps,
        (unsigned long long)average_kbps,
        (unsigned long)snapshot.total_frames,
        (unsigned long long)snapshot.total_bytes,
        (unsigned)queue_depth,
        (unsigned long)snapshot.errors,
        esp_err_to_name(result));
}

static int32_t topology_encode_rssi(int32_t rssi_dbm)
{
    if (rssi_dbm == 0) {
        /* 1000 is the existing unknown-RSSI sentinel. Keep a valid 0 dBm
         * measurement visible to clients by encoding it as the nearest
         * representable value (-1 dBm => 999). */
        return EDGEZ_TOPOLOGY_RSSI_OFFSET - 1;
    }
    return (rssi_dbm < 0 && rssi_dbm > -EDGEZ_TOPOLOGY_RSSI_OFFSET)
               ? EDGEZ_TOPOLOGY_RSSI_OFFSET + rssi_dbm
               : EDGEZ_TOPOLOGY_RSSI_OFFSET;
}

static bool location_is_valid(float latitude, float longitude)
{
    return isfinite(latitude) && isfinite(longitude) &&
           latitude >= -90.0f && latitude <= 90.0f &&
           longitude >= -180.0f && longitude <= 180.0f &&
           (latitude != 0.0f || longitude != 0.0f);
}

static void topology_note_peer_sensor_data(uint64_t peer_id,
                                           const ai_edgez_halow_Beacon *beacon)
{
    if (!beacon) {
        return;
    }
    float latitude = beacon->latitude;
    float longitude = beacon->longitude;
    bool has_latitude = false;
    bool has_longitude = false;
    for (pb_size_t i = 0; i < beacon->sensor_data_count; ++i) {
        const ai_edgez_halow_SensorData *entry = &beacon->sensor_data[i];
        if (entry->which_value != ai_edgez_halow_SensorData_float_value_tag) {
            continue;
        }
        if (entry->type == ai_edgez_halow_SensorType_SENSOR_LATITUDE) {
            latitude = entry->value.float_value;
            has_latitude = true;
        } else if (entry->type == ai_edgez_halow_SensorType_SENSOR_LONGITUDE) {
            longitude = entry->value.float_value;
            has_longitude = true;
        }
    }
    bool has_valid_location = has_latitude == has_longitude &&
                              location_is_valid(latitude, longitude);

    peer_id &= 0xffffffffffffULL;
    portENTER_CRITICAL(&s_topology_lock);
    for (int i = 0; i < EDGEZ_TOPOLOGY_MAX_PEERS; ++i) {
        if (s_topology_peers[i].valid && s_topology_peers[i].peer_id == peer_id) {
            edgez_topology_peer_t *cached = &s_topology_peers[i];
            cached->sensor_data_count = 0;
            if (has_valid_location) {
                ai_edgez_halow_SensorData *latitude_entry =
                    &cached->sensor_data[cached->sensor_data_count++];
                latitude_entry->type = ai_edgez_halow_SensorType_SENSOR_LATITUDE;
                latitude_entry->which_value = ai_edgez_halow_SensorData_float_value_tag;
                latitude_entry->value.float_value = latitude;
                ai_edgez_halow_SensorData *longitude_entry =
                    &cached->sensor_data[cached->sensor_data_count++];
                longitude_entry->type = ai_edgez_halow_SensorType_SENSOR_LONGITUDE;
                longitude_entry->which_value = ai_edgez_halow_SensorData_float_value_tag;
                longitude_entry->value.float_value = longitude;
            }
            for (pb_size_t sensor_index = 0;
                 sensor_index < beacon->sensor_data_count && cached->sensor_data_count < 7;
                 ++sensor_index) {
                const ai_edgez_halow_SensorData *entry = &beacon->sensor_data[sensor_index];
                if (entry->type == ai_edgez_halow_SensorType_SENSOR_LATITUDE ||
                    entry->type == ai_edgez_halow_SensorType_SENSOR_LONGITUDE) {
                    continue;
                }
                cached->sensor_data[cached->sensor_data_count++] = *entry;
            }
            break;
        }
    }
    portEXIT_CRITICAL(&s_topology_lock);

}

static void topology_fill_peer_sensor_data(ai_edgez_halow_Peer *peer,
                                           const edgez_topology_peer_t *cached)
{
    if (!peer || !cached) {
        return;
    }
    peer->sensor_data_count = cached->sensor_data_count;
    memcpy(peer->sensor_data,
           cached->sensor_data,
           cached->sensor_data_count * sizeof(cached->sensor_data[0]));
}

static void topology_note_peer(uint64_t peer_id,
                               int32_t rssi_dbm,
                               bool heard_beacon,
                               bool connected)
{
    peer_id &= 0xffffffffffffULL;
    if (mac_is_zero_u64(peer_id) || mac_is_broadcast_u64(peer_id)) {
        return;
    }

    uint32_t now = route_now_ms();
    int slot = -1;
    int oldest_slot = 0;
    uint32_t oldest_seen = UINT32_MAX;

    portENTER_CRITICAL(&s_topology_lock);
    for (int i = 0; i < EDGEZ_TOPOLOGY_MAX_PEERS; ++i) {
        if (s_topology_peers[i].valid && s_topology_peers[i].peer_id == peer_id) {
            slot = i;
            break;
        }
        if (!s_topology_peers[i].valid && slot < 0) {
            slot = i;
        }
        if (s_topology_peers[i].last_seen_ms < oldest_seen) {
            oldest_seen = s_topology_peers[i].last_seen_ms;
            oldest_slot = i;
        }
    }
    if (slot < 0) {
        slot = oldest_slot;
    }
    if (!s_topology_peers[slot].valid || s_topology_peers[slot].peer_id != peer_id) {
        memset(&s_topology_peers[slot], 0, sizeof(s_topology_peers[slot]));
    }
    s_topology_peers[slot].valid = true;
    s_topology_peers[slot].heard_beacon |= heard_beacon;
    s_topology_peers[slot].connected |= connected;
    s_topology_peers[slot].peer_id = peer_id;
    if (rssi_dbm <= 0 && rssi_dbm > -EDGEZ_TOPOLOGY_RSSI_OFFSET) {
        s_topology_peers[slot].encoded_rssi = topology_encode_rssi(rssi_dbm);
    } else if (s_topology_peers[slot].encoded_rssi == 0) {
        s_topology_peers[slot].encoded_rssi = EDGEZ_TOPOLOGY_RSSI_OFFSET;
    }
    s_topology_peers[slot].last_seen_ms = now;
    if (heard_beacon) {
        s_topology_peers[slot].beacon_last_seen_ms = now;
    }
    if (connected) {
        s_topology_peers[slot].connected_last_seen_ms = now;
    }
    portEXIT_CRITICAL(&s_topology_lock);
}

void halow_sync_bridge_fill_report_peers(ai_edgez_halow_Report *report)
{
    if (report == NULL) {
        return;
    }

#ifdef CONFIG_ENABLE_MM_HALOW
    uint8_t authorized_count = mmwlan_get_mesh_authorized_peer_count();
    for (uint8_t i = 0; i < authorized_count; ++i) {
        uint8_t peer_mac[6] = {0};
        int16_t rssi_dbm = 0;
        bool rssi_valid = false;
        enum mmwlan_status status = mmwlan_get_mesh_authorized_peer(
            i, peer_mac, &rssi_dbm, &rssi_valid);
        if (status == MMWLAN_SUCCESS) {
            uint64_t peer_id = mac_to_u64(peer_mac);
            topology_note_peer(peer_id,
                               rssi_valid ? rssi_dbm : INT32_MIN,
                               false,
                               true);
            ESP_LOGI(TAG,
                     "Topology peer snapshot peer=%02x:%02x:%02x:%02x:%02x:%02x rssi_valid=%u rssi_dbm=%d encoded_rssi=%ld",
                     peer_mac[0], peer_mac[1], peer_mac[2],
                     peer_mac[3], peer_mac[4], peer_mac[5],
                     rssi_valid ? 1U : 0U,
                     (int)rssi_dbm,
                     (long)(rssi_valid ? topology_encode_rssi(rssi_dbm) :
                                          EDGEZ_TOPOLOGY_RSSI_OFFSET));
        } else {
            ESP_LOGW(TAG,
                     "Topology authorized peer snapshot unavailable index=%u/%u status=%d",
                     (unsigned)i, (unsigned)authorized_count, (int)status);
        }
    }
#endif

    uint32_t now = route_now_ms();
    report->peers_count = 0;
    portENTER_CRITICAL(&s_topology_lock);

    /* Topology reports describe radio observations, not routing state.
     * Connected peers are emitted first and recent beacons follow. BATMAN's
     * route table is exposed separately through an on-demand local request. */
    for (int i = 0; i < EDGEZ_TOPOLOGY_MAX_PEERS; ++i) {
        edgez_topology_peer_t *cached = &s_topology_peers[i];
        if (cached->valid && cached->connected &&
            (now - cached->connected_last_seen_ms) <= EDGEZ_TOPOLOGY_CONNECTED_MS &&
            report->peers_count < EDGEZ_TOPOLOGY_MAX_PEERS) {
            ai_edgez_halow_Peer *peer = &report->peers[report->peers_count++];
            *peer = (ai_edgez_halow_Peer)ai_edgez_halow_Peer_init_zero;
            peer->id = cached->peer_id;
            peer->rssi = cached->encoded_rssi != 0 ?
                         cached->encoded_rssi : EDGEZ_TOPOLOGY_RSSI_OFFSET;
            topology_fill_peer_sensor_data(peer, cached);
        }
    }

    /* Append recently heard beacons only when that peer was not already
     * emitted as a currently connected peer. */
    for (int i = 0;
         i < EDGEZ_TOPOLOGY_MAX_PEERS && report->peers_count < EDGEZ_TOPOLOGY_MAX_PEERS;
         ++i) {
        edgez_topology_peer_t *cached = &s_topology_peers[i];
        bool connected_is_fresh = cached->connected &&
                                  (now - cached->connected_last_seen_ms) <=
                                  EDGEZ_TOPOLOGY_CONNECTED_MS;
        bool beacon_is_fresh = cached->heard_beacon &&
                               (now - cached->beacon_last_seen_ms) <=
                               EDGEZ_TOPOLOGY_RECENT_MS;
        if (!cached->valid || connected_is_fresh || !beacon_is_fresh) {
            continue;
        }
        ai_edgez_halow_Peer *peer = &report->peers[report->peers_count++];
        *peer = (ai_edgez_halow_Peer)ai_edgez_halow_Peer_init_zero;
        peer->id = cached->peer_id;
        peer->rssi = cached->encoded_rssi != 0 ?
                     cached->encoded_rssi : EDGEZ_TOPOLOGY_RSSI_OFFSET;
        topology_fill_peer_sensor_data(peer, cached);
    }
    portEXIT_CRITICAL(&s_topology_lock);
}

static uint64_t fnv1a64_update(uint64_t hash, const uint8_t *data, size_t len)
{
    if (!data && len > 0) {
        return hash;
    }
    for (size_t i = 0; i < len; ++i) {
        hash ^= data[i];
        hash *= 1099511628211ULL;
    }
    return hash;
}

static uint64_t delivery_dedupe_checksum(uint64_t source, const uint8_t *message, size_t message_len)
{
    uint8_t source_bytes[6] = {
        (uint8_t)(source >> 40),
        (uint8_t)(source >> 32),
        (uint8_t)(source >> 24),
        (uint8_t)(source >> 16),
        (uint8_t)(source >> 8),
        (uint8_t)source,
    };
    uint64_t hash = 1469598103934665603ULL;
    hash = fnv1a64_update(hash, source_bytes, sizeof(source_bytes));
    return fnv1a64_update(hash, message, message_len);
}

static bool delivery_dedupe_check_and_add(uint64_t source,
                                          const uint8_t *message,
                                          size_t message_len,
                                          uint64_t *checksum_out)
{
    uint64_t checksum = delivery_dedupe_checksum(source, message, message_len);
    uint32_t now = route_now_ms();
    int free_slot = -1;
    int expired_slot = -1;
    int oldest_slot = 0;
    uint32_t oldest_ms = UINT32_MAX;
    bool duplicate = false;

    portENTER_CRITICAL(&s_delivery_dedupe_lock);
    for (int i = 0; i < EDGEZ_DELIVERY_DEDUPE_TABLE_SIZE; ++i) {
        if (!s_delivery_dedupe[i].valid) {
            if (free_slot < 0) {
                free_slot = i;
            }
            continue;
        }

        uint32_t age_ms = now - s_delivery_dedupe[i].last_seen_ms;
        if (age_ms > EDGEZ_DELIVERY_DEDUPE_WINDOW_MS) {
            if (expired_slot < 0) {
                expired_slot = i;
            }
        } else if (s_delivery_dedupe[i].checksum == checksum) {
            s_delivery_dedupe[i].last_seen_ms = now;
            duplicate = true;
            break;
        }

        if (s_delivery_dedupe[i].last_seen_ms < oldest_ms) {
            oldest_ms = s_delivery_dedupe[i].last_seen_ms;
            oldest_slot = i;
        }
    }

    if (!duplicate) {
        int slot = (expired_slot >= 0) ? expired_slot : ((free_slot >= 0) ? free_slot : oldest_slot);
        s_delivery_dedupe[slot].valid = true;
        s_delivery_dedupe[slot].checksum = checksum;
        s_delivery_dedupe[slot].last_seen_ms = now;
    }
    portEXIT_CRITICAL(&s_delivery_dedupe_lock);

    if (checksum_out) {
        *checksum_out = checksum;
    }
    return duplicate;
}

static bool device_type_is_device_profile(ai_edgez_halow_DeviceType device_type)
{
    return device_type == ai_edgez_halow_DeviceType_DEVICE_TYPE_BEACON ||
           device_type == ai_edgez_halow_DeviceType_DEVICE_TYPE_SENSOR;
}

static bool device_type_is_autonomous(ai_edgez_halow_DeviceType device_type)
{
    return device_type == ai_edgez_halow_DeviceType_DEVICE_TYPE_BEACON ||
           device_type == ai_edgez_halow_DeviceType_DEVICE_TYPE_SENSOR ||
           device_type == ai_edgez_halow_DeviceType_DEVICE_TYPE_RELAY;
}

static ai_edgez_halow_DeviceType device_type_normalize(ai_edgez_halow_DeviceType device_type,
                                                        bool legacy_device_mode_enabled)
{
    switch (device_type) {
    case ai_edgez_halow_DeviceType_DEVICE_TYPE_USER:
    case ai_edgez_halow_DeviceType_DEVICE_TYPE_BEACON:
    case ai_edgez_halow_DeviceType_DEVICE_TYPE_SENSOR:
    case ai_edgez_halow_DeviceType_DEVICE_TYPE_RELAY:
        return device_type;
    default:
        return legacy_device_mode_enabled
                   ? ai_edgez_halow_DeviceType_DEVICE_TYPE_BEACON
                   : ai_edgez_halow_DeviceType_DEVICE_TYPE_USER;
    }
}

static bool device_mode_is_enabled(void)
{
    bool enabled;
    portENTER_CRITICAL(&s_device_settings_lock);
    enabled = device_type_is_device_profile(s_device_settings.device_type);
    portEXIT_CRITICAL(&s_device_settings_lock);
    return enabled;
}

static bool device_type_uses_user_idle_sleep(ai_edgez_halow_DeviceType device_type)
{
    switch (device_type) {
    case ai_edgez_halow_DeviceType_DEVICE_TYPE_UNSPECIFIED:
    case ai_edgez_halow_DeviceType_DEVICE_TYPE_UNKNOWN:
    case ai_edgez_halow_DeviceType_DEVICE_TYPE_USER:
        return true;
    case ai_edgez_halow_DeviceType_DEVICE_TYPE_BEACON:
    case ai_edgez_halow_DeviceType_DEVICE_TYPE_SENSOR:
    case ai_edgez_halow_DeviceType_DEVICE_TYPE_RELAY:
    default:
        return false;
    }
}

static bool device_type_uses_status_led(ai_edgez_halow_DeviceType device_type)
{
    return device_type != ai_edgez_halow_DeviceType_DEVICE_TYPE_BEACON &&
           device_type != ai_edgez_halow_DeviceType_DEVICE_TYPE_SENSOR;
}

static void device_type_apply_status_led(ai_edgez_halow_DeviceType device_type)
{
    bool enabled = device_type_uses_status_led(device_type);
    if (enabled) {
        esp_err_t err = edgez_platform_get()->led_init();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "HT-HC33 status LED init failed: %s", esp_err_to_name(err));
            edgez_platform_get()->led_set_enabled(false);
            return;
        }
    }
    edgez_platform_get()->led_set_enabled(enabled);
    edgez_platform_get()->led_set_user_mode(device_type_uses_user_idle_sleep(device_type));
    ESP_LOGI(TAG,
             "HT-HC33 status LED mode type=%u enabled=%u",
             (unsigned)device_type,
             enabled ? 1U : 0U);
}

static bool device_sleep_policy_enabled(void)
{
    ai_edgez_halow_DeviceType device_type;
    portENTER_CRITICAL(&s_device_settings_lock);
    device_type = s_device_settings.device_type;
    portEXIT_CRITICAL(&s_device_settings_lock);
    return device_type_uses_user_idle_sleep(device_type);
}

bool halow_sync_bridge_forwarding_enabled(void)
{
    if (device_mode_is_enabled()) {
        return false;
    }

    ai_edgez_halow_DeviceSettings settings = ai_edgez_halow_DeviceSettings_init_zero;
    device_settings_get_snapshot(&settings);

    if (!sensor_selector_is_empty(settings.uart_i2c_sensor_type) ||
        !sensor_selector_is_empty(settings.rs485_sensor_type)) {
        return false;
    }

    return true;
}

static void device_mode_ble_shutdown_cancel(void)
{
    portENTER_CRITICAL(&s_ble_shutdown_lock);
    s_ble_shutdown_pending = false;
    s_ble_shutdown_deadline_ms = 0;
    portEXIT_CRITICAL(&s_ble_shutdown_lock);
}

static void device_mode_ble_shutdown_schedule(uint32_t delay_ms, const char *reason)
{
    if (!edgez_platform_get()->ble_is_enabled() || bridge_has_connected_interface()) {
        device_mode_ble_shutdown_cancel();
        return;
    }

    uint32_t deadline_ms = route_now_ms() + delay_ms;
    portENTER_CRITICAL(&s_ble_shutdown_lock);
    s_ble_shutdown_deadline_ms = deadline_ms;
    s_ble_shutdown_pending = true;
    portEXIT_CRITICAL(&s_ble_shutdown_lock);

    ESP_LOGI(TAG,
             "Device mode BLE shutdown scheduled in %lu ms reason=%s",
             (unsigned long)delay_ms,
             reason ? reason : "unknown");
}

static void mode_deep_sleep_cancel(void)
{
    portENTER_CRITICAL(&s_ble_shutdown_lock);
    s_mode_sleep_pending = false;
    s_mode_sleep_deadline_ms = 0;
    portEXIT_CRITICAL(&s_ble_shutdown_lock);
}

static void mode_deep_sleep_schedule(uint32_t delay_ms, const char *reason)
{
    if (!device_sleep_policy_enabled() || bridge_has_connected_interface()) {
        mode_deep_sleep_cancel();
        return;
    }

    ai_edgez_halow_DeviceSettings settings = ai_edgez_halow_DeviceSettings_init_zero;
    device_settings_get_snapshot(&settings);
    uint32_t deadline_ms = route_now_ms() + delay_ms;
    portENTER_CRITICAL(&s_ble_shutdown_lock);
    s_mode_sleep_deadline_ms = deadline_ms;
    s_mode_sleep_pending = true;
    portEXIT_CRITICAL(&s_ble_shutdown_lock);

    ESP_LOGI(TAG,
             "User/unknown mode idle sleep scheduled in %lu ms type=%u reason=%s",
             (unsigned long)delay_ms,
             (unsigned)settings.device_type,
             reason ? reason : "unknown");
}

static void disconnected_power_policy_schedule(const char *reason)
{
    device_mode_ble_shutdown_schedule(BLE_DISCONNECTED_WINDOW_MS, reason);
    mode_deep_sleep_schedule(DEVICE_DEEP_SLEEP_WINDOW_MS, reason);
}

static void ble_disconnect_power_policy_schedule(void)
{
    ai_edgez_halow_DeviceSettings settings = ai_edgez_halow_DeviceSettings_init_zero;
    device_settings_get_snapshot(&settings);
    uint32_t sleep_delay_ms = device_type_uses_user_idle_sleep(settings.device_type)
                                  ? USER_DISCONNECT_DEEP_SLEEP_WINDOW_MS
                                  : DEVICE_DEEP_SLEEP_WINDOW_MS;

    device_mode_ble_shutdown_schedule(BLE_DISCONNECTED_WINDOW_MS, "ble_disconnected");
    mode_deep_sleep_schedule(sleep_delay_ms, "ble_disconnected");

    if (device_type_uses_user_idle_sleep(settings.device_type)) {
        portENTER_CRITICAL(&s_ble_shutdown_lock);
        s_user_halow_shutdown_pending = true;
        portEXIT_CRITICAL(&s_ble_shutdown_lock);
        if (s_ble_shutdown_task != NULL) {
            xTaskNotifyGive(s_ble_shutdown_task);
        }
    }
}

static void boot_power_policy_schedule(void)
{
    uint32_t uptime_ms = route_now_ms();
    uint32_t ble_delay_ms = uptime_ms < BLE_DISCONNECTED_WINDOW_MS
                                ? BLE_DISCONNECTED_WINDOW_MS - uptime_ms
                                : 0;
    uint32_t sleep_delay_ms = uptime_ms < DEVICE_DEEP_SLEEP_WINDOW_MS
                                  ? DEVICE_DEEP_SLEEP_WINDOW_MS - uptime_ms
                                  : 0;
    device_mode_ble_shutdown_schedule(ble_delay_ms, "boot_no_connection");
    mode_deep_sleep_schedule(sleep_delay_ms, "boot_no_connection");
}

static void device_mode_ble_shutdown_task(void *arg)
{
    (void)arg;

    while (true) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

        bool pending;
        uint32_t deadline_ms;
        bool sleep_pending;
        uint32_t sleep_deadline_ms;
        bool user_halow_shutdown_pending;
        portENTER_CRITICAL(&s_ble_shutdown_lock);
        pending = s_ble_shutdown_pending;
        deadline_ms = s_ble_shutdown_deadline_ms;
        sleep_pending = s_mode_sleep_pending;
        sleep_deadline_ms = s_mode_sleep_deadline_ms;
        user_halow_shutdown_pending = s_user_halow_shutdown_pending;
        portEXIT_CRITICAL(&s_ble_shutdown_lock);

        if (bridge_has_connected_interface()) {
            device_mode_ble_shutdown_cancel();
            mode_deep_sleep_cancel();
            portENTER_CRITICAL(&s_ble_shutdown_lock);
            s_user_halow_shutdown_pending = false;
            portEXIT_CRITICAL(&s_ble_shutdown_lock);
            continue;
        }

        uint32_t now_ms = route_now_ms();
        if (user_halow_shutdown_pending && device_sleep_policy_enabled()) {
            portENTER_CRITICAL(&s_ble_shutdown_lock);
            s_user_halow_shutdown_pending = false;
            portEXIT_CRITICAL(&s_ble_shutdown_lock);
            ESP_LOGI(TAG,
                     "User-mode BLE disconnected; stopping beacons and unloading HaLow");
            edgez_platform_get()->network_shutdown_halow();
        }
        if (pending && (int32_t)(now_ms - deadline_ms) >= 0) {
            device_mode_ble_shutdown_cancel();
            ESP_LOGI(TAG, "BLE disconnected deadline reached; disabling BLE");
            esp_err_t err = edgez_platform_get()->ble_set_enabled(false);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "BLE shutdown failed: %s", esp_err_to_name(err));
            }
        }

        if (!sleep_pending) {
            continue;
        }
        if (!device_sleep_policy_enabled()) {
            mode_deep_sleep_cancel();
            continue;
        }
        if ((int32_t)(now_ms - sleep_deadline_ms) < 0) {
            continue;
        }

        device_mode_ble_shutdown_cancel();
        mode_deep_sleep_cancel();
        ESP_LOGI(TAG,
                 "Device disconnected deadline reached; preparing deep sleep");
        edgez_platform_get()->led_prepare_for_sleep();
        if (edgez_platform_get()->ble_is_enabled()) {
            esp_err_t ble_err = edgez_platform_get()->ble_set_enabled(false);
            if (ble_err != ESP_OK) {
                ESP_LOGW(TAG,
                         "BLE shutdown before deep sleep failed: %s",
                         esp_err_to_name(ble_err));
            }
        }
        esp_err_t prep_err = edgez_platform_get()->network_prepare_for_deep_sleep();
        if (prep_err != ESP_OK) {
            ESP_LOGW(TAG,
                     "Network shutdown before deep sleep completed with warning: %s",
                     esp_err_to_name(prep_err));
        }
        ESP_LOGI(TAG, "Entering device deep sleep");
        esp_deep_sleep_start();
    }
}

static bool batman_route_lookup(uint64_t target, uint64_t *next_hop_out)
{
    target &= 0xffffffffffffULL;
    if (mac_is_zero_u64(target) || mac_is_broadcast_u64(target) || !next_hop_out) {
        return false;
    }
    const edgez_platform_api_t *platform = edgez_platform_get();
    if (!platform || !platform->halow_lookup_route) {
        return false;
    }
    uint8_t destination[6] = {0};
    uint8_t next_hop[6] = {0};
    u64_to_mac_bytes(target, destination);
    if (!platform->halow_lookup_route(destination, next_hop, NULL, NULL, NULL) ||
        mac_is_zero_bytes(next_hop)) {
        return false;
    }
    *next_hop_out = mac_to_u64(next_hop);
    return true;
}

static bool batman_route_lookup_hop(uint64_t target,
                                    uint64_t *next_hop_out,
                                    uint8_t *hop_out)
{
    target &= 0xffffffffffffULL;
    if (mac_is_zero_u64(target) || mac_is_broadcast_u64(target) ||
        !next_hop_out) {
        return false;
    }

    const edgez_platform_api_t *platform = edgez_platform_get();
    if (!platform || !platform->halow_lookup_route) {
        return false;
    }
    uint8_t destination[6] = {0};
    uint8_t next_hop[6] = {0};
    uint8_t hop_count = 0;
    u64_to_mac_bytes(target, destination);
    if (!platform->halow_lookup_route(destination, next_hop, &hop_count,
                                      NULL, NULL) ||
        hop_count == 0 || mac_is_zero_bytes(next_hop)) {
        return false;
    }
    *next_hop_out = mac_to_u64(next_hop);
    /* Existing speed-policy code represents a direct destination as hop 0;
     * BATMAN represents the same route as one link. Preserve that contract. */
    if (hop_out) *hop_out = (uint8_t)(hop_count - 1U);
    return true;
}

typedef struct {
    bool valid;
    uint64_t source;
    uint64_t target;
    uint64_t transfer_id;
    uint64_t next_hop;
    uint32_t last_used_ms;
    uint8_t requested_hops;
    uint8_t completed_hop;
} edgez_speed_route_pin_t;

static edgez_speed_route_pin_t
    s_speed_route_pins[EDGEZ_SPEED_ROUTE_PIN_TABLE_SIZE];
static portMUX_TYPE s_speed_route_pin_lock = portMUX_INITIALIZER_UNLOCKED;

static bool speed_route_pin_get(uint64_t source, uint64_t target,
                                uint64_t transfer_id, uint8_t requested_hops,
                                uint8_t completed_hop, uint64_t *next_hop_out)
{
    if (!next_hop_out) return false;
    const uint32_t now = route_now_ms();
    bool found = false;
    portENTER_CRITICAL(&s_speed_route_pin_lock);
    for (size_t i = 0; i < EDGEZ_SPEED_ROUTE_PIN_TABLE_SIZE; ++i) {
        edgez_speed_route_pin_t *pin = &s_speed_route_pins[i];
        if (pin->valid && (uint32_t)(now - pin->last_used_ms) >=
                              EDGEZ_SPEED_ROUTE_PIN_TTL_MS) {
            pin->valid = false;
        }
        if (pin->valid && pin->source == source && pin->target == target &&
            pin->transfer_id == transfer_id &&
            pin->requested_hops == requested_hops &&
            pin->completed_hop == completed_hop) {
            pin->last_used_ms = now;
            *next_hop_out = pin->next_hop;
            found = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_speed_route_pin_lock);
    return found;
}

static void speed_route_pin_put(uint64_t source, uint64_t target,
                                uint64_t transfer_id, uint8_t requested_hops,
                                uint8_t completed_hop, uint64_t next_hop)
{
    const uint32_t now = route_now_ms();
    size_t slot = 0;
    uint32_t oldest_age = 0;
    portENTER_CRITICAL(&s_speed_route_pin_lock);
    for (size_t i = 0; i < EDGEZ_SPEED_ROUTE_PIN_TABLE_SIZE; ++i) {
        edgez_speed_route_pin_t *pin = &s_speed_route_pins[i];
        if (!pin->valid ||
            (pin->source == source && pin->target == target &&
             pin->transfer_id == transfer_id &&
             pin->requested_hops == requested_hops &&
             pin->completed_hop == completed_hop)) {
            slot = i;
            oldest_age = UINT32_MAX;
            break;
        }
        const uint32_t age = now - pin->last_used_ms;
        if (age >= oldest_age) {
            oldest_age = age;
            slot = i;
        }
    }
    s_speed_route_pins[slot] = (edgez_speed_route_pin_t) {
        .valid = true,
        .source = source,
        .target = target,
        .transfer_id = transfer_id,
        .next_hop = next_hop,
        .last_used_ms = now,
        .requested_hops = requested_hops,
        .completed_hop = completed_hop,
    };
    portEXIT_CRITICAL(&s_speed_route_pin_lock);
}

static bool speed_random_direct_peer(uint64_t exclude_a,
                                     uint64_t exclude_b,
                                     uint64_t exclude_c,
                                     uint64_t route_key,
                                     uint64_t *peer_out)
{
    if (!peer_out) return false;
    exclude_a &= 0xffffffffffffULL;
    exclude_b &= 0xffffffffffffULL;
    exclude_c &= 0xffffffffffffULL;
    const edgez_platform_api_t *platform = edgez_platform_get();
    if (!platform || !platform->halow_select_direct_peer) return false;
    uint8_t exclude_a_mac[6] = {0};
    uint8_t exclude_b_mac[6] = {0};
    uint8_t exclude_c_mac[6] = {0};
    uint8_t peer[6] = {0};
    u64_to_mac_bytes(exclude_a, exclude_a_mac);
    u64_to_mac_bytes(exclude_b, exclude_b_mac);
    u64_to_mac_bytes(exclude_c, exclude_c_mac);
    if (!platform->halow_select_direct_peer(exclude_a_mac, exclude_b_mac,
                                             exclude_c_mac,
                                             route_key, peer)) return false;
    *peer_out = mac_to_u64(peer);
    return true;
}

/* Mode 2 is the only explicit path-length override. If BATMAN already has a
 * two-link route, address the first BATMAN hop as a waypoint. If the final
 * destination is directly connected, choose a different direct peer and let
 * that peer route the second leg. Modes 0, 1, and 3 remain automatic and are
 * sent directly to BATMAN's final-destination lookup. */
static bool speed_two_hop_waypoint(uint64_t target,
                                   uint64_t source,
                                   uint64_t transfer_id,
                                   uint64_t *waypoint_out)
{
    if (!waypoint_out) return false;
    if (speed_route_pin_get(source, target, transfer_id, 2, 0,
                            waypoint_out)) {
        return true;
    }

    uint64_t route_next = 0;
    uint8_t route_hop = 0;
    if (!batman_route_lookup_hop(target, &route_next, &route_hop)) {
        return false;
    }

    bool selected = false;
    if (route_hop == 1U) {
        /* BATMAN reports two radio links using the legacy value one. */
        *waypoint_out = route_next;
        selected = true;
    } else if (route_hop == 0U) {
        /* The final destination is one link away. Force one different direct
         * peer to become the application waypoint for this transfer. */
        selected = speed_random_direct_peer(target, source, 0,
                                             transfer_id, waypoint_out);
    }

    if (selected) {
        speed_route_pin_put(source, target, transfer_id, 2, 0,
                            *waypoint_out);
    }
    return selected;
}

static bool seen_message_drop_higher_hop(uint64_t high,
                                         uint64_t low,
                                         uint32_t sequence,
                                         uint8_t hop,
                                         uint8_t *best_hop_out)
{
    if (high == 0 && low == 0) {
        return false;
    }

    bool drop = false;
    portENTER_CRITICAL(&s_seen_message_lock);
    for (int i = 0; i < EDGEZ_SEEN_MESSAGE_TABLE_SIZE; ++i) {
        if (s_seen_messages[i].valid &&
            s_seen_messages[i].high == high &&
            s_seen_messages[i].low == low &&
            s_seen_messages[i].sequence == sequence) {
            if (best_hop_out) {
                *best_hop_out = s_seen_messages[i].best_hop;
            }
            if (s_seen_messages[i].best_hop == 0) {
                s_seen_messages[i].best_hop = hop;
                if (best_hop_out) {
                    *best_hop_out = hop;
                }
            } else if (hop > s_seen_messages[i].best_hop) {
                drop = true;
            } else if (hop < s_seen_messages[i].best_hop) {
                s_seen_messages[i].best_hop = hop;
                if (best_hop_out) {
                    *best_hop_out = hop;
                }
            }
            portEXIT_CRITICAL(&s_seen_message_lock);
            return drop;
        }
    }

    s_seen_messages[s_seen_message_replace_index].valid = true;
    s_seen_messages[s_seen_message_replace_index].high = high;
    s_seen_messages[s_seen_message_replace_index].low = low;
    s_seen_messages[s_seen_message_replace_index].sequence = sequence;
    s_seen_messages[s_seen_message_replace_index].best_hop = hop;
    if (best_hop_out) {
        *best_hop_out = hop;
    }
    s_seen_message_replace_index =
        (uint8_t)((s_seen_message_replace_index + 1U) % EDGEZ_SEEN_MESSAGE_TABLE_SIZE);
    portEXIT_CRITICAL(&s_seen_message_lock);
    return false;
}

static uint64_t read_u64_be(const uint8_t *in)
{
    uint64_t value = 0;
    for (size_t i = 0; i < sizeof(uint64_t); ++i) {
        value = (value << 8) | in[i];
    }
    return value;
}

static uint32_t read_u32_be(const uint8_t *in)
{
    uint32_t value = 0;
    for (size_t i = 0; i < sizeof(uint32_t); ++i) {
        value = (value << 8) | in[i];
    }
    return value;
}

static void generate_network_packet_message_id(uint64_t *message_id_high, uint64_t *message_id_low)
{
    if (!message_id_high || !message_id_low) {
        return;
    }
    *message_id_high = ((uint64_t)esp_random() << 32U) | esp_random();
    *message_id_low = ((uint64_t)esp_random() << 32U) | esp_random();
    if (*message_id_high == 0 && *message_id_low == 0) {
        *message_id_low = 1;
    }
}

static bool encode_network_packet(const ai_edgez_halow_NetworkPacket *msg,
                                  uint8_t *out,
                                  size_t out_size,
                                  uint16_t *out_len)
{
    pb_ostream_t stream = pb_ostream_from_buffer(out, out_size);
    if (!pb_encode(&stream, ai_edgez_halow_NetworkPacket_fields, msg)) {
        ESP_LOGW(TAG, "NetworkPacket encode failed: %s", PB_GET_ERROR(&stream));
        return false;
    }
    *out_len = (uint16_t)stream.bytes_written;
    return true;
}

static bool send_network_packet(const ai_edgez_halow_NetworkPacket *msg)
{
    uint8_t encoded[ai_edgez_halow_NetworkPacket_size] = {0};
    uint16_t encoded_len = 0;
    if (!encode_network_packet(msg, encoded, sizeof(encoded), &encoded_len)) {
        return false;
    }

    if (!bridge_queue_mobile_rx_frame(encoded, encoded_len,
                                      EDGEZ_MOBILE_RX_CONTROL,
                                      pdMS_TO_TICKS(20))) {
        ESP_LOGW(TAG, "Mobile control queue enqueue failed len=%u",
                 (unsigned)encoded_len);
        return false;
    }
    return true;
}

static bool reliable_packet_is_direct_unicast(const ai_edgez_halow_NetworkPacket *msg)
{
    if (!msg || msg->which_body != ai_edgez_halow_NetworkPacket_msg_tag ||
        msg->operation == ai_edgez_halow_Operation_ACKNOWLEDGE ||
        (msg->body.msg.message_id_high == 0 && msg->body.msg.message_id_low == 0)) {
        return false;
    }
    uint64_t target = msg->to & 0xffffffffffffULL;
    return !mac_is_zero_u64(target) && !mac_is_broadcast_u64(target) &&
           !halow_sync_is_public_channel(target);
}

static bool global_buffer_packet_is_request(const ai_edgez_halow_NetworkPacket *msg)
{
    if (!msg ||
        msg->operation != ai_edgez_halow_Operation_REQUEST ||
        msg->which_body != ai_edgez_halow_NetworkPacket_msg_tag ||
        msg->body.msg.mime != ai_edgez_halow_Mime_MIME_BINARY) {
        return false;
    }
    const bool full_request =
           msg->body.msg.payload.size == EDGEZ_GLOBAL_BUFFER_REQUEST_SIZE &&
           memcmp(msg->body.msg.payload.bytes,
                  EDGEZ_GLOBAL_BUFFER_REQUEST_MAGIC,
                  sizeof(EDGEZ_GLOBAL_BUFFER_REQUEST_MAGIC)) == 0;
    const bool chunk_request =
           msg->body.msg.payload.size == EDGEZ_GLOBAL_BUFFER_CHUNK_REQUEST_SIZE &&
           memcmp(msg->body.msg.payload.bytes,
                  EDGEZ_GLOBAL_BUFFER_CHUNK_REQUEST_MAGIC,
                  sizeof(EDGEZ_GLOBAL_BUFFER_CHUNK_REQUEST_MAGIC)) == 0;
    return full_request || chunk_request;
}

static bool global_buffer_packet_is_status(const ai_edgez_halow_NetworkPacket *msg)
{
    return msg &&
           msg->which_body == ai_edgez_halow_NetworkPacket_msg_tag &&
           msg->body.msg.mime == ai_edgez_halow_Mime_MIME_BINARY &&
           msg->body.msg.payload.size == EDGEZ_GLOBAL_BUFFER_RESPONSE_SIZE &&
           memcmp(msg->body.msg.payload.bytes,
                  EDGEZ_GLOBAL_BUFFER_RESPONSE_MAGIC,
                  sizeof(EDGEZ_GLOBAL_BUFFER_RESPONSE_MAGIC)) == 0;
}

static bool global_buffer_packet_is_chunk(const ai_edgez_halow_NetworkPacket *msg)
{
    return msg &&
           msg->operation == ai_edgez_halow_Operation_REQUEST &&
           msg->which_body == ai_edgez_halow_NetworkPacket_msg_tag &&
           msg->body.msg.mime == ai_edgez_halow_Mime_MIME_BINARY &&
           msg->body.msg.payload.size > EDGEZ_CONVERSATION_CHUNK_HEADER_SIZE &&
           memcmp(msg->body.msg.payload.bytes,
                  EDGEZ_CONVERSATION_CHUNK_MAGIC,
                  EDGEZ_CONVERSATION_CHUNK_MAGIC_SIZE) == 0;
}

static bool global_buffer_packet_is_sensor_exchange(const ai_edgez_halow_NetworkPacket *msg)
{
    if (!msg) {
        return false;
    }
    if (msg->operation == ai_edgez_halow_Operation_ACKNOWLEDGE &&
        msg->which_body == ai_edgez_halow_NetworkPacket_msg_tag &&
        msg->body.msg.sequence != 0) {
        return true;
    }
    return global_buffer_packet_is_request(msg) ||
           global_buffer_packet_is_status(msg) ||
           global_buffer_packet_is_chunk(msg);
}

static void reliable_pending_store(const ai_edgez_halow_NetworkPacket *msg,
                                   const uint8_t *payload,
                                   size_t payload_len,
                                   uint64_t next_hop)
{
    if (!msg || !payload || payload_len == 0 || payload_len > ai_edgez_halow_NetworkPacket_size) {
        return;
    }

    int free_slot = -1;
    int replace_slot = 0;
    uint32_t oldest_ms = UINT32_MAX;
    uint32_t now = route_now_ms();
    uint64_t route_next_hop = next_hop;
    uint8_t route_hop = 0;
    bool routed = batman_route_lookup_hop(msg->to, &route_next_hop,
                                           &route_hop);
    /* batman_route_lookup_hop preserves the legacy convention where a direct
     * route is hop 0. Add 180 ms for every additional radio hop so multi-hop
     * ACKs do not trigger premature duplicate retransmissions. */
    uint32_t ack_timeout_ms = 250U + (routed ? (uint32_t)route_hop * 180U : 0U);
    if (ack_timeout_ms > 1200U) ack_timeout_ms = 1200U;

    portENTER_CRITICAL(&s_reliable_pending_lock);
    for (int i = 0; i < EDGEZ_RELIABLE_PENDING_TABLE_SIZE; ++i) {
        if (s_reliable_pending[i].valid &&
            s_reliable_pending[i].message_id_high == msg->body.msg.message_id_high &&
            s_reliable_pending[i].message_id_low == msg->body.msg.message_id_low &&
            s_reliable_pending[i].sequence == msg->body.msg.sequence) {
            replace_slot = i;
            free_slot = i;
            break;
        }
        if (!s_reliable_pending[i].valid && free_slot < 0) {
            free_slot = i;
        }
        if (s_reliable_pending[i].last_send_ms < oldest_ms) {
            oldest_ms = s_reliable_pending[i].last_send_ms;
            replace_slot = i;
        }
    }

    int slot = (free_slot >= 0) ? free_slot : replace_slot;
    s_reliable_pending[slot].valid = true;
    s_reliable_pending[slot].message_id_high = msg->body.msg.message_id_high;
    s_reliable_pending[slot].message_id_low = msg->body.msg.message_id_low;
    s_reliable_pending[slot].sequence = msg->body.msg.sequence;
    s_reliable_pending[slot].from = msg->from;
    s_reliable_pending[slot].to = msg->to;
    s_reliable_pending[slot].next_hop = next_hop;
    s_reliable_pending[slot].max_hop = default_network_packet_max_hop();
    s_reliable_pending[slot].attempts = 1;
    s_reliable_pending[slot].last_send_ms = now;
    s_reliable_pending[slot].ack_timeout_ms = ack_timeout_ms;
    s_reliable_pending[slot].payload_len = payload_len;
    memcpy(s_reliable_pending[slot].payload, payload, payload_len);
    portEXIT_CRITICAL(&s_reliable_pending_lock);

    MESH_DEBUG_LOGI(
             "NetworkPacket reliable track message_id=%016llx-%016llx seq=%lu from=0x%012llx to=0x%012llx next=0x%012llx len=%u ack_timeout=%lu",
             (unsigned long long)msg->body.msg.message_id_high,
             (unsigned long long)msg->body.msg.message_id_low,
             (unsigned long)msg->body.msg.sequence,
             (unsigned long long)(msg->from & 0xffffffffffffULL),
             (unsigned long long)(msg->to & 0xffffffffffffULL),
             (unsigned long long)(next_hop & 0xffffffffffffULL),
             (unsigned)payload_len,
             (unsigned long)ack_timeout_ms);
}

static bool reliable_pending_ack(uint64_t message_id_high,
                                 uint64_t message_id_low,
                                 uint32_t sequence)
{
    bool matched = false;
    portENTER_CRITICAL(&s_reliable_pending_lock);
    for (int i = 0; i < EDGEZ_RELIABLE_PENDING_TABLE_SIZE; ++i) {
        if (!s_reliable_pending[i].valid ||
            s_reliable_pending[i].message_id_high != message_id_high ||
            s_reliable_pending[i].message_id_low != message_id_low) {
            continue;
        }
        if (s_reliable_pending[i].sequence == sequence || sequence == 0) {
            s_reliable_pending[i].valid = false;
            matched = true;
        }
    }
    portEXIT_CRITICAL(&s_reliable_pending_lock);

    MESH_DEBUG_LOGI(
             "NetworkPacket reliable ACK message_id=%016llx-%016llx seq=%lu matched=%u",
             (unsigned long long)message_id_high,
             (unsigned long long)message_id_low,
             (unsigned long)sequence,
             matched ? 1 : 0);
    return matched;
}

static bool reliable_pending_is_active(uint64_t message_id_high,
                                       uint64_t message_id_low,
                                       uint32_t sequence)
{
    bool active = false;
    portENTER_CRITICAL(&s_reliable_pending_lock);
    for (int i = 0; i < EDGEZ_RELIABLE_PENDING_TABLE_SIZE; ++i) {
        if (s_reliable_pending[i].valid &&
            s_reliable_pending[i].message_id_high == message_id_high &&
            s_reliable_pending[i].message_id_low == message_id_low &&
            s_reliable_pending[i].sequence == sequence) {
            active = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_reliable_pending_lock);
    return active;
}

static void global_buffer_tx_schedule_send(void)
{
    TaskHandle_t worker = NULL;
    portENTER_CRITICAL(&s_global_buffer_tx_lock);
    if (s_global_buffer_tx_active) {
        s_global_buffer_send_pending = true;
        worker = s_reliable_tx_task;
    }
    portEXIT_CRITICAL(&s_global_buffer_tx_lock);
    if (worker != NULL) {
        xTaskNotifyGive(worker);
    }
}

static esp_err_t send_reliable_ack(const ai_edgez_halow_NetworkPacket *msg,
                                   uint64_t self_mac_u64)
{
    if (!msg || msg->from == 0 ||
        msg->operation == ai_edgez_halow_Operation_ACKNOWLEDGE ||
        msg->body.msg.mime == ai_edgez_halow_Mime_MIME_VOICE_CALL ||
        (msg->body.msg.message_id_high == 0 && msg->body.msg.message_id_low == 0)) {
        return ESP_OK;
    }
    /* The complete global-buffer protocol is application-retried: the app
     * retries GBR1/GBR2 after its 2 second timeout and asks for missing chunk
     * indexes. Do not add packet ACK traffic for requests, status replies, or
     * image chunks. */
    if (global_buffer_packet_is_sensor_exchange(msg)) {
        return ESP_OK;
    }
    if (device_mode_is_enabled() && !global_buffer_packet_is_sensor_exchange(msg)) {
        uint64_t target = msg->to & 0xffffffffffffULL;
        ESP_LOGI(TAG,
                 "Reliable ACK suppressed in device mode msg=%016llx-%016llx to=0x%012llx",
                 (unsigned long long)msg->body.msg.message_id_high,
                 (unsigned long long)msg->body.msg.message_id_low,
                 (unsigned long long)target);
        return ESP_OK;
    }

    ai_edgez_halow_NetworkPacket ack = ai_edgez_halow_NetworkPacket_init_zero;
    ack.which_body = ai_edgez_halow_NetworkPacket_msg_tag;
    ack.body.msg.message_id_high = msg->body.msg.message_id_high;
    ack.body.msg.message_id_low = msg->body.msg.message_id_low;
    ack.from = self_mac_u64;
    ack.to = msg->from;
    ack.operation = ai_edgez_halow_Operation_ACKNOWLEDGE;
    ack.interface = ai_edgez_halow_Interface_HALOW;
    ack.body.msg.sequence = msg->body.msg.sequence;
    ack.body.msg.mime = ai_edgez_halow_Mime_MIME_TEXT;
    uint32_t ack_max_hop = default_network_packet_max_hop();

    uint8_t encoded[ai_edgez_halow_NetworkPacket_size] = {0};
    uint16_t encoded_len = 0;
    if (!encode_network_packet(&ack, encoded, sizeof(encoded), &encoded_len)) {
        return ESP_FAIL;
    }

    uint64_t next_hop = msg->from;
    (void)batman_route_lookup(msg->from, &next_hop);
    esp_err_t err = edgez_platform_get()->halow_send_mesh_payload_via(encoded,
                                                              encoded_len,
                                                              ack.from,
                                                              ack.to,
                                                              next_hop,
                                                              ack.body.msg.message_id_high,
                                                              ack.body.msg.message_id_low,
                                                              ack_max_hop,
                                                              ack.body.msg.sequence);
    MESH_DEBUG_LOGI(
             "NetworkPacket reliable ACK send message_id=%016llx-%016llx seq=%lu to=0x%012llx next=0x%012llx err=%s",
             (unsigned long long)ack.body.msg.message_id_high,
             (unsigned long long)ack.body.msg.message_id_low,
             (unsigned long)ack.body.msg.sequence,
             (unsigned long long)(ack.to & 0xffffffffffffULL),
             (unsigned long long)(next_hop & 0xffffffffffffULL),
             esp_err_to_name(err));
    return err;
}

static void reliable_tx_task(void *arg)
{
    (void)arg;
    static uint8_t retry_payload[ai_edgez_halow_NetworkPacket_size];

    while (true) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        bool send_global_buffer_chunk = false;
        portENTER_CRITICAL(&s_global_buffer_tx_lock);
        if (s_global_buffer_send_pending) {
            s_global_buffer_send_pending = false;
            send_global_buffer_chunk = true;
        }
        portEXIT_CRITICAL(&s_global_buffer_tx_lock);
        if (send_global_buffer_chunk) {
            global_buffer_tx_send_current_chunk();
        }

        uint64_t message_id_high = 0;
        uint64_t message_id_low = 0;
        uint32_t sequence = 0;
        uint64_t from = 0;
        uint64_t to = 0;
        uint64_t next_hop = 0;
        uint32_t max_hop = 0;
        uint8_t attempts = 0;
        size_t payload_len = 0;
        bool have_retry = false;
        bool exhausted = false;
        bool payload_loaded = false;
        uint32_t now = route_now_ms();

        portENTER_CRITICAL(&s_reliable_pending_lock);
        for (int i = 0; i < EDGEZ_RELIABLE_PENDING_TABLE_SIZE; ++i) {
            edgez_reliable_pending_t *entry = &s_reliable_pending[i];
            if (!entry->valid) {
                continue;
            }
            uint32_t ack_timeout_ms = entry->ack_timeout_ms != 0
                                          ? entry->ack_timeout_ms
                                          : EDGEZ_RELIABLE_ACK_TIMEOUT_MS;
            if ((uint32_t)(now - entry->last_send_ms) < ack_timeout_ms) {
                continue;
            }
            if (entry->attempts >= EDGEZ_RELIABLE_MAX_ATTEMPTS) {
                message_id_high = entry->message_id_high;
                message_id_low = entry->message_id_low;
                sequence = entry->sequence;
                attempts = entry->attempts;
                entry->valid = false;
                exhausted = true;
                have_retry = true;
                break;
            }
            entry->attempts++;
            entry->last_send_ms = now;
            message_id_high = entry->message_id_high;
            message_id_low = entry->message_id_low;
            sequence = entry->sequence;
            from = entry->from;
            to = entry->to;
            next_hop = entry->next_hop;
            max_hop = entry->max_hop;
            attempts = entry->attempts;
            payload_len = entry->payload_len;
            if (payload_len <= sizeof(retry_payload)) {
                memcpy(retry_payload, entry->payload, payload_len);
                payload_loaded = true;
            } else {
                entry->valid = false;
            }
            have_retry = true;
            break;
        }
        portEXIT_CRITICAL(&s_reliable_pending_lock);

        if (!have_retry) {
            continue;
        }
        if (!payload_loaded && !exhausted) {
            continue;
        }
        if (exhausted) {
            ESP_LOGW(TAG,
                     "NetworkPacket reliable failed message_id=%016llx-%016llx seq=%lu attempts=%u",
                     (unsigned long long)message_id_high,
                     (unsigned long long)message_id_low,
                     (unsigned long)sequence,
                     (unsigned)attempts);
            continue;
        }

        /* An ACK can clear this entry after the retry task copied it but
         * before transmission. Do not emit that stale retry alongside the
         * next chunk scheduled by the ACK handler. */
        if (!reliable_pending_is_active(message_id_high, message_id_low, sequence)) {
            MESH_DEBUG_LOGI(
                     "NetworkPacket reliable retry canceled after ACK message_id=%016llx-%016llx seq=%lu",
                     (unsigned long long)message_id_high,
                     (unsigned long long)message_id_low,
                     (unsigned long)sequence);
            continue;
        }

        /* The route may have changed while the ACK timer was running. Retry
         * through BATMAN's current best next hop instead of repeatedly using
         * the candidate selected for the first transmission. */
        uint64_t refreshed_next_hop = to;
        if (batman_route_lookup(to, &refreshed_next_hop)) {
            next_hop = refreshed_next_hop;
        }

        esp_err_t err = edgez_platform_get()->halow_send_mesh_payload_via(retry_payload,
                                                                  payload_len,
                                                                  from,
                                                                  to,
                                                                  next_hop,
                                                                  message_id_high,
                                                                  message_id_low,
                                                                  max_hop,
                                                                  sequence);
        MESH_DEBUG_LOGI(
                 "NetworkPacket reliable retry message_id=%016llx-%016llx seq=%lu attempt=%u err=%s",
                 (unsigned long long)message_id_high,
                 (unsigned long long)message_id_low,
                 (unsigned long)sequence,
                 (unsigned)attempts,
                 esp_err_to_name(err));
    }
}

typedef struct {
    bool valid;
    uint64_t target;
    uint32_t sampled_ms;
    UBaseType_t limit;
    uint8_t tq;
    uint8_t hops;
    uint32_t route_age_ms;
} edgez_realtime_path_cache_t;

static edgez_realtime_path_cache_t
    s_realtime_path_cache[EDGEZ_REALTIME_PATH_CACHE_SIZE];
static uint8_t s_realtime_path_cache_replace_index;
static portMUX_TYPE s_realtime_path_cache_lock = portMUX_INITIALIZER_UNLOCKED;

static UBaseType_t realtime_tx_path_limit(uint64_t target, uint8_t *tq_out,
                                          uint8_t *hops_out,
                                          uint32_t *route_age_ms_out)
{
    const uint32_t now_ms = route_now_ms();
    uint8_t tq = 0;
    uint8_t hops = 0;
    uint32_t route_age_ms = UINT32_MAX;
    UBaseType_t limit = 4;
    const edgez_platform_api_t *platform = edgez_platform_get();
    uint8_t destination[EDGEZ_ROUTE_MAC_LEN] = {0};
    uint8_t next_hop[EDGEZ_ROUTE_MAC_LEN] = {0};

    /* Route selection has already run for this packet. Cache the secondary
     * lookup used only for queue sizing so a high-rate stream does not walk
     * the BATMAN route table again for every frame. */
    portENTER_CRITICAL(&s_realtime_path_cache_lock);
    for (size_t i = 0; i < EDGEZ_REALTIME_PATH_CACHE_SIZE; ++i) {
        edgez_realtime_path_cache_t *cached = &s_realtime_path_cache[i];
        if (cached->valid && cached->target == target &&
            (uint32_t)(now_ms - cached->sampled_ms) <
                EDGEZ_REALTIME_PATH_CACHE_TTL_MS) {
            limit = cached->limit;
            tq = cached->tq;
            hops = cached->hops;
            route_age_ms = cached->route_age_ms;
            portEXIT_CRITICAL(&s_realtime_path_cache_lock);
            if (tq_out) *tq_out = tq;
            if (hops_out) *hops_out = hops;
            if (route_age_ms_out) *route_age_ms_out = route_age_ms;
            return limit;
        }
    }
    portEXIT_CRITICAL(&s_realtime_path_cache_lock);

    for (size_t i = 0; i < EDGEZ_ROUTE_MAC_LEN; ++i) {
        destination[EDGEZ_ROUTE_MAC_LEN - 1U - i] = (uint8_t)(target >> (i * 8U));
    }
    bool have_path = platform && platform->halow_lookup_route &&
                     platform->halow_lookup_route(destination, next_hop,
                                                  &hops, &tq,
                                                  &route_age_ms);
    if (have_path) {
        if (tq >= 208U) {
            limit = EDGEZ_REALTIME_TX_QUEUE_DEPTH;
        } else if (tq >= 160U) {
            limit = 12;
        } else if (tq >= 112U) {
            limit = 8;
        } else {
            limit = 4;
        }
        /* Each additional radio hop consumes airtime on another link. Do not
         * allow a long source-side queue to amplify congestion downstream. */
        if (hops >= 3U && limit > 4U) {
            limit = 4U;
        } else if (hops == 2U && limit > 8U) {
            limit = 8U;
        }
        if (route_age_ms > 4000U && limit > 4U) {
            limit = 4U;
        }
    }

    portENTER_CRITICAL(&s_realtime_path_cache_lock);
    edgez_realtime_path_cache_t *cached =
        &s_realtime_path_cache[s_realtime_path_cache_replace_index];
    *cached = (edgez_realtime_path_cache_t) {
        .valid = true,
        .target = target,
        .sampled_ms = now_ms,
        .limit = limit,
        .tq = tq,
        .hops = hops,
        .route_age_ms = route_age_ms,
    };
    s_realtime_path_cache_replace_index =
        (uint8_t)((s_realtime_path_cache_replace_index + 1U) %
                  EDGEZ_REALTIME_PATH_CACHE_SIZE);
    portEXIT_CRITICAL(&s_realtime_path_cache_lock);

    if (tq_out) *tq_out = tq;
    if (hops_out) *hops_out = hops;
    if (route_age_ms_out) *route_age_ms_out = route_age_ms;
    return limit;
}

static esp_err_t voice_tx_enqueue(const uint8_t *payload,
                                  size_t payload_len,
                                  const ai_edgez_halow_NetworkPacket *msg,
                                  uint32_t max_hop,
                                  uint64_t next_hop,
                                  bool forward_from_mobile,
                                  uint32_t mobile_initial_hop)
{
    if (!payload || !msg || !s_voice_tx_queue || payload_len == 0 ||
        payload_len > ai_edgez_halow_NetworkPacket_size) {
        return ESP_ERR_INVALID_ARG;
    }

    edgez_voice_tx_item_t item = {
        .payload_len = (uint16_t)payload_len,
        .from = msg->from,
        .to = msg->to,
        .next_hop = next_hop,
        .message_id_high = msg->body.msg.message_id_high,
        .message_id_low = msg->body.msg.message_id_low,
        .max_hop = max_hop,
        .sequence = msg->body.msg.sequence,
        .forward_from_mobile = forward_from_mobile,
        .mobile_initial_hop = mobile_initial_hop,
    };
    memcpy(item.payload, payload, payload_len);

    const bool is_speed = payload_len >= EDGEZ_SPEED_RAW_MAGIC_SIZE &&
                          memcmp(payload, EDGEZ_SPEED_RAW_MAGIC,
                                 EDGEZ_SPEED_RAW_MAGIC_SIZE) == 0;

    if (is_speed) {
        if (xQueueSend(s_voice_tx_queue, &item, 0) == pdPASS) {
            return ESP_OK;
        }

        uint32_t drops = (uint32_t)atomic_fetch_add_explicit(
                             &s_realtime_tx_queue_drops, 1,
                             memory_order_relaxed) + 1U;
        if (drops == 1U || (drops % 64U) == 0U) {
            ESP_LOGW(TAG,
                     "Speed TX queue full drops=%lu seq=%lu queue=%u nonblocking=1",
                     (unsigned long)drops,
                     (unsigned long)msg->body.msg.sequence,
                     (unsigned)uxQueueMessagesWaiting(s_voice_tx_queue));
        }
        return ESP_ERR_TIMEOUT;
    }

    uint8_t path_tq = 0;
    uint8_t path_hops = 0;
    uint32_t route_age_ms = UINT32_MAX;
    UBaseType_t path_limit = realtime_tx_path_limit(
        msg->to, &path_tq, &path_hops, &route_age_ms);

    bool dropped_oldest = false;
    edgez_voice_tx_item_t dropped;
    if (uxQueueMessagesWaiting(s_voice_tx_queue) >= path_limit &&
        xQueueReceive(s_voice_tx_queue, &dropped, 0) == pdPASS) {
        dropped_oldest = true;
    }
    bool enqueued = xQueueSend(s_voice_tx_queue, &item, 0) == pdPASS;
    if (!enqueued && !dropped_oldest &&
        xQueueReceive(s_voice_tx_queue, &dropped, 0) == pdPASS) {
        dropped_oldest = true;
        enqueued = xQueueSend(s_voice_tx_queue, &item, 0) == pdPASS;
    }
    if (!dropped_oldest) {
        return enqueued ? ESP_OK : ESP_ERR_TIMEOUT;
    }

    /* Old voice and speed-test frames have less value than the newest frame.
     * Count adaptive shedding as loss; sequence numbers let the receiver
     * include it in its observed packet-loss measurement. */
    uint32_t drops = (uint32_t)atomic_fetch_add_explicit(
                         &s_realtime_tx_queue_drops, 1,
                         memory_order_relaxed) + 1U;
    if (drops == 1U || (drops % 64U) == 0U) {
        ESP_LOGW(TAG,
                 "Realtime BATMAN TX adaptive drop count=%lu newest=%s seq=%lu queue=%u soft_limit=%u tq=%u hops=%u route_age_ms=%lu accepted=%u",
                 (unsigned long)drops,
                 is_speed ? "speed" : "voice",
                 (unsigned long)msg->body.msg.sequence,
                 (unsigned)uxQueueMessagesWaiting(s_voice_tx_queue),
                 (unsigned)path_limit,
                 (unsigned)path_tq,
                 (unsigned)path_hops,
                 (unsigned long)route_age_ms,
                 enqueued ? 1U : 0U);
    }
    return enqueued ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t halow_sync_bridge_handle_voice_to_radio(const uint8_t *payload,
                                                   size_t payload_len)
{
    if (!payload || payload_len <= EDGEZ_VOICE_BLE_ROUTE_SIZE + EDGEZ_VOICE_NONCE_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    uint64_t to = 0;
    for (size_t i = 0; i < EDGEZ_ROUTE_MAC_LEN; ++i) {
        to = (to << 8) | payload[i];
    }
    uint32_t max_hop = payload[EDGEZ_ROUTE_MAC_LEN];
    uint32_t sequence = read_u32_be(payload + EDGEZ_ROUTE_MAC_LEN + 1);
    if (mac_is_zero_u64(to) || sequence == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t self_mac[EDGEZ_ROUTE_MAC_LEN] = {0};
    if (!edgez_platform_get()->halow_get_self_mac(self_mac)) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint8_t *crypto = payload + EDGEZ_VOICE_BLE_ROUTE_SIZE;
    size_t crypto_len = payload_len - EDGEZ_VOICE_BLE_ROUTE_SIZE;
    uint8_t raw[EDGEZ_VOICE_RAW_MAGIC_SIZE + ai_edgez_halow_NetworkPacket_size];
    if (crypto_len > sizeof(raw) - EDGEZ_VOICE_RAW_MAGIC_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(raw, EDGEZ_VOICE_RAW_MAGIC, EDGEZ_VOICE_RAW_MAGIC_SIZE);
    memcpy(raw + EDGEZ_VOICE_RAW_MAGIC_SIZE, crypto, crypto_len);

    ai_edgez_halow_NetworkPacket route = ai_edgez_halow_NetworkPacket_init_zero;
    route.which_body = ai_edgez_halow_NetworkPacket_msg_tag;
    route.from = mac_to_u64(self_mac);
    route.to = to;
    route.body.msg.sequence = sequence;
    generate_network_packet_message_id(&route.body.msg.message_id_high, &route.body.msg.message_id_low);
    uint64_t next_hop = to;
    (void)batman_route_lookup(to, &next_hop);
    return voice_tx_enqueue(raw,
                            EDGEZ_VOICE_RAW_MAGIC_SIZE + crypto_len,
                            &route,
                            max_hop,
                            next_hop,
                            false,
                            0);
}

esp_err_t halow_sync_bridge_handle_speed_to_radio(const uint8_t *payload,
                                                   size_t payload_len)
{
    if (!payload || payload_len < EDGEZ_VOICE_BLE_ROUTE_SIZE +
                               EDGEZ_SPEED_FRAME_HEADER_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    uint64_t to = 0;
    for (size_t i = 0; i < EDGEZ_ROUTE_MAC_LEN; ++i) {
        to = (to << 8) | payload[i];
    }
    uint32_t max_hop = payload[EDGEZ_ROUTE_MAC_LEN];
    uint32_t sequence = read_u32_be(payload + EDGEZ_ROUTE_MAC_LEN + 1);
    const uint8_t *speed_frame = payload + EDGEZ_VOICE_BLE_ROUTE_SIZE;
    size_t speed_frame_len = payload_len - EDGEZ_VOICE_BLE_ROUTE_SIZE;
    uint64_t transfer_id = read_u64_be(speed_frame + 6);
    if (mac_is_zero_u64(to) || sequence == 0 || max_hop > 3 ||
        transfer_id == 0 || speed_frame_len < EDGEZ_SPEED_FRAME_HEADER_SIZE ||
        memcmp(speed_frame, EDGEZ_SPEED_RAW_MAGIC,
               EDGEZ_SPEED_RAW_MAGIC_SIZE) != 0 ||
        speed_frame[4] != 3) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t self_mac[EDGEZ_ROUTE_MAC_LEN] = {0};
    if (!edgez_platform_get()->halow_get_self_mac(self_mac)) {
        return ESP_ERR_INVALID_STATE;
    }

    ai_edgez_halow_NetworkPacket route = ai_edgez_halow_NetworkPacket_init_zero;
    route.which_body = ai_edgez_halow_NetworkPacket_msg_tag;
    route.from = mac_to_u64(self_mac);
    route.to = to;
    route.body.msg.sequence = sequence;
    generate_network_packet_message_id(&route.body.msg.message_id_high,
                                       &route.body.msg.message_id_low);
    uint64_t next_hop = to;
    if (max_hop == 2U &&
        !speed_two_hop_waypoint(to, route.from, transfer_id, &next_hop)) {
        ESP_LOGW(TAG,
                 "Speed two-hop waypoint unavailable transfer=%016llx seq=%lu target=0x%012llx",
                 (unsigned long long)transfer_id,
                 (unsigned long)sequence,
                 (unsigned long long)(to & 0xffffffffffffULL));
        speed_trace_record(SPEED_TRACE_MOBILE_IN, speed_frame,
                           speed_frame_len, sequence, 0, (uint8_t)max_hop,
                           s_voice_tx_queue ?
                               uxQueueMessagesWaiting(s_voice_tx_queue) : 0,
                           ESP_ERR_NOT_FOUND);
        return ESP_ERR_NOT_FOUND;
    }
    MESH_DEBUG_LOGI(
        "Speed TX policy transfer=%016llx target=0x%012llx mode=%lu rf_waypoint=0x%012llx forced_waypoint=%u",
        (unsigned long long)transfer_id,
        (unsigned long long)(to & 0xffffffffffffULL),
        (unsigned long)max_hop,
        (unsigned long long)(next_hop & 0xffffffffffffULL),
        max_hop == 2U && next_hop != to ? 1U : 0U);
    if (speed_frame[5] == 1U) {
        ESP_LOGI(TAG,
                 "Speed test route start transfer=%016llx mode=%lu final=0x%012llx rf_waypoint=0x%012llx forced=%u",
                 (unsigned long long)transfer_id,
                 (unsigned long)max_hop,
                 (unsigned long long)(to & 0xffffffffffffULL),
                 (unsigned long long)(next_hop & 0xffffffffffffULL),
                 max_hop == 2U && next_hop != to ? 1U : 0U);
    }
    esp_err_t queue_err = voice_tx_enqueue(speed_frame, speed_frame_len,
                                           &route, max_hop, next_hop,
                                           false, 0);
    speed_trace_record(SPEED_TRACE_MOBILE_IN, speed_frame, speed_frame_len,
                       sequence, 0, (uint8_t)max_hop,
                       s_voice_tx_queue ?
                           uxQueueMessagesWaiting(s_voice_tx_queue) : 0,
                       queue_err);
    return queue_err;
}

static void voice_tx_task(void *arg)
{
    (void)arg;
    static edgez_voice_tx_item_t item;

    while (true) {
        if (xQueueReceive(s_voice_tx_queue, &item, portMAX_DELAY) != pdPASS) {
            continue;
        }

        const bool is_speed = item.payload_len >= EDGEZ_SPEED_FRAME_HEADER_SIZE &&
                              memcmp(item.payload, EDGEZ_SPEED_RAW_MAGIC,
                                     EDGEZ_SPEED_RAW_MAGIC_SIZE) == 0;
        const uint8_t max_attempts = is_speed
                                         ? EDGEZ_SPEED_TX_MAX_ATTEMPTS
                                         : EDGEZ_VOICE_TX_MAX_ATTEMPTS;
        esp_err_t err = ESP_FAIL;
        uint8_t attempts = 0;
        do {
            attempts++;
            err = item.forward_from_mobile
                ? edgez_platform_get()->halow_forward_mesh_payload(
                      item.payload, item.payload_len, item.from, item.to,
                      item.next_hop, item.message_id_high, item.message_id_low,
                      item.max_hop, item.sequence, item.mobile_initial_hop)
                : edgez_platform_get()->halow_send_mesh_payload_via(
                      item.payload, item.payload_len, item.from, item.to,
                      item.next_hop, item.message_id_high, item.message_id_low,
                      item.max_hop, item.sequence);
            bool retryable = err == ESP_ERR_TIMEOUT;
#if !defined(CONFIG_BUILD_EDGEZ_FROM_SOURCE) || !CONFIG_BUILD_EDGEZ_FROM_SOURCE
            /* The legacy prebuilt adapter predates EDGEZ_RADIO_RETRY and can
             * only report generic ESP_FAIL for temporary TX pressure. */
            retryable = retryable || err == ESP_FAIL;
#endif
            if (err == ESP_OK || !retryable || attempts >= max_attempts) {
                break;
            }

            /* Keep the item at the head of the logical stream while Morse
             * returns completed DMA packets. Bulk traffic backs off more as
             * pressure persists; live voice keeps its short fixed delay. */
            uint32_t retry_delay_ms = EDGEZ_VOICE_TX_RETRY_DELAY_MS;
            if (is_speed) {
                retry_delay_ms = EDGEZ_SPEED_TX_RETRY_BASE_MS * attempts;
                if (retry_delay_ms > EDGEZ_SPEED_TX_RETRY_MAX_MS) {
                    retry_delay_ms = EDGEZ_SPEED_TX_RETRY_MAX_MS;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(retry_delay_ms));
        } while (true);
        if (is_speed) {
            speed_trace_record(
                SPEED_TRACE_RADIO_TX, item.payload, item.payload_len,
                item.sequence, (uint8_t)(item.mobile_initial_hop + 1U),
                (uint8_t)item.max_hop,
                uxQueueMessagesWaiting(s_voice_tx_queue), err);
        }
        if (is_speed && (item.payload[5] == 1 || item.payload[5] == 3)) {
            ESP_LOGD(TAG,
                     "Stream TX speed type=%u transfer=%016llx seq=%lu bytes=%u queue=%u err=%s",
                     (unsigned)item.payload[5],
                     (unsigned long long)read_u64_be(item.payload + 6),
                     (unsigned long)item.sequence,
                     (unsigned)item.payload_len,
                     (unsigned)uxQueueMessagesWaiting(s_voice_tx_queue),
                     esp_err_to_name(err));
        }
        MESH_DEBUG_LOGI(
                 "Voice TX dequeue message_id=%016llx-%016llx seq=%lu queue=%u err=%s",
                 (unsigned long long)item.message_id_high,
                 (unsigned long long)item.message_id_low,
                 (unsigned long)item.sequence,
                 (unsigned)uxQueueMessagesWaiting(s_voice_tx_queue),
                 esp_err_to_name(err));
        if (err != ESP_OK) {
            uint32_t failures = (uint32_t)atomic_fetch_add_explicit(
                                    &s_realtime_tx_send_failures, 1,
                                    memory_order_relaxed) + 1U;
            if (failures == 1U || (failures % 64U) == 0U) {
                ESP_LOGW(
                    TAG,
                    "Realtime BATMAN TX dropped after %u attempts failures=%lu kind=%s seq=%lu bytes=%u err=%s queue=%u",
                    (unsigned)attempts,
                    (unsigned long)failures,
                    is_speed ? "speed" : "voice",
                    (unsigned long)item.sequence,
                    (unsigned)item.payload_len,
                    esp_err_to_name(err),
                    (unsigned)uxQueueMessagesWaiting(s_voice_tx_queue));
            }
        }
    }
}

static void mobile_rx_task(void *arg)
{
    (void)arg;
    static edgez_mobile_rx_item_t item;

    while (true) {
        if (xQueueReceive(s_mobile_rx_queue, &item, portMAX_DELAY) != pdPASS) {
            continue;
        }
        if (item.kind == EDGEZ_MOBILE_RX_LOG) {
            atomic_store_explicit(&s_mobile_log_delivery_active, true,
                                  memory_order_release);
            bridge_send_log_frame_now(item.payload, item.payload_len);
            atomic_store_explicit(&s_mobile_log_delivery_active, false,
                                  memory_order_release);
        } else if (item.kind == EDGEZ_MOBILE_RX_CONTROL) {
            bridge_send_frame(item.payload, item.payload_len, false);
        } else {
            bridge_send_voice_frame_now(item.payload, item.payload_len);
        }
    }
}

static void global_buffer_tx_reset_state(void)
{
    bool release_serving = false;
    portENTER_CRITICAL(&s_global_buffer_tx_lock);
    memset(s_global_buffer_targets, 0, sizeof(s_global_buffer_targets));
    s_global_buffer_data = NULL;
    s_global_buffer_length = 0;
    s_global_buffer_target_count = 0;
    s_global_buffer_target_index = 0;
    s_global_buffer_tx_active = false;
    release_serving = s_global_buffer_serving_acquired;
    s_global_buffer_serving_acquired = false;
    s_global_buffer_requester = 0;
    s_global_buffer_request_message_id_high = 0;
    s_global_buffer_request_message_id_low = 0;
    s_global_buffer_targets_completed = 0;
    s_global_buffer_send_pending = false;
    portEXIT_CRITICAL(&s_global_buffer_tx_lock);
    if (release_serving) {
        edgez_platform_get()->sampling_buffer_release();
    }
}

static void global_buffer_tx_send_current_chunk(void)
{
    bool found_target = false;
    bool has_pending_send = false;
    bool send_failed = false;
    uint8_t payload_chunk[EDGEZ_GLOBAL_BUFFER_CHUNK_PAYLOAD_SIZE] = {0};
    size_t payload_len = 0;
    size_t payload_data_len = 0;
    uint16_t total_chunks = 0;
    uint32_t sequence = 0;
    uint16_t chunk_index = 0;
    uint64_t target = 0;
    uint64_t next_hop = 0;
    uint64_t next_offset = 0;
    uint32_t remaining_after_send = 0;
    uint64_t message_id_high = 0;
    uint64_t message_id_low = 0;
    uint32_t max_hop = 0;
    uint8_t source_mac[6] = {0};
    uint64_t from = 0;

    if (!edgez_platform_get()->halow_get_self_mac(source_mac)) {
        ESP_LOGW(TAG, "Global buffer TX skipped; self MAC not available");
        global_buffer_tx_reset_state();
        return;
    }
    from = mac_to_u64(source_mac);

    while (!found_target) {
        portENTER_CRITICAL(&s_global_buffer_tx_lock);
        if (!s_global_buffer_tx_active || s_global_buffer_data == NULL || s_global_buffer_length == 0 ||
            s_global_buffer_target_count == 0 || s_global_buffer_target_index >= s_global_buffer_target_count) {
            portEXIT_CRITICAL(&s_global_buffer_tx_lock);
            global_buffer_tx_reset_state();
            return;
        }

        while (s_global_buffer_target_index < s_global_buffer_target_count &&
               (!s_global_buffer_targets[s_global_buffer_target_index].active ||
                s_global_buffer_targets[s_global_buffer_target_index].next_offset >=
                    s_global_buffer_targets[s_global_buffer_target_index].end_offset)) {
            s_global_buffer_targets[s_global_buffer_target_index].active = false;
            ++s_global_buffer_target_index;
        }
        if (s_global_buffer_target_index >= s_global_buffer_target_count) {
            portEXIT_CRITICAL(&s_global_buffer_tx_lock);
            global_buffer_tx_reset_state();
            return;
        }

        edgez_global_buffer_target_t *current = &s_global_buffer_targets[s_global_buffer_target_index];
        if (current->message_id_high == 0 && current->message_id_low == 0) {
            uint64_t message_id_high_tmp = 0;
            uint64_t message_id_low_tmp = 0;
            generate_network_packet_message_id(&message_id_high_tmp, &message_id_low_tmp);
            current->message_id_high = message_id_high_tmp;
            current->message_id_low = message_id_low_tmp;
        }
        total_chunks = (uint16_t)((s_global_buffer_length + EDGEZ_GLOBAL_BUFFER_CHUNK_SIZE - 1) / EDGEZ_GLOBAL_BUFFER_CHUNK_SIZE);
        if (total_chunks == 0) {
            total_chunks = 1;
        }

        payload_data_len = current->end_offset - current->next_offset;
        if (payload_data_len > EDGEZ_GLOBAL_BUFFER_CHUNK_SIZE) {
            payload_data_len = EDGEZ_GLOBAL_BUFFER_CHUNK_SIZE;
        }
        if (payload_data_len == 0) {
            current->active = false;
            current->next_offset = 0;
            ++s_global_buffer_target_index;
            portEXIT_CRITICAL(&s_global_buffer_tx_lock);
            continue;
        }

        sequence = current->chunk_sequence + 1;
        chunk_index = (uint16_t)(sequence - 1);

        memcpy(payload_chunk + EDGEZ_CONVERSATION_CHUNK_HEADER_SIZE,
               s_global_buffer_data + current->next_offset,
               payload_data_len);
        uint8_t *payload_cursor = payload_chunk;
        memcpy(payload_cursor, EDGEZ_CONVERSATION_CHUNK_MAGIC, EDGEZ_CONVERSATION_CHUNK_MAGIC_SIZE);
        payload_cursor += EDGEZ_CONVERSATION_CHUNK_MAGIC_SIZE;
        uint64_t chunk_group_id = current->chunk_group_id != 0 ?
            current->chunk_group_id : current->message_id_low;
        *payload_cursor++ = (uint8_t)(chunk_group_id & 0xff);
        *payload_cursor++ = (uint8_t)((chunk_group_id >> 8) & 0xff);
        *payload_cursor++ = (uint8_t)((chunk_group_id >> 16) & 0xff);
        *payload_cursor++ = (uint8_t)((chunk_group_id >> 24) & 0xff);
        *payload_cursor++ = (uint8_t)((chunk_group_id >> 32) & 0xff);
        *payload_cursor++ = (uint8_t)((chunk_group_id >> 40) & 0xff);
        *payload_cursor++ = (uint8_t)((chunk_group_id >> 48) & 0xff);
        *payload_cursor++ = (uint8_t)((chunk_group_id >> 56) & 0xff);
        *payload_cursor++ = 0;
        *payload_cursor++ = 0;
        *payload_cursor++ = 0;
        *payload_cursor++ = 0;
        *payload_cursor++ = (uint8_t)(total_chunks & 0xff);
        *payload_cursor++ = (uint8_t)((total_chunks >> 8) & 0xff);
        *payload_cursor++ = (uint8_t)(chunk_index & 0xff);
        *payload_cursor++ = (uint8_t)((chunk_index >> 8) & 0xff);
        *payload_cursor++ = 0;
        current->chunk_sequence = sequence;
        next_offset = current->next_offset;
        remaining_after_send = s_global_buffer_length -
                               (current->next_offset + payload_data_len);
        message_id_high = current->message_id_high;
        message_id_low = current->message_id_low;
        target = current->target;
        next_hop = current->next_hop;
        max_hop = default_network_packet_max_hop();
        found_target = true;
        has_pending_send = true;
        portEXIT_CRITICAL(&s_global_buffer_tx_lock);
    }

    if (!has_pending_send) {
        return;
    }

    ai_edgez_halow_NetworkPacket msg = ai_edgez_halow_NetworkPacket_init_zero;
        msg.operation = ai_edgez_halow_Operation_REQUEST;
        msg.interface = ai_edgez_halow_Interface_HALOW;
        msg.body.msg.message_id_high = message_id_high;
        msg.body.msg.message_id_low = message_id_low;
        msg.from = from;
        msg.to = target;
        msg.body.msg.mime = ai_edgez_halow_Mime_MIME_BINARY;
    msg.body.msg.sequence = sequence;
    msg.which_body = ai_edgez_halow_NetworkPacket_msg_tag;
    payload_len = EDGEZ_CONVERSATION_CHUNK_HEADER_SIZE + payload_data_len;
    if (payload_len > sizeof(msg.body.msg.payload.bytes)) {
        ESP_LOGW(TAG,
                 "Global buffer TX payload too large for packet payload buffer; truncating len=%lu cap=%u",
                 (unsigned long)payload_len,
                 (unsigned)sizeof(msg.body.msg.payload.bytes));
        payload_len = sizeof(msg.body.msg.payload.bytes);
    }
    msg.body.msg.payload.size = payload_len;
    memcpy(msg.body.msg.payload.bytes, payload_chunk, payload_len);

    uint8_t encoded[ai_edgez_halow_NetworkPacket_size] = {0};
    uint16_t encoded_len = 0;
    if (!encode_network_packet(&msg, encoded, sizeof(encoded), &encoded_len)) {
        send_failed = true;
    } else {
        esp_err_t err = edgez_platform_get()->halow_send_mesh_payload_via(
            encoded,
            encoded_len,
            msg.from,
            msg.to,
            next_hop,
            msg.body.msg.message_id_high,
            msg.body.msg.message_id_low,
            max_hop,
            msg.body.msg.sequence);
        if (err == ESP_OK) {
            bool transfer_complete = false;
            bool send_next = false;
            portENTER_CRITICAL(&s_global_buffer_tx_lock);
            edgez_global_buffer_target_t *current =
                &s_global_buffer_targets[s_global_buffer_target_index];
            current->next_offset = next_offset + payload_data_len;
            if (current->next_offset >= current->end_offset) {
                current->active = false;
                if (s_global_buffer_targets_completed < UINT8_MAX) {
                    ++s_global_buffer_targets_completed;
                }
                ++s_global_buffer_target_index;
                transfer_complete =
                    s_global_buffer_target_index >= s_global_buffer_target_count;
            } else {
                send_next = true;
            }
            portEXIT_CRITICAL(&s_global_buffer_tx_lock);

            if (transfer_complete) {
                MESH_DEBUG_LOGI(
                    "Global buffer TX best-effort target complete target=0x%012llx msg=%016llx-%016llx",
                    (unsigned long long)target,
                    (unsigned long long)message_id_high,
                    (unsigned long long)message_id_low);
                global_buffer_tx_reset_state();
            } else if (send_next) {
                /* Pace the initial burst like realtime voice. Missing chunks
                 * are requested later through the reliable GBR2 control path. */
                vTaskDelay(pdMS_TO_TICKS(EDGEZ_GLOBAL_BUFFER_TX_SPACING_MS));
                global_buffer_tx_schedule_send();
            }
        } else {
            send_failed = true;
        }
    }

    if (send_failed) {
        ESP_LOGW(TAG,
                 "Global buffer TX send failed target=0x%012llx seq=%lu len=%u err_reset=1",
                 (unsigned long long)target,
                 (unsigned long)sequence,
                 (unsigned)payload_len);
        global_buffer_tx_reset_state();
        (void)global_buffer_send_status(target,
                                        EDGEZ_GLOBAL_BUFFER_STATUS_BUSY,
                                        EDGEZ_GLOBAL_BUFFER_BUSY_RETRY_MS,
                                        (uint32_t)edgez_platform_get()->sampling_buffer_serving_length());
    } else {
        MESH_DEBUG_LOGI(
                 "Global buffer TX best-effort chunk target=0x%012llx target_offset=%llu seq=%lu len=%u remaining=%u msg=%016llx-%016llx",
                 (unsigned long long)target,
                 (unsigned long long)next_offset,
                 (unsigned long)sequence,
                 (unsigned)payload_len,
                 (unsigned long)remaining_after_send,
                 (unsigned long long)message_id_high,
                 (unsigned long long)message_id_low);
    }
}

static esp_err_t global_buffer_send_status(uint64_t requester,
                                           uint8_t status,
                                           uint32_t retry_after_ms,
                                           uint32_t available_length)
{
    uint8_t source_mac[EDGEZ_ROUTE_MAC_LEN] = {0};
    if (!edgez_platform_get()->halow_get_self_mac(source_mac)) {
        return ESP_ERR_INVALID_STATE;
    }

    ai_edgez_halow_NetworkPacket response = ai_edgez_halow_NetworkPacket_init_zero;
    response.operation = ai_edgez_halow_Operation_RESPONSE;
    response.interface = ai_edgez_halow_Interface_HALOW;
    response.from = mac_to_u64(source_mac);
    response.to = requester & 0xffffffffffffULL;
    response.which_body = ai_edgez_halow_NetworkPacket_msg_tag;
    generate_network_packet_message_id(&response.body.msg.message_id_high,
                                       &response.body.msg.message_id_low);
    response.body.msg.sequence = 1;
    response.body.msg.mime = ai_edgez_halow_Mime_MIME_BINARY;
    response.body.msg.payload.size = EDGEZ_GLOBAL_BUFFER_RESPONSE_SIZE;
    uint8_t *payload = response.body.msg.payload.bytes;
    memcpy(payload, EDGEZ_GLOBAL_BUFFER_RESPONSE_MAGIC,
           sizeof(EDGEZ_GLOBAL_BUFFER_RESPONSE_MAGIC));
    payload[4] = status;
    payload[5] = 0;
    payload[6] = 0;
    payload[7] = 0;
    payload[8] = (uint8_t)(retry_after_ms >> 24);
    payload[9] = (uint8_t)(retry_after_ms >> 16);
    payload[10] = (uint8_t)(retry_after_ms >> 8);
    payload[11] = (uint8_t)retry_after_ms;
    payload[12] = (uint8_t)(available_length >> 24);
    payload[13] = (uint8_t)(available_length >> 16);
    payload[14] = (uint8_t)(available_length >> 8);
    payload[15] = (uint8_t)available_length;

    uint8_t encoded[ai_edgez_halow_NetworkPacket_size] = {0};
    uint16_t encoded_len = 0;
    if (!encode_network_packet(&response, encoded, sizeof(encoded), &encoded_len)) {
        return ESP_FAIL;
    }

    uint64_t next_hop = response.to;
    (void)batman_route_lookup(response.to, &next_hop);
    return edgez_platform_get()->halow_send_mesh_payload_via(
        encoded,
        encoded_len,
        response.from,
        response.to,
        next_hop,
        response.body.msg.message_id_high,
        response.body.msg.message_id_low,
        default_network_packet_max_hop(),
        response.body.msg.sequence);
}

static esp_err_t global_buffer_tx_start_for_requester(uint64_t requester,
                                                      uint64_t request_message_id_high,
                                                      uint64_t request_message_id_low,
                                                      uint32_t expected_length,
                                                      uint64_t requested_group_id,
                                                      int requested_chunk_index)
{
    requester &= 0xffffffffffffULL;
    if (mac_is_zero_u64(requester) || mac_is_broadcast_u64(requester)) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_global_buffer_tx_lock);
    bool already_active = s_global_buffer_tx_active;
    bool duplicate_active_request = already_active &&
        s_global_buffer_requester == requester &&
        s_global_buffer_request_message_id_high == request_message_id_high &&
        s_global_buffer_request_message_id_low == request_message_id_low;
    if (!already_active) {
        /* Reserve the single serving slot before pinning the published sample. */
        s_global_buffer_tx_active = true;
        s_global_buffer_requester = requester;
        s_global_buffer_request_message_id_high = request_message_id_high;
        s_global_buffer_request_message_id_low = request_message_id_low;
    }
    portEXIT_CRITICAL(&s_global_buffer_tx_lock);
    if (duplicate_active_request) {
        MESH_DEBUG_LOGI(
            "Global buffer duplicate pull request ignored requester=0x%012llx msg=%016llx-%016llx",
            (unsigned long long)requester,
            (unsigned long long)request_message_id_high,
            (unsigned long long)request_message_id_low);
        return ESP_OK;
    }
    if (already_active) {
        ESP_LOGW(TAG,
                 "Global buffer pull rejected requester=0x%012llx; transfer already active",
                 (unsigned long long)requester);
        return ESP_ERR_INVALID_STATE;
    }

    const uint8_t *data = NULL;
    size_t snapshot_length = 0;
    esp_err_t snapshot_err = edgez_platform_get()->sampling_buffer_acquire(&data,
                                                                         &snapshot_length);
    if (snapshot_err != ESP_OK || data == NULL || snapshot_length == 0 ||
        snapshot_length > UINT32_MAX) {
        global_buffer_tx_reset_state();
        if (snapshot_err == ESP_OK) {
            snapshot_err = ESP_ERR_NOT_FOUND;
        }
        ESP_LOGW(TAG,
                 "Global buffer snapshot rejected requester=0x%012llx: %s",
                 (unsigned long long)requester,
                 esp_err_to_name(snapshot_err));
        return snapshot_err;
    }
    if (expected_length != 0 && snapshot_length != expected_length) {
        ESP_LOGW(TAG,
                 "Global buffer snapshot changed requester=0x%012llx expected=%lu available=%lu",
                 (unsigned long long)requester,
                 (unsigned long)expected_length,
                 (unsigned long)snapshot_length);
        edgez_platform_get()->sampling_buffer_release();
        global_buffer_tx_reset_state();
        return ESP_ERR_NOT_FOUND;
    }

    uint32_t length = (uint32_t)snapshot_length;
    uint32_t start_offset = 0;
    uint32_t end_offset = length;
    uint32_t chunk_sequence = 0;
    if (requested_chunk_index >= 0) {
        uint64_t requested_offset =
            (uint64_t)(uint32_t)requested_chunk_index * EDGEZ_GLOBAL_BUFFER_CHUNK_SIZE;
        if (requested_group_id == 0 || requested_offset >= length) {
            edgez_platform_get()->sampling_buffer_release();
            global_buffer_tx_reset_state();
            return ESP_ERR_INVALID_ARG;
        }
        start_offset = (uint32_t)requested_offset;
        end_offset = start_offset + EDGEZ_GLOBAL_BUFFER_CHUNK_SIZE;
        if (end_offset > length) {
            end_offset = length;
        }
        chunk_sequence = (uint32_t)requested_chunk_index;
    }
    uint64_t next_hop = requester;
    (void)batman_route_lookup(requester, &next_hop);

    portENTER_CRITICAL(&s_global_buffer_tx_lock);
    memset(s_global_buffer_targets, 0, sizeof(s_global_buffer_targets));
    s_global_buffer_targets_completed = 0;
    s_global_buffer_data = data;
    s_global_buffer_length = length;
    s_global_buffer_targets[0].active = true;
    s_global_buffer_targets[0].target = requester;
    s_global_buffer_targets[0].next_hop = next_hop;
    s_global_buffer_targets[0].next_offset = start_offset;
    s_global_buffer_targets[0].end_offset = end_offset;
    s_global_buffer_targets[0].chunk_sequence = chunk_sequence;
    s_global_buffer_targets[0].chunk_group_id = requested_group_id;
    s_global_buffer_target_count = 1;
    s_global_buffer_tx_active = true;
    s_global_buffer_serving_acquired = true;
    s_global_buffer_target_index = 0;
    portEXIT_CRITICAL(&s_global_buffer_tx_lock);

    MESH_DEBUG_LOGI(
             "Global buffer pull start requester=0x%012llx next=0x%012llx len=%u chunk=%d group=%016llx",
             (unsigned long long)requester,
             (unsigned long long)next_hop,
             (unsigned)length,
             requested_chunk_index,
             (unsigned long long)requested_group_id);
    global_buffer_tx_schedule_send();
    return ESP_OK;
}

static bool beacon_identity_is_complete(const ai_edgez_halow_Beacon *beacon)
{
    if (!beacon) {
        return false;
    }

    return (beacon->user_id_high != 0 || beacon->user_id_low != 0) &&
           beacon->user_name[0] != '\0' &&
           beacon->user_public_key.size > 0 &&
           beacon->device_type >= ai_edgez_halow_DeviceType_DEVICE_TYPE_USER &&
           beacon->device_type <= ai_edgez_halow_DeviceType_DEVICE_TYPE_RELAY;
}

static bool decode_beacon_payload(const uint8_t *encoded,
                                  size_t encoded_len,
                                  ai_edgez_halow_Beacon *beacon)
{
    if (!encoded || encoded_len == 0 || !beacon) {
        return false;
    }

    uint8_t decoded[ai_edgez_halow_Beacon_size] = {0};
    size_t decoded_len = 0;
    const uint8_t *nodeinfo = encoded;
    size_t nodeinfo_len = encoded_len;
    if (mbedtls_base64_decode(decoded, sizeof(decoded), &decoded_len, encoded, encoded_len) == 0 && decoded_len > 0) {
        nodeinfo = decoded;
        nodeinfo_len = decoded_len;
    }

    memset(beacon, 0, sizeof(*beacon));
    pb_istream_t stream = pb_istream_from_buffer(nodeinfo, nodeinfo_len);
    if (!pb_decode(&stream, ai_edgez_halow_Beacon_fields, beacon)) {
        return false;
    }

    if (!beacon_identity_is_complete(beacon)) {
        ESP_LOGW(TAG,
                 "HaLow RX beacon skipped; incomplete identity user=%016llx-%016llx name=%u key_len=%u type=%u",
                 (unsigned long long)beacon->user_id_high,
                 (unsigned long long)beacon->user_id_low,
                 beacon->user_name[0] != '\0' ? 1U : 0U,
                 (unsigned)beacon->user_public_key.size,
                 (unsigned)beacon->device_type);
        return false;
    }

    MESH_DEBUG_LOGI(
             "HaLow RX beacon decoded user_id=%016llx-%016llx user_name=%s key_len=%u type=%u geo=%u geo_index=%lu sensor_data=%u",
             (unsigned long long)beacon->user_id_high,
             (unsigned long long)beacon->user_id_low,
             beacon->user_name,
             (unsigned)beacon->user_public_key.size,
             (unsigned)beacon->device_type,
             beacon->has_geo_fence ? 1 : 0,
             beacon->has_geo_fence ? (unsigned long)beacon->geo_fence.geo_index : 0UL,
             (unsigned)beacon->sensor_data_count);
    return true;
}

static bool decode_beacon_string(const char *encoded, ai_edgez_halow_Beacon *beacon)
{
    if (!encoded || encoded[0] == '\0') {
        return false;
    }
    return decode_beacon_payload((const uint8_t *)encoded, strnlen(encoded, EDGEZ_BEACON_TEXT_MAX_LEN + 1), beacon);
}

static bool encrypt_beacon_payload(const uint8_t *payload,
                                   size_t payload_len,
                                   const char *passphrase,
                                   uint8_t *out,
                                   size_t out_size,
                                   size_t *out_len)
{
    if (!payload || payload_len == 0 || !passphrase || passphrase[0] == '\0' ||
        !out || !out_len ||
        out_size < EDGEZ_BEACON_ENCRYPTED_MAGIC_SIZE + EDGEZ_BEACON_AES_GCM_NONCE_SIZE +
                       payload_len + EDGEZ_BEACON_AES_GCM_TAG_SIZE) {
        return false;
    }

    uint8_t key[32] = {0};
    mbedtls_sha256((const unsigned char *)passphrase, strnlen(passphrase, 64), key, 0);

    size_t off = 0;
    memcpy(&out[off], EDGEZ_BEACON_ENCRYPTED_MAGIC, EDGEZ_BEACON_ENCRYPTED_MAGIC_SIZE);
    off += EDGEZ_BEACON_ENCRYPTED_MAGIC_SIZE;
    esp_fill_random(&out[off], EDGEZ_BEACON_AES_GCM_NONCE_SIZE);
    const uint8_t *nonce = &out[off];
    off += EDGEZ_BEACON_AES_GCM_NONCE_SIZE;

    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);
    int rc = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, key, sizeof(key) * 8);
    if (rc == 0) {
        rc = mbedtls_gcm_crypt_and_tag(&gcm,
                                       MBEDTLS_GCM_ENCRYPT,
                                       payload_len,
                                       nonce,
                                       EDGEZ_BEACON_AES_GCM_NONCE_SIZE,
                                       NULL,
                                       0,
                                       payload,
                                       &out[off],
                                       EDGEZ_BEACON_AES_GCM_TAG_SIZE,
                                       &out[off + payload_len]);
    }
    mbedtls_gcm_free(&gcm);
    memset(key, 0, sizeof(key));
    if (rc != 0) {
        ESP_LOGW(TAG, "HaLow Beacon encrypt failed rc=%d", rc);
        return false;
    }

    *out_len = off + payload_len + EDGEZ_BEACON_AES_GCM_TAG_SIZE;
    return true;
}

static bool encode_beacon_string_with_passphrase(const ai_edgez_halow_Beacon *beacon,
                                                 const char *passphrase,
                                                 char *out,
                                                 size_t out_size)
{
    if (!beacon || !out || out_size == 0) {
        return false;
    }

    uint8_t proto[ai_edgez_halow_Beacon_size] = {0};
    pb_ostream_t stream = pb_ostream_from_buffer(proto, sizeof(proto));
    if (!pb_encode(&stream, ai_edgez_halow_Beacon_fields, beacon)) {
        ESP_LOGW(TAG, "HaLow Beacon encode failed: %s", PB_GET_ERROR(&stream));
        return false;
    }

    const uint8_t *payload = proto;
    size_t payload_len = stream.bytes_written;
    uint8_t encrypted[EDGEZ_BEACON_ENCRYPTED_MAGIC_SIZE +
                      EDGEZ_BEACON_AES_GCM_NONCE_SIZE +
                      ai_edgez_halow_Beacon_size +
                      EDGEZ_BEACON_AES_GCM_TAG_SIZE] = {0};
    size_t encrypted_len = 0;
    bool encrypted_payload = false;
    if (passphrase && passphrase[0] != '\0') {
        if (!encrypt_beacon_payload(proto, stream.bytes_written, passphrase,
                                    encrypted, sizeof(encrypted), &encrypted_len)) {
            out[0] = '\0';
            return false;
        }
        payload = encrypted;
        payload_len = encrypted_len;
        encrypted_payload = true;
    }

    size_t encoded_len = 0;
    int ret = mbedtls_base64_encode((uint8_t *)out, out_size, &encoded_len, payload, payload_len);
    if (ret != 0 || encoded_len >= out_size) {
        ESP_LOGW(TAG, "HaLow Beacon base64 encode failed payload_len=%u out_len=%u encrypted=%u",
                 (unsigned)payload_len,
                 (unsigned)out_size);
        out[0] = '\0';
        return false;
    }
    out[encoded_len] = '\0';
    ESP_LOGD(TAG,
             "HaLow Beacon encoded proto_len=%u payload_len=%u text_len=%u encrypted=%u",
             (unsigned)stream.bytes_written,
             (unsigned)payload_len,
             (unsigned)encoded_len,
             encrypted_payload ? 1 : 0);
    return true;
}

static bool encode_beacon_string(const ai_edgez_halow_Beacon *beacon, char *out, size_t out_size)
{
    return encode_beacon_string_with_passphrase(beacon, "", out, out_size);
}

void halow_sync_bridge_note_active_interface(halow_sync_active_interface_t active_interface)
{
    if (active_interface != HALOW_SYNC_ACTIVE_INTERFACE_NONE &&
        active_interface != HALOW_SYNC_ACTIVE_INTERFACE_BLE &&
        active_interface != HALOW_SYNC_ACTIVE_INTERFACE_USB) {
        active_interface = HALOW_SYNC_ACTIVE_INTERFACE_NONE;
    }
    if (active_interface == HALOW_SYNC_ACTIVE_INTERFACE_USB &&
        edgez_platform_get()->ble_is_connected()) {
        active_interface = HALOW_SYNC_ACTIVE_INTERFACE_BLE;
    }
    s_active_interface = active_interface;
}

static ai_edgez_halow_LicenseStatus halow_license_status(void)
{
    if (!factory_data_license_authorize(EDGEZ_LICENSE_CAP_MOBILE_CONTROL)) {
        return ai_edgez_halow_LicenseStatus_LICENSE_STATUS_DEVICE_NOT_LICENSED;
    }

    switch (factory_data_sdk_release_auth_state()) {
    case EDGEZ_SDK_RELEASE_AUTHORIZED:
        return ai_edgez_halow_LicenseStatus_LICENSE_STATUS_AUTHORIZED;
    case EDGEZ_SDK_RELEASE_AUTH_INCOMPATIBLE:
        return ai_edgez_halow_LicenseStatus_LICENSE_STATUS_SDK_VERSION_INCOMPATIBLE;
    case EDGEZ_SDK_RELEASE_AUTH_INVALID:
        return ai_edgez_halow_LicenseStatus_LICENSE_STATUS_SDK_RELEASE_INVALID;
    case EDGEZ_SDK_RELEASE_AUTH_REQUIRED:
    default:
        return ai_edgez_halow_LicenseStatus_LICENSE_STATUS_SDK_RELEASE_REQUIRED;
    }
}

void halow_sync_bridge_fill_status(ai_edgez_halow_HaLowInterfaceStatus *status)
{
    if (!status) {
        return;
    }

    edgez_platform_halow_status_t snapshot = {0};
    edgez_platform_get()->halow_get_status(&snapshot);

    status->supported = snapshot.supported;
    status->stack_initialized = snapshot.stack_initialized;
    status->mesh_mode = snapshot.mesh_mode;
    status->link_up = snapshot.link_up;
    status->route_ready = snapshot.route_ready;
    status->ready_for_report = snapshot.ready_for_report;
    status->ethertype = HALOW_SYNC_ETHERTYPE;
    status->license_status = halow_license_status();
    status->has_public_channel_mask = true;
    status->public_channel_mask = (uint32_t)atomic_load_explicit(
        &s_public_channel_mask, memory_order_acquire);
    strlcpy(status->firmware_version,
            esp_app_get_description()->version,
            sizeof(status->firmware_version));
    strlcpy(status->mesh_id, snapshot.mesh_id, sizeof(status->mesh_id));
    strlcpy(status->ip_addr, snapshot.ip_addr, sizeof(status->ip_addr));
    strlcpy(status->gateway, snapshot.gateway, sizeof(status->gateway));

    uint8_t mac[6] = {0};
#ifdef CONFIG_ENABLE_MM_HALOW
    if (mmwlan_get_mac_addr(mac) == MMWLAN_SUCCESS) {
        status->mac_address = mac_to_u64(mac);
        return;
    }
#endif
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK ||
        esp_read_mac(mac, ESP_MAC_BT) == ESP_OK ||
        esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK) {
        status->mac_address = mac_to_u64(mac);
    }
}

static void fill_response_base(ai_edgez_halow_NetworkPacket *response,
                               const ai_edgez_halow_NetworkPacket *request)
{
    response->operation = ai_edgez_halow_Operation_RESPONSE;
    response->interface = ai_edgez_halow_Interface_HALOW;
    if (request) {
        response->to = request->from;
        response->from = request->to;
    }
}

static esp_err_t send_routing_table_response(
    const ai_edgez_halow_NetworkPacket *request)
{
    if (!request || request->operation != ai_edgez_halow_Operation_REQUEST) {
        return ESP_ERR_INVALID_ARG;
    }

    ai_edgez_halow_NetworkPacket response =
        ai_edgez_halow_NetworkPacket_init_zero;
    fill_response_base(&response, request);
    const edgez_platform_api_t *platform = edgez_platform_get();
    uint8_t self_mac[EDGEZ_ROUTE_MAC_LEN] = {0};
    if (platform && platform->halow_get_self_mac &&
        platform->halow_get_self_mac(self_mac)) {
        response.from = mac_to_u64(self_mac);
    }
    response.which_body =
        ai_edgez_halow_NetworkPacket_routing_table_tag;

    edgez_platform_halow_route_t routes[
        EDGEZ_TOPOLOGY_ROUTE_SNAPSHOT_MAX] = {0};
    size_t count = platform && platform->halow_get_routes
                       ? platform->halow_get_routes(
                             routes, EDGEZ_TOPOLOGY_ROUTE_SNAPSHOT_MAX)
                       : 0;
    if (count > EDGEZ_TOPOLOGY_ROUTE_SNAPSHOT_MAX) {
        count = EDGEZ_TOPOLOGY_ROUTE_SNAPSHOT_MAX;
    }
    response.body.routing_table.routes_count = (pb_size_t)count;
    for (size_t i = 0; i < count; ++i) {
        ai_edgez_halow_RouteEntry *entry =
            &response.body.routing_table.routes[i];
        entry->destination = mac_to_u64(routes[i].originator);
        entry->next_hop = mac_to_u64(routes[i].next_hop);
        entry->tq = routes[i].tq;
        entry->hops = routes[i].hops;
        entry->age_ms = routes[i].age_ms;
    }
    if (!send_network_packet(&response)) {
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BATMAN routing table response routes=%u",
             (unsigned)count);
    return ESP_OK;
}

static uint32_t normalize_device_beacon_interval(uint32_t seconds)
{
    if (seconds < 5) {
        return 5;
    }
    if (seconds > 3600) {
        return 3600;
    }
    return seconds;
}

uint32_t halow_sync_bridge_beacon_interval_seconds(void)
{
    ai_edgez_halow_DeviceSettings settings = ai_edgez_halow_DeviceSettings_init_zero;
    device_settings_get_snapshot(&settings);
    return normalize_device_beacon_interval(settings.beacon_interval_seconds);
}

static uint32_t normalize_device_max_hop(uint32_t max_hop)
{
    return (max_hop > UINT8_MAX) ? UINT8_MAX : max_hop;
}

static uint32_t default_network_packet_max_hop(void)
{
    ai_edgez_halow_DeviceSettings settings = ai_edgez_halow_DeviceSettings_init_zero;
    device_settings_get_snapshot(&settings);
    return normalize_device_max_hop(settings.max_hop);
}

static bool sensor_selector_is_empty(const char *selector)
{
    return !selector ||
           selector[0] == '\0' ||
           strcmp(selector, "none") == 0 ||
           strcmp(selector, "unspecified") == 0;
}

#ifdef CONFIG_EDGEZ_SENSOR_SCRIPTING
static void device_settings_apply_sensor_config(const ai_edgez_halow_DeviceSettings *settings)
{
    if (!settings) {
        return;
    }

    const char *uart_i2c_selector = sensor_selector_is_empty(settings->uart_i2c_sensor_type)
                                        ? ""
                                        : settings->uart_i2c_sensor_type;
    const char *rs485_selector = sensor_selector_is_empty(settings->rs485_sensor_type)
                                     ? ""
                                     : settings->rs485_sensor_type;

    esp_err_t err = edgez_platform_get()->sampling_set_sensor_selectors(uart_i2c_selector, rs485_selector);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,
                 "Device sensor config apply failed uart_i2c=%s rs485=%s err=%s",
                 uart_i2c_selector,
                 rs485_selector,
                 esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG,
                 "Device sensor config applied uart_i2c=%s rs485=%s",
                 uart_i2c_selector[0] ? uart_i2c_selector : "none",
                 rs485_selector[0] ? rs485_selector : "none");
        edgez_platform_get()->sampling_refresh_script_cache();
    }
}

static void script_config_upload_reset(void)
{
    if (s_script_config_upload.buffer) {
        free(s_script_config_upload.buffer);
    }
    memset(&s_script_config_upload, 0, sizeof(s_script_config_upload));
}

static size_t script_config_safe_name(const ai_edgez_halow_ScriptConfig *config,
                                      char *out,
                                      size_t out_size)
{
    if (!config || !out || out_size == 0) {
        return 0;
    }

    const char *source = config->sensor_type[0] != '\0' ? config->sensor_type : config->name;
    size_t written = 0;
    for (size_t i = 0; source[i] != '\0' && written + 1 < out_size; i++) {
        unsigned char ch = (unsigned char)source[i];
        if ((ch >= 'A' && ch <= 'Z') ||
            (ch >= 'a' && ch <= 'z') ||
            (ch >= '0' && ch <= '9') ||
            ch == '_' ||
            ch == '-' ||
            ch == '.' ||
            ch == ':') {
            out[written++] = (char)ch;
        } else if (written > 0 && out[written - 1] != '_') {
            out[written++] = '_';
        }
    }

    while (written > 0 && out[written - 1] == '_') {
        written--;
    }

    if (written == 0) {
        written = (size_t)snprintf(out,
                                   out_size,
                                   "script_%lu",
                                   (unsigned long)config->script_id);
        if (written >= out_size) {
            written = out_size - 1;
        }
    }

    out[written] = '\0';
    return written;
}

static esp_err_t script_config_select_sensor_script(const ai_edgez_halow_ScriptConfig *config)
{
    if (!config || (!config->select_uart_i2c && !config->select_rs485)) {
        return ESP_OK;
    }

    char selector[33] = {0};
    if (config->sensor_type[0] != '\0') {
        strlcpy(selector, config->sensor_type, sizeof(selector));
    } else {
        (void)snprintf(selector, sizeof(selector), "%lu", (unsigned long)config->script_id);
    }

    char current_uart_i2c[32] = {0};
    char current_rs485[32] = {0};
    edgez_platform_get()->sampling_get_sensor_selectors(current_uart_i2c,
                                                        sizeof(current_uart_i2c),
                                                        current_rs485,
                                                        sizeof(current_rs485));
    const char *uart_i2c = config->select_uart_i2c ? selector : current_uart_i2c;
    const char *rs485 = config->select_rs485 ? selector : current_rs485;
    esp_err_t err = edgez_platform_get()->sampling_set_sensor_selectors(uart_i2c, rs485);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,
                 "Script sensor selection failed script_id=%lu selector=%s err=%s",
                 (unsigned long)config->script_id,
                 selector,
                 esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG,
                 "Script sensor selection applied script_id=%lu selector=%s",
                 (unsigned long)config->script_id,
                 selector);
    }
    return err;
}

static esp_err_t script_config_commit_buffer(const ai_edgez_halow_ScriptConfig *config,
                                             const uint8_t *script,
                                             size_t script_len)
{
    if (!config || !script || script_len == 0 || config->script_id == 0 || config->script_id > UINT16_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    char safe_name[64] = {0};
    size_t safe_name_len = script_config_safe_name(config, safe_name, sizeof(safe_name));

    edgez_platform_script_upsert_t upsert = {
        .script_id = (uint16_t)config->script_id,
        .name = (const uint8_t *)safe_name,
        .name_len = safe_name_len,
        .script = script,
        .script_len = script_len,
        .has_version = config->version > 0,
        .version = config->version,
        .has_global_buffer_size = config->global_buffer_size > 0,
        .global_buffer_size = config->global_buffer_size,
        .mime_type = (const uint8_t *)config->mime_type,
        .mime_type_len = strnlen(config->mime_type, sizeof(config->mime_type)),
    };

    esp_err_t err = edgez_platform_get()->script_store_upsert(&upsert);
    if (err == ESP_OK) {
        esp_err_t select_err = script_config_select_sensor_script(config);
        esp_err_t persist_err = device_settings_persist_script_sensor_selection(config);
        edgez_platform_get()->sampling_refresh_script_cache();
        if (select_err != ESP_OK) {
            err = select_err;
        } else if (persist_err != ESP_OK) {
            err = persist_err;
        }
    } else {
        ESP_LOGW(TAG,
                 "Script upsert failed script_id=%lu name=%s safe_name=%s sensor_type=%s script_len=%u err=%s",
                 (unsigned long)config->script_id,
                 config->name,
                 safe_name,
                 config->sensor_type,
                 (unsigned)script_len,
                 esp_err_to_name(err));
    }
    return err;
}

static esp_err_t handle_script_config(const ai_edgez_halow_NetworkPacket *msg,
                                      uint32_t request_id)
{
    const ai_edgez_halow_ScriptConfig *config = &msg->body.script_config;
    esp_err_t err = ESP_OK;

    if (config->script_id == 0 || config->script_id > UINT16_MAX) {
        err = ESP_ERR_INVALID_ARG;
        goto done;
    }

    switch (config->action) {
    case ai_edgez_halow_ScriptConfigAction_SCRIPT_CONFIG_BEGIN:
        if (config->total_size == 0) {
            err = ESP_ERR_INVALID_SIZE;
            break;
        }
        script_config_upload_reset();
        s_script_config_upload.buffer = (uint8_t *)malloc(config->total_size);
        if (!s_script_config_upload.buffer) {
            err = ESP_ERR_NO_MEM;
            break;
        }
        s_script_config_upload.active = true;
        s_script_config_upload.script_id = config->script_id;
        s_script_config_upload.total_size = config->total_size;
        s_script_config_upload.meta = *config;
        if (config->chunk.size > 0) {
            if (config->offset != 0 || config->chunk.size > config->total_size) {
                err = ESP_ERR_INVALID_ARG;
                script_config_upload_reset();
                break;
            }
            memcpy(s_script_config_upload.buffer, config->chunk.bytes, config->chunk.size);
            s_script_config_upload.received = config->chunk.size;
        }
        break;

    case ai_edgez_halow_ScriptConfigAction_SCRIPT_CONFIG_CHUNK:
        if (!s_script_config_upload.active ||
            s_script_config_upload.script_id != config->script_id ||
            config->offset != s_script_config_upload.received ||
            config->chunk.size == 0 ||
            config->offset + config->chunk.size > s_script_config_upload.total_size) {
            err = ESP_ERR_INVALID_ARG;
            break;
        }
        memcpy(s_script_config_upload.buffer + config->offset, config->chunk.bytes, config->chunk.size);
        s_script_config_upload.received += config->chunk.size;
        break;

    case ai_edgez_halow_ScriptConfigAction_SCRIPT_CONFIG_COMMIT:
        if (s_script_config_upload.active && s_script_config_upload.script_id == config->script_id) {
            if (config->chunk.size > 0) {
                if (config->offset != s_script_config_upload.received ||
                    config->offset + config->chunk.size > s_script_config_upload.total_size) {
                    err = ESP_ERR_INVALID_ARG;
                    break;
                }
                memcpy(s_script_config_upload.buffer + config->offset, config->chunk.bytes, config->chunk.size);
                s_script_config_upload.received += config->chunk.size;
            }
            if (s_script_config_upload.received != s_script_config_upload.total_size) {
                err = ESP_ERR_INVALID_SIZE;
                break;
            }
            s_script_config_upload.meta.action = config->action;
            if (config->name[0] != '\0') {
                strlcpy(s_script_config_upload.meta.name, config->name, sizeof(s_script_config_upload.meta.name));
            }
            err = script_config_commit_buffer(&s_script_config_upload.meta,
                                              s_script_config_upload.buffer,
                                              s_script_config_upload.received);
            script_config_upload_reset();
        } else if (config->chunk.size > 0) {
            err = script_config_commit_buffer(config, config->chunk.bytes, config->chunk.size);
        } else {
            err = ESP_ERR_INVALID_STATE;
        }
        break;

    case ai_edgez_halow_ScriptConfigAction_SCRIPT_CONFIG_DELETE:
        err = edgez_platform_get()->script_store_delete((uint16_t)config->script_id);
        if (err == ESP_OK) {
            edgez_platform_get()->sampling_refresh_script_cache();
        }
        break;

    default:
        err = ESP_ERR_INVALID_ARG;
        break;
    }

done:
    ESP_LOGI(TAG,
             "Script config action=%u script_id=%lu name=%s total=%lu offset=%lu chunk=%u select_u2c=%u select_rs485=%u err=%s",
             (unsigned)config->action,
             (unsigned long)config->script_id,
             config->name,
             (unsigned long)config->total_size,
             (unsigned long)config->offset,
             (unsigned)config->chunk.size,
             config->select_uart_i2c ? 1 : 0,
             config->select_rs485 ? 1 : 0,
             esp_err_to_name(err));

    ai_edgez_halow_NetworkPacket response = ai_edgez_halow_NetworkPacket_init_zero;
    fill_response_base(&response, msg);
    (void)request_id;
    response.which_body = ai_edgez_halow_NetworkPacket_script_config_tag;
    response.body.script_config = *config;
    response.body.script_config.action = ai_edgez_halow_ScriptConfigAction_SCRIPT_CONFIG_REPORT;
    response.body.script_config.chunk.size = 0;
    send_network_packet(&response);
    return err;
}
#else
static void device_settings_apply_sensor_config(const ai_edgez_halow_DeviceSettings *settings)
{
    (void)settings;
}

static esp_err_t handle_script_config(const ai_edgez_halow_NetworkPacket *msg,
                                      uint32_t request_id)
{
    const ai_edgez_halow_ScriptConfig *config = &msg->body.script_config;
    ESP_LOGW(TAG,
             "Script config unavailable in lean communicator build action=%u script_id=%lu",
             (unsigned)config->action,
             (unsigned long)config->script_id);

    ai_edgez_halow_NetworkPacket response = ai_edgez_halow_NetworkPacket_init_zero;
    fill_response_base(&response, msg);
    (void)request_id;
    response.which_body = ai_edgez_halow_NetworkPacket_script_config_tag;
    response.body.script_config = *config;
    response.body.script_config.action = ai_edgez_halow_ScriptConfigAction_SCRIPT_CONFIG_REPORT;
    response.body.script_config.chunk.size = 0;
    send_network_packet(&response);
    return ESP_ERR_NOT_SUPPORTED;
}
#endif

static void device_settings_apply_defaults(ai_edgez_halow_DeviceSettings *settings)
{
    if (!settings) {
        return;
    }
    if (settings->beacon_interval_seconds == 0) {
        settings->beacon_interval_seconds = 30;
    }
    settings->beacon_interval_seconds = normalize_device_beacon_interval(settings->beacon_interval_seconds);
    if (settings->max_hop == 0) {
        settings->max_hop = 2;
    }
    settings->max_hop = normalize_device_max_hop(settings->max_hop);
    if (settings->user_name[0] == '\0') {
        strlcpy(settings->user_name, "EdgeZ User", sizeof(settings->user_name));
    }
    settings->device_type = device_type_normalize(settings->device_type,
                                                   settings->device_mode_enabled);
}

static void device_settings_get_snapshot(ai_edgez_halow_DeviceSettings *out)
{
    if (!out) {
        return;
    }
    portENTER_CRITICAL(&s_device_settings_lock);
    *out = s_device_settings;
    portEXIT_CRITICAL(&s_device_settings_lock);
    out->action = ai_edgez_halow_DeviceSettingsAction_DEVICE_SETTINGS_REPORT;
    device_settings_apply_defaults(out);
}

static void device_settings_apply_snapshot(const ai_edgez_halow_DeviceSettings *settings)
{
    if (!settings) {
        return;
    }
    portENTER_CRITICAL(&s_device_settings_lock);
    s_device_settings = *settings;
    s_device_settings.action = ai_edgez_halow_DeviceSettingsAction_DEVICE_SETTINGS_REPORT;
    device_settings_apply_defaults(&s_device_settings);
    portEXIT_CRITICAL(&s_device_settings_lock);
}

static void device_gps_apply_if_halow_ready(
    const ai_edgez_halow_DeviceSettings *settings)
{
    if (!settings) return;
    bool enable = settings->device_gps_enabled && settings->share_location;
    if (enable && !edgez_platform_get()->halow_ready()) {
        ESP_LOGI(TAG, "L76K initialization deferred until HaLow is ready");
        return;
    }
    esp_err_t err = edgez_platform_get()->gps_configure(
        enable, settings->beacon_interval_seconds);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Optional L76K configuration skipped: %s",
                 esp_err_to_name(err));
    }
}

static void device_settings_apply_identity(const ai_edgez_halow_DeviceSettings *settings)
{
    if (!settings) {
        return;
    }

    /* DEVICE_SETTINGS_SET is also the live discovery-beacon profile update
     * protocol. Apply marker/location independently of identity so the mobile
     * app can change map presentation without resending keys or restarting
     * HaLow. device_beacon_task reads the updated settings snapshot on its
     * next interval and emits the complete Beacon protobuf. */
    bool use_static_location = settings->share_location &&
                               !settings->device_gps_enabled;
    (void)edgez_platform_get()->network_set_halow_beacon_profile((uint32_t)settings->marker,
                                             use_static_location,
                                             settings->latitude,
                                             settings->longitude);
    ai_edgez_halow_DeviceType beacon_device_type = settings->device_type;
    (void)edgez_platform_get()->network_set_halow_beacon_device_type((uint32_t)beacon_device_type);
    ESP_LOGI(TAG,
             "HaLow beacon profile updated marker=%u share_location=%u device_gps=%u lat=%f lon=%f type=%u",
             (unsigned)settings->marker,
             settings->share_location ? 1 : 0,
             settings->device_gps_enabled ? 1 : 0,
             (double)settings->latitude,
             (double)settings->longitude,
             (unsigned)beacon_device_type);

    if (settings->user_id_high != 0 ||
        settings->user_id_low != 0 ||
        settings->user_name[0] != '\0' ||
        settings->user_public_key.size != 0) {
        (void)edgez_platform_get()->network_set_halow_user_identity(settings->user_id_high,
                                                settings->user_id_low,
                                                settings->user_name,
                                                settings->user_public_key.bytes,
                                                settings->user_public_key.size);
        (void)edgez_platform_get()->halow_set_user_identity(settings->user_id_high,
                                                    settings->user_id_low,
                                                    settings->user_name,
                                                    settings->user_public_key.bytes,
                                                    settings->user_public_key.size);
    }

    esp_err_t refresh_err = edgez_platform_get()->network_refresh_halow_mesh_vendor_ie(settings->mesh_id,
                                                                   settings->passphrase);
    if (refresh_err != ESP_OK) {
        ESP_LOGW(TAG, "HaLow management beacon Vendor IE refresh deferred/failed: %s",
                 esp_err_to_name(refresh_err));
    }
}

static void device_mode_start_halow_from_settings(void)
{
    ai_edgez_halow_DeviceSettings settings = ai_edgez_halow_DeviceSettings_init_zero;
    device_settings_get_snapshot(&settings);
    if (!device_type_is_autonomous(settings.device_type)) {
        return;
    }
    if (settings.mesh_id[0] == '\0') {
        ESP_LOGW(TAG, "Device mode HaLow auto-start skipped; mesh_id is empty");
        return;
    }

    if (settings.mesh_frequency_khz != 0 && settings.mesh_bandwidth_mhz != 0) {
        esp_err_t radio_err = edgez_platform_get()->network_set_halow_mesh_radio(
            settings.mesh_frequency_khz,
            settings.mesh_bandwidth_mhz);
        if (radio_err != ESP_OK) {
            ESP_LOGW(TAG,
                     "Device mode HaLow radio restore failed freq=%lu kHz bw=%luMHz: %s",
                     (unsigned long)settings.mesh_frequency_khz,
                     (unsigned long)settings.mesh_bandwidth_mhz,
                     esp_err_to_name(radio_err));
            return;
        }
    }

    device_settings_apply_identity(&settings);
    (void)edgez_platform_get()->network_set_halow_max_hop(settings.max_hop);
    if (settings.device_type != ai_edgez_halow_DeviceType_DEVICE_TYPE_BEACON &&
        settings.upstream_wifi_ssid[0] != '\0') {
        esp_err_t wifi_err = edgez_platform_get()->network_connect_upstream_wifi(settings.upstream_wifi_ssid,
                                                             settings.upstream_wifi_passphrase);
        if (wifi_err != ESP_OK) {
            ESP_LOGW(TAG,
                     "Device mode upstream Wi-Fi connect failed ssid=%s passphrase=%s err=%s",
                     settings.upstream_wifi_ssid,
                     settings.upstream_wifi_passphrase[0] ? "set" : "open",
                     esp_err_to_name(wifi_err));
        }
    }
    bool beacon_only =
        settings.device_type == ai_edgez_halow_DeviceType_DEVICE_TYPE_BEACON;
    ESP_LOGI(TAG,
             "Autonomous HaLow start type=%u mesh_id=%s passphrase=%s max_hop=%lu upstream_wifi=%s beacon_only=%u beacon_unicast=0x%012llx",
             (unsigned)settings.device_type,
             settings.mesh_id,
             settings.passphrase[0] ? "set" : "open",
             (unsigned long)settings.max_hop,
             settings.upstream_wifi_ssid[0] ? settings.upstream_wifi_ssid : "none",
             beacon_only ? 1U : 0U,
             (unsigned long long)(settings.beacon_unicast & 0xffffffffffffULL));
    esp_err_t err = beacon_only
                        ? edgez_platform_get()->network_start_halow_beacon_only(settings.mesh_id,
                                                           settings.passphrase)
                        : edgez_platform_get()->network_connect_halow(settings.mesh_id,
                                                 settings.passphrase);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Device mode HaLow auto-start failed: %s", esp_err_to_name(err));
    }
}

static esp_err_t device_settings_load_from_nvs(void)
{
    ai_edgez_halow_DeviceSettings loaded = ai_edgez_halow_DeviceSettings_init_zero;
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(DEVICE_SETTINGS_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        device_settings_apply_defaults(&loaded);
        device_settings_apply_snapshot(&loaded);
        (void)edgez_platform_get()->network_set_halow_max_hop(loaded.max_hop);
        (void)edgez_platform_get()->network_set_halow_beacon_device_type((uint32_t)loaded.device_type);
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Device settings NVS open failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t mode = 0;
    uint8_t device_type = ai_edgez_halow_DeviceType_DEVICE_TYPE_UNSPECIFIED;
    uint8_t share_location = 0;
    uint8_t has_geo_fence = 0;
    uint8_t sleep_mode_enabled = 0;
    uint8_t device_gps_enabled = 0;
    uint32_t marker = 0;
    (void)nvs_get_u8(nvs, DEVICE_SETTINGS_NVS_MODE, &mode);
    (void)nvs_get_u8(nvs, DEVICE_SETTINGS_NVS_TYPE, &device_type);
    (void)nvs_get_u8(nvs, DEVICE_SETTINGS_NVS_SHARE_LOC, &share_location);
    (void)nvs_get_u8(nvs, DEVICE_SETTINGS_NVS_HAS_GEO_FENCE, &has_geo_fence);
    (void)nvs_get_u8(nvs, DEVICE_SETTINGS_NVS_SLEEP_ENABLED, &sleep_mode_enabled);
    (void)nvs_get_u8(nvs, DEVICE_SETTINGS_NVS_DEVICE_GPS, &device_gps_enabled);
    (void)nvs_get_u32(nvs, DEVICE_SETTINGS_NVS_MARKER, &marker);
    (void)nvs_get_u32(nvs, DEVICE_SETTINGS_NVS_INTERVAL, &loaded.beacon_interval_seconds);
    (void)nvs_get_u32(nvs, DEVICE_SETTINGS_NVS_MAX_HOP, &loaded.max_hop);
    (void)nvs_get_u32(nvs, DEVICE_SETTINGS_NVS_GEO_INDEX, &loaded.geo_index);
    (void)nvs_get_u32(nvs, DEVICE_SETTINGS_NVS_MESH_FREQUENCY, &loaded.mesh_frequency_khz);
    (void)nvs_get_u32(nvs, DEVICE_SETTINGS_NVS_MESH_BANDWIDTH, &loaded.mesh_bandwidth_mhz);
    (void)nvs_get_u64(nvs, DEVICE_SETTINGS_NVS_USER_HIGH, &loaded.user_id_high);
    (void)nvs_get_u64(nvs, DEVICE_SETTINGS_NVS_USER_LOW, &loaded.user_id_low);
    size_t len = sizeof(loaded.mesh_id);
    (void)nvs_get_str(nvs, DEVICE_SETTINGS_NVS_MESH_ID, loaded.mesh_id, &len);
    len = sizeof(loaded.passphrase);
    (void)nvs_get_str(nvs, DEVICE_SETTINGS_NVS_PASSPHRASE, loaded.passphrase, &len);
    len = sizeof(loaded.upstream_wifi_ssid);
    (void)nvs_get_str(nvs, DEVICE_SETTINGS_NVS_UPSTREAM_WIFI_SSID, loaded.upstream_wifi_ssid, &len);
    len = sizeof(loaded.upstream_wifi_passphrase);
    (void)nvs_get_str(nvs, DEVICE_SETTINGS_NVS_UPSTREAM_WIFI_PASSPHRASE, loaded.upstream_wifi_passphrase, &len);
    (void)nvs_get_u64(nvs, DEVICE_SETTINGS_NVS_BEACON_UNICAST, &loaded.beacon_unicast);
    len = sizeof(loaded.user_name);
    (void)nvs_get_str(nvs, DEVICE_SETTINGS_NVS_USER_NAME, loaded.user_name, &len);
    len = sizeof(loaded.uart_i2c_sensor_type);
    (void)nvs_get_str(nvs, DEVICE_SETTINGS_NVS_UART_I2C_SENSOR_TYPE, loaded.uart_i2c_sensor_type, &len);
    len = sizeof(loaded.rs485_sensor_type);
    (void)nvs_get_str(nvs, DEVICE_SETTINGS_NVS_RS485_SENSOR_TYPE, loaded.rs485_sensor_type, &len);
    len = sizeof(loaded.user_public_key.bytes);
    if (nvs_get_blob(nvs, DEVICE_SETTINGS_NVS_PUBLIC_KEY, loaded.user_public_key.bytes, &len) == ESP_OK) {
        loaded.user_public_key.size = len;
    }
    len = sizeof(loaded.user_private_key.bytes);
    if (nvs_get_blob(nvs, DEVICE_SETTINGS_NVS_PRIVATE_KEY, loaded.user_private_key.bytes, &len) == ESP_OK) {
        loaded.user_private_key.size = len;
    }
    len = sizeof(loaded.latitude);
    (void)nvs_get_blob(nvs, DEVICE_SETTINGS_NVS_LATITUDE, &loaded.latitude, &len);
    len = sizeof(loaded.longitude);
    (void)nvs_get_blob(nvs, DEVICE_SETTINGS_NVS_LONGITUDE, &loaded.longitude, &len);
    len = sizeof(loaded.geo_fence);
    if (nvs_get_blob(nvs, DEVICE_SETTINGS_NVS_GEO_FENCE, &loaded.geo_fence, &len) == ESP_OK &&
        len == sizeof(loaded.geo_fence)) {
        loaded.has_geo_fence = has_geo_fence != 0;
    }
    if (loaded.has_geo_fence) {
        loaded.geo_fence.geo_index = loaded.geo_index;
    }
    nvs_close(nvs);

    loaded.device_type = device_type_normalize((ai_edgez_halow_DeviceType)device_type, mode != 0);
    loaded.sleep_mode_enabled = sleep_mode_enabled != 0;
    loaded.device_gps_enabled = device_gps_enabled != 0 &&
                                edgez_platform_get()->gps_supported();
    loaded.share_location = share_location != 0;
    loaded.marker = (ai_edgez_halow_MarkerColor)marker;
    loaded.action = ai_edgez_halow_DeviceSettingsAction_DEVICE_SETTINGS_REPORT;
    device_settings_apply_defaults(&loaded);
    device_settings_apply_snapshot(&loaded);
    (void)edgez_platform_get()->network_set_halow_max_hop(loaded.max_hop);
    (void)edgez_platform_get()->network_set_halow_beacon_device_type((uint32_t)loaded.device_type);
    device_settings_apply_sensor_config(&loaded);
    ESP_LOGI(TAG,
             "Device settings loaded type=%u sleep_enabled=%u device_gps=%u mesh_id=%s passphrase=%s upstream_wifi=%s upstream_passphrase=%s beacon_unicast=0x%012llx user=%s marker=%u interval=%lu max_hop=%lu radio=%lu kHz/%luMHz share_location=%u lat=%f lon=%f geo=%u geo_index=%lu uart_i2c_sensor=%s rs485_sensor=%s pub=%u priv=%u",
             (unsigned)loaded.device_type,
             loaded.sleep_mode_enabled ? 1U : 0U,
             loaded.device_gps_enabled ? 1U : 0U,
             loaded.mesh_id,
             loaded.passphrase[0] ? "set" : "open",
             loaded.upstream_wifi_ssid[0] ? loaded.upstream_wifi_ssid : "none",
             loaded.upstream_wifi_passphrase[0] ? "set" : "open",
             (unsigned long long)(loaded.beacon_unicast & 0xffffffffffffULL),
             loaded.user_name,
             (unsigned)loaded.marker,
             (unsigned long)loaded.beacon_interval_seconds,
             (unsigned long)loaded.max_hop,
             (unsigned long)loaded.mesh_frequency_khz,
             (unsigned long)loaded.mesh_bandwidth_mhz,
             loaded.share_location ? 1 : 0,
             (double)loaded.latitude,
             (double)loaded.longitude,
             loaded.has_geo_fence ? 1 : 0,
             (unsigned long)loaded.geo_index,
             loaded.uart_i2c_sensor_type,
             loaded.rs485_sensor_type,
             (unsigned)loaded.user_public_key.size,
             (unsigned)loaded.user_private_key.size);
    return ESP_OK;
}

static esp_err_t device_settings_save_to_nvs(const ai_edgez_halow_DeviceSettings *settings)
{
    if (!settings) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(DEVICE_SETTINGS_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Device settings NVS open for write failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_u8(nvs, DEVICE_SETTINGS_NVS_TYPE, (uint8_t)settings->device_type);
    if (err == ESP_OK) err = nvs_set_u8(nvs, DEVICE_SETTINGS_NVS_SLEEP_ENABLED, settings->sleep_mode_enabled ? 1 : 0);
    if (err == ESP_OK) err = nvs_set_u8(nvs, DEVICE_SETTINGS_NVS_DEVICE_GPS, settings->device_gps_enabled ? 1 : 0);
    if (err == ESP_OK) err = nvs_set_str(nvs, DEVICE_SETTINGS_NVS_MESH_ID, settings->mesh_id);
    if (err == ESP_OK) err = nvs_set_str(nvs, DEVICE_SETTINGS_NVS_PASSPHRASE, settings->passphrase);
    if (err == ESP_OK) err = nvs_set_str(nvs, DEVICE_SETTINGS_NVS_UPSTREAM_WIFI_SSID, settings->upstream_wifi_ssid);
    if (err == ESP_OK) err = nvs_set_str(nvs, DEVICE_SETTINGS_NVS_UPSTREAM_WIFI_PASSPHRASE, settings->upstream_wifi_passphrase);
    if (err == ESP_OK) err = nvs_set_u64(nvs, DEVICE_SETTINGS_NVS_BEACON_UNICAST, settings->beacon_unicast & 0xffffffffffffULL);
    if (err == ESP_OK) err = nvs_set_u8(nvs, DEVICE_SETTINGS_NVS_SHARE_LOC, settings->share_location ? 1 : 0);
    if (err == ESP_OK) err = nvs_set_u8(nvs, DEVICE_SETTINGS_NVS_HAS_GEO_FENCE, settings->has_geo_fence ? 1 : 0);
    if (err == ESP_OK) err = nvs_set_str(nvs, DEVICE_SETTINGS_NVS_USER_NAME, settings->user_name);
    if (err == ESP_OK) err = nvs_set_u32(nvs, DEVICE_SETTINGS_NVS_MARKER, (uint32_t)settings->marker);
    if (err == ESP_OK) err = nvs_set_u32(nvs, DEVICE_SETTINGS_NVS_INTERVAL, normalize_device_beacon_interval(settings->beacon_interval_seconds));
    if (err == ESP_OK) err = nvs_set_u32(nvs, DEVICE_SETTINGS_NVS_MAX_HOP, normalize_device_max_hop(settings->max_hop));
    if (err == ESP_OK) err = nvs_set_u32(nvs, DEVICE_SETTINGS_NVS_MESH_FREQUENCY, settings->mesh_frequency_khz);
    if (err == ESP_OK) err = nvs_set_u32(nvs, DEVICE_SETTINGS_NVS_MESH_BANDWIDTH, settings->mesh_bandwidth_mhz);
    if (err == ESP_OK) err = nvs_set_u32(nvs, DEVICE_SETTINGS_NVS_GEO_INDEX, settings->geo_index);
    if (err == ESP_OK) err = nvs_set_str(nvs, DEVICE_SETTINGS_NVS_UART_I2C_SENSOR_TYPE, settings->uart_i2c_sensor_type);
    if (err == ESP_OK) err = nvs_set_str(nvs, DEVICE_SETTINGS_NVS_RS485_SENSOR_TYPE, settings->rs485_sensor_type);
    if (err == ESP_OK) err = nvs_set_u64(nvs, DEVICE_SETTINGS_NVS_USER_HIGH, settings->user_id_high);
    if (err == ESP_OK) err = nvs_set_u64(nvs, DEVICE_SETTINGS_NVS_USER_LOW, settings->user_id_low);
    if (err == ESP_OK) err = nvs_set_blob(nvs, DEVICE_SETTINGS_NVS_PUBLIC_KEY, settings->user_public_key.bytes, settings->user_public_key.size);
    if (err == ESP_OK) err = nvs_set_blob(nvs, DEVICE_SETTINGS_NVS_PRIVATE_KEY, settings->user_private_key.bytes, settings->user_private_key.size);
    if (err == ESP_OK) err = nvs_set_blob(nvs, DEVICE_SETTINGS_NVS_LATITUDE, &settings->latitude, sizeof(settings->latitude));
    if (err == ESP_OK) err = nvs_set_blob(nvs, DEVICE_SETTINGS_NVS_LONGITUDE, &settings->longitude, sizeof(settings->longitude));
    if (err == ESP_OK) err = nvs_set_blob(nvs, DEVICE_SETTINGS_NVS_GEO_FENCE, &settings->geo_fence, sizeof(settings->geo_fence));
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Device settings NVS save failed: %s", esp_err_to_name(err));
    }
    return err;
}

static esp_err_t device_settings_persist_script_sensor_selection(const ai_edgez_halow_ScriptConfig *config)
{
    if (!config || sensor_selector_is_empty(config->sensor_type) ||
        (!config->select_uart_i2c && !config->select_rs485)) {
        return ESP_OK;
    }

    ai_edgez_halow_DeviceSettings updated = ai_edgez_halow_DeviceSettings_init_zero;
    device_settings_get_snapshot(&updated);
    if (config->select_uart_i2c) {
        strlcpy(updated.uart_i2c_sensor_type, config->sensor_type, sizeof(updated.uart_i2c_sensor_type));
    }
    if (config->select_rs485) {
        strlcpy(updated.rs485_sensor_type, config->sensor_type, sizeof(updated.rs485_sensor_type));
    }
    if (updated.has_geo_fence) {
        updated.geo_fence.geo_index = updated.geo_index;
    }
    device_settings_apply_snapshot(&updated);

    esp_err_t err = device_settings_save_to_nvs(&updated);
    if (err == ESP_OK) {
        ESP_LOGI(TAG,
                 "Device sensor setting persisted from script upload uart_i2c=%s rs485=%s",
                 updated.uart_i2c_sensor_type,
                 updated.rs485_sensor_type);
    } else {
        ESP_LOGW(TAG,
                 "Device sensor setting persist failed uart_i2c=%s rs485=%s err=%s",
                 updated.uart_i2c_sensor_type,
                 updated.rs485_sensor_type,
                 esp_err_to_name(err));
    }
    return err;
}

static void send_device_settings_frame(uint16_t seq, const ai_edgez_halow_NetworkPacket *request)
{
    ai_edgez_halow_NetworkPacket msg = ai_edgez_halow_NetworkPacket_init_zero;
    fill_response_base(&msg, request);
    (void)seq;
    msg.which_body = ai_edgez_halow_NetworkPacket_device_settings_tag;
    device_settings_get_snapshot(&msg.body.device_settings);
    send_network_packet(&msg);
}

static void send_status_frame(uint16_t seq, const ai_edgez_halow_NetworkPacket *request)
{
    ai_edgez_halow_NetworkPacket msg = ai_edgez_halow_NetworkPacket_init_zero;
    fill_response_base(&msg, request);
    (void)seq;
    msg.which_body = ai_edgez_halow_NetworkPacket_status_tag;
    halow_sync_bridge_fill_status(&msg.body.status);
    send_network_packet(&msg);
}

void halow_sync_bridge_send_status_frame(uint16_t seq)
{
    send_status_frame(seq, NULL);
}

static void send_periodic_status_frame(void)
{
    if (!bridge_has_connected_interface()) {
        return;
    }

    if (s_ble_profile_reports_remaining > 0) {
        --s_ble_profile_reports_remaining;
        /* Android must learn the connected device mode before it decides
         * whether a following uninitialized status should trigger INIT_HALOW. */
        send_device_settings_frame(++s_from_radio_seq, NULL);
    }
    send_status_frame(++s_from_radio_seq, NULL);
}

static void status_report_task(void *arg)
{
    (void)arg;

    while (true) {
        (void)ulTaskNotifyTake(pdTRUE,
                               pdMS_TO_TICKS(CONFIG_MM_HALOW_STATUS_REPORT_INTERVAL_MS));
        send_periodic_status_frame();
    }
}

void halow_sync_bridge_request_status_report(void)
{
    if (s_status_report_task != NULL) {
        xTaskNotifyGive(s_status_report_task);
    }
}

void halow_sync_bridge_note_ble_connected(bool connected)
{
    edgez_platform_get()->led_note_ble_connected(connected);
    halow_sync_bridge_set_log_stream_enabled(false);
    if (connected) {
        s_active_interface = HALOW_SYNC_ACTIVE_INTERFACE_BLE;
        /* Repeat across several status periods so a CCC subscription racing
         * pairing completion cannot lose the only device-mode report. */
        s_ble_profile_reports_remaining = 3;
        device_mode_ble_shutdown_cancel();
        mode_deep_sleep_cancel();
        portENTER_CRITICAL(&s_ble_shutdown_lock);
        s_user_halow_shutdown_pending = false;
        portEXIT_CRITICAL(&s_ble_shutdown_lock);
        if (s_device_beacon_task != NULL) {
            xTaskNotifyGive(s_device_beacon_task);
        }
        halow_sync_bridge_request_status_report();
        return;
    }
    if (s_active_interface == HALOW_SYNC_ACTIVE_INTERFACE_BLE) {
        s_active_interface = HALOW_SYNC_ACTIVE_INTERFACE_NONE;
    }
    s_ble_profile_reports_remaining = 0;
    if (s_device_beacon_task != NULL) {
        xTaskNotifyGive(s_device_beacon_task);
    }
    if (!bridge_has_connected_interface()) {
        ble_disconnect_power_policy_schedule();
    }
}

void halow_sync_bridge_note_usb_connected(bool connected)
{
    if (connected) {
        if (!edgez_platform_get()->ble_is_connected()) {
            halow_sync_bridge_set_log_stream_enabled(false);
            s_active_interface = HALOW_SYNC_ACTIVE_INTERFACE_USB;
        }
        device_mode_ble_shutdown_cancel();
        mode_deep_sleep_cancel();
        portENTER_CRITICAL(&s_ble_shutdown_lock);
        s_user_halow_shutdown_pending = false;
        portEXIT_CRITICAL(&s_ble_shutdown_lock);
        if (s_device_beacon_task != NULL) {
            xTaskNotifyGive(s_device_beacon_task);
        }
        return;
    }

    if (s_active_interface == HALOW_SYNC_ACTIVE_INTERFACE_USB) {
        halow_sync_bridge_set_log_stream_enabled(false);
        s_active_interface = HALOW_SYNC_ACTIVE_INTERFACE_NONE;
    }
    if (s_device_beacon_task != NULL) {
        xTaskNotifyGive(s_device_beacon_task);
    }
    if (!bridge_has_connected_interface()) {
        ble_disconnect_power_policy_schedule();
    }
}

static bool device_settings_build_beacon(ai_edgez_halow_Beacon *beacon,
                                         uint32_t *interval_seconds,
                                         uint32_t *max_hop,
                                         char *passphrase,
                                         size_t passphrase_size,
                                         uint64_t *beacon_unicast,
                                         bool allow_relay_mode)
{
    (void)allow_relay_mode;
    if (!beacon || !interval_seconds || !max_hop) {
        return false;
    }

    ai_edgez_halow_DeviceSettings settings = ai_edgez_halow_DeviceSettings_init_zero;
    device_settings_get_snapshot(&settings);
    *interval_seconds = normalize_device_beacon_interval(settings.beacon_interval_seconds);
    *max_hop = normalize_device_max_hop(settings.max_hop);
    if (passphrase && passphrase_size > 0) {
        strlcpy(passphrase, settings.passphrase, passphrase_size);
    }
    if (beacon_unicast) {
        *beacon_unicast = settings.beacon_unicast & 0xffffffffffffULL;
    }
    if (settings.user_id_high == 0 && settings.user_id_low == 0 && settings.user_name[0] == '\0') {
        return false;
    }

    memset(beacon, 0, sizeof(*beacon));
    beacon->user_id_high = settings.user_id_high;
    beacon->user_id_low = settings.user_id_low;
    strlcpy(beacon->user_name, settings.user_name, sizeof(beacon->user_name));
    beacon->user_public_key.size = settings.user_public_key.size;
    memcpy(beacon->user_public_key.bytes, settings.user_public_key.bytes, settings.user_public_key.size);
    beacon->marker = settings.marker;
    beacon->device_type = settings.device_type;
    if (settings.has_geo_fence) {
        beacon->has_geo_fence = true;
        beacon->geo_fence = settings.geo_fence;
        beacon->geo_fence.geo_index = settings.geo_index;
    }

#define APPEND_SENSOR_FLOAT(sensor_type_value, sensor_value) do { \
        if (beacon->sensor_data_count < 7) { \
            ai_edgez_halow_SensorData *entry = \
                &beacon->sensor_data[beacon->sensor_data_count++]; \
            entry->type = (sensor_type_value); \
            entry->which_value = ai_edgez_halow_SensorData_float_value_tag; \
            entry->value.float_value = (sensor_value); \
        } \
    } while (0)
#define APPEND_SENSOR_INT(sensor_type_value, sensor_value) do { \
        if (beacon->sensor_data_count < 7) { \
            ai_edgez_halow_SensorData *entry = \
                &beacon->sensor_data[beacon->sensor_data_count++]; \
            entry->type = (sensor_type_value); \
            entry->which_value = ai_edgez_halow_SensorData_int_value_tag; \
            entry->value.int_value = (sensor_value); \
        } \
    } while (0)

    edgez_platform_sensor_data_t sensor_data = {0};
    bool have_sensor_data = edgez_platform_get()->sampling_get_latest_sensor_data(&sensor_data);
    float device_gps_latitude = 0.0f;
    float device_gps_longitude = 0.0f;
    uint64_t device_gps_timestamp_ms = 0;
    bool device_gps_location_valid = settings.share_location &&
                                     settings.device_gps_enabled &&
                                     edgez_platform_get()->gps_get_latest(
                                         &device_gps_latitude,
                                         &device_gps_longitude,
                                         &device_gps_timestamp_ms) &&
                                     location_is_valid(device_gps_latitude,
                                                       device_gps_longitude);
    bool sampled_location_valid = !settings.device_gps_enabled &&
                                  have_sensor_data &&
                                  sensor_data.has_latitude &&
                                  sensor_data.has_longitude &&
                                  location_is_valid(sensor_data.latitude, sensor_data.longitude);
    bool settings_location_valid = !settings.device_gps_enabled &&
                                   settings.share_location &&
                                   location_is_valid(settings.latitude, settings.longitude);
    if (device_gps_location_valid || sampled_location_valid || settings_location_valid) {
        float latitude = device_gps_location_valid
                             ? device_gps_latitude
                             : (sampled_location_valid ? sensor_data.latitude : settings.latitude);
        float longitude = device_gps_location_valid
                              ? device_gps_longitude
                              : (sampled_location_valid ? sensor_data.longitude : settings.longitude);
        /* GPS is reserved first so a full environmental sample cannot crowd
         * it out of the fixed seven-entry beacon SensorData array. */
        APPEND_SENSOR_FLOAT(ai_edgez_halow_SensorType_SENSOR_LATITUDE, latitude);
        APPEND_SENSOR_FLOAT(ai_edgez_halow_SensorType_SENSOR_LONGITUDE, longitude);
    }
    if (have_sensor_data) {
        if (sensor_data.has_accel_x) {
            APPEND_SENSOR_FLOAT(ai_edgez_halow_SensorType_SENSOR_ACCEL_X, sensor_data.accel_x);
        }
        if (sensor_data.has_accel_y) {
            APPEND_SENSOR_FLOAT(ai_edgez_halow_SensorType_SENSOR_ACCEL_Y, sensor_data.accel_y);
        }
        if (sensor_data.has_accel_z) {
            APPEND_SENSOR_FLOAT(ai_edgez_halow_SensorType_SENSOR_ACCEL_Z, sensor_data.accel_z);
        }
        if (sensor_data.has_gyro_x) {
            APPEND_SENSOR_FLOAT(ai_edgez_halow_SensorType_SENSOR_GYRO_X, sensor_data.gyro_x);
        }
        if (sensor_data.has_gyro_y) {
            APPEND_SENSOR_FLOAT(ai_edgez_halow_SensorType_SENSOR_GYRO_Y, sensor_data.gyro_y);
        }
        if (sensor_data.has_gyro_z) {
            APPEND_SENSOR_FLOAT(ai_edgez_halow_SensorType_SENSOR_GYRO_Z, sensor_data.gyro_z);
        }
        if (sensor_data.has_temperature) {
            APPEND_SENSOR_FLOAT(ai_edgez_halow_SensorType_SENSOR_TEMPERATURE, sensor_data.temperature);
        }
        if (sensor_data.has_humidity) {
            APPEND_SENSOR_FLOAT(ai_edgez_halow_SensorType_SENSOR_HUMIDITY, sensor_data.humidity);
        }
        if (sensor_data.has_length) {
            APPEND_SENSOR_INT(ai_edgez_halow_SensorType_SENSOR_LENGTH, sensor_data.length);
        }
    }

#undef APPEND_SENSOR_FLOAT
#undef APPEND_SENSOR_INT
    return true;
}

static bool beacon_sensor_location(const ai_edgez_halow_Beacon *beacon,
                                   float *latitude,
                                   float *longitude)
{
    bool has_latitude = false;
    bool has_longitude = false;
    if (!beacon || !latitude || !longitude) {
        return false;
    }

    for (pb_size_t i = 0; i < beacon->sensor_data_count; i++) {
        const ai_edgez_halow_SensorData *entry = &beacon->sensor_data[i];
        if (entry->which_value != ai_edgez_halow_SensorData_float_value_tag) {
            continue;
        }
        if (entry->type == ai_edgez_halow_SensorType_SENSOR_LATITUDE) {
            *latitude = entry->value.float_value;
            has_latitude = true;
        } else if (entry->type == ai_edgez_halow_SensorType_SENSOR_LONGITUDE) {
            *longitude = entry->value.float_value;
            has_longitude = true;
        }
    }
    return has_latitude && has_longitude;
}

static void send_local_beacon_to_mobile(const ai_edgez_halow_Beacon *beacon)
{
    if (!beacon || !bridge_has_connected_interface()) return;
    uint8_t self_mac[EDGEZ_ROUTE_MAC_LEN] = {0};
    if (!edgez_platform_get()->halow_get_self_mac(self_mac)) return;

    ai_edgez_halow_NetworkPacket packet = ai_edgez_halow_NetworkPacket_init_zero;
    packet.operation = ai_edgez_halow_Operation_BROADCAST;
    packet.interface = ai_edgez_halow_Interface_HALOW;
    packet.from = mac_to_u64(self_mac);
    packet.which_body = ai_edgez_halow_NetworkPacket_beacon_tag;
    packet.body.beacon = *beacon;
    send_network_packet(&packet);
}

static void device_beacon_task(void *arg)
{
    (void)arg;

    while (true) {
        uint32_t interval_seconds = 30;
        uint32_t max_hop = 2;
        char passphrase[65] = {0};
        uint64_t beacon_unicast = 0;
        bool force_once = s_device_beacon_force_once;
        s_device_beacon_force_once = false;
        if (device_sleep_policy_enabled() && !bridge_has_connected_interface()) {
            ESP_LOGI(TAG, "User-mode periodic beacon paused while mobile transport is absent");
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }
        ai_edgez_halow_Beacon beacon = ai_edgez_halow_Beacon_init_zero;
        ai_edgez_halow_DeviceSettings gps_settings =
            ai_edgez_halow_DeviceSettings_init_zero;
        device_settings_get_snapshot(&gps_settings);
        device_gps_apply_if_halow_ready(&gps_settings);
        if (device_settings_build_beacon(&beacon, &interval_seconds, &max_hop,
                                         passphrase, sizeof(passphrase), &beacon_unicast,
                                         force_once)) {
            if (!edgez_platform_get()->halow_ready()) {
                ESP_LOGD(TAG, "Device mode beacon deferred; HaLow interface not ready");
            } else if (device_sleep_policy_enabled() &&
                       !bridge_has_connected_interface()) {
                ESP_LOGI(TAG, "User-mode beacon cancelled while mobile transport is absent");
            } else {
                /* Beacon publication must never wait for a slow UART camera
                 * capture. The sampling task updates the retained SensorData
                 * independently; each beacon uses the latest completed sample. */
                ai_edgez_halow_Report report = ai_edgez_halow_Report_init_zero;
                halow_sync_bridge_fill_report_peers(&report);
                ai_edgez_halow_DeviceSettings settings = ai_edgez_halow_DeviceSettings_init_zero;
                device_settings_get_snapshot(&settings);
                float beacon_latitude = 0.0f;
                float beacon_longitude = 0.0f;
                bool beacon_has_location = beacon_sensor_location(&beacon,
                                                                   &beacon_latitude,
                                                                   &beacon_longitude);
                (void)edgez_platform_get()->network_set_halow_beacon_profile((uint32_t)settings.marker,
                                                         beacon_has_location,
                                                         beacon_latitude,
                                                         beacon_longitude);
                esp_err_t sensor_err = edgez_platform_get()->network_set_halow_beacon_sensor_data(
                    beacon.sensor_data,
                    beacon.sensor_data_count);
                if (sensor_err != ESP_OK) {
                    ESP_LOGW(TAG,
                             "HaLow beacon SensorData update failed count=%u: %s",
                             (unsigned)beacon.sensor_data_count,
                             esp_err_to_name(sensor_err));
                }
                esp_err_t err = edgez_platform_get()->network_refresh_halow_mesh_vendor_ie(settings.mesh_id,
                                                                       settings.passphrase);
                if (err == ESP_OK) {
                    s_device_beacon_last_refresh_ms = route_now_ms();
                }
                bool beacon_error = sensor_err != ESP_OK || err != ESP_OK;
                edgez_platform_get()->led_set_error(EDGEZ_PLATFORM_LED_ERROR_BEACON, beacon_error);
                if (!beacon_error) {
                    edgez_platform_get()->led_flash_beacon();
                }
                send_local_beacon_to_mobile(&beacon);
            }
        }
        /* Preserve the periodic interval, but allow an accepted mobile
         * DEVICE_SETTINGS_SET packet to wake us for an immediate updated
         * beacon instead of waiting up to another full interval. */
        (void)ulTaskNotifyTake(pdTRUE,
                               pdMS_TO_TICKS(interval_seconds * 1000UL));
    }
}

static void apply_beacon_identity(const ai_edgez_halow_Beacon *beacon)
{
    if (!beacon) {
        return;
    }

    (void)edgez_platform_get()->network_set_halow_user_identity(beacon->user_id_high,
                                            beacon->user_id_low,
                                            beacon->user_name,
                                            beacon->user_public_key.bytes,
                                            beacon->user_public_key.size);
    (void)edgez_platform_get()->halow_set_user_identity(beacon->user_id_high,
                                                beacon->user_id_low,
                                                beacon->user_name,
                                                beacon->user_public_key.bytes,
                                                beacon->user_public_key.size);
}

static esp_err_t handle_halow_init_config(const ai_edgez_halow_NetworkPacket *msg)
{
    const ai_edgez_halow_HaLowInitConfig *config = &msg->body.init;
    if (!factory_data_sdk_release_is_authorized()) {
        ESP_LOGW(TAG, "HaLow init rejected: SDK release is not authorized");
        return ESP_ERR_NOT_ALLOWED;
    }

    if (config->has_public_channel_mask) {
        public_channel_mask_set(config->public_channel_mask);
    }

    /* An INIT carrying only the signed SDK release is the authorization
     * handshake. Report the resulting license state without changing radio,
     * identity, or mesh configuration. */
    if (config->mesh_id[0] == '\0' && config->passphrase[0] == '\0') {
        ESP_LOGI(TAG, "HaLow authorization-only init accepted; mesh startup skipped");
        return ESP_OK;
    }

    if (config->country_code[0] == '\0' || config->mesh_id[0] == '\0') {
        ESP_LOGW(TAG, "HaLow init rejected: country_code and mesh_id are required");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG,
             "HaLow init command message_id=%016llx-%016llx country=%s mesh_id=%s passphrase=%s max_hop=%lu user=%016llx-%016llx",
             0ULL,
             0ULL,
             config->country_code,
             config->mesh_id,
             config->passphrase[0] ? "set" : "open",
             (unsigned long)config->max_hop,
             (unsigned long long)config->user_id_high,
             (unsigned long long)config->user_id_low);

    uint64_t user_id_high = config->user_id_high;
    uint64_t user_id_low = config->user_id_low;
    ai_edgez_halow_MarkerColor marker = ai_edgez_halow_MarkerColor_MARKER_DEFAULT;
    if (strcmp(config->marker, "red") == 0) marker = ai_edgez_halow_MarkerColor_MARKER_RED;
    else if (strcmp(config->marker, "orange") == 0) marker = ai_edgez_halow_MarkerColor_MARKER_ORANGE;
    else if (strcmp(config->marker, "yellow") == 0) marker = ai_edgez_halow_MarkerColor_MARKER_YELLOW;
    else if (strcmp(config->marker, "green") == 0) marker = ai_edgez_halow_MarkerColor_MARKER_GREEN;
    else if (strcmp(config->marker, "blue") == 0) marker = ai_edgez_halow_MarkerColor_MARKER_BLUE;
    else if (strcmp(config->marker, "purple") == 0) marker = ai_edgez_halow_MarkerColor_MARKER_PURPLE;
    (void)edgez_platform_get()->network_set_halow_beacon_profile(marker,
                                             config->has_location,
                                             config->latitude,
                                             config->longitude);
    if (user_id_high != 0 || user_id_low != 0 || config->user_name[0] != '\0' || config->user_public_key.size > 0) {
        (void)edgez_platform_get()->network_set_halow_user_identity(user_id_high,
                                                user_id_low,
                                                config->user_name,
                                                config->user_public_key.bytes,
                                                config->user_public_key.size);
        (void)edgez_platform_get()->halow_set_user_identity(user_id_high,
                                                    user_id_low,
                                                    config->user_name,
                                                    config->user_public_key.bytes,
                                                    config->user_public_key.size);
    }

    esp_err_t err = edgez_platform_get()->network_set_halow_country_code(config->country_code);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "HaLow init country update failed: %s", esp_err_to_name(err));
        return err;
    }

    err = edgez_platform_get()->network_set_halow_max_hop(config->max_hop);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "HaLow init max hop update failed: %s", esp_err_to_name(err));
        return err;
    }

    if (config->mesh_frequency_khz != 0 || config->mesh_bandwidth_mhz != 0) {
        err = edgez_platform_get()->network_set_halow_mesh_radio(config->mesh_frequency_khz,
                                             config->mesh_bandwidth_mhz);
        if (err != ESP_OK) {
            ESP_LOGW(TAG,
                     "HaLow init mesh radio update failed: freq=%lu kHz bw=%luMHz: %s",
                     (unsigned long)config->mesh_frequency_khz,
                     (unsigned long)config->mesh_bandwidth_mhz,
                     esp_err_to_name(err));
            return err;
        }
    }

    err = edgez_platform_get()->network_connect_halow(config->mesh_id, config->passphrase);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "HaLow init connect failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static esp_err_t handle_device_settings(const ai_edgez_halow_NetworkPacket *msg,
                                        uint32_t request_id,
                                        bool *sent_status_response)
{
    const ai_edgez_halow_DeviceSettings *settings = &msg->body.device_settings;
    esp_err_t err = ESP_OK;
    if (settings->action == ai_edgez_halow_DeviceSettingsAction_DEVICE_SETTINGS_SET) {
        ai_edgez_halow_DeviceSettings previous = ai_edgez_halow_DeviceSettings_init_zero;
        ai_edgez_halow_DeviceSettings updated = *settings;
        device_settings_get_snapshot(&previous);
        if (updated.mesh_frequency_khz == 0) {
            updated.mesh_frequency_khz = previous.mesh_frequency_khz;
        }
        if (updated.mesh_bandwidth_mhz == 0) {
            updated.mesh_bandwidth_mhz = previous.mesh_bandwidth_mhz;
        }
        if (updated.device_gps_enabled &&
            !edgez_platform_get()->gps_supported()) {
            ESP_LOGW(TAG, "Device GPS enable ignored: L76K support is unavailable");
            updated.device_gps_enabled = false;
        }
        updated.action = ai_edgez_halow_DeviceSettingsAction_DEVICE_SETTINGS_REPORT;
        device_settings_apply_defaults(&updated);
        bool radio_changed = updated.mesh_frequency_khz != 0 &&
                             updated.mesh_bandwidth_mhz != 0 &&
                             (updated.mesh_frequency_khz != previous.mesh_frequency_khz ||
                              updated.mesh_bandwidth_mhz != previous.mesh_bandwidth_mhz);
        if (radio_changed) {
            err = edgez_platform_get()->network_set_halow_mesh_radio(
                updated.mesh_frequency_khz,
                updated.mesh_bandwidth_mhz);
            if (err != ESP_OK) {
                ESP_LOGW(TAG,
                         "Device settings mesh radio rejected freq=%lu kHz bw=%luMHz: %s",
                         (unsigned long)updated.mesh_frequency_khz,
                         (unsigned long)updated.mesh_bandwidth_mhz,
                         esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG,
                         "Device settings mesh radio changed %lu kHz/%luMHz -> %lu kHz/%luMHz; firmware peer/key teardown requested",
                         (unsigned long)previous.mesh_frequency_khz,
                         (unsigned long)previous.mesh_bandwidth_mhz,
                         (unsigned long)updated.mesh_frequency_khz,
                         (unsigned long)updated.mesh_bandwidth_mhz);
            }
        }
        if (updated.has_geo_fence) {
            updated.geo_fence.geo_index = updated.geo_index;
        }
        if (err == ESP_OK) {
            device_settings_apply_snapshot(&updated);
            err = device_settings_save_to_nvs(&updated);
            device_settings_apply_identity(&updated);
            (void)edgez_platform_get()->network_set_halow_max_hop(updated.max_hop);
        }
        if (err == ESP_OK) {
            device_settings_apply_sensor_config(&updated);
            device_gps_apply_if_halow_ready(&updated);
            if (s_device_beacon_task != NULL) {
                s_device_beacon_force_once = true;
                xTaskNotifyGive(s_device_beacon_task);
                ESP_LOGI(TAG,
                         "BLE NetworkPacket.device_settings accepted; immediate beacon triggered");
            }
        }
        device_type_apply_status_led(updated.device_type);
        if (bridge_has_connected_interface()) {
            device_mode_ble_shutdown_cancel();
            mode_deep_sleep_cancel();
        } else {
            disconnected_power_policy_schedule("device_type");
        }
        if (err == ESP_OK && device_type_is_autonomous(updated.device_type)) {
            device_mode_start_halow_from_settings();
        }
    } else {
        ESP_LOGI(TAG, "Device settings GET action=%u", (unsigned)settings->action);
    }

    send_device_settings_frame((uint16_t)request_id, msg);
    if (sent_status_response) {
        *sent_status_response = true;
    }
    return err;
}

static esp_err_t handle_location_update(const ai_edgez_halow_NetworkPacket *msg,
                                        uint32_t request_id,
                                        bool *sent_status_response)
{
    const ai_edgez_halow_LocationUpdate *location = &msg->body.location_update;
    if (!location_is_valid(location->latitude, location->longitude)) {
        ESP_LOGW(TAG,
                 "GPS location update rejected lat=%f lon=%f timestamp_ms=%llu",
                 (double)location->latitude,
                 (double)location->longitude,
                 (unsigned long long)location->timestamp_ms);
        send_status_frame((uint16_t)request_id, msg);
        if (sent_status_response) {
            *sent_status_response = true;
        }
        return ESP_ERR_INVALID_ARG;
    }

    ai_edgez_halow_DeviceSettings updated = ai_edgez_halow_DeviceSettings_init_zero;
    device_settings_get_snapshot(&updated);
    updated.share_location = true;
    updated.latitude = location->latitude;
    updated.longitude = location->longitude;
    /* Tracking fixes are intentionally RAM-only to avoid periodic NVS wear. */
    device_settings_apply_snapshot(&updated);
    (void)edgez_platform_get()->network_set_halow_beacon_profile(
        (uint32_t)updated.marker,
        true,
        updated.latitude,
        updated.longitude);
    const uint32_t now_ms = route_now_ms();
    /* This accessor reads the retained DeviceSettings snapshot populated from
     * DEVICE_SETTINGS_NVS_INTERVAL during startup (and updated after SET). */
    const uint32_t configured_interval_ms =
        halow_sync_bridge_beacon_interval_seconds() * 1000U;
    const uint32_t last_refresh_ms = s_device_beacon_last_refresh_ms;
    const bool immediate_beacon = s_device_beacon_task != NULL &&
        (last_refresh_ms == 0 ||
         (uint32_t)(now_ms - last_refresh_ms) >= configured_interval_ms);
    if (immediate_beacon) {
        s_device_beacon_force_once = true;
        xTaskNotifyGive(s_device_beacon_task);
    }
    ESP_LOGI(TAG,
             "GPS location update accepted lat=%f lon=%f timestamp_ms=%llu immediate_beacon=%u",
             (double)updated.latitude,
             (double)updated.longitude,
             (unsigned long long)location->timestamp_ms,
             immediate_beacon ? 1U : 0U);
    send_status_frame((uint16_t)request_id, msg);
    if (sent_status_response) {
        *sent_status_response = true;
    }
    return ESP_OK;
}

static void notify_beacon_to_mobile(const ai_edgez_halow_Beacon *beacon,
                                    const uint8_t peer_mac[6])
{
    ai_edgez_halow_NetworkPacket msg = ai_edgez_halow_NetworkPacket_init_zero;
    fill_response_base(&msg, NULL);
    if (peer_mac != NULL && !mac_is_zero_bytes(peer_mac)) {
        msg.from = mac_to_u64(peer_mac);
    }
    msg.operation = ai_edgez_halow_Operation_BROADCAST;
    msg.which_body = ai_edgez_halow_NetworkPacket_beacon_tag;
    msg.body.beacon = *beacon;
    send_network_packet(&msg);
}

static void notify_topology_report(const uint8_t peer_mac[6], uint16_t seq)
{
    ai_edgez_halow_NetworkPacket msg = ai_edgez_halow_NetworkPacket_init_zero;
    fill_response_base(&msg, NULL);
    uint8_t self_mac[EDGEZ_ROUTE_MAC_LEN] = {0};
    if (!edgez_platform_get()->halow_get_self_mac(self_mac)) {
        ESP_LOGW(TAG, "NetworkPacket report skipped; local HaLow MAC unavailable");
        return;
    }
    msg.from = mac_to_u64(self_mac);
    msg.to = 0;
    msg.operation = ai_edgez_halow_Operation_BROADCAST;
    msg.which_body = ai_edgez_halow_NetworkPacket_report_tag;
    halow_sync_bridge_fill_report_peers(&msg.body.report);
    if (msg.body.report.peers_count == 0) {
        return;
    }

    const uint32_t now_ms = route_now_ms();
    const uint32_t last_broadcast_ms = s_topology_report_last_broadcast_ms;
    const uint32_t topology_interval_ms =
        halow_sync_bridge_beacon_interval_seconds() * 1000U;
    if (last_broadcast_ms != 0 &&
        (uint32_t)(now_ms - last_broadcast_ms) <
            topology_interval_ms) {
        return;
    }
    s_topology_report_last_broadcast_ms = now_ms;

    /* Deliver locally and publish through BATMAN broadcast at most once per
     * topology interval. BATMAN owns multi-hop fanout and duplicate
     * suppression; the peer cache still updates for every received beacon. */
    send_network_packet(&msg);

    uint8_t encoded[ai_edgez_halow_NetworkPacket_size] = {0};
    pb_ostream_t stream = pb_ostream_from_buffer(encoded, sizeof(encoded));
    if (!pb_encode(&stream, ai_edgez_halow_NetworkPacket_fields, &msg)) {
        ESP_LOGW(TAG,
                 "NetworkPacket report broadcast encode failed: %s",
                 PB_GET_ERROR(&stream));
        return;
    }

    esp_err_t report_err = edgez_platform_get()->halow_send_peer_independent_beacon(
        encoded,
        stream.bytes_written,
        msg.from,
        0,
        0,
        0,
        default_network_packet_max_hop(),
        seq);
    if (report_err == ESP_OK) {
        ESP_LOGI(TAG,
                 "NetworkPacket report broadcast seq=%u observer=0x%012llx heard=0x%012llx peers=%u len=%u",
                 (unsigned)seq,
                 (unsigned long long)(msg.from & 0xffffffffffffULL),
                 (unsigned long long)(peer_mac != NULL ? mac_to_u64(peer_mac) : 0),
                 (unsigned)msg.body.report.peers_count,
                 (unsigned)stream.bytes_written);
    } else if (report_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG,
                 "NetworkPacket report broadcast failed observer=0x%012llx peers=%u err=%s",
                 (unsigned long long)(msg.from & 0xffffffffffffULL),
                 (unsigned)msg.body.report.peers_count,
                 esp_err_to_name(report_err));
    }
}

static void broadcast_beacon(const ai_edgez_halow_Beacon *beacon,
                             const uint8_t peer_mac[6],
                             uint16_t seq)
{
    ai_edgez_halow_NetworkPacket msg = ai_edgez_halow_NetworkPacket_init_zero;
    fill_response_base(&msg, NULL);
    if (!beacon || !peer_mac || mac_is_zero_bytes(peer_mac)) {
        ESP_LOGW(TAG, "NetworkPacket beacon broadcast skipped; peer MAC unavailable");
        return;
    }
    msg.from = mac_to_u64(peer_mac);
    msg.to = 0;
    msg.operation = ai_edgez_halow_Operation_BROADCAST;
    msg.which_body = ai_edgez_halow_NetworkPacket_beacon_tag;
    msg.body.beacon = *beacon;

    uint8_t encoded[ai_edgez_halow_NetworkPacket_size] = {0};
    pb_ostream_t stream = pb_ostream_from_buffer(encoded, sizeof(encoded));
    if (!pb_encode(&stream, ai_edgez_halow_NetworkPacket_fields, &msg)) {
        ESP_LOGW(TAG,
                 "NetworkPacket beacon broadcast encode failed: %s",
                 PB_GET_ERROR(&stream));
        return;
    }

    /* Seed app-delivery dedupe with the locally emitted packet. If another
     * observer rebroadcasts the same beacon back to this node, the app still
     * sees a single discovery event. */
    uint64_t checksum = 0;
    (void)delivery_dedupe_check_and_add(msg.from,
                                        encoded,
                                        stream.bytes_written,
                                        &checksum);

    esp_err_t beacon_err = edgez_platform_get()->halow_send_peer_independent_beacon(
        encoded,
        stream.bytes_written,
        msg.from,
        0,
        0,
        0,
        default_network_packet_max_hop(),
        seq);
    if (beacon_err == ESP_OK) {
        ESP_LOGI(TAG,
                 "NetworkPacket beacon BATMAN broadcast seq=%u from=0x%012llx len=%u checksum=%016llx",
                 (unsigned)seq,
                 (unsigned long long)(msg.from & 0xffffffffffffULL),
                 (unsigned)stream.bytes_written,
                 (unsigned long long)checksum);
    } else if (beacon_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG,
                 "NetworkPacket beacon BATMAN broadcast failed from=0x%012llx err=%s",
                 (unsigned long long)(msg.from & 0xffffffffffffULL),
                 esp_err_to_name(beacon_err));
    }
}

void halow_sync_bridge_notify_beacon(const ai_edgez_halow_Beacon *beacon,
                                     const uint8_t peer_mac[6],
                                     int32_t rssi_dbm)
{
    static portMUX_TYPE notify_dedupe_lock = portMUX_INITIALIZER_UNLOCKED;
    static uint8_t last_peer_mac[6];
    static uint64_t last_user_id_high;
    static uint64_t last_user_id_low;
    static uint32_t last_channel_number;
    static uint32_t last_notify_ms;

    if (!beacon_identity_is_complete(beacon)) {
        ESP_LOGW(TAG,
                 "NetworkPacket beacon notify skipped; incomplete identity user=%016llx-%016llx name=%u key_len=%u type=%u",
                 (unsigned long long)(beacon ? beacon->user_id_high : 0),
                 (unsigned long long)(beacon ? beacon->user_id_low : 0),
                 beacon && beacon->user_name[0] != '\0' ? 1U : 0U,
                 beacon ? (unsigned)beacon->user_public_key.size : 0U,
                 beacon ? (unsigned)beacon->device_type : 0U);
        return;
    }

    /* The Morse live Vendor IE callback and scan-result callback may report
     * the same management frame. Deduplicate only after successful decoding,
     * so a malformed live-filter event can never suppress the scan fallback. */
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    bool duplicate = false;
    if (peer_mac != NULL && !mac_is_zero_bytes(peer_mac)) {
        portENTER_CRITICAL(&notify_dedupe_lock);
        const bool same_identity =
            memcmp(last_peer_mac, peer_mac, sizeof(last_peer_mac)) == 0 &&
            last_user_id_high == beacon->user_id_high &&
            last_user_id_low == beacon->user_id_low;
        const bool channel_is_duplicate =
            beacon->channel_number == 0U ||
            beacon->channel_number == last_channel_number;
        duplicate = same_identity && channel_is_duplicate &&
                    (uint32_t)(now - last_notify_ms) < 500U;
        if (!duplicate) {
            memcpy(last_peer_mac, peer_mac, sizeof(last_peer_mac));
            last_user_id_high = beacon->user_id_high;
            last_user_id_low = beacon->user_id_low;
            last_channel_number = beacon->channel_number;
            last_notify_ms = now;
        }
        portEXIT_CRITICAL(&notify_dedupe_lock);
    }
    if (duplicate) {
        ESP_LOGD(TAG, "Duplicate HaLow beacon suppressed after queued decode");
        return;
    }

    if (peer_mac != NULL && !mac_is_zero_bytes(peer_mac)) {
        uint64_t peer_id = mac_to_u64(peer_mac);
        topology_note_peer(peer_id, rssi_dbm, true, false);
        topology_note_peer_sensor_data(peer_id, beacon);
    }

    uint16_t seq = ++s_from_radio_seq;
    notify_beacon_to_mobile(beacon, peer_mac);
    if (beacon->device_type == ai_edgez_halow_DeviceType_DEVICE_TYPE_BEACON) {
        broadcast_beacon(beacon, peer_mac, seq);
    }
    notify_topology_report(peer_mac, seq);
}

void halow_sync_bridge_notify_batman_peer(const uint8_t originator[6],
                                          const uint8_t last_sender[6],
                                          int32_t rssi_dbm,
                                          uint32_t sequence,
                                          bool direct)
{
    if (!originator || mac_is_zero_bytes(originator)) {
        return;
    }

    uint64_t peer_id = mac_to_u64(originator);
    /* Only direct BATMAN neighbors are physical topology links. Multi-hop
     * originators remain available through the on-demand routing table. */
    topology_note_peer(peer_id, direct ? rssi_dbm : INT32_MIN, false, direct);

    /* BATMAN-IV OGMs are routing control-plane traffic. Keep their peer data
     * in the local topology table, but do not synthesize NetworkPacket.beacon
     * messages for the mobile client. Mobile discovery must contain only
     * real EdgeZ application beacons. */

    ESP_LOGI(TAG,
             "BATMAN peer topology updated orig=%02x:%02x:%02x:%02x:%02x:%02x via=%02x:%02x:%02x:%02x:%02x:%02x rssi=%ld seq=%lu direct=%u",
             originator[0], originator[1], originator[2],
             originator[3], originator[4], originator[5],
             last_sender ? last_sender[0] : 0, last_sender ? last_sender[1] : 0,
             last_sender ? last_sender[2] : 0, last_sender ? last_sender[3] : 0,
             last_sender ? last_sender[4] : 0, last_sender ? last_sender[5] : 0,
             (long)rssi_dbm, (unsigned long)sequence, direct ? 1U : 0U);
}

esp_err_t halow_sync_bridge_handle_to_radio(const uint8_t *payload,
                                            size_t payload_len,
                                            uint32_t request_id,
                                            bool *sent_status_response,
                                            bool via_forward_interface)
{
    if (sent_status_response) {
        *sent_status_response = false;
    }

    ai_edgez_halow_NetworkPacket msg = ai_edgez_halow_NetworkPacket_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(payload, payload_len);
    if (!pb_decode(&stream, ai_edgez_halow_NetworkPacket_fields, &msg)) {
        ESP_LOGW(TAG, "NetworkPacket decode failed: %s", PB_GET_ERROR(&stream));
        return ESP_ERR_INVALID_ARG;
    }

    const bool is_init =
        msg.which_body == ai_edgez_halow_NetworkPacket_init_tag;
    const bool device_authorized =
        factory_data_license_authorize(EDGEZ_LICENSE_CAP_MOBILE_CONTROL);
    bool sdk_authorized = factory_data_sdk_release_is_authorized();
    if (is_init && device_authorized) {
        sdk_authorized = factory_data_sdk_release_authorize(
            msg.body.init.sdk_compatibility,
            msg.body.init.sdk_release_id,
            msg.body.init.sdk_release_signature.bytes,
            msg.body.init.sdk_release_signature.size);
    }
    if (msg.which_body != ai_edgez_halow_NetworkPacket_status_tag &&
        (!device_authorized || !sdk_authorized)) {
        const ai_edgez_halow_LicenseStatus license_status = halow_license_status();
        ESP_LOGW(TAG,
                 "Mobile HaLow request blocked: license_status=%u body=%u",
                 (unsigned)license_status,
                 (unsigned)msg.which_body);
        send_status_frame((uint16_t)request_id, &msg);
        if (sent_status_response) {
            *sent_status_response = true;
        }
        return ESP_ERR_NOT_ALLOWED;
    }

    ai_edgez_halow_Beacon beacon = ai_edgez_halow_Beacon_init_zero;
    char beacon_text[EDGEZ_BEACON_TEXT_MAX_LEN + 1] = {0};
    bool have_beacon = false;
    if (msg.which_body == ai_edgez_halow_NetworkPacket_payload_tag &&
        msg.body.payload.size > 0 && msg.body.payload.size <= EDGEZ_BEACON_TEXT_MAX_LEN) {
        memcpy(beacon_text, msg.body.payload.bytes, msg.body.payload.size);
        have_beacon = decode_beacon_string(beacon_text, &beacon);
    }
    uint32_t packet_max_hop = default_network_packet_max_hop();
    MESH_DEBUG_LOGI(
             "Mobile NetworkPacket message_id=%016llx-%016llx from=0x%012llx to=0x%012llx op=%u iface=%u body=%u user=%016llx-%016llx mime=%u max_hop=%lu payload_len=%u beacon_len=%u beacon_decoded=%u",
             (unsigned long long)msg.body.msg.message_id_high,
             (unsigned long long)msg.body.msg.message_id_low,
             (unsigned long long)(msg.from & 0xffffffffffffULL),
             (unsigned long long)(msg.to & 0xffffffffffffULL),
             (unsigned)msg.operation,
             (unsigned)msg.interface,
             (unsigned)msg.which_body,
             0ULL,
             0ULL,
             msg.which_body == ai_edgez_halow_NetworkPacket_msg_tag ? (unsigned)msg.body.msg.mime : 0U,
             (unsigned long)packet_max_hop,
             msg.which_body == ai_edgez_halow_NetworkPacket_msg_tag ? (unsigned)msg.body.msg.payload.size : 0,
             have_beacon ? (unsigned)msg.body.payload.size : 0,
             have_beacon ? 1 : 0);

    bool forwarding_enabled = halow_sync_bridge_forwarding_enabled();
    if (via_forward_interface && !forwarding_enabled) {
        ESP_LOGW(TAG,
                 "Mobile BLE forward packet ignored; forwarding disabled message_id=%016llx-%016llx",
                 (unsigned long long)msg.body.msg.message_id_high,
                 (unsigned long long)msg.body.msg.message_id_low);
        return ESP_ERR_INVALID_STATE;
    }

    bool is_forward_from_mobile = via_forward_interface ||
                                 (forwarding_enabled &&
                                  msg.interface == ai_edgez_halow_Interface_BLE);
    uint32_t mobile_initial_hop = is_forward_from_mobile ? 1U : 0U;

    esp_err_t err = ESP_OK;
    if (msg.operation == ai_edgez_halow_Operation_ACKNOWLEDGE &&
        msg.which_body == ai_edgez_halow_NetworkPacket_msg_tag) {
        uint64_t next_hop = msg.to;
        bool routed = batman_route_lookup(msg.to, &next_hop);
        MESH_DEBUG_LOGI(
                 "NetworkPacket TX message ACK route target=0x%012llx next=0x%012llx routed=%u message_id=%016llx-%016llx seq=%lu",
                 (unsigned long long)(msg.to & 0xffffffffffffULL),
                 (unsigned long long)(next_hop & 0xffffffffffffULL),
                 routed ? 1 : 0,
                 (unsigned long long)msg.body.msg.message_id_high,
                 (unsigned long long)msg.body.msg.message_id_low,
                 (unsigned long)msg.body.msg.sequence);
        if (is_forward_from_mobile) {
            return edgez_platform_get()->halow_forward_mesh_payload(payload,
                                                          payload_len,
                                                          msg.from,
                                                          msg.to,
                                                          next_hop,
                                                          msg.body.msg.message_id_high,
                                                          msg.body.msg.message_id_low,
                                                          packet_max_hop,
                                                          msg.body.msg.sequence,
                                                          mobile_initial_hop);
        }
        return edgez_platform_get()->halow_send_mesh_payload_via(payload,
                                                        payload_len,
                                                        msg.from,
                                                        msg.to,
                                                        next_hop,
                                                        msg.body.msg.message_id_high,
                                                        msg.body.msg.message_id_low,
                                                        packet_max_hop,
                                                        msg.body.msg.sequence);
    }
    switch (msg.which_body) {
    case ai_edgez_halow_NetworkPacket_msg_tag:
        {
            const bool public_channel = halow_sync_is_public_channel(msg.to);
            if (public_channel &&
                msg.body.msg.mime != ai_edgez_halow_Mime_MIME_TEXT &&
                msg.body.msg.mime != ai_edgez_halow_Mime_MIME_VOICE) {
                ESP_LOGW(TAG,
                         "Public channel TX rejected channel=%llu mime=%u; use conversation text/voice or the OMC PTT transport",
                         (unsigned long long)msg.to,
                         (unsigned)msg.body.msg.mime);
                return ESP_ERR_NOT_SUPPORTED;
            }
            if (public_channel && !halow_sync_public_channel_enabled(msg.to)) {
                ESP_LOGW(TAG,
                         "Public channel TX rejected for disabled channel=%llu",
                         (unsigned long long)msg.to);
                return ESP_ERR_INVALID_STATE;
            }
            uint64_t next_hop = msg.to;
            bool routed = batman_route_lookup(msg.to, &next_hop);
            MESH_DEBUG_LOGI(
                     "NetworkPacket TX route target=0x%012llx next=0x%012llx routed=%u message_id=%016llx-%016llx",
                     (unsigned long long)(msg.to & 0xffffffffffffULL),
                     (unsigned long long)(next_hop & 0xffffffffffffULL),
                     routed ? 1 : 0,
                     (unsigned long long)msg.body.msg.message_id_high,
                     (unsigned long long)msg.body.msg.message_id_low);
            if (msg.body.msg.mime == ai_edgez_halow_Mime_MIME_VOICE_CALL) {
                return voice_tx_enqueue(payload,
                                        payload_len,
                                        &msg,
                                        packet_max_hop,
                                        next_hop,
                                        is_forward_from_mobile,
                                        mobile_initial_hop);
            }
            if (is_forward_from_mobile) {
                err = edgez_platform_get()->halow_forward_mesh_payload(payload,
                                                             payload_len,
                                                             msg.from,
                                                             msg.to,
                                                             next_hop,
                                                             msg.body.msg.message_id_high,
                                                             msg.body.msg.message_id_low,
                                                             packet_max_hop,
                                                             msg.body.msg.sequence,
                                                             mobile_initial_hop);
            } else {
                err = edgez_platform_get()->halow_send_mesh_payload_via(payload,
                                                                payload_len,
                                                                msg.from,
                                                                msg.to,
                                                                next_hop,
                                                                msg.body.msg.message_id_high,
                                                                msg.body.msg.message_id_low,
                                                                packet_max_hop,
                                                                msg.body.msg.sequence);
            }
            /* Global-buffer control and media packets follow the same
             * best-effort contract as realtime voice. Recovery is owned by
             * the app's 2 second GBR1/GBR2 retry loop, so storing these in the
             * firmware pending table only creates duplicate transmissions. */
            if (err == ESP_OK &&
                reliable_packet_is_direct_unicast(&msg) &&
                !global_buffer_packet_is_sensor_exchange(&msg)) {
                reliable_pending_store(&msg,
                                       payload,
                                       payload_len,
                                       next_hop);
            }
            return err;
        }
    case ai_edgez_halow_NetworkPacket_status_tag:
        send_status_frame((uint16_t)request_id, &msg);
        if (sent_status_response) {
            *sent_status_response = true;
        }
        return ESP_OK;
    case ai_edgez_halow_NetworkPacket_init_tag:
        err = handle_halow_init_config(&msg);
        send_status_frame((uint16_t)request_id, &msg);
        if (sent_status_response) {
            *sent_status_response = true;
        }
        return err;
    case ai_edgez_halow_NetworkPacket_payload_tag:
        if (!have_beacon) {
            ESP_LOGW(TAG,
                     "Mobile NetworkPacket beacon skipped; decode failed or identity incomplete");
            err = ESP_ERR_INVALID_ARG;
            send_status_frame((uint16_t)request_id, &msg);
            if (sent_status_response) {
                *sent_status_response = true;
            }
            return err;
        }
        apply_beacon_identity(&beacon);
        err = edgez_platform_get()->halow_request_beacon(beacon_text,
                                                packet_max_hop,
                                                0,
                                                0,
                                                0,
                                                0);
        send_status_frame((uint16_t)request_id, &msg);
        if (sent_status_response) {
            *sent_status_response = true;
        }
        return err;
    case ai_edgez_halow_NetworkPacket_device_settings_tag:
        return handle_device_settings(&msg, request_id, sent_status_response);
    case ai_edgez_halow_NetworkPacket_script_config_tag:
        err = handle_script_config(&msg, request_id);
        if (sent_status_response) {
            *sent_status_response = true;
        }
        return err;
    case ai_edgez_halow_NetworkPacket_location_update_tag:
        return handle_location_update(&msg, request_id, sent_status_response);
    case ai_edgez_halow_NetworkPacket_routing_table_tag:
        err = send_routing_table_response(&msg);
        if (sent_status_response) {
            *sent_status_response = true;
        }
        return err;
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

bool halow_sync_bridge_handle_rx_frame(uint8_t *header,
                                       unsigned header_len,
                                       uint8_t *payload,
                                       unsigned payload_len,
                                       const uint8_t *remote_bssid,
                                       bool batman_delivered)
{
    if (!factory_data_license_authorize(EDGEZ_LICENSE_CAP_MESH_RX)) {
        return false;
    }

    if (!header || header_len < 14 || !payload || payload_len == 0) {
        return false;
    }

    uint16_t ethertype = ((uint16_t)header[12] << 8) | header[13];
    bool is_peer_independent = ethertype == HALOW_SYNC_REPORT_ETHERTYPE;
    if ((ethertype != HALOW_SYNC_ETHERTYPE &&
         !is_peer_independent) ||
        payload_len > (EDGEZ_ROUTE_PREFIX_LEN + ai_edgez_halow_NetworkPacket_size)) {
        return false;
    }

    ai_edgez_halow_NetworkPacket msg = ai_edgez_halow_NetworkPacket_init_zero;
    pb_istream_t network_stream = pb_istream_from_buffer(payload, payload_len);
    bool have_network_packet = false;
    bool have_route_prefix = false;
    uint64_t prefix_message_id_high = 0;
    uint64_t prefix_message_id_low = 0;
    uint32_t prefix_sequence = 0;
    uint64_t prefix_from = 0;
    uint64_t prefix_to = 0;
    uint8_t prefix_max_hop = 0;
    uint8_t prefix_hop = 0;
    const uint8_t *network_payload = payload;
    unsigned network_payload_len = payload_len;
    bool deliver_original_network_payload = false;
    const uint8_t *learn_next_hop = (remote_bssid && !mac_is_zero_bytes(remote_bssid)) ? remote_bssid : &header[6];

    if (payload_len > EDGEZ_ROUTE_PREFIX_LEN) {
        uint64_t candidate_message_id_high = read_u64_be(payload);
        uint64_t candidate_message_id_low = read_u64_be(payload + sizeof(uint64_t));
        uint32_t candidate_sequence = read_u32_be(payload + EDGEZ_ROUTE_SEQUENCE_OFFSET);
        uint64_t candidate_from = read_u64_be(payload + EDGEZ_ROUTE_FROM_OFFSET);
        uint64_t candidate_to = read_u64_be(payload + EDGEZ_ROUTE_TO_OFFSET);
        uint8_t candidate_max_hop = payload[EDGEZ_ROUTE_MAX_HOP_OFFSET];
        uint8_t candidate_hop = payload[EDGEZ_ROUTE_HOP_OFFSET];
        bool route_prefix_candidate = (candidate_message_id_high != 0 || candidate_message_id_low != 0) &&
                                      (candidate_from & 0xffff000000000000ULL) == 0 &&
                                      (candidate_to & 0xffff000000000000ULL) == 0;
        if (route_prefix_candidate) {
            have_route_prefix = true;
            prefix_message_id_high = candidate_message_id_high;
            prefix_message_id_low = candidate_message_id_low;
            prefix_sequence = candidate_sequence;
            prefix_from = candidate_from;
            prefix_to = candidate_to;
            prefix_max_hop = candidate_max_hop;
            prefix_hop = candidate_hop;
            network_payload = payload + EDGEZ_ROUTE_PREFIX_LEN;
            network_payload_len = payload_len - EDGEZ_ROUTE_PREFIX_LEN;

            uint8_t self_mac[EDGEZ_ROUTE_MAC_LEN] = {0};
            bool have_self_mac = edgez_platform_get()->halow_get_self_mac(self_mac);
            uint64_t self_mac_u64 = have_self_mac ? mac_to_u64(self_mac) : 0;
            uint64_t target_mac = prefix_to & 0xffffffffffffULL;
            bool target_is_local = mac_is_zero_u64(target_mac) ||
                                   mac_is_broadcast_u64(target_mac) ||
                                   halow_sync_is_public_channel(target_mac) ||
                                   (have_self_mac && target_mac == self_mac_u64);

            MESH_DEBUG_LOGI(
                     "NetworkPacket RX route prefix message_id=%016llx-%016llx seq=%lu from=0x%012llx to=0x%012llx max_hop=%u hop=%u local=%u da=%02x:%02x:%02x:%02x:%02x:%02x sa=%02x:%02x:%02x:%02x:%02x:%02x ta=%02x:%02x:%02x:%02x:%02x:%02x",
                     (unsigned long long)prefix_message_id_high,
                     (unsigned long long)prefix_message_id_low,
                     (unsigned long)prefix_sequence,
                     (unsigned long long)(prefix_from & 0xffffffffffffULL),
                     (unsigned long long)target_mac,
                     (unsigned)prefix_max_hop,
                     (unsigned)prefix_hop,
                     target_is_local ? 1 : 0,
                     header[0], header[1], header[2], header[3], header[4], header[5],
                     header[6], header[7], header[8], header[9], header[10], header[11],
                     remote_bssid ? remote_bssid[0] : 0, remote_bssid ? remote_bssid[1] : 0,
                     remote_bssid ? remote_bssid[2] : 0, remote_bssid ? remote_bssid[3] : 0,
                     remote_bssid ? remote_bssid[4] : 0, remote_bssid ? remote_bssid[5] : 0);

            const bool is_speed =
                network_payload_len >= EDGEZ_SPEED_FRAME_HEADER_SIZE &&
                memcmp(network_payload, EDGEZ_SPEED_RAW_MAGIC,
                       EDGEZ_SPEED_RAW_MAGIC_SIZE) == 0;
            if (!target_is_local && batman_delivered) {
                /* BATMAN only releases a unicast after its destination
                 * originator has been reached. The application address may
                 * name a translated client identity, so do not start a
                 * second, competing forwarding pass here. */
                MESH_DEBUG_LOGI(
                    "NetworkPacket RX accept BATMAN terminal delivery logical_target=0x%012llx local=0x%012llx",
                    (unsigned long long)target_mac,
                    (unsigned long long)self_mac_u64);
            }
            if (!target_is_local && !batman_delivered) {
                uint64_t next_hop = prefix_to;
                bool routed = batman_route_lookup(prefix_to, &next_hop);
                uint64_t ingress_peer = mac_to_u64(learn_next_hop);
                if (is_speed &&
                    (network_payload[4] != 3 || prefix_max_hop > 3 ||
                     read_u64_be(network_payload + 6) == 0)) {
                    MESH_DEBUG_LOGI("Speed RX drop invalid v3 frame/TTL");
                    return true;
                }
                bool invalid_pseudo_target = target_mac <= 0x00000000ffffffffULL;
                bool reverses_to_ingress = target_mac == ingress_peer ||
                                           (routed && (next_hop & 0xffffffffffffULL) == ingress_peer);
                if (invalid_pseudo_target || reverses_to_ingress) {
                    MESH_DEBUG_LOGI(
                        "NetworkPacket RX drop route reversal target=0x%012llx next=0x%012llx ingress=0x%012llx pseudo=%u",
                        (unsigned long long)target_mac,
                        (unsigned long long)(next_hop & 0xffffffffffffULL),
                        (unsigned long long)ingress_peer,
                        invalid_pseudo_target ? 1U : 0U);
                    return true;
                }
                uint8_t best_hop = prefix_hop;
                bool drop_higher_hop = seen_message_drop_higher_hop(prefix_message_id_high,
                                                                    prefix_message_id_low,
                                                                    prefix_sequence,
                                                                    prefix_hop,
                                                                    &best_hop);
                if (drop_higher_hop) {
                    MESH_DEBUG_LOGI(
                             "NetworkPacket RX drop higher-hop message_id=%016llx-%016llx seq=%lu target=0x%012llx self=0x%012llx max_hop=%u hop=%u best_hop=%u",
                             (unsigned long long)prefix_message_id_high,
                             (unsigned long long)prefix_message_id_low,
                             (unsigned long)prefix_sequence,
                             (unsigned long long)target_mac,
                             (unsigned long long)self_mac_u64,
                             (unsigned)prefix_max_hop,
                             (unsigned)prefix_hop,
                             (unsigned)best_hop);
                    return true;
                }
                if (is_speed) {
                    ai_edgez_halow_NetworkPacket forward_route =
                        ai_edgez_halow_NetworkPacket_init_zero;
                    forward_route.which_body =
                        ai_edgez_halow_NetworkPacket_msg_tag;
                    forward_route.from = prefix_from;
                    forward_route.to = prefix_to;
                    forward_route.body.msg.message_id_high =
                        prefix_message_id_high;
                    forward_route.body.msg.message_id_low =
                        prefix_message_id_low;
                    forward_route.body.msg.sequence = prefix_sequence;
                    esp_err_t queue_err = voice_tx_enqueue(
                        network_payload,
                        network_payload_len,
                        &forward_route,
                        prefix_max_hop,
                        next_hop,
                        true,
                        (uint32_t)prefix_hop + 1U);
                    speed_trace_record(
                        SPEED_TRACE_RELAY_IN, network_payload,
                        network_payload_len, prefix_sequence,
                        (uint8_t)(prefix_hop + 1U), prefix_max_hop,
                        s_voice_tx_queue ?
                            uxQueueMessagesWaiting(s_voice_tx_queue) : 0,
                        queue_err);
                    if (queue_err != ESP_OK) {
                        ESP_LOGW(
                            TAG,
                            "Speed relay queue failed transfer=%016llx seq=%lu hop=%u/%u err=%s",
                            (unsigned long long)read_u64_be(network_payload + 6),
                            (unsigned long)prefix_sequence,
                            (unsigned)(prefix_hop + 1U),
                            (unsigned)prefix_max_hop,
                            esp_err_to_name(queue_err));
                    }
                    return true;
                }
                esp_err_t fwd_err = edgez_platform_get()->halow_forward_mesh_payload(network_payload,
                                                                             network_payload_len,
                                                                             prefix_from,
                                                                             prefix_to,
                                                                             next_hop,
                                                                             prefix_message_id_high,
                                                                             prefix_message_id_low,
                                                                             prefix_max_hop,
                                                                             prefix_sequence,
                                                                             (uint32_t)prefix_hop + 1U);
                if (is_speed && (network_payload[5] == 1 || network_payload[5] == 3)) {
                    ESP_LOGD(TAG,
                             "Stream forward speed type=%u transfer=%016llx to=0x%012llx next=0x%012llx hop=%u/%u bytes=%u err=%s",
                             (unsigned)network_payload[5],
                             (unsigned long long)read_u64_be(network_payload + 6),
                             (unsigned long long)target_mac,
                             (unsigned long long)(next_hop & 0xffffffffffffULL),
                             (unsigned)(prefix_hop + 1U),
                             (unsigned)prefix_max_hop,
                             (unsigned)network_payload_len,
                             esp_err_to_name(fwd_err));
                }
                MESH_DEBUG_LOGI(
                         "NetworkPacket RX forward message_id=%016llx-%016llx seq=%lu from=0x%012llx target=0x%012llx next=0x%012llx routed=%u self=0x%012llx max_hop=%u hop=%u best_hop=%u next_hop_count=%u err=%s",
                         (unsigned long long)prefix_message_id_high,
                         (unsigned long long)prefix_message_id_low,
                         (unsigned long)prefix_sequence,
                         (unsigned long long)(prefix_from & 0xffffffffffffULL),
                         (unsigned long long)target_mac,
                         (unsigned long long)(next_hop & 0xffffffffffffULL),
                         routed ? 1 : 0,
                         (unsigned long long)self_mac_u64,
                         (unsigned)prefix_max_hop,
                         (unsigned)prefix_hop,
                         (unsigned)best_hop,
                         (unsigned)(prefix_hop + 1U),
                         esp_err_to_name(fwd_err));
                return true;
            }

            if (network_payload_len >= EDGEZ_SPEED_FRAME_HEADER_SIZE &&
                memcmp(network_payload, EDGEZ_SPEED_RAW_MAGIC,
                       EDGEZ_SPEED_RAW_MAGIC_SIZE) == 0) {
                speed_trace_record(
                    SPEED_TRACE_DEST_OUT, network_payload,
                    network_payload_len, prefix_sequence,
                    (uint8_t)(prefix_hop + 1U), prefix_max_hop,
                    s_mobile_rx_queue ?
                        uxQueueMessagesWaiting(s_mobile_rx_queue) : 0,
                    ESP_OK);
                uint8_t notify[EDGEZ_ROUTE_MAC_LEN + ai_edgez_halow_NetworkPacket_size];
                if (network_payload_len > sizeof(notify) - EDGEZ_ROUTE_MAC_LEN) {
                    return true;
                }
                uint64_t source = prefix_from & 0xffffffffffffULL;
                for (size_t i = 0; i < EDGEZ_ROUTE_MAC_LEN; ++i) {
                    notify[i] = (uint8_t)(source >> (40 - i * 8));
                }
                memcpy(notify + EDGEZ_ROUTE_MAC_LEN,
                       network_payload, network_payload_len);
                bridge_send_voice_frame(
                    notify,
                    (uint16_t)(EDGEZ_ROUTE_MAC_LEN + network_payload_len));
                return true;
            }

            if (network_payload_len > EDGEZ_VOICE_RAW_MAGIC_SIZE + EDGEZ_VOICE_NONCE_SIZE &&
                memcmp(network_payload, EDGEZ_VOICE_RAW_MAGIC, EDGEZ_VOICE_RAW_MAGIC_SIZE) == 0) {
                size_t crypto_len = network_payload_len - EDGEZ_VOICE_RAW_MAGIC_SIZE;
                uint8_t notify[EDGEZ_ROUTE_MAC_LEN + EDGEZ_ROUTE_SEQUENCE_LEN +
                               ai_edgez_halow_NetworkPacket_size];
                if (crypto_len > sizeof(notify) - EDGEZ_ROUTE_MAC_LEN - EDGEZ_ROUTE_SEQUENCE_LEN) {
                    return true;
                }
                uint64_t source = prefix_from & 0xffffffffffffULL;
                for (size_t i = 0; i < EDGEZ_ROUTE_MAC_LEN; ++i) {
                    notify[i] = (uint8_t)(source >> (40 - i * 8));
                }
                notify[6] = (uint8_t)(prefix_sequence >> 24);
                notify[7] = (uint8_t)(prefix_sequence >> 16);
                notify[8] = (uint8_t)(prefix_sequence >> 8);
                notify[9] = (uint8_t)prefix_sequence;
                memcpy(notify + EDGEZ_ROUTE_MAC_LEN + EDGEZ_ROUTE_SEQUENCE_LEN,
                       network_payload + EDGEZ_VOICE_RAW_MAGIC_SIZE,
                       crypto_len);
                bridge_send_voice_frame(
                    notify,
                    (uint16_t)(EDGEZ_ROUTE_MAC_LEN + EDGEZ_ROUTE_SEQUENCE_LEN + crypto_len));
                return true;
            }

            network_stream = pb_istream_from_buffer(network_payload, network_payload_len);
            have_network_packet = pb_decode(&network_stream,
                                            ai_edgez_halow_NetworkPacket_fields,
                                            &msg);
            if (have_network_packet) {
                deliver_original_network_payload = true;
            }
        }
    }
    if (!have_network_packet && !have_route_prefix && payload_len <= ai_edgez_halow_NetworkPacket_size) {
        msg = (ai_edgez_halow_NetworkPacket)ai_edgez_halow_NetworkPacket_init_zero;
        network_stream = pb_istream_from_buffer(payload, payload_len);
        have_network_packet = pb_decode(&network_stream,
                                        ai_edgez_halow_NetworkPacket_fields,
                                        &msg);
    }
    if (is_peer_independent &&
        (!have_network_packet ||
         (msg.which_body != ai_edgez_halow_NetworkPacket_report_tag &&
          msg.which_body != ai_edgez_halow_NetworkPacket_beacon_tag) ||
         (msg.which_body == ai_edgez_halow_NetworkPacket_beacon_tag &&
          (msg.body.beacon.device_type !=
               ai_edgez_halow_DeviceType_DEVICE_TYPE_BEACON ||
           !beacon_identity_is_complete(&msg.body.beacon))))) {
        MESH_DEBUG_LOGI(
            "NetworkPacket RX drop invalid peer-independent packet decoded=%u body=%u len=%u",
            have_network_packet ? 1U : 0U,
            have_network_packet ? (unsigned)msg.which_body : 0U,
            (unsigned)payload_len);
        return false;
    }
    bool have_beacon = false;
    if (have_network_packet) {
        bool is_ack = msg.operation == ai_edgez_halow_Operation_ACKNOWLEDGE;
        if (!is_ack &&
            msg.which_body != ai_edgez_halow_NetworkPacket_msg_tag &&
            msg.which_body != ai_edgez_halow_NetworkPacket_payload_tag &&
            msg.which_body != ai_edgez_halow_NetworkPacket_beacon_tag &&
            msg.which_body != ai_edgez_halow_NetworkPacket_report_tag) {
            ESP_LOGW(TAG,
                     "NetworkPacket RX notify skipped; unsupported body=%u mime=%u len=%u",
                     (unsigned)msg.which_body,
                     msg.which_body == ai_edgez_halow_NetworkPacket_msg_tag ?
                         (unsigned)msg.body.msg.mime : 0U,
                     (unsigned)payload_len);
            return false;
        }
        uint8_t self_mac[EDGEZ_ROUTE_MAC_LEN] = {0};
        bool have_self_mac = edgez_platform_get()->halow_get_self_mac(self_mac);
        uint64_t self_mac_u64 = have_self_mac ? mac_to_u64(self_mac) : 0;
        uint8_t max_hop = prefix_max_hop ? prefix_max_hop : (uint8_t)default_network_packet_max_hop();
        uint8_t hop = have_route_prefix ? prefix_hop : 0;
        if (!have_route_prefix && msg.which_body == ai_edgez_halow_NetworkPacket_msg_tag) {
            uint64_t target_mac = msg.to & 0xffffffffffffULL;
            bool target_is_local = mac_is_zero_u64(target_mac) ||
                                   mac_is_broadcast_u64(target_mac) ||
                                   halow_sync_is_public_channel(target_mac) ||
                                   (have_self_mac && target_mac == self_mac_u64);
            if (!target_is_local && batman_delivered) {
                MESH_DEBUG_LOGI(
                    "NetworkPacket RX accept BATMAN terminal delivery without prefix logical_target=0x%012llx local=0x%012llx",
                    (unsigned long long)target_mac,
                    (unsigned long long)self_mac_u64);
            }
            if (!target_is_local && !batman_delivered) {
                uint64_t next_hop = msg.to;
                bool routed = batman_route_lookup(msg.to, &next_hop);
                uint8_t best_hop = hop;
                bool drop_higher_hop = seen_message_drop_higher_hop(msg.body.msg.message_id_high,
                                                                    msg.body.msg.message_id_low,
                                                                    msg.body.msg.sequence,
                                                                    hop,
                                                                    &best_hop);
                if (drop_higher_hop) {
                    MESH_DEBUG_LOGI(
                             "NetworkPacket RX drop higher-hop message_id=%016llx-%016llx seq=%lu target=0x%012llx self=0x%012llx max_hop=%u hop=%u best_hop=%u",
                             (unsigned long long)msg.body.msg.message_id_high,
                             (unsigned long long)msg.body.msg.message_id_low,
                             (unsigned long)msg.body.msg.sequence,
                             (unsigned long long)target_mac,
                             (unsigned long long)self_mac_u64,
                             (unsigned)max_hop,
                             (unsigned)hop,
                             (unsigned)best_hop);
                    return true;
                }
                esp_err_t fwd_err = edgez_platform_get()->halow_forward_mesh_payload(network_payload,
                                                                             network_payload_len,
                                                                             msg.from,
                                                                             msg.to,
                                                                             next_hop,
                                                                             msg.body.msg.message_id_high,
                                                                             msg.body.msg.message_id_low,
                                                                             max_hop,
                                                                             msg.body.msg.sequence,
                                                                             (uint32_t)hop + 1U);
                MESH_DEBUG_LOGI(
                         "NetworkPacket RX forward message_id=%016llx-%016llx seq=%lu from=0x%012llx target=0x%012llx next=0x%012llx routed=%u self=0x%012llx max_hop=%u hop=%u best_hop=%u next_hop_count=%u err=%s",
                         (unsigned long long)msg.body.msg.message_id_high,
                         (unsigned long long)msg.body.msg.message_id_low,
                         (unsigned long)msg.body.msg.sequence,
                         (unsigned long long)(msg.from & 0xffffffffffffULL),
                         (unsigned long long)target_mac,
                         (unsigned long long)(next_hop & 0xffffffffffffULL),
                         routed ? 1 : 0,
                         (unsigned long long)self_mac_u64,
                         (unsigned)max_hop,
                         (unsigned)hop,
                         (unsigned)best_hop,
                         (unsigned)(hop + 1U),
                         esp_err_to_name(fwd_err));
                return true;
            }
        }
        if (msg.operation == ai_edgez_halow_Operation_ACKNOWLEDGE) {
            if (msg.body.msg.sequence != 0) {
                reliable_pending_ack(msg.body.msg.message_id_high, msg.body.msg.message_id_low, msg.body.msg.sequence);
                MESH_DEBUG_LOGI(
                         "NetworkPacket RX reliable ACK consumed without mobile notify message_id=%016llx-%016llx from=0x%012llx seq=%u",
                         (unsigned long long)msg.body.msg.message_id_high,
                         (unsigned long long)msg.body.msg.message_id_low,
                         (unsigned long long)(msg.from & 0xffffffffffffULL),
                         (unsigned)msg.body.msg.sequence);
                return true;
            }
            MESH_DEBUG_LOGI(
                     "NetworkPacket RX message ACK notify message_id=%016llx-%016llx from=0x%012llx seq=0",
                     (unsigned long long)msg.body.msg.message_id_high,
                     (unsigned long long)msg.body.msg.message_id_low,
                     (unsigned long long)(msg.from & 0xffffffffffffULL));
        } else if (msg.which_body == ai_edgez_halow_NetworkPacket_msg_tag &&
                   reliable_packet_is_direct_unicast(&msg)) {
            (void)send_reliable_ack(&msg, self_mac_u64);
        }

        if (global_buffer_packet_is_request(&msg)) {
            const uint8_t *request = msg.body.msg.payload.bytes;
            const bool chunk_request =
                msg.body.msg.payload.size == EDGEZ_GLOBAL_BUFFER_CHUNK_REQUEST_SIZE;
            uint32_t expected_length = 0;
            uint64_t requested_group_id = 0;
            int requested_chunk_index = -1;
            if (chunk_request) {
                for (size_t i = 4; i < 12; ++i) {
                    requested_group_id = (requested_group_id << 8) | request[i];
                }
                requested_chunk_index = ((int)request[12] << 8) | request[13];
            } else {
                expected_length = ((uint32_t)request[4] << 24) |
                                  ((uint32_t)request[5] << 16) |
                                  ((uint32_t)request[6] << 8) |
                                  (uint32_t)request[7];
            }
            uint32_t available_length =
                (uint32_t)edgez_platform_get()->sampling_buffer_serving_length();
            MESH_DEBUG_LOGI(
                     "Global buffer pull request from=0x%012llx expected=%lu available=%lu chunk=%d group=%016llx",
                     (unsigned long long)(msg.from & 0xffffffffffffULL),
                     (unsigned long)expected_length,
                     (unsigned long)available_length,
                     requested_chunk_index,
                     (unsigned long long)requested_group_id);
            esp_err_t pull_err = global_buffer_tx_start_for_requester(
                msg.from,
                msg.body.msg.message_id_high,
                msg.body.msg.message_id_low,
                expected_length,
                requested_group_id,
                requested_chunk_index);
            if (pull_err != ESP_OK) {
                uint8_t response_status = EDGEZ_GLOBAL_BUFFER_STATUS_ERROR;
                uint32_t retry_after_ms = 0;
                if (pull_err == ESP_ERR_INVALID_STATE) {
                    response_status = EDGEZ_GLOBAL_BUFFER_STATUS_BUSY;
                    retry_after_ms = EDGEZ_GLOBAL_BUFFER_BUSY_RETRY_MS;
                } else if (pull_err == ESP_ERR_NOT_FOUND) {
                    response_status = EDGEZ_GLOBAL_BUFFER_STATUS_NOT_FOUND;
                }
                esp_err_t status_err = global_buffer_send_status(msg.from,
                                                                 response_status,
                                                                 retry_after_ms,
                                                                 available_length);
                ESP_LOGW(TAG,
                         "Global buffer pull request failed from=0x%012llx: %s status_reply=%s",
                         (unsigned long long)(msg.from & 0xffffffffffffULL),
                         esp_err_to_name(pull_err),
                         esp_err_to_name(status_err));
            }
            return true;
        }
    } else {
        if (!have_route_prefix && remote_bssid == NULL) {
            ESP_LOGW(TAG, "NetworkPacket RX beacon notify skipped; missing remote BSSID");
            return false;
        }
        fill_response_base(&msg, NULL);
        msg.from = have_route_prefix ? prefix_from : mac_to_u64(remote_bssid);
        MESH_DEBUG_LOGI(
                 "NetworkPacket RX beacon source from=0x%012llx source=%s max_hop=%u hop=%u eth_src=%02x:%02x:%02x:%02x:%02x:%02x",
                 (unsigned long long)(msg.from & 0xffffffffffffULL),
                 have_route_prefix ? "route_prefix" : "rx_ta",
                 have_route_prefix ? (unsigned)prefix_max_hop : 0,
                 have_route_prefix ? (unsigned)prefix_hop : 0,
                 header[6], header[7], header[8],
                 header[9], header[10], header[11]);
        msg.operation = ai_edgez_halow_Operation_BROADCAST;
        msg.which_body = ai_edgez_halow_NetworkPacket_payload_tag;
        if (network_payload_len == 0 || network_payload_len > sizeof(msg.body.payload.bytes)) {
            ESP_LOGW(TAG,
                     "NetworkPacket RX beacon notify skipped; raw len=%u capacity=%u",
                     (unsigned)network_payload_len,
                     (unsigned)sizeof(msg.body.payload.bytes));
            return false;
        }
        msg.body.payload.size = network_payload_len;
        memcpy(msg.body.payload.bytes, network_payload, network_payload_len);
        have_beacon = true;
        MESH_DEBUG_LOGI(
                 "NetworkPacket RX beacon forwarded raw len=%u",
                 (unsigned)network_payload_len);
    }

    uint16_t seq = ++s_from_radio_seq;
    bool notify_has_message = msg.which_body == ai_edgez_halow_NetworkPacket_msg_tag;
    uint64_t notify_message_id_high = have_route_prefix ? prefix_message_id_high :
                                      (notify_has_message ? msg.body.msg.message_id_high : 0);
    uint64_t notify_message_id_low = have_route_prefix ? prefix_message_id_low :
                                     (notify_has_message ? msg.body.msg.message_id_low : 0);
    uint64_t notify_from = have_route_prefix ? prefix_from : msg.from;
    uint64_t notify_to = have_route_prefix ? prefix_to : msg.to;
    if (halow_sync_is_public_channel(notify_to) &&
        !halow_sync_public_channel_enabled(notify_to)) {
        MESH_DEBUG_LOGI(
            "NetworkPacket RX mobile notify suppressed for disabled public channel=%llu",
            (unsigned long long)notify_to);
        return true;
    }
    bool notify_is_ack = have_network_packet && msg.operation == ai_edgez_halow_Operation_ACKNOWLEDGE;
    bool notify_is_report = have_network_packet &&
                            msg.which_body == ai_edgez_halow_NetworkPacket_report_tag;
    bool notify_is_beacon = have_network_packet &&
                            msg.which_body == ai_edgez_halow_NetworkPacket_beacon_tag;
    /* A BATMAN-delivered beacon is an application discovery announcement,
     * not evidence of a direct radio link. Do not feed it through
     * halow_sync_bridge_notify_beacon() or update s_topology_peers here;
     * topology reports remain the sole authority for physical adjacency. */
    bool notify_is_global_buffer_chunk = have_network_packet &&
                                         global_buffer_packet_is_chunk(&msg);
    const uint8_t *dedupe_message = have_beacon ?
                                    msg.body.payload.bytes :
                                    ((notify_is_ack || notify_is_report || notify_is_beacon ||
                                      notify_is_global_buffer_chunk) ?
                                         network_payload : msg.body.msg.payload.bytes);
    size_t dedupe_message_len = have_beacon ?
                                msg.body.payload.size :
                                ((notify_is_ack || notify_is_report || notify_is_beacon ||
                                  notify_is_global_buffer_chunk) ?
                                     network_payload_len : msg.body.msg.payload.size);
    uint64_t delivery_checksum = 0;
    if (delivery_dedupe_check_and_add(notify_from,
                                      dedupe_message,
                                      dedupe_message_len,
                                      &delivery_checksum)) {
        MESH_DEBUG_LOGI(
                 "NetworkPacket RX notify dedupe drop checksum=%016llx from=0x%012llx message_id=%016llx-%016llx window_ms=%u message_len=%u",
                 (unsigned long long)delivery_checksum,
                 (unsigned long long)(notify_from & 0xffffffffffffULL),
                 (unsigned long long)notify_message_id_high,
                 (unsigned long long)notify_message_id_low,
                 (unsigned)EDGEZ_DELIVERY_DEDUPE_WINDOW_MS,
                 (unsigned)dedupe_message_len);
        return true;
    }
    MESH_DEBUG_LOGI(
             "NetworkPacket RX notify seq=%u message_id=%016llx-%016llx from=0x%012llx to=0x%012llx body_from=0x%012llx body_to=0x%012llx user=%016llx-%016llx mime=%u packet_seq=%u route_prefix=%u original=%u beacon=%u beacon_len=%u payload_len=%u checksum=%016llx",
             (unsigned)seq,
             (unsigned long long)notify_message_id_high,
             (unsigned long long)notify_message_id_low,
             (unsigned long long)(notify_from & 0xffffffffffffULL),
             (unsigned long long)(notify_to & 0xffffffffffffULL),
             (unsigned long long)(msg.from & 0xffffffffffffULL),
             (unsigned long long)(msg.to & 0xffffffffffffULL),
             0ULL,
             0ULL,
             notify_has_message ? (unsigned)msg.body.msg.mime : 0U,
             notify_has_message ? (unsigned)msg.body.msg.sequence : 0U,
             have_route_prefix ? 1 : 0,
             deliver_original_network_payload ? 1 : 0,
             have_beacon ? 1 : 0,
             have_beacon ? (unsigned)msg.body.payload.size : 0,
             (!have_beacon && notify_has_message) ? (unsigned)msg.body.msg.payload.size : 0U,
             (unsigned long long)delivery_checksum);
    if (deliver_original_network_payload) {
        if (notify_has_message && msg.body.msg.mime == ai_edgez_halow_Mime_MIME_VOICE_CALL) {
            bridge_send_voice_frame(network_payload, (uint16_t)network_payload_len);
        } else if ((s_active_interface == HALOW_SYNC_ACTIVE_INTERFACE_BLE ||
                    s_active_interface == HALOW_SYNC_ACTIVE_INTERFACE_USB) &&
                   notify_has_message && global_buffer_packet_is_chunk(&msg)) {
            /* The radio packet has already been decoded and ACKed locally.
             * Avoid wrapping the image chunk in another protobuf/FFF2 frame:
             * FFF8 can carry the source plus the existing EV2 chunk in one
             * media notification, using the same high-throughput path as a
             * realtime voice frame. */
            uint8_t media[EDGEZ_GLOBAL_BUFFER_BLE_MAGIC_SIZE + EDGEZ_ROUTE_MAC_LEN +
                          EDGEZ_GLOBAL_BUFFER_CHUNK_PAYLOAD_SIZE];
            size_t media_len = EDGEZ_GLOBAL_BUFFER_BLE_MAGIC_SIZE +
                               EDGEZ_ROUTE_MAC_LEN + msg.body.msg.payload.size;
            if (media_len <= sizeof(media)) {
                memcpy(media, EDGEZ_GLOBAL_BUFFER_BLE_MAGIC,
                       EDGEZ_GLOBAL_BUFFER_BLE_MAGIC_SIZE);
                uint64_t source = msg.from & 0xffffffffffffULL;
                for (size_t i = 0; i < EDGEZ_ROUTE_MAC_LEN; ++i) {
                    media[EDGEZ_GLOBAL_BUFFER_BLE_MAGIC_SIZE + i] =
                        (uint8_t)(source >> (40 - i * 8));
                }
                memcpy(media + EDGEZ_GLOBAL_BUFFER_BLE_MAGIC_SIZE + EDGEZ_ROUTE_MAC_LEN,
                       msg.body.msg.payload.bytes,
                       msg.body.msg.payload.size);
                bridge_send_voice_frame(media, (uint16_t)media_len);
                MESH_DEBUG_LOGI(
                    "Global buffer BLE media notify from=0x%012llx seq=%lu chunk_len=%u media_len=%u",
                    (unsigned long long)source,
                    (unsigned long)msg.body.msg.sequence,
                    (unsigned)msg.body.msg.payload.size,
                    (unsigned)media_len);
            } else {
                ESP_LOGW(TAG,
                         "Global buffer BLE media frame too large chunk=%u cap=%u",
                         (unsigned)msg.body.msg.payload.size,
                         (unsigned)sizeof(media));
            }
        } else {
            bridge_send_frame(network_payload,
                             (uint16_t)network_payload_len,
                             halow_sync_bridge_forwarding_enabled());
        }
        return true;
    }
    send_network_packet(&msg);
    return true;
}

static bool mobile_data_queues_init(void)
{
    if (s_voice_tx_queue && s_mobile_rx_queue) return true;

    const UBaseType_t tx_depth = EDGEZ_REALTIME_TX_QUEUE_DEPTH;
    const UBaseType_t rx_depth =
        (UBaseType_t)(EDGEZ_MOBILE_RX_QUEUE_BYTES / sizeof(edgez_mobile_rx_item_t));
    if (tx_depth == 0 || rx_depth == 0) return false;

    const size_t tx_queue_bytes = tx_depth * sizeof(edgez_voice_tx_item_t);
    s_voice_tx_queue_storage = (uint8_t *)heap_caps_malloc(
        tx_queue_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_mobile_rx_queue_storage = (uint8_t *)heap_caps_malloc(
        EDGEZ_MOBILE_RX_QUEUE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_voice_tx_queue_storage || !s_mobile_rx_queue_storage) {
        heap_caps_free(s_voice_tx_queue_storage);
        heap_caps_free(s_mobile_rx_queue_storage);
        s_voice_tx_queue_storage = NULL;
        s_mobile_rx_queue_storage = NULL;
        return false;
    }

    s_voice_tx_queue = xQueueCreateStatic(
        tx_depth, sizeof(edgez_voice_tx_item_t), s_voice_tx_queue_storage,
        &s_voice_tx_queue_control);
    s_mobile_rx_queue = xQueueCreateStatic(
        rx_depth, sizeof(edgez_mobile_rx_item_t), s_mobile_rx_queue_storage,
        &s_mobile_rx_queue_control);
    if (!s_voice_tx_queue || !s_mobile_rx_queue) {
        if (s_voice_tx_queue) vQueueDelete(s_voice_tx_queue);
        if (s_mobile_rx_queue) vQueueDelete(s_mobile_rx_queue);
        s_voice_tx_queue = NULL;
        s_mobile_rx_queue = NULL;
        heap_caps_free(s_voice_tx_queue_storage);
        heap_caps_free(s_mobile_rx_queue_storage);
        s_voice_tx_queue_storage = NULL;
        s_mobile_rx_queue_storage = NULL;
        return false;
    }

    ESP_LOGI(TAG,
             "Mobile data queues ready in PSRAM realtime_TX=%u bytes/%u frames best_effort=1 RX=%u bytes/%u frames",
             (unsigned)tx_queue_bytes,
             (unsigned)tx_depth,
             (unsigned)EDGEZ_MOBILE_RX_QUEUE_BYTES,
             (unsigned)rx_depth);
    return true;
}

void halow_sync_bridge_init(void)
{
    atomic_store_explicit(&s_mobile_log_stream_enabled, false,
                          memory_order_release);
    atomic_store_explicit(&s_mobile_log_delivery_active, false,
                          memory_order_release);
    s_from_radio_seq = 0;
    portENTER_CRITICAL(&s_seen_message_lock);
    memset(s_seen_messages, 0, sizeof(s_seen_messages));
    s_seen_message_replace_index = 0;
    portEXIT_CRITICAL(&s_seen_message_lock);
    portENTER_CRITICAL(&s_delivery_dedupe_lock);
    memset(s_delivery_dedupe, 0, sizeof(s_delivery_dedupe));
    portEXIT_CRITICAL(&s_delivery_dedupe_lock);
    portENTER_CRITICAL(&s_topology_lock);
    memset(s_topology_peers, 0, sizeof(s_topology_peers));
    portEXIT_CRITICAL(&s_topology_lock);
    portENTER_CRITICAL(&s_reliable_pending_lock);
    memset(s_reliable_pending, 0, sizeof(s_reliable_pending));
    portEXIT_CRITICAL(&s_reliable_pending_lock);
    portENTER_CRITICAL(&s_realtime_path_cache_lock);
    memset(s_realtime_path_cache, 0, sizeof(s_realtime_path_cache));
    s_realtime_path_cache_replace_index = 0;
    portEXIT_CRITICAL(&s_realtime_path_cache_lock);
    if (!mobile_data_queues_init()) {
        ESP_LOGE(TAG, "Failed to create PSRAM mobile TX/RX queues");
    }
    if (factory_data_license_authorize(EDGEZ_LICENSE_CAP_RADIO_INIT)) {
        esp_err_t halow_err = edgez_platform_get()->halow_init();
        edgez_platform_get()->led_set_error(EDGEZ_PLATFORM_LED_ERROR_HALOW, halow_err != ESP_OK);
    } else {
        ESP_LOGW(TAG, "HaLow adapter init skipped: device is not licensed");
        edgez_platform_get()->led_set_error(EDGEZ_PLATFORM_LED_ERROR_HALOW, true);
    }
    (void)device_settings_load_from_nvs();
    ai_edgez_halow_DeviceSettings led_settings = ai_edgez_halow_DeviceSettings_init_zero;
    device_settings_get_snapshot(&led_settings);
    device_type_apply_status_led(led_settings.device_type);

    if (s_status_report_task == NULL) {
        BaseType_t created = xTaskCreateWithCaps(status_report_task,
                                                 "halow_status",
                                                 4096,
                                                 NULL,
                                                 4,
                                                 &s_status_report_task,
                                                 EDGEZ_APP_TASK_STACK_CAPS);
        if (created != pdPASS) {
            s_status_report_task = NULL;
            ESP_LOGW(TAG, "Failed to create HaLow status report task");
        }
    }
    if (s_device_beacon_task == NULL) {
        BaseType_t created = xTaskCreateWithCaps(device_beacon_task,
                                                 "device_beacon",
                                                 DEVICE_BEACON_TASK_STACK_SIZE,
                                                 NULL,
                                                 4,
                                                 &s_device_beacon_task,
                                                 EDGEZ_APP_TASK_STACK_CAPS);
        if (created != pdPASS) {
            s_device_beacon_task = NULL;
            ESP_LOGW(TAG, "Failed to create device beacon task");
        }
    }
    if (s_ble_shutdown_task == NULL) {
        /* esp_deep_sleep_start() requires an internal-RAM caller stack when
         * deep-sleep GPIO hold is enabled for the HaLow reset pin. */
        BaseType_t created = xTaskCreateWithCaps(device_mode_ble_shutdown_task,
                                                 "device_ble_off",
                                                 4096,
                                                 NULL,
                                                 4,
                                                 &s_ble_shutdown_task,
                                                 EDGEZ_SLEEP_TASK_STACK_CAPS);
        if (created != pdPASS) {
            s_ble_shutdown_task = NULL;
            ESP_LOGW(TAG, "Failed to create device mode BLE shutdown task");
        }
    }
    if (s_reliable_tx_task == NULL) {
        BaseType_t created = xTaskCreateWithCaps(reliable_tx_task,
                                                 "halow_reliable",
                                                 DEVICE_RELIABLE_TASK_STACK_SIZE,
                                                 NULL,
                                                 4,
                                                 &s_reliable_tx_task,
                                                 EDGEZ_APP_TASK_STACK_CAPS);
        if (created != pdPASS) {
            s_reliable_tx_task = NULL;
            ESP_LOGW(TAG, "Failed to create reliable TX task");
        }
    }
    if (s_voice_tx_queue != NULL && s_voice_tx_task == NULL) {
        BaseType_t created = xTaskCreateWithCaps(voice_tx_task,
                                                 "halow_voice_tx",
                                                 DEVICE_VOICE_TX_TASK_STACK_SIZE,
                                                 NULL,
                                                 5,
                                                 &s_voice_tx_task,
                                                 EDGEZ_APP_TASK_STACK_CAPS);
        if (created != pdPASS) {
            s_voice_tx_task = NULL;
            ESP_LOGW(TAG, "Failed to create voice TX task");
        }
    }
    if (s_mobile_rx_queue != NULL && s_mobile_rx_task == NULL) {
        BaseType_t created = xTaskCreateWithCaps(mobile_rx_task,
                                                 "halow_mobile_rx",
                                                 DEVICE_VOICE_TX_TASK_STACK_SIZE,
                                                 NULL,
                                                 4,
                                                 &s_mobile_rx_task,
                                                 EDGEZ_APP_TASK_STACK_CAPS);
        if (created != pdPASS) {
            s_mobile_rx_task = NULL;
            ESP_LOGW(TAG, "Failed to create mobile RX task");
        }
    }
    ai_edgez_halow_DeviceSettings startup_settings = ai_edgez_halow_DeviceSettings_init_zero;
    device_settings_get_snapshot(&startup_settings);
    boot_power_policy_schedule();
    if (device_type_is_autonomous(startup_settings.device_type)) {
        device_mode_start_halow_from_settings();
    }
}
