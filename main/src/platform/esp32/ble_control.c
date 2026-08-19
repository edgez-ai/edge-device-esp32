#include "ble_control.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_bt.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_hs_id.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"

#include "edgez_frame_protocol.h"
#include "factory_data.h"
#include "halow_sync_bridge.h"
#include "openmanet_comms.h"
#include "log_stream_protocol.h"
#include "usb_control_transport.h"

void ble_store_config_init(void);

static const char *TAG = "ble_control";

#ifndef BLE_CONTROL_HEX_DUMP_LOG
#define BLE_CONTROL_HEX_DUMP_LOG 0
#endif

#define BLE_LOG_PROTOCOL_MAX_PAYLOAD 256

#ifdef CONFIG_MM_MESH_DEBUG_LOG
#define MESH_DEBUG_LOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define MESH_DEBUG_LOGI(...) do {} while (0)
#endif

static void ble_control_apply_log_exclusions(void)
{
    /* NimBLE emits INFO/DEBUG for every GATT notify. Streaming those records
     * over GATT creates noise and can form a notify -> log feedback loop.
     * Preserve only actionable NimBLE warnings and errors. */
    esp_log_level_set("NimBLE", ESP_LOG_WARN);
}

void ble_control_cap_log_level_for_ble(void)
{
    esp_log_level_t level = esp_log_level_get(NULL);
    if (level > ESP_LOG_DEBUG) level = ESP_LOG_DEBUG;
    /* Wildcard reset also removes tag-specific VERBOSE overrides previously
     * installed while USB was the active transport. */
    esp_log_level_set("*", level);
    ble_control_apply_log_exclusions();
}

static bool s_bt_classic_mem_released;
static bool s_ble_enabled;
static bool s_ble_pairing_enabled;
static bool s_ble_secured;
static bool s_ble_host_synced;
static bool s_ble_factory_passkey_valid;
static uint8_t s_ble_addr_type;
static uint32_t s_ble_factory_passkey;
static uint16_t s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool s_ble_data_len_requested;
static bool s_ble_phy_requested;
static uint16_t s_ble_control_tx_val_handle;
static uint16_t s_ble_voice_tx_val_handle;
static uint16_t s_ble_ota_status_val_handle;
static uint8_t s_ble_control_rx[EDGEZ_FRAME_MAX_LEN * 2];
static uint8_t s_ble_forward_rx[EDGEZ_FRAME_MAX_LEN * 2];
static uint16_t s_ble_control_rx_len;
static uint16_t s_ble_forward_rx_len;
static const char *const s_ble_control_name = "BLE control";
static esp_ota_handle_t s_ble_ota_handle;
static const esp_partition_t *s_ble_ota_partition;
static uint32_t s_ble_ota_expected_size;
static uint32_t s_ble_ota_received_size;
static const ble_uuid16_t s_ble_service_uuid = BLE_UUID16_INIT(0xfff0);
typedef enum {
    BLE_CONTROL_CHANNEL_CONTROL,
    BLE_CONTROL_CHANNEL_FORWARD,
    BLE_CONTROL_CHANNEL_VOICE,
    BLE_CONTROL_CHANNEL_OTA,
} ble_control_channel_t;

enum {
    BLE_VOICE_PROTOCOL_VERSION = 2,
    BLE_VOICE_PROTOCOL_HEADER_LEN = 3,
};

static const uint8_t s_ble_voice_protocol_magic[BLE_VOICE_PROTOCOL_HEADER_LEN] = {'V', 'C', BLE_VOICE_PROTOCOL_VERSION};
static const uint8_t s_ble_speed_protocol_magic[BLE_VOICE_PROTOCOL_HEADER_LEN] = {'S', 'T', BLE_VOICE_PROTOCOL_VERSION};

enum {
    BLE_OTA_BEGIN = 1,
    BLE_OTA_DATA = 2,
    BLE_OTA_END = 3,
    BLE_OTA_ABORT = 4,
    BLE_OTA_STATUS_READY = 1,
    BLE_OTA_STATUS_ERROR = 2,
    BLE_OTA_STATUS_COMPLETE = 3,
    BLE_OTA_STATUS_ABORTED = 4,
};

typedef struct {
    ble_control_channel_t channel;
    const char *name;
    uint8_t *rx_buf;
    uint16_t *rx_len;
    uint16_t tx_val_handle;
} ble_control_endpoint_t;

static const ble_control_endpoint_t s_ble_endpoints[] = {
    {
        .channel = BLE_CONTROL_CHANNEL_CONTROL,
        .name = "BLE control",
        .rx_buf = s_ble_control_rx,
        .rx_len = &s_ble_control_rx_len,
        .tx_val_handle = 0,
    },
    {
        .channel = BLE_CONTROL_CHANNEL_FORWARD,
        .name = "BLE forward",
        .rx_buf = s_ble_forward_rx,
        .rx_len = &s_ble_forward_rx_len,
        .tx_val_handle = 0,
    },
};

static const ble_control_endpoint_t *ble_control_endpoint_for(ble_control_channel_t channel)
{
    if (channel == BLE_CONTROL_CHANNEL_FORWARD) {
        return &s_ble_endpoints[1];
    }
    return &s_ble_endpoints[0];
}

static void log_hex_limited(const char *label, const uint8_t *data, size_t len)
{
#if BLE_CONTROL_HEX_DUMP_LOG
    if (data == NULL || len == 0) {
        ESP_LOGI(TAG, "%s empty", label);
        return;
    }

    size_t dump_len = len > 96 ? 96 : len;
    ESP_LOGI(TAG, "%s len=%zu dump=%zu%s", label, len, dump_len, len > dump_len ? " truncated" : "");
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, dump_len, ESP_LOG_INFO);
#else
    (void)label;
    (void)data;
    (void)len;
#endif
}

static esp_err_t ble_control_start_advertising(void);
static void ble_control_request_fast_connection(uint16_t conn_handle);
static void ble_control_request_data_length(uint16_t conn_handle);
static void ble_control_request_2m_phy(uint16_t conn_handle);
static int ble_control_gatt_access(uint16_t conn_handle,
                                   uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt,
                                   void *arg);

static void ble_control_request_fast_connection(uint16_t conn_handle)
{
    const struct ble_gap_upd_params params = {
        .itvl_min = 6, /* 7.5 ms, in 1.25 ms units. */
        .itvl_max = 6, /* Keep bulk realtime transfers at the fastest interval. */
        .latency = 0,
        .supervision_timeout = 600, /* 6 seconds, in 10 ms units. */
        .min_ce_len = 0,
        .max_ce_len = 0,
    };
    int rc = ble_gap_update_params(conn_handle, &params);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "BLE fast connection parameter request failed: %d", rc);
    } else {
        MESH_DEBUG_LOGI("BLE fast connection parameters requested (7.5 ms)");
    }

}

static void ble_control_request_data_length(uint16_t conn_handle)
{
    if (s_ble_data_len_requested) {
        return;
    }
    s_ble_data_len_requested = true;
    int rc = ble_gap_set_data_len(conn_handle, 251, 2120);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "BLE 251-byte data length request failed: %d", rc);
    } else {
        MESH_DEBUG_LOGI("BLE 251-byte data length requested");
    }
    if (rc == BLE_HS_EALREADY) {
        ble_control_request_2m_phy(conn_handle);
    }
}

static void ble_control_request_2m_phy(uint16_t conn_handle)
{
    if (s_ble_phy_requested) {
        return;
    }

    /* Some ESP-IDF NimBLE DATA_LEN_CHG events have reported a stale handle.
     * The handle captured by CONNECT remains the source of truth while the
     * secured link is active. */
    if (s_ble_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_ble_secured) {
        ESP_LOGW(TAG, "BLE 2M PHY request deferred; no secured connection");
        return;
    }
    if (conn_handle != s_ble_conn_handle) {
        ESP_LOGW(TAG,
                 "BLE 2M PHY event handle=%u differs from active handle=%u; using active handle",
                 conn_handle,
                 s_ble_conn_handle);
        conn_handle = s_ble_conn_handle;
    }

    struct ble_gap_conn_desc desc;
    int rc = ble_gap_conn_find(conn_handle, &desc);
    if (rc != 0) {
        ESP_LOGW(TAG,
                 "BLE 2M PHY request deferred; active connection handle=%u unavailable: %d",
                 conn_handle,
                 rc);
        return;
    }

    rc = ble_gap_set_prefered_le_phy(conn_handle,
                                     BLE_GAP_LE_PHY_2M_MASK,
                                     BLE_GAP_LE_PHY_2M_MASK,
                                     0);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        /* Keep retries enabled when the host rejects a transient request. */
        ESP_LOGW(TAG, "BLE 2M PHY request failed handle=%u: %d", conn_handle, rc);
    } else {
        s_ble_phy_requested = true;
        ESP_LOGI(TAG, "BLE 2M PHY requested handle=%u", conn_handle);
    }
}

static const struct ble_gatt_svc_def s_ble_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0xfff0),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0xfff1),
                .access_cb = ble_control_gatt_access,
                .arg = (void *)(uintptr_t)BLE_CONTROL_CHANNEL_CONTROL,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP |
                         BLE_GATT_CHR_F_WRITE_ENC | BLE_GATT_CHR_F_WRITE_AUTHEN,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xfff2),
                .access_cb = ble_control_gatt_access,
                .arg = (void *)(uintptr_t)BLE_CONTROL_CHANNEL_CONTROL,
                .val_handle = &s_ble_control_tx_val_handle,
                /* ESP-IDF NimBLE has no per-notify security declaration flags.
                 * All notification sends are gated on s_ble_secured. */
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xfff5),
                .access_cb = ble_control_gatt_access,
                .arg = (void *)(uintptr_t)BLE_CONTROL_CHANNEL_OTA,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC |
                         BLE_GATT_CHR_F_WRITE_AUTHEN,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xfff6),
                .access_cb = ble_control_gatt_access,
                .arg = (void *)(uintptr_t)BLE_CONTROL_CHANNEL_OTA,
                .val_handle = &s_ble_ota_status_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            /* Add new characteristics after established OTA handles to preserve
             * the GATT handle mapping cached by already-paired Android devices. */
            {
                .uuid = BLE_UUID16_DECLARE(0xfff7),
                .access_cb = ble_control_gatt_access,
                .arg = (void *)(uintptr_t)BLE_CONTROL_CHANNEL_VOICE,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP |
                         BLE_GATT_CHR_F_WRITE_ENC |
                         BLE_GATT_CHR_F_WRITE_AUTHEN,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xfff8),
                .access_cb = ble_control_gatt_access,
                .arg = (void *)(uintptr_t)BLE_CONTROL_CHANNEL_VOICE,
                .val_handle = &s_ble_voice_tx_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 },
        },
    },
    { 0 },
};

static uint16_t ble_control_read_le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void ble_control_write_le16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xff);
    p[1] = (uint8_t)(value >> 8);
}

static uint32_t ble_control_read_le32(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void ble_control_write_le32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value & 0xff);
    p[1] = (uint8_t)((value >> 8) & 0xff);
    p[2] = (uint8_t)((value >> 16) & 0xff);
    p[3] = (uint8_t)((value >> 24) & 0xff);
}

static void ble_control_ota_notify(uint8_t status, uint32_t value)
{
    if (!s_ble_secured || s_ble_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_ble_ota_status_val_handle == 0) {
        return;
    }
    uint8_t response[5] = { status, 0, 0, 0, 0 };
    ble_control_write_le32(&response[1], value);
    struct os_mbuf *om = ble_hs_mbuf_from_flat(response, sizeof(response));
    if (om == NULL) {
        ESP_LOGW(TAG, "BLE OTA status allocation failed");
        return;
    }
    int rc = ble_gatts_notify_custom(s_ble_conn_handle, s_ble_ota_status_val_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "BLE OTA status notify failed rc=%d", rc);
    }
}

static void ble_control_ota_reset(bool abort_update)
{
    if (abort_update && s_ble_ota_handle != 0) {
        (void)esp_ota_abort(s_ble_ota_handle);
    }
    s_ble_ota_handle = 0;
    s_ble_ota_partition = NULL;
    s_ble_ota_expected_size = 0;
    s_ble_ota_received_size = 0;
}

static void ble_control_ota_restart_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(750));
    esp_restart();
}

static int ble_control_process_ota_write(struct ble_gatt_access_ctxt *ctxt)
{
    uint8_t packet[512];
    uint16_t copied = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, packet, sizeof(packet), &copied);
    if (rc != 0 || copied < 1) {
        ble_control_ota_notify(BLE_OTA_STATUS_ERROR, ESP_ERR_INVALID_ARG);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    esp_err_t err = ESP_OK;
    switch (packet[0]) {
    case BLE_OTA_BEGIN: {
        if (copied != 5) {
            err = ESP_ERR_INVALID_ARG;
            break;
        }
        uint32_t size = ble_control_read_le32(&packet[1]);
        const esp_partition_t *partition = esp_ota_get_next_update_partition(NULL);
        if (size == 0 || partition == NULL || size > partition->size) {
            err = ESP_ERR_INVALID_SIZE;
            break;
        }
        ble_control_ota_reset(true);
        err = esp_ota_begin(partition, size, &s_ble_ota_handle);
        if (err == ESP_OK) {
            s_ble_ota_partition = partition;
            s_ble_ota_expected_size = size;
            s_ble_ota_received_size = 0;
            ESP_LOGI(TAG, "BLE OTA started size=%" PRIu32, size);
            ble_control_ota_notify(BLE_OTA_STATUS_READY, size);
            return 0;
        }
        break;
    }
    case BLE_OTA_DATA: {
        if (copied <= 5 || s_ble_ota_handle == 0) {
            err = ESP_ERR_INVALID_STATE;
            break;
        }
        uint32_t offset = ble_control_read_le32(&packet[1]);
        uint16_t data_len = copied - 5;
        if (offset != s_ble_ota_received_size || data_len > s_ble_ota_expected_size - s_ble_ota_received_size) {
            err = ESP_ERR_INVALID_ARG;
            break;
        }
        err = esp_ota_write(s_ble_ota_handle, &packet[5], data_len);
        if (err == ESP_OK) {
            s_ble_ota_received_size += data_len;
            return 0;
        }
        break;
    }
    case BLE_OTA_END:
        if (copied != 1 || s_ble_ota_handle == 0 || s_ble_ota_received_size != s_ble_ota_expected_size) {
            err = ESP_ERR_INVALID_STATE;
            break;
        }
        err = esp_ota_end(s_ble_ota_handle);
        if (err == ESP_OK) {
            err = esp_ota_set_boot_partition(s_ble_ota_partition);
        }
        if (err == ESP_OK) {
            uint32_t image_size = s_ble_ota_received_size;
            ble_control_ota_reset(false);
            ESP_LOGI(TAG, "BLE OTA complete size=%" PRIu32, image_size);
            ble_control_ota_notify(BLE_OTA_STATUS_COMPLETE, image_size);
            xTaskCreate(ble_control_ota_restart_task, "ble_ota_restart", 2048, NULL, 5, NULL);
            return 0;
        }
        break;
    case BLE_OTA_ABORT:
        ble_control_ota_reset(true);
        ble_control_ota_notify(BLE_OTA_STATUS_ABORTED, 0);
        return 0;
    default:
        err = ESP_ERR_NOT_SUPPORTED;
        break;
    }

    ESP_LOGW(TAG, "BLE OTA command=%u failed: %s", packet[0], esp_err_to_name(err));
    ble_control_ota_notify(BLE_OTA_STATUS_ERROR, (uint32_t)err);
    ble_control_ota_reset(true);
    return BLE_ATT_ERR_UNLIKELY;
}

static void ble_control_send_frame_internal(void *ctx,
                                           const uint8_t *payload,
                                           uint16_t payload_len,
                                           uint16_t tx_val_handle,
                                           const char *channel_name)
{
    (void)ctx;

    if (!s_ble_secured || s_ble_conn_handle == BLE_HS_CONN_HANDLE_NONE || tx_val_handle == 0) {
        ESP_LOGW(TAG,
                 "%s TX skipped: secure=%u conn=%u tx_handle=%u",
                 channel_name ? channel_name : "BLE",
                 s_ble_secured ? 1 : 0,
                 s_ble_conn_handle,
                 tx_val_handle);
        return;
    }

    if (payload_len > EDGEZ_FRAME_MAX_PAYLOAD) {
        payload_len = EDGEZ_FRAME_MAX_PAYLOAD;
    }

    uint8_t ble_tx_frame[EDGEZ_FRAME_MAX_LEN] = {0};
    ble_tx_frame[0] = EDGEZ_FRAME_MAGIC_0;
    ble_tx_frame[1] = EDGEZ_FRAME_MAGIC_1;
    ble_control_write_le16(&ble_tx_frame[2], payload_len);
    if (payload_len > 0 && payload != NULL) {
        memcpy(&ble_tx_frame[EDGEZ_FRAME_HEADER_LEN], payload, payload_len);
    }

    uint16_t frame_len = EDGEZ_FRAME_HEADER_LEN + payload_len;
    log_hex_limited("BLE TX frame", ble_tx_frame, frame_len);
    log_hex_limited("BLE TX payload", payload, payload_len);
    struct os_mbuf *om = ble_hs_mbuf_from_flat(ble_tx_frame, frame_len);
    if (om == NULL) {
        ESP_LOGW(TAG, "BLE TX alloc failed len=%u", payload_len);
        return;
    }

    int rc = ble_gatts_notify_custom(s_ble_conn_handle, tx_val_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "BLE notify failed rc=%d", rc);
    } else {
        MESH_DEBUG_LOGI("%s TX protobuf len=%u",
                        channel_name ? channel_name : "BLE", payload_len);
    }
}

void ble_control_send_frame(const uint8_t *payload, uint16_t payload_len)
{
    ble_control_send_frame_internal(NULL,
                                    payload,
                                    payload_len,
                                    s_ble_control_tx_val_handle,
                                    s_ble_control_name);
}

void ble_control_send_forward_frame(const uint8_t *payload, uint16_t payload_len)
{
    /*
     * Keep this compatibility entry point for the mesh bridge, but expose only
     * the control GATT channel. Mobile receives beacons, settings, and peer
     * NetworkPackets from the same subscribed TX characteristic (0xfff2).
     */
    ble_control_send_frame(payload, payload_len);
}

void ble_control_send_voice_frame(const uint8_t *payload, uint16_t payload_len)
{
    bool is_log = payload && payload_len >= EDGEZ_LOG_STREAM_HEADER_LEN &&
                  payload[0] == EDGEZ_LOG_STREAM_MAGIC_0 &&
                  payload[1] == EDGEZ_LOG_STREAM_MAGIC_1 &&
                  payload[2] == EDGEZ_LOG_STREAM_VERSION;
    if (!s_ble_secured || s_ble_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_ble_voice_tx_val_handle == 0) {
        return;
    }
    uint16_t prefix_len = is_log ? 0 : BLE_VOICE_PROTOCOL_HEADER_LEN;
    if (!payload || payload_len == 0 ||
        payload_len > EDGEZ_FRAME_MAX_PAYLOAD - prefix_len) {
        if (!is_log) ESP_LOGW(TAG, "BLE voice TX invalid len=%u", payload_len);
        return;
    }

    uint8_t frame[EDGEZ_FRAME_MAX_PAYLOAD];
    if (prefix_len > 0) {
        memcpy(frame, s_ble_voice_protocol_magic, prefix_len);
    }
    memcpy(&frame[prefix_len], payload, payload_len);
    struct os_mbuf *om = ble_hs_mbuf_from_flat(frame, prefix_len + payload_len);
    if (!om) {
        if (!is_log) ESP_LOGW(TAG, "BLE voice TX allocation failed len=%u", payload_len);
        return;
    }
    int rc = ble_gatts_notify_custom(s_ble_conn_handle, s_ble_voice_tx_val_handle, om);
    if (rc != 0 && !is_log) {
        ESP_LOGW(TAG, "BLE voice notify failed rc=%d", rc);
    }
}

static bool ble_control_handle_log_command(const uint8_t *payload,
                                           uint16_t payload_len)
{
    if (!payload || payload_len < EDGEZ_LOG_STREAM_HEADER_LEN ||
        payload[0] != EDGEZ_LOG_STREAM_MAGIC_0 ||
        payload[1] != EDGEZ_LOG_STREAM_MAGIC_1 ||
        payload[2] != EDGEZ_LOG_STREAM_VERSION ||
        payload[3] != EDGEZ_LOG_STREAM_SET_LEVEL) {
        return false;
    }

    uint8_t requested_level = payload[4];
    uint16_t tag_len = payload_len - EDGEZ_LOG_STREAM_HEADER_LEN;
    uint8_t effective_level = EDGEZ_LOG_STREAM_LEVEL_ERROR;
    bool level_applied = false;
    char tag[64] = "*";
    if (tag_len <= BLE_LOG_PROTOCOL_MAX_PAYLOAD &&
        tag_len < sizeof(tag) && requested_level <= ESP_LOG_VERBOSE) {
        if (tag_len > 0) {
            memcpy(tag, &payload[EDGEZ_LOG_STREAM_HEADER_LEN], tag_len);
            tag[tag_len] = '\0';
        }
        effective_level = requested_level > ESP_LOG_DEBUG
            ? ESP_LOG_DEBUG
            : requested_level;
        if (effective_level > CONFIG_LOG_MAXIMUM_LEVEL) {
            effective_level = CONFIG_LOG_MAXIMUM_LEVEL;
        }
        esp_log_level_set(tag, (esp_log_level_t)effective_level);
        ble_control_apply_log_exclusions();
        level_applied = true;
    }

    uint16_t response_tag_len = effective_level == EDGEZ_LOG_STREAM_LEVEL_ERROR
        ? 0
        : (uint16_t)strlen(tag);
    uint8_t response[EDGEZ_LOG_STREAM_HEADER_LEN + sizeof(tag)] = {
        EDGEZ_LOG_STREAM_MAGIC_0, EDGEZ_LOG_STREAM_MAGIC_1,
        EDGEZ_LOG_STREAM_VERSION, EDGEZ_LOG_STREAM_LEVEL_RESPONSE,
        effective_level,
    };
    if (response_tag_len > 0) {
        memcpy(&response[EDGEZ_LOG_STREAM_HEADER_LEN], tag, response_tag_len);
    }
    /* Level NONE is also the explicit stream-off command. Temporarily allow
     * the response into the asynchronous mobile queue, then stop capturing
     * new ESP_LOG records before this function returns. */
    if (level_applied) {
        halow_sync_bridge_set_log_stream_enabled(true);
    }
    (void)halow_sync_bridge_queue_log_frame(
        response, EDGEZ_LOG_STREAM_HEADER_LEN + response_tag_len);
    if (level_applied && effective_level == ESP_LOG_NONE) {
        halow_sync_bridge_set_log_stream_enabled(false);
    } else if (level_applied) {
        halow_sync_bridge_request_log_level_test();
    }
    return true;
}

static void ble_only_send_frame(void *ctx,
                               const uint8_t *payload,
                               uint16_t payload_len)
{
    (void)ctx;
    ble_control_send_frame_internal(NULL,
                                   payload,
                                   payload_len,
                                   s_ble_control_tx_val_handle,
                                   s_ble_control_name);
}

static void ble_control_process_rx(ble_control_channel_t channel)
{
    const ble_control_endpoint_t *endpoint = ble_control_endpoint_for(channel);
    if (!endpoint) {
        return;
    }

    if (channel == BLE_CONTROL_CHANNEL_FORWARD && !halow_sync_bridge_forwarding_enabled()) {
        ESP_LOGW(TAG,
                 "BLE forward RX ignored: forwarding disabled for %s",
                 endpoint->name ? endpoint->name : "BLE");
        *endpoint->rx_len = 0;
        return;
    }

    uint16_t frame_len = 0;

    while (*endpoint->rx_len >= EDGEZ_FRAME_HEADER_LEN) {
        if (endpoint->rx_buf[0] != EDGEZ_FRAME_MAGIC_0 ||
            endpoint->rx_buf[1] != EDGEZ_FRAME_MAGIC_1) {
            ESP_LOGW(TAG,
                     "%s bad frame header, dropping %u buffered byte(s)",
                     endpoint->name ? endpoint->name : "BLE",
                     *endpoint->rx_len);
            *endpoint->rx_len = 0;
            break;
        }

        uint16_t payload_len = ble_control_read_le16(&endpoint->rx_buf[2]);
        if (payload_len > EDGEZ_FRAME_MAX_PAYLOAD) {
            ESP_LOGW(TAG,
                     "%s bad BLE frame length: %u",
                     endpoint->name ? endpoint->name : "BLE",
                     payload_len);
            *endpoint->rx_len = 0;
            break;
        }

        frame_len = EDGEZ_FRAME_HEADER_LEN + payload_len;
        if (*endpoint->rx_len < frame_len) {
            break;
        }

        MESH_DEBUG_LOGI(
            "%s RX protobuf len=%u",
            endpoint->name ? endpoint->name : "BLE",
            payload_len);
        log_hex_limited("BLE RX frame", endpoint->rx_buf, frame_len);
        log_hex_limited("BLE RX payload",
                        &endpoint->rx_buf[EDGEZ_FRAME_HEADER_LEN],
                        payload_len);
        halow_sync_bridge_note_active_interface(HALOW_SYNC_ACTIVE_INTERFACE_BLE);
        edgez_frame_protocol_handle_frame(endpoint->rx_buf,
                                         frame_len,
                                         ble_only_send_frame,
                                         NULL,
                                         channel == BLE_CONTROL_CHANNEL_FORWARD);

        *endpoint->rx_len -= frame_len;
        if (*endpoint->rx_len > 0) {
            memmove(endpoint->rx_buf, &endpoint->rx_buf[frame_len], *endpoint->rx_len);
        }
    }
}

static int ble_control_gatt_access(uint16_t conn_handle,
                                   uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt,
                                   void *arg)
{
    (void)attr_handle;
    if (!s_ble_secured || conn_handle != s_ble_conn_handle) {
        ESP_LOGW(TAG, "BLE write rejected before pairing completes");
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    ble_control_channel_t channel = (ble_control_channel_t)(uintptr_t)arg;
    if (channel == BLE_CONTROL_CHANNEL_OTA) {
        return ble_control_process_ota_write(ctxt);
    }
    if (channel == BLE_CONTROL_CHANNEL_VOICE) {
        uint8_t packet[EDGEZ_FRAME_MAX_PAYLOAD];
        uint16_t copied = 0;
        int rc = ble_hs_mbuf_to_flat(ctxt->om, packet, sizeof(packet), &copied);
        if (rc != 0 || copied == 0) {
            ESP_LOGW(TAG, "BLE realtime RX invalid frame len=%u rc=%d", copied, rc);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        if (ble_control_handle_log_command(packet, copied)) {
            return 0;
        }
        if (copied <= BLE_VOICE_PROTOCOL_HEADER_LEN) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        esp_err_t err;
        if (memcmp(packet, s_ble_voice_protocol_magic, BLE_VOICE_PROTOCOL_HEADER_LEN) == 0) {
            const uint8_t *realtime = &packet[BLE_VOICE_PROTOCOL_HEADER_LEN];
            size_t realtime_len = copied - BLE_VOICE_PROTOCOL_HEADER_LEN;
            err = realtime_len > OPENMANET_COMMS_MAGIC_LEN &&
                  memcmp(realtime, "OMC", OPENMANET_COMMS_MAGIC_LEN - 1U) == 0 &&
                  (realtime[OPENMANET_COMMS_MAGIC_LEN - 1U] == 1U ||
                   realtime[OPENMANET_COMMS_MAGIC_LEN - 1U] == 2U)
                ? openmanet_comms_send_phone_frame(realtime, realtime_len)
                : halow_sync_bridge_handle_voice_to_radio(realtime, realtime_len);
        } else if (memcmp(packet, s_ble_speed_protocol_magic, BLE_VOICE_PROTOCOL_HEADER_LEN) == 0) {
            err = halow_sync_bridge_handle_speed_to_radio(
                &packet[BLE_VOICE_PROTOCOL_HEADER_LEN],
                copied - BLE_VOICE_PROTOCOL_HEADER_LEN);
        } else {
            ESP_LOGW(TAG, "BLE realtime RX unknown protocol len=%u", copied);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "BLE realtime RX rejected: %s", esp_err_to_name(err));
            return BLE_ATT_ERR_UNLIKELY;
        }
        return 0;
    }
    const ble_control_endpoint_t *endpoint = ble_control_endpoint_for(channel);
    if (!endpoint || !endpoint->rx_buf || !endpoint->rx_len) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (channel == BLE_CONTROL_CHANNEL_FORWARD && !halow_sync_bridge_forwarding_enabled()) {
        ESP_LOGW(TAG,
                 "BLE forward write rejected: forwarding disabled for %s",
                 endpoint->name ? endpoint->name : "BLE");
        *endpoint->rx_len = 0;
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    uint16_t space = sizeof(s_ble_control_rx) - *endpoint->rx_len;
    if (space == 0) {
        ESP_LOGW(TAG,
                 "%s RX overflow, dropping buffered frame",
                 endpoint->name ? endpoint->name : "BLE");
        *endpoint->rx_len = 0;
        space = sizeof(s_ble_control_rx);
    }

    uint16_t copied = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, &endpoint->rx_buf[*endpoint->rx_len], space, &copied);
    if (rc != 0) {
        ESP_LOGW(TAG, "BLE RX flatten failed: %d", rc);
        return BLE_ATT_ERR_UNLIKELY;
    }

    *endpoint->rx_len += copied;
    MESH_DEBUG_LOGI(
        "%s RX chunk=%u buffered=%u",
        endpoint->name ? endpoint->name : "BLE",
        copied,
        *endpoint->rx_len);
    log_hex_limited("BLE RX chunk", &endpoint->rx_buf[*endpoint->rx_len - copied], copied);
    ble_control_process_rx(channel);
    return 0;
}

static void ble_control_host_task(void *param)
{
    (void)param;

    nimble_port_run();
    nimble_port_freertos_deinit();
}

static int ble_control_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG,
                 "BLE connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);
        if (event->connect.status == 0) {
            s_ble_conn_handle = event->connect.conn_handle;
            s_ble_secured = false;
            s_ble_data_len_requested = false;
            s_ble_phy_requested = false;
            s_ble_control_rx_len = 0;
            s_ble_forward_rx_len = 0;
            int rc = ble_gap_security_initiate(event->connect.conn_handle);
            if (rc != 0 && rc != BLE_HS_EALREADY) {
                ESP_LOGW(TAG, "BLE security initiate failed: %d", rc);
                (void)ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            }
        } else if (s_ble_enabled && s_ble_pairing_enabled) {
            (void)ble_control_start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnected; reason=%d", event->disconnect.reason);
        factory_data_sdk_release_reset();
        s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_ble_secured = false;
        s_ble_data_len_requested = false;
        s_ble_phy_requested = false;
        s_ble_control_rx_len = 0;
        s_ble_forward_rx_len = 0;
        halow_sync_bridge_note_ble_connected(false);
        if (s_ble_enabled && s_ble_pairing_enabled) {
            (void)ble_control_start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG,
                 "BLE subscribe attr=%u notify=%d indicate=%d",
                 event->subscribe.attr_handle,
                 event->subscribe.cur_notify,
                 event->subscribe.cur_indicate);
        if (s_ble_secured && event->subscribe.cur_indicate) {
            /* Pairing completes before Android enables the control CCC. Wake
             * the bridge only after indications are writable so the initial
             * device-settings/status pair cannot be lost in that race. */
            halow_sync_bridge_request_status_report();
        }
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "BLE advertising complete; reason=%d", event->adv_complete.reason);
        if (s_ble_enabled && s_ble_pairing_enabled) {
            (void)ble_control_start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc desc;
        int rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        if (rc != 0) {
            ESP_LOGW(TAG, "BLE repeat pairing could not find connection: %d", rc);
            return BLE_GAP_REPEAT_PAIRING_IGNORE;
        }
        rc = ble_store_util_delete_peer(&desc.peer_id_addr);
        if (rc != 0) {
            ESP_LOGW(TAG, "BLE repeat pairing could not delete stale bond: %d", rc);
            return BLE_GAP_REPEAT_PAIRING_IGNORE;
        }
        ESP_LOGI(TAG, "BLE stale bond removed; retrying pairing");
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
        if (event->passkey.params.action != BLE_SM_IOACT_DISP ||
            !s_ble_factory_passkey_valid) {
            ESP_LOGW(TAG, "BLE passkey action unsupported or factory PIN unavailable");
            return BLE_HS_EAUTHEN;
        }

        struct ble_sm_io io = {
            .action = BLE_SM_IOACT_DISP,
            .passkey = s_ble_factory_passkey,
        };
        int rc = ble_sm_inject_io(event->passkey.conn_handle, &io);
        if (rc != 0) {
            ESP_LOGW(TAG, "BLE factory PIN display submission failed: %d", rc);
        }
        return rc;
    }

    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status == 0) {
            ESP_LOGI(TAG, "BLE pairing complete; encrypted connection ready");
            s_ble_secured = true;
            if (!usb_control_transport_is_connected()) {
                ble_control_cap_log_level_for_ble();
            }
            halow_sync_bridge_note_ble_connected(true);
            ble_control_request_fast_connection(event->enc_change.conn_handle);
            return 0;
        }
        ESP_LOGW(TAG,
                 "BLE pairing failed or canceled; status=%d, disconnecting",
                 event->enc_change.status);
        s_ble_secured = false;
        (void)ble_gap_terminate(event->enc_change.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE: {
        struct ble_gap_conn_desc desc;
        int rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        if (event->conn_update.status == 0 && rc == 0) {
            MESH_DEBUG_LOGI("BLE connection interval updated to %.2f ms latency=%u",
                            desc.conn_itvl * 1.25,
                            desc.conn_latency);
        } else {
            ESP_LOGW(TAG,
                     "BLE connection parameter update failed status=%d rc=%d",
                     event->conn_update.status,
                     rc);
        }
        if (s_ble_secured) {
            ble_control_request_data_length(event->conn_update.conn_handle);
        }
        return 0;
    }

    case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
        ESP_LOGI(TAG,
                 "BLE PHY update status=%u handle=%u tx=%u rx=%u",
                 event->phy_updated.status,
                 event->phy_updated.conn_handle,
                 event->phy_updated.tx_phy,
                 event->phy_updated.rx_phy);
        return 0;

    case BLE_GAP_EVENT_DATA_LEN_CHG:
        MESH_DEBUG_LOGI("BLE data length tx=%u/%uus rx=%u/%uus",
                        event->data_len_chg.max_tx_octets,
                        event->data_len_chg.max_tx_time,
                        event->data_len_chg.max_rx_octets,
                        event->data_len_chg.max_rx_time);
        if (s_ble_secured) {
            ble_control_request_2m_phy(s_ble_conn_handle);
        }
        return 0;

    default:
        return 0;
    }
}

static esp_err_t ble_control_start_advertising(void)
{
    if (!s_ble_enabled || !s_ble_host_synced || !s_ble_pairing_enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (const uint8_t *)ble_svc_gap_device_name();
    fields.name_len = strlen((const char *)fields.name);
    fields.name_is_complete = 1;
    fields.uuids16 = &s_ble_service_uuid;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return ESP_FAIL;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_ble_addr_type,
                           NULL,
                           BLE_HS_FOREVER,
                           &adv_params,
                           ble_control_gap_event,
                           NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BLE pairing advertising enabled");
    return ESP_OK;
}

static void ble_control_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed: %d", rc);
        return;
    }

    rc = ble_hs_id_infer_auto(0, &s_ble_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }

    s_ble_host_synced = true;
    ESP_LOGI(TAG,
             "BLE GATT active: control_tx=0x%04x voice_rx=FFF7 voice_tx=0x%04x ota_status=0x%04x",
             s_ble_control_tx_val_handle,
             s_ble_voice_tx_val_handle,
             s_ble_ota_status_val_handle);
    if (s_ble_pairing_enabled) {
        (void)ble_control_start_advertising();
    }
}

static void ble_control_on_reset(int reason)
{
    s_ble_host_synced = false;
    ESP_LOGW(TAG, "BLE host reset: %d", reason);
}

static void ble_control_make_device_name(char *out, size_t out_len)
{
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_BT) == ESP_OK) {
        snprintf(out, out_len, "EdgeZ-%02X%02X", mac[4], mac[5]);
    } else {
        snprintf(out, out_len, "EdgeZ");
    }
}

static bool ble_control_load_factory_passkey(void)
{
    factory_data_config_t factory_cfg = {0};
    esp_err_t err = factory_data_load(&factory_cfg);
    const char *pin = factory_cfg.ble_pin_code;
    uint32_t passkey = 0;

    s_ble_factory_passkey_valid = false;
    s_ble_factory_passkey = 0;
    if (err != ESP_OK || !factory_cfg.present || strlen(pin) != 6) {
        ESP_LOGW(TAG, "BLE pairing unavailable: factory BLE PIN is missing");
        return false;
    }

    for (size_t i = 0; i < 6; ++i) {
        if (pin[i] < '0' || pin[i] > '9') {
            ESP_LOGW(TAG, "BLE pairing unavailable: factory BLE PIN is invalid");
            return false;
        }
        passkey = (passkey * 10U) + (uint32_t)(pin[i] - '0');
    }

    s_ble_factory_passkey = passkey;
    s_ble_factory_passkey_valid = true;
    ESP_LOGI(TAG, "BLE pairing PIN loaded from factory data");
    return true;
}

esp_err_t ble_control_set_enabled(bool enabled)
{
    if (enabled == s_ble_enabled) {
        return ESP_OK;
    }

    if (!enabled) {
        if (s_ble_pairing_enabled) {
            (void)ble_control_set_pairing_enabled(false);
        }

        int rc = nimble_port_stop();
        if (rc != 0) {
            ESP_LOGW(TAG, "nimble_port_stop failed: %d", rc);
        }

        vTaskDelay(pdMS_TO_TICKS(100));

        esp_err_t err = nimble_port_deinit();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "nimble_port_deinit failed: %s", esp_err_to_name(err));
            return err;
        }

        s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_ble_control_rx_len = 0;
        s_ble_forward_rx_len = 0;
        s_ble_host_synced = false;
        s_ble_enabled = false;
        ESP_LOGI(TAG, "BLE disabled");
        return ESP_OK;
    }

    if (!s_bt_classic_mem_released) {
        esp_err_t err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Failed to release classic BT memory: %s", esp_err_to_name(err));
            return err;
        }
        s_bt_classic_mem_released = true;
    }

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return err;
    }

    ble_hs_cfg.reset_cb = ble_control_on_reset;
    ble_hs_cfg.sync_cb = ble_control_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    (void)ble_control_load_factory_passkey();
    /* The peripheral supplies the factory PIN; Android is the input device. */
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_DISP_ONLY;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_store_config_init();

    int rc = 0;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    rc = ble_gatts_count_cfg(s_ble_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        (void)nimble_port_deinit();
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(s_ble_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        (void)nimble_port_deinit();
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BLE GATT configured: control=FFF1/FFF2 voice=FFF7/FFF8 ota=FFF5/FFF6");

    char device_name[16] = {0};
    ble_control_make_device_name(device_name, sizeof(device_name));
    rc = ble_svc_gap_device_name_set(device_name);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_svc_gap_device_name_set failed: %d", rc);
        (void)nimble_port_deinit();
        return ESP_FAIL;
    }

    nimble_port_freertos_init(ble_control_host_task);

    s_ble_enabled = true;
    ESP_LOGI(TAG, "BLE enabled as %s", device_name);
    return ESP_OK;
}

esp_err_t ble_control_set_pairing_enabled(bool enabled)
{
    if (enabled && !s_ble_enabled) {
        esp_err_t err = ble_control_set_enabled(true);
        if (err != ESP_OK) {
            return err;
        }
    }

    if (enabled && !s_ble_factory_passkey_valid) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!enabled) {
        if (s_ble_enabled && s_ble_host_synced) {
            int rc = ble_gap_adv_stop();
            if (rc != 0 && rc != BLE_HS_EALREADY && rc != BLE_HS_ENOENT) {
                ESP_LOGW(TAG, "ble_gap_adv_stop failed: %d", rc);
            }
        }
        s_ble_pairing_enabled = false;
        ESP_LOGI(TAG, "BLE pairing advertising disabled");
        return ESP_OK;
    }

    s_ble_pairing_enabled = true;
    if (s_ble_host_synced) {
        return ble_control_start_advertising();
    }

    ESP_LOGI(TAG, "BLE pairing advertising pending host sync");
    return ESP_OK;
}

bool ble_control_is_enabled(void)
{
    return s_ble_enabled;
}

bool ble_control_is_pairing_enabled(void)
{
    return s_ble_pairing_enabled;
}

bool ble_control_is_connected(void)
{
    return s_ble_secured &&
           s_ble_conn_handle != BLE_HS_CONN_HANDLE_NONE &&
           s_ble_control_tx_val_handle != 0;
}
