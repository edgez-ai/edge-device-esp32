#include "halow_interface_app.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "factory_data.h"
#include "edgez_halow_events.h"
#include "edgez_halow_frame.hpp"
#include "edgez_halow_radio.hpp"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "mbedtls/base64.h"
#include "pb_encode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "halow_sync_bridge.h"
#include "openmanet_alfred.h"
#include "openmanet_comms.h"
#include "usb_control.pb.h"

extern "C" {
#include "driver/gpio.h"
#include "mmwlan.h"
}

static const char *TAG = "halow_adapter";

#ifdef CONFIG_MM_MESH_DEBUG_LOG
#define MESH_DEBUG_LOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define MESH_DEBUG_LOGI(...) do {} while (0)
#endif

static HaLowInterface s_halow;
static EventGroupHandle_t s_ready_events;
static SemaphoreHandle_t s_lifecycle_mutex;
static TaskHandle_t s_halow_event_task;
static bool s_adapter_initialized;
static bool s_halow_started;
static char s_mesh_id[33];
static char s_country_code[3] = CONFIG_MM_HALOW_COUNTRY_CODE;
static char s_passphrase[65];
static bool s_beacon_only;
static uint32_t s_mesh_frequency_khz;
static uint8_t s_mesh_bandwidth_mhz;

static constexpr EventBits_t HALOW_ADAPTER_READY_BIT = BIT0;
static constexpr uint32_t HALOW_EVENT_TASK_STACK_SIZE = 8192;
static constexpr UBaseType_t HALOW_EVENT_STACK_WARN_BYTES = 1024;
static constexpr UBaseType_t HALOW_EVENT_TASK_PRIORITY = 8;
static constexpr size_t EDGEZ_ROUTE_MESSAGE_ID_LEN = 16;
static constexpr size_t EDGEZ_ROUTE_SEQUENCE_LEN = 4;
static constexpr size_t EDGEZ_ROUTE_MAC_LEN = 6;
static constexpr size_t EDGEZ_ROUTE_SEQUENCE_OFFSET = EDGEZ_ROUTE_MESSAGE_ID_LEN;
static constexpr size_t EDGEZ_ROUTE_FROM_OFFSET = EDGEZ_ROUTE_SEQUENCE_OFFSET + EDGEZ_ROUTE_SEQUENCE_LEN;
static constexpr size_t EDGEZ_ROUTE_TO_OFFSET = EDGEZ_ROUTE_FROM_OFFSET + sizeof(uint64_t);
static constexpr size_t EDGEZ_ROUTE_MAX_HOP_OFFSET = EDGEZ_ROUTE_TO_OFFSET + sizeof(uint64_t);
static constexpr size_t EDGEZ_ROUTE_HOP_OFFSET = EDGEZ_ROUTE_MAX_HOP_OFFSET + 1;
static constexpr size_t EDGEZ_ROUTE_PREFIX_LEN = EDGEZ_ROUTE_HOP_OFFSET + 1;
static constexpr size_t EDGEZ_BEACON_VENDOR_PAYLOAD_MAX_LEN = 480;
static constexpr uint8_t EDGEZ_SPEED_RAW_MAGIC[] = {'E', 'Z', 'S', 'T'};

static char s_user_name[65] = "Edge Device";
uint32_t millis(void);
static void write_u64_be(uint8_t *out, uint64_t value);
static void write_u32_be(uint8_t *out, uint32_t value);
static uint64_t mac_to_u64(const uint8_t mac[EDGEZ_ROUTE_MAC_LEN]);
static bool u64_to_mac(uint64_t value, uint8_t mac[EDGEZ_ROUTE_MAC_LEN]);
static bool get_self_mac(uint8_t mac[EDGEZ_ROUTE_MAC_LEN]);

static esp_err_t radio_error_to_esp_err(EdgezRadioError err)
{
    if (err == EDGEZ_RADIO_OK) {
        return ESP_OK;
    }
#if defined(CONFIG_BUILD_EDGEZ_FROM_SOURCE) && CONFIG_BUILD_EDGEZ_FROM_SOURCE
    /* Keep temporary Morse TX pressure distinct from a terminal send error.
     * The realtime worker can then wait for DMA/mmpkt capacity without
     * repeatedly retrying invalid or disabled sends. */
    if (err == EDGEZ_RADIO_RETRY) {
        return ESP_ERR_TIMEOUT;
    }
#endif
    if (err == EDGEZ_RADIO_DISABLED) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_FAIL;
}


static uint8_t s_user_public_key[32] = {0};
static size_t s_user_public_key_len = 0;
static uint64_t s_user_id_high = 0;
static uint64_t s_user_id_low = 0;

extern "C" esp_err_t halow_build_edgez_mesh_vendor_ie(
    const char *mesh_id,
    const char *country_code,
    const char *passphrase,
    uint8_t *out,
    size_t out_size,
    size_t *out_len);
extern "C" esp_err_t halow_notify_mesh_beacon_to_mobile(
    const struct mmwlan_scan_result *result,
    const char *expected_mesh_id,
    const char *passphrase);
extern "C" bool halow_edgez_mesh_peer_admission_allowed(
    const uint8_t *ies,
    size_t ies_len,
    uint32_t *device_type);

static void halow_event_task(void *arg)
{
    (void)arg;
    static edgez_halow_event_t event;
    UBaseType_t lowest_free_stack = HALOW_EVENT_TASK_STACK_SIZE;
    while (true) {
        if (!edgez_halow_event_receive(&event, portMAX_DELAY)) {
            continue;
        }
        switch (event.type) {
        case EDGEZ_HALOW_EVENT_RX_FRAME:
            (void)halow_sync_bridge_handle_rx_frame(
                event.data.rx_frame.header,
                event.data.rx_frame.header_len,
                event.data.rx_frame.payload,
                event.data.rx_frame.payload_len,
                event.data.rx_frame.remote_bssid,
                false);
            break;
        case EDGEZ_HALOW_EVENT_BEACON: {
            struct mmwlan_scan_result result = {};
            result.bssid = event.data.beacon.bssid;
            result.ies = event.data.beacon.ies;
            result.ies_len = event.data.beacon.ies_len;
            result.rssi = event.data.beacon.rssi_dbm;
            esp_err_t err = halow_notify_mesh_beacon_to_mobile(
                &result,
                event.data.beacon.mesh_id,
                event.data.beacon.passphrase);
            if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
                ESP_LOGW(TAG, "Queued HaLow beacon handling failed: %s",
                         esp_err_to_name(err));
            }
            break;
        }
        case EDGEZ_HALOW_EVENT_PEER_ADMISSION: {
            uint32_t device_type = 0;
            bool allow = halow_edgez_mesh_peer_admission_allowed(
                event.data.peer_admission.ies,
                event.data.peer_admission.ies_len,
                &device_type);
            (void)edgez_halow_event_respond_peer_admission(
                event.request_id, allow, device_type);
            break;
        }
        case EDGEZ_HALOW_EVENT_BATMAN_PEER:
            halow_sync_bridge_notify_batman_peer(
                event.data.batman_peer.originator,
                event.data.batman_peer.last_sender,
                event.data.batman_peer.rssi_dbm,
                event.data.batman_peer.sequence,
                event.data.batman_peer.direct);
            break;
        case EDGEZ_HALOW_EVENT_BATMAN_PAYLOAD:
            if (!openmanet_comms_handle_frame(
                    event.data.batman_payload.payload,
                    event.data.batman_payload.payload_len) &&
                !(event.data.batman_payload.payload_len >= 14 &&
                  halow_sync_bridge_handle_rx_frame(
                      event.data.batman_payload.payload,
                      14,
                      event.data.batman_payload.payload + 14,
                      event.data.batman_payload.payload_len - 14,
                      event.data.batman_payload.originator,
                      true))) {
                openmanet_alfred_handle_frame(
                    event.data.batman_payload.originator,
                    event.data.batman_payload.payload,
                    event.data.batman_payload.payload_len);
            }
            break;
        default:
            ESP_LOGW(TAG, "Unknown HaLow event type=%u", (unsigned)event.type);
            break;
        }
        UBaseType_t free_stack = uxTaskGetStackHighWaterMark(NULL);
        if (free_stack < lowest_free_stack) {
            lowest_free_stack = free_stack;
            if (free_stack < HALOW_EVENT_STACK_WARN_BYTES) {
                ESP_LOGW(TAG, "HaLow event task stack low-watermark=%u bytes",
                         (unsigned)free_stack);
            }
        }
    }
}

static size_t fill_discovery_payload(uint8_t *out, size_t out_len)
{
    if (!out || out_len == 0) {
        return 0;
    }

    ai_edgez_halow_Beacon discover = ai_edgez_halow_Beacon_init_zero;
    discover.user_id_high = s_user_id_high;
    discover.user_id_low = s_user_id_low;
    strlcpy(discover.user_name,
            s_user_name[0] ? s_user_name : "Edge Device",
            sizeof(discover.user_name));
    if (s_user_public_key_len > 0) {
        discover.user_public_key.size = (pb_size_t)s_user_public_key_len;
        memcpy(discover.user_public_key.bytes, s_user_public_key, discover.user_public_key.size);
    }

    uint8_t proto[ai_edgez_halow_Beacon_size] = {0};
    pb_ostream_t stream = pb_ostream_from_buffer(proto, sizeof(proto));
    if (!pb_encode(&stream, ai_edgez_halow_Beacon_fields, &discover)) {
        ESP_LOGW(TAG, "HaLow discovery Beacon encode failed: %s", PB_GET_ERROR(&stream));
        return 0;
    }

    size_t encoded_len = 0;
    if (mbedtls_base64_encode(out, out_len, &encoded_len, proto, stream.bytes_written) != 0) {
        ESP_LOGW(TAG, "HaLow discovery Beacon base64 encode failed proto_len=%u out_len=%u",
                 (unsigned)stream.bytes_written,
                 (unsigned)out_len);
        return 0;
    }

    return encoded_len;
}

static EdgezRadioError send_beacon_payload(const uint8_t *payload,
                                     size_t payload_len,
                                     uint64_t destination,
                                     uint8_t channel,
                                     uint32_t max_hop = 0,
                                     uint64_t message_id_high = 0,
                                     uint64_t message_id_low = 0,
                                     uint32_t sequence = 0,
                                     uint64_t next_hop = 0)
{
    (void)destination;
    (void)channel;

    if (!s_halow_started) {
        ESP_LOGW(TAG, "HaLow Beacon skipped; interface not started");
        return EDGEZ_RADIO_DISABLED;
    }
    if (!payload || payload_len == 0) {
        ESP_LOGW(TAG, "HaLow Beacon skipped; invalid payload_len=%u", (unsigned)payload_len);
        return EDGEZ_RADIO_ERROR;
    }

    const uint8_t *vendor_payload = payload;
    size_t vendor_payload_len = payload_len;
    uint8_t route_payload[EDGEZ_ROUTE_PREFIX_LEN + EDGEZ_BEACON_VENDOR_PAYLOAD_MAX_LEN] = {0};
    uint64_t from = 0;
    uint64_t to = (next_hop & 0xffffffffffffULL) ? (next_hop & 0xffffffffffffULL) : 0xffffffffffffULL;
    uint8_t dest_mac[EDGEZ_ROUTE_MAC_LEN] = {0};
    bool directed = u64_to_mac(to, dest_mac);

    if (max_hop > 0) {
        if (payload_len > sizeof(route_payload) - EDGEZ_ROUTE_PREFIX_LEN) {
            ESP_LOGW(TAG, "HaLow Beacon skipped; routed payload too large len=%u", (unsigned)payload_len);
            return EDGEZ_RADIO_ERROR;
        }
        if (message_id_high == 0 && message_id_low == 0) {
            ESP_LOGW(TAG, "HaLow Beacon skipped; routed beacon missing message_id");
            return EDGEZ_RADIO_ERROR;
        }

        uint8_t self_mac[EDGEZ_ROUTE_MAC_LEN] = {0};
        if (get_self_mac(self_mac)) {
            from = mac_to_u64(self_mac);
        }

        size_t off = 0;
        write_u64_be(&route_payload[off], message_id_high);
        off += sizeof(uint64_t);
        write_u64_be(&route_payload[off], message_id_low);
        off += sizeof(uint64_t);
        write_u32_be(&route_payload[off], sequence);
        off += sizeof(uint32_t);
        write_u64_be(&route_payload[off], from);
        off += sizeof(uint64_t);
        write_u64_be(&route_payload[off], to);
        off += sizeof(uint64_t);
        route_payload[off++] = (uint8_t)((max_hop > UINT8_MAX) ? UINT8_MAX : max_hop);
        route_payload[off++] = 0;
        memcpy(&route_payload[off], payload, payload_len);
        off += payload_len;
        vendor_payload = route_payload;
        vendor_payload_len = off;
    }

    ESP_LOGI(TAG,
             "HaLow Beacon vendor payload len=%u routed_len=%u max_hop=%lu hop=0 seq=%lu from=0x%012llx to=0x%012llx directed=%u message_id=%016llx-%016llx key_len=%u",
             (unsigned)payload_len,
             (unsigned)vendor_payload_len,
             (unsigned long)max_hop,
             (unsigned long)sequence,
             (unsigned long long)(from & 0xffffffffffffULL),
             (unsigned long long)to,
             directed ? 1 : 0,
             (unsigned long long)message_id_high,
             (unsigned long long)message_id_low,
             (unsigned)s_user_public_key_len);

    EdgezRadioError err = directed ? s_halow.sendVendorPayloadTo(dest_mac, vendor_payload, vendor_payload_len)
                             : s_halow.sendVendorPayload(vendor_payload, vendor_payload_len);
    if (err != EDGEZ_RADIO_OK) {
        ESP_LOGW(TAG, "HaLow Beacon send failed err=%d", err);
    }
    return err;
}


uint32_t millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}


static bool is_valid_country_code(const char *country)
{
    return country && isalpha((unsigned char)country[0]) &&
           isalpha((unsigned char)country[1]) && country[2] == '\0';
}

static void normalize_country_code(char out[3], const char *country)
{
    out[0] = (char)toupper((unsigned char)country[0]);
    out[1] = (char)toupper((unsigned char)country[1]);
    out[2] = '\0';
}

static uint32_t node_num_from_user_id(uint64_t user_id_high, uint64_t user_id_low)
{
    uint32_t nodeNum = (uint32_t)user_id_low;
    if (nodeNum == 0 && user_id_low != 0) {
        nodeNum = (uint32_t)(user_id_low >> 32);
    }
    if (nodeNum == 0 && user_id_high != 0) {
        nodeNum = (uint32_t)user_id_high;
    }
    if (nodeNum == 0 && user_id_high != 0) {
        nodeNum = (uint32_t)(user_id_high >> 32);
    }
    if (nodeNum == 0 && (user_id_high != 0 || user_id_low != 0)) {
        nodeNum = 1;
    }
    return nodeNum;
}

static void write_u64_be(uint8_t *out, uint64_t value)
{
    for (size_t i = 0; i < sizeof(uint64_t); ++i) {
        out[i] = (uint8_t)(value >> ((7U - i) * 8U));
    }
}

static void write_u32_be(uint8_t *out, uint32_t value)
{
    for (size_t i = 0; i < sizeof(uint32_t); ++i) {
        out[i] = (uint8_t)(value >> ((3U - i) * 8U));
    }
}

static uint64_t mac_to_u64(const uint8_t mac[EDGEZ_ROUTE_MAC_LEN])
{
    if (!mac) {
        return 0;
    }
    uint64_t value = 0;
    for (size_t i = 0; i < EDGEZ_ROUTE_MAC_LEN; ++i) {
        value = (value << 8U) | mac[i];
    }
    return value;
}

static bool u64_to_mac(uint64_t value, uint8_t mac[EDGEZ_ROUTE_MAC_LEN])
{
    if (!mac) {
        return false;
    }

    uint64_t masked = value & 0xffffffffffffULL;
    mac[0] = (uint8_t)((masked >> 40) & 0xffU);
    mac[1] = (uint8_t)((masked >> 32) & 0xffU);
    mac[2] = (uint8_t)((masked >> 24) & 0xffU);
    mac[3] = (uint8_t)((masked >> 16) & 0xffU);
    mac[4] = (uint8_t)((masked >> 8) & 0xffU);
    mac[5] = (uint8_t)(masked & 0xffU);
    return masked != 0 && masked != 0xffffffffffffULL;
}

static bool get_self_mac(uint8_t mac[EDGEZ_ROUTE_MAC_LEN])
{
    if (!mac) {
        return false;
    }

    memset(mac, 0, EDGEZ_ROUTE_MAC_LEN);
    if (mmwlan_get_mac_addr(mac) == MMWLAN_SUCCESS) {
        return true;
    }
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK ||
        esp_read_mac(mac, ESP_MAC_BT) == ESP_OK ||
        esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK) {
        return true;
    }
    return false;
}

static void set_identity(const halow_interface_init_config_t *init)
{
    uint32_t nodeNum = node_num_from_user_id(init->user_id_high, init->user_id_low);
    const char *nodeSource = (init->user_id_high || init->user_id_low) ? "init" : "unset";
    if (nodeNum == 0) {
        uint8_t mac[6] = {0};
        if (mmwlan_get_mac_addr(mac) == MMWLAN_SUCCESS) {
            nodeNum = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) |
                      ((uint32_t)mac[4] << 8) | mac[5];
            nodeSource = "morse_mac";
        } else if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK ||
                   esp_read_mac(mac, ESP_MAC_BT) == ESP_OK ||
                   esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK) {
            nodeNum = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) |
                      ((uint32_t)mac[4] << 8) | mac[5];
            nodeSource = "esp_mac";
        } else {
            nodeNum = esp_random();
            nodeSource = "random";
        }
    }
    ESP_LOGI(TAG, "HaLow identity node=0x%08lx source=%s user_id=%016llx-%016llx key_len=%u",
             (unsigned long)nodeNum,
             nodeSource,
             (unsigned long long)init->user_id_high,
             (unsigned long long)init->user_id_low,
             (unsigned)init->user_public_key_len);

    s_user_id_high = init->user_id_high;
    s_user_id_low = init->user_id_low;
    s_user_public_key_len = init->user_public_key_len;
    if (s_user_public_key_len > sizeof(s_user_public_key)) {
        s_user_public_key_len = sizeof(s_user_public_key);
    }
    memset(s_user_public_key, 0, sizeof(s_user_public_key));
    if (s_user_public_key_len > 0) {
        memcpy(s_user_public_key, init->user_public_key, s_user_public_key_len);
    }

    strlcpy(s_user_name,
            init->user_name[0] ? init->user_name :
            (CONFIG_EDGEZ_HALOW_DISCOVERY_NAME[0] ? CONFIG_EDGEZ_HALOW_DISCOVERY_NAME : "EdgeZ HaLow"),
            sizeof(s_user_name));
    openmanet_alfred_set_hostname(s_user_name);
    ESP_LOGI(TAG, "HaLow discovery identity name='%s'", s_user_name);
}

extern "C" esp_err_t halow_interface_app_set_user_identity(uint64_t user_id_high,
                                                           uint64_t user_id_low,
                                                           const char *user_name,
                                                           const uint8_t *user_public_key,
                                                           size_t user_public_key_len)
{
    halow_interface_init_config_t init = {};
    init.user_id_high = user_id_high;
    init.user_id_low = user_id_low;
    if (user_name) {
        strlcpy(init.user_name, user_name, sizeof(init.user_name));
    }
    if (user_public_key && user_public_key_len > 0) {
        init.user_public_key_len = user_public_key_len;
        if (init.user_public_key_len > sizeof(init.user_public_key)) {
            init.user_public_key_len = sizeof(init.user_public_key);
        }
        memcpy(init.user_public_key, user_public_key, init.user_public_key_len);
    }
    set_identity(&init);
    return ESP_OK;
}

extern "C" esp_err_t halow_interface_app_init(void)
{
    if (!factory_data_license_authorize(EDGEZ_LICENSE_CAP_RADIO_INIT)) {
        ESP_LOGW(TAG, "HaLowInterface initialization blocked: device is not licensed");
        return ESP_ERR_NOT_ALLOWED;
    }

    if (s_adapter_initialized) {
        ESP_LOGI(TAG, "HaLowInterface adapter already initialized; Morse stack started=%d", s_halow_started);
        return ESP_OK;
    }

    esp_err_t queue_err = edgez_halow_event_queue_init();
    if (queue_err != ESP_OK) {
        ESP_LOGE(TAG, "HaLow application event queue init failed: %s",
                 esp_err_to_name(queue_err));
        return queue_err;
    }
    if (!s_halow_event_task) {
        BaseType_t created = xTaskCreate(halow_event_task,
                                         "halow_events",
                                         HALOW_EVENT_TASK_STACK_SIZE,
                                         NULL,
                                         HALOW_EVENT_TASK_PRIORITY,
                                         &s_halow_event_task);
        if (created != pdPASS) {
            s_halow_event_task = nullptr;
            return ESP_ERR_NO_MEM;
        }
    }

    s_lifecycle_mutex = xSemaphoreCreateMutex();
    if (!s_lifecycle_mutex) {
        return ESP_ERR_NO_MEM;
    }
    s_ready_events = xEventGroupCreate();
    if (!s_ready_events) {
        vSemaphoreDelete(s_lifecycle_mutex);
        s_lifecycle_mutex = nullptr;
        return ESP_ERR_NO_MEM;
    }

    s_adapter_initialized = true;
    ESP_LOGI(TAG, "HaLowInterface adapter initialized; Morse stack will start on credentials");
    return ESP_OK;
}

extern "C" esp_err_t halow_interface_app_start(const halow_interface_init_config_t *init)
{
    if (!factory_data_license_authorize(EDGEZ_LICENSE_CAP_RADIO_INIT)) {
        return ESP_ERR_NOT_ALLOWED;
    }

    if (!init || !is_valid_country_code(init->country_code) || init->mesh_id[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t initErr = halow_interface_app_init();
    if (initErr != ESP_OK) {
        return initErr;
    }
    if (xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    char requested_country[3] = {0};
    normalize_country_code(requested_country, init->country_code);
    bool proactive_join = init->device_type == 2U || init->device_type == 6U;
    bool relay_mode = init->device_type == 6U;
    bool same_profile = s_halow_started &&
                        strcmp(s_country_code, requested_country) == 0 &&
                        strcmp(s_mesh_id, init->mesh_id) == 0 &&
                        strcmp(s_passphrase, init->passphrase) == 0 &&
                        s_mesh_frequency_khz == init->mesh_frequency_khz &&
                        s_mesh_bandwidth_mhz == init->mesh_bandwidth_mhz &&
                        s_beacon_only == init->beacon_only;

    if (same_profile) {
        s_halow.setProactiveJoinEnabled(proactive_join);
        s_halow.setRelayModeEnabled(relay_mode);
        set_identity(init);
        xEventGroupSetBits(s_ready_events, HALOW_ADAPTER_READY_BIT);
        ESP_LOGI(TAG,
                 "HaLowInterface already running country=%s mesh_id=%s; refreshed identity only",
                 s_country_code,
                 s_mesh_id);
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_OK;
    }

    strlcpy(s_country_code, requested_country, sizeof(s_country_code));
    strlcpy(s_mesh_id, init->mesh_id, sizeof(s_mesh_id));
    strlcpy(s_passphrase, init->passphrase, sizeof(s_passphrase));
    s_mesh_frequency_khz = init->mesh_frequency_khz;
    s_mesh_bandwidth_mhz = init->mesh_bandwidth_mhz;
    s_beacon_only = init->beacon_only;
    s_halow.setCountryCode(s_country_code);
    s_halow.setMeshId(s_mesh_id);
    s_halow.setMeshSaePassphrase(s_passphrase);
    s_halow.setMeshRadio(init->mesh_frequency_khz, init->mesh_bandwidth_mhz);
    s_halow.setBeaconOnly(s_beacon_only);
    s_halow.setProactiveJoinEnabled(proactive_join);
    s_halow.setRelayModeEnabled(relay_mode);
    set_identity(init);

    uint8_t vendor_ies[257] = {0};
    size_t vendor_ies_len = 0;
    esp_err_t vendor_ie_err = halow_build_edgez_mesh_vendor_ie(
        s_mesh_id, s_country_code, s_passphrase,
        vendor_ies, sizeof(vendor_ies), &vendor_ies_len);
    if (vendor_ie_err != ESP_OK ||
        !s_halow.setMeshVendorIes(vendor_ies, vendor_ies_len)) {
        ESP_LOGE(TAG, "HaLow initial Vendor IE configuration failed: %s",
                 esp_err_to_name(vendor_ie_err));
        xSemaphoreGive(s_lifecycle_mutex);
        return vendor_ie_err != ESP_OK ? vendor_ie_err : ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG,
             "Starting HaLowInterface Morse stack country=%s mesh_id=%s reconfigure=%d beacon_only=%u",
             s_country_code,
             s_mesh_id,
             s_halow_started,
             s_beacon_only ? 1U : 0U);
    bool ok = s_halow_started ? s_halow.reconfigure() : s_halow.init();
    if (!ok) {
        xEventGroupClearBits(s_ready_events, HALOW_ADAPTER_READY_BIT);
        ESP_LOGE(TAG, "HaLowInterface Morse stack start failed");
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_FAIL;
    }

    s_halow_started = true;
    xEventGroupSetBits(s_ready_events, HALOW_ADAPTER_READY_BIT);
    ESP_LOGI(TAG, "HaLowInterface started country=%s mesh_id=%s", s_country_code, s_mesh_id);
    xSemaphoreGive(s_lifecycle_mutex);
    return ESP_OK;
}


extern "C" esp_err_t halow_interface_app_send_mesh_payload(const uint8_t *payload,
                                                           size_t payload_len,
                                                           uint64_t from,
                                                           uint64_t to,
                                                           uint64_t message_id_high,
                                                           uint64_t message_id_low,
                                                           uint32_t max_hop,
                                                           uint32_t sequence)
{
    return halow_interface_app_send_mesh_payload_via(payload,
                                                     payload_len,
                                                     from,
                                                     to,
                                                     to,
                                                     message_id_high,
                                                     message_id_low,
                                                     max_hop,
                                                     sequence);
}

extern "C" esp_err_t halow_interface_app_send_mesh_payload_via(const uint8_t *payload,
                                                               size_t payload_len,
                                                               uint64_t from,
                                                               uint64_t to,
                                                               uint64_t next_hop,
                                                               uint64_t message_id_high,
                                                               uint64_t message_id_low,
                                                               uint32_t max_hop,
                                                               uint32_t sequence)
{
    return halow_interface_app_forward_mesh_payload(payload,
                                                    payload_len,
                                                    from,
                                                    to,
                                                    next_hop,
                                                    message_id_high,
                                                    message_id_low,
                                                    max_hop,
                                                    sequence,
                                                    0);
}

static esp_err_t send_mesh_payload_internal(const uint8_t *payload,
                                            size_t payload_len,
                                            uint64_t from,
                                            uint64_t to,
                                            uint64_t next_hop,
                                            uint64_t message_id_high,
                                            uint64_t message_id_low,
                                            uint32_t max_hop,
                                            uint32_t sequence,
                                            uint32_t hop,
                                            uint16_t peer_independent_ethertype)
{
    if (!factory_data_license_authorize(EDGEZ_LICENSE_CAP_MESH_TX)) {
        return ESP_ERR_NOT_ALLOWED;
    }

    if (!payload || payload_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_halow_started) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Topology reports are already complete NetworkPacket protobuf messages.
     * Carry them as an Ethernet frame inside BATMAN broadcast so every mesh
     * originator participates in duplicate suppression and multi-hop fanout.
     * The management/device beacon remains a separate radio-native path. */
    if (peer_independent_ethertype == HALOW_SYNC_REPORT_ETHERTYPE) {
        constexpr size_t batmanPayloadCapacity =
            EDGEZ_BATADV_MAX_PACKET_LEN - EDGEZ_BATADV_BCAST_HEADER_LEN;
        uint8_t frame[batmanPayloadCapacity] = {0};
        uint8_t self_mac[EDGEZ_ROUTE_MAC_LEN] = {0};
        if (payload_len > sizeof(frame) - 14 || !get_self_mac(self_mac)) {
            return payload_len > sizeof(frame) - 14 ? ESP_ERR_INVALID_SIZE
                                                    : ESP_ERR_INVALID_STATE;
        }
        memset(frame, 0xff, EDGEZ_ROUTE_MAC_LEN);
        memcpy(frame + EDGEZ_ROUTE_MAC_LEN, self_mac, EDGEZ_ROUTE_MAC_LEN);
        frame[12] = (uint8_t)(HALOW_SYNC_REPORT_ETHERTYPE >> 8);
        frame[13] = (uint8_t)HALOW_SYNC_REPORT_ETHERTYPE;
        memcpy(frame + 14, payload, payload_len);
        MESH_DEBUG_LOGI(
            "HaLow topology report send BATMAN broadcast protobuf len=%u",
            (unsigned)payload_len);
        EdgezRadioError err = s_halow.sendBatmanBroadcastPayload(
            frame, 14 + payload_len);
        return radio_error_to_esp_err(err);
    }

    /* Routed application traffic is foreground work. Keep disruptive mesh
     * scans and periodic application broadcasts out of its active window. */
    s_halow.noteForegroundTraffic();

    uint8_t route_payload[EDGEZ_ROUTE_PREFIX_LEN + ai_edgez_halow_NetworkPacket_size] = {0};
    if (payload_len > sizeof(route_payload) - EDGEZ_ROUTE_PREFIX_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    size_t off = 0;
    write_u64_be(&route_payload[off], message_id_high);
    off += sizeof(uint64_t);
    write_u64_be(&route_payload[off], message_id_low);
    off += sizeof(uint64_t);
    write_u32_be(&route_payload[off], sequence);
    off += sizeof(uint32_t);
    write_u64_be(&route_payload[off], from);
    off += sizeof(uint64_t);
    write_u64_be(&route_payload[off], to);
    off += sizeof(uint64_t);
    route_payload[off++] = (uint8_t)((max_hop > UINT8_MAX) ? UINT8_MAX : max_hop);
    route_payload[off++] = (uint8_t)((hop > UINT8_MAX) ? UINT8_MAX : hop);
    memcpy(&route_payload[off], payload, payload_len);
    off += payload_len;

    const uint64_t target_mac = to & 0xffffffffffffULL;
    const bool batman_broadcast = halow_sync_is_public_channel(to) ||
                                  target_mac == 0 ||
                                  target_mac == 0xffffffffffffULL;
    if (batman_broadcast) {
        /* Public channels and generic application broadcasts use BATMAN's
         * broadcast bearer. Live PTT remains on the separate RTP/Opus
         * OpenMANET Comms path. */
        uint8_t self_mac[EDGEZ_ROUTE_MAC_LEN] = {0};
        constexpr size_t batmanPayloadCapacity =
            EDGEZ_BATADV_MAX_PACKET_LEN - EDGEZ_BATADV_BCAST_HEADER_LEN;
        uint8_t frame[batmanPayloadCapacity] = {0};
        if (off > sizeof(frame) - 14 || !get_self_mac(self_mac)) {
            return off > sizeof(frame) - 14 ? ESP_ERR_INVALID_SIZE
                                            : ESP_ERR_INVALID_STATE;
        }
        memset(frame, 0xff, EDGEZ_ROUTE_MAC_LEN);
        memcpy(frame + EDGEZ_ROUTE_MAC_LEN, self_mac, EDGEZ_ROUTE_MAC_LEN);
        frame[12] = (uint8_t)(HALOW_SYNC_ETHERTYPE >> 8);
        frame[13] = (uint8_t)HALOW_SYNC_ETHERTYPE;
        memcpy(frame + 14, route_payload, off);
        return halow_interface_app_send_batman_broadcast_payload(frame,
                                                                 14 + off);
    }

    uint8_t dest_mac[EDGEZ_ROUTE_MAC_LEN] = {0};
    uint8_t self_mac[EDGEZ_ROUTE_MAC_LEN] = {0};
    const bool directed = u64_to_mac(to, dest_mac);
    const bool have_self_mac = get_self_mac(self_mac);
    constexpr size_t batmanPayloadCapacity =
        EDGEZ_BATADV_MAX_PACKET_LEN - EDGEZ_BATADV_UNICAST_HEADER_LEN;
    uint8_t frame[batmanPayloadCapacity] = {0};
    if (!directed || !have_self_mac || off > sizeof(frame) - 14) {
        return !directed || !have_self_mac ? ESP_ERR_INVALID_STATE
                                           : ESP_ERR_INVALID_SIZE;
    }
    memcpy(frame, dest_mac, EDGEZ_ROUTE_MAC_LEN);
    memcpy(frame + EDGEZ_ROUTE_MAC_LEN, self_mac, EDGEZ_ROUTE_MAC_LEN);
    frame[12] = (uint8_t)(HALOW_SYNC_ETHERTYPE >> 8);
    frame[13] = (uint8_t)HALOW_SYNC_ETHERTYPE;
    memcpy(frame + 14, route_payload, off);

    /* Only speed-test mode 2 may override BATMAN's first RF hop. The BATMAN
     * unicast header and inner Ethernet DA still name the final destination,
     * so an ESP32 or Linux batman-adv waypoint forwards it natively. */
    uint8_t forced_next_hop_mac[EDGEZ_ROUTE_MAC_LEN] = {0};
    const bool speed_two_hop_waypoint =
        max_hop == 2U && hop == 0U && next_hop != target_mac &&
        payload_len >= sizeof(EDGEZ_SPEED_RAW_MAGIC) &&
        memcmp(payload, EDGEZ_SPEED_RAW_MAGIC,
               sizeof(EDGEZ_SPEED_RAW_MAGIC)) == 0 &&
        u64_to_mac(next_hop, forced_next_hop_mac);

    MESH_DEBUG_LOGI(
             "HaLow NetworkPacket BATMAN unicast message_id=%016llx-%016llx seq=%lu from=0x%012llx to=0x%012llx route_hint=0x%012llx forced_rf_next=%02x:%02x:%02x:%02x:%02x:%02x forced_waypoint=%u max_hop=%lu hop=%lu payload=%u route_payload=%u",
             (unsigned long long)message_id_high,
             (unsigned long long)message_id_low,
             (unsigned long)sequence,
             (unsigned long long)(from & 0xffffffffffffULL),
             (unsigned long long)(to & 0xffffffffffffULL),
             (unsigned long long)(next_hop & 0xffffffffffffULL),
             forced_next_hop_mac[0], forced_next_hop_mac[1],
             forced_next_hop_mac[2], forced_next_hop_mac[3],
             forced_next_hop_mac[4], forced_next_hop_mac[5],
             speed_two_hop_waypoint ? 1U : 0U,
             (unsigned long)max_hop,
             (unsigned long)hop,
             (unsigned)payload_len,
             (unsigned)off);

    /* BATMAN owns next-hop selection and forwarding. The final destination
     * remains in the inner Ethernet DA; sendBatmanPayloadTo resolves a TT
     * client to its originator and places that originator in the BATMAN
     * unicast header. */
    EdgezRadioError err = s_halow.sendBatmanPayloadTo(
        dest_mac, frame, 14 + off,
        speed_two_hop_waypoint ? forced_next_hop_mac : nullptr);
    return radio_error_to_esp_err(err);
}

extern "C" esp_err_t halow_interface_app_forward_mesh_payload(const uint8_t *payload,
                                                              size_t payload_len,
                                                              uint64_t from,
                                                              uint64_t to,
                                                              uint64_t next_hop,
                                                              uint64_t message_id_high,
                                                              uint64_t message_id_low,
                                                              uint32_t max_hop,
                                                              uint32_t sequence,
                                                              uint32_t hop)
{
    return send_mesh_payload_internal(payload,
                                      payload_len,
                                      from,
                                      to,
                                      next_hop,
                                      message_id_high,
                                      message_id_low,
                                      max_hop,
                                      sequence,
                                      hop,
                                      0);
}

extern "C" esp_err_t halow_interface_app_send_peer_independent_beacon(
    const uint8_t *payload,
    size_t payload_len,
    uint64_t from,
    uint64_t to,
    uint64_t message_id_high,
    uint64_t message_id_low,
    uint32_t max_hop,
    uint32_t sequence)
{
    return send_mesh_payload_internal(payload,
                                      payload_len,
                                      from,
                                      to,
                                      0,
                                      message_id_high,
                                      message_id_low,
                                      max_hop,
                                      sequence,
                                      0,
                                      HALOW_SYNC_REPORT_ETHERTYPE);
}

extern "C" bool halow_interface_app_get_self_mac(uint8_t mac[6])
{
    return get_self_mac(mac);
}

extern "C" bool halow_interface_app_lookup_batman_route(
    const uint8_t destination[6], uint8_t next_hop[6], uint8_t *hop_count,
    uint8_t *tq, uint32_t *route_age_ms)
{
    return s_halow_started &&
           s_halow.lookupBatmanRoute(destination, next_hop, hop_count, tq,
                                     route_age_ms);
}

extern "C" size_t halow_interface_app_get_batman_routes(
    edgez_platform_halow_route_t *routes, size_t capacity)
{
    if (!s_halow_started || !routes || capacity == 0) return 0;
    edgez_batadv_route_snapshot_t snapshot[EDGEZ_BATADV_MAX_ROUTES] = {};
    size_t requested = capacity < EDGEZ_BATADV_MAX_ROUTES
                           ? capacity
                           : EDGEZ_BATADV_MAX_ROUTES;
    size_t count = s_halow.snapshotBatmanRoutes(snapshot, requested);
    for (size_t i = 0; i < count; ++i) {
        memcpy(routes[i].originator, snapshot[i].originator, 6);
        memcpy(routes[i].next_hop, snapshot[i].next_hop, 6);
        routes[i].tq = snapshot[i].tq;
        routes[i].hops = snapshot[i].hops;
        routes[i].age_ms = snapshot[i].age_ms;
    }
    return count;
}

extern "C" bool halow_interface_app_select_batman_direct_peer(
    const uint8_t exclude_a[6], const uint8_t exclude_b[6],
    const uint8_t exclude_c[6],
    uint64_t selection_key, uint8_t peer[6])
{
    return s_halow_started &&
           s_halow.selectBatmanDirectPeer(exclude_a, exclude_b, exclude_c,
                                           selection_key, peer);
}

extern "C" void halow_interface_app_note_foreground_traffic(void)
{
    s_halow.noteForegroundTraffic();
}

extern "C" bool halow_interface_app_foreground_traffic_active(void)
{
    return s_halow.foregroundTrafficActive();
}

extern "C" esp_err_t halow_interface_app_send_batman_payload(
    const uint8_t destination[6], const uint8_t *payload, size_t payload_len)
{
    if (!s_halow_started || !destination || !payload || payload_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    s_halow.noteForegroundTraffic();
    EdgezRadioError err = s_halow.sendBatmanPayloadTo(destination, payload,
                                                       payload_len);
    return radio_error_to_esp_err(err);
}

extern "C" esp_err_t halow_interface_app_send_batman_broadcast_payload(
    const uint8_t *payload, size_t payload_len)
{
    if (!s_halow_started || !payload || payload_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
#if defined(CONFIG_BUILD_EDGEZ_FROM_SOURCE) && CONFIG_BUILD_EDGEZ_FROM_SOURCE
    s_halow.noteForegroundTraffic();
    EdgezRadioError err = s_halow.sendBatmanBroadcastPayload(payload,
                                                              payload_len);
    return radio_error_to_esp_err(err);
#else
    ESP_LOGW(TAG, "OpenMANET Comms requires the source EdgeZ mesh component");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

extern "C" bool halow_interface_app_handle_batman_rx(const uint8_t *ethernet_frame,
                                                       size_t frame_len,
                                                       const uint8_t transmitter[6])
{
#if defined(CONFIG_MM_BATMAN_ADV_LITE) && CONFIG_MM_BATMAN_ADV_LITE
    if (!s_halow_started || !ethernet_frame || frame_len < 14) {
        return false;
    }

    uint16_t ethertype = ((uint16_t)ethernet_frame[12] << 8) | ethernet_frame[13];
    if (ethertype != EDGEZ_BATADV_ETHERTYPE) {
        return false;
    }

    const uint8_t *ingress = transmitter ? transmitter : &ethernet_frame[6];
    ESP_LOGI("edgez_batman",
             "RX dispatch ingress=%02x:%02x:%02x:%02x:%02x:%02x ethernet_sa=%02x:%02x:%02x:%02x:%02x:%02x payload_len=%u",
             ingress[0], ingress[1], ingress[2], ingress[3], ingress[4],
             ingress[5], ethernet_frame[6], ethernet_frame[7],
             ethernet_frame[8], ethernet_frame[9], ethernet_frame[10],
             ethernet_frame[11],
             (unsigned)(frame_len - 14));
    s_halow.receiveBatmanAdv(ingress, ethernet_frame + 14, frame_len - 14);
    return true;
#else
    (void)ethernet_frame;
    (void)frame_len;
    (void)transmitter;
    return false;
#endif
}

extern "C" esp_err_t halow_interface_app_request_beacon(const char *beacon,
                                                        uint32_t max_hop,
                                                        uint64_t message_id_high,
                                                        uint64_t message_id_low,
                                                        uint32_t sequence,
                                                        uint64_t next_hop)
{
    if (!s_halow_started) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!beacon || beacon[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    size_t beacon_len = strnlen(beacon, EDGEZ_BEACON_VENDOR_PAYLOAD_MAX_LEN + 1U);
    if (beacon_len == 0 || beacon_len > EDGEZ_BEACON_VENDOR_PAYLOAD_MAX_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    EdgezRadioError err = send_beacon_payload((const uint8_t *)beacon,
                                        beacon_len,
                                        UINT64_MAX,
                                        0,
                                        max_hop,
                                        message_id_high,
                                        message_id_low,
                                        sequence,
                                        next_hop);
    return (err == EDGEZ_RADIO_OK) ? ESP_OK : ESP_FAIL;
}

extern "C" void halow_interface_app_get_status(wifi_prov_halow_status_t *out_status)
{
    if (!out_status) {
        return;
    }

    memset(out_status, 0, sizeof(*out_status));
    out_status->supported = true;
    out_status->stack_initialized = s_halow_started;
    out_status->mesh_mode = s_halow_started;
    out_status->ready_for_report = halow_interface_app_ready();
    out_status->link_up = out_status->ready_for_report;
    out_status->route_ready = out_status->ready_for_report;
    strlcpy(out_status->mesh_id, s_mesh_id, sizeof(out_status->mesh_id));
}

extern "C" bool halow_interface_app_ready(void)
{
    if (!s_ready_events) {
        return false;
    }
    return (xEventGroupGetBits(s_ready_events) & HALOW_ADAPTER_READY_BIT) != 0;
}

extern "C" bool halow_interface_app_mesh_initialized(void)
{
    return s_halow_started && mmwlan_mesh_is_initialized();
}

extern "C" bool halow_interface_app_beacon_only(void)
{
    return s_beacon_only;
}

extern "C" esp_err_t halow_interface_app_wait_ready(uint32_t timeout_ms)
{
    if (!s_ready_events) {
        return ESP_ERR_INVALID_STATE;
    }
    EventBits_t bits = xEventGroupWaitBits(s_ready_events,
                                           HALOW_ADAPTER_READY_BIT,
                                           false,
                                           true,
                                           pdMS_TO_TICKS(timeout_ms));
    return (bits & HALOW_ADAPTER_READY_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

extern "C" esp_err_t halow_interface_app_shutdown(bool hold_reset_n)
{
    if (s_lifecycle_mutex &&
        xSemaphoreTake(s_lifecycle_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (hold_reset_n) {
        /* Match the proven main-branch deep-sleep path: do not start an
         * asynchronous Morse STA/supplicant teardown immediately before the
         * MCU sleeps. Hard-reset the transceiver and hold RESET_N low across
         * deep sleep instead. */
        if (s_ready_events) {
            xEventGroupClearBits(s_ready_events, HALOW_ADAPTER_READY_BIT);
        }
        gpio_set_level((gpio_num_t)CONFIG_MM_RESET_N, 0);
        gpio_hold_en((gpio_num_t)CONFIG_MM_RESET_N);
        gpio_deep_sleep_hold_en();
        s_halow_started = false;
        ESP_LOGI(TAG,
                 "HaLow transceiver hard-powered down for deep sleep, GPIO %d held LOW",
                 CONFIG_MM_RESET_N);
        if (s_lifecycle_mutex) {
            xSemaphoreGive(s_lifecycle_mutex);
        }
        return ESP_OK;
    }

    bool ok = true;
    if (s_halow_started) {
        ok = s_halow.shutdown();
        s_halow_started = false;
    }
    if (s_ready_events) {
        xEventGroupClearBits(s_ready_events, HALOW_ADAPTER_READY_BIT);
    }

    if (s_lifecycle_mutex) {
        xSemaphoreGive(s_lifecycle_mutex);
    }
    return ok ? ESP_OK : ESP_FAIL;
}

extern "C" edgez_transport_type_t halow_interface_app_connection_type(void)
{
    return s_halow_started ? EDGEZ_TRANSPORT_HALOW : EDGEZ_TRANSPORT_WIFI;
}
