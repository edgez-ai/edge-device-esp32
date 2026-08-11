/**
 * @file prov_esp32.c
 * @brief ESP32-specific Wi-Fi/Network connection implementation
 * This implementation is specific to ESP32 series chips.
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <esp_event.h>
#include <esp_app_desc.h>
#include <esp_netif.h>
#include <esp_system.h>
#include <esp_random.h>
#include <driver/uart.h>
#include <sdkconfig.h>
#include <mbedtls/aes.h>
#include <mbedtls/base64.h>
#include <mbedtls/sha256.h>
#include <mbedtls/version.h>

#include "prov.h"
#include "factory_data.h"
#include "halow_interface_app.h"
#include "halow_sync_bridge.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "usb_control.pb.h"

#ifdef CONFIG_ENABLE_MM_HALOW
#include "mmhal.h"
#include "mmwlan.h"
#include "mmipal.h"
#include "mmregdb.h"
#include <lwip/netif.h>
#include <lwip/dhcp.h>
#include <lwip/ip4_addr.h>
#include <lwip/etharp.h>
#include <lwip/pbuf.h>
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <lwip/tcpip.h>
#include <driver/gpio.h>
#endif

static const char *TAG = "network";

#ifdef CONFIG_MM_MESH_DEBUG_LOG
#define MM_MESH_DBG_PRINTF(...) printf(__VA_ARGS__)
#else
#define MM_MESH_DBG_PRINTF(...) do {} while (0)
#endif

/* Global flag to track if lwIP was already initialized by esp_netif_init() */
bool g_lwip_initialized_by_esp_netif = false;

static char s_halow_country_code[3] = CONFIG_MM_HALOW_COUNTRY_CODE;

#ifdef CONFIG_ENABLE_MM_HALOW
static bool halow_country_code_is_configured(void)
{
    return (s_halow_country_code[0] != '\0' && s_halow_country_code[1] != '\0');
}
#endif

/* Track active connection type (default Wi-Fi) */
static edgez_transport_type_t s_connection_type = EDGEZ_TRANSPORT_WIFI;

/* Signal Wi-Fi events on this event-group */
static EventGroupHandle_t wifi_event_group;
static bool s_prov_event_handlers_registered = false;
static bool s_wifi_provisioned_cached = false;
static bool s_startup_halow_mesh_mode = false;
static uint16_t s_startup_halow_mesh_channel = 0;
static uint8_t s_startup_halow_mesh_bandwidth_mhz = 0;
static uint32_t s_runtime_halow_mesh_frequency_khz = 0;
static uint8_t s_runtime_halow_mesh_bandwidth_mhz = 0;
static bool s_wifi_connect_inflight = false;
static esp_netif_t *s_upstream_wifi_netif = NULL;
static bool s_upstream_wifi_driver_initialized = false;
static bool s_upstream_wifi_started = false;
#ifdef CONFIG_ENABLE_MM_HALOW
typedef struct {
    uint64_t user_id_high;
    uint64_t user_id_low;
    char user_name[65];
    uint8_t user_public_key[32];
    size_t user_public_key_len;
    ai_edgez_halow_MarkerColor marker;
    ai_edgez_halow_DeviceType device_type;
    bool has_location;
    float latitude;
    float longitude;
    pb_size_t sensor_data_count;
    ai_edgez_halow_SensorData sensor_data[7];
} halow_user_identity_t;

static volatile bool s_power_transition_in_progress = false;
static bool s_active_halow_credentials_valid = false;
static char s_active_halow_ssid[65] = {0};
static char s_active_halow_password[65] = {0};
static halow_user_identity_t s_halow_user_identity = {0};
static uint8_t s_halow_max_hop = 0;
#endif

static esp_err_t request_wifi_sta_connect(const char *reason)
{
    const char *src = reason ? reason : "unspecified";

    if (s_wifi_connect_inflight) {
        ESP_LOGI(TAG, "Skipping duplicate Wi-Fi connect request (%s)", src);
        return ESP_OK;
    }

    esp_err_t err = esp_wifi_connect();
    if (err == ESP_OK) {
        s_wifi_connect_inflight = true;
        return ESP_OK;
    }

    if (err == ESP_ERR_WIFI_CONN) {
        /* Driver reports connection already in progress. Treat as in-flight. */
        s_wifi_connect_inflight = true;
        ESP_LOGI(TAG, "Wi-Fi connect already in progress (%s)", src);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "esp_wifi_connect failed (%s): %s", src, esp_err_to_name(err));
    return err;
}

esp_err_t wifi_prov_set_halow_country_code(const char *country_code)
{
#ifdef CONFIG_ENABLE_MM_HALOW
    if (!country_code || country_code[0] == '\0' || country_code[1] == '\0' || country_code[2] != '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    char normalized[3] = {
        (char)toupper((unsigned char)country_code[0]),
        (char)toupper((unsigned char)country_code[1]),
        '\0'
    };

    memcpy(s_halow_country_code, normalized, sizeof(s_halow_country_code));
    ESP_LOGI(TAG, "Runtime HaLow country override set to %s", s_halow_country_code);
    return ESP_OK;
#else
    (void)country_code;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t wifi_prov_set_halow_user_identity(uint64_t user_id_high,
                                            uint64_t user_id_low,
                                            const char *user_name,
                                            const uint8_t *user_public_key,
                                            size_t user_public_key_len)
{
#ifdef CONFIG_ENABLE_MM_HALOW
    if (user_public_key_len > sizeof(s_halow_user_identity.user_public_key) ||
        (!user_public_key && user_public_key_len > 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_halow_user_identity.user_id_high = user_id_high;
    s_halow_user_identity.user_id_low = user_id_low;
    strlcpy(s_halow_user_identity.user_name,
            user_name ? user_name : "",
            sizeof(s_halow_user_identity.user_name));
    memset(s_halow_user_identity.user_public_key, 0, sizeof(s_halow_user_identity.user_public_key));
    s_halow_user_identity.user_public_key_len = user_public_key_len;
    if (user_public_key_len > 0) {
        memcpy(s_halow_user_identity.user_public_key, user_public_key, user_public_key_len);
    }
    return ESP_OK;
#else
    (void)user_id_high;
    (void)user_id_low;
    (void)user_name;
    (void)user_public_key;
    (void)user_public_key_len;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t wifi_prov_set_halow_beacon_profile(uint32_t marker,
                                             bool has_location,
                                             float latitude,
                                             float longitude)
{
#ifdef CONFIG_ENABLE_MM_HALOW
    s_halow_user_identity.marker = (ai_edgez_halow_MarkerColor)marker;
    s_halow_user_identity.has_location = has_location;
    s_halow_user_identity.latitude = has_location ? latitude : 0.0f;
    s_halow_user_identity.longitude = has_location ? longitude : 0.0f;
    return ESP_OK;
#else
    (void)marker;
    (void)has_location;
    (void)latitude;
    (void)longitude;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t wifi_prov_set_halow_beacon_sensor_data(
    const ai_edgez_halow_SensorData *sensor_data,
    size_t sensor_data_count)
{
#ifdef CONFIG_ENABLE_MM_HALOW
    if ((!sensor_data && sensor_data_count > 0) ||
        sensor_data_count > (sizeof(s_halow_user_identity.sensor_data) /
                             sizeof(s_halow_user_identity.sensor_data[0]))) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(s_halow_user_identity.sensor_data,
           0,
           sizeof(s_halow_user_identity.sensor_data));
    s_halow_user_identity.sensor_data_count = (pb_size_t)sensor_data_count;
    if (sensor_data_count > 0) {
        memcpy(s_halow_user_identity.sensor_data,
               sensor_data,
               sensor_data_count * sizeof(sensor_data[0]));
    }
    return ESP_OK;
#else
    (void)sensor_data;
    (void)sensor_data_count;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t wifi_prov_set_halow_beacon_device_type(uint32_t device_type)
{
#ifdef CONFIG_ENABLE_MM_HALOW
    s_halow_user_identity.device_type = (ai_edgez_halow_DeviceType)device_type;
    return ESP_OK;
#else
    (void)device_type;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t wifi_prov_set_halow_max_hop(uint32_t max_hop)
{
#ifdef CONFIG_ENABLE_MM_HALOW
    s_halow_max_hop = (max_hop > UINT8_MAX) ? UINT8_MAX : (uint8_t)max_hop;
    return ESP_OK;
#else
    (void)max_hop;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t wifi_prov_set_startup_halow_mesh_profile(bool mesh_mode,
                                                   uint16_t channel,
                                                   uint8_t bandwidth_mhz)
{
#ifdef CONFIG_ENABLE_MM_HALOW
    if (!mesh_mode) {
        s_startup_halow_mesh_mode = false;
        s_startup_halow_mesh_channel = 0;
        s_startup_halow_mesh_bandwidth_mhz = 0;
        ESP_LOGI(TAG, "Startup HaLow mesh profile disabled");
        return ESP_OK;
    }

    if (channel == 0 || bandwidth_mhz == 0) {
        s_startup_halow_mesh_mode = true;
        s_startup_halow_mesh_channel = 0;
        s_startup_halow_mesh_bandwidth_mhz = 0;
        ESP_LOGI(TAG, "Startup HaLow mesh profile enabled (no fixed channel hint)");
        return ESP_OK;
    }

    if (!(bandwidth_mhz == 1 || bandwidth_mhz == 2 || bandwidth_mhz == 4 || bandwidth_mhz == 8)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_startup_halow_mesh_mode = true;
    s_startup_halow_mesh_channel = channel;
    s_startup_halow_mesh_bandwidth_mhz = bandwidth_mhz;
    ESP_LOGI(TAG,
             "Startup HaLow mesh profile enabled (channel=%u bw=%uMHz)",
             (unsigned)s_startup_halow_mesh_channel,
             (unsigned)s_startup_halow_mesh_bandwidth_mhz);
    return ESP_OK;
#else
    (void)mesh_mode;
    (void)channel;
    (void)bandwidth_mhz;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

#ifdef CONFIG_ENABLE_MM_HALOW
#define HALOW_HINT_MAX_CHANNELS 64

#ifndef HALOW_HINT_KEEP_PERCENT
#ifdef CONFIG_HALOW_HINT_KEEP_PERCENT
#define HALOW_HINT_KEEP_PERCENT CONFIG_HALOW_HINT_KEEP_PERCENT
#else
#define HALOW_HINT_KEEP_PERCENT 100
#endif
#endif

#ifndef HALOW_VERBOSE_CHANNEL_LOG
#ifdef CONFIG_HALOW_VERBOSE_CHANNEL_LOG
#define HALOW_VERBOSE_CHANNEL_LOG CONFIG_HALOW_VERBOSE_CHANNEL_LOG
#else
#define HALOW_VERBOSE_CHANNEL_LOG 0
#endif
#endif

#ifndef HALOW_HINT_USE_BSSID_LOCK
#ifdef CONFIG_HALOW_HINT_USE_BSSID_LOCK
#define HALOW_HINT_USE_BSSID_LOCK CONFIG_HALOW_HINT_USE_BSSID_LOCK
#else
#define HALOW_HINT_USE_BSSID_LOCK 0
#endif
#endif

#ifndef HALOW_HINT_STRICT_SINGLE_CHANNEL
#ifdef CONFIG_HALOW_HINT_STRICT_SINGLE_CHANNEL
#define HALOW_HINT_STRICT_SINGLE_CHANNEL CONFIG_HALOW_HINT_STRICT_SINGLE_CHANNEL
#else
#define HALOW_HINT_STRICT_SINGLE_CHANNEL 0
#endif
#endif

#ifndef HALOW_SLEEP_SHUTDOWN_TIMEOUT_MS
#define HALOW_SLEEP_SHUTDOWN_TIMEOUT_MS 3000
#endif

#ifndef HALOW_SLEEP_SHUTDOWN_POLL_MS
#define HALOW_SLEEP_SHUTDOWN_POLL_MS 20
#endif

#ifndef HALOW_WAKE_RECONNECT_READY_TIMEOUT_MS
#define HALOW_WAKE_RECONNECT_READY_TIMEOUT_MS 30000
#endif

#ifndef HALOW_WAKE_RECONNECT_MAX_ATTEMPTS
#define HALOW_WAKE_RECONNECT_MAX_ATTEMPTS 2
#endif

#ifndef HALOW_WAKE_STATIC_IP_FALLBACK_TIMEOUT_MS
#define HALOW_WAKE_STATIC_IP_FALLBACK_TIMEOUT_MS 5000
#endif

#ifndef HALOW_WAKE_SCAN_NO_AUTH_RETRY_THRESHOLD
#define HALOW_WAKE_SCAN_NO_AUTH_RETRY_THRESHOLD 3
#endif

#ifndef HALOW_WAKE_ALLOW_WIDEBW_BSSID_LOCK
#define HALOW_WAKE_ALLOW_WIDEBW_BSSID_LOCK 0
#endif

#ifndef HALOW_WAKE_HINT_REGION_HALF_SPAN_MHZ
#define HALOW_WAKE_HINT_REGION_HALF_SPAN_MHZ 4
#endif

#ifndef HALOW_MESH_CONNECT_SCAN_DWELL_MS
#ifdef CONFIG_MM_EXPERIMENTAL_MESH_SCAN_DWELL_MS
#define HALOW_MESH_CONNECT_SCAN_DWELL_MS CONFIG_MM_EXPERIMENTAL_MESH_SCAN_DWELL_MS
#else
#define HALOW_MESH_CONNECT_SCAN_DWELL_MS 60
#endif
#endif

#ifndef HALOW_MESH_SCAN_INTERVAL_BASE_S
#ifdef CONFIG_HALOW_MESH_SCAN_INTERVAL_BASE_S
#define HALOW_MESH_SCAN_INTERVAL_BASE_S CONFIG_HALOW_MESH_SCAN_INTERVAL_BASE_S
#elif defined(CONFIG_MM_EXPERIMENTAL_MESH_SCAN_INTERVAL_BASE_S)
#define HALOW_MESH_SCAN_INTERVAL_BASE_S CONFIG_MM_EXPERIMENTAL_MESH_SCAN_INTERVAL_BASE_S
#else
#define HALOW_MESH_SCAN_INTERVAL_BASE_S 1
#endif
#endif

#ifndef HALOW_MESH_SCAN_INTERVAL_LIMIT_S
#ifdef CONFIG_HALOW_MESH_SCAN_INTERVAL_LIMIT_S
#define HALOW_MESH_SCAN_INTERVAL_LIMIT_S CONFIG_HALOW_MESH_SCAN_INTERVAL_LIMIT_S
#elif defined(CONFIG_MM_EXPERIMENTAL_MESH_SCAN_INTERVAL_LIMIT_S)
#define HALOW_MESH_SCAN_INTERVAL_LIMIT_S CONFIG_MM_EXPERIMENTAL_MESH_SCAN_INTERVAL_LIMIT_S
#else
#define HALOW_MESH_SCAN_INTERVAL_LIMIT_S 8
#endif
#endif

#ifndef HALOW_MESH_SCAN_HOME_DWELL_MS
#ifdef CONFIG_MM_EXPERIMENTAL_MESH_SCAN_HOME_DWELL_MS
#define HALOW_MESH_SCAN_HOME_DWELL_MS CONFIG_MM_EXPERIMENTAL_MESH_SCAN_HOME_DWELL_MS
#else
#define HALOW_MESH_SCAN_HOME_DWELL_MS 0
#endif
#endif

#ifndef HALOW_WAKE_HINT_MAX_CHANNELS
#define HALOW_WAKE_HINT_MAX_CHANNELS 12
#endif

#ifndef HALOW_LIGHT_SLEEP_POST_IP_GUARD_MS
#ifdef CONFIG_HALOW_LIGHT_SLEEP_POST_IP_GUARD_MS
#define HALOW_LIGHT_SLEEP_POST_IP_GUARD_MS CONFIG_HALOW_LIGHT_SLEEP_POST_IP_GUARD_MS
#else
#define HALOW_LIGHT_SLEEP_POST_IP_GUARD_MS 3000
#endif
#endif

#ifndef HALOW_LIGHT_SLEEP_POST_IP_GUARD_STATIC_MS
#define HALOW_LIGHT_SLEEP_POST_IP_GUARD_STATIC_MS 1000
#endif

#ifndef HALOW_WAKE_READY_POLL_STEP_MS
#define HALOW_WAKE_READY_POLL_STEP_MS 100
#endif

/* Forward declarations for HaLow functions and variables used in event_handler */
static bool halow_sta_connected = false;
static bool s_halow_light_sleep_active = false;
static bool s_halow_light_sleep_shutdown = false;
static bool s_halow_stack_initialized = false;
static bool s_mmipal_initialized = false;
static bool is_halow_ssid(const char *ssid);
static esp_err_t halow_sta_connect(const char *ssid, const char *password);
static void halow_log_tx_power_stats(void);
static void halow_stats_timer_callback(TimerHandle_t xTimer);
static void halow_stats_task(void *arg);
static void halow_reset_provisioning_state(void);  // Reset state for new provisioning attempt
static void halow_refresh_ip_network_after_wake(void);
static void halow_register_link_status_callbacks(void);
static void halow_ip_poll_task(void *arg);
static void halow_rx_sync_task(void *arg);
static void halow_start_rx_sync_task_if_needed(const char *reason);
static esp_err_t halow_set_full_regulatory_channel_list(void);
static bool halow_set_hint_channel_list_from_freq(uint32_t freq_hz, uint8_t bw_mhz);
static void halow_log_channel_list(const char *label, const struct mmwlan_s1g_channel_list *channel_list);
static void halow_sta_event_callback(const struct mmwlan_sta_event_cb_args *sta_event, void *arg);
static void halow_sta_status_callback(enum mmwlan_sta_state sta_state);
static void halow_mmwlan_link_state_callback(enum mmwlan_link_state link_state, void *arg);
static void halow_mmwlan_rx_to_lwip(struct mmpkt *rxpkt,
                                    const struct mmwlan_rx_metadata *metadata,
                                    void *arg);
static void halow_bssid_unlock_retry_task(void *arg);
static void halow_seed_gateway_arp_task(void *arg);
static void halow_scan_fallback_retry_task(void *arg);
static struct netif *halow_find_mmnetif(void);
static void halow_start_ip_poll_task_if_needed(const char *reason);
static void halow_stop_ip_poll_task_if_running(const char *reason);

/* Deferred HaLow connection worker - must not run inside event handler */
typedef struct {
    char ssid[65];
    char password[65];
} halow_connect_request_t;
static QueueHandle_t s_halow_connect_queue = NULL;
static TaskHandle_t s_halow_connect_task_handle = NULL;
static void halow_connect_worker_task(void *arg);
static esp_err_t halow_connect_worker_init(void);
static void halow_shutdown_for_power_transition(bool hold_reset_n, const char *reason);

static bool halow_wait_for_sta_disabled(uint32_t timeout_ms)
{
    if (mmwlan_get_sta_state() == MMWLAN_STA_DISABLED) {
        return true;
    }

    if (timeout_ms == 0) {
        return false;
    }

    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    TickType_t poll_ticks = pdMS_TO_TICKS(HALOW_SLEEP_SHUTDOWN_POLL_MS);
    if (poll_ticks == 0) {
        poll_ticks = 1;
    }

    while ((xTaskGetTickCount() - start) < timeout_ticks) {
        if (mmwlan_get_sta_state() == MMWLAN_STA_DISABLED) {
            return true;
        }
        vTaskDelay(poll_ticks);
    }

    return mmwlan_get_sta_state() == MMWLAN_STA_DISABLED;
}

static void halow_shutdown_for_power_transition(bool hold_reset_n, const char *reason)
{
    if (s_connection_type != EDGEZ_TRANSPORT_HALOW) {
        return; /* HaLow was never used this boot */
    }

    const char *shutdown_reason = reason ? reason : (hold_reset_n ? "sleep" : "restart");
    ESP_LOGI(TAG, "Shutting down HaLow transceiver before %s", shutdown_reason);

    if (hold_reset_n) {
        /* Deep sleep path: avoid asynchronous supplicant/interface teardown in
         * vendor stack, which can race and double-free during power transition.
         * Hard-reset and hold RESET_N LOW, then let deep sleep reset the MCU. */
        halow_stop_ip_poll_task_if_running("deep sleep hard power-off");
        halow_reset_provisioning_state();

        gpio_set_level((gpio_num_t)CONFIG_MM_RESET_N, 0);
        gpio_hold_en((gpio_num_t)CONFIG_MM_RESET_N);
        gpio_deep_sleep_hold_en();

        s_halow_stack_initialized = false;
        halow_sta_connected = false;
        s_halow_light_sleep_active = false;

        ESP_LOGI(TAG, "HaLow transceiver hard-powered down for %s, GPIO %d held LOW",
                 shutdown_reason, CONFIG_MM_RESET_N);
        return;
    }

    enum mmwlan_status disable_status = mmwlan_sta_disable();
    if (disable_status != MMWLAN_SUCCESS) {
        ESP_LOGW(TAG, "mmwlan_sta_disable returned %d", (int)disable_status);
    }

    /* Wait for STA disable to complete BEFORE calling mmwlan_shutdown().
     * mmwlan_sta_disable() is async — the supplicant teardown runs on the
     * UMAC event loop.  If mmwlan_shutdown() frees internal state first,
     * the in-flight supplicant cleanup will access freed memory. */
    bool disabled = halow_wait_for_sta_disabled(HALOW_SLEEP_SHUTDOWN_TIMEOUT_MS);
    if (!disabled) {
        ESP_LOGW(TAG, "Timed out waiting for HaLow STA to disable (%d ms)", HALOW_SLEEP_SHUTDOWN_TIMEOUT_MS);
    }

    /* Extra settle time: mmwlan_get_sta_state() can report DISABLED while
     * the mesh/supplicant teardown (MPM CLOSE, wpa_config_free, etc.) is
     * still completing on the UMAC event loop.  Without this pause,
     * mmwlan_shutdown() → umac_connection_stop triggers a double-free of
     * the WPA SSID config that the event loop is still cleaning up. */
    vTaskDelay(pdMS_TO_TICKS(200));

    enum mmwlan_status st = mmwlan_shutdown();
    if (st != MMWLAN_SUCCESS) {
        ESP_LOGW(TAG, "mmwlan_shutdown returned %d", (int)st);
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    mmhal_wlan_deinit();
    s_halow_stack_initialized = false;

    halow_reset_provisioning_state();

    gpio_hold_dis((gpio_num_t)CONFIG_MM_RESET_N);
    ESP_LOGI(TAG, "HaLow transceiver powered down for %s, GPIO %d hold disabled",
             shutdown_reason, CONFIG_MM_RESET_N);

    halow_sta_connected = false;
    s_halow_light_sleep_active = false;
}

#endif

/* Event handler for catching system events */
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                (void)request_wifi_sta_connect("WIFI_EVENT_STA_START");
                break;
            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
                ESP_LOGW(TAG, "Disconnected from AP. Reason: %d. Reconnecting...", event->reason);

                s_wifi_connect_inflight = false;
                (void)request_wifi_sta_connect("WIFI_EVENT_STA_DISCONNECTED");
                break;
            }
            default:
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_connect_inflight = false;
#ifdef CONFIG_ENABLE_MM_HALOW
        if (!halow_sta_connected) {
            s_connection_type = EDGEZ_TRANSPORT_WIFI;
        }
#else
        s_connection_type = EDGEZ_TRANSPORT_WIFI;
#endif
        /* Signal main application to continue execution */
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
        xEventGroupSetBits(wifi_event_group, WIFI_NETWORK_READY_EVENT);
    }
}

static esp_err_t register_event_handler_once(esp_event_base_t base, int32_t event_id)
{
    esp_err_t err = esp_event_handler_register(base, event_id, &event_handler, NULL);
    if (err == ESP_ERR_INVALID_STATE) {
        return ESP_OK;
    }
    return err;
}

static esp_err_t ensure_prov_event_handlers_registered(void)
{
    if (s_prov_event_handlers_registered) {
        return ESP_OK;
    }

    esp_err_t err = register_event_handler_once(WIFI_EVENT, ESP_EVENT_ANY_ID);
    if (err != ESP_OK) {
        return err;
    }
    err = register_event_handler_once(IP_EVENT, IP_EVENT_STA_GOT_IP);
    if (err != ESP_OK) {
        return err;
    }

    s_prov_event_handlers_registered = true;
    return ESP_OK;
}

#ifdef CONFIG_ENABLE_MM_HALOW
/* Forward declaration for HaLow init function */
static esp_err_t halow_init(void);

static struct mmosal_semb *halow_link_up_semaphore = NULL;
static uint32_t halow_connection_attempts = 0;
static uint32_t halow_first_connect_time = 0;
static uint32_t s_halow_last_network_ready_ms = 0;
static uint32_t s_halow_last_sleep_guard_log_ms = 0;
static TimerHandle_t halow_stats_timer = NULL;
static QueueHandle_t halow_stats_queue = NULL;
static TaskHandle_t halow_stats_task_handle = NULL;
static TaskHandle_t halow_ip_poll_task_handle = NULL;
static TaskHandle_t halow_rx_sync_task_handle = NULL;
static bool s_halow_phy_link_up = false;
static bool s_halow_route_ready = false;
static bool s_halow_rx_sync_registered = false;
static bool s_halow_wake_static_ip_active = false;
static bool s_halow_real_ip_event_posted = false;
static bool s_halow_last_link_up_report_valid = false;
static mmipal_ip_addr_t s_halow_last_link_up_report_ip = {0};

typedef struct {
    bool valid;
    char ssid[MMWLAN_SSID_MAXLEN + 1];
    uint32_t channel_freq_hz;
    uint8_t bw_mhz;
    uint8_t bssid[6];
} halow_fast_hint_t;

typedef struct {
    bool active;
    char target_ssid[MMWLAN_SSID_MAXLEN + 1];
    char best_ssid[MMWLAN_SSID_MAXLEN + 1];
    bool best_valid;
    uint8_t best_priority;
    int16_t best_rssi;
    uint32_t best_freq_hz;
    uint8_t best_bw_mhz;
    uint8_t best_bssid[6];
} halow_connect_runtime_t;

typedef struct {
    bool valid;
    char ssid[MMWLAN_SSID_MAXLEN + 1];
    char passphrase[MMWLAN_PASSPHRASE_MAXLEN + 1];
    uint16_t passphrase_len;
    char edgez_ie_passphrase[MMWLAN_PASSPHRASE_MAXLEN + 1];
    uint16_t edgez_ie_passphrase_len;
    enum mmwlan_security_type security_type;
} halow_retry_profile_t;

typedef struct {
    bool valid;
    mmipal_ip_addr_t ip_addr;
    mmipal_ip_addr_t netmask;
    mmipal_ip_addr_t gateway;
} halow_cached_ip_config_t;

static halow_connect_runtime_t s_halow_connect_runtime = {0};

/* Snoop cache: source-IP -> source-MAC observed on incoming ethernet frames
 * (DHCP, ARP, ICMP, anything).  Used to seed the gateway ARP entry with the
 * REAL gateway MAC observed on the wire instead of the radio peer's BSSID,
 * which may be a relay that does not L3-forward our packets. */
#define HALOW_SRC_MAC_CACHE_SIZE 8
typedef struct {
    bool valid;
    ip4_addr_t ip;
    struct eth_addr mac;
    uint32_t last_ms;
} halow_src_mac_entry_t;
static halow_src_mac_entry_t s_halow_src_mac_cache[HALOW_SRC_MAC_CACHE_SIZE];
static portMUX_TYPE s_halow_src_mac_lock = portMUX_INITIALIZER_UNLOCKED;

static void halow_src_mac_observe(uint32_t ip_be, const uint8_t mac[6])
{
    if (ip_be == 0 || mac == NULL) {
        return;
    }
    /* Skip multicast/broadcast and all-zero MACs */
    if (mac[0] & 0x01) {
        return;
    }
    bool all_zero = true;
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0) { all_zero = false; break; }
    }
    if (all_zero) {
        return;
    }

    uint32_t now = mmosal_get_time_ms();
    bool inserted = false;
    bool changed_mac = false;
    uint8_t prev_mac[6] = {0};

    portENTER_CRITICAL(&s_halow_src_mac_lock);
    int free_idx = -1;
    int oldest_idx = 0;
    uint32_t oldest_ms = UINT32_MAX;
    int hit_idx = -1;
    for (int i = 0; i < HALOW_SRC_MAC_CACHE_SIZE; i++) {
        if (s_halow_src_mac_cache[i].valid &&
            s_halow_src_mac_cache[i].ip.addr == ip_be) {
            hit_idx = i;
            break;
        }
        if (!s_halow_src_mac_cache[i].valid && free_idx < 0) {
            free_idx = i;
        }
        if (s_halow_src_mac_cache[i].last_ms < oldest_ms) {
            oldest_ms = s_halow_src_mac_cache[i].last_ms;
            oldest_idx = i;
        }
    }

    if (hit_idx >= 0) {
        if (memcmp(s_halow_src_mac_cache[hit_idx].mac.addr, mac, 6) != 0) {
            memcpy(prev_mac, s_halow_src_mac_cache[hit_idx].mac.addr, 6);
            memcpy(s_halow_src_mac_cache[hit_idx].mac.addr, mac, 6);
            changed_mac = true;
        }
        s_halow_src_mac_cache[hit_idx].last_ms = now;
    } else {
        int slot = (free_idx >= 0) ? free_idx : oldest_idx;
        s_halow_src_mac_cache[slot].valid = true;
        s_halow_src_mac_cache[slot].ip.addr = ip_be;
        memcpy(s_halow_src_mac_cache[slot].mac.addr, mac, 6);
        s_halow_src_mac_cache[slot].last_ms = now;
        inserted = true;
    }
    portEXIT_CRITICAL(&s_halow_src_mac_lock);

    if (inserted) {
        ip4_addr_t tmp = { .addr = ip_be };
        ESP_LOGI(TAG,
                 "Observed peer " IPSTR " -> %02x:%02x:%02x:%02x:%02x:%02x",
                 IP2STR(&tmp),
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else if (changed_mac) {
        ip4_addr_t tmp = { .addr = ip_be };
        ESP_LOGI(TAG,
                 "Peer MAC changed for " IPSTR ": %02x:%02x:%02x:%02x:%02x:%02x -> %02x:%02x:%02x:%02x:%02x:%02x",
                 IP2STR(&tmp),
                 prev_mac[0], prev_mac[1], prev_mac[2],
                 prev_mac[3], prev_mac[4], prev_mac[5],
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
}

static bool halow_src_mac_lookup(uint32_t ip_be, struct eth_addr *out_mac)
{
    if (ip_be == 0 || out_mac == NULL) {
        return false;
    }
    bool found = false;
    portENTER_CRITICAL(&s_halow_src_mac_lock);
    for (int i = 0; i < HALOW_SRC_MAC_CACHE_SIZE; i++) {
        if (s_halow_src_mac_cache[i].valid &&
            s_halow_src_mac_cache[i].ip.addr == ip_be) {
            memcpy(out_mac->addr, s_halow_src_mac_cache[i].mac.addr, 6);
            found = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_halow_src_mac_lock);
    return found;
}

static void halow_src_mac_snoop_frame(const uint8_t *eth_frame, uint16_t len)
{
    if (eth_frame == NULL || len < 14) {
        return;
    }
    uint16_t etype = ((uint16_t)eth_frame[12] << 8) | eth_frame[13];
    if (etype == 0x0800 && len >= 34) {
        /* IPv4: src IP at offset 26..29 (network byte order = lwIP ip4_addr_t.addr) */
        uint32_t src_ip;
        memcpy(&src_ip, &eth_frame[26], sizeof(src_ip));
        halow_src_mac_observe(src_ip, &eth_frame[6]);
    } else if (etype == 0x0806 && len >= 14 + 28) {
        /* ARP: hw_type(2) proto_type(2) hw_len(1) proto_len(1) op(2) sender_hw(6) sender_ip(4) target_hw(6) target_ip(4) */
        const uint8_t *arp = &eth_frame[14];
        uint16_t hw_type = ((uint16_t)arp[0] << 8) | arp[1];
        uint16_t proto_type = ((uint16_t)arp[2] << 8) | arp[3];
        uint16_t op = ((uint16_t)arp[6] << 8) | arp[7];
        if (hw_type == 1 && proto_type == 0x0800 && (op == 1 || op == 2)) {
            uint32_t sender_ip;
            memcpy(&sender_ip, &arp[14], sizeof(sender_ip));
            halow_src_mac_observe(sender_ip, &arp[8]);
        }
    }
}
static struct mmwlan_s1g_channel s_halow_hint_channels[HALOW_HINT_MAX_CHANNELS] = {0};
static struct mmwlan_s1g_channel_list s_halow_hint_channel_list = {0};
static bool s_halow_pending_hint_valid = false;
static uint32_t s_halow_pending_hint_freq_hz = 0;
static uint8_t s_halow_pending_hint_bw_mhz = 0;
static uint32_t s_halow_active_hint_freq_hz = 0;
static uint8_t s_halow_active_hint_bw_mhz = 0;
static uint32_t s_halow_sta_enable_start_ms = 0;
static bool s_halow_logged_first_scan_candidate = false;
static bool s_halow_bssid_lock_active = false;
static uint8_t s_halow_auth_request_count = 0;
static uint8_t s_halow_auth_request_total = 0;
static uint8_t s_halow_scan_complete_count = 0;
static TaskHandle_t s_halow_bssid_unlock_task_handle = NULL;
static TaskHandle_t s_halow_scan_fallback_task_handle = NULL;
static halow_retry_profile_t s_halow_retry_profile = {0};
static bool s_halow_unlock_retry_used = false;
static halow_cached_ip_config_t s_halow_cached_ip = {0};

#define EDGEZ_ASSOC_IE_ID_VENDOR_SPECIFIC 221
#define EDGEZ_ASSOC_IE_VENDOR "EdgeZ"
#define EDGEZ_ASSOC_IE_VENDOR_LEN 5
#define EDGEZ_PEER_MODE_IE_VENDOR "EdgeZP"
#define EDGEZ_PEER_MODE_IE_VENDOR_LEN 6
#define EDGEZ_PEER_MODE_IE_VERSION 1
#define EDGEZ_PEER_MODE_IE_PAYLOAD_LEN (EDGEZ_PEER_MODE_IE_VENDOR_LEN + 2U)
#define EDGEZ_PEER_MODE_IE_LEN (2U + EDGEZ_PEER_MODE_IE_PAYLOAD_LEN)
#define EDGEZ_ASSOC_IE_NONCE_LEN 16
#define EDGEZ_ASSOC_IE_MAX_LEN 257
#define EDGEZ_MESH_VENDOR_IES_MAX_LEN (EDGEZ_ASSOC_IE_MAX_LEN + EDGEZ_PEER_MODE_IE_LEN)
#define EDGEZ_ASSOC_IE_DEFAULT_HOP 0
#define EDGEZ_ASSOC_IE_VERSION_LEN 4
#define EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN 16
#define EDGEZ_ASSOC_IE_ROUTE_MAC_LEN 6
#define EDGEZ_ASSOC_IE_ROUTE_TAIL_LEN_NO_UUID (2U + (2U * EDGEZ_ASSOC_IE_ROUTE_MAC_LEN))
#define EDGEZ_ASSOC_IE_ROUTE_TAIL_LEN (EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN + EDGEZ_ASSOC_IE_ROUTE_TAIL_LEN_NO_UUID)
#define EDGEZ_ASSOC_ROUTE_TABLE_SIZE 16
#define EDGEZ_ASSOC_SEEN_UUID_CACHE_SIZE 16

typedef struct {
    uint8_t message_uuid[EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN];
    uint8_t max_hop;
    uint8_t hop;
    uint8_t last_mac[EDGEZ_ASSOC_IE_ROUTE_MAC_LEN];
    uint8_t current_mac[EDGEZ_ASSOC_IE_ROUTE_MAC_LEN];
} halow_edgez_route_tail_t;

typedef struct {
    bool valid;
    uint64_t target;
    uint8_t next_hop[6];
    uint8_t message_uuid[EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN];
    uint8_t max_hop;
    uint8_t hop;
    uint8_t last_mac[EDGEZ_ASSOC_IE_ROUTE_MAC_LEN];
    uint8_t current_mac[EDGEZ_ASSOC_IE_ROUTE_MAC_LEN];
    bool forwardable;
    halow_edgez_route_tail_t forwarded_tail;
    uint32_t last_seen_ms;
} halow_edgez_route_entry_t;

static halow_edgez_route_entry_t s_edgez_route_table[EDGEZ_ASSOC_ROUTE_TABLE_SIZE];
static uint8_t s_edgez_route_replace_index;
static uint8_t s_edgez_seen_message_uuids[EDGEZ_ASSOC_SEEN_UUID_CACHE_SIZE][EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN];
static uint8_t s_edgez_seen_uuid_replace_index;
static uint8_t s_edgez_notified_message_uuids[EDGEZ_ASSOC_SEEN_UUID_CACHE_SIZE][EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN];
static uint8_t s_edgez_notified_uuid_replace_index;
static portMUX_TYPE s_edgez_notified_uuid_lock = portMUX_INITIALIZER_UNLOCKED;

static void halow_edgez_random_bytes(uint8_t *out, size_t len)
{
    if (!out) {
        return;
    }

    for (size_t off = 0; off < len; off += sizeof(uint32_t)) {
        uint32_t random_word = esp_random();
        size_t copy_len = len - off;
        if (copy_len > sizeof(random_word)) {
            copy_len = sizeof(random_word);
        }
        memcpy(&out[off], &random_word, copy_len);
    }
}

static void halow_edgez_random_nonce(uint8_t nonce[EDGEZ_ASSOC_IE_NONCE_LEN])
{
    halow_edgez_random_bytes(nonce, EDGEZ_ASSOC_IE_NONCE_LEN);
}

static uint32_t halow_edgez_firmware_version_numeric(void)
{
    const char *version = esp_app_get_description()->version;
    uint32_t parts[3] = {0};

    for (size_t i = 0; i < 3 && version && *version; i++) {
        char *end = NULL;
        unsigned long parsed = strtoul(version, &end, 10);
        if (end == version) {
            break;
        }
        if (parsed > UINT8_MAX) {
            parsed = UINT8_MAX;
        }
        parts[i] = (uint32_t)parsed;
        version = (*end == '.') ? (end + 1) : end;
    }

    return (parts[0] << 16) | (parts[1] << 8) | parts[2];
}

static void halow_edgez_write_u32_le(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value & 0xffU);
    out[1] = (uint8_t)((value >> 8) & 0xffU);
    out[2] = (uint8_t)((value >> 16) & 0xffU);
    out[3] = (uint8_t)((value >> 24) & 0xffU);
}

static uint32_t halow_edgez_read_u32_le(const uint8_t *in)
{
    return ((uint32_t)in[0]) |
           ((uint32_t)in[1] << 8) |
           ((uint32_t)in[2] << 16) |
           ((uint32_t)in[3] << 24);
}

static void halow_edgez_write_route_tail(uint8_t *out, const halow_edgez_route_tail_t *tail)
{
    if (!out) {
        return;
    }

    if (!tail) {
        memset(out, 0, EDGEZ_ASSOC_IE_ROUTE_TAIL_LEN);
        return;
    }

    size_t off = 0;
    memcpy(&out[off], tail->message_uuid, EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN);
    off += EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN;
    out[off++] = tail->max_hop;
    out[off++] = tail->hop;
    memcpy(&out[off], tail->last_mac, EDGEZ_ASSOC_IE_ROUTE_MAC_LEN);
    off += EDGEZ_ASSOC_IE_ROUTE_MAC_LEN;
    memcpy(&out[off], tail->current_mac, EDGEZ_ASSOC_IE_ROUTE_MAC_LEN);
}

static uint64_t halow_edgez_route_target_from_beacon(const ai_edgez_halow_Beacon *metadata)
{
    if (!metadata) {
        return 0;
    }
    return metadata->user_id_low != 0 ? metadata->user_id_low : metadata->user_id_high;
}

static halow_edgez_route_entry_t *halow_edgez_find_route(uint64_t target)
{
    if (target == 0) {
        return NULL;
    }
    for (uint8_t i = 0; i < EDGEZ_ASSOC_ROUTE_TABLE_SIZE; ++i) {
        if (s_edgez_route_table[i].valid && s_edgez_route_table[i].target == target) {
            return &s_edgez_route_table[i];
        }
    }
    return NULL;
}

static bool halow_edgez_mac_is_zero(const uint8_t mac[EDGEZ_ASSOC_IE_ROUTE_MAC_LEN])
{
    return !mac || (mac[0] == 0 && mac[1] == 0 && mac[2] == 0 &&
                    mac[3] == 0 && mac[4] == 0 && mac[5] == 0);
}

static bool halow_edgez_uuid_is_zero(const uint8_t uuid[EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN])
{
    if (!uuid) {
        return true;
    }

    for (size_t i = 0; i < EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN; ++i) {
        if (uuid[i] != 0) {
            return false;
        }
    }

    return true;
}

static bool halow_edgez_message_uuid_seen(const uint8_t uuid[EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN])
{
    if (halow_edgez_uuid_is_zero(uuid)) {
        return false;
    }

    for (uint8_t i = 0; i < EDGEZ_ASSOC_SEEN_UUID_CACHE_SIZE; ++i) {
        if (memcmp(s_edgez_seen_message_uuids[i], uuid, EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN) == 0) {
            return true;
        }
    }

    memcpy(s_edgez_seen_message_uuids[s_edgez_seen_uuid_replace_index],
           uuid,
           EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN);
    s_edgez_seen_uuid_replace_index = (uint8_t)((s_edgez_seen_uuid_replace_index + 1U) % EDGEZ_ASSOC_SEEN_UUID_CACHE_SIZE);
    return false;
}

static bool halow_edgez_notification_uuid_seen(
    const uint8_t uuid[EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN])
{
    if (halow_edgez_uuid_is_zero(uuid)) {
        return false;
    }

    portENTER_CRITICAL(&s_edgez_notified_uuid_lock);
    for (uint8_t i = 0; i < EDGEZ_ASSOC_SEEN_UUID_CACHE_SIZE; ++i) {
        if (memcmp(s_edgez_notified_message_uuids[i],
                   uuid,
                   EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN) == 0) {
            portEXIT_CRITICAL(&s_edgez_notified_uuid_lock);
            return true;
        }
    }

    memcpy(s_edgez_notified_message_uuids[s_edgez_notified_uuid_replace_index],
           uuid,
           EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN);
    s_edgez_notified_uuid_replace_index =
        (uint8_t)((s_edgez_notified_uuid_replace_index + 1U) %
                  EDGEZ_ASSOC_SEEN_UUID_CACHE_SIZE);
    portEXIT_CRITICAL(&s_edgez_notified_uuid_lock);
    return false;
}

static bool halow_edgez_get_self_mac(uint8_t mac[EDGEZ_ASSOC_IE_ROUTE_MAC_LEN])
{
    if (!mac) {
        return false;
    }

    memset(mac, 0, EDGEZ_ASSOC_IE_ROUTE_MAC_LEN);
    if (mmwlan_get_mac_addr(mac) == MMWLAN_SUCCESS && !halow_edgez_mac_is_zero(mac)) {
        return true;
    }

    if ((esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK ||
         esp_read_mac(mac, ESP_MAC_BT) == ESP_OK ||
         esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK) &&
        !halow_edgez_mac_is_zero(mac)) {
        return true;
    }

    return false;
}

static bool halow_edgez_route_can_forward(const halow_edgez_route_tail_t *tail)
{
    return tail && tail->max_hop > 0 && tail->hop < tail->max_hop;
}

static bool halow_edgez_make_forward_tail(const halow_edgez_route_tail_t *in,
                                          const uint8_t self_mac[EDGEZ_ASSOC_IE_ROUTE_MAC_LEN],
                                          halow_edgez_route_tail_t *out)
{
    if (!in || !self_mac || !out || !halow_edgez_route_can_forward(in)) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    memcpy(out->message_uuid, in->message_uuid, EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN);
    out->max_hop = in->max_hop;
    out->hop = (uint8_t)(in->hop + 1U);
    memcpy(out->last_mac, in->current_mac, EDGEZ_ASSOC_IE_ROUTE_MAC_LEN);
    memcpy(out->current_mac, self_mac, EDGEZ_ASSOC_IE_ROUTE_MAC_LEN);
    return true;
}

static void halow_edgez_update_route(const char *mesh_id,
                                     const uint8_t bssid[6],
                                     const uint8_t self_mac[EDGEZ_ASSOC_IE_ROUTE_MAC_LEN],
                                     const ai_edgez_halow_Beacon *metadata,
                                     const halow_edgez_route_tail_t *tail)
{
    uint64_t target = halow_edgez_route_target_from_beacon(metadata);
    if (target == 0 || !bssid || !tail) {
        return;
    }

    halow_edgez_route_entry_t *entry = halow_edgez_find_route(target);
    if (!entry) {
        entry = &s_edgez_route_table[s_edgez_route_replace_index];
        memset(entry, 0, sizeof(*entry));
        s_edgez_route_replace_index = (uint8_t)((s_edgez_route_replace_index + 1U) % EDGEZ_ASSOC_ROUTE_TABLE_SIZE);
    }

    entry->valid = true;
    entry->target = target;
    memcpy(entry->next_hop, bssid, 6);
    memcpy(entry->message_uuid, tail->message_uuid, EDGEZ_ASSOC_IE_MESSAGE_UUID_LEN);
    entry->max_hop = tail->max_hop;
    entry->hop = tail->hop;
    memcpy(entry->last_mac, tail->last_mac, EDGEZ_ASSOC_IE_ROUTE_MAC_LEN);
    memcpy(entry->current_mac, tail->current_mac, EDGEZ_ASSOC_IE_ROUTE_MAC_LEN);
    bool duplicate = halow_edgez_message_uuid_seen(tail->message_uuid);
    entry->forwardable = !duplicate && halow_edgez_make_forward_tail(tail, self_mac, &entry->forwarded_tail);
    entry->last_seen_ms = mmosal_get_time_ms();

    ESP_LOGI(TAG,
             "EdgeZ route update mesh_id=%s target=0x%016llx next=%02x:%02x:%02x:%02x:%02x:%02x uuid=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x max_hop=%u hop=%u last=%02x:%02x:%02x:%02x:%02x:%02x current=%02x:%02x:%02x:%02x:%02x:%02x duplicate=%u forward=%u",
             mesh_id && mesh_id[0] ? mesh_id : "<none>",
             (unsigned long long)entry->target,
             entry->next_hop[0], entry->next_hop[1], entry->next_hop[2],
             entry->next_hop[3], entry->next_hop[4], entry->next_hop[5],
             entry->message_uuid[0], entry->message_uuid[1], entry->message_uuid[2], entry->message_uuid[3],
             entry->message_uuid[4], entry->message_uuid[5], entry->message_uuid[6], entry->message_uuid[7],
             entry->message_uuid[8], entry->message_uuid[9], entry->message_uuid[10], entry->message_uuid[11],
             entry->message_uuid[12], entry->message_uuid[13], entry->message_uuid[14], entry->message_uuid[15],
             (unsigned)entry->max_hop,
             (unsigned)entry->hop,
             entry->last_mac[0], entry->last_mac[1], entry->last_mac[2],
             entry->last_mac[3], entry->last_mac[4], entry->last_mac[5],
             entry->current_mac[0], entry->current_mac[1], entry->current_mac[2],
             entry->current_mac[3], entry->current_mac[4], entry->current_mac[5],
             duplicate ? 1 : 0,
             entry->forwardable ? 1 : 0);

    if (entry->forwardable) {
        ESP_LOGI(TAG,
                 "EdgeZ route forward tail target=0x%016llx uuid=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x max_hop=%u hop=%u last=%02x:%02x:%02x:%02x:%02x:%02x current=%02x:%02x:%02x:%02x:%02x:%02x",
                 (unsigned long long)entry->target,
                 entry->forwarded_tail.message_uuid[0], entry->forwarded_tail.message_uuid[1],
                 entry->forwarded_tail.message_uuid[2], entry->forwarded_tail.message_uuid[3],
                 entry->forwarded_tail.message_uuid[4], entry->forwarded_tail.message_uuid[5],
                 entry->forwarded_tail.message_uuid[6], entry->forwarded_tail.message_uuid[7],
                 entry->forwarded_tail.message_uuid[8], entry->forwarded_tail.message_uuid[9],
                 entry->forwarded_tail.message_uuid[10], entry->forwarded_tail.message_uuid[11],
                 entry->forwarded_tail.message_uuid[12], entry->forwarded_tail.message_uuid[13],
                 entry->forwarded_tail.message_uuid[14], entry->forwarded_tail.message_uuid[15],
                 (unsigned)entry->forwarded_tail.max_hop,
                 (unsigned)entry->forwarded_tail.hop,
                 entry->forwarded_tail.last_mac[0], entry->forwarded_tail.last_mac[1], entry->forwarded_tail.last_mac[2],
                 entry->forwarded_tail.last_mac[3], entry->forwarded_tail.last_mac[4], entry->forwarded_tail.last_mac[5],
                 entry->forwarded_tail.current_mac[0], entry->forwarded_tail.current_mac[1], entry->forwarded_tail.current_mac[2],
                 entry->forwarded_tail.current_mac[3], entry->forwarded_tail.current_mac[4], entry->forwarded_tail.current_mac[5]);
    }
}

static esp_err_t halow_edgez_sha256(const uint8_t *input,
                                    size_t input_len,
                                    uint8_t output[32])
{
    if ((!input && input_len > 0) || !output) {
        return ESP_ERR_INVALID_ARG;
    }

    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
#if MBEDTLS_VERSION_MAJOR >= 3
    if (mbedtls_sha256_starts(&ctx, 0) != 0 ||
        mbedtls_sha256_update(&ctx, input, input_len) != 0 ||
        mbedtls_sha256_finish(&ctx, output) != 0) {
        mbedtls_sha256_free(&ctx);
        return ESP_FAIL;
    }
#else
    if (mbedtls_sha256_starts_ret(&ctx, 0) != 0 ||
        mbedtls_sha256_update_ret(&ctx, input, input_len) != 0 ||
        mbedtls_sha256_finish_ret(&ctx, output) != 0) {
        mbedtls_sha256_free(&ctx);
        return ESP_FAIL;
    }
#endif
    mbedtls_sha256_free(&ctx);
    return ESP_OK;
}

static esp_err_t halow_edgez_encrypt_payload(const uint8_t *passphrase,
                                             size_t passphrase_len,
                                             const uint8_t nonce[EDGEZ_ASSOC_IE_NONCE_LEN],
                                             const uint8_t *plain,
                                             size_t plain_len,
                                             uint8_t *cipher)
{
    if (!passphrase || !nonce || (!plain && plain_len > 0) || !cipher) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t key[32] = {0};
    esp_err_t err = halow_edgez_sha256(passphrase, passphrase_len, key);
    if (err != ESP_OK) {
        return err;
    }

    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    if (mbedtls_aes_setkey_enc(&aes, key, sizeof(key) * 8) != 0) {
        mbedtls_aes_free(&aes);
        memset(key, 0, sizeof(key));
        return ESP_FAIL;
    }

    size_t nc_off = 0;
    uint8_t nonce_counter[EDGEZ_ASSOC_IE_NONCE_LEN] = {0};
    uint8_t stream_block[EDGEZ_ASSOC_IE_NONCE_LEN] = {0};
    memcpy(nonce_counter, nonce, sizeof(nonce_counter));
    int aes_ret = mbedtls_aes_crypt_ctr(&aes,
                                        plain_len,
                                        &nc_off,
                                        nonce_counter,
                                        stream_block,
                                        plain,
                                        cipher);
    mbedtls_aes_free(&aes);
    memset(key, 0, sizeof(key));
    memset(nonce_counter, 0, sizeof(nonce_counter));
    memset(stream_block, 0, sizeof(stream_block));
    return (aes_ret == 0) ? ESP_OK : ESP_FAIL;
}

static esp_err_t halow_edgez_encode_assoc_metadata(uint8_t *out,
                                                   size_t out_size,
                                                   size_t *out_len)
{
    if (!out || !out_len) {
        return ESP_ERR_INVALID_ARG;
    }

    ai_edgez_halow_Beacon metadata = ai_edgez_halow_Beacon_init_zero;
    metadata.user_id_high = s_halow_user_identity.user_id_high;
    metadata.user_id_low = s_halow_user_identity.user_id_low;
    strlcpy(metadata.user_name, s_halow_user_identity.user_name, sizeof(metadata.user_name));
    metadata.user_public_key.size = (pb_size_t)s_halow_user_identity.user_public_key_len;
    memcpy(metadata.user_public_key.bytes,
           s_halow_user_identity.user_public_key,
           metadata.user_public_key.size);
    metadata.marker = s_halow_user_identity.marker;
    metadata.device_type = s_halow_user_identity.device_type;
    metadata.sensor_data_count = s_halow_user_identity.sensor_data_count;
    memcpy(metadata.sensor_data,
           s_halow_user_identity.sensor_data,
           metadata.sensor_data_count * sizeof(metadata.sensor_data[0]));

    bool has_sensor_latitude = false;
    bool has_sensor_longitude = false;
    for (pb_size_t i = 0; i < metadata.sensor_data_count; ++i) {
        has_sensor_latitude |=
            metadata.sensor_data[i].type == ai_edgez_halow_SensorType_SENSOR_LATITUDE;
        has_sensor_longitude |=
            metadata.sensor_data[i].type == ai_edgez_halow_SensorType_SENSOR_LONGITUDE;
    }
    if (s_halow_user_identity.has_location &&
        !has_sensor_latitude &&
        metadata.sensor_data_count < 7) {
        ai_edgez_halow_SensorData *latitude =
            &metadata.sensor_data[metadata.sensor_data_count++];
        latitude->type = ai_edgez_halow_SensorType_SENSOR_LATITUDE;
        latitude->which_value = ai_edgez_halow_SensorData_float_value_tag;
        latitude->value.float_value = s_halow_user_identity.latitude;
    }
    if (s_halow_user_identity.has_location &&
        !has_sensor_longitude &&
        metadata.sensor_data_count < 7) {
        ai_edgez_halow_SensorData *longitude =
            &metadata.sensor_data[metadata.sensor_data_count++];
        longitude->type = ai_edgez_halow_SensorType_SENSOR_LONGITUDE;
        longitude->which_value = ai_edgez_halow_SensorData_float_value_tag;
        longitude->value.float_value = s_halow_user_identity.longitude;
    }

    pb_ostream_t stream = pb_ostream_from_buffer(out, out_size);
    if (!pb_encode(&stream, ai_edgez_halow_Beacon_fields, &metadata)) {
        ESP_LOGW(TAG, "EdgeZ assoc metadata encode failed: %s", PB_GET_ERROR(&stream));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG,
             "EdgeZ beacon metadata encoded sensor_data=%u len=%u",
             (unsigned)metadata.sensor_data_count,
             (unsigned)stream.bytes_written);
    for (pb_size_t i = 0; i < metadata.sensor_data_count; ++i) {
        const ai_edgez_halow_SensorData *entry = &metadata.sensor_data[i];
        if (entry->which_value == ai_edgez_halow_SensorData_float_value_tag) {
            ESP_LOGI(TAG,
                     "EdgeZ beacon SensorData[%u] type=%u float_value=%.3f",
                     (unsigned)i,
                     (unsigned)entry->type,
                     (double)entry->value.float_value);
        } else if (entry->which_value == ai_edgez_halow_SensorData_int_value_tag) {
            ESP_LOGI(TAG,
                     "EdgeZ beacon SensorData[%u] type=%u int_value=%ld",
                     (unsigned)i,
                     (unsigned)entry->type,
                     (long)entry->value.int_value);
        } else if (entry->which_value == ai_edgez_halow_SensorData_bool_value_tag) {
            ESP_LOGI(TAG,
                     "EdgeZ beacon SensorData[%u] type=%u bool_value=%u",
                     (unsigned)i,
                     (unsigned)entry->type,
                     entry->value.bool_value ? 1U : 0U);
        }
    }

    *out_len = stream.bytes_written;
    return ESP_OK;
}

static bool halow_edgez_decode_assoc_metadata(const uint8_t *payload,
                                              size_t payload_len,
                                              ai_edgez_halow_Beacon *metadata)
{
    if (!payload || payload_len == 0 || !metadata) {
        return false;
    }

    memset(metadata, 0, sizeof(*metadata));
    pb_istream_t stream = pb_istream_from_buffer(payload, payload_len);
    if (!pb_decode(&stream, ai_edgez_halow_Beacon_fields, metadata)) {
        ESP_LOGW(TAG, "EdgeZ assoc metadata decode failed: %s", PB_GET_ERROR(&stream));
        return false;
    }

    return true;
}

static void halow_edgez_log_beacon_base64(const char *prefix,
                                          const uint8_t *payload,
                                          size_t payload_len)
{
    if (!prefix || !payload || payload_len == 0) {
        return;
    }

    char beacon_b64[((ai_edgez_halow_Beacon_size + 2U) / 3U * 4U) + 1U] = {0};
    size_t beacon_b64_len = 0;
    if (mbedtls_base64_encode((uint8_t *)beacon_b64,
                              sizeof(beacon_b64),
                              &beacon_b64_len,
                              payload,
                              payload_len) != 0 ||
        beacon_b64_len >= sizeof(beacon_b64)) {
        ESP_LOGW(TAG, "%s beacon_base64 encode failed len=%u", prefix, (unsigned)payload_len);
        return;
    }

    beacon_b64[beacon_b64_len] = '\0';
    ESP_LOGI(TAG, "%s beacon_base64 len=%u text=%s",
             prefix,
             (unsigned)beacon_b64_len,
             beacon_b64);
}

static esp_err_t halow_build_edgez_assoc_ie(const char *mesh_id,
                                            const char *country_code,
                                            const char *passphrase,
                                            uint8_t *out,
                                            size_t out_size,
                                            size_t *out_len)
{
    if (!out || !out_len || out_size < EDGEZ_ASSOC_IE_MAX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    (void)mesh_id;
    (void)country_code;
    (void)passphrase;

    const size_t payload_off = 2U + EDGEZ_ASSOC_IE_VENDOR_LEN;
    if (out_size < payload_off) {
        return ESP_ERR_NO_MEM;
    }

    size_t beacon_len = 0;
    esp_err_t err = halow_edgez_encode_assoc_metadata(&out[payload_off],
                                                      out_size - payload_off,
                                                      &beacon_len);
    if (err != ESP_OK) {
        return err;
    }
    if (beacon_len > (UINT8_MAX - EDGEZ_ASSOC_IE_VENDOR_LEN)) {
        return ESP_ERR_NO_MEM;
    }

    out[0] = EDGEZ_ASSOC_IE_ID_VENDOR_SPECIFIC;
    out[1] = (uint8_t)(EDGEZ_ASSOC_IE_VENDOR_LEN + beacon_len);
    memcpy(&out[2], EDGEZ_ASSOC_IE_VENDOR, EDGEZ_ASSOC_IE_VENDOR_LEN);
    *out_len = payload_off + beacon_len;
    ESP_LOGI(TAG,
             "EdgeZ compact beacon IE prepared: vendor=%s beacon_len=%u ie_len=%u",
             EDGEZ_ASSOC_IE_VENDOR,
             (unsigned)beacon_len,
             (unsigned)*out_len);
    return ESP_OK;
}

esp_err_t halow_build_edgez_mesh_vendor_ie(const char *mesh_id,
                                           const char *country_code,
                                           const char *passphrase,
                                           uint8_t *out,
                                           size_t out_size,
                                           size_t *out_len)
{
    return halow_build_edgez_assoc_ie(mesh_id, country_code, passphrase,
                                      out, out_size, out_len);
}

esp_err_t wifi_prov_refresh_halow_mesh_vendor_ie(const char *mesh_id,
                                                 const char *passphrase)
{
#ifdef CONFIG_ENABLE_MM_HALOW
    uint8_t ies[EDGEZ_MESH_VENDOR_IES_MAX_LEN] = {0};
    size_t ies_len = 0;
    esp_err_t err = halow_build_edgez_mesh_vendor_ie(mesh_id,
                                                     s_halow_country_code,
                                                     passphrase,
                                                     ies,
                                                     sizeof(ies),
                                                     &ies_len);
    if (err != ESP_OK) {
        return err;
    }

    enum mmwlan_status status = mmwlan_update_mesh_vendor_ies(ies, ies_len);
    if (status != MMWLAN_SUCCESS) {
        ESP_LOGW(TAG, "EdgeZ mesh Vendor IE refresh failed status=%d", (int)status);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "EdgeZ mesh Vendor IE refreshed len=%u", (unsigned)ies_len);
    return ESP_OK;
#else
    (void)mesh_id;
    (void)passphrase;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static esp_err_t halow_attach_edgez_assoc_ie(struct mmwlan_sta_args *sta_args,
                                             const char *ie_passphrase,
                                             uint8_t *ie_buf,
                                             size_t ie_buf_size)
{
    if (!sta_args || !ie_buf) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t ie_len = 0;
    esp_err_t err = halow_build_edgez_assoc_ie((const char *)sta_args->ssid,
                                               s_halow_country_code,
                                               ie_passphrase,
                                               ie_buf,
                                               ie_buf_size,
                                               &ie_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to prepare EdgeZ association IE: %s", esp_err_to_name(err));
        return err;
    }

    sta_args->extra_assoc_ies = ie_buf;
    sta_args->extra_assoc_ies_len = ie_len;
    return ESP_OK;
}

static bool halow_edgez_payload_is_valid(const uint8_t *payload, size_t payload_len)
{
    if (!payload || payload_len == 0) {
        return false;
    }

    ai_edgez_halow_Beacon metadata = ai_edgez_halow_Beacon_init_zero;
    if (!halow_edgez_decode_assoc_metadata(payload, payload_len, &metadata)) {
        return false;
    }

    bool complete =
        (metadata.user_id_high != 0 || metadata.user_id_low != 0) &&
        metadata.user_name[0] != '\0' &&
        metadata.user_public_key.size > 0 &&
        metadata.device_type >= ai_edgez_halow_DeviceType_DEVICE_TYPE_USER &&
        metadata.device_type <= ai_edgez_halow_DeviceType_DEVICE_TYPE_RELAY;

    if (!complete) {
        ESP_LOGW(TAG,
                 "Invalid EdgeZ beacon IE: incomplete identity user=%016llx-%016llx name=%u key_len=%u type=%u",
                 (unsigned long long)metadata.user_id_high,
                 (unsigned long long)metadata.user_id_low,
                 metadata.user_name[0] != '\0' ? 1U : 0U,
                 (unsigned)metadata.user_public_key.size,
                 (unsigned)metadata.device_type);
    }
    return complete;
}

static bool halow_parse_edgez_discovery_ie(const uint8_t *ies,
                                           size_t ies_len,
                                           const char *expected_mesh_id,
                                           const char *passphrase,
                                           uint8_t *metadata_out,
                                           size_t metadata_out_size,
                                           size_t *metadata_out_len,
                                           halow_edgez_route_tail_t *route_tail_out)
{
    if (!ies || !expected_mesh_id || !metadata_out || !metadata_out_len) {
        return false;
    }

    *metadata_out_len = 0;
    if (route_tail_out) {
        memset(route_tail_out, 0, sizeof(*route_tail_out));
    }
    (void)expected_mesh_id;
    (void)passphrase;

    size_t ie_off = 0;
    while (ie_off + 2U <= ies_len) {
        uint8_t ie_id = ies[ie_off];
        uint8_t ie_len = ies[ie_off + 1U];
        size_t body_off = ie_off + 2U;
        size_t next = body_off + ie_len;
        if (next > ies_len) {
            break;
        }

        if (ie_id != EDGEZ_ASSOC_IE_ID_VENDOR_SPECIFIC ||
            ie_len <= EDGEZ_ASSOC_IE_VENDOR_LEN ||
            memcmp(&ies[body_off], EDGEZ_ASSOC_IE_VENDOR, EDGEZ_ASSOC_IE_VENDOR_LEN) != 0) {
            ie_off = next;
            continue;
        }

        const size_t off = body_off + EDGEZ_ASSOC_IE_VENDOR_LEN;
        const size_t beacon_len = next - off;
        if (beacon_len == 0 || beacon_len > metadata_out_size) {
            ESP_LOGW(TAG,
                     "Invalid EdgeZ compact beacon IE: bad beacon_len=%u max=%u",
                     (unsigned)beacon_len,
                     (unsigned)metadata_out_size);
            ie_off = next;
            continue;
        }
        memcpy(metadata_out, &ies[off], beacon_len);

        if (!halow_edgez_payload_is_valid(metadata_out, beacon_len)) {
            ESP_LOGW(TAG,
                     "Invalid EdgeZ beacon IE: metadata decode/validation failed beacon_len=%u",
                     (unsigned)beacon_len);
            ie_off = next;
            continue;
        }

        ESP_LOGI(TAG, "EdgeZ compact beacon IE accepted: vendor=%s beacon_len=%u",
                 EDGEZ_ASSOC_IE_VENDOR, (unsigned)beacon_len);
        halow_edgez_log_beacon_base64("EdgeZ beacon IE", metadata_out, beacon_len);
        *metadata_out_len = beacon_len;
        return true;
    }

    return false;
}

bool halow_edgez_mesh_peer_admission_allowed(const uint8_t *ies,
                                             size_t ies_len,
                                             uint32_t *device_type)
{
    if (device_type) {
        *device_type = ai_edgez_halow_DeviceType_DEVICE_TYPE_UNSPECIFIED;
    }
    if (!ies) {
        return true;
    }

    size_t ie_off = 0;
    while (ie_off + 2U <= ies_len) {
        uint8_t ie_id = ies[ie_off];
        uint8_t ie_len = ies[ie_off + 1U];
        size_t body_off = ie_off + 2U;
        size_t next = body_off + ie_len;
        if (next > ies_len) {
            break;
        }
        if (ie_id == EDGEZ_ASSOC_IE_ID_VENDOR_SPECIFIC &&
            ie_len > EDGEZ_ASSOC_IE_VENDOR_LEN &&
            memcmp(&ies[body_off], EDGEZ_ASSOC_IE_VENDOR,
                   EDGEZ_ASSOC_IE_VENDOR_LEN) == 0) {
            ai_edgez_halow_Beacon beacon = ai_edgez_halow_Beacon_init_zero;
            const uint8_t *payload = &ies[body_off + EDGEZ_ASSOC_IE_VENDOR_LEN];
            size_t payload_len = ie_len - EDGEZ_ASSOC_IE_VENDOR_LEN;

            if (!halow_edgez_decode_assoc_metadata(payload, payload_len, &beacon)) {
                return true;
            }
            if (device_type) {
                *device_type = (uint32_t)beacon.device_type;
            }
            return beacon.device_type !=
                       ai_edgez_halow_DeviceType_DEVICE_TYPE_BEACON &&
                   beacon.device_type !=
                       ai_edgez_halow_DeviceType_DEVICE_TYPE_SENSOR;
        }
        ie_off = next;
    }
    return true;
}

esp_err_t halow_notify_mesh_beacon_to_mobile(const struct mmwlan_scan_result *result,
                                             const char *expected_mesh_id,
                                             const char *passphrase)
{
    static const uint8_t unknown_bssid[6] = {0};
    uint8_t metadata[ai_edgez_halow_Beacon_size] = {0};
    size_t metadata_len = 0;
    ai_edgez_halow_Beacon beacon = ai_edgez_halow_Beacon_init_zero;
    halow_edgez_route_tail_t route_tail = {0};

    if (result == NULL || result->ies == NULL || expected_mesh_id == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!halow_parse_edgez_discovery_ie(result->ies,
                                        result->ies_len,
                                        expected_mesh_id,
                                        passphrase,
                                        metadata,
                                        sizeof(metadata),
                                        &metadata_len,
                                        &route_tail))
    {
        return ESP_ERR_NOT_FOUND;
    }
    if (!halow_edgez_decode_assoc_metadata(metadata, metadata_len, &beacon))
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Live Vendor IE callbacks provide the beacon IE list but not transmitter
     * metadata. Keep the shared scan/live decoder safe and report an unknown
     * peer address instead of dereferencing a NULL synthetic-result BSSID. */
    const uint8_t *peer_bssid = result->bssid != NULL ? result->bssid : unknown_bssid;
    const uint8_t *peer_mac = peer_bssid;
    ESP_LOGI(TAG,
             "Mesh beacon notify BLE peer=%02x:%02x:%02x:%02x:%02x:%02x user=%016llx-%016llx hop=%u/%u",
             peer_bssid[0], peer_bssid[1], peer_bssid[2],
             peer_bssid[3], peer_bssid[4], peer_bssid[5],
             (unsigned long long)beacon.user_id_high,
             (unsigned long long)beacon.user_id_low,
             0U,
             0U);
    /* This callback carries the RSSI for the actual peer beacon. The global
     * STA RSSI is AP-oriented and remains zero in mesh mode. */
    halow_sync_bridge_notify_beacon(&beacon, peer_mac, result->rssi);
    return ESP_OK;
}

static bool halow_is_valid_ipv4_string(const char *ip)
{
    if (ip == NULL || ip[0] == '\0') {
        return false;
    }

    struct in_addr addr = {0};
    if (inet_pton(AF_INET, ip, &addr) != 1) {
        return false;
    }

    uint32_t host_addr = ntohl(addr.s_addr);
    if (host_addr == 0U || host_addr == 0xFFFFFFFFU) {
        return false;
    }

    return true;
}

static void halow_cache_valid_ip_config(const struct mmipal_link_status *link_status)
{
    if (!link_status) {
        return;
    }

    if (!halow_is_valid_ipv4_string(link_status->ip_addr) ||
        !halow_is_valid_ipv4_string(link_status->netmask) ||
        !halow_is_valid_ipv4_string(link_status->gateway)) {
        return;
    }

    strncpy(s_halow_cached_ip.ip_addr,
            link_status->ip_addr,
            sizeof(s_halow_cached_ip.ip_addr) - 1);
    s_halow_cached_ip.ip_addr[sizeof(s_halow_cached_ip.ip_addr) - 1] = '\0';
    strncpy(s_halow_cached_ip.netmask,
            link_status->netmask,
            sizeof(s_halow_cached_ip.netmask) - 1);
    s_halow_cached_ip.netmask[sizeof(s_halow_cached_ip.netmask) - 1] = '\0';
    strncpy(s_halow_cached_ip.gateway,
            link_status->gateway,
            sizeof(s_halow_cached_ip.gateway) - 1);
    s_halow_cached_ip.gateway[sizeof(s_halow_cached_ip.gateway) - 1] = '\0';
    s_halow_cached_ip.valid = true;
}

static bool halow_bssid_is_nonzero(const uint8_t bssid[6])
{
    for (int i = 0; i < 6; i++) {
        if (bssid[i] != 0) {
            return true;
        }
    }
    return false;
}

static int halow_hex_nibble(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return 10 + (c - 'a');
    }
    if (c >= 'A' && c <= 'F') {
        return 10 + (c - 'A');
    }
    return -1;
}

static bool halow_parse_mac_addr(const char *mac_str, uint8_t mac[6])
{
    if (!mac_str || !mac) {
        return false;
    }

    for (unsigned i = 0; i < 6; i++) {
        int hi = halow_hex_nibble(mac_str[i * 3]);
        int lo = halow_hex_nibble(mac_str[i * 3 + 1]);
        if (hi < 0 || lo < 0) {
            return false;
        }
        mac[i] = (uint8_t)((hi << 4) | lo);
        if (i < 5 && mac_str[i * 3 + 2] != ':') {
            return false;
        }
    }

    return mac_str[17] == '\0';
}

/**
 * tcpip_callback handler: bring mesh netif link up and force-restart DHCP.
 *
 * In mesh mode the connection FSM never transitions through CONNECTING →
 * CONNECTED the same way a normal STA join does.  SUPP_PORT_OPEN (surfaced
 * as CTRL_PORT_OPEN) signals that mesh peering + SAE is complete, but the
 * internal connection_fsm_connected_entry is never reached.  Without an
 * explicit link-up + DHCP kick here, DHCP never starts and the device never
 * obtains an IP address.
 *
 * This mirrors the mesh_probe reference implementation.
 */
static void halow_mesh_link_up_cb(void *arg)
{
    struct netif *nif = (struct netif *)arg;
    if (nif == NULL) {
        return;
    }

    ESP_LOGI(TAG, "mesh_link_up_cb: netif=%p flags=0x%02x link_up=%d",
             nif, nif->flags, netif_is_link_up(nif));

    netif_set_link_up(nif);

    /* Force-restart DHCP so it doesn't stay stuck from a previous cycle */
    struct dhcp *d = netif_dhcp_data(nif);
    if (d) {
        dhcp_release_and_stop(nif);
        dhcp_start(nif);
        ESP_LOGI(TAG, "mesh_link_up_cb: DHCP restarted");
    } else {
        err_t err = dhcp_start(nif);
        ESP_LOGI(TAG, "mesh_link_up_cb: DHCP started fresh err=%d", err);
    }
}

static void halow_apply_mesh_scan_profile(void)
{
    ESP_LOGI(TAG, "Factory mesh scan profile disabled");
}

static void halow_apply_mesh_identity_profile(struct mmwlan_sta_args *sta_args,
                                              const char *fallback_ssid,
                                              const char *fallback_password)
{
    if (!sta_args) {
        return;
    }

    const char *mesh_id = fallback_ssid;
    const char *mesh_key = fallback_password;

#ifdef CONFIG_MM_EXPERIMENTAL_MESH_KEY
    if (strlen(CONFIG_MM_EXPERIMENTAL_MESH_KEY) > 0) {
        mesh_key = CONFIG_MM_EXPERIMENTAL_MESH_KEY;
    }
#endif

    if (!mesh_id) {
        mesh_id = "";
    }
    if (!mesh_key) {
        mesh_key = "";
    }

    size_t mesh_id_len = strlen(mesh_id);
    if (mesh_id_len > MMWLAN_SSID_MAXLEN) {
        mesh_id_len = MMWLAN_SSID_MAXLEN;
    }
    memset(sta_args->ssid, 0, sizeof(sta_args->ssid));
    if (mesh_id_len > 0) {
        memcpy(sta_args->ssid, mesh_id, mesh_id_len);
    }
    sta_args->ssid_len = (uint8_t)mesh_id_len;

    size_t mesh_key_len = strlen(mesh_key);
    if (mesh_key_len > MMWLAN_PASSPHRASE_MAXLEN) {
        mesh_key_len = MMWLAN_PASSPHRASE_MAXLEN;
    }
    memset(sta_args->passphrase, 0, sizeof(sta_args->passphrase));
    if (mesh_key_len > 0) {
        memcpy(sta_args->passphrase, mesh_key, mesh_key_len);
        sta_args->passphrase_len = (uint16_t)mesh_key_len;
        sta_args->security_type = MMWLAN_SAE;
    } else {
        sta_args->passphrase_len = 0;
        sta_args->security_type = MMWLAN_OPEN;
    }

    ESP_LOGI(TAG,
             "Mesh identity profile: mesh_id=%s key_len=%u",
             (const char *)sta_args->ssid,
             (unsigned)sta_args->passphrase_len);
}

static void halow_apply_mesh_sta_profile(struct mmwlan_sta_args *sta_args,
                                         bool allow_config_bssid_override)
{
    if (!sta_args) {
        return;
    }

    sta_args->mesh_mode = true;
    sta_args->mesh_frequency_khz = s_runtime_halow_mesh_frequency_khz;
    sta_args->mesh_bandwidth_mhz = s_runtime_halow_mesh_bandwidth_mhz;
    sta_args->scan_rx_cb = NULL;
    sta_args->scan_rx_cb_arg = NULL;
    sta_args->scan_interval_base_s = HALOW_MESH_SCAN_INTERVAL_BASE_S;
    sta_args->scan_interval_limit_s = HALOW_MESH_SCAN_INTERVAL_LIMIT_S;

    /* Beacon and sensor devices publish their application beacon explicitly at
     * the configured report interval. Mark them as sensor STAs so the mesh
     * stack does not run its 100 ms maintenance beacon or discovery scans. */
    const bool low_duty_mesh =
        s_halow_user_identity.device_type == ai_edgez_halow_DeviceType_DEVICE_TYPE_BEACON ||
        s_halow_user_identity.device_type == ai_edgez_halow_DeviceType_DEVICE_TYPE_SENSOR;
    sta_args->sta_type = low_duty_mesh ? MMWLAN_STA_TYPE_SENSOR : MMWLAN_STA_TYPE_NON_SENSOR;

    halow_apply_mesh_scan_profile();

    ESP_LOGI(TAG,
             "Mesh runtime profile: active_mesh_id=%s freq=%lu kHz bw=%uMHz",
             (const char *)sta_args->ssid,
             (unsigned long)sta_args->mesh_frequency_khz,
             (unsigned)sta_args->mesh_bandwidth_mhz);

#ifdef CONFIG_MM_EXPERIMENTAL_MESH_BSSID
    if (allow_config_bssid_override &&
#ifdef CONFIG_MM_EXPERIMENTAL_MESH_LOCK_BSSID
        (CONFIG_MM_EXPERIMENTAL_MESH_LOCK_BSSID != 0) &&
#endif
        strlen(CONFIG_MM_EXPERIMENTAL_MESH_BSSID) > 0) {
        uint8_t cfg_bssid[6] = {0};
        if (halow_parse_mac_addr(CONFIG_MM_EXPERIMENTAL_MESH_BSSID, cfg_bssid)) {
            memcpy(sta_args->bssid, cfg_bssid, sizeof(sta_args->bssid));
            s_halow_bssid_lock_active = true;
            ESP_LOGI(TAG,
                     "Mesh config BSSID lock enabled: %s",
                     CONFIG_MM_EXPERIMENTAL_MESH_BSSID);
        } else {
            ESP_LOGW(TAG,
                     "Mesh config BSSID invalid: %s",
                     CONFIG_MM_EXPERIMENTAL_MESH_BSSID);
        }
    }
#endif
}

static void halow_clear_fast_hint_bssid_for_ssid(const char *ssid)
{
    (void)ssid;
}

static bool halow_normalize_hint_channel(uint32_t *freq_hz, uint8_t *bw_mhz)
{
    if (!freq_hz || !bw_mhz || *freq_hz == 0) {
        return false;
    }

    const struct mmwlan_s1g_channel_list *full_list =
        mmwlan_lookup_regulatory_domain(get_regulatory_db(), s_halow_country_code);
    if (!full_list || !full_list->channels || full_list->num_channels == 0) {
        return false;
    }

    int best_idx = -1;
    uint32_t best_diff = UINT32_MAX;

    for (int pass = 0; pass < 2 && best_idx < 0; pass++) {
        bool require_bw_match = (pass == 0 && *bw_mhz != 0);

        for (uint32_t i = 0; i < full_list->num_channels; i++) {
            const struct mmwlan_s1g_channel *ch = &full_list->channels[i];

            if (require_bw_match && ch->bw_mhz != *bw_mhz) {
                continue;
            }

            uint32_t diff = (ch->centre_freq_hz > *freq_hz)
                ? (ch->centre_freq_hz - *freq_hz)
                : (*freq_hz - ch->centre_freq_hz);

            if (best_idx < 0 || diff < best_diff) {
                best_idx = (int)i;
                best_diff = diff;
            }
        }
    }

    if (best_idx < 0) {
        return false;
    }

    *freq_hz = full_list->channels[best_idx].centre_freq_hz;
    *bw_mhz = full_list->channels[best_idx].bw_mhz;
    return true;
}

esp_err_t wifi_prov_set_halow_mesh_radio(uint32_t frequency_khz,
                                         uint32_t bandwidth_mhz)
{
#ifdef CONFIG_ENABLE_MM_HALOW
    if (frequency_khz == 0 ||
        !(bandwidth_mhz == 1 || bandwidth_mhz == 2 ||
          bandwidth_mhz == 4 || bandwidth_mhz == 8)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t frequency_hz = frequency_khz * 1000U;
    uint8_t bandwidth_mhz_u8 = (uint8_t)bandwidth_mhz;
    if (!halow_normalize_hint_channel(&frequency_hz, &bandwidth_mhz_u8)) {
        ESP_LOGW(TAG,
                 "HaLow mesh radio rejected: freq=%lu kHz bw=%luMHz country=%s",
                 (unsigned long)frequency_khz,
                 (unsigned long)bandwidth_mhz,
                 s_halow_country_code);
        return ESP_ERR_INVALID_ARG;
    }

    s_halow_pending_hint_valid = true;
    s_halow_pending_hint_freq_hz = frequency_hz;
    s_halow_pending_hint_bw_mhz = bandwidth_mhz_u8;
    s_halow_active_hint_freq_hz = frequency_hz;
    s_halow_active_hint_bw_mhz = bandwidth_mhz_u8;
    s_runtime_halow_mesh_frequency_khz = frequency_hz / 1000U;
    s_runtime_halow_mesh_bandwidth_mhz = bandwidth_mhz_u8;
    ESP_LOGI(TAG,
             "HaLow mesh radio selected: requested=%lu kHz/%luMHz resolved=%lu Hz/%uMHz country=%s",
             (unsigned long)frequency_khz,
             (unsigned long)bandwidth_mhz,
             (unsigned long)frequency_hz,
             (unsigned)bandwidth_mhz_u8,
             s_halow_country_code);
    return ESP_OK;
#else
    (void)frequency_khz;
    (void)bandwidth_mhz;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static bool halow_lookup_freq_for_channel(uint16_t channel, uint8_t bw_mhz, uint32_t *out_freq_hz)
{
    if (!out_freq_hz || channel == 0 || bw_mhz == 0) {
        return false;
    }

    const struct mmwlan_s1g_channel_list *full_list =
        mmwlan_lookup_regulatory_domain(get_regulatory_db(), s_halow_country_code);
    if (!full_list || !full_list->channels || full_list->num_channels == 0) {
        return false;
    }

    for (uint32_t i = 0; i < full_list->num_channels; i++) {
        const struct mmwlan_s1g_channel *ch = &full_list->channels[i];
        if (ch->s1g_chan_num == channel && ch->bw_mhz == bw_mhz) {
            *out_freq_hz = ch->centre_freq_hz;
            return true;
        }
    }

    return false;
}

static void halow_save_fast_hint(const char *ssid, uint32_t freq_hz, uint8_t bw_mhz, const uint8_t *bssid)
{
    (void)ssid;
    (void)freq_hz;
    (void)bw_mhz;
    (void)bssid;
}

static bool halow_load_fast_hint_for_ssid(const char *ssid, halow_fast_hint_t *hint)
{
    (void)ssid;
    if (hint) {
        memset(hint, 0, sizeof(*hint));
    }
    return false;
}

static bool halow_set_hint_channel_list_from_freq(uint32_t freq_hz, uint8_t bw_mhz)
{
    const struct mmwlan_s1g_channel_list *full_list =
        mmwlan_lookup_regulatory_domain(get_regulatory_db(), s_halow_country_code);
    if (!full_list || !full_list->channels || full_list->num_channels == 0) {
        return false;
    }

    halow_log_channel_list("Full regulatory channel list", full_list);

    const uint32_t candidate_count = full_list->num_channels;
    const uint32_t region_half_span_hz = HALOW_WAKE_HINT_REGION_HALF_SPAN_MHZ * 1000000U;
    const uint32_t channel_cap =
        (HALOW_WAKE_HINT_MAX_CHANNELS == 0 || HALOW_WAKE_HINT_MAX_CHANNELS > HALOW_HINT_MAX_CHANNELS)
            ? HALOW_HINT_MAX_CHANNELS
            : HALOW_WAKE_HINT_MAX_CHANNELS;
    uint64_t min_freq_hz = (freq_hz > region_half_span_hz) ? (uint64_t)(freq_hz - region_half_span_hz) : 0;
    uint64_t max_freq_hz = (uint64_t)freq_hz + region_half_span_hz;

    uint32_t selected_diff_hz[HALOW_HINT_MAX_CHANNELS] = {0};
    uint32_t selected = 0;

    uint32_t in_region_total = 0;
    for (uint32_t i = 0; i < candidate_count; i++) {
        const struct mmwlan_s1g_channel *ch = &full_list->channels[i];
        if (!(ch->bw_mhz == 1 || ch->bw_mhz == 2 || ch->bw_mhz == 4 || ch->bw_mhz == 8)) {
            continue;
        }

        if ((uint64_t)ch->centre_freq_hz < min_freq_hz || (uint64_t)ch->centre_freq_hz > max_freq_hz) {
            continue;
        }

        in_region_total++;
        uint32_t diff_hz = (ch->centre_freq_hz > freq_hz)
            ? (ch->centre_freq_hz - freq_hz)
            : (freq_hz - ch->centre_freq_hz);

        if (selected < channel_cap) {
            s_halow_hint_channels[selected] = *ch;
            selected_diff_hz[selected] = diff_hz;
            selected++;
            continue;
        }

        uint32_t worst_idx = 0;
        uint32_t worst_diff = selected_diff_hz[0];
        for (uint32_t j = 1; j < channel_cap; j++) {
            if (selected_diff_hz[j] > worst_diff) {
                worst_diff = selected_diff_hz[j];
                worst_idx = j;
            }
        }

        if (diff_hz < worst_diff) {
            s_halow_hint_channels[worst_idx] = *ch;
            selected_diff_hz[worst_idx] = diff_hz;
        }
    }

    if (selected == 0) {
        for (uint8_t target_bw = 1; target_bw <= 8 && selected < channel_cap; target_bw <<= 1) {
            int best_index = -1;
            uint32_t best_diff_hz = UINT32_MAX;
            for (uint32_t i = 0; i < full_list->num_channels; i++) {
                const struct mmwlan_s1g_channel *ch = &full_list->channels[i];
                if (ch->bw_mhz != target_bw) {
                    continue;
                }
                uint32_t diff_hz = (ch->centre_freq_hz > freq_hz)
                    ? (ch->centre_freq_hz - freq_hz)
                    : (freq_hz - ch->centre_freq_hz);
                if (diff_hz < best_diff_hz) {
                    best_diff_hz = diff_hz;
                    best_index = (int)i;
                }
            }
            if (best_index >= 0) {
                bool duplicate = false;
                for (uint32_t j = 0; j < selected; j++) {
                    if (s_halow_hint_channels[j].centre_freq_hz == full_list->channels[best_index].centre_freq_hz &&
                        s_halow_hint_channels[j].bw_mhz == full_list->channels[best_index].bw_mhz) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate) {
                    s_halow_hint_channels[selected++] = full_list->channels[best_index];
                }
            }
        }

        if (selected == 0) {
            return false;
        }
    }

    for (uint32_t i = 0; i < selected; i++) {
        for (uint32_t j = i + 1; j < selected; j++) {
            bool swap_needed = false;
            if (s_halow_hint_channels[j].centre_freq_hz < s_halow_hint_channels[i].centre_freq_hz) {
                swap_needed = true;
            } else if (s_halow_hint_channels[j].centre_freq_hz == s_halow_hint_channels[i].centre_freq_hz &&
                       s_halow_hint_channels[j].bw_mhz < s_halow_hint_channels[i].bw_mhz) {
                swap_needed = true;
            }

            if (swap_needed) {
                struct mmwlan_s1g_channel tmp = s_halow_hint_channels[i];
                s_halow_hint_channels[i] = s_halow_hint_channels[j];
                s_halow_hint_channels[j] = tmp;
            }
        }
    }

    memset(&s_halow_hint_channel_list, 0, sizeof(s_halow_hint_channel_list));
    s_halow_hint_channel_list.country_code[0] = full_list->country_code[0];
    s_halow_hint_channel_list.country_code[1] = full_list->country_code[1];
    s_halow_hint_channel_list.country_code[2] = '\0';
    s_halow_hint_channel_list.num_channels = selected;
    s_halow_hint_channel_list.channels = s_halow_hint_channels;

    halow_log_channel_list("Filtered hint-centered channel list", &s_halow_hint_channel_list);

    enum mmwlan_status status = mmwlan_set_channel_list(&s_halow_hint_channel_list);
    if (status != MMWLAN_SUCCESS) {
        return false;
    }

    ESP_LOGI(TAG,
             "Using hint-centered consecutive region list: hint_freq=%lu hint_bw=%u span=%uMHz cap=%lu range=[%lu,%lu] channels=%lu in_region=%lu",
             (unsigned long)freq_hz,
             (unsigned)bw_mhz,
             (unsigned)HALOW_WAKE_HINT_REGION_HALF_SPAN_MHZ,
             (unsigned long)channel_cap,
             (unsigned long)min_freq_hz,
             (unsigned long)max_freq_hz,
             (unsigned long)selected,
             (unsigned long)in_region_total);
#if HALOW_HINT_STRICT_SINGLE_CHANNEL
    ESP_LOGI(TAG, "Strict single-channel fast hint mode enabled");
#endif
    return true;
}

static void halow_log_channel_list(const char *label, const struct mmwlan_s1g_channel_list *channel_list)
{
    if (!label || !channel_list || !channel_list->channels) {
        return;
    }

    ESP_LOGI(TAG, "%s: country=%s count=%lu", label, channel_list->country_code,
             (unsigned long)channel_list->num_channels);

    if (!HALOW_VERBOSE_CHANNEL_LOG && channel_list->num_channels > 12) {
        const struct mmwlan_s1g_channel *first = &channel_list->channels[0];
        const struct mmwlan_s1g_channel *last = &channel_list->channels[channel_list->num_channels - 1];
        ESP_LOGI(TAG,
                 "  (summary) first: freq=%lu bw=%u s1g=%u, last: freq=%lu bw=%u s1g=%u",
                 (unsigned long)first->centre_freq_hz,
                 (unsigned)first->bw_mhz,
                 (unsigned)first->s1g_chan_num,
                 (unsigned long)last->centre_freq_hz,
                 (unsigned)last->bw_mhz,
                 (unsigned)last->s1g_chan_num);
        ESP_LOGI(TAG, "  (set CONFIG_HALOW_VERBOSE_CHANNEL_LOG=y for full channel dump)");
        return;
    }

    for (uint32_t i = 0; i < channel_list->num_channels; i++) {
        const struct mmwlan_s1g_channel *ch = &channel_list->channels[i];
        ESP_LOGI(TAG,
                 "  ch[%02lu]: freq=%lu bw=%u s1g=%u max_eirp=%d",
                 (unsigned long)i,
                 (unsigned long)ch->centre_freq_hz,
                 (unsigned)ch->bw_mhz,
                 (unsigned)ch->s1g_chan_num,
                 (int)ch->max_tx_eirp_dbm);
    }
}

static void halow_sta_scan_capture_cb(const struct mmwlan_scan_result *result, void *arg)
{
    (void)arg;
    if (!result || !s_halow_connect_runtime.active) {
        return;
    }

    char found_ssid[MMWLAN_SSID_MAXLEN + 1] = {0};
    if (result->ssid_len > 0 && result->ssid_len <= MMWLAN_SSID_MAXLEN) {
        memcpy(found_ssid, result->ssid, result->ssid_len);
        found_ssid[result->ssid_len] = '\0';
    }

    bool ssid_match = (found_ssid[0] != '\0') &&
                      (strcmp(found_ssid, s_halow_connect_runtime.target_ssid) == 0);
    bool mesh_id_match = false;
    char mesh_id[MMWLAN_SSID_MAXLEN + 1] = {0};
    if (ssid_match) {
        strlcpy(mesh_id, found_ssid, sizeof(mesh_id));
        mesh_id_match = true;
    }

    if (!mesh_id_match && s_startup_halow_mesh_mode && result->ies && result->ies_len > 0) {
        size_t off = 0;
        while (off + 2 <= result->ies_len) {
            uint8_t eid = result->ies[off];
            uint8_t elen = result->ies[off + 1];
            size_t next = off + 2 + elen;
            if (next > result->ies_len) {
                break;
            }
            if (eid == 114) { /* WLAN_IE_ID_MESH_ID */
                size_t copy_len = (elen < MMWLAN_SSID_MAXLEN) ? elen : MMWLAN_SSID_MAXLEN;
                memcpy(mesh_id, &result->ies[off + 2], copy_len);
                mesh_id[copy_len] = '\0';
                mesh_id_match = (strcmp(mesh_id, s_halow_connect_runtime.target_ssid) == 0);
                break;
            }
            off = next;
        }
    }

    uint8_t observed_bw = (result->op_bw_mhz != 0) ? result->op_bw_mhz : result->bw_mhz;
    if (!ssid_match && !mesh_id_match) {
        if (s_startup_halow_mesh_mode) {
            ESP_LOGI(TAG,
                     "Mesh scan candidate miss: ssid=%s mesh_id=%s rssi=%d freq=%lu bw=%u",
                     (found_ssid[0] != '\0') ? found_ssid : "<empty>",
                     (mesh_id[0] != '\0') ? mesh_id : "<none>",
                     (int)result->rssi,
                     (unsigned long)result->channel_freq_hz,
                     (unsigned)((result->op_bw_mhz != 0) ? result->op_bw_mhz : result->bw_mhz));
        }
        return;
    }

    uint8_t edgez_metadata[ai_edgez_halow_Beacon_size] = {0};
    size_t edgez_metadata_len = 0;
    ai_edgez_halow_Beacon edgez_assoc_metadata = ai_edgez_halow_Beacon_init_zero;
    halow_edgez_route_tail_t edgez_route_tail = {0};
    if (s_startup_halow_mesh_mode) {
        const char *expected_mesh_id = s_halow_connect_runtime.target_ssid;
        const char *ie_passphrase = s_halow_retry_profile.edgez_ie_passphrase;

        if (!mesh_id_match) {
            ESP_LOGW(TAG,
                     "Invalid EdgeZ beacon: mesh id mismatch ssid=%s mesh_id=%s expected=%s",
                     (found_ssid[0] != '\0') ? found_ssid : "<empty>",
                     (mesh_id[0] != '\0') ? mesh_id : "<none>",
                     expected_mesh_id);
            return;
        }

        if (!halow_parse_edgez_discovery_ie(result->ies,
                                           result->ies_len,
                                           expected_mesh_id,
                                           ie_passphrase,
                                           edgez_metadata,
                                           sizeof(edgez_metadata),
                                           &edgez_metadata_len,
                                           &edgez_route_tail)) {
            ESP_LOGW(TAG,
                     "Invalid EdgeZ beacon: missing or invalid EdgeZ IE mesh_id=%s ssid=%s rssi=%d freq=%lu bw=%u",
                     (mesh_id[0] != '\0') ? mesh_id : "<none>",
                     (found_ssid[0] != '\0') ? found_ssid : "<empty>",
                     (int)result->rssi,
                     (unsigned long)result->channel_freq_hz,
                     (unsigned)observed_bw);
            return;
        }
        if (!halow_edgez_decode_assoc_metadata(edgez_metadata,
                                               edgez_metadata_len,
                                               &edgez_assoc_metadata)) {
            ESP_LOGW(TAG,
                     "Invalid EdgeZ beacon: metadata decode failed after validation mesh_id=%s ssid=%s",
                     (mesh_id[0] != '\0') ? mesh_id : "<none>",
                     (found_ssid[0] != '\0') ? found_ssid : "<empty>");
            return;
        }
        uint8_t self_mac[EDGEZ_ASSOC_IE_ROUTE_MAC_LEN] = {0};
        (void)halow_edgez_get_self_mac(self_mac);
        halow_edgez_update_route(mesh_id,
                                 result->bssid,
                                 self_mac,
                                 &edgez_assoc_metadata,
                                 &edgez_route_tail);
    }

    if (s_startup_halow_mesh_mode) {
        ESP_LOGI(TAG,
                 "Mesh scan candidate hit: ssid=%s mesh_id=%s rssi=%d freq=%lu bw=%u edgez_meta_len=%u max_hop=%u hop=%u",
                 (found_ssid[0] != '\0') ? found_ssid : "<empty>",
                 (mesh_id[0] != '\0') ? mesh_id : "<none>",
                 (int)result->rssi,
                 (unsigned long)result->channel_freq_hz,
                 (unsigned)observed_bw,
                 (unsigned)edgez_metadata_len,
                 (unsigned)edgez_route_tail.max_hop,
                 (unsigned)edgez_route_tail.hop);
    }

    if (s_startup_halow_mesh_mode) {
        halow_sync_bridge_notify_beacon(&edgez_assoc_metadata, result->bssid, result->rssi);
    }

    if (!s_halow_logged_first_scan_candidate) {
        uint32_t now_ms = mmosal_get_time_ms();
        ESP_LOGI(TAG,
                 "HaLow timing: first matching scan candidate at +%lu ms from sta_enable",
                 (unsigned long)(now_ms - s_halow_sta_enable_start_ms));
        s_halow_logged_first_scan_candidate = true;
    }

    uint8_t candidate_bw = (result->op_bw_mhz != 0) ? result->op_bw_mhz : result->bw_mhz;
    uint32_t candidate_freq_hz = result->channel_freq_hz;
    if (candidate_bw != 0) {
        uint32_t normalized_freq_hz = candidate_freq_hz;
        uint8_t normalized_bw_mhz = candidate_bw;
        if (halow_normalize_hint_channel(&normalized_freq_hz, &normalized_bw_mhz)) {
            candidate_freq_hz = normalized_freq_hz;
            candidate_bw = normalized_bw_mhz;
        }
    }

    uint8_t candidate_priority = 2;
    if (s_halow_active_hint_bw_mhz != 0 && candidate_bw == s_halow_active_hint_bw_mhz) {
        candidate_priority = 1;
        if (s_halow_active_hint_freq_hz != 0 && candidate_freq_hz == s_halow_active_hint_freq_hz) {
            candidate_priority = 0;
        }
    }

    bool take_candidate = !s_halow_connect_runtime.best_valid;
    if (!take_candidate) {
        if (candidate_priority < s_halow_connect_runtime.best_priority) {
            take_candidate = true;
        } else if (candidate_priority == s_halow_connect_runtime.best_priority &&
                   result->rssi > s_halow_connect_runtime.best_rssi) {
            take_candidate = true;
        }
    }

    if (take_candidate) {
        s_halow_connect_runtime.best_valid = true;
        s_halow_connect_runtime.best_priority = candidate_priority;
        s_halow_connect_runtime.best_rssi = result->rssi;
        s_halow_connect_runtime.best_freq_hz = candidate_freq_hz;
        s_halow_connect_runtime.best_bw_mhz = candidate_bw;
        memcpy(s_halow_connect_runtime.best_bssid, result->bssid, 6);
        if (found_ssid[0] != '\0') {
            strncpy(s_halow_connect_runtime.best_ssid,
                    found_ssid,
                    sizeof(s_halow_connect_runtime.best_ssid) - 1);
            s_halow_connect_runtime.best_ssid[sizeof(s_halow_connect_runtime.best_ssid) - 1] = '\0';
        } else {
            strncpy(s_halow_connect_runtime.best_ssid,
                    s_halow_connect_runtime.target_ssid,
                    sizeof(s_halow_connect_runtime.best_ssid) - 1);
            s_halow_connect_runtime.best_ssid[sizeof(s_halow_connect_runtime.best_ssid) - 1] = '\0';
        }

        ESP_LOGI(TAG,
                 "Updated best HaLow candidate: rx_freq=%lu norm_freq=%lu rx_bw=%u op_bw=%u norm_bw=%u prio=%u rssi=%d",
                 (unsigned long)result->channel_freq_hz,
                 (unsigned long)candidate_freq_hz,
                 (unsigned)result->bw_mhz,
                 (unsigned)result->op_bw_mhz,
                 (unsigned)candidate_bw,
                 (unsigned)candidate_priority,
                 (int)result->rssi);
    }
}

static void halow_sta_event_callback(const struct mmwlan_sta_event_cb_args *sta_event, void *arg)
{
    (void)arg;
    if (!sta_event) {
        return;
    }

    const char *event_name = "UNKNOWN";
    switch (sta_event->event) {
        case MMWLAN_STA_EVT_SCAN_REQUEST:
            event_name = "SCAN_REQUEST";
            break;
        case MMWLAN_STA_EVT_SCAN_COMPLETE:
            event_name = "SCAN_COMPLETE";
            if (s_halow_scan_complete_count < UINT8_MAX) {
                s_halow_scan_complete_count++;
            }
            if (s_halow_light_sleep_active &&
                s_halow_light_sleep_shutdown &&
                !halow_sta_connected &&
                s_halow_retry_profile.valid &&
                s_halow_auth_request_total == 0 &&
                s_halow_scan_complete_count >= HALOW_WAKE_SCAN_NO_AUTH_RETRY_THRESHOLD &&
                s_halow_scan_fallback_task_handle == NULL) {
                ESP_LOGW(TAG,
                         "HaLow wake: %u scan completes without AUTH, scheduling full-channel fallback retry",
                         (unsigned)s_halow_scan_complete_count);
                if (xTaskCreate(halow_scan_fallback_retry_task,
                                "halow_scan_fallback",
                                6144,
                                NULL,
                                5,
                                &s_halow_scan_fallback_task_handle) != pdPASS) {
                    ESP_LOGE(TAG, "Failed to create scan fallback retry task");
                    s_halow_scan_fallback_task_handle = NULL;
                }
            }
            break;
        case MMWLAN_STA_EVT_SCAN_ABORT:
            event_name = "SCAN_ABORT";
            break;
        case MMWLAN_STA_EVT_AUTH_REQUEST:
            event_name = "AUTH_REQUEST";
            s_halow_scan_complete_count = 0;
            if (s_halow_auth_request_total < UINT8_MAX) {
                s_halow_auth_request_total++;
            }
            if (s_halow_bssid_lock_active && !halow_sta_connected) {
                s_halow_auth_request_count++;
                if (s_halow_auth_request_count >= 2 && s_halow_bssid_unlock_task_handle == NULL) {
                    ESP_LOGW(TAG,
                             "HaLow timing: repeated AUTH_REQUEST with BSSID lock (%u), scheduling unlock retry",
                             (unsigned)s_halow_auth_request_count);
                    if (xTaskCreate(halow_bssid_unlock_retry_task,
                                    "halow_unlock_retry",
                                    6144,
                                    NULL,
                                    5,
                                    &s_halow_bssid_unlock_task_handle) != pdPASS) {
                        ESP_LOGE(TAG, "Failed to create BSSID unlock retry task");
                        s_halow_bssid_unlock_task_handle = NULL;
                    }
                }
            }
            break;
        case MMWLAN_STA_EVT_ASSOC_REQUEST:
            event_name = "ASSOC_REQUEST";
            s_halow_scan_complete_count = 0;
            break;
        case MMWLAN_STA_EVT_DEAUTH_TX:
            event_name = "DEAUTH_TX";
            break;
        case MMWLAN_STA_EVT_CTRL_PORT_OPEN:
            event_name = "CTRL_PORT_OPEN";
            /* In mesh mode the connection FSM never reaches
             * connection_fsm_connected_entry, so the normal
             * MMWLAN_STA_CONNECTED callback (which stops WiFi, marks
             * halow_sta_connected, kicks DHCP, etc.) is bypassed.
             *
             * CTRL_PORT_OPEN signals that mesh peering + SAE is complete.
             * We must:
             *   1. Force the MM netif link up
             *   2. Force-start DHCP
             *   3. Stop ESP32 WiFi so it doesn't interfere with HaLow DHCP
             *   4. Mark connection state and start IP poll fallback
             *
             * This mirrors the mesh_probe reference implementation. */
            if (s_startup_halow_mesh_mode) {
                struct netif *mm_nif = halow_find_mmnetif();
                if (mm_nif != NULL) {
                    ESP_LOGI(TAG,
                             "CTRL_PORT_OPEN (mesh): netif=%p link_up=%d -> bringing link up for DHCP",
                             mm_nif, netif_is_link_up(mm_nif));
                    err_t tcpip_err = tcpip_callback_with_block(
                        halow_mesh_link_up_cb, mm_nif, 0);
                    ESP_LOGI(TAG,
                             "CTRL_PORT_OPEN (mesh): tcpip_callback err=%d",
                             tcpip_err);
                } else {
                    ESP_LOGW(TAG,
                             "CTRL_PORT_OPEN (mesh): MM netif not found, cannot bring link up");
                }

                /* Mark connected and stop WiFi — same as STA_CONNECTED path */
                if (!halow_sta_connected) {
                    halow_sta_connected = true;
                    s_connection_type = EDGEZ_TRANSPORT_HALOW;

                    ESP_LOGI(TAG, "CTRL_PORT_OPEN (mesh): stopping ESP32 WiFi for HaLow DHCP");
                    esp_wifi_stop();

                    if (wifi_event_group) {
                        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
                    }

                    halow_start_ip_poll_task_if_needed("mesh CTRL_PORT_OPEN");
                }
            }
            break;
        case MMWLAN_STA_EVT_CTRL_PORT_CLOSED:
            event_name = "CTRL_PORT_CLOSED";
            ESP_LOGW(TAG, "CTRL_PORT_CLOSED: mesh peer link lost, halow_sta_connected=%d", halow_sta_connected);
            if (halow_sta_connected) {
                halow_sta_connected = false;
                s_halow_route_ready = false;
                s_halow_last_network_ready_ms = 0;
                if (wifi_event_group) {
                    xEventGroupClearBits(wifi_event_group,
                                         WIFI_CONNECTED_EVENT | WIFI_NETWORK_READY_EVENT);
                }
                ESP_LOGW(TAG, "CTRL_PORT_CLOSED: marking link down, will need reconnect");
            }
            break;
        default:
            break;
    }

    uint32_t now_ms = mmosal_get_time_ms();
    ESP_LOGI(TAG,
             "HaLow timing: STA_EVT_%s at +%lu ms from sta_enable",
             event_name,
             (unsigned long)(now_ms - s_halow_sta_enable_start_ms));
}

static void halow_bssid_unlock_retry_task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(20));

    if (halow_sta_connected || !s_halow_retry_profile.valid) {
        s_halow_bssid_unlock_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGW(TAG, "Retrying HaLow connect without BSSID lock due to auth retries");
    halow_clear_fast_hint_bssid_for_ssid(s_halow_retry_profile.ssid);
    s_halow_unlock_retry_used = true;

    enum mmwlan_status status = mmwlan_sta_disable();
    if (status != MMWLAN_SUCCESS) {
        ESP_LOGW(TAG, "mmwlan_sta_disable before retry returned status=%d", status);
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    struct mmwlan_sta_args sta_args = MMWLAN_STA_ARGS_INIT;
    sta_args.ssid_len = strlen(s_halow_retry_profile.ssid);
    memcpy(sta_args.ssid, s_halow_retry_profile.ssid, sta_args.ssid_len);
    sta_args.security_type = s_halow_retry_profile.security_type;
    sta_args.passphrase_len = s_halow_retry_profile.passphrase_len;
    if (sta_args.passphrase_len > 0) {
        memcpy(sta_args.passphrase, s_halow_retry_profile.passphrase, sta_args.passphrase_len);
    }
    sta_args.scan_rx_cb = NULL;
    sta_args.scan_rx_cb_arg = NULL;
    sta_args.sta_evt_cb = halow_sta_event_callback;
    sta_args.sta_evt_cb_arg = NULL;
    sta_args.scan_interval_base_s = UINT16_MAX;
    sta_args.scan_interval_limit_s = UINT16_MAX;

    uint8_t edgez_assoc_ie[EDGEZ_ASSOC_IE_MAX_LEN] = {0};
    if (s_startup_halow_mesh_mode) {
        halow_apply_mesh_sta_profile(&sta_args, true);
        esp_err_t ie_err = halow_attach_edgez_assoc_ie(&sta_args,
                                                       s_halow_retry_profile.edgez_ie_passphrase,
                                                       edgez_assoc_ie,
                                                       sizeof(edgez_assoc_ie));
        if (ie_err != ESP_OK) {
            ESP_LOGE(TAG, "Unlock retry aborted: EdgeZ association IE unavailable");
            s_halow_bssid_unlock_task_handle = NULL;
            vTaskDelete(NULL);
            return;
        }
    }

    s_halow_bssid_lock_active = false;
    s_halow_auth_request_count = 0;
    s_halow_auth_request_total = 0;
    s_halow_sta_enable_start_ms = mmosal_get_time_ms();
    s_halow_logged_first_scan_candidate = false;

    status = mmwlan_sta_enable(&sta_args, halow_sta_status_callback);
    if (status != MMWLAN_SUCCESS) {
        ESP_LOGW(TAG, "Unlock retry failed status=%d, retrying with full regulatory channel list", status);
        if (halow_set_full_regulatory_channel_list() == ESP_OK) {
            status = mmwlan_sta_enable(&sta_args, halow_sta_status_callback);
        }
    }

    if (status == MMWLAN_SUCCESS) {
        ESP_LOGI(TAG, "Unlock retry started successfully");
    } else {
        ESP_LOGE(TAG, "Unlock retry failed status=%d", status);
    }

    s_halow_bssid_unlock_task_handle = NULL;
    vTaskDelete(NULL);
}

static void halow_scan_fallback_retry_task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(20));

    if (halow_sta_connected || !s_halow_retry_profile.valid) {
        s_halow_scan_fallback_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGW(TAG,
             "Retrying HaLow connect with full regulatory channels after repeated scan-without-auth");

    s_halow_pending_hint_valid = false;
    s_halow_pending_hint_freq_hz = 0;
    s_halow_pending_hint_bw_mhz = 0;
    s_halow_active_hint_freq_hz = 0;
    s_halow_active_hint_bw_mhz = 0;
    s_halow_bssid_lock_active = false;
    s_halow_auth_request_count = 0;
    s_halow_auth_request_total = 0;
    s_halow_scan_complete_count = 0;
    s_halow_unlock_retry_used = true;

    enum mmwlan_status status = mmwlan_sta_disable();
    if (status != MMWLAN_SUCCESS) {
        ESP_LOGW(TAG, "mmwlan_sta_disable before scan-fallback retry returned status=%d", status);
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    if (halow_set_full_regulatory_channel_list() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restore full regulatory channel list for fallback retry");
        s_halow_scan_fallback_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    struct mmwlan_sta_args sta_args = MMWLAN_STA_ARGS_INIT;
    sta_args.ssid_len = strlen(s_halow_retry_profile.ssid);
    memcpy(sta_args.ssid, s_halow_retry_profile.ssid, sta_args.ssid_len);
    sta_args.security_type = s_halow_retry_profile.security_type;
    sta_args.passphrase_len = s_halow_retry_profile.passphrase_len;
    if (sta_args.passphrase_len > 0) {
        memcpy(sta_args.passphrase, s_halow_retry_profile.passphrase, sta_args.passphrase_len);
    }
    sta_args.scan_rx_cb = NULL;
    sta_args.scan_rx_cb_arg = NULL;
    sta_args.sta_evt_cb = halow_sta_event_callback;
    sta_args.sta_evt_cb_arg = NULL;
    sta_args.scan_interval_base_s = UINT16_MAX;
    sta_args.scan_interval_limit_s = UINT16_MAX;

    uint8_t edgez_assoc_ie[EDGEZ_ASSOC_IE_MAX_LEN] = {0};
    if (s_startup_halow_mesh_mode) {
        halow_apply_mesh_sta_profile(&sta_args, true);
        esp_err_t ie_err = halow_attach_edgez_assoc_ie(&sta_args,
                                                       s_halow_retry_profile.edgez_ie_passphrase,
                                                       edgez_assoc_ie,
                                                       sizeof(edgez_assoc_ie));
        if (ie_err != ESP_OK) {
            ESP_LOGE(TAG, "Scan-fallback retry aborted: EdgeZ association IE unavailable");
            s_halow_scan_fallback_task_handle = NULL;
            vTaskDelete(NULL);
            return;
        }
    }

    s_halow_sta_enable_start_ms = mmosal_get_time_ms();
    s_halow_logged_first_scan_candidate = false;

    status = mmwlan_sta_enable(&sta_args, halow_sta_status_callback);
    if (status == MMWLAN_SUCCESS) {
        ESP_LOGI(TAG, "Scan-fallback retry started successfully");
    } else {
        ESP_LOGE(TAG, "Scan-fallback retry failed status=%d", status);
    }

    s_halow_scan_fallback_task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * Reset HaLow runtime connection state before a new connection attempt.
 */
static void halow_reset_provisioning_state(void)
{
    halow_sta_connected = false;
    halow_connection_attempts = 0;
    halow_first_connect_time = 0;
    s_halow_pending_hint_valid = false;
    s_halow_pending_hint_freq_hz = 0;
    s_halow_pending_hint_bw_mhz = 0;
    s_halow_active_hint_freq_hz = 0;
    s_halow_active_hint_bw_mhz = 0;
    s_halow_last_network_ready_ms = 0;
    s_halow_last_sleep_guard_log_ms = 0;
    s_halow_sta_enable_start_ms = 0;
    s_halow_logged_first_scan_candidate = false;
    s_halow_bssid_lock_active = false;
    s_halow_auth_request_count = 0;
    s_halow_auth_request_total = 0;
    s_halow_scan_complete_count = 0;
    s_halow_unlock_retry_used = false;
    s_halow_retry_profile.valid = false;
    if (s_halow_bssid_unlock_task_handle != NULL) {
        vTaskDelete(s_halow_bssid_unlock_task_handle);
        s_halow_bssid_unlock_task_handle = NULL;
    }
    if (s_halow_scan_fallback_task_handle != NULL) {
        vTaskDelete(s_halow_scan_fallback_task_handle);
        s_halow_scan_fallback_task_handle = NULL;
    }
    s_halow_connect_runtime.active = false;
    s_halow_connect_runtime.best_valid = false;
    s_halow_phy_link_up = false;
    s_halow_route_ready = false;
    s_halow_real_ip_event_posted = false;
    
    // Clean up any existing IP poll task from previous attempt
    if (halow_ip_poll_task_handle != NULL) {
        ESP_LOGI(TAG, "Cleaning up previous IP poll task");
        vTaskDelete(halow_ip_poll_task_handle);
        halow_ip_poll_task_handle = NULL;
    }

    if (halow_rx_sync_task_handle != NULL) {
        ESP_LOGI(TAG, "Cleaning up previous RX sync task");
        vTaskDelete(halow_rx_sync_task_handle);
        halow_rx_sync_task_handle = NULL;
    }

    s_halow_rx_sync_registered = false;
}

/* Forward declaration */
static void halow_link_status_callback(const struct mmipal_link_status *link_status);

static void halow_link_status_ext_callback(const struct mmipal_link_status *link_status, void *arg)
{
    (void)arg;
    halow_link_status_callback(link_status);
}

static void halow_register_link_status_callbacks(void)
{
    mmipal_set_link_status_callback(halow_link_status_callback);
    mmipal_set_ext_link_status_callback(halow_link_status_ext_callback, NULL);
}

static void halow_start_ip_poll_task_if_needed(const char *reason)
{
    if (halow_ip_poll_task_handle != NULL) {
        return;
    }

    BaseType_t created = xTaskCreate(halow_ip_poll_task,
                                     "halow_ip_poll",
                                     4096,
                                     NULL,
                                     5,
                                     &halow_ip_poll_task_handle);
    if (created == pdPASS) {
        ESP_LOGI(TAG, "Started IP poll task (%s)", reason ? reason : "unspecified");
    } else {
        ESP_LOGW(TAG, "Failed to start IP poll task (%s)", reason ? reason : "unspecified");
    }
}

static void halow_stop_ip_poll_task_if_running(const char *reason)
{
    if (halow_ip_poll_task_handle == NULL) {
        return;
    }

    ESP_LOGI(TAG, "Stopping IP poll task (%s)", reason ? reason : "unspecified");
    vTaskDelete(halow_ip_poll_task_handle);
    halow_ip_poll_task_handle = NULL;
}

static bool halow_wait_report_ready_after_wake(uint32_t timeout_ms)
{
    const TickType_t step = pdMS_TO_TICKS(HALOW_WAKE_READY_POLL_STEP_MS);
    TickType_t waited = 0;
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while (waited < timeout_ticks) {
        if (wifi_prov_halow_ready_for_report()) {
            return true;
        }
        vTaskDelay(step);
        waited += step;
    }

    return wifi_prov_halow_ready_for_report();
}

static bool halow_route_ready_probe(const char *target_ip)
{
    if (!halow_is_valid_ipv4_string(target_ip)) {
        return false;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGW(TAG, "Route probe socket create failed: errno=%d (%s)", errno, strerror(errno));
        return false;
    }

    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port = htons(9); /* discard port, only route resolution matters */
    if (inet_pton(AF_INET, target_ip, &dst.sin_addr) != 1) {
        close(sock);
        return false;
    }

    uint8_t probe = 0;
    int rc = sendto(sock, &probe, sizeof(probe), 0, (struct sockaddr *)&dst, sizeof(dst));
    int saved_errno = errno;
    close(sock);

    if (rc >= 0) {
        return true;
    }

    if (saved_errno == EHOSTUNREACH || saved_errno == ENETUNREACH) {
        return false;
    }

    return false;
}

static struct netif *halow_find_mmnetif(void)
{
    struct netif *target = NULL;

    LOCK_TCPIP_CORE();
    for (struct netif *walk = netif_list; walk != NULL; walk = walk->next)
    {
        if (walk->name[0] == 'M' && walk->name[1] == 'M')
        {
            target = walk;
            break;
        }

        if (walk->name[0] == 'm' && walk->name[1] == 'm')
        {
            target = walk;
            break;
        }
    }
    UNLOCK_TCPIP_CORE();

    return target;
}

static bool halow_try_restore_mmwlan_rx_binding(const char *reason, bool log_if_missing)
{
    if (s_halow_rx_sync_registered)
    {
        return true;
    }

    struct netif *mm_netif = halow_find_mmnetif();
    if (mm_netif == NULL)
    {
        if (log_if_missing)
        {
            ESP_LOGI(TAG,
                     "HaLow RX sync: MM netif not found yet (%s), will retry",
                     reason ? reason : "unspecified");
        }
        return false;
    }

    enum mmwlan_status rx_status =
        mmwlan_register_rx_pkt_ext_cb(MMWLAN_VIF_STA, halow_mmwlan_rx_to_lwip, mm_netif);
    if (rx_status != MMWLAN_SUCCESS)
    {
        ESP_LOGW(TAG,
                 "HaLow RX sync: failed to register RX callback (status=%d, reason=%s)",
                 (int)rx_status,
                 reason ? reason : "unspecified");
        return false;
    }

    s_halow_rx_sync_registered = true;
    ESP_LOGI(TAG, "HaLow RX sync: registered RX->lwIP callback (%s)", reason ? reason : "unspecified");
    return true;
}

static bool halow_mmnetif_ready_for_ip_config(const char *context)
{
    struct netif *target = halow_find_mmnetif();
    if (target == NULL)
    {
        ESP_LOGW(TAG, "%s: MM netif not found, deferring IP config",
                 context ? context : "HaLow");
        return false;
    }

    bool netif_up = false;
    bool link_up = false;

    LOCK_TCPIP_CORE();
    netif_up = netif_is_up(target);
    link_up = netif_is_link_up(target);
    UNLOCK_TCPIP_CORE();

    if (!netif_up || !link_up)
    {
        ESP_LOGW(TAG,
                 "%s: MM netif not ready for IP config (up=%d link=%d), deferring",
                 context ? context : "HaLow",
                 netif_up,
                 link_up);
        return false;
    }

    return true;
}

static void halow_log_mmipal_ip_state(const char *context)
{
    struct mmipal_ip_config ip_cfg = {0};
    enum mmipal_status ip_status = mmipal_get_ip_config(&ip_cfg);
    if (ip_status == MMIPAL_SUCCESS) {
        ESP_LOGI(TAG,
                 "%s: mmipal ip_mode=%d ip=%s nm=%s gw=%s",
                 context ? context : "HaLow",
                 (int)ip_cfg.mode,
                 ip_cfg.ip_addr,
                 ip_cfg.netmask,
                 ip_cfg.gateway_addr);
    } else {
        ESP_LOGW(TAG,
                 "%s: mmipal_get_ip_config failed (%d)",
                 context ? context : "HaLow",
                 (int)ip_status);
    }
}

static void halow_mmwlan_rx_to_lwip(struct mmpkt *rxpkt,
                                    const struct mmwlan_rx_metadata *metadata,
                                    void *arg)
{
    struct netif *netif = (struct netif *)arg;

    if (rxpkt == NULL)
    {
        return;
    }

    if (netif == NULL)
    {
        MM_MESH_DBG_PRINTF("[halow_rx] netif NULL, dropping\n");
        mmpkt_release(rxpkt);
        return;
    }

    struct mmpktview *pktview = mmpkt_open(rxpkt);
    if (pktview == NULL)
    {
        MM_MESH_DBG_PRINTF("[halow_rx] mmpkt_open NULL, dropping\n");
        mmpkt_release(rxpkt);
        return;
    }

    uint16_t payload_len = mmpkt_get_data_length(pktview);
    uint8_t *payload = mmpkt_get_data_start(pktview);

    /* Validate minimum ethernet frame size */
    if (payload_len < 14)
    {
        MM_MESH_DBG_PRINTF("[halow_rx] short frame len=%u, dropping\n", payload_len);
        mmpkt_close(&pktview);
        mmpkt_release(rxpkt);
        return;
    }

    if (halow_sync_bridge_handle_rx_frame(payload,
                                          14,
                                          payload + 14,
                                          payload_len - 14,
                                          metadata ? metadata->ta : NULL)) {
        mmpkt_close(&pktview);
        mmpkt_release(rxpkt);
        return;
    }

    struct pbuf *p = pbuf_alloc(PBUF_RAW, payload_len, PBUF_POOL);
    if (p == NULL)
    {
        MM_MESH_DBG_PRINTF("[halow_rx] pbuf_alloc FAILED len=%u\n", payload_len);
        mmpkt_close(&pktview);
        mmpkt_release(rxpkt);
        return;
    }

    err_t copy_err = pbuf_take(p, payload, payload_len);
    if (copy_err != ERR_OK)
    {
        MM_MESH_DBG_PRINTF("[halow_rx] pbuf_take FAILED err=%d\n", copy_err);
        pbuf_free(p);
        mmpkt_close(&pktview);
        mmpkt_release(rxpkt);
        return;
    }

    /* Snoop src-IP -> src-MAC for gateway ARP seeding (must happen before
     * tcpip_input since we hand ownership of the buffer to lwIP). */
    halow_src_mac_snoop_frame(payload, payload_len);

    err_t input_err = tcpip_input(p, netif);
    if (input_err != ERR_OK)
    {
        MM_MESH_DBG_PRINTF("[halow_rx] tcpip_input FAILED err=%d len=%u\n", input_err, payload_len);
        pbuf_free(p);
    }

    mmpkt_close(&pktview);
    mmpkt_release(rxpkt);
}

/* One-shot task: poll the snoop cache for the gateway IP and seed a static
 * ARP entry once the real MAC is observed.  Started from the link-up handler
 * when the cache does not yet contain the gateway. */
static void halow_seed_gateway_arp_task(void *arg)
{
    ip4_addr_t *gw_ip_p = (ip4_addr_t *)arg;
    if (gw_ip_p == NULL) {
        vTaskDelete(NULL);
        return;
    }
    ip4_addr_t gw_ip = *gw_ip_p;
    free(gw_ip_p);

    const int timeout_ms = 5000;
    const int step_ms = 200;
    int waited = 0;

    while (waited < timeout_ms) {
        struct eth_addr mac;
        if (halow_src_mac_lookup(gw_ip.addr, &mac)) {
            struct netif *mm_nif = halow_find_mmnetif();
            err_t arp_err = ERR_IF;
            if (mm_nif != NULL) {
                LOCK_TCPIP_CORE();
                arp_err = etharp_add_static_entry(&gw_ip, &mac);
                UNLOCK_TCPIP_CORE();
            }
            ESP_LOGI(TAG,
                     "Deferred static ARP for gateway " IPSTR
                     " -> %02x:%02x:%02x:%02x:%02x:%02x (observed after %d ms, err=%d)",
                     IP2STR(&gw_ip),
                     mac.addr[0], mac.addr[1], mac.addr[2],
                     mac.addr[3], mac.addr[4], mac.addr[5],
                     waited, (int)arp_err);
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        waited += step_ms;
    }

    ESP_LOGW(TAG,
             "Gateway " IPSTR " MAC not observed within %d ms; relying on lwIP dynamic ARP",
             IP2STR(&gw_ip), timeout_ms);
    vTaskDelete(NULL);
}

static void halow_restore_mmwlan_rx_binding(void)
{
    if (halow_try_restore_mmwlan_rx_binding("immediate", true)) {
        return;
    }

    if (!s_mmipal_initialized) {
        ESP_LOGI(TAG,
                 "HaLow RX sync: skip deferred retry because mmipal_init was skipped (MM netif not expected yet)");
        return;
    }

    halow_start_rx_sync_task_if_needed("deferred after WLAN boot");
}

static void halow_rx_sync_task(void *arg)
{
    (void)arg;

    const int poll_interval_ms = 200;
    const int timeout_ms = 10000;
    int elapsed_ms = 0;

    ESP_LOGI(TAG, "HaLow RX sync task started");

    while (elapsed_ms < timeout_ms)
    {
        if (halow_try_restore_mmwlan_rx_binding("retry task", false))
        {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(poll_interval_ms));
        elapsed_ms += poll_interval_ms;
    }

    if (!s_halow_rx_sync_registered)
    {
        ESP_LOGW(TAG, "HaLow RX sync task timed out waiting for MM netif");
    }

    halow_rx_sync_task_handle = NULL;
    vTaskDelete(NULL);
}

static void halow_start_rx_sync_task_if_needed(const char *reason)
{
    if (s_halow_rx_sync_registered || halow_rx_sync_task_handle != NULL)
    {
        return;
    }

    BaseType_t created = xTaskCreate(halow_rx_sync_task,
                                     "halow_rx_sync",
                                     3072,
                                     NULL,
                                     5,
                                     &halow_rx_sync_task_handle);
    if (created == pdPASS)
    {
        ESP_LOGI(TAG, "Started RX sync task (%s)", reason ? reason : "unspecified");
    }
    else
    {
        ESP_LOGW(TAG, "Failed to start RX sync task (%s)", reason ? reason : "unspecified");
    }
}

static void halow_set_mmnetif_link_state(bool link_up)
{
    struct netif *target = halow_find_mmnetif();

    if (target == NULL)
    {
        ESP_LOGW(TAG, "HaLow link sync: MM netif not found");
        return;
    }

    err_t err = tcpip_callback_with_block(
        (tcpip_callback_fn)(link_up ? netif_set_link_up : netif_set_link_down),
        target,
        0);
    if (err != ERR_OK)
    {
        ESP_LOGW(TAG, "HaLow link sync: failed to set MM netif link %s (err=%d)",
                 link_up ? "UP" : "DOWN", (int)err);
    }
    else
    {
        ESP_LOGI(TAG, "HaLow link sync: set MM netif link %s", link_up ? "UP" : "DOWN");
    }
}

static void halow_mmwlan_link_state_callback(enum mmwlan_link_state link_state, void *arg)
{
    (void)arg;

    (void)halow_try_restore_mmwlan_rx_binding("link-state callback", false);
    if (!s_halow_rx_sync_registered && s_mmipal_initialized) {
        halow_start_rx_sync_task_if_needed("link-state callback");
    }

    halow_set_mmnetif_link_state(link_state == MMWLAN_LINK_UP);

    if (link_state == MMWLAN_LINK_UP) {
        if (s_halow_wake_static_ip_active) {
            ESP_LOGI(TAG, "HaLow link sync: keeping wake static IP mode (skip DHCP re-arm)");
        } else {
            if (!halow_mmnetif_ready_for_ip_config("HaLow link sync")) {
                halow_start_ip_poll_task_if_needed("link up with deferred DHCP re-arm");
                return;
            }

            struct mmipal_ip_config ip_cfg = {0};
            ip_cfg.mode = MMIPAL_DHCP;
            enum mmipal_status status = mmipal_set_ip_config(&ip_cfg);
            if (status == MMIPAL_SUCCESS) {
                ESP_LOGI(TAG, "HaLow link sync: re-armed DHCP after link UP");
            } else {
                ESP_LOGW(TAG, "HaLow link sync: failed to re-arm DHCP after link UP (status=%d)", (int)status);
            }
        }
    }

    if (link_state == MMWLAN_LINK_DOWN) {
        struct mmipal_link_status link_status;
        memset(&link_status, 0, sizeof(link_status));
        link_status.link_state = MMIPAL_LINK_DOWN;
        halow_link_status_callback(&link_status);
    }
}

static bool halow_configure_dhcp_after_wake(const char *reason)
{
    if (!halow_mmnetif_ready_for_ip_config("HaLow wake DHCP")) {
        return false;
    }

    struct mmipal_ip_config ip_cfg = {0};
    ip_cfg.mode = MMIPAL_DHCP;

    enum mmipal_status status = mmipal_set_ip_config(&ip_cfg);
    if (status == MMIPAL_SUCCESS) {
        s_halow_wake_static_ip_active = false;
        ESP_LOGI(TAG, "HaLow wake: configured mmipal for DHCP (%s)", reason ? reason : "unspecified");
        return true;
    }

    ESP_LOGW(TAG, "HaLow wake: mmipal_set_ip_config(DHCP) failed (%s): %d",
             reason ? reason : "unspecified", (int)status);
    return false;
}

static bool halow_configure_cached_static_ip_after_wake(void)
{
    if (!s_halow_cached_ip.valid ||
        !halow_is_valid_ipv4_string(s_halow_cached_ip.ip_addr) ||
        !halow_is_valid_ipv4_string(s_halow_cached_ip.netmask) ||
        !halow_is_valid_ipv4_string(s_halow_cached_ip.gateway)) {
        return false;
    }

    if (!halow_mmnetif_ready_for_ip_config("HaLow wake static")) {
        return false;
    }

    struct mmipal_ip_config ip_cfg = {0};
    ip_cfg.mode = MMIPAL_STATIC;
    strncpy(ip_cfg.ip_addr, s_halow_cached_ip.ip_addr, sizeof(ip_cfg.ip_addr) - 1);
    ip_cfg.ip_addr[sizeof(ip_cfg.ip_addr) - 1] = '\0';
    strncpy(ip_cfg.netmask, s_halow_cached_ip.netmask, sizeof(ip_cfg.netmask) - 1);
    ip_cfg.netmask[sizeof(ip_cfg.netmask) - 1] = '\0';
    strncpy(ip_cfg.gateway_addr, s_halow_cached_ip.gateway, sizeof(ip_cfg.gateway_addr) - 1);
    ip_cfg.gateway_addr[sizeof(ip_cfg.gateway_addr) - 1] = '\0';

    enum mmipal_status status = mmipal_set_ip_config(&ip_cfg);
    if (status == MMIPAL_SUCCESS) {
        s_halow_wake_static_ip_active = true;
        ESP_LOGI(TAG,
                 "HaLow wake: restored cached static IP ip=%s nm=%s gw=%s",
                 ip_cfg.ip_addr,
                 ip_cfg.netmask,
                 ip_cfg.gateway_addr);
        return true;
    }

    ESP_LOGW(TAG, "HaLow wake: static IP restore failed: %d", (int)status);
    s_halow_wake_static_ip_active = false;
    return false;
}

static void halow_refresh_ip_network_after_wake(void)
{
    halow_register_link_status_callbacks();

    enum mmipal_status hook_status = mmipal_rehook_lwip_callbacks();
    if (hook_status == MMIPAL_SUCCESS) {
        ESP_LOGI(TAG, "HaLow wake: re-hooked MMIPAL lwIP callbacks");
    } else if (hook_status != MMIPAL_NOT_SUPPORTED && hook_status != MMIPAL_NO_LINK) {
        ESP_LOGW(TAG, "HaLow wake: failed to re-hook MMIPAL lwIP callbacks: %d", (int)hook_status);
    }

    if (!halow_configure_cached_static_ip_after_wake()) {
        (void)halow_configure_dhcp_after_wake("cached static unavailable");
    }
}

/**
 * Task to poll for IP address when mmipal_init() was skipped.
 * When esp_netif_init() initializes lwIP before mmipal, the mmipal link status
 * callback won't fire because mmipal's DHCP client isn't running. This task
 * polls mmipal_get_ip_config() to detect when IP is assigned and manually
 * triggers the link status callback.
 */
static void halow_ip_poll_task(void *arg)
{
    const int POLL_INTERVAL_MS = 500;
    const int MAX_POLL_TIME_MS = 60000;  // 60 seconds max
    int elapsed_ms = 0;
    
    ESP_LOGI(TAG, "Starting IP address poll task");
    
    while (elapsed_ms < MAX_POLL_TIME_MS) {
        if (halow_sta_connected && s_halow_route_ready) {
            ESP_LOGI(TAG, "IP poll task: route already ready from callbacks, exiting");
            break;
        }

        struct mmipal_ip_config ip_config = {0};
        enum mmipal_status status = mmipal_get_ip_config(&ip_config);
        bool has_valid_ip = (status == MMIPAL_SUCCESS &&
                             halow_is_valid_ipv4_string(ip_config.ip_addr));
        bool has_valid_network = (has_valid_ip &&
                                  halow_is_valid_ipv4_string(ip_config.netmask) &&
                                  halow_is_valid_ipv4_string(ip_config.gateway_addr));

        if (status == MMIPAL_SUCCESS && !has_valid_network && (elapsed_ms % 5000 == 0)) {
            ESP_LOGW(TAG,
                     "Ignoring incomplete/invalid DHCP snapshot: ip=%s nm=%s gw=%s",
                     ip_config.ip_addr,
                     ip_config.netmask,
                     ip_config.gateway_addr);
        }

        if (has_valid_network) {
            enum mmwlan_sta_state sta_state = mmwlan_get_sta_state();
            bool sta_ready = (sta_state == MMWLAN_STA_CONNECTED);
            if (s_startup_halow_mesh_mode && halow_sta_connected) {
                /* Mesh mode can bypass the normal CONNECTED callback and
                 * only signal readiness via CTRL_PORT_OPEN. */
                sta_ready = true;
            }
            if (!sta_ready) {
                if (elapsed_ms % 5000 == 0) {
                    ESP_LOGI(TAG,
                             "IP present (%s) but HaLow STA not connected yet; waiting before marking network ready",
                             ip_config.ip_addr);
                }
                vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
                elapsed_ms += POLL_INTERVAL_MS;
                continue;
            }

            if (!halow_route_ready_probe(ip_config.gateway_addr)) {
                if (elapsed_ms % 5000 == 0) {
                    ESP_LOGI(TAG,
                             "IP present (%s) but route to gateway %s not ready; waiting",
                             ip_config.ip_addr,
                             ip_config.gateway_addr);
                }
                vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
                elapsed_ms += POLL_INTERVAL_MS;
                continue;
            }

            s_halow_route_ready = true;
            
            ESP_LOGI(TAG, "IP address obtained via polling: %s", ip_config.ip_addr);
            
            // Manually construct and invoke the link status callback
            struct mmipal_link_status link_status;
            memset(&link_status, 0, sizeof(link_status));
            link_status.link_state = MMIPAL_LINK_UP;
            strncpy(link_status.ip_addr, ip_config.ip_addr, sizeof(link_status.ip_addr) - 1);
            strncpy(link_status.netmask, ip_config.netmask, sizeof(link_status.netmask) - 1);
            strncpy(link_status.gateway, ip_config.gateway_addr, sizeof(link_status.gateway) - 1);
            
            halow_link_status_callback(&link_status);
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
        elapsed_ms += POLL_INTERVAL_MS;
        
        if (elapsed_ms % 5000 == 0) {
            ESP_LOGI(TAG, "Still waiting for IP address... (%d seconds)", elapsed_ms / 1000);
            halow_log_mmipal_ip_state("IP poll");
        }
    }
    
    if (elapsed_ms >= MAX_POLL_TIME_MS) {
        ESP_LOGW(TAG, "IP address poll timed out after %d seconds", MAX_POLL_TIME_MS / 1000);
    }
    
    halow_ip_poll_task_handle = NULL;
    vTaskDelete(NULL);
}

/* mesh_etharp_output removed: the old implementation forced ALL unicast ARP
 * resolution through the gateway IP, causing eth_dst to always be the relay
 * MAC.  This broke 802.11s mesh addressing because the proxied destination
 * (Addr5) in the mesh header became the relay's own MAC instead of the real
 * destination, so the relay consumed every frame locally instead of forwarding.
 *
 * With standard etharp_output, lwIP resolves each destination IP to its real
 * MAC via ARP.  The mesh proxy table (in umac_datapath.c) then correctly
 * populates A3 (mesh DA) from incoming AE=2 frames, and Addr5 carries the
 * real destination MAC.  The static ARP entry for the gateway IP is kept
 * so that off-subnet traffic still routes through the relay. */

/* HaLow link status callback (from mmipal - includes IP configuration) */
static void halow_link_status_callback(const struct mmipal_link_status *link_status)
{
    uint32_t time_ms = mmosal_get_time_ms();

    if (!link_status) {
        return;
    }

    s_halow_phy_link_up = (link_status->link_state == MMIPAL_LINK_UP);
    
    if (link_status->link_state == MMIPAL_LINK_UP) {
        bool has_valid_ipv4 = halow_is_valid_ipv4_string(link_status->ip_addr);
        bool duplicate_up_report = has_valid_ipv4 &&
                                   s_halow_last_link_up_report_valid &&
                                   strncmp(s_halow_last_link_up_report_ip,
                                           link_status->ip_addr,
                                           sizeof(s_halow_last_link_up_report_ip)) == 0;

        if (!duplicate_up_report) {
            ESP_LOGI(TAG, "HaLow link is UP at %lu ms", time_ms);
            ESP_LOGI(TAG, "  IP Address: %s", link_status->ip_addr);
            ESP_LOGI(TAG, "  Netmask:    %s", link_status->netmask);
            ESP_LOGI(TAG, "  Gateway:    %s", link_status->gateway);
        }

        if (has_valid_ipv4) {
            s_halow_route_ready = true;
            halow_cache_valid_ip_config(link_status);
            s_halow_last_network_ready_ms = time_ms;
            strncpy(s_halow_last_link_up_report_ip,
                    link_status->ip_addr,
                    sizeof(s_halow_last_link_up_report_ip) - 1);
            s_halow_last_link_up_report_ip[sizeof(s_halow_last_link_up_report_ip) - 1] = '\0';
            s_halow_last_link_up_report_valid = true;

            /* mesh_etharp_output override removed — standard etharp_output
             * is used so lwIP resolves real destination MACs via ARP.
             * The mesh proxy table handles A3 addressing. */
            struct netif *mm_nif = halow_find_mmnetif();

            /* Pre-populate the ARP table with a static entry for the gateway.
             *
             * Use the gateway MAC observed on the wire (DHCP/ARP/ICMP src MAC
             * cache) rather than the radio peer's BSSID.  The BSSID is the
             * mesh next-hop, which may be a relay that does NOT L3-forward
             * for us; if we set eth_da = relay MAC, the relay sees A3/Addr5
             * = its own MAC and consumes the frame instead of forwarding.
             *
             * The radio next-hop (A1/RA) is handled automatically by mmwlan
             * from the peer association — we do not need to set it. */
            if (mm_nif) {
                ip4_addr_t gw_ip;
                if (ip4addr_aton(link_status->gateway, &gw_ip) && gw_ip.addr != 0) {
                    struct eth_addr observed_gw_mac;
                    bool have_observed = halow_src_mac_lookup(gw_ip.addr, &observed_gw_mac);

                    if (have_observed) {
                        LOCK_TCPIP_CORE();
                        err_t arp_err = etharp_add_static_entry(&gw_ip, &observed_gw_mac);
                        UNLOCK_TCPIP_CORE();
                        ESP_LOGI(TAG,
                                 "Static ARP for gateway " IPSTR " -> %02x:%02x:%02x:%02x:%02x:%02x (observed, err=%d)",
                                 IP2STR(&gw_ip),
                                 observed_gw_mac.addr[0], observed_gw_mac.addr[1],
                                 observed_gw_mac.addr[2], observed_gw_mac.addr[3],
                                 observed_gw_mac.addr[4], observed_gw_mac.addr[5],
                                 (int)arp_err);
                    } else {
                        /* No observation yet — kick a normal ARP request so
                         * lwIP can resolve it dynamically and our snoop
                         * cache picks up the reply.  Spawn a deferred task
                         * to seed the static entry once observed. */
                        LOCK_TCPIP_CORE();
                        err_t req_err = etharp_request(mm_nif, &gw_ip);
                        UNLOCK_TCPIP_CORE();
                        ESP_LOGI(TAG,
                                 "No observed MAC for gateway " IPSTR " yet; sent ARP request (err=%d)",
                                 IP2STR(&gw_ip), (int)req_err);

                        ip4_addr_t *gw_ip_arg = malloc(sizeof(ip4_addr_t));
                        if (gw_ip_arg != NULL) {
                            *gw_ip_arg = gw_ip;
                            if (xTaskCreate(halow_seed_gateway_arp_task,
                                            "halow_arp_seed",
                                            3072,
                                            gw_ip_arg,
                                            5,
                                            NULL) != pdPASS) {
                                ESP_LOGW(TAG, "Failed to start halow_arp_seed task");
                                free(gw_ip_arg);
                            }
                        }
                    }

                    /* Announce our IP/MAC via gratuitous ARP broadcast.
                     *
                     * In multi-hop mesh (ESP32 → relay → mesh gate) the
                     * static ARP above uses the relay's MAC.  The relay
                     * L3-forwards our traffic, changing the Ethernet SA to
                     * its own MAC.  The mesh gate bridge therefore never
                     * learns that our MAC is reachable on its mesh port,
                     * so return traffic (server → gateway → mesh gate →
                     * ??? → us) cannot be forwarded.
                     *
                     * A gratuitous ARP broadcast traverses the full mesh
                     * path (ESP32 → relay → mesh gate) with our real MAC
                     * as the 802.11s source address.  This causes:
                     *  1) Mesh gate bridge FDB learns our MAC → mesh port
                     *  2) Gateway updates its ARP cache for our IP → MAC
                     * so return unicast traffic can find us. */
                    if (s_startup_halow_mesh_mode) {
                        LOCK_TCPIP_CORE();
                        etharp_gratuitous(mm_nif);
                        UNLOCK_TCPIP_CORE();
                        ESP_LOGI(TAG,
                                 "Sent gratuitous ARP for mesh bridge FDB learning");
                    }

                    /* Enable ARP offload on the Morse Micro chip so the
                     * gateway's ARP entry for us does not expire.
                     * 1) ARP response offload: chip auto-replies to ARP
                     *    requests for our IP.
                     * 2) ARP refresh offload: chip periodically sends
                     *    gratuitous ARPs to the gateway every 15 s. */
                    ip4_addr_t our_ip;
                    if (ip4addr_aton(link_status->ip_addr, &our_ip) &&
                        our_ip.addr != 0) {
                        enum mmwlan_status arp_resp_rc =
                            mmwlan_enable_arp_response_offload(
                                ip4_addr_get_u32(&our_ip));
                        ESP_LOGI(TAG,
                                 "ARP response offload: rc=%d (ip=" IPSTR ")",
                                 (int)arp_resp_rc, IP2STR(&our_ip));

                        enum mmwlan_status arp_ref_rc =
                            mmwlan_enable_arp_refresh_offload(
                                15, ip4_addr_get_u32(&gw_ip), true);
                        ESP_LOGI(TAG,
                                 "ARP refresh offload: rc=%d (interval=15s gw=" IPSTR " garp=true)",
                                 (int)arp_ref_rc, IP2STR(&gw_ip));
                    }
                }
            }

            if (!s_halow_real_ip_event_posted) {
                ip_event_got_ip_t evt = {0};
                bool parsed = true;
                ip4_addr_t ip_addr = {0};
                ip4_addr_t netmask = {0};
                ip4_addr_t gateway = {0};
                parsed = parsed && (ip4addr_aton(link_status->ip_addr, &ip_addr) != 0);
                parsed = parsed && (ip4addr_aton(link_status->netmask, &netmask) != 0);
                parsed = parsed && (ip4addr_aton(link_status->gateway, &gateway) != 0);
                evt.ip_info.ip.addr = ip_addr.addr;
                evt.ip_info.netmask.addr = netmask.addr;
                evt.ip_info.gw.addr = gateway.addr;
                evt.ip_changed = true;

                if (!parsed) {
                    ESP_LOGW(TAG,
                             "HaLow real IP callback has invalid IPv4 strings, delaying provisioning success event");
                } else {
                    esp_err_t err = esp_event_post(IP_EVENT,
                                                   IP_EVENT_STA_GOT_IP,
                                                   &evt,
                                                   sizeof(evt),
                                                   pdMS_TO_TICKS(100));
                    if (err == ESP_OK) {
                        s_halow_real_ip_event_posted = true;
                        ESP_LOGI(TAG,
                                 "HaLow real IP event posted for provisioning success: %s",
                                 link_status->ip_addr);
                    } else {
                        ESP_LOGW(TAG,
                                 "Failed to post HaLow real IP event: %s",
                                 esp_err_to_name(err));
                    }
                }
            }

        } else {
            s_halow_route_ready = false;
            s_halow_last_network_ready_ms = 0;
            s_halow_last_link_up_report_valid = false;
            s_halow_last_link_up_report_ip[0] = '\0';
        }
        
        halow_sta_connected = true;
        s_connection_type = EDGEZ_TRANSPORT_HALOW;
        
        if (halow_link_up_semaphore) {
            mmosal_semb_give(halow_link_up_semaphore);
        }
        
        // Signal to main WiFi event group
        if (wifi_event_group) {
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
            if (s_halow_route_ready) {
                xEventGroupSetBits(wifi_event_group, WIFI_NETWORK_READY_EVENT);
            } else {
                xEventGroupClearBits(wifi_event_group, WIFI_NETWORK_READY_EVENT);
            }
        }

        if (has_valid_ipv4) {
            if (!duplicate_up_report) {
                ESP_LOGI(TAG, "HaLow has real IP address now: %s", link_status->ip_addr);
            }
        } else {
            ESP_LOGW(TAG, "HaLow link is up but IPv4 is not assigned yet");
        }
    } else {
        ESP_LOGI(TAG, "HaLow link is DOWN at %lu ms", time_ms);
        halow_sta_connected = false;
        s_halow_last_network_ready_ms = 0;
        s_halow_route_ready = false;
        s_halow_real_ip_event_posted = false;
        s_halow_last_link_up_report_valid = false;
        s_halow_last_link_up_report_ip[0] = '\0';
        if (wifi_event_group) {
            xEventGroupClearBits(wifi_event_group, WIFI_NETWORK_READY_EVENT);
        }
    }
}

/* HaLow STA status callback */
static void halow_sta_status_callback(enum mmwlan_sta_state sta_state)
{
    const char *sta_state_desc[] = {
        "DISABLED",
        "CONNECTING",
        "CONNECTED",
    };
    uint32_t current_time = mmosal_get_time_ms();
    
    if (sta_state == MMWLAN_STA_CONNECTING) {
        ESP_LOGI(TAG,
                 "HaLow timing: CONNECTING callback at +%lu ms from sta_enable",
                 (unsigned long)(current_time - s_halow_sta_enable_start_ms));
        if (halow_connection_attempts == 0) {
            halow_first_connect_time = current_time;
        }
        halow_connection_attempts++;
        
        uint32_t elapsed_sec = (current_time - halow_first_connect_time) / 1000;
        ESP_LOGI(TAG, "HaLow STA state: %s (attempt #%lu, %lu seconds elapsed)", 
                 sta_state_desc[sta_state], 
                 (unsigned long)halow_connection_attempts,
                 (unsigned long)elapsed_sec);
        
        // Provide hints after multiple attempts
        if (halow_connection_attempts > 5) {
            ESP_LOGW(TAG, "Connection taking longer than expected. Possible issues:");
            ESP_LOGW(TAG, "  1. Wrong security type (AP may use WPA2-PSK which is NOT supported)");
            ESP_LOGW(TAG, "  2. Incorrect passphrase");
            ESP_LOGW(TAG, "  3. AP is out of range or signal too weak");
            ESP_LOGW(TAG, "  4. Channel/bandwidth mismatch");
        }
        
        if (halow_connection_attempts > 10) {
            ESP_LOGE(TAG, "Too many connection attempts (%lu). Connection likely will not succeed.",
                     (unsigned long)halow_connection_attempts);
            ESP_LOGE(TAG, "Consider checking AP settings and trying OPEN or different security type");
        }
    } else if (sta_state == MMWLAN_STA_CONNECTED) {
        ESP_LOGI(TAG,
                 "HaLow timing: CONNECTED callback at +%lu ms from sta_enable",
                 (unsigned long)(current_time - s_halow_sta_enable_start_ms));
        ESP_LOGI(TAG, "HaLow STA state: %s (after %lu attempts)", 
                 sta_state_desc[sta_state],
                 (unsigned long)halow_connection_attempts);
        halow_connection_attempts = 0;  // Reset counter
        s_halow_bssid_lock_active = false;
        s_halow_auth_request_count = 0;
        s_halow_active_hint_freq_hz = 0;
        s_halow_active_hint_bw_mhz = 0;

        if (s_halow_connect_runtime.active && s_halow_connect_runtime.best_valid) {
            const uint8_t *persist_bssid =
                s_halow_unlock_retry_used ? NULL : s_halow_connect_runtime.best_bssid;
            if (s_halow_unlock_retry_used) {
                ESP_LOGW(TAG,
                         "HaLow connected after unlock-retry; saving fast hint without BSSID lock");
            } else if (s_halow_auth_request_total > 1) {
                persist_bssid = NULL;
                ESP_LOGW(TAG,
                         "HaLow auth needed %u AUTH_REQUEST events; saving fast hint without BSSID lock",
                         (unsigned)s_halow_auth_request_total);
            }
            halow_save_fast_hint(
                s_halow_connect_runtime.target_ssid,
                s_halow_connect_runtime.best_freq_hz,
                s_halow_connect_runtime.best_bw_mhz,
                persist_bssid);
        }
        s_halow_auth_request_total = 0;
        s_halow_unlock_retry_used = false;
        s_halow_connect_runtime.active = false;

        // Mark link up even if the mmipal link callback is not firing
        halow_sta_connected = true;
        s_connection_type = EDGEZ_TRANSPORT_HALOW;

        if (s_mmipal_initialized) {
            enum mmipal_status hook_status = mmipal_rehook_lwip_callbacks();
            if (hook_status == MMIPAL_SUCCESS) {
                ESP_LOGI(TAG, "HaLow connect: re-hooked MMIPAL lwIP callbacks");
            } else if (hook_status != MMIPAL_NOT_SUPPORTED && hook_status != MMIPAL_NO_LINK) {
                ESP_LOGW(TAG, "HaLow connect: failed to re-hook MMIPAL callbacks (%d)", (int)hook_status);
            }

            if (!s_halow_wake_static_ip_active) {
                struct mmipal_ip_config ip_cfg = {0};
                ip_cfg.mode = MMIPAL_DHCP;
                enum mmipal_status dhcp_status = mmipal_set_ip_config(&ip_cfg);
                if (dhcp_status == MMIPAL_SUCCESS) {
                    ESP_LOGI(TAG, "HaLow connect: re-armed MMIPAL DHCP on STA_CONNECTED");
                } else {
                    ESP_LOGW(TAG, "HaLow connect: failed to re-arm MMIPAL DHCP (%d)", (int)dhcp_status);
                }
            }

            halow_log_mmipal_ip_state("HaLow connect");
        }

        // Signal connectivity to the main app / provisioning flow
        if (wifi_event_group) {
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
        }
        
        // Don't stop provisioning here. Wait for real HaLow IP assignment and route-ready
        // in halow_link_status_callback before posting IP_EVENT_STA_GOT_IP.
        
        // IMPORTANT: Stop ESP32 WiFi IMMEDIATELY when HaLow connects to avoid coexistence
        // issues that prevent HaLow DHCP from working. We do this BEFORE posting the fake
        // IP event so the WiFi driver doesn't interfere with HaLow networking.
        ESP_LOGI(TAG, "Stopping ESP32 WiFi to allow HaLow DHCP to work");
        esp_wifi_stop();
        
        // When mmipal_init() was skipped (because esp_netif_init() already initialized lwIP),
        // the mmipal link status callback won't fire automatically. Start a polling task
        // to detect when IP address is assigned and manually trigger the callback.
        halow_start_ip_poll_task_if_needed("STA connected");
        
        // Create queue and task for stats logging if not already created
        if (halow_stats_queue == NULL) {
            halow_stats_queue = xQueueCreate(2, sizeof(uint32_t));
        }
        //if (halow_stats_task_handle == NULL && halow_stats_queue != NULL) {
        //    xTaskCreate(halow_stats_task, "halow_stats_task", 4096, NULL, 5, &halow_stats_task_handle);
        //}
        // Start periodic timer (every 5 seconds)
        if (halow_stats_timer == NULL) {
            halow_stats_timer = xTimerCreate("halow_stats", pdMS_TO_TICKS(5000), pdTRUE, NULL, halow_stats_timer_callback);
        }
        if (halow_stats_timer != NULL) {
            xTimerStart(halow_stats_timer, 0);
            ESP_LOGI(TAG, "Started TX power stats timer (every 5 seconds)");
        }
    } else {
        ESP_LOGI(TAG, "HaLow STA state: %s (%u)", sta_state_desc[sta_state], sta_state);
        halow_connection_attempts = 0;  // Reset counter
        
        // Stop stats timer when disconnected
        if (halow_stats_timer != NULL) {
            xTimerStop(halow_stats_timer, 0);
        }
    }
}

/* Timer callback: send message to stats queue */
static void halow_stats_timer_callback(TimerHandle_t xTimer)
{
    uint32_t msg = 1;
    if (halow_stats_queue != NULL) {
        xQueueSend(halow_stats_queue, &msg, 0);
    }
}

/* tcpip_callback target: send gratuitous ARP from lwIP thread */
static void halow_mesh_garp_cb(void *arg)
{
    struct netif *nif = (struct netif *)arg;
    if (nif != NULL && netif_is_link_up(nif) &&
        !ip4_addr_isany_val(*netif_ip4_addr(nif))) {
        etharp_gratuitous(nif);
    }
}

/* Dedicated task for stats logging */
static void halow_stats_task(void *arg)
{
    uint32_t msg;
    while (1) {
        if (xQueueReceive(halow_stats_queue, &msg, portMAX_DELAY) == pdTRUE) {
            halow_log_tx_power_stats();

            /* In mesh mode, periodically send gratuitous ARP so that
             * the mesh gate bridge FDB keeps our MAC → mesh-port mapping
             * alive and the gateway refreshes its ARP cache for our IP.
             * Without this, return traffic stops after the FDB entry
             * ages out (typically 30–300 s). */
            if (s_startup_halow_mesh_mode && s_halow_route_ready) {
                struct netif *mm_nif = halow_find_mmnetif();
                if (mm_nif != NULL) {
                    tcpip_callback_with_block(halow_mesh_garp_cb, mm_nif, 0);
                }
            }
        }
    }
}

/* Helper function to check if SSID is a HaLow AP (has bandwidth suffix) */
static bool is_halow_ssid(const char *ssid)
{
    // Check if SSID ends with "(X MHz)" pattern
    size_t len = strlen(ssid);
    if (len < 8) return false;  // Minimum: "X(X MHz)"
    
    // Look for " MHz)" at the end
    if (strcmp(&ssid[len - 5], " MHz)") == 0) {
        // Find opening parenthesis
        for (int i = len - 6; i >= 0; i--) {
            if (ssid[i] == '(') {
                return true;
            }
        }
    }
    return false;
}

/* Remove bandwidth suffix from HaLow SSID to get original SSID */
static void remove_bandwidth_suffix(const char *halow_ssid, char *original_ssid, size_t max_len)
{
    size_t len = strlen(halow_ssid);
    
    // Find opening parenthesis
    for (int i = len - 1; i >= 0; i--) {
        if (halow_ssid[i] == '(') {
            // Trim trailing spaces before the parenthesis
            int end_pos = i;
            while (end_pos > 0 && halow_ssid[end_pos - 1] == ' ') {
                end_pos--;
            }
            
            size_t copy_len = end_pos < max_len ? end_pos : max_len - 1;
            memcpy(original_ssid, halow_ssid, copy_len);
            original_ssid[copy_len] = '\0';
            return;
        }
    }
    
    // Fallback: copy as-is if no parenthesis found
    strncpy(original_ssid, halow_ssid, max_len - 1);
    original_ssid[max_len - 1] = '\0';
}

/* Connect to HaLow AP using mmwlan */
static esp_err_t halow_sta_connect(const char *ssid, const char *password)
{
    enum mmwlan_status status;
    char original_ssid[MMWLAN_SSID_MAXLEN + 1];
    bool used_fast_hint = false;
    bool is_sleep_wake_reconnect = (s_halow_light_sleep_active && s_halow_light_sleep_shutdown);
    halow_fast_hint_t fast_hint = {0};

    if (!halow_country_code_is_configured()) {
        ESP_LOGE(TAG,
                 "Refusing HaLow connect for SSID '%s': country code is empty",
                 ssid ? ssid : "(null)");
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (s_power_transition_in_progress) {
        ESP_LOGW(TAG, "Skipping HaLow connect: deep-sleep power transition in progress");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Remove bandwidth suffix to get original SSID
    remove_bandwidth_suffix(ssid, original_ssid, sizeof(original_ssid));
    
    ESP_LOGI(TAG, "=== HaLow Connection Details ===");
    ESP_LOGI(TAG, "Original SSID with suffix: %s", ssid);
    ESP_LOGI(TAG, "SSID after removing suffix: %s", original_ssid);
    ESP_LOGI(TAG, "Password: %s", password ? password : "(none)");
    ESP_LOGI(TAG, "SSID length: %d", strlen(original_ssid));
    ESP_LOGI(TAG, "Password length: %d", password ? strlen(password) : 0);
    ESP_LOGI(TAG, "================================");
    
    // Create semaphore if not exists
    if (!halow_link_up_semaphore) {
        halow_link_up_semaphore = mmosal_semb_create("halow_link");
        if (!halow_link_up_semaphore) {
            ESP_LOGE(TAG, "Failed to create HaLow link semaphore");
            return ESP_FAIL;
        }
    }

    if (s_startup_halow_mesh_mode &&
        s_startup_halow_mesh_channel > 0 &&
        s_startup_halow_mesh_bandwidth_mhz > 0) {
        uint32_t factory_hint_freq_hz = 0;
        bool factory_hint_ok = halow_lookup_freq_for_channel(
            s_startup_halow_mesh_channel,
            s_startup_halow_mesh_bandwidth_mhz,
            &factory_hint_freq_hz);

        if (factory_hint_ok) {
            s_halow_pending_hint_valid = true;
            s_halow_pending_hint_freq_hz = factory_hint_freq_hz;
            s_halow_pending_hint_bw_mhz = s_startup_halow_mesh_bandwidth_mhz;
            s_halow_active_hint_freq_hz = factory_hint_freq_hz;
            s_halow_active_hint_bw_mhz = s_startup_halow_mesh_bandwidth_mhz;
            used_fast_hint = true;
            ESP_LOGI(TAG,
                     "Factory mesh hint applied: channel=%u bw=%uMHz freq=%lu",
                     (unsigned)s_startup_halow_mesh_channel,
                     (unsigned)s_startup_halow_mesh_bandwidth_mhz,
                     (unsigned long)factory_hint_freq_hz);
        } else {
            ESP_LOGW(TAG,
                     "Factory mesh hint not found in regdb: channel=%u bw=%uMHz country=%s",
                     (unsigned)s_startup_halow_mesh_channel,
                     (unsigned)s_startup_halow_mesh_bandwidth_mhz,
                     s_halow_country_code);
            s_halow_pending_hint_valid = false;
            s_halow_pending_hint_freq_hz = 0;
            s_halow_pending_hint_bw_mhz = 0;
            s_halow_active_hint_freq_hz = 0;
            s_halow_active_hint_bw_mhz = 0;
        }
    } else if (is_sleep_wake_reconnect && halow_load_fast_hint_for_ssid(original_ssid, &fast_hint)) {
        s_halow_pending_hint_valid = true;
        s_halow_pending_hint_freq_hz = fast_hint.channel_freq_hz;
        s_halow_pending_hint_bw_mhz = fast_hint.bw_mhz;
        s_halow_active_hint_freq_hz = fast_hint.channel_freq_hz;
        s_halow_active_hint_bw_mhz = fast_hint.bw_mhz;
        used_fast_hint = true;
        ESP_LOGI(TAG,
                 "Wake reconnect: loaded fast HaLow hint (freq=%lu bw=%u)",
                 (unsigned long)s_halow_pending_hint_freq_hz,
                 (unsigned)s_halow_pending_hint_bw_mhz);
    } else if (!is_sleep_wake_reconnect) {
        ESP_LOGI(TAG, "Normal connect path: fast hint bypass disabled (wake-only)");
        s_halow_pending_hint_valid = false;
        s_halow_pending_hint_freq_hz = 0;
        s_halow_pending_hint_bw_mhz = 0;
        s_halow_active_hint_freq_hz = 0;
        s_halow_active_hint_bw_mhz = 0;
    } else {
        s_halow_pending_hint_valid = false;
        s_halow_pending_hint_freq_hz = 0;
        s_halow_pending_hint_bw_mhz = 0;
        s_halow_active_hint_freq_hz = 0;
        s_halow_active_hint_bw_mhz = 0;
    }
    
    // Ensure HaLow is initialized
    if (halow_init() != ESP_OK) {
        ESP_LOGE(TAG, "HaLow initialization failed");
        return ESP_FAIL;
    }

    status = mmwlan_set_power_save_mode(MMWLAN_PS_DISABLED);
    if (status != MMWLAN_SUCCESS) {
        ESP_LOGW(TAG, "Failed to disable HaLow power save mode: status %d", status);
    } else {
        ESP_LOGI(TAG, "HaLow power save mode disabled");
    }
    
    // Register link status callback (mmipal callback includes IP configuration from DHCP)
    halow_register_link_status_callbacks();
    
    // Set up STA arguments
    struct mmwlan_sta_args sta_args = MMWLAN_STA_ARGS_INIT;
    sta_args.ssid_len = strlen(original_ssid);
    memcpy(sta_args.ssid, original_ssid, sta_args.ssid_len);
    sta_args.scan_rx_cb = NULL;
    sta_args.scan_rx_cb_arg = NULL;
    sta_args.sta_evt_cb = halow_sta_event_callback;
    sta_args.sta_evt_cb_arg = NULL;
    sta_args.scan_interval_base_s = UINT16_MAX;
    sta_args.scan_interval_limit_s = UINT16_MAX;

    if (s_startup_halow_mesh_mode) {
        halow_apply_mesh_sta_profile(&sta_args, false);
    }

    if (used_fast_hint) {
        bool allow_bssid_lock = true;
#if !HALOW_WAKE_ALLOW_WIDEBW_BSSID_LOCK
        if (is_sleep_wake_reconnect && fast_hint.bw_mhz >= 4) {
            allow_bssid_lock = false;
            ESP_LOGI(TAG,
                     "Wake reconnect: skipping BSSID lock for wide-band hint bw=%u to avoid primary-channel lock-in",
                     (unsigned)fast_hint.bw_mhz);
        }
#else
        if (is_sleep_wake_reconnect && fast_hint.bw_mhz >= 4) {
            ESP_LOGI(TAG,
                     "Wake reconnect: wide-band BSSID lock experiment enabled (bw=%u)",
                     (unsigned)fast_hint.bw_mhz);
        }
#endif

        if (allow_bssid_lock && halow_bssid_is_nonzero(fast_hint.bssid)) {
            memcpy(sta_args.bssid, fast_hint.bssid, sizeof(sta_args.bssid));
            s_halow_bssid_lock_active = true;
            s_halow_auth_request_count = 0;
            ESP_LOGI(TAG,
                     "Using fast-hint BSSID lock: %02x:%02x:%02x:%02x:%02x:%02x",
                     fast_hint.bssid[0], fast_hint.bssid[1], fast_hint.bssid[2],
                     fast_hint.bssid[3], fast_hint.bssid[4], fast_hint.bssid[5]);
            ESP_LOGI(TAG, "Applying fast HaLow reconnect hint (pre-boot channel subset + BSSID lock)");
        } else {
            s_halow_bssid_lock_active = false;
            ESP_LOGI(TAG, "Applying fast HaLow reconnect hint (pre-boot channel subset, no BSSID lock)");
        }
    }
    
    // Determine security type based on password presence
    if (password && strlen(password) > 0) {
        sta_args.passphrase_len = strlen(password);
        memcpy(sta_args.passphrase, password, sta_args.passphrase_len);
        
        // Try SAE (WPA3) first as it's the most common for HaLow APs with passwords
        // This matches the behavior of the Morse Micro iperf example
        sta_args.security_type = MMWLAN_SAE;
        ESP_LOGI(TAG, "Attempting connection with SAE (WPA3) security");
        ESP_LOGI(TAG, "SSID: %s, Passphrase length: %d", original_ssid, sta_args.passphrase_len);
    } else {
        sta_args.security_type = MMWLAN_OPEN;
        ESP_LOGI(TAG, "Using OPEN security (no password)");
    }

    if (s_startup_halow_mesh_mode) {
        halow_apply_mesh_identity_profile(&sta_args, original_ssid, password);
    }

    memset(&s_halow_connect_runtime, 0, sizeof(s_halow_connect_runtime));
    s_halow_connect_runtime.active = true;
    s_halow_unlock_retry_used = false;
    strncpy(s_halow_connect_runtime.target_ssid,
            (const char *)sta_args.ssid,
            sizeof(s_halow_connect_runtime.target_ssid) - 1);

    memset(&s_halow_retry_profile, 0, sizeof(s_halow_retry_profile));
    strncpy(s_halow_retry_profile.ssid,
            (const char *)sta_args.ssid,
            sizeof(s_halow_retry_profile.ssid) - 1);
    s_halow_retry_profile.security_type = sta_args.security_type;
    s_halow_retry_profile.passphrase_len = sta_args.passphrase_len;
    if (s_halow_retry_profile.passphrase_len > 0) {
        memcpy(s_halow_retry_profile.passphrase,
               sta_args.passphrase,
               s_halow_retry_profile.passphrase_len);
    }
    const char *edgez_ie_passphrase = password ? password : "";
    size_t edgez_ie_passphrase_len = strlen(edgez_ie_passphrase);
    if (edgez_ie_passphrase_len > MMWLAN_PASSPHRASE_MAXLEN) {
        edgez_ie_passphrase_len = MMWLAN_PASSPHRASE_MAXLEN;
    }
    s_halow_retry_profile.edgez_ie_passphrase_len = (uint16_t)edgez_ie_passphrase_len;
    if (edgez_ie_passphrase_len > 0) {
        memcpy(s_halow_retry_profile.edgez_ie_passphrase,
               edgez_ie_passphrase,
               edgez_ie_passphrase_len);
    }
    s_halow_retry_profile.valid = true;

    uint8_t edgez_assoc_ie[EDGEZ_ASSOC_IE_MAX_LEN] = {0};
    if (s_startup_halow_mesh_mode) {
        halow_apply_mesh_sta_profile(&sta_args, true);
        esp_err_t ie_err = halow_attach_edgez_assoc_ie(&sta_args,
                                                       edgez_ie_passphrase,
                                                       edgez_assoc_ie,
                                                       sizeof(edgez_assoc_ie));
        if (ie_err != ESP_OK) {
            s_halow_connect_runtime.active = false;
            ESP_LOGE(TAG, "HaLow mesh connect aborted: EdgeZ association IE unavailable");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Factory mode=mesh: enabling HaLow mesh_mode connect path");
    }

    ESP_LOGI(TAG,
             "HaLow connect profile: mesh_mode=%u sta_type=%u scan_base=%u scan_limit=%u ssid=%s",
             (unsigned)sta_args.mesh_mode,
             (unsigned)sta_args.sta_type,
             (unsigned)sta_args.scan_interval_base_s,
             (unsigned)sta_args.scan_interval_limit_s,
             (const char *)sta_args.ssid);
    
    // Enable verbose connection logging
    s_halow_sta_enable_start_ms = mmosal_get_time_ms();
    s_halow_logged_first_scan_candidate = false;
    s_halow_auth_request_total = 0;
    ESP_LOGI(TAG, "Calling mmwlan_sta_enable with security type: %d", sta_args.security_type);
    status = mmwlan_sta_enable(&sta_args, halow_sta_status_callback);
    if (status != MMWLAN_SUCCESS && used_fast_hint) {
        ESP_LOGW(TAG, "Fast hint connect failed (status %d), retrying with full channel list", status);
        if (halow_set_full_regulatory_channel_list() == ESP_OK) {
            status = mmwlan_sta_enable(&sta_args, halow_sta_status_callback);
        }
    }
    if (status != MMWLAN_SUCCESS) {
        s_halow_connect_runtime.active = false;
        s_halow_active_hint_freq_hz = 0;
        s_halow_active_hint_bw_mhz = 0;
        ESP_LOGE(TAG, "Failed to enable HaLow STA mode: status %d", status);
        ESP_LOGE(TAG, "Supported security types: OPEN, OWE, SAE/WPA3 (WPA2-PSK NOT supported)");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "mmwlan_sta_enable returned SUCCESS");
    ESP_LOGI(TAG, "HaLow timing: sta_enable API returned in %lu ms",
             (unsigned long)(mmosal_get_time_ms() - s_halow_sta_enable_start_ms));
    
    ESP_LOGI(TAG, "HaLow STA enabled, waiting for connection...");
    return ESP_OK;
}

/**
 * Deferred HaLow connection worker task.
 * Runs halow_sta_connect() outside the event loop so heavy init (mmhal_init,
 * mmwlan_boot, mmipal_init) doesn't block the event dispatcher.
 */
static void halow_connect_worker_task(void *arg)
{
    (void)arg;
    halow_connect_request_t req = {0};

    for (;;) {
        if (xQueueReceive(s_halow_connect_queue, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (s_power_transition_in_progress) {
            ESP_LOGW(TAG, "Dropping deferred HaLow connect request during power transition");
            continue;
        }

        ESP_LOGI(TAG, "Deferred HaLow connect worker received request");
        esp_err_t err = halow_sta_connect(req.ssid, req.password);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to connect to HaLow AP");
        } else {
            ESP_LOGI(TAG, "HaLow provisioning initiated (WiFi will timeout naturally)");
        }
    }
}

static esp_err_t halow_connect_worker_init(void)
{
    if (s_halow_connect_queue != NULL && s_halow_connect_task_handle != NULL) {
        return ESP_OK;
    }

    if (s_halow_connect_queue == NULL) {
        s_halow_connect_queue = xQueueCreate(1, sizeof(halow_connect_request_t));
        if (s_halow_connect_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create HaLow connect queue");
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_halow_connect_task_handle == NULL) {
        BaseType_t created = xTaskCreate(
            halow_connect_worker_task,
            "halow_conn",
            8192,
            NULL,
            5,
            &s_halow_connect_task_handle
        );
        if (created != pdPASS) {
            ESP_LOGE(TAG, "Failed to create HaLow connect worker task");
            vQueueDelete(s_halow_connect_queue);
            s_halow_connect_queue = NULL;
            return ESP_ERR_NO_MEM;
        }
    }

    return ESP_OK;
}

#ifdef CONFIG_ENABLE_MM_HALOW
/* Return the highest regulatory EIRP (dBm) for the configured country code. */
static uint16_t halow_max_eirp_dbm(const struct mmwlan_s1g_channel_list *channel_list)
{
    if (!channel_list || !channel_list->channels || channel_list->num_channels == 0) {
        return 0;
    }

    int8_t max_dbm = channel_list->channels[0].max_tx_eirp_dbm;
    for (uint32_t i = 1; i < channel_list->num_channels; i++) {
        if (channel_list->channels[i].max_tx_eirp_dbm > max_dbm) {
            max_dbm = channel_list->channels[i].max_tx_eirp_dbm;
        }
    }

    return max_dbm > 0 ? (uint16_t)max_dbm : 0;
}

/* Query current TX power statistics from rate control */
static void halow_log_tx_power_stats(void)
{
    // Get RSSI (signal strength from AP)
    int32_t rssi = mmwlan_get_rssi();
    
    // Get rate control stats
    struct mmwlan_rc_stats *stats = mmwlan_get_rc_stats();
    if (stats == NULL) {
        ESP_LOGW(TAG, "Failed to get rate control stats");
        return;
    }

    ESP_LOGI(TAG, "=== HaLow Link & Rate Statistics ===");
    ESP_LOGI(TAG, "RSSI: %ld dBm", (long)rssi);
    ESP_LOGI(TAG, "Note: TX power is dynamically controlled by chip based on regulatory limits");
    ESP_LOGI(TAG, "Active transmission rates (out of %lu total):", (unsigned long)stats->n_entries);
    
    uint32_t active_count = 0;
    for (uint32_t i = 0; i < stats->n_entries && i < 20; i++) {
        if (stats->total_sent[i] > 0) {
            uint32_t rate_info = stats->rate_info[i];
            uint32_t bw = (rate_info >> 0) & 0xF;  // Bits 0-3
            uint32_t mcs = (rate_info >> 4) & 0xF; // Bits 4-7
            uint32_t gi = (rate_info >> 8) & 0x1;  // Bit 8
            
            const char *bw_str = (bw == 0) ? "1MHz" : (bw == 1) ? "2MHz" : (bw == 2) ? "4MHz" : "8MHz";
            const char *gi_str = (gi == 0) ? "LGI" : "SGI";
            
            uint32_t success_rate = (stats->total_success[i] * 100) / stats->total_sent[i];
            ESP_LOGI(TAG, "  MCS%lu %s %s: Sent=%lu Success=%lu (%lu%%)",
                     (unsigned long)mcs, bw_str, gi_str,
                     (unsigned long)stats->total_sent[i],
                     (unsigned long)stats->total_success[i],
                     (unsigned long)success_rate);
            active_count++;
        }
    }
    
    if (active_count == 0) {
        ESP_LOGI(TAG, "  (No packets transmitted yet)");
    }
    
    mmwlan_free_rc_stats(stats);
    ESP_LOGI(TAG, "====================================");
}
#endif

/* Initialize HaLow subsystem */
static esp_err_t halow_init(void)
{
    if (!halow_country_code_is_configured()) {
        ESP_LOGW(TAG, "Skipping HaLow init: country code is empty");
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (s_power_transition_in_progress) {
        ESP_LOGW(TAG, "Skipping HaLow init: power transition in progress");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_halow_stack_initialized) {
        return ESP_OK;  // Already initialized
    }
    
    ESP_LOGI(TAG, "HaLow country before MM IoT init: %s", s_halow_country_code);
    ESP_LOGI(TAG, "Initializing MM HaLow subsystem");
    
    ESP_LOGI(TAG, "Calling mmhal_init...");
    fflush(stdout);
    uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);
    
    // Initialize Morse subsystems
    mmhal_init();
    ESP_LOGI(TAG, "mmhal_init completed");
    vTaskDelay(pdMS_TO_TICKS(10));  // Yield between heavy init calls
    
    mmwlan_init();
    vTaskDelay(pdMS_TO_TICKS(10));  // Yield between heavy init calls

    if (s_halow_pending_hint_valid) {
        if (halow_set_hint_channel_list_from_freq(s_halow_pending_hint_freq_hz, s_halow_pending_hint_bw_mhz)) {
            ESP_LOGI(TAG, "Applied hint-centered channel list before interface start");
        } else {
            ESP_LOGW(TAG, "Failed to apply hint-centered channel list, using full regulatory list");
            if (halow_set_full_regulatory_channel_list() != ESP_OK) {
                return ESP_FAIL;
            }
        }
    } else {
        if (halow_set_full_regulatory_channel_list() != ESP_OK) {
            return ESP_FAIL;
        }
    }

    s_halow_pending_hint_valid = false;
    s_halow_pending_hint_freq_hz = 0;
    s_halow_pending_hint_bw_mhz = 0;

    const struct mmwlan_s1g_channel_list* channel_list;
    channel_list = mmwlan_lookup_regulatory_domain(get_regulatory_db(), s_halow_country_code);
    if (channel_list == NULL) {
        ESP_LOGE(TAG, "Could not find regulatory domain for country code %s", s_halow_country_code);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Regulatory domain set to %s", s_halow_country_code);
    
    // Log regulatory max EIRP for information
    uint16_t max_eirp_dbm = halow_max_eirp_dbm(channel_list);
    ESP_LOGI(TAG, "Regulatory max EIRP: %u dBm", max_eirp_dbm);
    
    // MMIPAL must be initialized at least once per boot to create/register the MM lwIP netif.
    // If lwIP is already initialized by esp_netif, attach MMIPAL netif to existing lwIP
    // instead of calling tcpip_init() again.
    if (!s_mmipal_initialized) {
        struct mmipal_init_args mmipal_args = MMIPAL_INIT_ARGS_DEFAULT;
        enum mmipal_status mmipal_status;

        if (g_lwip_initialized_by_esp_netif) {
            ESP_LOGI(TAG, "Attaching mmipal to existing lwIP (esp_netif already initialized)");
            mmipal_status = mmipal_init_on_existing_lwip(&mmipal_args);
        } else {
            ESP_LOGI(TAG, "Initializing mmipal (lwIP network stack + WLAN boot)...");
            mmipal_status = mmipal_init(&mmipal_args);
        }

        if (mmipal_status == MMIPAL_SUCCESS) {
            s_mmipal_initialized = true;
            ESP_LOGI(TAG, "mmipal initialized successfully - MM netif ready");
        } else {
            if (!g_lwip_initialized_by_esp_netif) {
                ESP_LOGE(TAG, "Failed to initialize mmipal: %d", (int)mmipal_status);
                return ESP_FAIL;
            }

            ESP_LOGW(TAG,
                     "mmipal_init failed with pre-initialized lwIP (status=%d), "
                     "falling back to manual WLAN boot",
                     (int)mmipal_status);

            ESP_LOGI(TAG, "Booting WLAN (this may take 10-20 seconds)...");
            struct mmwlan_boot_args boot_args = MMWLAN_BOOT_ARGS_INIT;
            enum mmwlan_status wlan_status = mmwlan_boot(&boot_args);
            if (wlan_status != MMWLAN_SUCCESS) {
                ESP_LOGE(TAG, "Failed to boot WLAN: %d", wlan_status);
                return ESP_FAIL;
            }
            ESP_LOGI(TAG, "WLAN booted successfully");

            halow_restore_mmwlan_rx_binding();

            enum mmwlan_status link_cb_status =
                mmwlan_register_link_state_cb(halow_mmwlan_link_state_callback, NULL);
            if (link_cb_status != MMWLAN_SUCCESS) {
                ESP_LOGW(TAG, "Failed to register mmwlan link-state callback after WLAN boot: %d",
                         (int)link_cb_status);
            }
        }
    }
    
    // Note: mmipal_init() boots WLAN internally, so we don't need to call mmwlan_boot()
    s_halow_stack_initialized = true;

    /* Re-register callback after every init cycle so wake-from-light-sleep
     * full unload/reinit has a valid link-status hook. */
    halow_register_link_status_callbacks();

    ESP_LOGI(TAG, "HaLow subsystem initialized successfully");
    
    return ESP_OK;
}

static esp_err_t halow_set_full_regulatory_channel_list(void)
{
    const struct mmwlan_s1g_channel_list *channel_list =
        mmwlan_lookup_regulatory_domain(get_regulatory_db(), s_halow_country_code);
    if (channel_list == NULL) {
        ESP_LOGE(TAG, "Could not find regulatory domain for country code %s", s_halow_country_code);
        return ESP_FAIL;
    }

    enum mmwlan_status status = mmwlan_set_channel_list(channel_list);
    if (status != MMWLAN_SUCCESS) {
        ESP_LOGE(TAG, "Failed to set country code %s", channel_list->country_code);
        return ESP_FAIL;
    }

    return ESP_OK;
}

#endif  // CONFIG_ENABLE_MM_HALOW

esp_err_t wifi_prov_init_and_start(void)
{
#ifdef CONFIG_ENABLE_MM_HALOW
    if (!factory_data_license_authorize(EDGEZ_LICENSE_CAP_PROVISION)) {
        ESP_LOGW(TAG, "HaLow provisioning disabled: device is not licensed");
        return ESP_ERR_NOT_ALLOWED;
    }
#endif

#ifdef CONFIG_ENABLE_MM_HALOW
    s_power_transition_in_progress = false;
#endif

    if (wifi_event_group == NULL) {
        wifi_event_group = xEventGroupCreate();
        if (wifi_event_group == NULL) {
            ESP_LOGE(TAG, "Failed to create event group");
            return ESP_FAIL;
        }
    }

    /* Ensure startup waits observe fresh connection/readiness transitions */
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_EVENT | WIFI_NETWORK_READY_EVENT);

    esp_err_t event_err = ensure_prov_event_handlers_registered();
    if (event_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register network event handlers: %s", esp_err_to_name(event_err));
        return event_err;
    }

#ifdef CONFIG_ENABLE_MM_HALOW
    esp_err_t halow_init_err = halow_interface_app_init();
    if (halow_init_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize HaLow interface adapter: %s", esp_err_to_name(halow_init_err));
        return halow_init_err;
    }

#endif

    s_wifi_provisioned_cached = false;
    s_wifi_connect_inflight = false;

    ESP_LOGI(TAG, "Network bootstrap ready; waiting for runtime HaLow credentials");

    return ESP_OK;
}

static esp_err_t wifi_prov_start_halow_profile(const char *ssid,
                                               const char *password,
                                               bool beacon_only)
{
#ifdef CONFIG_ENABLE_MM_HALOW
    if (!ssid || ssid[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    if (!halow_country_code_is_configured()) {
        ESP_LOGE(TAG, "Cannot connect HaLow: country code is empty");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t init_err = wifi_prov_init_and_start();
    if (init_err != ESP_OK) {
        return init_err;
    }

    if (strlen(ssid) >= sizeof(s_active_halow_ssid) ||
        (password && strlen(password) >= sizeof(s_active_halow_password))) {
        return ESP_ERR_INVALID_SIZE;
    }

    strncpy(s_active_halow_ssid, ssid, sizeof(s_active_halow_ssid) - 1);
    s_active_halow_ssid[sizeof(s_active_halow_ssid) - 1] = '\0';
    strncpy(s_active_halow_password, password ? password : "", sizeof(s_active_halow_password) - 1);
    s_active_halow_password[sizeof(s_active_halow_password) - 1] = '\0';
    s_active_halow_credentials_valid = true;

    halow_interface_init_config_t init = {0};
    strlcpy(init.country_code, s_halow_country_code, sizeof(init.country_code));
    strlcpy(init.mesh_id, s_active_halow_ssid, sizeof(init.mesh_id));
    strlcpy(init.passphrase, s_active_halow_password, sizeof(init.passphrase));
    init.user_id_high = s_halow_user_identity.user_id_high;
    init.user_id_low = s_halow_user_identity.user_id_low;
    strlcpy(init.user_name, s_halow_user_identity.user_name, sizeof(init.user_name));
    init.user_public_key_len = s_halow_user_identity.user_public_key_len;
    if (init.user_public_key_len > sizeof(init.user_public_key)) {
        init.user_public_key_len = sizeof(init.user_public_key);
    }
    if (init.user_public_key_len > 0) {
        memcpy(init.user_public_key,
               s_halow_user_identity.user_public_key,
               init.user_public_key_len);
    }
    init.max_hop = s_halow_max_hop;
    init.mesh_frequency_khz = s_runtime_halow_mesh_frequency_khz;
    init.mesh_bandwidth_mhz = s_runtime_halow_mesh_bandwidth_mhz;
    init.device_type = (uint32_t)s_halow_user_identity.device_type;
    init.beacon_only = beacon_only;

    ESP_LOGI(TAG,
             "Runtime HaLow credentials accepted; starting HaLow interface beacon_only=%u",
             beacon_only ? 1U : 0U);
    esp_err_t err = halow_interface_app_start(&init);
    if (err == ESP_OK) {
        s_connection_type = EDGEZ_TRANSPORT_HALOW;
        if (wifi_event_group) {
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT | WIFI_NETWORK_READY_EVENT);
        }
    }
    return err;
#else
    (void)ssid;
    (void)password;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t wifi_prov_connect_halow(const char *ssid, const char *password)
{
    return wifi_prov_start_halow_profile(ssid, password, false);
}

esp_err_t wifi_prov_start_halow_beacon_only(const char *ssid, const char *password)
{
    return wifi_prov_start_halow_profile(ssid, password, true);
}

esp_err_t wifi_prov_connect_upstream_wifi(const char *ssid, const char *passphrase)
{
    if (!ssid || ssid[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    size_t ssid_len = strlen(ssid);
    size_t pass_len = passphrase ? strlen(passphrase) : 0;
    if (ssid_len > 32 || pass_len > 64) {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "upstream Wi-Fi esp_netif_init failed: %s", esp_err_to_name(err));
        return err;
    }
#ifdef CONFIG_ENABLE_MM_HALOW
    g_lwip_initialized_by_esp_netif = true;
#endif

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "upstream Wi-Fi event loop init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = wifi_prov_init_and_start();
    if (err != ESP_OK) {
        return err;
    }

    if (!s_upstream_wifi_netif) {
        s_upstream_wifi_netif = esp_netif_create_default_wifi_sta();
        if (!s_upstream_wifi_netif) {
            ESP_LOGW(TAG, "upstream Wi-Fi STA netif create failed");
            return ESP_FAIL;
        }
    }

    if (!s_upstream_wifi_driver_initialized) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        err = esp_wifi_init(&cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "upstream Wi-Fi driver init failed: %s", esp_err_to_name(err));
            return err;
        }
        s_upstream_wifi_driver_initialized = true;
    }

    wifi_config_t wifi_cfg = {0};
    memcpy(wifi_cfg.sta.ssid, ssid, ssid_len);
    if (pass_len > 0) {
        memcpy(wifi_cfg.sta.password, passphrase, pass_len);
    }
    wifi_cfg.sta.threshold.authmode = (pass_len == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    wifi_cfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "upstream Wi-Fi set storage failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "upstream Wi-Fi set mode failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "upstream Wi-Fi set config failed: %s", esp_err_to_name(err));
        return err;
    }

    if (!s_upstream_wifi_started) {
        err = esp_wifi_start();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "upstream Wi-Fi start failed: %s", esp_err_to_name(err));
            return err;
        }
        s_upstream_wifi_started = true;
    } else {
        esp_err_t disconnect_err = esp_wifi_disconnect();
        if (disconnect_err != ESP_OK &&
            disconnect_err != ESP_ERR_WIFI_NOT_STARTED &&
            disconnect_err != ESP_ERR_WIFI_NOT_CONNECT) {
            ESP_LOGW(TAG, "upstream Wi-Fi disconnect before reconnect failed: %s",
                     esp_err_to_name(disconnect_err));
        }
    }

    s_wifi_connect_inflight = false;
    ESP_LOGI(TAG, "upstream Wi-Fi STA connect requested ssid=%s passphrase=%s",
             ssid,
             pass_len > 0 ? "set" : "open");
    return request_wifi_sta_connect("device-upstream");
}

esp_err_t wifi_prov_set_wifi_credentials(const char *ssid,
                                         const char *passphrase,
                                         bool connect_after_set)
{
    if (!ssid || ssid[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    size_t ssid_len = strlen(ssid);
    size_t pass_len = passphrase ? strlen(passphrase) : 0;
    if (ssid_len > 32 || pass_len > 64) {
        return ESP_ERR_INVALID_SIZE;
    }

#ifdef CONFIG_ENABLE_MM_HALOW
    ESP_LOGI(TAG,
             "Runtime credentials received via BLE/USB; starting HaLowInterface%s (mesh_id=%s)",
             connect_after_set ? "" : " without explicit reconnect request",
             ssid);
    return wifi_prov_connect_halow(ssid, passphrase);
#endif

    wifi_config_t wifi_cfg = {0};
    memcpy(wifi_cfg.sta.ssid, ssid, ssid_len);
    if (pass_len > 0) {
        memcpy(wifi_cfg.sta.password, passphrase, pass_len);
    }
    wifi_cfg.sta.threshold.authmode = (pass_len == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    wifi_cfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    if (err != ESP_OK) {
#ifdef CONFIG_ENABLE_MM_HALOW
        if (err == ESP_ERR_WIFI_NOT_INIT) {
            ESP_LOGI(TAG, "ESP Wi-Fi is not initialized; applying credentials to runtime HaLow");
            return wifi_prov_connect_halow(ssid, passphrase);
        }
#endif
        ESP_LOGW(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(err));
        return err;
    }

    s_wifi_provisioned_cached = true;
    ESP_LOGI(TAG, "Wi-Fi STA credentials updated from USB control (ssid=%s)", ssid);

    if (connect_after_set) {
        if (wifi_event_group) {
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_EVENT | WIFI_NETWORK_READY_EVENT);
        }
        s_wifi_connect_inflight = false;
        esp_err_t disconnect_err = esp_wifi_disconnect();
        if (disconnect_err != ESP_OK &&
            disconnect_err != ESP_ERR_WIFI_NOT_STARTED &&
            disconnect_err != ESP_ERR_WIFI_NOT_CONNECT) {
            ESP_LOGW(TAG, "esp_wifi_disconnect before USB reconnect failed: %s",
                     esp_err_to_name(disconnect_err));
        }
        return request_wifi_sta_connect("usb-control");
    }

#ifdef CONFIG_ENABLE_MM_HALOW
    if (s_connection_type == EDGEZ_TRANSPORT_HALOW) {
        return wifi_prov_connect_halow(ssid, passphrase);
    }
#endif

    return ESP_OK;
}

esp_err_t wifi_prov_wait_connected(void)
{
    if (wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Event group not initialized");
        return ESP_FAIL;
    }

    /* Wait for Wi-Fi connection */
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, false, true, portMAX_DELAY);
    return ESP_OK;
}

esp_err_t wifi_prov_wait_connected_timeout(uint32_t timeout_ms)
{
    if (wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Event group not initialized");
        return ESP_FAIL;
    }

    /* Wait for Wi-Fi connection with timeout */
    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group, 
        WIFI_CONNECTED_EVENT, 
        false,  /* Keep bit latched */
        true,  /* Wait for all bits */
        pdMS_TO_TICKS(timeout_ms)
    );

    if (bits & WIFI_CONNECTED_EVENT) {
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Wi-Fi connection timeout after %lu ms", (unsigned long)timeout_ms);
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t wifi_prov_wait_network_ready_timeout(uint32_t timeout_ms)
{
    if (wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Event group not initialized");
        return ESP_FAIL;
    }

    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_NETWORK_READY_EVENT,
        false,  /* Keep bit latched */
        true,  /* Wait for all bits */
        pdMS_TO_TICKS(timeout_ms)
    );

    if (bits & WIFI_NETWORK_READY_EVENT) {
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Network readiness timeout after %lu ms", (unsigned long)timeout_ms);
    return ESP_ERR_TIMEOUT;
}

esp_err_t wifi_prov_get_active_gateway_ipv4(char *out_gateway, size_t out_gateway_len)
{
    if (!out_gateway || out_gateway_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    out_gateway[0] = '\0';

#ifdef CONFIG_ENABLE_MM_HALOW
    struct mmipal_ip_config ip_cfg = {0};
    if (mmipal_get_ip_config(&ip_cfg) == MMIPAL_SUCCESS &&
        halow_is_valid_ipv4_string(ip_cfg.gateway_addr)) {
        int written = snprintf(out_gateway, out_gateway_len, "%s", ip_cfg.gateway_addr);
        if (written <= 0 || (size_t)written >= out_gateway_len) {
            return ESP_ERR_INVALID_SIZE;
        }
        return ESP_OK;
    }

    if (s_halow_cached_ip.valid && halow_is_valid_ipv4_string(s_halow_cached_ip.gateway)) {
        int written = snprintf(out_gateway, out_gateway_len, "%s", s_halow_cached_ip.gateway);
        if (written <= 0 || (size_t)written >= out_gateway_len) {
            return ESP_ERR_INVALID_SIZE;
        }
        return ESP_OK;
    }
#endif

    esp_netif_t *netif = esp_netif_next_unsafe(NULL);
    while (netif != NULL) {
        esp_netif_ip_info_t ip_info = {0};
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.gw.addr != 0) {
            int written = snprintf(out_gateway,
                                   out_gateway_len,
                                   IPSTR,
                                   IP2STR(&ip_info.gw));
            if (written <= 0 || (size_t)written >= out_gateway_len) {
                return ESP_ERR_INVALID_SIZE;
            }
            return ESP_OK;
        }
        netif = esp_netif_next_unsafe(netif);
    }

    return ESP_ERR_NOT_FOUND;
}

EventGroupHandle_t wifi_prov_get_event_group(void)
{
    return wifi_event_group;
}

bool wifi_prov_ble_session_active(void)
{
    return false;
}

bool wifi_prov_service_active(void)
{
    return false;
}

bool wifi_prov_recent_success(uint32_t window_ms)
{
    (void)window_ms;
    return false;
}

bool wifi_prov_has_bootstrap_credentials(void)
{
#ifdef CONFIG_ENABLE_MM_HALOW
    if (s_active_halow_credentials_valid) {
        return true;
    }
#endif

    return false;
}

bool wifi_is_provisioned(void)
{
    /* Check if WiFi credentials are already configured in NVS */
    wifi_config_t wifi_cfg = {0};
    esp_err_t ret = esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg);
    if (ret == ESP_OK) {
        s_wifi_provisioned_cached = (strlen((const char *)wifi_cfg.sta.ssid) > 0);
        return s_wifi_provisioned_cached;
    }

    return s_wifi_provisioned_cached;
}

esp_err_t wifi_prov_prepare_for_deep_sleep(void)
{
    ESP_LOGI(TAG, "Preparing network stacks for deep sleep");

#ifdef CONFIG_ENABLE_MM_HALOW
    s_power_transition_in_progress = true;
#endif

#ifdef CONFIG_ENABLE_MM_HALOW
    if (s_connection_type == EDGEZ_TRANSPORT_HALOW ||
        halow_interface_app_connection_type() == EDGEZ_TRANSPORT_HALOW) {
        wifi_prov_shutdown_halow_for_sleep();
    }
#endif

    esp_err_t first_err = ESP_OK;

    esp_err_t stop_err = esp_wifi_stop();
    if (stop_err != ESP_OK &&
        stop_err != ESP_ERR_WIFI_NOT_INIT &&
        stop_err != ESP_ERR_WIFI_NOT_STARTED) {
        ESP_LOGW(TAG, "esp_wifi_stop before deep sleep failed: %s", esp_err_to_name(stop_err));
        first_err = stop_err;
    } else {
        s_upstream_wifi_started = false;
    }

    esp_err_t deinit_err = esp_wifi_deinit();
    if (deinit_err != ESP_OK &&
        deinit_err != ESP_ERR_WIFI_NOT_INIT &&
        deinit_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "esp_wifi_deinit before deep sleep failed: %s", esp_err_to_name(deinit_err));
        if (first_err == ESP_OK) {
            first_err = deinit_err;
        }
    } else {
        s_upstream_wifi_driver_initialized = false;
    }

    if (wifi_event_group) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_EVENT | WIFI_NETWORK_READY_EVENT);
    }

    return first_err;
}

edgez_transport_type_t wifi_prov_get_connection_type(void)
{
#ifdef CONFIG_ENABLE_MM_HALOW
    return halow_interface_app_connection_type();
#else
    return EDGEZ_TRANSPORT_WIFI;
#endif
}

esp_err_t wifi_prov_start_ble_reprovisioning(void)
{
    ESP_LOGW(TAG, "Wi-Fi provisioning has been removed");
    return ESP_ERR_NOT_SUPPORTED;
}

#ifdef CONFIG_ENABLE_MM_HALOW
void wifi_prov_enter_halow_light_sleep(void)
{
    if (halow_interface_app_connection_type() != EDGEZ_TRANSPORT_HALOW) {
        return;
    }

    if (s_halow_light_sleep_active) {
        return;
    }

    ESP_LOGI(TAG, "HaLow light sleep enter: shutting down HaLowInterface adapter");
    (void)halow_interface_app_shutdown(false);
    s_halow_light_sleep_active = true;
    s_halow_light_sleep_shutdown = true;
}

esp_err_t wifi_prov_exit_halow_light_sleep(void)
{
    if (!s_halow_light_sleep_active) {
        return ESP_OK;
    }

    esp_err_t wake_status = ESP_OK;

    if (s_halow_light_sleep_shutdown) {
        if (wifi_event_group) {
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_EVENT | WIFI_NETWORK_READY_EVENT);
        }

        if (!s_active_halow_credentials_valid || s_active_halow_ssid[0] == '\0') {
            ESP_LOGW(TAG, "HaLow light sleep exit: no BLE-supplied credentials available for reconnect");
            wake_status = ESP_ERR_INVALID_STATE;
        } else {
            ESP_LOGI(TAG, "HaLow light sleep exit: restarting HaLowInterface adapter");
            wake_status = wifi_prov_connect_halow(s_active_halow_ssid, s_active_halow_password);
            if (wake_status == ESP_OK) {
            }
        }
    }

    s_halow_light_sleep_active = false;
    s_halow_light_sleep_shutdown = false;
    return wake_status;
}

bool wifi_prov_halow_ready_for_light_sleep(void)
{
    if (halow_interface_app_connection_type() != EDGEZ_TRANSPORT_HALOW) {
        return true;
    }

    return wifi_prov_halow_ready_for_report();
}

bool wifi_prov_halow_ready_for_report(void)
{
    return halow_interface_app_ready();
}

void wifi_prov_get_halow_status(wifi_prov_halow_status_t *out_status)
{
    halow_interface_app_get_status(out_status);
}

void wifi_prov_shutdown_halow_for_sleep(void)
{
    (void)halow_interface_app_shutdown(true);
}

void wifi_prov_shutdown_halow_for_restart(void)
{
    (void)halow_interface_app_shutdown(false);
}
#endif
