#include "edgez_platform_adapter.h"

#include <string.h>

#include "ble_control.h"
#include "edgez_platform.h"
#include "halow_interface_app.h"
#include "l76k_gps.h"
#include "prov.h"
#include "sampling.h"
#ifdef CONFIG_EDGEZ_SENSOR_SCRIPTING
#include "script_interface_runtime.h"
#include "script_store.h"
#endif
#include "status_led.h"
#include "usb_control_transport.h"

static bool platform_host_is_connected(void)
{
    return ble_control_is_connected() || usb_control_transport_is_connected();
}

static void platform_send_frame(const uint8_t *payload, uint16_t payload_len)
{
    if (ble_control_is_connected()) {
        ble_control_send_frame(payload, payload_len);
    } else if (usb_control_transport_is_connected()) {
        usb_control_transport_send_frame(payload, payload_len);
    }
}

static void platform_send_forward_frame(const uint8_t *payload, uint16_t payload_len)
{
    if (ble_control_is_connected()) {
        ble_control_send_forward_frame(payload, payload_len);
    } else if (usb_control_transport_is_connected()) {
        usb_control_transport_send_frame(payload, payload_len);
    }
}

static void platform_send_voice_frame(const uint8_t *payload, uint16_t payload_len)
{
    if (ble_control_is_connected()) {
        ble_control_send_voice_frame(payload, payload_len);
    } else if (usb_control_transport_is_connected()) {
        usb_control_transport_send_voice_frame(payload, payload_len);
    }
}

static void platform_halow_get_status(edgez_platform_halow_status_t *out)
{
    wifi_prov_halow_status_t host = {0};
    halow_interface_app_get_status(&host);
    if (out) {
        _Static_assert(sizeof(*out) == sizeof(host), "HaLow status contract changed");
        memcpy(out, &host, sizeof(*out));
    }
}

static void platform_led_set_error(edgez_platform_led_error_t source, bool active)
{
    status_led_set_error((status_led_error_t)source, active);
}

static bool platform_sampling_get_latest_sensor_data(edgez_platform_sensor_data_t *out)
{
    sample_sensor_data_t host = {0};
    bool available = sample_get_latest_sensor_data(&host);
    if (out) {
        _Static_assert(sizeof(*out) == sizeof(host), "sensor data contract changed");
        memcpy(out, &host, sizeof(*out));
    }
    return available;
}

static void platform_sampling_get_sensor_selectors(char *uart_i2c, size_t uart_i2c_size,
                                                   char *rs485, size_t rs485_size)
{
    if (uart_i2c && uart_i2c_size) {
#ifdef CONFIG_EDGEZ_SENSOR_SCRIPTING
        strlcpy(uart_i2c, g_sample_sensor_uart_i2c, uart_i2c_size);
#else
        uart_i2c[0] = '\0';
#endif
    }
    if (rs485 && rs485_size) {
#ifdef CONFIG_EDGEZ_SENSOR_SCRIPTING
        strlcpy(rs485, g_sample_sensor_rs485, rs485_size);
#else
        rs485[0] = '\0';
#endif
    }
}

#ifdef CONFIG_EDGEZ_SENSOR_SCRIPTING
static esp_err_t platform_script_store_upsert(const edgez_platform_script_upsert_t *config)
{
    script_store_upsert_t host;
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    _Static_assert(sizeof(host) == sizeof(*config), "script store contract changed");
    memcpy(&host, config, sizeof(host));
    return script_store_upsert(&host);
}

static esp_err_t platform_interface_connect(edgez_platform_interface_kind_t kind,
                                            const edgez_platform_interface_config_t *config)
{
    script_interface_config_t host;
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    _Static_assert(sizeof(host) == sizeof(*config), "script interface contract changed");
    memcpy(&host, config, sizeof(host));
    return script_interface_connect((script_interface_kind_t)kind, &host);
}

static esp_err_t platform_interface_close(edgez_platform_interface_kind_t kind)
{
    return script_interface_close((script_interface_kind_t)kind);
}

static esp_err_t platform_interface_reset_rx_cursor(edgez_platform_interface_kind_t kind)
{
    return script_interface_reset_rx_cursor((script_interface_kind_t)kind);
}

static esp_err_t platform_interface_set_rx_size(edgez_platform_interface_kind_t kind, int32_t size)
{
    return script_interface_set_rx_size((script_interface_kind_t)kind, size);
}

static esp_err_t platform_interface_write(edgez_platform_interface_kind_t kind,
                                          const uint8_t *payload, size_t payload_len)
{
    return script_interface_write((script_interface_kind_t)kind, payload, payload_len);
}

static esp_err_t platform_interface_read(edgez_platform_interface_kind_t kind, uint8_t *out,
                                         size_t out_size, size_t *out_len)
{
    return script_interface_read((script_interface_kind_t)kind, out, out_size, out_len);
}
#endif

static const edgez_platform_api_t s_edgez_platform_api = {
    .ble_is_connected = platform_host_is_connected,
    .ble_is_enabled = ble_control_is_enabled,
    .ble_set_enabled = ble_control_set_enabled,
    .ble_send_frame = platform_send_frame,
    .ble_send_forward_frame = platform_send_forward_frame,
    .ble_send_voice_frame = platform_send_voice_frame,
    .network_prepare_for_deep_sleep = wifi_prov_prepare_for_deep_sleep,
    .network_shutdown_halow = wifi_prov_shutdown_halow_for_restart,
    .network_connect_halow = wifi_prov_connect_halow,
    .network_start_halow_beacon_only = wifi_prov_start_halow_beacon_only,
    .network_connect_upstream_wifi = wifi_prov_connect_upstream_wifi,
    .network_set_halow_user_identity = wifi_prov_set_halow_user_identity,
    .network_set_halow_beacon_profile = wifi_prov_set_halow_beacon_profile,
    .network_set_halow_beacon_sensor_data = wifi_prov_set_halow_beacon_sensor_data,
    .network_set_halow_beacon_device_type = wifi_prov_set_halow_beacon_device_type,
    .network_refresh_halow_mesh_vendor_ie = wifi_prov_refresh_halow_mesh_vendor_ie,
    .network_set_halow_max_hop = wifi_prov_set_halow_max_hop,
    .network_set_halow_mesh_radio = wifi_prov_set_halow_mesh_radio,
    .network_set_halow_country_code = wifi_prov_set_halow_country_code,
    .halow_init = halow_interface_app_init,
    .halow_ready = halow_interface_app_ready,
    .halow_get_self_mac = halow_interface_app_get_self_mac,
    .halow_get_status = platform_halow_get_status,
    .halow_set_user_identity = halow_interface_app_set_user_identity,
    .halow_request_beacon = halow_interface_app_request_beacon,
    .halow_send_mesh_payload_via = halow_interface_app_send_mesh_payload_via,
    .halow_forward_mesh_payload = halow_interface_app_forward_mesh_payload,
    .halow_send_peer_independent_beacon = halow_interface_app_send_peer_independent_beacon,
    .led_init = status_led_init,
    .led_set_enabled = status_led_set_enabled,
    .led_set_user_mode = status_led_set_user_mode,
    .led_note_ble_connected = status_led_note_ble_connected,
    .led_flash_beacon = status_led_flash_beacon,
    .led_set_error = platform_led_set_error,
    .led_prepare_for_sleep = status_led_prepare_for_sleep,
    .gps_supported = l76k_gps_supported,
    .gps_configure = l76k_gps_configure,
    .gps_get_latest = l76k_gps_get_latest,
    .sampling_get_latest_sensor_data = platform_sampling_get_latest_sensor_data,
    .sampling_get_sensor_selectors = platform_sampling_get_sensor_selectors,
    .sampling_buffer_acquire = sample_script_global_buffer_acquire_serving,
    .sampling_buffer_serving_length = sample_script_global_buffer_get_serving_length,
    .sampling_buffer_release = sample_script_global_buffer_release_serving,
#ifdef CONFIG_EDGEZ_SENSOR_SCRIPTING
    .sampling_set_sensor_selectors = sample_set_sensor_selectors,
    .sampling_refresh_script_cache = sample_refresh_script_cache_from_selectors,
    .script_store_upsert = platform_script_store_upsert,
    .script_store_delete = script_store_delete,
    .interface_connect = platform_interface_connect,
    .interface_close = platform_interface_close,
    .interface_reset_rx_cursor = platform_interface_reset_rx_cursor,
    .interface_set_rx_size = platform_interface_set_rx_size,
    .interface_write = platform_interface_write,
    .interface_read = platform_interface_read,
#endif
};

esp_err_t edgez_platform_adapter_init(void)
{
    return edgez_platform_register(&s_edgez_platform_api);
}
