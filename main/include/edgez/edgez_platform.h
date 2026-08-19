#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "usb_control.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EDGEZ_PLATFORM_INTERFACE_I2C = 1,
    EDGEZ_PLATFORM_INTERFACE_RS485 = 2,
    EDGEZ_PLATFORM_INTERFACE_UART = 3,
} edgez_platform_interface_kind_t;

typedef enum {
    EDGEZ_PLATFORM_LED_ERROR_BLE = 1U << 0,
    EDGEZ_PLATFORM_LED_ERROR_HALOW = 1U << 1,
    EDGEZ_PLATFORM_LED_ERROR_BEACON = 1U << 2,
} edgez_platform_led_error_t;

typedef struct {
    uint32_t address_or_baud;
    int32_t rx_size;
    uint32_t unit_id;
    uint32_t mode;
    int32_t tx_pin;
    int32_t rx_pin;
} edgez_platform_interface_config_t;

typedef struct {
    bool supported;
    bool stack_initialized;
    bool mesh_mode;
    bool link_up;
    bool route_ready;
    bool ready_for_report;
    char mesh_id[33];
    char ip_addr[16];
    char gateway[16];
} edgez_platform_halow_status_t;

typedef struct {
    uint8_t originator[6];
    uint8_t next_hop[6];
    uint8_t tq;
    uint8_t hops;
    uint32_t age_ms;
} edgez_platform_halow_route_t;

typedef struct {
    bool has_latitude;
    bool has_longitude;
    bool has_altitude;
    bool has_temperature;
    bool has_humidity;
    bool has_pressure;
    bool has_vibration_average;
    bool has_length;
    bool has_accel_x;
    bool has_accel_y;
    bool has_accel_z;
    bool has_gyro_x;
    bool has_gyro_y;
    bool has_gyro_z;
    float latitude;
    float longitude;
    float altitude;
    float temperature;
    float humidity;
    float pressure;
    float vibration_average;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    int32_t length;
} edgez_platform_sensor_data_t;

typedef struct {
    uint16_t script_id;
    const uint8_t *name;
    size_t name_len;
    const uint8_t *script;
    size_t script_len;
    bool has_version;
    uint32_t version;
    bool has_global_buffer_size;
    uint32_t global_buffer_size;
    const uint8_t *mime_type;
    size_t mime_type_len;
} edgez_platform_script_upsert_t;

/** Host services consumed by EdgeZ business logic.
 *
 * The application owns this table and must keep it alive after registration.
 * EdgeZ depends only on this SDK contract, never on application headers.
 */
typedef struct {
    bool (*ble_is_connected)(void);
    bool (*ble_is_enabled)(void);
    esp_err_t (*ble_set_enabled)(bool enabled);
    void (*ble_send_frame)(const uint8_t *payload, uint16_t payload_len);
    void (*ble_send_forward_frame)(const uint8_t *payload, uint16_t payload_len);
    void (*ble_send_voice_frame)(const uint8_t *payload, uint16_t payload_len);

    esp_err_t (*network_prepare_for_deep_sleep)(void);
    void (*network_shutdown_halow)(void);
    esp_err_t (*network_connect_halow)(const char *ssid, const char *password);
    esp_err_t (*network_start_halow_beacon_only)(const char *ssid, const char *password);
    esp_err_t (*network_connect_upstream_wifi)(const char *ssid, const char *password);
    esp_err_t (*network_set_halow_user_identity)(uint64_t user_id_high, uint64_t user_id_low,
                                                 const char *user_name,
                                                 const uint8_t *user_public_key,
                                                 size_t user_public_key_len);
    esp_err_t (*network_set_halow_beacon_profile)(uint32_t marker, bool has_location,
                                                  float latitude, float longitude);
    esp_err_t (*network_set_halow_beacon_sensor_data)(const ai_edgez_halow_SensorData *sensor_data,
                                                      size_t sensor_data_count);
    esp_err_t (*network_set_halow_beacon_device_type)(uint32_t device_type);
    esp_err_t (*network_refresh_halow_mesh_vendor_ie)(const char *mesh_id,
                                                      const char *passphrase);
    esp_err_t (*network_set_halow_max_hop)(uint32_t max_hop);
    esp_err_t (*network_set_halow_mesh_radio)(uint32_t frequency_khz, uint32_t bandwidth_mhz);
    esp_err_t (*network_set_halow_country_code)(const char *country_code);

    esp_err_t (*halow_init)(void);
    bool (*halow_ready)(void);
    bool (*halow_get_self_mac)(uint8_t mac[6]);
    bool (*halow_lookup_route)(const uint8_t destination[6], uint8_t next_hop[6],
                               uint8_t *hop_count, uint8_t *tq,
                               uint32_t *route_age_ms);
    size_t (*halow_get_routes)(edgez_platform_halow_route_t *routes,
                               size_t capacity);
    bool (*halow_select_direct_peer)(const uint8_t exclude_a[6],
                                     const uint8_t exclude_b[6],
                                     const uint8_t exclude_c[6],
                                     uint64_t selection_key, uint8_t peer[6]);
    void (*halow_note_foreground_traffic)(void);
    bool (*halow_foreground_traffic_active)(void);
    void (*halow_get_status)(edgez_platform_halow_status_t *out_status);
    esp_err_t (*halow_set_user_identity)(uint64_t user_id_high, uint64_t user_id_low,
                                         const char *user_name,
                                         const uint8_t *user_public_key,
                                         size_t user_public_key_len);
    esp_err_t (*halow_request_beacon)(const char *beacon, uint32_t max_hop,
                                      uint64_t message_id_high, uint64_t message_id_low,
                                      uint32_t sequence, uint64_t next_hop);
    esp_err_t (*halow_send_mesh_payload_via)(const uint8_t *payload, size_t payload_len,
                                             uint64_t from, uint64_t to, uint64_t next_hop,
                                             uint64_t message_id_high, uint64_t message_id_low,
                                             uint32_t max_hop, uint32_t sequence);
    esp_err_t (*halow_forward_mesh_payload)(const uint8_t *payload, size_t payload_len,
                                            uint64_t from, uint64_t to, uint64_t next_hop,
                                            uint64_t message_id_high, uint64_t message_id_low,
                                            uint32_t max_hop, uint32_t sequence, uint32_t hop);
    esp_err_t (*halow_send_peer_independent_beacon)(const uint8_t *payload, size_t payload_len,
                                                    uint64_t from, uint64_t to,
                                                    uint64_t message_id_high,
                                                    uint64_t message_id_low,
                                                    uint32_t max_hop, uint32_t sequence);

    esp_err_t (*led_init)(void);
    void (*led_set_enabled)(bool enabled);
    void (*led_set_user_mode)(bool user_mode);
    void (*led_note_ble_connected)(bool connected);
    void (*led_flash_beacon)(void);
    void (*led_set_error)(edgez_platform_led_error_t source, bool active);
    void (*led_prepare_for_sleep)(void);

    bool (*gps_supported)(void);
    esp_err_t (*gps_configure)(bool enabled, uint32_t update_interval_seconds);
    bool (*gps_get_latest)(float *latitude, float *longitude,
                           uint64_t *timestamp_ms);

    bool (*sampling_get_latest_sensor_data)(edgez_platform_sensor_data_t *out);
    void (*sampling_get_sensor_selectors)(char *uart_i2c, size_t uart_i2c_size,
                                          char *rs485, size_t rs485_size);
    esp_err_t (*sampling_set_sensor_selectors)(const char *uart_i2c, const char *rs485);
    void (*sampling_refresh_script_cache)(void);
    esp_err_t (*sampling_buffer_acquire)(const uint8_t **data, size_t *length);
    size_t (*sampling_buffer_serving_length)(void);
    void (*sampling_buffer_release)(void);

    esp_err_t (*script_store_upsert)(const edgez_platform_script_upsert_t *config);
    esp_err_t (*script_store_delete)(uint16_t script_id);
    esp_err_t (*interface_connect)(edgez_platform_interface_kind_t kind,
                                   const edgez_platform_interface_config_t *config);
    esp_err_t (*interface_close)(edgez_platform_interface_kind_t kind);
    esp_err_t (*interface_reset_rx_cursor)(edgez_platform_interface_kind_t kind);
    esp_err_t (*interface_set_rx_size)(edgez_platform_interface_kind_t kind, int32_t size);
    esp_err_t (*interface_write)(edgez_platform_interface_kind_t kind,
                                 const uint8_t *payload, size_t payload_len);
    esp_err_t (*interface_read)(edgez_platform_interface_kind_t kind, uint8_t *out,
                                size_t out_size, size_t *out_len);
} edgez_platform_api_t;

esp_err_t edgez_platform_register(const edgez_platform_api_t *api);
const edgez_platform_api_t *edgez_platform_get(void);

#ifdef __cplusplus
}
#endif
