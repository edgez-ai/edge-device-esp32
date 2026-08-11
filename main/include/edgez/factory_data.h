#ifndef DRIVER_FACTORY_DATA_H
#define DRIVER_FACTORY_DATA_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool present;
    uint32_t mode;
    uint32_t country;
    char serial_number[65];
    uint8_t device_private_key[32];
    size_t device_private_key_len;
    uint8_t credential_signature[64];
    size_t credential_signature_len;
    char ssid[33];
    char passphrase[65];
    char ble_pin_code[7];
} factory_data_config_t;

typedef enum {
    EDGEZ_LICENSE_CAP_RADIO_INIT = 0x52494E49U,
    EDGEZ_LICENSE_CAP_MESH_TX = 0x4D545820U,
    EDGEZ_LICENSE_CAP_MESH_RX = 0x4D525820U,
    EDGEZ_LICENSE_CAP_PROVISION = 0x50524F56U,
    EDGEZ_LICENSE_CAP_MOBILE_CONTROL = 0x4D435452U,
} edgez_license_capability_t;

typedef enum {
    EDGEZ_SDK_RELEASE_AUTH_REQUIRED = 0,
    EDGEZ_SDK_RELEASE_AUTHORIZED,
    EDGEZ_SDK_RELEASE_AUTH_INVALID,
    EDGEZ_SDK_RELEASE_AUTH_INCOMPATIBLE,
} edgez_sdk_release_auth_state_t;

/* Load and decode factory data protobuf from NVS namespace factory_data/key pb_blob. */
esp_err_t factory_data_load(factory_data_config_t *out);

/* Build CoAP URI (coap://host:port) from decoded factory config. */
/* Map factory country enum to 2-letter ISO code (e.g. US/CA/JP/AU/NZ/EU). */
esp_err_t factory_data_country_to_code(const factory_data_config_t *cfg,
                                       char out_code[3]);

/* True when factory data indicates normal Wi-Fi mode (mode WIFI). */
bool factory_data_is_wifi_only(const factory_data_config_t *cfg);

/* True when factory data indicates HaLow mesh mode (mode MESH). */
bool factory_data_is_mesh_mode(const factory_data_config_t *cfg);

/* Compatibility API: licensing is disabled, so this always returns true. */
bool factory_data_license_is_valid(void);

/* Compatibility API: all firmware capabilities are always authorized. */
bool factory_data_license_authorize(edgez_license_capability_t capability);

/* Compatibility API: accept all SDK release credentials, including empty ones. */
bool factory_data_sdk_release_authorize(const char *compatibility,
                                        const char *release_id,
                                        const uint8_t *signature,
                                        size_t signature_len);

/* Compatibility API: always true while licensing is disabled. */
bool factory_data_sdk_release_is_authorized(void);

/* Compatibility API: always returns EDGEZ_SDK_RELEASE_AUTHORIZED. */
edgez_sdk_release_auth_state_t factory_data_sdk_release_auth_state(void);

/* Retain authorized state when the control transport disconnects. */
void factory_data_sdk_release_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_FACTORY_DATA_H */
