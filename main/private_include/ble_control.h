#ifndef DRIVER_BLE_CONTROL_H
#define DRIVER_BLE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ble_control_set_enabled(bool enabled);
esp_err_t ble_control_set_pairing_enabled(bool enabled);
bool ble_control_is_enabled(void);
bool ble_control_is_pairing_enabled(void);
bool ble_control_is_connected(void);
void ble_control_cap_log_level_for_ble(void);
void ble_control_send_frame(const uint8_t *payload, uint16_t payload_len);
void ble_control_send_forward_frame(const uint8_t *payload, uint16_t payload_len);
void ble_control_send_voice_frame(const uint8_t *payload, uint16_t payload_len);

#ifdef __cplusplus
}
#endif

#endif
