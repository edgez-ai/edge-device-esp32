#ifndef USB_CONTROL_TRANSPORT_H
#define USB_CONTROL_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t usb_control_transport_init(void);
bool usb_control_transport_is_connected(void);
void usb_control_transport_send_frame(const uint8_t *payload, uint16_t payload_len);
void usb_control_transport_send_voice_frame(const uint8_t *payload, uint16_t payload_len);
void usb_control_transport_send_log_frame(const uint8_t *payload, uint16_t payload_len);

#ifdef __cplusplus
}
#endif

#endif
