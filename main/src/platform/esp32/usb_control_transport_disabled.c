#include "usb_control_transport.h"

/* Custom-control-disabled stubs. Keeping the API available lets shared
 * mesh/platform code select BLE without scattering Kconfig checks throughout
 * the application. Crucially, this file does not install a UART driver or
 * vprintf handler, so the normal ESP-IDF UART console/logging remains intact. */
esp_err_t usb_control_transport_init(void)
{
    return ESP_OK;
}

bool usb_control_transport_is_connected(void)
{
    return false;
}

void usb_control_transport_send_frame(const uint8_t *payload,
                                      uint16_t payload_len)
{
    (void)payload;
    (void)payload_len;
}

void usb_control_transport_send_voice_frame(const uint8_t *payload,
                                            uint16_t payload_len)
{
    (void)payload;
    (void)payload_len;
}

void usb_control_transport_send_log_frame(const uint8_t *payload,
                                          uint16_t payload_len)
{
    (void)payload;
    (void)payload_len;
}
