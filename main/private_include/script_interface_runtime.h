#ifndef DRIVER_SCRIPT_INTERFACE_RUNTIME_H
#define DRIVER_SCRIPT_INTERFACE_RUNTIME_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SCRIPT_INTERFACE_I2C = 1,
    SCRIPT_INTERFACE_RS485 = 2,
    SCRIPT_INTERFACE_UART = 3,
} script_interface_kind_t;

typedef struct {
    uint32_t address_or_baud;
    int32_t rx_size;
    uint32_t unit_id;
    uint32_t mode;
    int32_t tx_pin;
    int32_t rx_pin;
} script_interface_config_t;

esp_err_t script_interface_connect(script_interface_kind_t kind,
                                   const script_interface_config_t *config);
esp_err_t script_interface_close(script_interface_kind_t kind);
esp_err_t script_interface_reset_rx_cursor(script_interface_kind_t kind);
esp_err_t script_interface_set_rx_size(script_interface_kind_t kind, int32_t size);
esp_err_t script_interface_write(script_interface_kind_t kind,
                                 const uint8_t *payload,
                                 size_t payload_len);
esp_err_t script_interface_read(script_interface_kind_t kind,
                                uint8_t *out,
                                size_t out_size,
                                size_t *out_len);
bool script_interface_is_open(script_interface_kind_t kind);

#ifdef __cplusplus
}
#endif

#endif
