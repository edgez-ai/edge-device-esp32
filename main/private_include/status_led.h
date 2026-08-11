#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    STATUS_LED_ERROR_BLE = 1U << 0,
    STATUS_LED_ERROR_HALOW = 1U << 1,
    STATUS_LED_ERROR_BEACON = 1U << 2,
} status_led_error_t;

esp_err_t status_led_init(void);
void status_led_set_enabled(bool enabled);
void status_led_set_user_mode(bool user_mode);
void status_led_note_ble_connected(bool connected);
void status_led_flash_beacon(void);
void status_led_set_error(status_led_error_t source, bool active);
void status_led_set_gpio_busy(int gpio_num, bool busy);
void status_led_prepare_for_sleep(void);
