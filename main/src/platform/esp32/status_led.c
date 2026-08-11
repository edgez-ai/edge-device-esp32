#include "status_led.h"

#include <string.h>

#include "ble_control.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "status_led";

#ifdef CONFIG_HT_HC33_STATUS_LED

enum {
    STATUS_LED_RMT_RESOLUTION_HZ = 10000000,
    STATUS_LED_TASK_STACK_SIZE = 3072,
    STATUS_LED_BLE_CONNECTED_OFF_MS = 5000,
};

static const rmt_symbol_word_t s_led_zero = {
    .level0 = 1,
    .duration0 = 3,
    .level1 = 0,
    .duration1 = 9,
};
static const rmt_symbol_word_t s_led_one = {
    .level0 = 1,
    .duration0 = 9,
    .level1 = 0,
    .duration1 = 3,
};
static const rmt_symbol_word_t s_led_reset = {
    .level0 = 0,
    .duration0 = 250,
    .level1 = 0,
    .duration1 = 250,
};

static rmt_channel_handle_t s_led_channel;
static rmt_encoder_handle_t s_led_encoder;
static TaskHandle_t s_led_task;
static SemaphoreHandle_t s_led_io_mutex;
static portMUX_TYPE s_led_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_logic_enabled = true;
static bool s_rmt_enabled;
static bool s_user_mode = true;
static bool s_ble_connected;
static bool s_force_off;
static bool s_gpio_suspended;
static uint32_t s_gpio_busy_count;
static uint32_t s_output_generation;
static uint32_t s_error_sources;
static int64_t s_beacon_until_ms;

static void status_led_wake_task(void)
{
    if (s_led_task != NULL) {
        xTaskNotifyGive(s_led_task);
    }
}

static size_t status_led_encode(const void *data,
                                size_t data_size,
                                size_t symbols_written,
                                size_t symbols_free,
                                rmt_symbol_word_t *symbols,
                                bool *done,
                                void *arg)
{
    (void)arg;
    if (symbols_free < 8) {
        return 0;
    }

    size_t byte_index = symbols_written / 8;
    if (byte_index < data_size) {
        const uint8_t *bytes = data;
        size_t output = 0;
        for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
            symbols[output++] = (bytes[byte_index] & mask) ? s_led_one : s_led_zero;
        }
        return output;
    }

    symbols[0] = s_led_reset;
    *done = true;
    return 1;
}

static esp_err_t status_led_write(uint8_t red, uint8_t green, uint8_t blue)
{
    if (s_led_io_mutex == NULL ||
        xSemaphoreTake(s_led_io_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    bool suspended;
    portENTER_CRITICAL(&s_led_lock);
    suspended = s_gpio_suspended;
    portEXIT_CRITICAL(&s_led_lock);
    if (suspended || !s_rmt_enabled) {
        xSemaphoreGive(s_led_io_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    /* SK6812 uses GRB byte order. */
    const uint8_t pixel[] = {green, red, blue};
    const rmt_transmit_config_t tx_config = {.loop_count = 0};
    esp_err_t err = rmt_transmit(s_led_channel,
                                 s_led_encoder,
                                 pixel,
                                 sizeof(pixel),
                                 &tx_config);
    if (err == ESP_OK) {
        err = rmt_tx_wait_all_done(s_led_channel, 20);
    }
    xSemaphoreGive(s_led_io_mutex);
    return err;
}

static void status_led_task(void *arg)
{
    (void)arg;
    uint8_t last_red = UINT8_MAX;
    uint8_t last_green = UINT8_MAX;
    uint8_t last_blue = UINT8_MAX;
    uint32_t seen_generation = UINT32_MAX;
    uint32_t phase = 0;
    bool connected_pattern_active = false;
    int64_t connected_cycle_start_ms = 0;

    while (true) {
        bool user_mode;
        bool ble_connected;
        bool logic_enabled;
        bool force_off;
        bool gpio_suspended;
        uint32_t output_generation;
        uint32_t errors;
        int64_t beacon_until_ms;
        portENTER_CRITICAL(&s_led_lock);
        logic_enabled = s_logic_enabled;
        user_mode = s_user_mode;
        ble_connected = s_ble_connected;
        force_off = s_force_off;
        gpio_suspended = s_gpio_suspended;
        output_generation = s_output_generation;
        errors = s_error_sources;
        beacon_until_ms = s_beacon_until_ms;
        portEXIT_CRITICAL(&s_led_lock);

        if (!logic_enabled || gpio_suspended) {
            (void)ulTaskNotifyTake(pdTRUE,
                                   pdMS_TO_TICKS(CONFIG_HT_HC33_LED_STEP_MS));
            continue;
        }

        uint8_t red = 0;
        uint8_t green = 0;
        uint8_t blue = 0;
        const uint8_t brightness = CONFIG_HT_HC33_LED_BRIGHTNESS;
        int64_t now_ms = esp_timer_get_time() / 1000;
        uint32_t connected_wait_ms = UINT32_MAX;
        bool connected_heartbeat_on = false;
        bool connected_pattern_requested = !force_off && user_mode &&
                                           ble_control_is_enabled() &&
                                           ble_connected;
        if (connected_pattern_requested) {
            if (!connected_pattern_active) {
                connected_pattern_active = true;
                connected_cycle_start_ms = now_ms;
            }

            const int64_t on_ms = CONFIG_HT_HC33_LED_STEP_MS;
            const int64_t cycle_ms = on_ms + STATUS_LED_BLE_CONNECTED_OFF_MS;
            int64_t elapsed_ms = (now_ms - connected_cycle_start_ms) % cycle_ms;
            connected_heartbeat_on = elapsed_ms < on_ms;
            connected_wait_ms = (uint32_t)(connected_heartbeat_on
                                               ? on_ms - elapsed_ms
                                               : cycle_ms - elapsed_ms);
        } else {
            connected_pattern_active = false;
        }

        if (!force_off && errors != 0) {
            red = brightness;
        } else if (!force_off && now_ms < beacon_until_ms) {
            green = brightness;
        } else if (!force_off && user_mode && ble_control_is_enabled()) {
            if (!ble_connected) {
                /* While disconnected, slowly flash blue and green with an
                 * off interval between colors. */
                if ((phase & 3U) == 0) {
                    blue = brightness;
                } else if ((phase & 3U) == 2U) {
                    green = brightness;
                }
            } else if (connected_heartbeat_on) {
                /* Connected user-mode BLE heartbeat: one pulse, then a
                 * five-second off interval. */
                blue = brightness;
            }
        }

        if (output_generation != seen_generation ||
            red != last_red || green != last_green || blue != last_blue) {
            esp_err_t err = status_led_write(red, green, blue);
            if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "HT-HC33 RGB LED update failed: %s", esp_err_to_name(err));
            } else {
                if (err == ESP_OK) {
                    last_red = red;
                    last_green = green;
                    last_blue = blue;
                    seen_generation = output_generation;
                }
            }
        }
        phase++;
        uint32_t wait_ms = CONFIG_HT_HC33_LED_STEP_MS;
        if (now_ms < beacon_until_ms) {
            int64_t remaining_ms = beacon_until_ms - now_ms;
            if (remaining_ms > 0 && remaining_ms < wait_ms) {
                wait_ms = (uint32_t)remaining_ms;
            }
        }
        if (connected_wait_ms < wait_ms) {
            wait_ms = connected_wait_ms;
        }
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_ms));
    }
}

esp_err_t status_led_init(void)
{
    if (s_led_task != NULL) {
        return ESP_OK;
    }

    s_led_io_mutex = xSemaphoreCreateMutex();
    if (s_led_io_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const rmt_tx_channel_config_t channel_config = {
        .gpio_num = CONFIG_HT_HC33_STATUS_LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = STATUS_LED_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 2,
    };
    esp_err_t err = rmt_new_tx_channel(&channel_config, &s_led_channel);
    if (err != ESP_OK) {
        vSemaphoreDelete(s_led_io_mutex);
        s_led_io_mutex = NULL;
        return err;
    }

    const rmt_simple_encoder_config_t encoder_config = {
        .callback = status_led_encode,
        .min_chunk_size = 8,
    };
    err = rmt_new_simple_encoder(&encoder_config, &s_led_encoder);
    if (err != ESP_OK) {
        (void)rmt_del_channel(s_led_channel);
        s_led_channel = NULL;
        vSemaphoreDelete(s_led_io_mutex);
        s_led_io_mutex = NULL;
        return err;
    }
    err = rmt_enable(s_led_channel);
    if (err != ESP_OK) {
        (void)rmt_del_encoder(s_led_encoder);
        (void)rmt_del_channel(s_led_channel);
        s_led_encoder = NULL;
        s_led_channel = NULL;
        vSemaphoreDelete(s_led_io_mutex);
        s_led_io_mutex = NULL;
        return err;
    }
    s_rmt_enabled = true;

    BaseType_t created = xTaskCreate(status_led_task,
                                     "status_led",
                                     STATUS_LED_TASK_STACK_SIZE,
                                     NULL,
                                     3,
                                     &s_led_task);
    if (created != pdPASS) {
        (void)rmt_disable(s_led_channel);
        s_rmt_enabled = false;
        (void)rmt_del_encoder(s_led_encoder);
        (void)rmt_del_channel(s_led_channel);
        s_led_encoder = NULL;
        s_led_channel = NULL;
        vSemaphoreDelete(s_led_io_mutex);
        s_led_io_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "HT-HC33 SK6812 status LED enabled gpio=%d brightness=%d step_ms=%d",
             CONFIG_HT_HC33_STATUS_LED_GPIO,
             CONFIG_HT_HC33_LED_BRIGHTNESS,
             CONFIG_HT_HC33_LED_STEP_MS);
    return ESP_OK;
}

void status_led_set_enabled(bool enabled)
{
    portENTER_CRITICAL(&s_led_lock);
    bool changed = s_logic_enabled != enabled;
    s_logic_enabled = enabled;
    bool suspended = s_gpio_suspended;
    portEXIT_CRITICAL(&s_led_lock);
    if (!changed || s_led_channel == NULL || s_led_io_mutex == NULL) {
        return;
    }

    if (!enabled) {
        status_led_wake_task();
        if (!suspended) {
            (void)status_led_write(0, 0, 0);
        }
        if (xSemaphoreTake(s_led_io_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (s_rmt_enabled) {
                if (rmt_disable(s_led_channel) == ESP_OK) {
                    s_rmt_enabled = false;
                }
            }
            xSemaphoreGive(s_led_io_mutex);
        }
        ESP_LOGI(TAG, "HT-HC33 status LED logic disabled for device mode");
        return;
    }

    esp_err_t err = ESP_OK;
    if (!suspended && xSemaphoreTake(s_led_io_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (!s_rmt_enabled) {
            err = rmt_enable(s_led_channel);
            if (err == ESP_OK) {
                s_rmt_enabled = true;
            }
        }
        xSemaphoreGive(s_led_io_mutex);
    } else if (!suspended) {
        err = ESP_ERR_TIMEOUT;
    }
    portENTER_CRITICAL(&s_led_lock);
    if (err == ESP_OK) {
        s_output_generation++;
    } else {
        s_logic_enabled = false;
    }
    portEXIT_CRITICAL(&s_led_lock);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HT-HC33 status LED logic enabled for device mode");
        status_led_wake_task();
    } else {
        ESP_LOGW(TAG, "HT-HC33 status LED enable failed: %s", esp_err_to_name(err));
    }
}

void status_led_set_user_mode(bool user_mode)
{
    portENTER_CRITICAL(&s_led_lock);
    s_user_mode = user_mode;
    portEXIT_CRITICAL(&s_led_lock);
    status_led_wake_task();
}

void status_led_note_ble_connected(bool connected)
{
    portENTER_CRITICAL(&s_led_lock);
    s_ble_connected = connected;
    portEXIT_CRITICAL(&s_led_lock);
    status_led_wake_task();
}

void status_led_flash_beacon(void)
{
    int64_t beacon_until_ms = (esp_timer_get_time() / 1000) +
                              CONFIG_HT_HC33_BEACON_FLASH_MS;
    portENTER_CRITICAL(&s_led_lock);
    s_beacon_until_ms = beacon_until_ms;
    portEXIT_CRITICAL(&s_led_lock);
    status_led_wake_task();
}

void status_led_set_error(status_led_error_t source, bool active)
{
    portENTER_CRITICAL(&s_led_lock);
    if (active) {
        s_error_sources |= (uint32_t)source;
    } else {
        s_error_sources &= ~(uint32_t)source;
    }
    portEXIT_CRITICAL(&s_led_lock);
    status_led_wake_task();
}

void status_led_set_gpio_busy(int gpio_num, bool busy)
{
    if (gpio_num != CONFIG_HT_HC33_STATUS_LED_GPIO || s_led_channel == NULL) {
        return;
    }

    bool transition = false;
    if (busy) {
        portENTER_CRITICAL(&s_led_lock);
        s_gpio_busy_count++;
        if (s_gpio_busy_count == 1) {
            s_gpio_suspended = true;
            transition = true;
        }
        portEXIT_CRITICAL(&s_led_lock);
        status_led_wake_task();

        if (transition &&
            xSemaphoreTake(s_led_io_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (s_rmt_enabled && rmt_disable(s_led_channel) == ESP_OK) {
                s_rmt_enabled = false;
            }
            xSemaphoreGive(s_led_io_mutex);
            ESP_LOGI(TAG, "Status LED suspended; GPIO%d owned by UART", gpio_num);
        }
        return;
    }

    portENTER_CRITICAL(&s_led_lock);
    if (s_gpio_busy_count > 0) {
        s_gpio_busy_count--;
        transition = s_gpio_busy_count == 0;
    }
    portEXIT_CRITICAL(&s_led_lock);
    if (!transition) {
        return;
    }

    bool logic_enabled;
    portENTER_CRITICAL(&s_led_lock);
    logic_enabled = s_logic_enabled;
    portEXIT_CRITICAL(&s_led_lock);

    esp_err_t err = ESP_OK;
    if (xSemaphoreTake(s_led_io_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_rmt_enabled) {
            if (rmt_disable(s_led_channel) == ESP_OK) {
                s_rmt_enabled = false;
            }
        }
        if (logic_enabled && !s_rmt_enabled) {
            err = rmt_enable(s_led_channel);
            if (err == ESP_OK) {
                s_rmt_enabled = true;
            }
        }
        xSemaphoreGive(s_led_io_mutex);
    } else {
        err = ESP_ERR_TIMEOUT;
    }
    portENTER_CRITICAL(&s_led_lock);
    if (err == ESP_OK) {
        s_gpio_suspended = false;
        if (logic_enabled) {
            s_output_generation++;
        }
    }
    portEXIT_CRITICAL(&s_led_lock);
    if (err == ESP_OK) {
        if (logic_enabled) {
            ESP_LOGI(TAG, "Status LED resumed; GPIO%d released by UART", gpio_num);
            status_led_wake_task();
        } else {
            ESP_LOGI(TAG, "Status LED remains disabled; GPIO%d released by UART", gpio_num);
        }
    } else {
        ESP_LOGW(TAG, "Status LED resume failed on GPIO%d: %s",
                 gpio_num,
                 esp_err_to_name(err));
    }
}

void status_led_prepare_for_sleep(void)
{
    portENTER_CRITICAL(&s_led_lock);
    s_force_off = true;
    portEXIT_CRITICAL(&s_led_lock);
    status_led_wake_task();
    if (s_led_channel != NULL && s_led_encoder != NULL) {
        (void)status_led_write(0, 0, 0);
    }
}

#else

esp_err_t status_led_init(void) { return ESP_OK; }
void status_led_set_enabled(bool enabled) { (void)enabled; }
void status_led_set_user_mode(bool user_mode) { (void)user_mode; }
void status_led_note_ble_connected(bool connected) { (void)connected; }
void status_led_flash_beacon(void) {}
void status_led_set_error(status_led_error_t source, bool active)
{
    (void)source;
    (void)active;
}
void status_led_set_gpio_busy(int gpio_num, bool busy)
{
    (void)gpio_num;
    (void)busy;
}
void status_led_prepare_for_sleep(void) {}

#endif
