#include <stdio.h>
#include <string.h>
#include "edgez_app.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "ble_control.h"
#include "edgez_platform_adapter.h"
#include "usb_control_transport.h"
#include "halow_sync_bridge.h"
#include "prov.h"
#include "status_led.h"
#ifdef CONFIG_EDGEZ_SENSOR_SCRIPTING
#include "sampling.h"
#endif

static const char *TAG = "main";

#ifdef CONFIG_ENABLE_MM_HALOW
#ifndef CONFIG_MM_EXPERIMENTAL_MESH_CHAN
#define CONFIG_MM_EXPERIMENTAL_MESH_CHAN 0
#endif
#endif


#define RESET_BUTTON_GPIO GPIO_NUM_0
#define RESET_HOLD_MS     5000
#define RESET_POLL_MS     50

#define RESET_TASK_STACK_SIZE 4096

#define FACTORY_RESET_MAGIC 0xA5F0C1E5u

static RTC_NOINIT_ATTR uint32_t s_factory_reset_magic;
static RTC_NOINIT_ATTR uint32_t s_factory_reset_magic_inv;

static void gpio0_long_press_reset_task(void *arg)
{
    (void)arg;

    const TickType_t poll_ticks = pdMS_TO_TICKS(RESET_POLL_MS);
    TickType_t pressed_since = 0;

    while (1) {
        bool pressed = (gpio_get_level(RESET_BUTTON_GPIO) == 0);

        if (pressed) {
            if (pressed_since == 0) {
                pressed_since = xTaskGetTickCount();
            } else {
                TickType_t held = xTaskGetTickCount() - pressed_since;
                if (held >= pdMS_TO_TICKS(RESET_HOLD_MS)) {
                    ESP_LOGW(TAG, "GPIO0 held for %d ms, scheduling factory reset (NVS erase)", RESET_HOLD_MS);
                    s_factory_reset_magic = FACTORY_RESET_MAGIC;
                    s_factory_reset_magic_inv = ~FACTORY_RESET_MAGIC;
                    pressed_since = 0;
                    vTaskDelay(pdMS_TO_TICKS(100));
                    esp_restart();
                }
            }
        } else {
            pressed_since = 0;
        }

        vTaskDelay(poll_ticks);
    }
}

static void start_gpio0_long_press_reset_monitor(void)
{
    const gpio_config_t reset_gpio_cfg = {
        .pin_bit_mask = (1ULL << RESET_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&reset_gpio_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO0 reset button: %s", esp_err_to_name(err));
        return;
    }

    BaseType_t created = xTaskCreate(gpio0_long_press_reset_task,
                                     "gpio0_reset",
                                     RESET_TASK_STACK_SIZE,
                                     NULL,
                                     5,
                                     NULL);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPIO0 reset monitor task");
        return;
    }

    ESP_LOGI(TAG, "GPIO0 long-press reset enabled (hold %d ms)", RESET_HOLD_MS);
}

static void configure_runtime_log_levels(void)
{
    /* NimBLE logs every GATT notify at INFO level. Keep failures visible while
     * avoiding serial output on the voice/OTA hot path at every app log level. */
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    esp_log_level_set("spi_master", ESP_LOG_WARN);
    esp_log_level_set("spi", ESP_LOG_WARN);
    esp_log_level_set("gdma", ESP_LOG_WARN);
    esp_log_level_set("gpio", ESP_LOG_WARN);
}


static esp_err_t init_network_runtime(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

#ifdef CONFIG_ENABLE_MM_HALOW
    extern bool g_lwip_initialized_by_esp_netif;
    g_lwip_initialized_by_esp_netif = true;
#endif

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    return ESP_OK;
}

static void init_halow_control_plane(void)
{
    esp_err_t err = init_network_runtime();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Network runtime init failed: %s", esp_err_to_name(err));
        return;
    }

    err = wifi_prov_init_and_start();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "HaLow control-plane init failed: %s", esp_err_to_name(err));
        return;
    }

#if CONFIG_EDGEZ_CUSTOM_USB_CONTROL
    ESP_LOGI(TAG, "HaLow control plane ready; waiting for BLE/USB INIT_HALOW");
#else
    ESP_LOGI(TAG, "HaLow control plane ready; waiting for BLE INIT_HALOW (USB transport disabled)");
#endif
}

void edgez_app_run(void)
{
    configure_runtime_log_levels();

    esp_err_t err = ESP_OK;

    ESP_ERROR_CHECK(edgez_platform_adapter_init());

    err = usb_control_transport_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "USB control transport init failed: %s", esp_err_to_name(err));
    }

    bool factory_reset_requested =
        (s_factory_reset_magic == FACTORY_RESET_MAGIC) &&
        (s_factory_reset_magic_inv == (uint32_t)~FACTORY_RESET_MAGIC) &&
        (esp_reset_reason() == ESP_RST_SW);

    if (factory_reset_requested) {
        ESP_LOGW(TAG, "Factory reset requested, erasing NVS on boot");
        s_factory_reset_magic = 0;
        s_factory_reset_magic_inv = 0;

        ESP_ERROR_CHECK(nvs_flash_init());
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
        ESP_LOGI(TAG, "Factory reset complete");
    } else {
        s_factory_reset_magic = 0;
        s_factory_reset_magic_inv = 0;
        // Initialize NVS
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* Reserve lwIP's TCP/IP thread and mailbox before BLE, Morse/HaLow, Lua,
     * and sensor tasks consume or fragment internal RAM. The later control
     * plane initialization reuses this idempotent network runtime. */
    err = init_network_runtime();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Early network runtime init failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "ESP-NETIF/lwIP runtime initialized before radio and sampling tasks");

    start_gpio0_long_press_reset_monitor();

    err = ble_control_set_enabled(true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BLE default enable failed: %s", esp_err_to_name(err));
        status_led_set_error(STATUS_LED_ERROR_BLE, true);
    } else {
        err = ble_control_set_pairing_enabled(true);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "BLE default pairing enable failed: %s", esp_err_to_name(err));
            status_led_set_error(STATUS_LED_ERROR_BLE, true);
        } else {
            status_led_set_error(STATUS_LED_ERROR_BLE, false);
            ESP_LOGI(TAG, "BLE enabled by default with pairing advertising");
        }
    }

    const gpio_config_t power_gpio_cfg = {
        .pin_bit_mask = (1ULL << GPIO_NUM_13) | (1ULL << GPIO_NUM_14),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&power_gpio_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Restarting due to critical GPIO init failure");
        esp_restart();
    }
    err = gpio_set_level(GPIO_NUM_13, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed setting GPIO13 low: %s", esp_err_to_name(err));
    }
    err = gpio_set_level(GPIO_NUM_14, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed setting GPIO14 low: %s", esp_err_to_name(err));
    }

    halow_sync_bridge_init();

#ifdef CONFIG_EDGEZ_SENSOR_SCRIPTING
    esp_reset_reason_t reset_reason = esp_reset_reason();
    ESP_LOGI(TAG,
             "Loading local sampling settings (reset_reason=%d, rtc_ready=%d)",
             (int)reset_reason,
             sample_is_rtc_cache_ready() ? 1 : 0);
    sample_settings_load_from_nvs_to_rtc();

    if (sample_is_sampling_enabled()) {
        sample_sleep_mode_t sleep_mode = sample_get_sleep_mode();
        if (sleep_mode != SAMPLE_SLEEP_MODE_NO) {
            ESP_LOGW(TAG,
                     "Local-only sampling does not manage network sleep; using no-sleep scheduler instead of %s",
                     sample_sleep_mode_to_string(sleep_mode));
        }
        sample_set_startup_network_guard(true);
        sample_start_no_sleep_tasks();
        ESP_LOGI(TAG, "Local Lua sampling configured; waiting for HaLow mesh initialization");
    } else {
        ESP_LOGI(TAG, "Local Lua sampling disabled by saved settings");
    }
#endif

#if CONFIG_EDGEZ_CUSTOM_USB_CONTROL
    ESP_LOGI(TAG, "Starting BLE/USB-owned HaLow control plane");
#else
    ESP_LOGI(TAG, "Starting BLE-owned HaLow control plane");
#endif
    init_halow_control_plane();
    ESP_LOGI(TAG, "Edge device initialization complete");
}
