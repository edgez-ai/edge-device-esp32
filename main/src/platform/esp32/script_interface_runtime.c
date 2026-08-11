#include "script_interface_runtime.h"

#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "interface_bridge.h"
#include "local_interface.h"

#ifndef CONFIG_I2C_DEFAULT_SDA_PIN
#define CONFIG_I2C_DEFAULT_SDA_PIN 19
#endif
#ifndef CONFIG_I2C_DEFAULT_SCL_PIN
#define CONFIG_I2C_DEFAULT_SCL_PIN 20
#endif
#ifndef CONFIG_RS485_DEFAULT_BAUD_RATE
#define CONFIG_RS485_DEFAULT_BAUD_RATE 9600
#endif
#ifndef CONFIG_RS485_UART_TXD
#define CONFIG_RS485_UART_TXD 17
#endif
#ifndef CONFIG_RS485_UART_RXD
#define CONFIG_RS485_UART_RXD 18
#endif
#ifndef CONFIG_RS485_RX_BUFFER_SIZE
#define CONFIG_RS485_RX_BUFFER_SIZE 256
#endif
#ifndef CONFIG_SAMPLE_UART_DEFAULT_BAUD_RATE
#define CONFIG_SAMPLE_UART_DEFAULT_BAUD_RATE 115200
#endif
#ifndef CONFIG_SAMPLE_UART_DEFAULT_TX_PIN
#define CONFIG_SAMPLE_UART_DEFAULT_TX_PIN 19
#endif
#ifndef CONFIG_SAMPLE_UART_DEFAULT_RX_PIN
#define CONFIG_SAMPLE_UART_DEFAULT_RX_PIN 20
#endif
#ifndef CONFIG_SAMPLE_UART_DEFAULT_RX_BUFFER_SIZE
#define CONFIG_SAMPLE_UART_DEFAULT_RX_BUFFER_SIZE 4096
#endif

#define SCRIPT_I2C_PORT I2C_NUM_0
#define SCRIPT_I2C_FREQ_HZ 100000
#define SCRIPT_I2C_TIMEOUT_MS 1000
#define SCRIPT_I2C_POWER_GPIO GPIO_NUM_14
#define SCRIPT_RS485_POWER_GPIO GPIO_NUM_13
#define SCRIPT_SENSOR_POWER_SETTLE_MS 1000

static const char *TAG = "script_interface";
static bool s_open[4];
static bool s_sensor_power_enabled;
static bool s_i2c_driver_installed;
static uint32_t s_i2c_address = 0x44;
static int32_t s_i2c_rx_size = 6;

static bool valid_kind(script_interface_kind_t kind)
{
    return kind >= SCRIPT_INTERFACE_I2C && kind <= SCRIPT_INTERFACE_UART;
}

bool script_interface_is_open(script_interface_kind_t kind)
{
    return valid_kind(kind) && s_open[kind];
}

static esp_err_t sensor_power_on(const char *reason)
{
    const gpio_config_t config = {
        .pin_bit_mask = (1ULL << SCRIPT_I2C_POWER_GPIO) | (1ULL << SCRIPT_RS485_POWER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&config);
    if (err == ESP_OK) err = gpio_set_level(SCRIPT_RS485_POWER_GPIO, 1);
    if (err == ESP_OK) err = gpio_set_level(SCRIPT_I2C_POWER_GPIO, 1);
    if (err != ESP_OK) return err;

    if (!s_sensor_power_enabled) {
        ESP_LOGI(TAG, "Sensor power enabled for %s; settling %d ms",
                 reason ? reason : "script interface", SCRIPT_SENSOR_POWER_SETTLE_MS);
        vTaskDelay(pdMS_TO_TICKS(SCRIPT_SENSOR_POWER_SETTLE_MS));
    }
    s_sensor_power_enabled = true;
    return ESP_OK;
}

static esp_err_t connect_i2c(const script_interface_config_t *config)
{
    esp_err_t err = sensor_power_on("i2c");
    if (err != ESP_OK) return err;

    uint32_t address = config->address_or_baud ? config->address_or_baud : 0x44;
    if (!s_i2c_driver_installed) {
        i2c_config_t i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = config->tx_pin >= 0 ? config->tx_pin : CONFIG_I2C_DEFAULT_SDA_PIN,
            .scl_io_num = config->rx_pin >= 0 ? config->rx_pin : CONFIG_I2C_DEFAULT_SCL_PIN,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = SCRIPT_I2C_FREQ_HZ,
        };
        err = i2c_param_config(SCRIPT_I2C_PORT, &i2c_config);
        if (err == ESP_OK) err = i2c_driver_install(SCRIPT_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
        if (err != ESP_OK) return err;
        s_i2c_driver_installed = true;
    }

    s_i2c_address = address & 0x7f;
    if (config->rx_size > 0) s_i2c_rx_size = config->rx_size > 256 ? 256 : config->rx_size;
    s_open[SCRIPT_INTERFACE_I2C] = true;
    ESP_LOGI(TAG, "I2C connected addr=0x%02lx rx_size=%ld",
             (unsigned long)s_i2c_address, (long)s_i2c_rx_size);
    return ESP_OK;
}

static esp_err_t connect_rs485(const script_interface_config_t *config)
{
    esp_err_t err = sensor_power_on("rs485");
    if (err != ESP_OK) return err;
    interface_bridge_register();

    uint32_t current_baud = CONFIG_RS485_DEFAULT_BAUD_RATE;
    uint32_t current_unit_id = 1;
    int32_t current_tx = CONFIG_RS485_UART_TXD;
    int32_t current_rx = CONFIG_RS485_UART_RXD;
    int32_t current_rx_size = CONFIG_RS485_RX_BUFFER_SIZE;
    (void)rs485_object_get_runtime(0, &current_baud, &current_unit_id,
                                   &current_tx, &current_rx, &current_rx_size);
    rs485_instance_cfg_t instance = {
        .enabled = true,
        .open_state = false,
        .baudrate = config->address_or_baud ? config->address_or_baud : current_baud,
        .modbus_unit_id = config->unit_id ? config->unit_id : current_unit_id,
        .mode = config->mode,
        .rx_buffer_size = config->rx_size > 0 ? config->rx_size : current_rx_size,
        .tx_pin = config->tx_pin >= 0 ? config->tx_pin : current_tx,
        .rx_pin = config->rx_pin >= 0 ? config->rx_pin : current_rx,
    };
    err = rs485_object_set_instance(0, &instance);
    if (err == ESP_OK) err = rs485_object_invoke_open(0, true);
    if (err == ESP_OK) s_open[SCRIPT_INTERFACE_RS485] = true;
    return err;
}

static esp_err_t connect_uart(const script_interface_config_t *config)
{
    interface_bridge_register();
    uint32_t current_baud = CONFIG_SAMPLE_UART_DEFAULT_BAUD_RATE;
    int32_t current_tx = CONFIG_SAMPLE_UART_DEFAULT_TX_PIN;
    int32_t current_rx = CONFIG_SAMPLE_UART_DEFAULT_RX_PIN;
    int32_t current_rx_size = CONFIG_SAMPLE_UART_DEFAULT_RX_BUFFER_SIZE;
    (void)uart_object_get_runtime(0, &current_baud, &current_tx, &current_rx, &current_rx_size);
    uart_instance_cfg_t instance = {
        .enabled = true,
        .open_state = false,
        .baudrate = config->address_or_baud ? config->address_or_baud : current_baud,
        .mode = config->mode,
        .rx_buffer_size = config->rx_size > 0 ? config->rx_size : current_rx_size,
        .tx_pin = config->tx_pin >= 0 ? config->tx_pin : current_tx,
        .rx_pin = config->rx_pin >= 0 ? config->rx_pin : current_rx,
    };
    esp_err_t err = uart_object_set_instance(0, &instance);
    if (err == ESP_OK) err = uart_object_invoke_open(0, true);
    if (err == ESP_OK) s_open[SCRIPT_INTERFACE_UART] = true;
    return err;
}

esp_err_t script_interface_connect(script_interface_kind_t kind,
                                   const script_interface_config_t *config)
{
    if (!valid_kind(kind) || !config) return ESP_ERR_INVALID_ARG;
    if (kind == SCRIPT_INTERFACE_I2C) return connect_i2c(config);
    if (kind == SCRIPT_INTERFACE_RS485) return connect_rs485(config);
    return connect_uart(config);
}

esp_err_t script_interface_close(script_interface_kind_t kind)
{
    if (!valid_kind(kind)) return ESP_ERR_INVALID_ARG;
    if (!s_open[kind]) return ESP_OK;
    esp_err_t err = ESP_OK;
    if (kind == SCRIPT_INTERFACE_RS485) err = rs485_object_invoke_open(0, false);
    if (kind == SCRIPT_INTERFACE_UART) err = uart_object_invoke_open(0, false);
    if (err == ESP_OK) s_open[kind] = false;
    return err;
}

esp_err_t script_interface_reset_rx_cursor(script_interface_kind_t kind)
{
    if (!script_interface_is_open(kind)) return ESP_ERR_INVALID_STATE;
    if (kind == SCRIPT_INTERFACE_I2C) return ESP_OK;
    if (kind == SCRIPT_INTERFACE_RS485) return rs485_object_set_rx_cursor(0, 0);
    return uart_object_set_rx_cursor(0, 0);
}

esp_err_t script_interface_set_rx_size(script_interface_kind_t kind, int32_t size)
{
    if (!script_interface_is_open(kind)) return ESP_ERR_INVALID_STATE;
    if (size < 1) size = 1;
    if (kind == SCRIPT_INTERFACE_I2C) {
        s_i2c_rx_size = size > 256 ? 256 : size;
        return ESP_OK;
    }
    if (kind == SCRIPT_INTERFACE_RS485) return rs485_object_set_rx_buffer_size(0, size);
    if (size > 4096) size = 4096;
    return uart_object_set_rx_buffer_size(0, size);
}

esp_err_t script_interface_write(script_interface_kind_t kind,
                                 const uint8_t *payload,
                                 size_t payload_len)
{
    if (!script_interface_is_open(kind)) return ESP_ERR_INVALID_STATE;
    if (!payload && payload_len > 0) return ESP_ERR_INVALID_ARG;
    if (kind == SCRIPT_INTERFACE_I2C) {
        return i2c_master_write_to_device(SCRIPT_I2C_PORT, (uint8_t)s_i2c_address,
                                          payload, payload_len,
                                          pdMS_TO_TICKS(SCRIPT_I2C_TIMEOUT_MS));
    }
    ssize_t written = kind == SCRIPT_INTERFACE_RS485
                          ? rs485_object_invoke_tx(0, payload, payload_len)
                          : uart_object_invoke_tx(0, payload, payload_len);
    return written == (ssize_t)payload_len ? ESP_OK : ESP_FAIL;
}

esp_err_t script_interface_read(script_interface_kind_t kind,
                                uint8_t *out,
                                size_t out_size,
                                size_t *out_len)
{
    if (!script_interface_is_open(kind)) return ESP_ERR_INVALID_STATE;
    if (!out || !out_len || out_size == 0) return ESP_ERR_INVALID_ARG;
    *out_len = 0;
    if (kind == SCRIPT_INTERFACE_I2C) {
        size_t read_len = (size_t)s_i2c_rx_size;
        if (read_len == 0 || read_len > out_size) read_len = out_size;
        esp_err_t err = i2c_master_read_from_device(SCRIPT_I2C_PORT, (uint8_t)s_i2c_address,
                                                    out, read_len,
                                                    pdMS_TO_TICKS(SCRIPT_I2C_TIMEOUT_MS));
        if (err == ESP_OK) *out_len = read_len;
        return err;
    }
    int32_t advanced = 0;
    ssize_t read = kind == SCRIPT_INTERFACE_RS485
                       ? rs485_object_invoke_rx(0, 0, out, out_size, &advanced)
                       : uart_object_invoke_rx(0, 0, out, out_size, &advanced);
    if (read < 0) return ESP_FAIL;
    *out_len = (size_t)read;
    return ESP_OK;
}
