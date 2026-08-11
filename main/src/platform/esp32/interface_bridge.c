#include "interface_bridge.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "local_interface.h"
#include "status_led.h"

#define I2C_MAX_INSTANCES 4
#define UART_MAX_INSTANCES 4
#define RS485_MAX_INSTANCES 4

static const char *TAG = "iface_bridge";

#ifndef CONFIG_IFACE_OPEN_DELAY_WHEN_DISABLED_MS
#define CONFIG_IFACE_OPEN_DELAY_WHEN_DISABLED_MS 500
#endif

#ifndef CONFIG_IFACE_BRIDGE_HEX_DUMP_LOG
#define CONFIG_IFACE_BRIDGE_HEX_DUMP_LOG 0
#endif

#define OPEN_DELAY_WHEN_DISABLED_MS CONFIG_IFACE_OPEN_DELAY_WHEN_DISABLED_MS
#define I2C_BRIDGE_PORT I2C_NUM_0
#define I2C_BRIDGE_FREQ_HZ 100000
#define I2C_BRIDGE_TIMEOUT_MS 1000

/* Simple I2C bridge state per instance */
typedef struct {
    int32_t sda_pin;
    int32_t scl_pin;
    uint32_t addr;
    bool bus_ready;
    uint8_t last_tx[32];
    size_t last_tx_len;
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t error_count;
} i2c_bridge_slot_t;

static i2c_bridge_slot_t s_i2c[I2C_MAX_INSTANCES];
static bool s_i2c_open[I2C_MAX_INSTANCES];
static bool s_i2c_driver_installed;
static int32_t s_i2c_driver_sda = -1;
static int32_t s_i2c_driver_scl = -1;

/* Simple UART bridge state per instance */
typedef struct {
    uart_port_t port;
    int32_t tx_pin;
    int32_t rx_pin;
    uint32_t baudrate;
    int rx_buffer_size;
    bool driver_installed;
    bool ready;
    uint8_t last_tx[64];
    size_t last_tx_len;
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t error_count;
} uart_bridge_slot_t;

static uart_bridge_slot_t s_uart[UART_MAX_INSTANCES];
static bool s_uart_open[UART_MAX_INSTANCES];
static uart_bridge_slot_t s_rs485[RS485_MAX_INSTANCES];
static bool s_rs485_open[RS485_MAX_INSTANCES];

static i2c_bridge_slot_t *slot_for(uint16_t instance_id);
static uart_port_t uart_port_for(uint16_t instance_id);
static uart_bridge_slot_t *uart_slot_for(uint16_t instance_id);
static uart_bridge_slot_t *rs485_slot_for(uint16_t instance_id);

static bool any_uart_open(void)
{
    for (size_t i = 0; i < UART_MAX_INSTANCES; i++) {
        if (s_uart_open[i]) return true;
    }
    return false;
}

static bool any_uart_installed(void)
{
    for (size_t i = 0; i < UART_MAX_INSTANCES; i++) {
        if (s_uart[i].driver_installed) return true;
    }
    return false;
}

static void close_all_uart(void)
{
    for (size_t i = 0; i < UART_MAX_INSTANCES; i++) {
        uart_bridge_slot_t *slot = uart_slot_for((uint16_t)i);
        if (!slot || (!slot->driver_installed && !slot->ready)) continue;
        if (slot->ready) {
            status_led_set_gpio_busy(slot->tx_pin, false);
        }
        if (slot->driver_installed) {
            uart_driver_delete(slot->port);
            slot->driver_installed = false;
            slot->rx_buffer_size = 0;
        }
        slot->ready = false;
        slot->last_tx_len = 0;
        s_uart_open[i] = false;
    }
}

static bool any_i2c_open(void)
{
    for (size_t i = 0; i < I2C_MAX_INSTANCES; i++) {
        if (s_i2c_open[i]) return true;
    }
    return false;
}

static void close_all_i2c(void)
{
    for (size_t i = 0; i < I2C_MAX_INSTANCES; i++) {
        if (!s_i2c_open[i]) continue;
        i2c_bridge_slot_t *slot = slot_for((uint16_t)i);
        if (slot) {
            slot->bus_ready = false;
            slot->last_tx_len = 0;
        }
        s_i2c_open[i] = false;
    }

    if (s_i2c_driver_installed) {
        i2c_driver_delete(I2C_BRIDGE_PORT);
        s_i2c_driver_installed = false;
        s_i2c_driver_sda = -1;
        s_i2c_driver_scl = -1;
    }
}

static uart_port_t uart_port_for(uint16_t instance_id)
{
    return (uart_port_t)(UART_NUM_1 + instance_id);
}

static uart_port_t rs485_port_for(uint16_t instance_id)
{
    /* ESP32-S3 has 3 UARTs: UART0 (console), UART1, UART2
     * Use UART2 for RS485 to avoid conflict with regular UART on UART1 */
#if SOC_UART_NUM > 2
    return (uart_port_t)(UART_NUM_2);
#else
    /* Fallback for chips with only 2 UARTs - share with UART */
    return (uart_port_t)(UART_NUM_1 + instance_id);
#endif
}

static uart_bridge_slot_t *uart_slot_for(uint16_t instance_id)
{
    if (instance_id >= UART_MAX_INSTANCES) return NULL;
    return &s_uart[instance_id];
}

static uart_bridge_slot_t *rs485_slot_for(uint16_t instance_id)
{
    if (instance_id >= RS485_MAX_INSTANCES) return NULL;
    return &s_rs485[instance_id];
}

static esp_err_t ensure_i2c_bus(i2c_bridge_slot_t *slot, int32_t sda, int32_t scl)
{
    if (slot->bus_ready && slot->sda_pin == sda && slot->scl_pin == scl &&
        s_i2c_driver_installed && s_i2c_driver_sda == sda && s_i2c_driver_scl == scl) {
        return ESP_OK;
    }

    if (s_i2c_driver_installed && (s_i2c_driver_sda != sda || s_i2c_driver_scl != scl)) {
        i2c_driver_delete(I2C_BRIDGE_PORT);
        s_i2c_driver_installed = false;
        s_i2c_driver_sda = -1;
        s_i2c_driver_scl = -1;
    }

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_BRIDGE_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_BRIDGE_PORT, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c param config failed sda=%ld scl=%ld: %s", (long)sda, (long)scl, esp_err_to_name(err));
        slot->bus_ready = false;
        return err;
    }

    if (!s_i2c_driver_installed) {
        err = i2c_driver_install(I2C_BRIDGE_PORT, I2C_MODE_MASTER, 0, 0, 0);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c driver install failed sda=%ld scl=%ld: %s", (long)sda, (long)scl, esp_err_to_name(err));
        slot->bus_ready = false;
        return err;
    }

    s_i2c_driver_installed = true;
    s_i2c_driver_sda = sda;
    s_i2c_driver_scl = scl;

    slot->sda_pin = sda;
    slot->scl_pin = scl;
    slot->bus_ready = true;
    return ESP_OK;
}

static esp_err_t ensure_i2c_device(i2c_bridge_slot_t *slot, uint32_t addr)
{
    if (!slot->bus_ready) return ESP_FAIL;
    slot->addr = addr & 0x7F;
    return ESP_OK;
}

static i2c_bridge_slot_t *slot_for(uint16_t instance_id)
{
    if (instance_id >= I2C_MAX_INSTANCES) return NULL;
    return &s_i2c[instance_id];
}

static esp_err_t bridge_i2c_open(uint16_t instance_id, bool open)
{
    i2c_bridge_slot_t *slot = slot_for(instance_id);
    if (!slot) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "i2c open inst=%u state=%s", (unsigned)instance_id, open ? "open" : "close");

    if (!open) {
        slot->bus_ready = false;
        slot->last_tx_len = 0;
        if (instance_id < I2C_MAX_INSTANCES) s_i2c_open[instance_id] = false;

        if (!any_i2c_open() && s_i2c_driver_installed) {
            i2c_driver_delete(I2C_BRIDGE_PORT);
            s_i2c_driver_installed = false;
            s_i2c_driver_sda = -1;
            s_i2c_driver_scl = -1;
        }

        bool i2c_uart_on = any_i2c_open() || any_uart_open();
        esp_err_t pin_err = gpio_set_level(GPIO_NUM_14, i2c_uart_on ? 1 : 0);
        if (pin_err != ESP_OK) {
            ESP_LOGW(TAG, "i2c close: set GPIO14=%d failed: %s", i2c_uart_on ? 1 : 0, esp_err_to_name(pin_err));
        }
        return ESP_OK;
    }

    if (any_uart_installed()) {
        ESP_LOGW(TAG, "i2c opening: auto-closing UART on shared pins");
        close_all_uart();
    }

    uint32_t addr = 0;
    int32_t sda = -1, scl = -1, rx_size = 0;
    if (i2c_object_get_runtime(instance_id, &addr, &sda, &scl, &rx_size) != ESP_OK) {
        ESP_LOGE(TAG, "runtime fetch failed for i2c inst %u", (unsigned)instance_id);
        return ESP_FAIL;
    }

    esp_err_t err = ensure_i2c_bus(slot, sda, scl);
    if (err != ESP_OK) return err;
    err = ensure_i2c_device(slot, addr);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "i2c ready inst=%u addr=0x%02lx sda=%ld scl=%ld rx_size=%ld", (unsigned)instance_id,
                 (unsigned long)addr, (long)sda, (long)scl, (long)rx_size);
    }
    if (err == ESP_OK && instance_id < I2C_MAX_INSTANCES) {
        s_i2c_open[instance_id] = true;

        bool i2c_uart_on = any_i2c_open() || any_uart_open();
        esp_err_t pin_err = gpio_set_level(GPIO_NUM_14, i2c_uart_on ? 1 : 0);
        if (pin_err != ESP_OK) {
            ESP_LOGW(TAG, "i2c open: set GPIO14=%d failed: %s", i2c_uart_on ? 1 : 0, esp_err_to_name(pin_err));
        }
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "i2c opened, delaying %d ms", OPEN_DELAY_WHEN_DISABLED_MS);
        vTaskDelay(pdMS_TO_TICKS(OPEN_DELAY_WHEN_DISABLED_MS));
    }
    return err;
}

static esp_err_t bridge_i2c_ensure_ready(uint16_t instance_id, i2c_bridge_slot_t *slot)
{
    if (!slot) return ESP_ERR_INVALID_ARG;

    if (slot->bus_ready) return ESP_OK;

    uint32_t addr = 0;
    int32_t sda = -1, scl = -1, rx_size = 0;
    if (i2c_object_get_runtime(instance_id, &addr, &sda, &scl, &rx_size) != ESP_OK) {
        ESP_LOGW(TAG, "runtime fetch failed inst=%u", (unsigned)instance_id);
        return ESP_FAIL;
    }

    esp_err_t err = ensure_i2c_bus(slot, sda, scl);
    if (err != ESP_OK) return err;
    err = ensure_i2c_device(slot, addr);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "lazy-open i2c inst=%u addr=0x%02lx sda=%ld scl=%ld", (unsigned)instance_id,
                 (unsigned long)addr, (long)sda, (long)scl);
        ESP_LOGI(TAG, "i2c lazy-open delaying %d ms", OPEN_DELAY_WHEN_DISABLED_MS);
        vTaskDelay(pdMS_TO_TICKS(OPEN_DELAY_WHEN_DISABLED_MS));
    }
    return err;
}

static ssize_t bridge_i2c_tx(uint16_t instance_id, const uint8_t *data, size_t len)
{
    i2c_bridge_slot_t *slot = slot_for(instance_id);
    if (!slot) return -1;
    if (!slot->bus_ready) {
        if (bridge_i2c_ensure_ready(instance_id, slot) != ESP_OK) {
            ESP_LOGW(TAG, "i2c tx inst=%u missing bus", (unsigned)instance_id);
            return -1;
        }
    }
#if CONFIG_IFACE_BRIDGE_HEX_DUMP_LOG
    ESP_LOGI(TAG, "i2c tx inst=%u len=%u", (unsigned)instance_id, (unsigned)len);
#endif

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) return -1;

    esp_err_t err = i2c_master_start(cmd);
    if (err == ESP_OK) err = i2c_master_write_byte(cmd, ((uint8_t)slot->addr << 1) | I2C_MASTER_WRITE, true);
    if (err == ESP_OK && len > 0) err = i2c_master_write(cmd, (uint8_t *)data, len, true);
    if (err == ESP_OK) err = i2c_master_stop(cmd);
    if (err == ESP_OK) {
        err = i2c_master_cmd_begin(I2C_BRIDGE_PORT, cmd, pdMS_TO_TICKS(I2C_BRIDGE_TIMEOUT_MS));
    }
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c tx fail inst=%u err=%s", (unsigned)instance_id, esp_err_to_name(err));
        slot->error_count++;
        i2c_object_update_counters(instance_id, slot->tx_bytes, slot->rx_bytes, slot->error_count, 0, 0, "tx_fail");
        return -1;
    }
    size_t copy = len < sizeof(slot->last_tx) ? len : sizeof(slot->last_tx);
    memcpy(slot->last_tx, data, copy);
    slot->last_tx_len = copy;
#if CONFIG_IFACE_BRIDGE_HEX_DUMP_LOG
    if (copy > 0) {
        ESP_LOGI(TAG, "i2c tx data len=%u", (unsigned)copy);
    }
#endif
    slot->tx_bytes += (uint32_t)len;
    i2c_object_update_counters(instance_id, slot->tx_bytes, slot->rx_bytes, slot->error_count, 0, 0, NULL);
    return (ssize_t)len;
}

static ssize_t bridge_i2c_rx(uint16_t instance_id, int32_t pos, uint8_t *out, size_t maxlen, int32_t *advanced)
{
    (void)pos;
    i2c_bridge_slot_t *slot = slot_for(instance_id);
    if (!slot) return -1;
    if (!slot->bus_ready) {
        if (bridge_i2c_ensure_ready(instance_id, slot) != ESP_OK) {
            ESP_LOGW(TAG, "i2c rx inst=%u missing bus", (unsigned)instance_id);
            return -1;
        }
    }

    uint32_t addr = 0;
    int32_t sda = -1, scl = -1, rx_size = 0;
    if (i2c_object_get_runtime(instance_id, &addr, &sda, &scl, &rx_size) != ESP_OK) {
        return -1;
    }

    size_t read_len = (rx_size > 0 && (size_t)rx_size <= maxlen) ? (size_t)rx_size : maxlen;
#if CONFIG_IFACE_BRIDGE_HEX_DUMP_LOG
    ESP_LOGI(TAG, "i2c rx inst=%u read_len=%u rx_size=%ld max=%u last_tx_len=%u", (unsigned)instance_id,
             (unsigned)read_len, (long)rx_size, (unsigned)maxlen, (unsigned)slot->last_tx_len);
#endif

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) return -1;

    esp_err_t err;
    if (slot->last_tx_len > 0) {
        /* Combine last TX as register pointer then read. */
        err = i2c_master_start(cmd);
        if (err == ESP_OK) err = i2c_master_write_byte(cmd, ((uint8_t)slot->addr << 1) | I2C_MASTER_WRITE, true);
        if (err == ESP_OK) err = i2c_master_write(cmd, slot->last_tx, slot->last_tx_len, true);
        if (err == ESP_OK) err = i2c_master_start(cmd);
        if (err == ESP_OK) err = i2c_master_write_byte(cmd, ((uint8_t)slot->addr << 1) | I2C_MASTER_READ, true);
        if (err == ESP_OK && read_len > 1) err = i2c_master_read(cmd, out, read_len - 1, I2C_MASTER_ACK);
        if (err == ESP_OK && read_len > 0) err = i2c_master_read_byte(cmd, out + read_len - 1, I2C_MASTER_NACK);
        if (err == ESP_OK) err = i2c_master_stop(cmd);
        if (err == ESP_OK) {
            err = i2c_master_cmd_begin(I2C_BRIDGE_PORT, cmd, pdMS_TO_TICKS(I2C_BRIDGE_TIMEOUT_MS));
        }
        slot->last_tx_len = 0;
    } else {
        err = i2c_master_start(cmd);
        if (err == ESP_OK) err = i2c_master_write_byte(cmd, ((uint8_t)slot->addr << 1) | I2C_MASTER_READ, true);
        if (err == ESP_OK && read_len > 1) err = i2c_master_read(cmd, out, read_len - 1, I2C_MASTER_ACK);
        if (err == ESP_OK && read_len > 0) err = i2c_master_read_byte(cmd, out + read_len - 1, I2C_MASTER_NACK);
        if (err == ESP_OK) err = i2c_master_stop(cmd);
        if (err == ESP_OK) {
            err = i2c_master_cmd_begin(I2C_BRIDGE_PORT, cmd, pdMS_TO_TICKS(I2C_BRIDGE_TIMEOUT_MS));
        }
    }
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c rx fail inst=%u err=%s", (unsigned)instance_id, esp_err_to_name(err));
        slot->error_count++;
        i2c_object_update_counters(instance_id, slot->tx_bytes, slot->rx_bytes, slot->error_count, 0, 0, "rx_fail");
        return -1;
    }
    slot->rx_bytes += (uint32_t)read_len;
    i2c_object_update_counters(instance_id, slot->tx_bytes, slot->rx_bytes, slot->error_count, 0, 0, NULL);
#if CONFIG_IFACE_BRIDGE_HEX_DUMP_LOG
    ESP_LOGI(TAG, "i2c rx inst=%u len=%u", (unsigned)instance_id, (unsigned)read_len);
    if (read_len > 0) {
        ESP_LOGI(TAG, "i2c rx data len=%u", (unsigned)read_len);
    }
#endif
    if (advanced) *advanced = pos + (int32_t)read_len;
    return (ssize_t)read_len;
}

static esp_err_t bridge_uart_open(uint16_t instance_id, bool open)
{
    uart_bridge_slot_t *slot = uart_slot_for(instance_id);
    if (!slot) return ESP_ERR_INVALID_ARG;

    uart_port_t port = uart_port_for(instance_id);

    uint32_t baud = 115200;
    int32_t tx_pin = -1, rx_pin = -1, rx_size = 0;
    if (uart_object_get_runtime(instance_id, &baud, &tx_pin, &rx_pin, &rx_size) != ESP_OK) {
        if (slot->baudrate > 0) {
            baud = slot->baudrate;
        }
    }

    ESP_LOGI(TAG,
             "uart open inst=%u port=%d state=%s baud=%lu",
             (unsigned)instance_id,
             (int)port,
             open ? "open" : "close",
             (unsigned long)baud);

    if (!open) {
        if (slot->ready) {
            (void)uart_wait_tx_done(port, pdMS_TO_TICKS(100));
            (void)uart_flush_input(port);
            status_led_set_gpio_busy(slot->tx_pin, false);
        }
        slot->ready = false;
        slot->last_tx_len = 0;
        if (instance_id < UART_MAX_INSTANCES) s_uart_open[instance_id] = false;

        bool i2c_uart_on = any_i2c_open() || any_uart_open();
        esp_err_t pin_err = gpio_set_level(GPIO_NUM_14, i2c_uart_on ? 1 : 0);
        if (pin_err != ESP_OK) {
            ESP_LOGW(TAG, "uart close: set GPIO14=%d failed: %s", i2c_uart_on ? 1 : 0, esp_err_to_name(pin_err));
        }
        return ESP_OK;
    }

    if (any_i2c_open()) {
        ESP_LOGW(TAG, "uart opening: auto-closing I2C on shared pins");
        close_all_i2c();
    }

    if (uart_object_get_runtime(instance_id, &baud, &tx_pin, &rx_pin, &rx_size) != ESP_OK) {
        ESP_LOGE(TAG, "runtime fetch failed for uart inst %u", (unsigned)instance_id);
        return ESP_FAIL;
    }

    uart_config_t cfg = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    int rx_buf = (rx_size > 0) ? (int)rx_size : 2048;
    if (slot->ready) {
        status_led_set_gpio_busy(slot->tx_pin, false);
        slot->ready = false;
        if (instance_id < UART_MAX_INSTANCES) s_uart_open[instance_id] = false;
    }

    esp_err_t err = ESP_OK;
    if (!slot->driver_installed) {
        /* uart_driver_install() allocates the RX ring buffer and several
         * synchronization objects before uart_module_enable() lazily creates
         * the UART context mutex. Under tight internal-memory conditions that
         * final lazy lock allocation aborts instead of returning an error.
         * Configure the port first so the mandatory context mutex is reserved
         * before the larger, fallible driver allocation. */
        err = uart_param_config(port, &cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "uart_param_config failed port=%d: %s", (int)port, esp_err_to_name(err));
            return err;
        }
        err = uart_driver_install(port, rx_buf, 0, 0, NULL, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "uart_driver_install failed port=%d: %s", (int)port, esp_err_to_name(err));
            return err;
        }
        slot->driver_installed = true;
        slot->port = port;
        slot->rx_buffer_size = rx_buf;
        ESP_LOGI(TAG,
                 "uart driver installed once inst=%u port=%d rx_buffer=%d",
                 (unsigned)instance_id,
                 (int)port,
                 rx_buf);
    } else {
        ESP_LOGI(TAG,
                 "uart driver reused inst=%u port=%d rx_buffer=%d requested=%d",
                 (unsigned)instance_id,
                 (int)port,
                 slot->rx_buffer_size,
                 rx_buf);
        err = uart_param_config(port, &cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "uart_param_config failed port=%d: %s", (int)port, esp_err_to_name(err));
            return err;
        }
    }

    status_led_set_gpio_busy(tx_pin, true);
    err = uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed port=%d tx=%ld rx=%ld: %s", (int)port, (long)tx_pin, (long)rx_pin, esp_err_to_name(err));
        status_led_set_gpio_busy(tx_pin, false);
        return err;
    }

    slot->port = port;
    slot->tx_pin = tx_pin;
    slot->rx_pin = rx_pin;
    slot->baudrate = baud;
    slot->ready = true;
    slot->last_tx_len = 0;
    if (instance_id < UART_MAX_INSTANCES) {
        s_uart_open[instance_id] = true;

        bool i2c_uart_on = any_i2c_open() || any_uart_open();
        esp_err_t pin_err = gpio_set_level(GPIO_NUM_14, i2c_uart_on ? 1 : 0);
        if (pin_err != ESP_OK) {
            ESP_LOGW(TAG, "uart open: set GPIO14=%d failed: %s", i2c_uart_on ? 1 : 0, esp_err_to_name(pin_err));
        }
    }
    ESP_LOGI(TAG, "uart opened, delaying %d ms", OPEN_DELAY_WHEN_DISABLED_MS);
    vTaskDelay(pdMS_TO_TICKS(OPEN_DELAY_WHEN_DISABLED_MS));
    ESP_LOGI(TAG, "uart ready inst=%u port=%d baud=%lu tx=%ld rx=%ld", (unsigned)instance_id, (int)port,
             (unsigned long)baud, (long)tx_pin, (long)rx_pin);
    return ESP_OK;
}

/* RS485 configuration from Kconfig */
#define RS485_TXD_PIN           (CONFIG_RS485_UART_TXD)
#define RS485_RXD_PIN           (CONFIG_RS485_UART_RXD)
#define RS485_RTS_PIN           (CONFIG_RS485_UART_RTS)
#define RS485_READ_TOUT         (CONFIG_RS485_READ_TIMEOUT)
#define RS485_RX_BUF_SIZE       (CONFIG_RS485_RX_BUFFER_SIZE)

static esp_err_t bridge_rs485_open(uint16_t instance_id, bool open)
{
    uart_bridge_slot_t *slot = rs485_slot_for(instance_id);
    if (!slot) return ESP_ERR_INVALID_ARG;

    uart_port_t port = rs485_port_for(instance_id);
    ESP_LOGI(TAG, "rs485 open inst=%u port=%d state=%s", (unsigned)instance_id, (int)port, open ? "open" : "close");

    if (!open) {
        uart_driver_delete(port);
        slot->ready = false;
        slot->last_tx_len = 0;
        if (instance_id < RS485_MAX_INSTANCES) s_rs485_open[instance_id] = false;

        bool enabled = false;
        if (rs485_object_is_enabled(instance_id, &enabled) == ESP_OK && !enabled) {
            esp_err_t pin_err = gpio_set_level(GPIO_NUM_13, 0);
            if (pin_err != ESP_OK) {
                ESP_LOGW(TAG, "rs485 close: set GPIO13=0 failed: %s", esp_err_to_name(pin_err));
            }
        }
        return ESP_OK;
    }

    uint32_t baud = 0;
    int32_t tx_pin = -1, rx_pin = -1, rx_size = 0;
    if (rs485_object_get_runtime(instance_id, &baud, NULL, &tx_pin, &rx_pin, &rx_size) != ESP_OK) {
        ESP_LOGE(TAG, "runtime fetch failed for rs485 inst %u", (unsigned)instance_id);
        return ESP_FAIL;
    }

    /* Use Kconfig defaults if pins are not configured by the local runtime. */
    if (tx_pin < 0) tx_pin = RS485_TXD_PIN;
    if (rx_pin < 0) rx_pin = RS485_RXD_PIN;

    uart_config_t cfg = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    size_t rx_buf = (rx_size > 0) ? (size_t)rx_size : RS485_RX_BUF_SIZE;
    /* Delete any existing driver first */
    uart_driver_delete(port);
    /* Reserve the UART context mutex before the RX buffer and driver
     * synchronization objects consume the remaining internal heap. */
    esp_err_t err = uart_param_config(port, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed port=%d: %s", (int)port, esp_err_to_name(err));
        return err;
    }

    err = uart_driver_install(port, rx_buf, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed port=%d: %s", (int)port, esp_err_to_name(err));
        return err;
    }

    /* Auto TX/RX transceiver: plain UART, no RTS direction control */
    err = uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed port=%d tx=%ld rx=%ld: %s", (int)port, (long)tx_pin, (long)rx_pin, esp_err_to_name(err));
        uart_driver_delete(port);
        return err;
    }

    err = uart_set_mode(port, UART_MODE_UART);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart_set_mode UART failed port=%d: %s", (int)port, esp_err_to_name(err));
    }

    /* Camera path uses normal UART RX polarity */

    /* Set read timeout of UART TOUT feature for proper packet detection */
    err = uart_set_rx_timeout(port, RS485_READ_TOUT);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart_set_rx_timeout failed port=%d: %s", (int)port, esp_err_to_name(err));
    }

    slot->port = port;
    slot->tx_pin = tx_pin;
    slot->rx_pin = rx_pin;
    slot->baudrate = baud;
    slot->ready = true;
    if (instance_id < RS485_MAX_INSTANCES) {
        s_rs485_open[instance_id] = true;
    }

    esp_err_t pin_err = gpio_set_level(GPIO_NUM_13, 1);
    if (pin_err != ESP_OK) {
        ESP_LOGW(TAG, "rs485 open: set GPIO13=1 failed: %s", esp_err_to_name(pin_err));
    }

    ESP_LOGI(TAG, "rs485 opened, delaying %d ms", OPEN_DELAY_WHEN_DISABLED_MS);
    vTaskDelay(pdMS_TO_TICKS(OPEN_DELAY_WHEN_DISABLED_MS));
    ESP_LOGI(TAG, "rs485 ready inst=%u port=%d baud=%lu tx=%ld rx=%ld mode=uart",
             (unsigned)instance_id, (int)port, (unsigned long)baud, (long)tx_pin, (long)rx_pin);
    return ESP_OK;
}

static esp_err_t bridge_rs485_ensure_ready(uint16_t instance_id, uart_bridge_slot_t *slot)
{
    if (!slot) return ESP_ERR_INVALID_ARG;

    if (slot->ready) return ESP_OK;
    return bridge_rs485_open(instance_id, true);
}

static esp_err_t bridge_uart_ensure_ready(uint16_t instance_id, uart_bridge_slot_t *slot)
{
    if (!slot) return ESP_ERR_INVALID_ARG;

    if (slot->ready) return ESP_OK;
    return bridge_uart_open(instance_id, true);
}

static ssize_t bridge_uart_tx(uint16_t instance_id, const uint8_t *data, size_t len)
{
    uart_bridge_slot_t *slot = uart_slot_for(instance_id);
    if (!slot) return -1;
    if (bridge_uart_ensure_ready(instance_id, slot) != ESP_OK) {
        ESP_LOGW(TAG, "uart tx inst=%u not ready", (unsigned)instance_id);
        return -1;
    }

#if CONFIG_IFACE_BRIDGE_HEX_DUMP_LOG
    ESP_LOGI(TAG, "uart tx inst=%u port=%d baud=%lu tx_pin=%ld rx_pin=%ld len=%u",
             (unsigned)instance_id, (int)slot->port, (unsigned long)slot->baudrate,
             (long)slot->tx_pin, (long)slot->rx_pin, (unsigned)len);
    if (len > 0) {
        ESP_LOG_BUFFER_HEXDUMP(TAG, data, len, ESP_LOG_INFO);
    }
#endif

    int sent = uart_write_bytes(slot->port, (const char *)data, (int)len);
    if (sent < 0) {
        ESP_LOGE(TAG, "uart_write_bytes failed: %d", sent);
        slot->error_count++;
        uart_object_update_counters(instance_id, slot->tx_bytes, slot->rx_bytes, slot->error_count, 0, 0, "tx_fail");
        return -1;
    }

#if CONFIG_IFACE_BRIDGE_HEX_DUMP_LOG
    ESP_LOGI(TAG, "uart tx sent=%d bytes", sent);
#endif

    size_t copy = (size_t)sent < sizeof(slot->last_tx) ? (size_t)sent : sizeof(slot->last_tx);
    if (copy > 0) {
        memcpy(slot->last_tx, data, copy);
        slot->last_tx_len = copy;
    }

    slot->tx_bytes += (uint32_t)sent;
    uart_object_update_counters(instance_id, slot->tx_bytes, slot->rx_bytes, slot->error_count, 0, 0, NULL);
    return (ssize_t)sent;
}

static ssize_t bridge_rs485_tx(uint16_t instance_id, const uint8_t *data, size_t len)
{
    uart_bridge_slot_t *slot = rs485_slot_for(instance_id);
    if (!slot) return -1;
    if (bridge_rs485_ensure_ready(instance_id, slot) != ESP_OK) {
        ESP_LOGW(TAG, "rs485 tx inst=%u not ready", (unsigned)instance_id);
        return -1;
    }

#if CONFIG_IFACE_BRIDGE_HEX_DUMP_LOG
    ESP_LOGI(TAG, "rs485 tx inst=%u port=%d baud=%lu tx_pin=%ld rx_pin=%ld len=%u",
             (unsigned)instance_id, (int)slot->port, (unsigned long)slot->baudrate,
             (long)slot->tx_pin, (long)slot->rx_pin, (unsigned)len);
    if (len > 0) {
        ESP_LOG_BUFFER_HEXDUMP(TAG, data, len, ESP_LOG_INFO);
    }
#endif

    /* Flush RX buffer before TX to clear any stale data */
    uart_flush_input(slot->port);

    int sent = uart_write_bytes(slot->port, (const char *)data, (int)len);
    if (sent < 0) {
        ESP_LOGE(TAG, "uart_write_bytes failed: %d", sent);
        slot->error_count++;
        rs485_object_update_counters(instance_id, slot->tx_bytes, slot->rx_bytes, slot->error_count, 0, 0, "tx_fail");
        return -1;
    }

    /* Wait for TX FIFO to empty before switching to RX (critical for half-duplex RS485) */
    esp_err_t err = uart_wait_tx_done(slot->port, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "rs485 tx wait failed: %s", esp_err_to_name(err));
    }

#if CONFIG_IFACE_BRIDGE_HEX_DUMP_LOG
    ESP_LOGI(TAG, "rs485 tx sent=%d bytes", sent);
#endif

    size_t copy = (size_t)sent < sizeof(slot->last_tx) ? (size_t)sent : sizeof(slot->last_tx);
    if (copy > 0) {
        memcpy(slot->last_tx, data, copy);
        slot->last_tx_len = copy;
    }

    slot->tx_bytes += (uint32_t)sent;
    rs485_object_update_counters(instance_id, slot->tx_bytes, slot->rx_bytes, slot->error_count, 0, 0, NULL);
    return (ssize_t)sent;
}

static ssize_t bridge_uart_rx(uint16_t instance_id, int32_t pos, uint8_t *out, size_t maxlen, int32_t *advanced)
{
    (void)pos;
    uart_bridge_slot_t *slot = uart_slot_for(instance_id);
    if (!slot) return -1;
    if (bridge_uart_ensure_ready(instance_id, slot) != ESP_OK) {
        ESP_LOGW(TAG, "uart rx inst=%u not ready", (unsigned)instance_id);
        return -1;
    }

    uint32_t baud = 0;
    int32_t tx_pin = -1, rx_pin = -1, rx_size = 0;
    if (uart_object_get_runtime(instance_id, &baud, &tx_pin, &rx_pin, &rx_size) != ESP_OK) {
        return -1;
    }

    size_t read_len = (rx_size > 0 && (size_t)rx_size <= maxlen) ? (size_t)rx_size : maxlen;
    int got = uart_read_bytes(slot->port, out, (uint32_t)read_len, pdMS_TO_TICKS(50));
    
#if CONFIG_IFACE_BRIDGE_HEX_DUMP_LOG
    ESP_LOGI(TAG, "uart rx inst=%u port=%d read_len=%u got=%d",
             (unsigned)instance_id, (int)slot->port, (unsigned)read_len, got);
    if (got > 0) {
        ESP_LOG_BUFFER_HEXDUMP(TAG, out, (size_t)got, ESP_LOG_INFO);
    }
#endif
    
    if (got < 0) {
        slot->error_count++;
        uart_object_update_counters(instance_id, slot->tx_bytes, slot->rx_bytes, slot->error_count, 0, 0, "rx_fail");
        return -1;
    }
    if (got == 0) {
        if (advanced) *advanced = pos;
        return 0;
    }

    slot->rx_bytes += (uint32_t)got;
    uart_object_update_counters(instance_id, slot->tx_bytes, slot->rx_bytes, slot->error_count, 0, 0, NULL);
    if (advanced) *advanced = pos + got;
    return (ssize_t)got;
}

static ssize_t bridge_rs485_rx(uint16_t instance_id, int32_t pos, uint8_t *out, size_t maxlen, int32_t *advanced)
{
    (void)pos;
    uart_bridge_slot_t *slot = rs485_slot_for(instance_id);
    if (!slot) return -1;
    if (bridge_rs485_ensure_ready(instance_id, slot) != ESP_OK) {
        ESP_LOGW(TAG, "rs485 rx inst=%u not ready", (unsigned)instance_id);
        return -1;
    }

    uint32_t baud = 0;
    int32_t tx_pin = -1, rx_pin = -1, rx_size = 0;
    if (rs485_object_get_runtime(instance_id, &baud, NULL, &tx_pin, &rx_pin, &rx_size) != ESP_OK) {
        return -1;
    }

    size_t read_len = (rx_size > 0 && (size_t)rx_size <= maxlen) ? (size_t)rx_size : maxlen;
    /* Use longer timeout for RS485 - Modbus slaves may need time to respond */
    int got = uart_read_bytes(slot->port, out, (uint32_t)read_len, pdMS_TO_TICKS(100));

#if CONFIG_IFACE_BRIDGE_HEX_DUMP_LOG
    ESP_LOGI(TAG, "rs485 rx inst=%u port=%d read_len=%u got=%d",
             (unsigned)instance_id, (int)slot->port, (unsigned)read_len, got);
    if (got > 0) {
        ESP_LOG_BUFFER_HEXDUMP(TAG, out, (size_t)got, ESP_LOG_INFO);
    }
#endif

    if (got < 0) {
        slot->error_count++;
        rs485_object_update_counters(instance_id, slot->tx_bytes, slot->rx_bytes, slot->error_count, 0, 0, "rx_fail");
        return -1;
    }
    if (got == 0) {
        if (advanced) *advanced = pos;
        return 0;
    }

    slot->rx_bytes += (uint32_t)got;
    rs485_object_update_counters(instance_id, slot->tx_bytes, slot->rx_bytes, slot->error_count, 0, 0, NULL);
    if (advanced) *advanced = pos + got;
    return (ssize_t)got;
}

void i2c_bridge_clear_pending_tx(uint16_t instance_id)
{
    if (instance_id >= I2C_MAX_INSTANCES) return;
    s_i2c[instance_id].last_tx_len = 0;
}

void interface_bridge_register(void)
{
    /* Make sure this tag prints at info level even if global default is lower */
    esp_log_level_set(TAG, ESP_LOG_INFO);
    i2c_object_set_handlers(bridge_i2c_tx, bridge_i2c_rx, bridge_i2c_open);
    uart_object_set_handlers(bridge_uart_tx, bridge_uart_rx, bridge_uart_open);
    rs485_object_set_handlers(bridge_rs485_tx, bridge_rs485_rx, bridge_rs485_open);
    ESP_LOGI(TAG, "I2C bridge handlers registered");
    ESP_LOGI(TAG, "UART bridge handlers registered");
    ESP_LOGI(TAG, "RS485 bridge handlers registered");
}
