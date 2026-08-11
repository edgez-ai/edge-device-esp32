#include "local_interface.h"

#include <string.h>

#define LOCAL_INTERFACE_INSTANCES 4

typedef struct {
    bool used;
    bool enabled;
    uint32_t address_or_baud;
    uint32_t unit_id;
    int32_t tx_pin;
    int32_t rx_pin;
    int32_t rx_buffer_size;
} local_interface_state_t;

typedef struct {
    local_interface_state_t state[LOCAL_INTERFACE_INSTANCES];
    interface_tx_handler_t tx;
    interface_rx_handler_t rx;
    interface_open_handler_t open;
} local_interface_t;

static local_interface_t s_i2c;
static local_interface_t s_rs485;
static local_interface_t s_uart;

static local_interface_state_t *local_state(local_interface_t *iface, uint16_t id, bool create)
{
    if (!iface || id >= LOCAL_INTERFACE_INSTANCES) return NULL;
    local_interface_state_t *state = &iface->state[id];
    if (create && !state->used) {
        memset(state, 0, sizeof(*state));
        state->used = true;
        state->enabled = true;
        state->rx_buffer_size = 4096;
    }
    return state->used ? state : NULL;
}

static esp_err_t local_open(local_interface_t *iface, uint16_t id, bool open)
{
    local_interface_state_t *state = local_state(iface, id, true);
    if (!state || !iface->open) return ESP_ERR_INVALID_STATE;
    esp_err_t err = iface->open(id, open);
    if (err == ESP_OK) state->enabled = true;
    return err;
}

static ssize_t local_tx(local_interface_t *iface, uint16_t id, const uint8_t *data, size_t len)
{
    if (!local_state(iface, id, true) || !iface->tx) return -1;
    return iface->tx(id, data, len);
}

static ssize_t local_rx(local_interface_t *iface, uint16_t id, int32_t pos,
                        uint8_t *out, size_t maxlen, int32_t *advanced)
{
    if (!local_state(iface, id, true) || !iface->rx) return -1;
    return iface->rx(id, pos, out, maxlen, advanced);
}

static esp_err_t local_enabled(local_interface_t *iface, uint16_t id, bool *enabled)
{
    if (!enabled) return ESP_ERR_INVALID_ARG;
    local_interface_state_t *state = local_state(iface, id, true);
    if (!state) return ESP_ERR_INVALID_ARG;
    *enabled = state->enabled;
    return ESP_OK;
}

static esp_err_t local_cursor(local_interface_t *iface, uint16_t id, int32_t pos)
{
    (void)pos;
    return local_state(iface, id, true) ? ESP_OK : ESP_ERR_INVALID_ARG;
}

static esp_err_t local_rx_size(local_interface_t *iface, uint16_t id, int32_t size)
{
    if (size <= 0) return ESP_ERR_INVALID_ARG;
    local_interface_state_t *state = local_state(iface, id, true);
    if (!state) return ESP_ERR_INVALID_ARG;
    state->rx_buffer_size = size;
    return ESP_OK;
}

static void local_handlers(local_interface_t *iface, interface_tx_handler_t tx,
                           interface_rx_handler_t rx, interface_open_handler_t open)
{
    iface->tx = tx;
    iface->rx = rx;
    iface->open = open;
}

esp_err_t i2c_object_set_instance(uint16_t id, const i2c_instance_cfg_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    local_interface_state_t *s = local_state(&s_i2c, id, true);
    if (!s) return ESP_ERR_INVALID_ARG;
    s->enabled = cfg->enabled;
    s->address_or_baud = cfg->i2c_address;
    s->tx_pin = cfg->tx_pin;
    s->rx_pin = cfg->rx_pin;
    s->rx_buffer_size = cfg->rx_buffer_size;
    return ESP_OK;
}

esp_err_t rs485_object_set_instance(uint16_t id, const rs485_instance_cfg_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    local_interface_state_t *s = local_state(&s_rs485, id, true);
    if (!s) return ESP_ERR_INVALID_ARG;
    s->enabled = cfg->enabled;
    s->address_or_baud = cfg->baudrate;
    s->unit_id = cfg->modbus_unit_id;
    s->tx_pin = cfg->tx_pin;
    s->rx_pin = cfg->rx_pin;
    s->rx_buffer_size = cfg->rx_buffer_size;
    return ESP_OK;
}

esp_err_t uart_object_set_instance(uint16_t id, const uart_instance_cfg_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    local_interface_state_t *s = local_state(&s_uart, id, true);
    if (!s) return ESP_ERR_INVALID_ARG;
    s->enabled = cfg->enabled;
    s->address_or_baud = cfg->baudrate;
    s->tx_pin = cfg->tx_pin;
    s->rx_pin = cfg->rx_pin;
    s->rx_buffer_size = cfg->rx_buffer_size;
    return ESP_OK;
}

esp_err_t i2c_object_get_runtime(uint16_t id, uint32_t *addr, int32_t *tx, int32_t *rx, int32_t *size)
{
    local_interface_state_t *s = local_state(&s_i2c, id, true);
    if (!s) return ESP_ERR_INVALID_ARG;
    if (addr && s->address_or_baud) *addr = s->address_or_baud;
    if (tx && s->tx_pin > 0) *tx = s->tx_pin;
    if (rx && s->rx_pin > 0) *rx = s->rx_pin;
    if (size && s->rx_buffer_size > 0) *size = s->rx_buffer_size;
    return ESP_OK;
}

esp_err_t rs485_object_get_runtime(uint16_t id, uint32_t *baud, uint32_t *unit,
                                   int32_t *tx, int32_t *rx, int32_t *size)
{
    local_interface_state_t *s = local_state(&s_rs485, id, true);
    if (!s) return ESP_ERR_INVALID_ARG;
    if (baud && s->address_or_baud) *baud = s->address_or_baud;
    if (unit && s->unit_id) *unit = s->unit_id;
    if (tx && s->tx_pin > 0) *tx = s->tx_pin;
    if (rx && s->rx_pin > 0) *rx = s->rx_pin;
    if (size && s->rx_buffer_size > 0) *size = s->rx_buffer_size;
    return ESP_OK;
}

esp_err_t uart_object_get_runtime(uint16_t id, uint32_t *baud, int32_t *tx, int32_t *rx, int32_t *size)
{
    local_interface_state_t *s = local_state(&s_uart, id, true);
    if (!s) return ESP_ERR_INVALID_ARG;
    if (baud && s->address_or_baud) *baud = s->address_or_baud;
    if (tx && s->tx_pin > 0) *tx = s->tx_pin;
    if (rx && s->rx_pin > 0) *rx = s->rx_pin;
    if (size && s->rx_buffer_size > 0) *size = s->rx_buffer_size;
    return ESP_OK;
}

esp_err_t i2c_object_update_runtime(uint16_t id, uint32_t addr, int32_t tx, int32_t rx, int32_t size)
{ i2c_instance_cfg_t c = {.enabled=true,.i2c_address=addr,.tx_pin=tx,.rx_pin=rx,.rx_buffer_size=size}; return i2c_object_set_instance(id, &c); }
esp_err_t rs485_object_update_runtime(uint16_t id, uint32_t baud, uint32_t unit, int32_t tx, int32_t rx, int32_t size)
{ rs485_instance_cfg_t c = {.enabled=true,.baudrate=baud,.modbus_unit_id=unit,.tx_pin=tx,.rx_pin=rx,.rx_buffer_size=size}; return rs485_object_set_instance(id, &c); }

void i2c_object_set_handlers(interface_tx_handler_t tx, interface_rx_handler_t rx, interface_open_handler_t open) { local_handlers(&s_i2c, tx, rx, open); }
void rs485_object_set_handlers(interface_tx_handler_t tx, interface_rx_handler_t rx, interface_open_handler_t open) { local_handlers(&s_rs485, tx, rx, open); }
void uart_object_set_handlers(interface_tx_handler_t tx, interface_rx_handler_t rx, interface_open_handler_t open) { local_handlers(&s_uart, tx, rx, open); }

esp_err_t i2c_object_is_enabled(uint16_t id, bool *v) { return local_enabled(&s_i2c, id, v); }
esp_err_t rs485_object_is_enabled(uint16_t id, bool *v) { return local_enabled(&s_rs485, id, v); }
esp_err_t uart_object_is_enabled(uint16_t id, bool *v) { return local_enabled(&s_uart, id, v); }
esp_err_t i2c_object_invoke_open(uint16_t id, bool v) { return local_open(&s_i2c, id, v); }
esp_err_t rs485_object_invoke_open(uint16_t id, bool v) { return local_open(&s_rs485, id, v); }
esp_err_t uart_object_invoke_open(uint16_t id, bool v) { return local_open(&s_uart, id, v); }
ssize_t i2c_object_invoke_tx(uint16_t id, const uint8_t *p, size_t n) { return local_tx(&s_i2c, id, p, n); }
ssize_t rs485_object_invoke_tx(uint16_t id, const uint8_t *p, size_t n) { return local_tx(&s_rs485, id, p, n); }
ssize_t uart_object_invoke_tx(uint16_t id, const uint8_t *p, size_t n) { return local_tx(&s_uart, id, p, n); }
ssize_t i2c_object_invoke_rx(uint16_t id, int32_t p, uint8_t *o, size_t n, int32_t *a) { return local_rx(&s_i2c, id, p, o, n, a); }
ssize_t rs485_object_invoke_rx(uint16_t id, int32_t p, uint8_t *o, size_t n, int32_t *a) { return local_rx(&s_rs485, id, p, o, n, a); }
ssize_t uart_object_invoke_rx(uint16_t id, int32_t p, uint8_t *o, size_t n, int32_t *a) { return local_rx(&s_uart, id, p, o, n, a); }
esp_err_t i2c_object_set_rx_cursor(uint16_t id, int32_t p) { return local_cursor(&s_i2c, id, p); }
esp_err_t rs485_object_set_rx_cursor(uint16_t id, int32_t p) { return local_cursor(&s_rs485, id, p); }
esp_err_t uart_object_set_rx_cursor(uint16_t id, int32_t p) { return local_cursor(&s_uart, id, p); }
esp_err_t i2c_object_set_rx_buffer_size(uint16_t id, int32_t n) { return local_rx_size(&s_i2c, id, n); }
esp_err_t rs485_object_set_rx_buffer_size(uint16_t id, int32_t n) { return local_rx_size(&s_rs485, id, n); }
esp_err_t uart_object_set_rx_buffer_size(uint16_t id, int32_t n) { return local_rx_size(&s_uart, id, n); }

esp_err_t i2c_object_update_counters(uint16_t id, uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e, const char *f) { (void)id;(void)a;(void)b;(void)c;(void)d;(void)e;(void)f; return ESP_OK; }
esp_err_t rs485_object_update_counters(uint16_t id, uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e, const char *f) { (void)id;(void)a;(void)b;(void)c;(void)d;(void)e;(void)f; return ESP_OK; }
esp_err_t uart_object_update_counters(uint16_t id, uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e, const char *f) { (void)id;(void)a;(void)b;(void)c;(void)d;(void)e;(void)f; return ESP_OK; }
