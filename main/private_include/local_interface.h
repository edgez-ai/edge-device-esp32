// Local sensor interface configuration and bridge contracts.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	IFACE_TYPE_UART = 0,
	IFACE_TYPE_I2C = 1,
	IFACE_TYPE_UVC = 2,
	IFACE_TYPE_MODBUS_UART = 3,
	IFACE_TYPE_BLE = 4,
} interface_type_t;

typedef ssize_t (*interface_tx_handler_t)(uint16_t instance_id, const uint8_t *data, size_t len);
typedef ssize_t (*interface_rx_handler_t)(uint16_t instance_id, int32_t pos, uint8_t *out, size_t maxlen, int32_t *advanced);
typedef esp_err_t (*interface_open_handler_t)(uint16_t instance_id, bool open);

typedef struct {
	bool enabled;
	bool open_state;
	uint32_t i2c_address;
	uint32_t mode;
	uint32_t stats_window_ms;
	uint32_t tx_rate;
	uint32_t rx_rate;
	int32_t rx_buffer_size;
	int32_t tx_pin;
	int32_t rx_pin;
} i2c_instance_cfg_t;

typedef struct {
	bool enabled;
	bool open_state;
	uint32_t baudrate;
	uint32_t modbus_unit_id;
	uint32_t mode;
	uint32_t stats_window_ms;
	uint32_t tx_rate;
	uint32_t rx_rate;
	int32_t rx_buffer_size;
	int32_t tx_pin;
	int32_t rx_pin;
} rs485_instance_cfg_t;

typedef struct {
	bool enabled;
	bool open_state;
	uint32_t baudrate;
	uint32_t mode;
	uint32_t stats_window_ms;
	uint32_t tx_rate;
	uint32_t rx_rate;
	int32_t rx_buffer_size;
	int32_t tx_pin;
	int32_t rx_pin;
} uart_instance_cfg_t;

esp_err_t i2c_object_set_instance(uint16_t instance_id, const i2c_instance_cfg_t *cfg);
esp_err_t i2c_object_update_counters(uint16_t instance_id, uint32_t tx_bytes, uint32_t rx_bytes,
									 uint32_t error_count, uint32_t tx_rate, uint32_t rx_rate,
									 const char *last_error);
esp_err_t i2c_object_set_rx_cursor(uint16_t instance_id, int32_t pos);

esp_err_t i2c_object_set_rx_buffer_size(uint16_t instance_id, int32_t size);
void i2c_object_set_handlers(interface_tx_handler_t tx_handler,
							 interface_rx_handler_t rx_handler,
							 interface_open_handler_t open_handler);
esp_err_t i2c_object_get_runtime(uint16_t instance_id, uint32_t *i2c_addr,
					 int32_t *sda_pin, int32_t *scl_pin,
					 int32_t *rx_buffer_size);
esp_err_t i2c_object_update_runtime(uint16_t instance_id, uint32_t i2c_addr,
							 int32_t sda_pin, int32_t scl_pin,
							 int32_t rx_buffer_size);
esp_err_t i2c_object_is_enabled(uint16_t instance_id, bool *enabled);

/* Invoke bridge handlers directly (used by Lua I2C bindings to share the bus) */
esp_err_t i2c_object_invoke_open(uint16_t instance_id, bool open);
ssize_t i2c_object_invoke_tx(uint16_t instance_id, const uint8_t *data, size_t len);
ssize_t i2c_object_invoke_rx(uint16_t instance_id, int32_t pos, uint8_t *out, size_t maxlen, int32_t *advanced);

esp_err_t rs485_object_set_instance(uint16_t instance_id, const rs485_instance_cfg_t *cfg);
esp_err_t rs485_object_update_counters(uint16_t instance_id, uint32_t tx_bytes, uint32_t rx_bytes,
									   uint32_t error_count, uint32_t tx_rate, uint32_t rx_rate,
									   const char *last_error);
esp_err_t rs485_object_set_rx_cursor(uint16_t instance_id, int32_t pos);

esp_err_t rs485_object_set_rx_buffer_size(uint16_t instance_id, int32_t size);
void rs485_object_set_handlers(interface_tx_handler_t tx_handler,
							   interface_rx_handler_t rx_handler,
							   interface_open_handler_t open_handler);
esp_err_t rs485_object_get_runtime(uint16_t instance_id, uint32_t *baudrate, uint32_t *modbus_unit_id,
					 int32_t *tx_pin, int32_t *rx_pin,
					 int32_t *rx_buffer_size);
esp_err_t rs485_object_update_runtime(uint16_t instance_id, uint32_t baudrate, uint32_t modbus_unit_id,
							   int32_t tx_pin, int32_t rx_pin,
							   int32_t rx_buffer_size);
esp_err_t rs485_object_is_enabled(uint16_t instance_id, bool *enabled);
esp_err_t rs485_object_invoke_open(uint16_t instance_id, bool open);
ssize_t rs485_object_invoke_tx(uint16_t instance_id, const uint8_t *data, size_t len);
ssize_t rs485_object_invoke_rx(uint16_t instance_id, int32_t pos, uint8_t *out, size_t maxlen, int32_t *advanced);

esp_err_t uart_object_set_instance(uint16_t instance_id, const uart_instance_cfg_t *cfg);
esp_err_t uart_object_update_counters(uint16_t instance_id, uint32_t tx_bytes, uint32_t rx_bytes,
								  uint32_t error_count, uint32_t tx_rate, uint32_t rx_rate,
								  const char *last_error);
esp_err_t uart_object_set_rx_cursor(uint16_t instance_id, int32_t pos);
esp_err_t uart_object_set_rx_buffer_size(uint16_t instance_id, int32_t size);
void uart_object_set_handlers(interface_tx_handler_t tx_handler,
						  interface_rx_handler_t rx_handler,
						  interface_open_handler_t open_handler);
esp_err_t uart_object_get_runtime(uint16_t instance_id, uint32_t *baudrate,
						 int32_t *tx_pin, int32_t *rx_pin,
						 int32_t *rx_buffer_size);
esp_err_t uart_object_is_enabled(uint16_t instance_id, bool *enabled);
esp_err_t uart_object_invoke_open(uint16_t instance_id, bool open);
ssize_t uart_object_invoke_tx(uint16_t instance_id, const uint8_t *data, size_t len);
ssize_t uart_object_invoke_rx(uint16_t instance_id, int32_t pos, uint8_t *out, size_t maxlen, int32_t *advanced);

#ifdef __cplusplus
}
#endif
