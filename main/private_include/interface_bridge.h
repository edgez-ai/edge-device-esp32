#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Register generic I2C, RS485, and UART interface bridges. */
void interface_bridge_register(void);

/* Clear pending TX data for an I2C bridge instance.
 * This ensures the next RX uses a pure read transaction instead of
 * a combined write-then-read transaction, which is required for sensors
 * that need separate write-then-read sequences (e.g. SHT3x). */
void i2c_bridge_clear_pending_tx(uint16_t instance_id);

#ifdef __cplusplus
}
#endif
