#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

bool l76k_gps_supported(void);
esp_err_t l76k_gps_configure(bool enabled, uint32_t update_interval_seconds);
bool l76k_gps_get_latest(float *latitude, float *longitude,
                         uint64_t *timestamp_ms);

#ifdef __cplusplus
}
#endif
