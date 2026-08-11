#include "sampling.h"

#include <string.h>

void sample_refresh_script_cache_from_selectors(void)
{
}

void sample_handle_sampling(void)
{
}

bool sample_get_latest_sensor_data(sample_sensor_data_t *out)
{
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }
    return false;
}

size_t sample_script_global_buffer_get_length(void)
{
    return 0;
}

size_t sample_script_global_buffer_get_serving_length(void)
{
    return 0;
}

esp_err_t sample_script_global_buffer_publish_for_serving(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t sample_script_global_buffer_acquire_serving(const uint8_t **data,
                                                      size_t *length)
{
    if (data != NULL) {
        *data = NULL;
    }
    if (length != NULL) {
        *length = 0;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

void sample_script_global_buffer_release_serving(void)
{
}

const uint8_t *sample_script_global_buffer_get_data(void)
{
    return NULL;
}
