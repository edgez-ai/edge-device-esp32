#include "edgez_platform.h"

static const edgez_platform_api_t *s_platform_api;

esp_err_t edgez_platform_register(const edgez_platform_api_t *api)
{
    if (!api) {
        return ESP_ERR_INVALID_ARG;
    }
    s_platform_api = api;
    return ESP_OK;
}

const edgez_platform_api_t *edgez_platform_get(void)
{
    return s_platform_api;
}
