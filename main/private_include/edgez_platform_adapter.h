#pragma once

#include "esp_err.h"

/** Register this application's services with the host-independent EdgeZ SDK. */
esp_err_t edgez_platform_adapter_init(void);
