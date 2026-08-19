#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void openmanet_alfred_set_hostname(const char *hostname);
void openmanet_alfred_handle_frame(const uint8_t originator[6],
                                   const uint8_t *frame, size_t frame_len);

#ifdef __cplusplus
}
#endif
