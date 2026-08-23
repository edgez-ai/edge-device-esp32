#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OPENMANET_COMMS_MAGIC_LEN 4U

/* Phone payload: OMC1, talkgroup port (BE16), raw 48 kHz/20 ms Opus frame. */
esp_err_t openmanet_comms_send_phone_frame(const uint8_t *payload,
                                            size_t payload_len);

/* Consumes an inner Ethernet frame delivered by BATMAN when it is a supported
 * OpenMANET Comms RTP talkgroup packet. */
bool openmanet_comms_handle_frame(const uint8_t *frame, size_t frame_len);

#ifdef __cplusplus
}
#endif
