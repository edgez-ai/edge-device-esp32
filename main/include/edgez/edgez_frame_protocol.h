#ifndef EDGEZ_FRAME_PROTOCOL_H
#define EDGEZ_FRAME_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EDGEZ_FRAME_MAGIC_0 'E'
#define EDGEZ_FRAME_MAGIC_1 'Z'
#define EDGEZ_FRAME_HEADER_LEN 4
#define EDGEZ_FRAME_MAX_PAYLOAD 512
#define EDGEZ_FRAME_MAX_LEN (EDGEZ_FRAME_HEADER_LEN + EDGEZ_FRAME_MAX_PAYLOAD)

typedef void (*edgez_frame_send_fn)(void *ctx,
                                    const uint8_t *payload,
                                    uint16_t payload_len);

void edgez_frame_protocol_handle_frame(const uint8_t *frame,
                                       uint16_t frame_len,
                                       edgez_frame_send_fn send_fn,
                                       void *send_ctx,
                                       bool from_forward_channel);

#ifdef __cplusplus
}
#endif

#endif
