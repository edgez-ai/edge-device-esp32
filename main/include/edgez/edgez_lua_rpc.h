#ifndef EDGEZ_LUA_RPC_H
#define EDGEZ_LUA_RPC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "edgez_frame_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

bool edgez_lua_rpc_is_request(const uint8_t *payload, size_t payload_len);
void edgez_lua_rpc_handle(const uint8_t *payload,
                          size_t payload_len,
                          edgez_frame_send_fn send_fn,
                          void *send_ctx);

#ifdef __cplusplus
}
#endif

#endif
