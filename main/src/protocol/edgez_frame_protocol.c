#include "edgez_frame_protocol.h"

#include "esp_err.h"
#include "esp_log.h"
#include "edgez_lua_rpc.h"
#include "halow_sync_bridge.h"

static const char *TAG = "edgez_frame";

static uint16_t read_le16(const uint8_t *value)
{
    return (uint16_t)value[0] | ((uint16_t)value[1] << 8);
}

void edgez_frame_protocol_handle_frame(const uint8_t *frame,
                                       uint16_t frame_len,
                                       edgez_frame_send_fn send_fn,
                                       void *send_ctx,
                                       bool from_forward_channel)
{
    if (!frame || frame_len < EDGEZ_FRAME_HEADER_LEN) {
        ESP_LOGW(TAG, "Frame is missing or shorter than its header");
        return;
    }

    if (frame[0] != EDGEZ_FRAME_MAGIC_0 || frame[1] != EDGEZ_FRAME_MAGIC_1) {
        ESP_LOGW(TAG, "Invalid frame magic");
        return;
    }

    uint16_t payload_len = read_le16(&frame[2]);
    if (payload_len > EDGEZ_FRAME_MAX_PAYLOAD ||
        frame_len != (uint16_t)(EDGEZ_FRAME_HEADER_LEN + payload_len)) {
        ESP_LOGW(TAG,
                 "Invalid frame length: frame=%u payload=%u",
                 (unsigned)frame_len,
                 (unsigned)payload_len);
        return;
    }

    const uint8_t *payload = &frame[EDGEZ_FRAME_HEADER_LEN];
    if (edgez_lua_rpc_is_request(payload, payload_len)) {
        edgez_lua_rpc_handle(payload, payload_len, send_fn, send_ctx);
        return;
    }

    bool sent_status_response = false;
    esp_err_t err = halow_sync_bridge_handle_to_radio(
        payload,
        payload_len,
        0,
        &sent_status_response,
        from_forward_channel);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Protobuf frame handling failed: %s", esp_err_to_name(err));
    }
}
