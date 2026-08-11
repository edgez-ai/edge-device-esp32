#include "edgez_lua_rpc.h"

#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#ifdef CONFIG_EDGEZ_SENSOR_SCRIPTING
#include "edgez_platform.h"
#endif

#define LUA_RPC_VERSION 1
#define LUA_RPC_REQUEST_HEADER_LEN 9
#define LUA_RPC_RESPONSE_HEADER_LEN 12

enum {
    LUA_RPC_OP_CONNECT = 1,
    LUA_RPC_OP_CLOSE = 2,
    LUA_RPC_OP_RESET_RX_CURSOR = 3,
    LUA_RPC_OP_SET_RX_SIZE = 4,
    LUA_RPC_OP_WRITE = 5,
    LUA_RPC_OP_READ = 6,
};

static const char *TAG = "edgez_lua_rpc";

static uint16_t read_le16(const uint8_t *value)
{
    return (uint16_t)value[0] | ((uint16_t)value[1] << 8);
}

static int16_t read_le_i16(const uint8_t *value)
{
    return (int16_t)read_le16(value);
}

static uint32_t read_le32(const uint8_t *value)
{
    return (uint32_t)value[0] |
           ((uint32_t)value[1] << 8) |
           ((uint32_t)value[2] << 16) |
           ((uint32_t)value[3] << 24);
}

static void write_le16(uint8_t *value, uint16_t input)
{
    value[0] = (uint8_t)(input & 0xff);
    value[1] = (uint8_t)(input >> 8);
}

static void write_le32(uint8_t *value, uint32_t input)
{
    value[0] = (uint8_t)(input & 0xff);
    value[1] = (uint8_t)((input >> 8) & 0xff);
    value[2] = (uint8_t)((input >> 16) & 0xff);
    value[3] = (uint8_t)((input >> 24) & 0xff);
}

bool edgez_lua_rpc_is_request(const uint8_t *payload, size_t payload_len)
{
    return payload && payload_len >= LUA_RPC_REQUEST_HEADER_LEN &&
           memcmp(payload, "LRPC", 4) == 0 && payload[4] == LUA_RPC_VERSION;
}

#ifdef CONFIG_EDGEZ_SENSOR_SCRIPTING
static esp_err_t rpc_connect(uint8_t interface_id, const uint8_t *body, size_t body_len)
{
    if (!body || body_len < 15) return ESP_ERR_INVALID_SIZE;
    const edgez_platform_interface_config_t config = {
        .address_or_baud = read_le32(body),
        .rx_size = (int32_t)read_le16(body + 4),
        .unit_id = read_le16(body + 6),
        .mode = body[8],
        .tx_pin = read_le_i16(body + 9),
        .rx_pin = read_le_i16(body + 11),
    };
    return edgez_platform_get()->interface_connect((edgez_platform_interface_kind_t)interface_id, &config);
}

static esp_err_t rpc_close(uint8_t interface_id)
{
    return edgez_platform_get()->interface_close((edgez_platform_interface_kind_t)interface_id);
}

static esp_err_t rpc_reset_cursor(uint8_t interface_id)
{
    return edgez_platform_get()->interface_reset_rx_cursor((edgez_platform_interface_kind_t)interface_id);
}

static esp_err_t rpc_set_rx_size(uint8_t interface_id, const uint8_t *body, size_t body_len)
{
    if (!body || body_len < 2) return ESP_ERR_INVALID_SIZE;
    int32_t size = (int32_t)read_le16(body);
    return edgez_platform_get()->interface_set_rx_size((edgez_platform_interface_kind_t)interface_id, size);
}

static esp_err_t rpc_write(uint8_t interface_id, const uint8_t *body, size_t body_len)
{
    return edgez_platform_get()->interface_write((edgez_platform_interface_kind_t)interface_id, body, body_len);
}

static esp_err_t rpc_read(uint8_t interface_id,
                          const uint8_t *body,
                          size_t body_len,
                          uint8_t *out,
                          size_t out_size,
                          size_t *out_len)
{
    if (!body || body_len < 2 || !out || !out_len) return ESP_ERR_INVALID_ARG;
    size_t requested = read_le16(body);
    if (requested > out_size) requested = out_size;
    return edgez_platform_get()->interface_read((edgez_platform_interface_kind_t)interface_id,
                                 out, requested, out_len);
}
#endif

void edgez_lua_rpc_handle(const uint8_t *payload,
                          size_t payload_len,
                          edgez_frame_send_fn send_fn,
                          void *send_ctx)
{
    if (!edgez_lua_rpc_is_request(payload, payload_len) || !send_fn) return;

    uint16_t request_id = read_le16(payload + 5);
    uint8_t operation = payload[7];
    uint8_t interface_id = payload[8];
    const uint8_t *body = payload + LUA_RPC_REQUEST_HEADER_LEN;
    size_t body_len = payload_len - LUA_RPC_REQUEST_HEADER_LEN;
    uint8_t response[EDGEZ_FRAME_MAX_PAYLOAD] = {'L', 'R', 'P', 'R', LUA_RPC_VERSION};
    write_le16(response + 5, request_id);
    response[7] = operation;
    size_t response_body_len = 0;
    esp_err_t err = ESP_ERR_NOT_SUPPORTED;

#ifdef CONFIG_EDGEZ_SENSOR_SCRIPTING
    switch (operation) {
    case LUA_RPC_OP_CONNECT:
        err = rpc_connect(interface_id, body, body_len);
        break;
    case LUA_RPC_OP_CLOSE:
        err = rpc_close(interface_id);
        break;
    case LUA_RPC_OP_RESET_RX_CURSOR:
        err = rpc_reset_cursor(interface_id);
        break;
    case LUA_RPC_OP_SET_RX_SIZE:
        err = rpc_set_rx_size(interface_id, body, body_len);
        break;
    case LUA_RPC_OP_WRITE:
        err = rpc_write(interface_id, body, body_len);
        break;
    case LUA_RPC_OP_READ:
        err = rpc_read(interface_id,
                       body,
                       body_len,
                       response + LUA_RPC_RESPONSE_HEADER_LEN,
                       sizeof(response) - LUA_RPC_RESPONSE_HEADER_LEN,
                       &response_body_len);
        break;
    default:
        err = ESP_ERR_NOT_SUPPORTED;
        break;
    }
#endif

    write_le32(response + 8, (uint32_t)err);
    ESP_LOGI(TAG,
             "BLE Lua RPC request=%u op=%u interface=%u request_len=%u response_len=%u err=%s",
             (unsigned)request_id,
             (unsigned)operation,
             (unsigned)interface_id,
             (unsigned)body_len,
             (unsigned)response_body_len,
             esp_err_to_name(err));
    send_fn(send_ctx, response, (uint16_t)(LUA_RPC_RESPONSE_HEADER_LEN + response_body_len));
}
