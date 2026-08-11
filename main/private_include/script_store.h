#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint16_t script_id;
    const uint8_t *name;
    size_t name_len;
    const uint8_t *script;
    size_t script_len;
    bool has_version;
    uint32_t version;
    bool has_global_buffer_size;
    uint32_t global_buffer_size;
    const uint8_t *mime_type;
    size_t mime_type_len;
} script_store_upsert_t;

esp_err_t script_store_upsert(const script_store_upsert_t *config);
esp_err_t script_store_delete(uint16_t script_id);
esp_err_t script_store_build_aggregate_for_ids(uint8_t *out_buf,
                                                size_t out_buf_size,
                                                const uint16_t *script_ids,
                                                size_t script_id_count,
                                                size_t *out_len);
