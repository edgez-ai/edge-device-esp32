#include "script_store.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nvs.h"
#include "nvs_flash.h"

#define STORE_PARTITION "driver_nvs"
#define STORE_NAMESPACE "lwm2m_script"
#define STORE_INDEX_KEY "index"
#define STORE_MAX_SCRIPTS 16
#define STORE_MAX_SCRIPT_SIZE (64 * 1024)

typedef struct {
    uint16_t id;
    uint16_t reserved;
    uint32_t size;
} store_meta_t;

static bool s_ready;
static bool s_loaded;
static store_meta_t s_meta[STORE_MAX_SCRIPTS];
static size_t s_count;

static void key_for(char prefix, uint16_t id, char key[16])
{
    snprintf(key, 16, "%c%05u", prefix, (unsigned)id);
}

static esp_err_t store_open(nvs_open_mode_t mode, nvs_handle_t *out)
{
    if (!s_ready) {
        esp_err_t err = nvs_flash_init_partition(STORE_PARTITION);
        if (err != ESP_OK) return err;
        s_ready = true;
    }
    return nvs_open_from_partition(STORE_PARTITION, STORE_NAMESPACE, mode, out);
}

static esp_err_t load_index(void)
{
    if (s_loaded) return ESP_OK;
    s_loaded = true;
    nvs_handle_t nvs;
    esp_err_t err = store_open(NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;
    if (err != ESP_OK) return err;
    size_t len = sizeof(s_meta);
    err = nvs_get_blob(nvs, STORE_INDEX_KEY, s_meta, &len);
    nvs_close(nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;
    if (err != ESP_OK || len % sizeof(store_meta_t) != 0) {
        s_count = 0;
        return err == ESP_OK ? ESP_ERR_INVALID_SIZE : err;
    }
    s_count = len / sizeof(store_meta_t);
    if (s_count > STORE_MAX_SCRIPTS) s_count = STORE_MAX_SCRIPTS;
    return ESP_OK;
}

static int find_index(uint16_t id)
{
    (void)load_index();
    for (size_t i = 0; i < s_count; i++) if (s_meta[i].id == id) return (int)i;
    return -1;
}

static esp_err_t save_index(nvs_handle_t nvs)
{
    return nvs_set_blob(nvs, STORE_INDEX_KEY, s_meta, s_count * sizeof(store_meta_t));
}

static esp_err_t set_optional_blob(nvs_handle_t nvs, char prefix, uint16_t id,
                                   const uint8_t *data, size_t len)
{
    if (!data || len == 0) return ESP_OK;
    char key[16]; key_for(prefix, id, key);
    return nvs_set_blob(nvs, key, data, len);
}

esp_err_t script_store_upsert(const script_store_upsert_t *config)
{
    if (!config || config->script_id == 0 || !config->script || config->script_len == 0 ||
        config->script_len > STORE_MAX_SCRIPT_SIZE) return ESP_ERR_INVALID_ARG;
    esp_err_t err = load_index();
    if (err != ESP_OK) return err;
    int idx = find_index(config->script_id);
    if (idx < 0) {
        if (s_count >= STORE_MAX_SCRIPTS) return ESP_ERR_NO_MEM;
        size_t insert_at = s_count;
        while (insert_at > 0 && s_meta[insert_at - 1].id > config->script_id) {
            s_meta[insert_at] = s_meta[insert_at - 1];
            insert_at--;
        }
        idx = (int)insert_at;
        s_count++;
        s_meta[idx].id = config->script_id;
    }
    s_meta[idx].size = config->script_len;

    nvs_handle_t nvs;
    err = store_open(NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    char key[16]; key_for('s', config->script_id, key);
    err = nvs_set_blob(nvs, key, config->script, config->script_len);
    if (err == ESP_OK) err = set_optional_blob(nvs, 'n', config->script_id, config->name, config->name_len);
    if (err == ESP_OK) err = set_optional_blob(nvs, 'm', config->script_id, config->mime_type, config->mime_type_len);
    if (err == ESP_OK && config->has_version) { key_for('v', config->script_id, key); err = nvs_set_u32(nvs, key, config->version); }
    if (err == ESP_OK && config->has_global_buffer_size) { key_for('b', config->script_id, key); err = nvs_set_u32(nvs, key, config->global_buffer_size); }
    if (err == ESP_OK) err = save_index(nvs);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

esp_err_t script_store_delete(uint16_t id)
{
    int idx = find_index(id);
    if (idx < 0) return ESP_ERR_NOT_FOUND;
    nvs_handle_t nvs;
    esp_err_t err = store_open(NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    const char prefixes[] = {'s','n','m','v','b'};
    for (size_t i = 0; i < sizeof(prefixes); i++) { char key[16]; key_for(prefixes[i], id, key); (void)nvs_erase_key(nvs, key); }
    for (size_t i = (size_t)idx; i + 1 < s_count; i++) s_meta[i] = s_meta[i + 1];
    s_count--;
    err = save_index(nvs);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

static bool selected(uint16_t id, const uint16_t *ids, size_t count)
{
    if (!ids) return true;
    for (size_t i = 0; i < count; i++) if (ids[i] == id) return true;
    return false;
}

static size_t trim_trailing_space(const uint8_t *s, size_t len)
{
    while (len > 0 && isspace((unsigned char)s[len - 1])) len--;
    return len;
}

static size_t strip_return_result(const uint8_t *script, size_t len)
{
    static const char suffix[] = "return result";
    size_t end = trim_trailing_space(script, len);
    size_t start = end;
    while (start > 0 && script[start - 1] != '\n' && script[start - 1] != '\r') start--;
    size_t begin = start;
    while (begin < end && (script[begin] == ' ' || script[begin] == '\t')) begin++;
    while (end > begin && (script[end - 1] == ' ' || script[end - 1] == '\t')) end--;
    return (end - begin == sizeof(suffix) - 1 && memcmp(script + begin, suffix, sizeof(suffix) - 1) == 0) ? start : len;
}

static size_t skip_local_result(const uint8_t *script, size_t len)
{
    static const char prefix[] = "local result = {}";
    size_t start = 0;
    while (start < len && isspace((unsigned char)script[start])) start++;
    size_t end = start;
    while (end < len && script[end] != '\n' && script[end] != '\r') end++;
    size_t check_end = end;
    while (check_end > start && (script[check_end - 1] == ' ' || script[check_end - 1] == '\t')) check_end--;
    if (check_end - start != sizeof(prefix) - 1 || memcmp(script + start, prefix, sizeof(prefix) - 1) != 0) return 0;
    if (end < len && script[end] == '\r') end++;
    if (end < len && script[end] == '\n') end++;
    return end;
}

static esp_err_t aggregate(uint8_t *out, size_t cap, const uint16_t *ids, size_t id_count, size_t *out_len)
{
    if (!out || cap == 0 || !out_len) return ESP_ERR_INVALID_ARG;
    *out_len = 0;
    esp_err_t err = load_index();
    if (err != ESP_OK) return err;
    nvs_handle_t nvs;
    err = store_open(NVS_READONLY, &nvs);
    if (err != ESP_OK) return err;
    size_t used = 0;
    size_t selected_count = 0;
    for (size_t i = 0; i < s_count; i++) if (selected(s_meta[i].id, ids, id_count) && s_meta[i].size > 0) selected_count++;
    size_t selected_index = 0;
    for (size_t i = 0; i < s_count; i++) {
        if (!selected(s_meta[i].id, ids, id_count) || s_meta[i].size == 0) continue;
        char key[16]; key_for('s', s_meta[i].id, key);
        size_t len = s_meta[i].size;
        uint8_t *script = malloc(len + 1);
        if (!script) { err = ESP_ERR_NO_MEM; break; }
        err = nvs_get_blob(nvs, key, script, &len);
        if (err != ESP_OK) { free(script); break; }
        script[len] = 0;
        size_t start = selected_index > 0 ? skip_local_result(script, len) : 0;
        size_t segment_len = len - start;
        if (selected_index + 1 < selected_count) segment_len = strip_return_result(script + start, segment_len);
        if (used && used < cap) out[used++] = '\n';
        if (segment_len > cap - used) { free(script); err = ESP_ERR_NO_MEM; break; }
        memcpy(out + used, script + start, segment_len);
        used += segment_len;
        selected_index++;
        free(script);
    }
    nvs_close(nvs);
    if (err == ESP_OK && used < cap) out[used] = 0;
    *out_len = used;
    return err;
}

esp_err_t script_store_build_aggregate_for_ids(uint8_t *out, size_t cap, const uint16_t *ids, size_t count, size_t *len) { return aggregate(out, cap, ids, count, len); }
