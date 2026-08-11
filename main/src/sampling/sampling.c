#include "sampling.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "nvs.h"
#include "local_interface.h"
#include "script_store.h"
#include "interface_bridge.h"
#include "script_interface_runtime.h"
#include "ble_control.h"
#include "halow_sync_bridge.h"
#include "prov.h"
#ifdef CONFIG_ENABLE_MM_HALOW
#include "halow_interface_app.h"
#endif
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"

#define SAMPLE_NVS_NAMESPACE "sample"
#define SAMPLE_NVS_KEY_VER "ver"
#define SAMPLE_NVS_KEY_EN "en"
#define SAMPLE_NVS_KEY_SLEEP "sleep"
#define SAMPLE_NVS_KEY_SLEEP_MODE "sleep_mode"
#define SAMPLE_NVS_KEY_S_RATE "s_rate"
#define SAMPLE_NVS_KEY_R_RATE "r_rate"
#define SAMPLE_NVS_KEY_SEND_ACK_TO "send_ack_to"
#define SAMPLE_NVS_KEY_SEND_RETRY_DELAY "send_retry_delay"
#define SAMPLE_NVS_KEY_SEND_RETRY_CNT "send_retry_cnt"
#define SAMPLE_NVS_KEY_S_U2C "s_u2c"
#define SAMPLE_NVS_KEY_S_RS "s_rs485"
#define SAMPLE_DATA_FIELD_MAX_LINES 30
#define SAMPLE_DATA_FIELD_LINE_MAX_LEN 350000

static bool sample_get_server_sec_of_year(uint32_t *out)
{
    if (!out) return false;
    time_t now = time(NULL);
    if (now < 1577836800) return false;
    struct tm local = {0};
    localtime_r(&now, &local);
    *out = (uint32_t)(local.tm_yday * 86400 + local.tm_hour * 3600 + local.tm_min * 60 + local.tm_sec);
    return true;
}


#ifndef CONFIG_SAMPLE_ENABLE_LUA_SCRIPT_DUMP
#define CONFIG_SAMPLE_ENABLE_LUA_SCRIPT_DUMP 0
#endif

#ifndef CONFIG_SAMPLE_SLEEP_IDLE_WAIT_STEP_MS
#define CONFIG_SAMPLE_SLEEP_IDLE_WAIT_STEP_MS 100
#endif

#ifndef CONFIG_SAMPLE_STARTUP_NET_GUARD_TIMEOUT_MS
#define CONFIG_SAMPLE_STARTUP_NET_GUARD_TIMEOUT_MS 15000
#endif

#ifndef CONFIG_SAMPLE_POST_REPORT_SLEEP_HOLDOFF_MS
#define CONFIG_SAMPLE_POST_REPORT_SLEEP_HOLDOFF_MS 3000
#endif

#ifndef CONFIG_SAMPLE_POST_REPORT_SLEEP_MIN_HOLDOFF_MS
#define CONFIG_SAMPLE_POST_REPORT_SLEEP_MIN_HOLDOFF_MS 1000
#endif

#ifndef CONFIG_SAMPLE_HALOW_SLEEP_READY_TIMEOUT_MS
#define CONFIG_SAMPLE_HALOW_SLEEP_READY_TIMEOUT_MS 6000
#endif

#ifndef CONFIG_SAMPLE_WAKE_BUTTON_GPIO
#define CONFIG_SAMPLE_WAKE_BUTTON_GPIO GPIO_NUM_5
#endif

#ifndef CONFIG_SAMPLE_SEND_ACK_TIMEOUT_MS
#define CONFIG_SAMPLE_SEND_ACK_TIMEOUT_MS 12000
#endif

#ifndef CONFIG_SAMPLE_SEND_RETRY_DELAY_MS
#define CONFIG_SAMPLE_SEND_RETRY_DELAY_MS 1000
#endif

#ifndef CONFIG_SAMPLE_SEND_RETRY_MAX_DELAY_MS
#define CONFIG_SAMPLE_SEND_RETRY_MAX_DELAY_MS 15000
#endif

#ifndef CONFIG_SAMPLE_TASK_STACK_SIZE
#define CONFIG_SAMPLE_TASK_STACK_SIZE 16384
#endif

#ifndef CONFIG_SAMPLE_STARTUP_LAUNCHER_STACK_SIZE
#define CONFIG_SAMPLE_STARTUP_LAUNCHER_STACK_SIZE 3072
#endif

#ifndef CONFIG_SAMPLE_TASK_STACK_WATERMARK_WARN_WORDS
#define CONFIG_SAMPLE_TASK_STACK_WATERMARK_WARN_WORDS 512
#endif

#ifndef CONFIG_SAMPLE_REPORT_FAIL_COOLDOWN_BASE_MS
#define CONFIG_SAMPLE_REPORT_FAIL_COOLDOWN_BASE_MS 5000
#endif

#ifndef CONFIG_SAMPLE_REPORT_FAIL_COOLDOWN_MAX_MS
#define CONFIG_SAMPLE_REPORT_FAIL_COOLDOWN_MAX_MS 120000
#endif

#define SAMPLE_RING_MAX_RECORDS 30
#define SAMPLE_RING_RECORD_MAX_LEN (256 * 1024)
#define SAMPLE_SEND_MAX_ATTEMPTS 3

#define SAMPLE_SCRIPT_GLOBAL_BUFFER_DEFAULT_SIZE (16 * 1024)
#define SAMPLE_SCRIPT_GLOBAL_BUFFER_MAX_SIZE     (16 * 1024)
#define SAMPLE_SCRIPT_CACHE_DEFAULT_SIZE          (16 * 1024)
#define SAMPLE_SCRIPT_CACHE_MAX_SIZE              SAMPLE_SCRIPT_GLOBAL_BUFFER_MAX_SIZE

#ifndef CONFIG_SAMPLE_DISABLE_CLIENT_REPORT_SEND
#define CONFIG_SAMPLE_DISABLE_CLIENT_REPORT_SEND 0
#endif

static const char *TAG = "sampling";
/* Camera bytes and the serving snapshot are CPU-only storage. Keep both in
 * PSRAM; UART/SPI DMA buffers remain owned by their drivers in internal RAM. */
static uint8_t *s_script_global_buffer = NULL;
static size_t s_script_global_buffer_capacity = 0;
static size_t s_script_global_buffer_length = 0;
static uint8_t *s_script_serving_buffer = NULL;
static size_t s_script_serving_buffer_capacity = 0;
static size_t s_script_serving_buffer_length = 0;
static SemaphoreHandle_t s_script_global_buffer_mutex = NULL;
static portMUX_TYPE s_script_global_buffer_mutex_lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_script_serving_buffer_mutex = NULL;
static portMUX_TYPE s_script_serving_buffer_mutex_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_script_execute_input_logged_once = false;
static uint32_t s_script_error_print_budget = 3;
static char *s_lua_result_line_buf = NULL;
static size_t s_lua_result_line_buf_capacity = 0;
static char *s_lua_result_escaped_buf = NULL;
static size_t s_lua_result_escaped_buf_capacity = 0;
static bool s_startup_network_guard_enabled = false;
static bool s_startup_network_guard_logged = false;
static uint64_t s_startup_network_guard_start_us = 0;
static volatile bool s_sample_in_progress = false;
static uint64_t s_sampling_gate_last_log_us = 0;
static uint32_t s_sample_task_stack_warn_budget = 8;
static uint8_t s_report_send_fail_streak = 0;
static uint64_t s_report_send_cooldown_until_us = 0;
static uint64_t s_report_send_cooldown_last_log_us = 0;
static bool s_beacon_report_cache_skip_logged = false;
static portMUX_TYPE s_sample_ring_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_latest_sensor_lock = portMUX_INITIALIZER_UNLOCKED;
static sample_sensor_data_t s_latest_sensor_data = {0};
static sample_sensor_data_t s_pending_sensor_data = {0};
static bool s_sensor_update_active = false;
static int64_t s_last_sample_finish_uptime_ms = -1;
static uint32_t s_sample_script_run_counter = 0;

typedef enum {
    SAMPLE_SENSOR_FIELD_NONE = 0,
    SAMPLE_SENSOR_FIELD_LATITUDE,
    SAMPLE_SENSOR_FIELD_LONGITUDE,
    SAMPLE_SENSOR_FIELD_ALTITUDE,
    SAMPLE_SENSOR_FIELD_TEMPERATURE,
    SAMPLE_SENSOR_FIELD_HUMIDITY,
    SAMPLE_SENSOR_FIELD_PRESSURE,
    SAMPLE_SENSOR_FIELD_VIBRATION_AVERAGE,
} sample_sensor_field_t;

static void sample_begin_sensor_data_update(void)
{
    portENTER_CRITICAL(&s_latest_sensor_lock);
    memset(&s_pending_sensor_data, 0, sizeof(s_pending_sensor_data));
    s_sensor_update_active = true;
    portEXIT_CRITICAL(&s_latest_sensor_lock);
}

static void sample_finish_sensor_data_update(bool success)
{
    portENTER_CRITICAL(&s_latest_sensor_lock);
    if (s_sensor_update_active && success) {
        s_latest_sensor_data = s_pending_sensor_data;
    }
    s_sensor_update_active = false;
    portEXIT_CRITICAL(&s_latest_sensor_lock);
}

static sample_sensor_field_t sample_sensor_field_from_selector(int obj, int res)
{
    switch (obj) {
    case 6: /* OMA Location */
        if (res == 0) return SAMPLE_SENSOR_FIELD_LATITUDE;
        if (res == 1) return SAMPLE_SENSOR_FIELD_LONGITUDE;
        if (res == 2) return SAMPLE_SENSOR_FIELD_ALTITUDE;
        break;
    case 3303: /* IPSO Temperature */
        if (res == 5700) return SAMPLE_SENSOR_FIELD_TEMPERATURE;
        break;
    case 3304: /* IPSO Humidity */
        if (res == 5700) return SAMPLE_SENSOR_FIELD_HUMIDITY;
        break;
    case 3323: /* IPSO Pressure */
        if (res == 5700) return SAMPLE_SENSOR_FIELD_PRESSURE;
        break;
    case 3300: /* Generic sensor */
        if (res == 10) return SAMPLE_SENSOR_FIELD_VIBRATION_AVERAGE;
        break;
    default:
        break;
    }

    /* Compatibility with scripts that return SensorData protobuf field IDs. */
    switch (res) {
    case 4:
        return SAMPLE_SENSOR_FIELD_LATITUDE;
    case 5:
        return SAMPLE_SENSOR_FIELD_LONGITUDE;
    case 6:
        return SAMPLE_SENSOR_FIELD_ALTITUDE;
    case 7:
        return SAMPLE_SENSOR_FIELD_TEMPERATURE;
    case 8:
        return SAMPLE_SENSOR_FIELD_HUMIDITY;
    case 9:
        return SAMPLE_SENSOR_FIELD_PRESSURE;
    case 10:
        return SAMPLE_SENSOR_FIELD_VIBRATION_AVERAGE;
    default:
        return SAMPLE_SENSOR_FIELD_NONE;
    }
}

static void sample_update_latest_sensor_value(int obj, int res, double value)
{
    bool updated = false;
    sample_sensor_field_t field = sample_sensor_field_from_selector(obj, res);

    portENTER_CRITICAL(&s_latest_sensor_lock);
    sample_sensor_data_t *sensor_data = s_sensor_update_active
                                            ? &s_pending_sensor_data
                                            : &s_latest_sensor_data;
    if (field == SAMPLE_SENSOR_FIELD_TEMPERATURE) {
        sensor_data->temperature = (float)value;
        sensor_data->has_temperature = true;
        updated = true;
    } else if (field == SAMPLE_SENSOR_FIELD_HUMIDITY) {
        sensor_data->humidity = (float)value;
        sensor_data->has_humidity = true;
        updated = true;
    } else if (field == SAMPLE_SENSOR_FIELD_PRESSURE) {
        sensor_data->pressure = (float)value;
        sensor_data->has_pressure = true;
        updated = true;
    } else if (field == SAMPLE_SENSOR_FIELD_LATITUDE) {
        sensor_data->latitude = (float)value;
        sensor_data->has_latitude = true;
        updated = true;
    } else if (field == SAMPLE_SENSOR_FIELD_LONGITUDE) {
        sensor_data->longitude = (float)value;
        sensor_data->has_longitude = true;
        updated = true;
    } else if (field == SAMPLE_SENSOR_FIELD_ALTITUDE) {
        sensor_data->altitude = (float)value;
        sensor_data->has_altitude = true;
        updated = true;
    } else if (field == SAMPLE_SENSOR_FIELD_VIBRATION_AVERAGE) {
        sensor_data->vibration_average = (float)value;
        sensor_data->has_vibration_average = true;
        updated = true;
    }
    portEXIT_CRITICAL(&s_latest_sensor_lock);

    if (updated) {
        ESP_LOGD(TAG,
                 "Latest sensor data updated: obj=%d res=%d value=%f",
                 obj,
                 res,
                 value);
    }
}

static void sample_update_latest_sensor_type(int type, double value)
{
    portENTER_CRITICAL(&s_latest_sensor_lock);
    sample_sensor_data_t *sensor_data = s_sensor_update_active
                                            ? &s_pending_sensor_data
                                            : &s_latest_sensor_data;
    switch (type) {
    case 1: /* SENSOR_TEMPERATURE */
        sensor_data->temperature = (float)value;
        sensor_data->has_temperature = true;
        break;
    case 2: /* SENSOR_HUMIDITY */
        sensor_data->humidity = (float)value;
        sensor_data->has_humidity = true;
        break;
    case 3: /* SENSOR_LATITUDE */
        sensor_data->latitude = (float)value;
        sensor_data->has_latitude = true;
        break;
    case 4: /* SENSOR_LONGITUDE */
        sensor_data->longitude = (float)value;
        sensor_data->has_longitude = true;
        break;
    case 5: /* SENSOR_LENGTH */
        sensor_data->length = (int32_t)value;
        sensor_data->has_length = true;
        break;
    case 6: /* SENSOR_ACCEL_X */
        sensor_data->accel_x = (float)value;
        sensor_data->has_accel_x = true;
        break;
    case 7: /* SENSOR_ACCEL_Y */
        sensor_data->accel_y = (float)value;
        sensor_data->has_accel_y = true;
        break;
    case 8: /* SENSOR_ACCEL_Z */
        sensor_data->accel_z = (float)value;
        sensor_data->has_accel_z = true;
        break;
    case 9: /* SENSOR_GYRO_X */
        sensor_data->gyro_x = (float)value;
        sensor_data->has_gyro_x = true;
        break;
    case 10: /* SENSOR_GYRO_Y */
        sensor_data->gyro_y = (float)value;
        sensor_data->has_gyro_y = true;
        break;
    case 11: /* SENSOR_GYRO_Z */
        sensor_data->gyro_z = (float)value;
        sensor_data->has_gyro_z = true;
        break;
    default:
        break;
    }
    portEXIT_CRITICAL(&s_latest_sensor_lock);
}

bool sample_get_latest_sensor_data(sample_sensor_data_t *out)
{
    if (!out) {
        return false;
    }

    portENTER_CRITICAL(&s_latest_sensor_lock);
    *out = s_latest_sensor_data;
    bool has_any = out->has_latitude || out->has_longitude || out->has_altitude ||
                   out->has_temperature || out->has_humidity || out->has_pressure ||
                   out->has_vibration_average || out->has_length ||
                   out->has_accel_x || out->has_accel_y || out->has_accel_z ||
                   out->has_gyro_x || out->has_gyro_y || out->has_gyro_z;
    portEXIT_CRITICAL(&s_latest_sensor_lock);
    return has_any;
}

#ifndef CONFIG_SAMPLE_UART_DEFAULT_BAUD_RATE
#define CONFIG_SAMPLE_UART_DEFAULT_BAUD_RATE 115200
#endif

#ifndef CONFIG_SAMPLE_UART_DEFAULT_TX_PIN
#define CONFIG_SAMPLE_UART_DEFAULT_TX_PIN 19
#endif

#ifndef CONFIG_SAMPLE_UART_DEFAULT_RX_PIN
#define CONFIG_SAMPLE_UART_DEFAULT_RX_PIN 20
#endif

#ifndef CONFIG_SAMPLE_UART_DEFAULT_RX_BUFFER_SIZE
#define CONFIG_SAMPLE_UART_DEFAULT_RX_BUFFER_SIZE 4096
#endif

static bool work_in_progress(void);
static void build_default_sample_payload(char *out, size_t out_size, const char *status);
static void sample_ring_push_payload(const char *payload, size_t len);
static esp_err_t sample_script_cache_set_capacity(size_t capacity);
static esp_err_t sample_script_cache_ensure_capacity(size_t min_capacity);
static esp_err_t sample_ring_ensure_slot_storage(uint8_t slot_index);

static uint32_t sample_retry_delay_with_jitter_ms(uint32_t base_delay_ms, uint32_t attempt_index)
{
    if (base_delay_ms == 0) {
        return 0;
    }

    uint32_t delay = base_delay_ms;
    for (uint32_t i = 1; i < attempt_index; i++) {
        if (delay >= CONFIG_SAMPLE_SEND_RETRY_MAX_DELAY_MS) {
            delay = CONFIG_SAMPLE_SEND_RETRY_MAX_DELAY_MS;
            break;
        }

        uint32_t next = delay * 2;
        if (next < delay || next > CONFIG_SAMPLE_SEND_RETRY_MAX_DELAY_MS) {
            next = CONFIG_SAMPLE_SEND_RETRY_MAX_DELAY_MS;
        }
        delay = next;
    }

    uint32_t jitter_span = delay / 5; /* +/-20% */
    if (jitter_span == 0) {
        return delay;
    }

    uint32_t random_offset = esp_random() % (2 * jitter_span + 1);
    uint32_t jittered = (delay - jitter_span) + random_offset;
    return jittered;
}

static uint32_t sample_retry_ack_timeout_ms(uint32_t base_timeout_ms, uint32_t attempt_index)
{
    uint32_t timeout = base_timeout_ms > 0 ? base_timeout_ms : CONFIG_SAMPLE_SEND_ACK_TIMEOUT_MS;
    if (attempt_index <= 1) {
        return timeout;
    }

    uint32_t extra = (timeout / 2) * (attempt_index - 1);
    uint32_t scaled = timeout + extra;
    if (scaled < timeout) {
        scaled = timeout;
    }

    if (scaled > 60000) {
        scaled = 60000;
    }
    return scaled;
}

static bool sample_report_send_cooldown_active(uint64_t now_us, uint32_t *remaining_ms)
{
    if (s_report_send_cooldown_until_us == 0 || now_us >= s_report_send_cooldown_until_us) {
        if (remaining_ms) {
            *remaining_ms = 0;
        }
        return false;
    }

    uint64_t remaining_us = s_report_send_cooldown_until_us - now_us;
    uint32_t ms = (uint32_t)((remaining_us + 999ULL) / 1000ULL);
    if (remaining_ms) {
        *remaining_ms = ms;
    }
    return true;
}

static void sample_report_send_mark_success(void)
{
    s_report_send_fail_streak = 0;
    s_report_send_cooldown_until_us = 0;
}

static void sample_report_send_mark_failure(void)
{
    if (s_report_send_fail_streak < 10) {
        s_report_send_fail_streak++;
    }

    uint32_t cooldown_ms = CONFIG_SAMPLE_REPORT_FAIL_COOLDOWN_BASE_MS;
    uint8_t exp_steps = (s_report_send_fail_streak > 0) ? (uint8_t)(s_report_send_fail_streak - 1) : 0;
    if (exp_steps > 6) {
        exp_steps = 6;
    }

    for (uint8_t i = 0; i < exp_steps; i++) {
        if (cooldown_ms >= CONFIG_SAMPLE_REPORT_FAIL_COOLDOWN_MAX_MS) {
            cooldown_ms = CONFIG_SAMPLE_REPORT_FAIL_COOLDOWN_MAX_MS;
            break;
        }
        uint32_t next = cooldown_ms * 2;
        if (next < cooldown_ms || next > CONFIG_SAMPLE_REPORT_FAIL_COOLDOWN_MAX_MS) {
            next = CONFIG_SAMPLE_REPORT_FAIL_COOLDOWN_MAX_MS;
        }
        cooldown_ms = next;
    }

    uint64_t now_us = (uint64_t)esp_timer_get_time();
    s_report_send_cooldown_until_us = now_us + ((uint64_t)cooldown_ms * 1000ULL);
    s_report_send_cooldown_last_log_us = 0;

    printf("[sampling] Report send failure streak=%u, applying cooldown=%lu ms\n",
           (unsigned)s_report_send_fail_streak,
           (unsigned long)cooldown_ms);
}

static bool sample_selector_id_exists(const uint16_t *ids, size_t count, uint16_t id)
{
    if (!ids || id == 0) {
        return false;
    }

    for (size_t i = 0; i < count; i++) {
        if (ids[i] == id) {
            return true;
        }
    }
    return false;
}

static bool sample_parse_script_selector_id(const char *selector, uint16_t *out_id)
{
    if (!selector || !out_id) {
        return false;
    }

    if (strncmp(selector, "id-version:", 11) == 0) {
        selector += 11;
    } else if (strncmp(selector, "id:", 3) == 0) {
        selector += 3;
    } else if (strncmp(selector, "script:", 7) == 0) {
        selector += 7;
    } else if (strncmp(selector, "name:", 5) == 0) {
        /* Explicitly reject name selectors so matching stays id-version based. */
        return false;
    }

    while (*selector != '\0' && isspace((unsigned char)*selector)) {
        selector++;
    }
    if (*selector == '\0') {
        return false;
    }

    char *end = NULL;
    unsigned long parsed_id = strtoul(selector, &end, 10);
    if (end == selector || parsed_id == 0 || parsed_id > UINT16_MAX) {
        return false;
    }

    if (*end == '\0') {
        *out_id = (uint16_t)parsed_id;
        return true;
    }

    if (*end == '-' || *end == ':') {
        char *version_end = NULL;
        unsigned long parsed_version = strtoul(end + 1, &version_end, 10);
        if (version_end == end + 1 || *version_end != '\0' || parsed_version == 0) {
            return false;
        }

        *out_id = (uint16_t)parsed_id;
        return true;
    }

    return false;
}

static size_t sample_trim_selector_token(const char *src, size_t src_len, char *dst, size_t dst_size)
{
    if (!src || !dst || dst_size == 0) {
        return 0;
    }

    size_t begin = 0;
    size_t end = src_len;

    while (begin < end && isspace((unsigned char)src[begin])) {
        begin++;
    }
    while (end > begin && isspace((unsigned char)src[end - 1])) {
        end--;
    }

    while (begin < end && (src[begin] == '"' || src[begin] == '\'' || src[begin] == '[' || src[begin] == '{' || src[begin] == '(')) {
        begin++;
    }
    while (end > begin && (src[end - 1] == '"' || src[end - 1] == '\'' || src[end - 1] == ']' || src[end - 1] == '}' || src[end - 1] == ')')) {
        end--;
    }

    size_t len = end > begin ? (end - begin) : 0;
    if (len >= dst_size) {
        len = dst_size - 1;
    }

    if (len > 0) {
        memcpy(dst, src + begin, len);
    }
    dst[len] = '\0';
    return len;
}

static bool sample_is_selector_separator(char c)
{
    return c == ',' || c == ';' || c == '|' || isspace((unsigned char)c);
}

static size_t sample_collect_script_ids_from_selector_text(const char *text,
                                                           uint16_t *ids,
                                                           size_t max_ids,
                                                           size_t offset,
                                                           const char *source_label)
{
    if (!text || !ids || max_ids == 0 || offset >= max_ids) {
        return offset;
    }

    size_t count = offset;
    const char *cursor = text;

    while (*cursor != '\0' && count < max_ids) {
        while (*cursor != '\0' && sample_is_selector_separator(*cursor)) {
            cursor++;
        }
        if (*cursor == '\0') {
            break;
        }

        const char *token_start = cursor;
        while (*cursor != '\0' && !sample_is_selector_separator(*cursor)) {
            cursor++;
        }

        size_t token_len = (size_t)(cursor - token_start);
        if (token_len == 0) {
            continue;
        }

        char token_raw[65] = {0};
        if (token_len >= sizeof(token_raw)) {
            token_len = sizeof(token_raw) - 1;
        }
        memcpy(token_raw, token_start, token_len);
        token_raw[token_len] = '\0';

        char token[65] = {0};
        size_t normalized_len = sample_trim_selector_token(token_raw, token_len, token, sizeof(token));
        if (normalized_len == 0) {
            continue;
        }

        uint16_t parsed_id = 0;
        if (sample_parse_script_selector_id(token, &parsed_id)) {
            ESP_LOGI(TAG,
                     "Script selector match (%s): '%s' -> id=%u (id-version)",
                     source_label ? source_label : "unknown",
                     token,
                     (unsigned)parsed_id);
        } else {
            ESP_LOGW(TAG,
                     "Script selector not matched (%s): '%s'",
                     source_label ? source_label : "unknown",
                     token);
            continue;
        }

        if (!sample_selector_id_exists(ids, count, parsed_id)) {
            ids[count++] = parsed_id;
        }
    }

    return count;
}

void sample_refresh_script_cache_from_selectors(void)
{
    size_t script_len = 0;
    uint16_t selected_script_ids[8] = {0};
    size_t selected_count = 0;

    selected_count = sample_collect_script_ids_from_selector_text(g_sample_sensor_uart_i2c,
                                                                  selected_script_ids,
                                                                  sizeof(selected_script_ids) / sizeof(selected_script_ids[0]),
                                                                  selected_count,
                                                                  "uart_i2c");
    selected_count = sample_collect_script_ids_from_selector_text(g_sample_sensor_rs485,
                                                                  selected_script_ids,
                                                                  sizeof(selected_script_ids) / sizeof(selected_script_ids[0]),
                                                                  selected_count,
                                                                  "rs485");

    if (selected_count > 0) {
        char id_list[192] = {0};
        size_t written = 0;
        for (size_t i = 0; i < selected_count && written + 16 < sizeof(id_list); i++) {
            int wrote = snprintf(id_list + written,
                                sizeof(id_list) - written,
                                "%s%u",
                                (i == 0) ? "" : ",",
                                (unsigned)selected_script_ids[i]);
            if (wrote <= 0) {
                break;
            }
            written += (size_t)wrote;
        }
        ESP_LOGI(TAG, "Script selectors resolved to ids=[%s]", id_list);
    } else {
        ESP_LOGI(TAG, "No valid script selectors resolved from sensor config");
    }

    esp_err_t err = ESP_ERR_NOT_FOUND;

    esp_err_t cache_err = sample_script_cache_ensure_capacity(SAMPLE_SCRIPT_CACHE_DEFAULT_SIZE);
    if (cache_err != ESP_OK) {
        g_sample_script_len = 0;
        g_sample_script_truncated = true;
        ESP_LOGW(TAG,
                 "Failed to allocate script cache buffer: %s",
                 esp_err_to_name(cache_err));
        return;
    }

    if (selected_count > 0) {
        while (true) {
            err = script_store_build_aggregate_for_ids(g_sample_script_buf,
                                                       g_sample_script_buf_capacity,
                                                       selected_script_ids,
                                                       selected_count,
                                                       &script_len);
            if (err != ESP_ERR_NO_MEM) {
                break;
            }

            if (g_sample_script_buf_capacity >= SAMPLE_SCRIPT_CACHE_MAX_SIZE) {
                break;
            }

            size_t next_capacity = g_sample_script_buf_capacity * 2;
            if (next_capacity < SAMPLE_SCRIPT_CACHE_DEFAULT_SIZE) {
                next_capacity = SAMPLE_SCRIPT_CACHE_DEFAULT_SIZE;
            }
            if (next_capacity > SAMPLE_SCRIPT_CACHE_MAX_SIZE) {
                next_capacity = SAMPLE_SCRIPT_CACHE_MAX_SIZE;
            }

            cache_err = sample_script_cache_set_capacity(next_capacity);
            if (cache_err != ESP_OK) {
                ESP_LOGW(TAG,
                         "Failed to grow script cache buffer to %u bytes: %s",
                         (unsigned)next_capacity,
                         esp_err_to_name(cache_err));
                err = ESP_ERR_NO_MEM;
                break;
            }

            ESP_LOGI(TAG,
                     "Retrying aggregate script build with larger cache: capacity=%u",
                     (unsigned)g_sample_script_buf_capacity);
        }
    }

    if (err == ESP_OK) {
        g_sample_script_len = (uint32_t)script_len;
        g_sample_script_truncated = false;
        return;
    }

    g_sample_script_len = 0;
    if (g_sample_script_buf && g_sample_script_buf_capacity > 0) {
        memset(g_sample_script_buf, 0, g_sample_script_buf_capacity);
    }

    if (err == ESP_ERR_NOT_FOUND) {
        g_sample_script_truncated = false;
        return;
    }

    g_sample_script_truncated = (err == ESP_ERR_NO_MEM);
    ESP_LOGW(TAG, "Failed to build script from script object: %s", esp_err_to_name(err));
}

static void print_lua_script_dump(const char *reason)
{
#if !CONFIG_SAMPLE_ENABLE_LUA_SCRIPT_DUMP
    (void)reason;
    return;
#endif

    if (g_sample_script_len == 0 || !g_sample_script_buf) {
        printf("[sampling] Lua script dump (%s): <empty>\n", reason ? reason : "unknown");
        return;
    }

    printf("[sampling] Lua script dump (%s), %lu bytes:\n",
           reason ? reason : "unknown",
           (unsigned long)g_sample_script_len);

    const char *script = (const char *)g_sample_script_buf;
    size_t start = 0;
    int line = 1;

    for (size_t i = 0; i < g_sample_script_len; i++) {
        if (script[i] == '\n') {
            printf("[sampling] %04d | %.*s\n", line, (int)(i - start), script + start);
            line++;
            start = i + 1;
        }
    }

    if (start < g_sample_script_len) {
        printf("[sampling] %04d | %.*s\n", line, (int)(g_sample_script_len - start), script + start);
    }
}

static void print_lua_script_dump_from_buffer(const uint8_t *script_buf, size_t script_len, const char *reason)
{
#if !CONFIG_SAMPLE_ENABLE_LUA_SCRIPT_DUMP
    (void)script_buf;
    (void)script_len;
    (void)reason;
    return;
#endif

    if (!script_buf || script_len == 0) {
        printf("[sampling] Lua script input dump (%s): <empty>\n", reason ? reason : "unknown");
        return;
    }

    printf("[sampling] Lua script input dump (%s), %lu bytes:\n",
           reason ? reason : "unknown",
           (unsigned long)script_len);

    const char *script = (const char *)script_buf;
    size_t start = 0;
    int line = 1;

    for (size_t i = 0; i < script_len; i++) {
        if (script[i] == '\n') {
            printf("[sampling] IN %04d | %.*s\n", line, (int)(i - start), script + start);
            line++;
            start = i + 1;
        }
    }

    if (start < script_len) {
        printf("[sampling] IN %04d | %.*s\n", line, (int)(script_len - start), script + start);
    }
}

static int lua_uart_read_frame(lua_State *lua_state)
{
    if (!script_interface_is_open(SCRIPT_INTERFACE_UART)) {
        lua_pushliteral(lua_state, "");
        return 1;
    }

    uint8_t buf[256];
    size_t got = 0;
    esp_err_t err = script_interface_read(SCRIPT_INTERFACE_UART, buf, sizeof(buf), &got);
    if (err != ESP_OK || got == 0) {
        lua_pushliteral(lua_state, "");
        return 1;
    }

    lua_pushlstring(lua_state, (const char *)buf, got);
    return 1;
}

static int lua_parse_pm25(lua_State *lua_state)
{
    size_t frame_len = 0;
    const char *frame = luaL_optlstring(lua_state, 1, "", &frame_len);

    double pm1 = 0;
    double pm25 = 0;
    double pm10 = 0;
    int parsed_fields = 0;

    if (frame_len > 0) {
        parsed_fields = sscanf(frame, "%lf,%lf,%lf", &pm1, &pm25, &pm10);
    }

    lua_newtable(lua_state);

    lua_pushnumber(lua_state, pm1);
    lua_setfield(lua_state, -2, "pm1_0");

    lua_pushnumber(lua_state, pm25);
    lua_setfield(lua_state, -2, "pm2_5");

    lua_pushnumber(lua_state, pm10);
    lua_setfield(lua_state, -2, "pm10");

    lua_pushboolean(lua_state, parsed_fields == 3);
    lua_setfield(lua_state, -2, "ok");

    return 1;
}

/* ============================================================
 * Lua global functions injected for flow.lua (Modbus RS485)
 * These mirror the globals set by util.lua and rs485_interface.lua
 * in the SDK, allowing the same flow.lua to run natively on device.
 * ============================================================ */


/* ------------ uart_connect(baud) -> true | false, err ------------ */
static int lua_uart_connect(lua_State *L)
{
    int baud = (int)luaL_optinteger(L, 1, CONFIG_SAMPLE_UART_DEFAULT_BAUD_RATE);
    script_interface_config_t config = {
        .address_or_baud = (uint32_t)baud,
        .rx_size = 0,
        .tx_pin = -1,
        .rx_pin = -1,
    };
    esp_err_t err = script_interface_connect(SCRIPT_INTERFACE_UART, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Lua uart_connect failed: %s", esp_err_to_name(err));
        lua_pushboolean(L, 0);
        lua_pushstring(L, esp_err_to_name(err));
        return 2;
    }
    lua_pushboolean(L, 1);
    return 1;
}

/* ------------ uart_safe_close() ------------ */
static int lua_uart_safe_close(lua_State *L)
{
    (void)L;
    (void)script_interface_close(SCRIPT_INTERFACE_UART);
    return 0;
}

/* ------------ uart_reset_rx_cursor() -> true | false, err ------------ */
static int lua_uart_reset_rx_cursor(lua_State *L)
{
    if (!script_interface_is_open(SCRIPT_INTERFACE_UART)) {
        lua_pushboolean(L, 0);
        lua_pushliteral(L, "uart not open");
        return 2;
    }

    (void)script_interface_reset_rx_cursor(SCRIPT_INTERFACE_UART);
    lua_pushboolean(L, 1);
    return 1;
}

/* ------------ uart_set_rx_size(n) -> true | false, err ------------ */
static int lua_uart_set_rx_size(lua_State *L)
{
    if (!script_interface_is_open(SCRIPT_INTERFACE_UART)) {
        lua_pushboolean(L, 0);
        lua_pushliteral(L, "uart not open");
        return 2;
    }

    int32_t size = (int32_t)luaL_checkinteger(L, 1);
    if (size < 1) {
        size = 1;
    }
    if (size > 4096) {
        size = 4096;
    }

    esp_err_t err = script_interface_set_rx_size(SCRIPT_INTERFACE_UART, size);
    if (err != ESP_OK) {
        lua_pushboolean(L, 0);
        lua_pushstring(L, esp_err_to_name(err));
        return 2;
    }

    lua_pushboolean(L, 1);
    return 1;
}

/* ------------ uart_write(payload) -> true | false, err ------------ */
static int lua_uart_write(lua_State *L)
{
    if (!script_interface_is_open(SCRIPT_INTERFACE_UART)) {
        lua_pushboolean(L, 0);
        lua_pushliteral(L, "uart not open");
        return 2;
    }

    size_t len = 0;
    const char *payload = luaL_checklstring(L, 1, &len);

    esp_err_t err = script_interface_write(SCRIPT_INTERFACE_UART, (const uint8_t *)payload, len);
    if (err != ESP_OK) {
        lua_pushboolean(L, 0);
        lua_pushliteral(L, "uart tx failed");
        return 2;
    }

    lua_pushboolean(L, 1);
    return 1;
}

/* ------------ uart_read_chunk() -> string ------------ */
static int lua_uart_read_chunk(lua_State *L)
{
    if (!script_interface_is_open(SCRIPT_INTERFACE_UART)) {
        lua_pushliteral(L, "");
        return 1;
    }

    uint8_t buf[256];
    size_t got = 0;
    esp_err_t err = script_interface_read(SCRIPT_INTERFACE_UART, buf, sizeof(buf), &got);
    if (err != ESP_OK || got == 0) {
        lua_pushliteral(L, "");
        return 1;
    }

    lua_pushlstring(L, (const char *)buf, got);
    return 1;
}

/* ------------ uart_sleep(seconds) ------------ */
static int lua_uart_sleep(lua_State *L)
{
    double seconds = luaL_optnumber(L, 1, 0.0);
    if (seconds > 0.0) {
        uint32_t ms = (uint32_t)(seconds * 1000.0);
        if (ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(ms));
        }
    }
    return 0;
}

static bool sample_script_global_buffer_mutex_ensure(void)
{
    if (s_script_global_buffer_mutex != NULL) {
        return true;
    }

    SemaphoreHandle_t created = xSemaphoreCreateMutex();
    if (created == NULL) {
        return false;
    }

    portENTER_CRITICAL(&s_script_global_buffer_mutex_lock);
    if (s_script_global_buffer_mutex == NULL) {
        s_script_global_buffer_mutex = created;
        created = NULL;
    }
    portEXIT_CRITICAL(&s_script_global_buffer_mutex_lock);

    if (created != NULL) {
        vSemaphoreDelete(created);
    }
    return s_script_global_buffer_mutex != NULL;
}

static bool sample_script_serving_buffer_mutex_ensure(void)
{
    if (s_script_serving_buffer_mutex != NULL) {
        return true;
    }

    /* A transfer can finish from the RX/retry task rather than the task which
     * accepted the request, so use a binary semaphore instead of an
     * owner-bound mutex for the serving-buffer lease. */
    SemaphoreHandle_t created = xSemaphoreCreateBinary();
    if (created == NULL) {
        return false;
    }
    xSemaphoreGive(created);

    portENTER_CRITICAL(&s_script_serving_buffer_mutex_lock);
    if (s_script_serving_buffer_mutex == NULL) {
        s_script_serving_buffer_mutex = created;
        created = NULL;
    }
    portEXIT_CRITICAL(&s_script_serving_buffer_mutex_lock);

    if (created != NULL) {
        vSemaphoreDelete(created);
    }
    return s_script_serving_buffer_mutex != NULL;
}

esp_err_t sample_script_global_buffer_set_capacity(size_t capacity)
{
    if (capacity == 0 || capacity > SAMPLE_SCRIPT_GLOBAL_BUFFER_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_script_global_buffer_capacity == capacity &&
        s_script_global_buffer != NULL &&
        s_script_serving_buffer != NULL) {
        if (s_script_global_buffer_length > capacity) {
            s_script_global_buffer_length = capacity;
        }
        return ESP_OK;
    }

    if (s_script_global_buffer == NULL) {
        s_script_global_buffer = (uint8_t *)heap_caps_malloc(
            SAMPLE_SCRIPT_GLOBAL_BUFFER_MAX_SIZE,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (s_script_serving_buffer == NULL) {
        s_script_serving_buffer = (uint8_t *)heap_caps_malloc(
            SAMPLE_SCRIPT_GLOBAL_BUFFER_MAX_SIZE,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (s_script_global_buffer == NULL || s_script_serving_buffer == NULL) {
        ESP_LOGE(TAG,
                 "global buffer PSRAM allocation failed working=%u serving=%u capacity=%u",
                 s_script_global_buffer != NULL ? 1U : 0U,
                 s_script_serving_buffer != NULL ? 1U : 0U,
                 (unsigned)SAMPLE_SCRIPT_GLOBAL_BUFFER_MAX_SIZE);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "global buffers allocated in PSRAM: working=%u serving=%u",
             (unsigned)SAMPLE_SCRIPT_GLOBAL_BUFFER_MAX_SIZE,
             (unsigned)SAMPLE_SCRIPT_GLOBAL_BUFFER_MAX_SIZE);

    s_script_global_buffer_capacity = capacity;
    s_script_serving_buffer_capacity = SAMPLE_SCRIPT_GLOBAL_BUFFER_MAX_SIZE;
    if (s_script_global_buffer_length > capacity) {
        s_script_global_buffer_length = capacity;
    }

    return ESP_OK;
}

size_t sample_script_global_buffer_get_capacity(void)
{
    return s_script_global_buffer_capacity;
}

size_t sample_script_global_buffer_get_length(void)
{
    return s_script_global_buffer_length;
}

size_t sample_script_global_buffer_get_serving_length(void)
{
    return s_script_serving_buffer_length;
}

const uint8_t *sample_script_global_buffer_get_data(void)
{
    return s_script_global_buffer;
}

esp_err_t sample_script_global_buffer_publish_for_serving(void)
{
    if (!sample_script_serving_buffer_mutex_ensure()) {
        return ESP_ERR_NO_MEM;
    }
    if (xSemaphoreTake(s_script_serving_buffer_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ESP_OK;
    if (s_script_global_buffer == NULL || s_script_serving_buffer == NULL) {
        err = ESP_ERR_INVALID_STATE;
    } else if (s_script_global_buffer_length > s_script_serving_buffer_capacity) {
        err = ESP_ERR_INVALID_SIZE;
    } else {
        if (s_script_global_buffer_length > 0) {
            memcpy(s_script_serving_buffer,
                   s_script_global_buffer,
                   s_script_global_buffer_length);
        }
        s_script_serving_buffer_length = s_script_global_buffer_length;
        ESP_LOGI(TAG,
                 "Global buffer sample published working_len=%u serving_len=%u",
                 (unsigned)s_script_global_buffer_length,
                 (unsigned)s_script_serving_buffer_length);
    }

    xSemaphoreGive(s_script_serving_buffer_mutex);
    return err;
}

esp_err_t sample_script_global_buffer_acquire_serving(const uint8_t **data,
                                                      size_t *length)
{
    if (data == NULL || length == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *data = NULL;
    *length = 0;

    if (!sample_script_serving_buffer_mutex_ensure()) {
        return ESP_ERR_NO_MEM;
    }
    if (xSemaphoreTake(s_script_serving_buffer_mutex, 0) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_script_serving_buffer == NULL || s_script_serving_buffer_length == 0) {
        xSemaphoreGive(s_script_serving_buffer_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    *data = s_script_serving_buffer;
    *length = s_script_serving_buffer_length;
    return ESP_OK;
}

void sample_script_global_buffer_release_serving(void)
{
    if (s_script_serving_buffer_mutex != NULL) {
        xSemaphoreGive(s_script_serving_buffer_mutex);
    }
}

void sample_script_global_buffer_reset(void)
{
    s_script_global_buffer_length = 0;
}

esp_err_t sample_script_global_buffer_append(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_script_global_buffer_capacity == 0 || s_script_global_buffer == NULL) {
        esp_err_t init_err = sample_script_global_buffer_set_capacity(SAMPLE_SCRIPT_GLOBAL_BUFFER_DEFAULT_SIZE);
        if (init_err != ESP_OK) {
            return init_err;
        }
    }

    if (len > (s_script_global_buffer_capacity - s_script_global_buffer_length)) {
        return ESP_ERR_NO_MEM;
    }

    memcpy(s_script_global_buffer + s_script_global_buffer_length, data, len);
    s_script_global_buffer_length += len;
    return ESP_OK;
}

esp_err_t sample_script_global_buffer_write_at(size_t pos, const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_script_global_buffer_capacity == 0 || s_script_global_buffer == NULL) {
        esp_err_t init_err = sample_script_global_buffer_set_capacity(SAMPLE_SCRIPT_GLOBAL_BUFFER_DEFAULT_SIZE);
        if (init_err != ESP_OK) {
            return init_err;
        }
    }

    if (pos >= s_script_global_buffer_capacity ||
        len > (s_script_global_buffer_capacity - pos)) {
        return ESP_ERR_NO_MEM;
    }

    if (pos > s_script_global_buffer_length) {
        /* Fill the gap to avoid exposing stale bytes when extending sparsely. */
        memset(s_script_global_buffer + s_script_global_buffer_length,
               0,
               pos - s_script_global_buffer_length);
    }

    memcpy(s_script_global_buffer + pos, data, len);
    const size_t new_length = pos + len;
    if (new_length > s_script_global_buffer_length) {
        s_script_global_buffer_length = new_length;
    }
    return ESP_OK;
}

/* ------------ Modbus CRC16 helper ------------ */
static uint16_t modbus_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ------------ util_bytes_to_hex(bytes) -> hex_string ------------ */
static int lua_util_bytes_to_hex(lua_State *L)
{
    size_t len = 0;
    const char *bytes = luaL_checklstring(L, 1, &len);
    luaL_Buffer buf;
    luaL_buffinit(L, &buf);
    for (size_t i = 0; i < len; i++) {
        char hex[3];
        snprintf(hex, sizeof(hex), "%02x", (unsigned char)bytes[i]);
        luaL_addlstring(&buf, hex, 2);
    }
    luaL_pushresult(&buf);
    return 1;
}

/* ------------ util_build_read_holding_request(unit_id, address, count) -> binary_frame ------------ */
static int lua_util_build_read_holding_request(lua_State *L)
{
    int unit_id = (int)luaL_checkinteger(L, 1);
    int address = (int)luaL_checkinteger(L, 2);
    int count   = (int)luaL_checkinteger(L, 3);

    uint8_t frame[8];
    frame[0] = (uint8_t)(unit_id & 0xFF);
    frame[1] = 0x03;
    frame[2] = (uint8_t)((address >> 8) & 0xFF);
    frame[3] = (uint8_t)(address & 0xFF);
    frame[4] = (uint8_t)((count >> 8) & 0xFF);
    frame[5] = (uint8_t)(count & 0xFF);

    uint16_t crc = modbus_crc16(frame, 6);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)((crc >> 8) & 0xFF);

    lua_pushlstring(L, (const char *)frame, 8);
    return 1;
}

/* ------------ util_extract_modbus_frame(buffer, unit_id, func_code, byte_count) -> frame|nil ------------ */
static int lua_util_extract_modbus_frame(lua_State *L)
{
    size_t buf_len = 0;
    const char *buffer = luaL_checklstring(L, 1, &buf_len);
    int unit_id       = (int)luaL_checkinteger(L, 2);
    int function_code = (int)luaL_checkinteger(L, 3);
    int byte_count    = (int)luaL_checkinteger(L, 4);

    size_t expected_len = (size_t)(5 + byte_count);
    if (buf_len < expected_len) {
        lua_pushnil(L);
        return 1;
    }

    const uint8_t *buf = (const uint8_t *)buffer;
    for (size_t start = 0; start <= buf_len - expected_len; start++) {
        if (buf[start]     == (uint8_t)unit_id &&
            buf[start + 1] == (uint8_t)function_code &&
            buf[start + 2] == (uint8_t)byte_count) {
            const uint8_t *frame_ptr = buf + start;
            uint16_t crc = modbus_crc16(frame_ptr, expected_len - 2);
            uint8_t lo = frame_ptr[expected_len - 2];
            uint8_t hi = frame_ptr[expected_len - 1];
            if (lo == (crc & 0xFF) && hi == ((crc >> 8) & 0xFF)) {
                lua_pushlstring(L, (const char *)frame_ptr, expected_len);
                return 1;
            }
        }
    }

    lua_pushnil(L);
    return 1;
}

/* ------------ util_decode_bcd_32(value) -> integer ------------ */
static int lua_util_decode_bcd_32(lua_State *L)
{
    uint32_t v = (uint32_t)luaL_checkinteger(L, 1);
    uint32_t result =
        ((v >> 28) & 0xF) * 10000000 +
        ((v >> 24) & 0xF) * 1000000 +
        ((v >> 20) & 0xF) * 100000 +
        ((v >> 16) & 0xF) * 10000 +
        ((v >> 12) & 0xF) * 1000 +
        ((v >>  8) & 0xF) * 100 +
        ((v >>  4) & 0xF) * 10 +
        (v & 0xF);
    lua_pushinteger(L, (lua_Integer)result);
    return 1;
}

/* ------------ util_log(cfg, tag, msg) ------------ */
static int lua_util_log(lua_State *L)
{
    bool quiet = false;
    if (lua_istable(L, 1)) {
        lua_getfield(L, 1, "quiet");
        quiet = lua_toboolean(L, -1);
        lua_pop(L, 1);
    }
    if (quiet) return 0;

    const char *tag = luaL_optstring(L, 2, "Lua");
    const char *msg = luaL_optstring(L, 3, "");
    ESP_LOGI(TAG, "[%s] %s", tag, msg);
    return 0;
}

/* ------------ util_get_sample_runtime() -> {sample_counter, server_sec_of_year, server_sec_of_year_valid} ------------ */
static int lua_util_get_sample_runtime(lua_State *L)
{
    uint32_t server_sec_of_year = 0;
    bool server_sec_valid = sample_get_server_sec_of_year(&server_sec_of_year);
    if (server_sec_valid) {
        server_sec_of_year %= (7U * 24U * 60U * 60U);
    }

    lua_newtable(L);

    lua_pushinteger(L, (lua_Integer)s_sample_script_run_counter);
    lua_setfield(L, -2, "sample_counter");

    lua_pushinteger(L, (lua_Integer)server_sec_of_year);
    lua_setfield(L, -2, "server_sec_of_year");

    lua_pushboolean(L, server_sec_valid ? 1 : 0);
    lua_setfield(L, -2, "server_sec_of_year_valid");

    return 1;
}

/* ------------ util_get_sample_counter() -> sample_counter ------------ */
static int lua_util_get_sample_counter(lua_State *L)
{
    lua_pushinteger(L, (lua_Integer)s_sample_script_run_counter);
    return 1;
}

/* ------------ util_get_server_sec_of_year() -> sec_of_year, valid ------------ */
static int lua_util_get_server_sec_of_year(lua_State *L)
{
    uint32_t server_sec_of_year = 0;
    bool server_sec_valid = sample_get_server_sec_of_year(&server_sec_of_year);
    if (server_sec_valid) {
        server_sec_of_year %= (7U * 24U * 60U * 60U);
    }

    lua_pushinteger(L, (lua_Integer)server_sec_of_year);
    lua_pushboolean(L, server_sec_valid ? 1 : 0);
    return 2;
}

/* ------------ util_get_day_of_week() -> day_of_week(1..7), valid ------------ */
static int lua_util_get_day_of_week(lua_State *L)
{
    uint32_t server_sec_of_year = 0;
    bool server_sec_valid = sample_get_server_sec_of_year(&server_sec_of_year);
    if (!server_sec_valid) {
        lua_pushinteger(L, 0);
        lua_pushboolean(L, 0);
        return 2;
    }

    const uint32_t sec_of_week = server_sec_of_year % (7U * 24U * 60U * 60U);
    const uint32_t day_of_week = (sec_of_week / (24U * 60U * 60U)) + 1U;
    lua_pushinteger(L, (lua_Integer)day_of_week);
    lua_pushboolean(L, 1);
    return 2;
}

/* ------------ util_get_hour_of_day() -> hour_of_day, valid ------------ */
static int lua_util_get_hour_of_day(lua_State *L)
{
    uint32_t server_sec_of_year = 0;
    bool server_sec_valid = sample_get_server_sec_of_year(&server_sec_of_year);
    if (!server_sec_valid) {
        lua_pushinteger(L, 0);
        lua_pushboolean(L, 0);
        return 2;
    }

    const uint32_t sec_of_week = server_sec_of_year % (7U * 24U * 60U * 60U);
    const uint32_t sec_of_day = sec_of_week % (24U * 60U * 60U);
    const uint32_t hour_of_day = sec_of_day / (60U * 60U);
    lua_pushinteger(L, (lua_Integer)hour_of_day);
    lua_pushboolean(L, 1);
    return 2;
}

/* ------------ util_get_min_of_day() -> min_of_day, valid ------------ */
static int lua_util_get_min_of_day(lua_State *L)
{
    uint32_t server_sec_of_year = 0;
    bool server_sec_valid = sample_get_server_sec_of_year(&server_sec_of_year);
    if (!server_sec_valid) {
        lua_pushinteger(L, 0);
        lua_pushboolean(L, 0);
        return 2;
    }

    const uint32_t sec_of_week = server_sec_of_year % (7U * 24U * 60U * 60U);
    const uint32_t sec_of_day = sec_of_week % (24U * 60U * 60U);
    const uint32_t min_of_day = sec_of_day / 60U;
    lua_pushinteger(L, (lua_Integer)min_of_day);
    lua_pushboolean(L, 1);
    return 2;
}

/* ------------ util_init_global_buffer([capacity]) -> true | false, err ------------ */
static int lua_util_init_global_buffer(lua_State *L)
{
    int argc = lua_gettop(L);
    if (argc >= 1 && !lua_isnil(L, 1)) {
        lua_Integer requested = luaL_checkinteger(L, 1);
        if (requested <= 0 || requested > (lua_Integer)SAMPLE_SCRIPT_GLOBAL_BUFFER_MAX_SIZE) {
            lua_pushboolean(L, 0);
            lua_pushliteral(L, "invalid global buffer size");
            return 2;
        }

        esp_err_t set_err = sample_script_global_buffer_set_capacity((size_t)requested);
        if (set_err != ESP_OK) {
            lua_pushboolean(L, 0);
            lua_pushstring(L, esp_err_to_name(set_err));
            return 2;
        }
    } else if (sample_script_global_buffer_get_capacity() == 0) {
        esp_err_t set_err = sample_script_global_buffer_set_capacity(SAMPLE_SCRIPT_GLOBAL_BUFFER_DEFAULT_SIZE);
        if (set_err != ESP_OK) {
            lua_pushboolean(L, 0);
            lua_pushstring(L, esp_err_to_name(set_err));
            return 2;
        }
    }

    sample_script_global_buffer_reset();
    lua_pushboolean(L, 1);
    return 1;
}

/* ------------ util_append_global_buffer(payload) -> true | false, err ------------ */
static int lua_util_append_global_buffer(lua_State *L)
{
    size_t payload_len = 0;
    const char *payload = luaL_checklstring(L, 1, &payload_len);

    esp_err_t append_err = sample_script_global_buffer_append((const uint8_t *)payload, payload_len);
    if (append_err != ESP_OK) {
        lua_pushboolean(L, 0);
        lua_pushstring(L, esp_err_to_name(append_err));
        return 2;
    }

    lua_pushboolean(L, 1);
    return 1;
}

/* ------------ util_write_global_buffer_at(pos, payload) -> true | false, err ------------ */
static int lua_util_write_global_buffer_at(lua_State *L)
{
    lua_Integer pos = luaL_checkinteger(L, 1);
    if (pos < 0) {
        lua_pushboolean(L, 0);
        lua_pushliteral(L, "invalid position");
        return 2;
    }

    size_t payload_len = 0;
    const char *payload = luaL_checklstring(L, 2, &payload_len);

    esp_err_t write_err = sample_script_global_buffer_write_at((size_t)pos,
                                                                (const uint8_t *)payload,
                                                                payload_len);
    if (write_err != ESP_OK) {
        lua_pushboolean(L, 0);
        lua_pushstring(L, esp_err_to_name(write_err));
        return 2;
    }

    lua_pushboolean(L, 1);
    return 1;
}

/* ------------ rs485_connect(baud) -> true | false, err ------------ */
static int lua_rs485_connect(lua_State *L)
{
    int baud = (int)luaL_optinteger(L, 1, CONFIG_RS485_DEFAULT_BAUD_RATE);
    script_interface_config_t config = {
        .address_or_baud = (uint32_t)baud,
        .rx_size = 0,
        .unit_id = 0,
        .mode = 0,
        .tx_pin = -1,
        .rx_pin = -1,
    };
    esp_err_t err = script_interface_connect(SCRIPT_INTERFACE_RS485, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Lua rs485_connect failed: %s", esp_err_to_name(err));
        lua_pushboolean(L, 0);
        lua_pushstring(L, esp_err_to_name(err));
        return 2;
    }
    lua_pushboolean(L, 1);
    return 1;
}

/* ------------ rs485_safe_close() ------------ */
static int lua_rs485_safe_close(lua_State *L)
{
    (void)L;
    (void)script_interface_close(SCRIPT_INTERFACE_RS485);
    return 0;
}

/* ------------ rs485_reset_rx_cursor() -> true | false, err ------------ */
static int lua_rs485_reset_rx_cursor(lua_State *L)
{
    if (!script_interface_is_open(SCRIPT_INTERFACE_RS485)) {
        lua_pushboolean(L, 0);
        lua_pushliteral(L, "rs485 not open");
        return 2;
    }
    (void)script_interface_reset_rx_cursor(SCRIPT_INTERFACE_RS485);
    lua_pushboolean(L, 1);
    return 1;
}

/* ------------ rs485_write(payload) -> true | false, err ------------ */
static int lua_rs485_write(lua_State *L)
{
    if (!script_interface_is_open(SCRIPT_INTERFACE_RS485)) {
        lua_pushboolean(L, 0);
        lua_pushliteral(L, "rs485 not open");
        return 2;
    }

    size_t len = 0;
    const char *payload = luaL_checklstring(L, 1, &len);

    esp_err_t err = script_interface_write(SCRIPT_INTERFACE_RS485, (const uint8_t *)payload, len);
    if (err != ESP_OK) {
        lua_pushboolean(L, 0);
        lua_pushliteral(L, "rs485 tx failed");
        return 2;
    }

    lua_pushboolean(L, 1);
    return 1;
}

/* ------------ rs485_read_chunk() -> string ------------ */
static int lua_rs485_read_chunk(lua_State *L)
{
    if (!script_interface_is_open(SCRIPT_INTERFACE_RS485)) {
        lua_pushliteral(L, "");
        return 1;
    }

    uint8_t buf[256];
    size_t got = 0;
    esp_err_t err = script_interface_read(SCRIPT_INTERFACE_RS485, buf, sizeof(buf), &got);
    if (err != ESP_OK || got == 0) {
        lua_pushliteral(L, "");
        return 1;
    }

    lua_pushlstring(L, (const char *)buf, got);
    return 1;
}

/* ------------ rs485_sleep(seconds) ------------ */
static int lua_rs485_sleep(lua_State *L)
{
    double seconds = luaL_optnumber(L, 1, 0.0);
    if (seconds > 0.0) {
        uint32_t ms = (uint32_t)(seconds * 1000.0);
        if (ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(ms));
        }
    }
    return 0;
}

/* ============================================================
 * Lua global functions injected for sht3x_temp.lua (I2C sensors)
 * These mirror the globals set by util.lua and i2c_interface.lua
 * in the SDK, allowing the same sht3x_temp.lua to run natively.
 * ============================================================ */

static bool sample_sensor_selector_is_empty(const char *selector)
{
    return !selector ||
           selector[0] == '\0' ||
           strcmp(selector, "none") == 0 ||
           strcmp(selector, "unspecified") == 0;
}

/* ------------ util_crc8(data [, poly [, init]]) -> integer ------------ */
static int lua_util_crc8(lua_State *L)
{
    size_t len = 0;
    const char *data = luaL_checklstring(L, 1, &len);
    int poly = (int)luaL_optinteger(L, 2, 0x31);
    int init = (int)luaL_optinteger(L, 3, 0xFF);

    uint8_t crc = (uint8_t)(init & 0xFF);
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint8_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ (uint8_t)(poly & 0xFF));
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }

    lua_pushinteger(L, (lua_Integer)crc);
    return 1;
}

/* ------------ i2c_connect(addr) -> true | false, err ------------ */
static int lua_i2c_connect(lua_State *L)
{
    uint32_t addr = (uint32_t)luaL_optinteger(L, 1, 0x44);
    script_interface_config_t config = {
        .address_or_baud = addr,
        .rx_size = 0,
        .tx_pin = -1,
        .rx_pin = -1,
    };
    esp_err_t err = script_interface_connect(SCRIPT_INTERFACE_I2C, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Lua i2c_connect failed: %s", esp_err_to_name(err));
        lua_pushboolean(L, 0);
        lua_pushstring(L, esp_err_to_name(err));
        return 2;
    }

    lua_pushboolean(L, 1);
    return 1;
}

/* ------------ i2c_safe_close() ------------ */
static int lua_i2c_safe_close(lua_State *L)
{
    (void)L;
    (void)script_interface_close(SCRIPT_INTERFACE_I2C);
    return 0;
}

/* ------------ i2c_reset_rx_cursor() -> true | false, err ------------ */
static int lua_i2c_reset_rx_cursor(lua_State *L)
{
    if (!script_interface_is_open(SCRIPT_INTERFACE_I2C)) {
        lua_pushboolean(L, 0);
        lua_pushliteral(L, "i2c not open");
        return 2;
    }
    lua_pushboolean(L, 1);
    return 1;
}

/* ------------ i2c_set_rx_size(n) -> true | false, err ------------ */
static int lua_i2c_set_rx_size(lua_State *L)
{
    if (!script_interface_is_open(SCRIPT_INTERFACE_I2C)) {
        lua_pushboolean(L, 0);
        lua_pushliteral(L, "i2c not open");
        return 2;
    }
    int32_t size = (int32_t)luaL_checkinteger(L, 1);
    esp_err_t err = script_interface_set_rx_size(SCRIPT_INTERFACE_I2C, size);
    if (err != ESP_OK) {
        lua_pushboolean(L, 0);
        lua_pushstring(L, esp_err_to_name(err));
        return 2;
    }
    lua_pushboolean(L, 1);
    return 1;
}

/* ------------ i2c_write(payload) -> true | false, err ------------ */
static int lua_i2c_write(lua_State *L)
{
    if (!script_interface_is_open(SCRIPT_INTERFACE_I2C)) {
        lua_pushboolean(L, 0);
        lua_pushliteral(L, "i2c not open");
        return 2;
    }

    size_t len = 0;
    const char *payload = luaL_checklstring(L, 1, &len);

    esp_err_t err = script_interface_write(SCRIPT_INTERFACE_I2C,
                                           (const uint8_t *)payload,
                                           len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Lua i2c_write failed: %s", esp_err_to_name(err));
        lua_pushboolean(L, 0);
        lua_pushstring(L, esp_err_to_name(err));
        return 2;
    }

    lua_pushboolean(L, 1);
    return 1;
}

/* ------------ i2c_read_chunk() -> string ------------ */
static int lua_i2c_read_chunk(lua_State *L)
{
    if (!script_interface_is_open(SCRIPT_INTERFACE_I2C)) {
        lua_pushliteral(L, "");
        return 1;
    }

    uint8_t buf[256] = {0};
    size_t read_len = 0;
    esp_err_t err = script_interface_read(SCRIPT_INTERFACE_I2C, buf, sizeof(buf), &read_len);
    if (err != ESP_OK || read_len == 0) {
        ESP_LOGW(TAG, "Lua i2c_read_chunk failed: %s", esp_err_to_name(err));
        lua_pushliteral(L, "");
        return 1;
    }

    lua_pushlstring(L, (const char *)buf, read_len);
    return 1;
}

/* ------------ i2c_sleep(seconds) ------------ */
static int lua_i2c_sleep(lua_State *L)
{
    double seconds = luaL_optnumber(L, 1, 0.0);
    if (seconds > 0.0) {
        uint32_t ms = (uint32_t)(seconds * 1000.0);
        if (ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(ms));
        }
    }
    return 0;
}

static bool lua_table_has_result_fields(lua_State *L, int table_idx);

static const char *lua_type_name_at(lua_State *L, int idx)
{
    return lua_typename(L, lua_type(L, idx));
}

static void lua_log_stack_snapshot(lua_State *L, const char *phase)
{
    if (!L) {
        return;
    }

    int top = lua_gettop(L);
    ESP_LOGI(TAG, "Lua stack snapshot (%s): top=%d", phase ? phase : "unknown", top);

    int max_dump = top;
    if (max_dump > 6) {
        max_dump = 6;
    }

    for (int i = 1; i <= max_dump; i++) {
        int t = lua_type(L, i);
        if (t == LUA_TNUMBER) {
            ESP_LOGI(TAG, "  stack[%d] type=%s value=%.6f", i, lua_type_name_at(L, i), lua_tonumber(L, i));
        } else if (t == LUA_TBOOLEAN) {
            ESP_LOGI(TAG, "  stack[%d] type=%s value=%s", i, lua_type_name_at(L, i), lua_toboolean(L, i) ? "true" : "false");
        } else if (t == LUA_TSTRING) {
            size_t slen = 0;
            const char *s = lua_tolstring(L, i, &slen);
            ESP_LOGI(TAG, "  stack[%d] type=%s len=%u value=%.*s",
                     i,
                     lua_type_name_at(L, i),
                     (unsigned)slen,
                     (int)((slen > 48) ? 48 : slen),
                     s ? s : "");
        } else if (t == LUA_TTABLE) {
            ESP_LOGI(TAG, "  stack[%d] type=%s rawlen=%u", i, lua_type_name_at(L, i), (unsigned)lua_rawlen(L, i));
        } else {
            ESP_LOGI(TAG, "  stack[%d] type=%s", i, lua_type_name_at(L, i));
        }
    }
}

static void lua_log_table_shape(lua_State *L, int table_idx, const char *label)
{
    if (!L || !lua_istable(L, table_idx)) {
        ESP_LOGI(TAG, "Lua table shape (%s): not a table", label ? label : "unknown");
        return;
    }

    if (table_idx < 0) {
        table_idx = lua_gettop(L) + table_idx + 1;
    }

    size_t rawlen = lua_rawlen(L, table_idx);
    ESP_LOGI(TAG,
             "Lua table shape (%s): rawlen=%u direct_record=%d",
             label ? label : "unknown",
             (unsigned)rawlen,
             lua_table_has_result_fields(L, table_idx) ? 1 : 0);

    int pair_count = 0;
    int nested_table_count = 0;
    int nested_record_count = 0;

    lua_pushnil(L);
    while (lua_next(L, table_idx) != 0) {
        pair_count++;

        if (lua_istable(L, -1)) {
            nested_table_count++;
            if (lua_table_has_result_fields(L, -1)) {
                nested_record_count++;
            }
        }

        if (pair_count <= 8) {
            const char *key_type = lua_type_name_at(L, -2);
            const char *val_type = lua_type_name_at(L, -1);
            if (lua_type(L, -2) == LUA_TNUMBER) {
                ESP_LOGI(TAG,
                         "  table[%s] entry %d: key(num)=%.0f val_type=%s",
                         label ? label : "unknown",
                         pair_count,
                         lua_tonumber(L, -2),
                         val_type);
            } else if (lua_type(L, -2) == LUA_TSTRING) {
                size_t key_len = 0;
                const char *key = lua_tolstring(L, -2, &key_len);
                ESP_LOGI(TAG,
                         "  table[%s] entry %d: key(str)=%.*s val_type=%s",
                         label ? label : "unknown",
                         pair_count,
                         (int)((key_len > 48) ? 48 : key_len),
                         key ? key : "",
                         val_type);
            } else {
                ESP_LOGI(TAG,
                         "  table[%s] entry %d: key_type=%s val_type=%s",
                         label ? label : "unknown",
                         pair_count,
                         key_type,
                         val_type);
            }
        }

        lua_pop(L, 1);
    }

    ESP_LOGI(TAG,
             "Lua table shape (%s) summary: pairs=%d nested_tables=%d nested_records=%d",
             label ? label : "unknown",
             pair_count,
             nested_table_count,
             nested_record_count);
}

static bool lua_table_has_result_fields(lua_State *L, int table_idx)
{
    if (!lua_istable(L, table_idx)) {
        return false;
    }

    if (table_idx < 0) {
        table_idx = lua_gettop(L) + table_idx + 1;
    }

    lua_getfield(L, table_idx, "object");
    bool has_object = !lua_isnil(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, table_idx, "instance");
    bool has_instance = !lua_isnil(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, table_idx, "resource");
    bool has_resource = !lua_isnil(L, -1);
    lua_pop(L, 1);

    if (has_object && has_instance && has_resource) {
        return true;
    }

    lua_getfield(L, table_idx, "type");
    bool has_type = lua_isinteger(L, -1);
    lua_pop(L, 1);

    const char *value_fields[] = {"bool_value", "int_value", "float_value"};
    bool has_value = false;
    for (size_t i = 0; i < sizeof(value_fields) / sizeof(value_fields[0]); i++) {
        lua_getfield(L, table_idx, value_fields[i]);
        has_value = !lua_isnil(L, -1);
        lua_pop(L, 1);
        if (has_value) break;
    }
    return has_type && has_value;
}

static size_t lua_append_serialized_result_record(lua_State *L,
                                                  int entry_idx,
                                                  char *out,
                                                  size_t out_size,
                                                  size_t written,
                                                  bool *first)
{
    if (!lua_istable(L, entry_idx) || !out || out_size == 0 || !first) {
        return written;
    }

    if (entry_idx < 0) {
        entry_idx = lua_gettop(L) + entry_idx + 1;
    }

    if (!*first && written < out_size - 1) {
        written += (size_t)snprintf(out + written, out_size - written, ",");
    }
    *first = false;

    lua_getfield(L, entry_idx, "type");
    bool protobuf_sensor_data = lua_isinteger(L, -1);
    int sensor_type = protobuf_sensor_data ? (int)lua_tointeger(L, -1) : 0;
    lua_pop(L, 1);
    if (protobuf_sensor_data) {
        lua_getfield(L, entry_idx, "bool_value");
        if (!lua_isnil(L, -1)) {
            int value = lua_toboolean(L, -1) ? 1 : 0;
            lua_pop(L, 1);
            return written + (size_t)snprintf(out + written, out_size - written,
                                               "{\"type\":%d,\"bool_value\":%s}",
                                               sensor_type, value ? "true" : "false");
        }
        lua_pop(L, 1);
        lua_getfield(L, entry_idx, "int_value");
        if (!lua_isnil(L, -1)) {
            lua_Integer value = lua_tointeger(L, -1);
            lua_pop(L, 1);
            return written + (size_t)snprintf(out + written, out_size - written,
                                               "{\"type\":%d,\"int_value\":%lld}",
                                               sensor_type, (long long)value);
        }
        lua_pop(L, 1);
        lua_getfield(L, entry_idx, "float_value");
        double value = lua_tonumber(L, -1);
        lua_pop(L, 1);
        return written + (size_t)snprintf(out + written, out_size - written,
                                           "{\"type\":%d,\"float_value\":%.6f}",
                                           sensor_type, value);
    }

    lua_getfield(L, entry_idx, "object");
    int obj = (int)lua_tointeger(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, entry_idx, "instance");
    int inst = (int)lua_tointeger(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, entry_idx, "resource");
    int res = (int)lua_tointeger(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, entry_idx, "value");
    if (lua_isboolean(L, -1)) {
        int v = lua_toboolean(L, -1) ? 1 : 0;
        written += (size_t)snprintf(out + written, out_size - written,
                                    "{\"o\":%d,\"i\":%d,\"r\":%d,\"v\":%d}",
                                    obj, inst, res, v);
    } else {
        double val = lua_tonumber(L, -1);
        written += (size_t)snprintf(out + written, out_size - written,
                                    "{\"o\":%d,\"i\":%d,\"r\":%d,\"v\":%.6f}",
                                    obj, inst, res, val);
    }
    lua_pop(L, 1);

    return written;
}

static void lua_log_result_record(lua_State *L, int entry_idx, int entry_index)
{
    if (!lua_istable(L, entry_idx)) {
        return;
    }

    if (entry_idx < 0) {
        entry_idx = lua_gettop(L) + entry_idx + 1;
    }

    lua_getfield(L, entry_idx, "type");
    bool protobuf_sensor_data = lua_isinteger(L, -1);
    int sensor_type = protobuf_sensor_data ? (int)lua_tointeger(L, -1) : 0;
    lua_pop(L, 1);
    if (protobuf_sensor_data) {
        const char *value_fields[] = {"bool_value", "int_value", "float_value"};
        for (size_t i = 0; i < sizeof(value_fields) / sizeof(value_fields[0]); i++) {
            lua_getfield(L, entry_idx, value_fields[i]);
            if (!lua_isnil(L, -1)) {
                const char *value_text = luaL_tolstring(L, -1, NULL);
                ESP_LOGI(TAG,
                         "Lua SensorData entry[%d]: type=%d %s=%s",
                         entry_index,
                         sensor_type,
                         value_fields[i],
                         value_text ? value_text : "");
                lua_pop(L, 2);
                return;
            }
            lua_pop(L, 1);
        }
    }

    lua_getfield(L, entry_idx, "object");
    int obj = (int)lua_tointeger(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, entry_idx, "instance");
    int inst = (int)lua_tointeger(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, entry_idx, "resource");
    int res = (int)lua_tointeger(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, entry_idx, "value");
    if (lua_isboolean(L, -1)) {
        bool val = lua_toboolean(L, -1);
        ESP_LOGI(TAG,
                 "Lua result entry[%d]: object=%d instance=%d resource=%d value=%s",
                 entry_index,
                 obj,
                 inst,
                 res,
                 val ? "true" : "false");
    } else if (lua_isnumber(L, -1)) {
        double val = lua_tonumber(L, -1);
        ESP_LOGI(TAG,
                 "Lua result entry[%d]: object=%d instance=%d resource=%d value=%.6f",
                 entry_index,
                 obj,
                 inst,
                 res,
                 val);
    } else if (lua_isstring(L, -1)) {
        const char *val = lua_tostring(L, -1);
        ESP_LOGI(TAG,
                 "Lua result entry[%d]: object=%d instance=%d resource=%d value=%s",
                 entry_index,
                 obj,
                 inst,
                 res,
                 val ? val : "");
    } else {
        ESP_LOGI(TAG,
                 "Lua result entry[%d]: object=%d instance=%d resource=%d value_type=%s",
                 entry_index,
                 obj,
                 inst,
                 res,
                 luaL_typename(L, -1));
    }
    lua_pop(L, 1);
}

static size_t lua_json_escape_string(const char *src, char *dst, size_t dst_size)
{
    if (!src || !dst || dst_size == 0) {
        return 0;
    }

    size_t out = 0;
    for (size_t i = 0; src[i] != '\0' && out + 1 < dst_size; i++) {
        unsigned char c = (unsigned char)src[i];
        if (c == '\\' || c == '"') {
            if (out + 2 >= dst_size) {
                break;
            }
            dst[out++] = '\\';
            dst[out++] = (char)c;
            continue;
        }

        if (c >= 0x20) {
            dst[out++] = (char)c;
        }
    }

    dst[out] = '\0';
    return out;
}

static esp_err_t sample_ensure_lua_result_workspace(void)
{
    size_t line_capacity = (size_t)SAMPLE_DATA_FIELD_LINE_MAX_LEN + 1;
    size_t escaped_capacity = (size_t)SAMPLE_DATA_FIELD_LINE_MAX_LEN;

    if (!s_lua_result_line_buf || s_lua_result_line_buf_capacity < line_capacity) {
        char *next_line = (char *)heap_caps_realloc(s_lua_result_line_buf,
                                                    line_capacity,
                                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!next_line) {
            next_line = (char *)heap_caps_realloc(s_lua_result_line_buf,
                                                  line_capacity,
                                                  MALLOC_CAP_8BIT);
        }
        if (!next_line) {
            return ESP_ERR_NO_MEM;
        }
        s_lua_result_line_buf = next_line;
        s_lua_result_line_buf_capacity = line_capacity;
    }

    if (!s_lua_result_escaped_buf || s_lua_result_escaped_buf_capacity < escaped_capacity) {
        char *next_escaped = (char *)heap_caps_realloc(s_lua_result_escaped_buf,
                                                       escaped_capacity,
                                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!next_escaped) {
            next_escaped = (char *)heap_caps_realloc(s_lua_result_escaped_buf,
                                                     escaped_capacity,
                                                     MALLOC_CAP_8BIT);
        }
        if (!next_escaped) {
            return ESP_ERR_NO_MEM;
        }
        s_lua_result_escaped_buf = next_escaped;
        s_lua_result_escaped_buf_capacity = escaped_capacity;
    }

    s_lua_result_line_buf[0] = '\0';
    s_lua_result_escaped_buf[0] = '\0';
    return ESP_OK;
}

static bool lua_enqueue_result_record_line(lua_State *L, int entry_idx)
{
    if (!lua_istable(L, entry_idx)) {
        return false;
    }

    if (entry_idx < 0) {
        entry_idx = lua_gettop(L) + entry_idx + 1;
    }

    lua_getfield(L, entry_idx, "type");
    bool protobuf_sensor_data = lua_isinteger(L, -1);
    int sensor_type = protobuf_sensor_data ? (int)lua_tointeger(L, -1) : 0;
    lua_pop(L, 1);
    if (protobuf_sensor_data) {
        if (sample_ensure_lua_result_workspace() != ESP_OK) return false;
        char *line = s_lua_result_line_buf;
        size_t line_capacity = s_lua_result_line_buf_capacity;

        lua_getfield(L, entry_idx, "bool_value");
        if (!lua_isnil(L, -1)) {
            int value = lua_toboolean(L, -1) ? 1 : 0;
            sample_update_latest_sensor_type(sensor_type, value);
            snprintf(line, line_capacity, "{\"type\":%d,\"bool_value\":%s}", sensor_type, value ? "true" : "false");
            lua_pop(L, 1);
        } else {
            lua_pop(L, 1);
            lua_getfield(L, entry_idx, "int_value");
            if (!lua_isnil(L, -1)) {
                lua_Integer value = lua_tointeger(L, -1);
                sample_update_latest_sensor_type(sensor_type, (double)value);
                snprintf(line, line_capacity, "{\"type\":%d,\"int_value\":%lld}", sensor_type, (long long)value);
                lua_pop(L, 1);
            } else {
                lua_pop(L, 1);
                lua_getfield(L, entry_idx, "float_value");
                if (!lua_isnumber(L, -1)) {
                    lua_pop(L, 1);
                    return false;
                }
                double value = lua_tonumber(L, -1);
                sample_update_latest_sensor_type(sensor_type, value);
                snprintf(line, line_capacity, "{\"type\":%d,\"float_value\":%.6f}", sensor_type, value);
                lua_pop(L, 1);
            }
        }
        size_t line_len = strnlen(line, line_capacity);
        if (line_len == 0 || line_len > SAMPLE_DATA_FIELD_LINE_MAX_LEN) return false;
        sample_ring_push_payload(line, line_len);
        return true;
    }

    lua_getfield(L, entry_idx, "object");
    int obj = (int)lua_tointeger(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, entry_idx, "instance");
    int inst = (int)lua_tointeger(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, entry_idx, "resource");
    int res = (int)lua_tointeger(L, -1);
    lua_pop(L, 1);

    if (obj < 0 || inst < 0 || res < 0) {
        return false;
    }

    if (sample_ensure_lua_result_workspace() != ESP_OK) {
        ESP_LOGW(TAG, "Failed to prepare Lua result workspace buffers");
        return false;
    }

    char *line = s_lua_result_line_buf;
    size_t line_capacity = s_lua_result_line_buf_capacity;

    lua_getfield(L, entry_idx, "value");
    if (lua_isboolean(L, -1)) {
        int v = lua_toboolean(L, -1) ? 1 : 0;
        (void)snprintf(line,
                       line_capacity,
                       "{\"o\":%d,\"i\":%d,\"r\":%d,\"v\":%d}",
                       obj,
                       inst,
                       res,
                       v);
    } else if (lua_isnumber(L, -1)) {
        double val = lua_tonumber(L, -1);
        sample_update_latest_sensor_value(obj, res, val);
        (void)snprintf(line,
                       line_capacity,
                       "{\"o\":%d,\"i\":%d,\"r\":%d,\"v\":%.6f}",
                       obj,
                       inst,
                       res,
                       val);
    } else if (lua_isstring(L, -1)) {
        size_t raw_len = 0;
        const char *raw = lua_tolstring(L, -1, &raw_len);
        if (raw && raw_len > 0) {
            char *end = NULL;
            double val = strtod(raw, &end);
            if (end && end != raw) {
                while (*end != '\0' && isspace((unsigned char)*end)) {
                    end++;
                }
                if (*end == '\0') {
                    sample_update_latest_sensor_value(obj, res, val);
                }
            }
        }
        char *escaped = s_lua_result_escaped_buf;
        escaped[0] = '\0';
        if (raw && raw_len > 0) {
            lua_json_escape_string(raw, escaped, s_lua_result_escaped_buf_capacity);
        }
        (void)snprintf(line,
                       line_capacity,
                       "{\"o\":%d,\"i\":%d,\"r\":%d,\"v\":\"%s\"}",
                       obj,
                       inst,
                       res,
                       escaped);
    } else {
        lua_pop(L, 1);
        return false;
    }
    lua_pop(L, 1);

    size_t line_len = strnlen(line, line_capacity);
    if (line_len == 0 || line_len > SAMPLE_DATA_FIELD_LINE_MAX_LEN) {
        return false;
    }

    sample_ring_push_payload(line, line_len);
    return true;
}

static size_t lua_enqueue_result_table_lines(lua_State *L, int table_idx)
{
    if (!lua_istable(L, table_idx)) {
        return 0;
    }

    if (table_idx < 0) {
        table_idx = lua_gettop(L) + table_idx + 1;
    }

    size_t enqueued = 0;
    if (lua_table_has_result_fields(L, table_idx)) {
        if (lua_enqueue_result_record_line(L, table_idx)) {
            enqueued++;
        }
        return enqueued;
    }

    lua_pushnil(L);
    while (lua_next(L, table_idx) != 0) {
        if (lua_istable(L, -1) && lua_table_has_result_fields(L, -1)) {
            if (lua_enqueue_result_record_line(L, -1)) {
                enqueued++;
            }
        }
        lua_pop(L, 1);
    }

    return enqueued;
}

/* ------------ Serialize Lua result table to JSON ------------ */
static bool lua_serialize_result_table(lua_State *L, int table_idx, char *out, size_t out_size)
{
    if (!lua_istable(L, table_idx)) return false;

    /* Convert to absolute index before pushing more values */
    if (table_idx < 0) table_idx = lua_gettop(L) + table_idx + 1;

    size_t written = 0;
    written += (size_t)snprintf(out + written, out_size - written, "[");

    bool first = true;
    size_t record_count = 0;

    if (lua_table_has_result_fields(L, table_idx)) {
        written = lua_append_serialized_result_record(L, table_idx, out, out_size, written, &first);
        record_count = 1;
        ESP_LOGI(TAG, "Lua serialize path=direct_record");
    } else {
        ESP_LOGI(TAG, "Lua serialize path=nested_records_scan");
        lua_pushnil(L);
        while (lua_next(L, table_idx) != 0) {
            if (lua_istable(L, -1)) {
                if (lua_table_has_result_fields(L, -1)) {
                    record_count++;
                }
                written = lua_append_serialized_result_record(L, -1, out, out_size, written, &first);
            }
            lua_pop(L, 1); /* remove value, keep key for lua_next */
        }
    }

    if (written < out_size - 1) {
        snprintf(out + written, out_size - written, "]");
    }

    ESP_LOGI(TAG,
             "Lua serialize summary: record_count=%u output_len=%u output=%s",
             (unsigned)record_count,
             (unsigned)strnlen(out, out_size),
             out);
    return true;
}

static void lua_log_result_table_entries(lua_State *L, int table_idx)
{
    if (!lua_istable(L, table_idx)) {
        return;
    }

    if (table_idx < 0) {
        table_idx = lua_gettop(L) + table_idx + 1;
    }

    int entry_index = 0;
    if (lua_table_has_result_fields(L, table_idx)) {
        ESP_LOGI(TAG, "Lua log results path=direct_record");
        lua_log_result_record(L, table_idx, entry_index);
        return;
    }

    ESP_LOGI(TAG, "Lua log results path=nested_records_scan");

    lua_pushnil(L);
    while (lua_next(L, table_idx) != 0) {
        if (lua_istable(L, -1)) {
            lua_log_result_record(L, -1, entry_index);
            entry_index++;
        }
        lua_pop(L, 1);
    }
}

esp_err_t sample_execute_script_and_log_results(const uint8_t *script,
                                                size_t script_len,
                                                char *out_payload,
                                                size_t out_payload_size)
{
    if (!script || script_len == 0 || !out_payload || out_payload_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    build_default_sample_payload(out_payload, out_payload_size, "ok");
    s_sample_script_run_counter++;

    if (!s_script_execute_input_logged_once) {
        print_lua_script_dump_from_buffer(script, script_len, "execute_input_boot");
        s_script_execute_input_logged_once = true;
    }

    lua_State *lua_state = luaL_newstate();
    if (!lua_state) {
        ESP_LOGE(TAG, "Failed to create Lua state");
        build_default_sample_payload(out_payload, out_payload_size, "lua_state_error");
        return ESP_FAIL;
    }

    luaL_openlibs(lua_state);
    lua_register(lua_state, "uart_read_frame", lua_uart_read_frame);
    lua_register(lua_state, "uart_connect", lua_uart_connect);
    lua_register(lua_state, "uart_safe_close", lua_uart_safe_close);
    lua_register(lua_state, "uart_reset_rx_cursor", lua_uart_reset_rx_cursor);
    lua_register(lua_state, "uart_set_rx_size", lua_uart_set_rx_size);
    lua_register(lua_state, "uart_write", lua_uart_write);
    lua_register(lua_state, "uart_read_chunk", lua_uart_read_chunk);
    lua_register(lua_state, "uart_sleep", lua_uart_sleep);
    lua_register(lua_state, "parse_pm25", lua_parse_pm25);

    lua_register(lua_state, "util_bytes_to_hex", lua_util_bytes_to_hex);
    lua_register(lua_state, "util_build_read_holding_request", lua_util_build_read_holding_request);
    lua_register(lua_state, "util_extract_modbus_frame", lua_util_extract_modbus_frame);
    lua_register(lua_state, "util_decode_bcd_32", lua_util_decode_bcd_32);
    lua_register(lua_state, "util_log", lua_util_log);
    lua_register(lua_state, "util_get_sample_counter", lua_util_get_sample_counter);
    lua_register(lua_state, "util_get_server_sec_of_year", lua_util_get_server_sec_of_year);
    lua_register(lua_state, "util_get_day_of_week", lua_util_get_day_of_week);
    lua_register(lua_state, "util_get_hour_of_day", lua_util_get_hour_of_day);
    lua_register(lua_state, "util_get_min_of_day", lua_util_get_min_of_day);
    lua_register(lua_state, "day_of_week", lua_util_get_day_of_week);
    lua_register(lua_state, "hour_of_day", lua_util_get_hour_of_day);
    lua_register(lua_state, "min_of_day", lua_util_get_min_of_day);
    lua_register(lua_state, "util_get_sample_runtime", lua_util_get_sample_runtime);
    lua_register(lua_state, "util_init_global_buffer", lua_util_init_global_buffer);
    lua_register(lua_state, "util_append_global_buffer", lua_util_append_global_buffer);
    lua_register(lua_state, "util_write_global_buffer_at", lua_util_write_global_buffer_at);
    lua_register(lua_state, "rs485_connect", lua_rs485_connect);
    lua_register(lua_state, "rs485_safe_close", lua_rs485_safe_close);
    lua_register(lua_state, "rs485_reset_rx_cursor", lua_rs485_reset_rx_cursor);
    lua_register(lua_state, "rs485_write", lua_rs485_write);
    lua_register(lua_state, "rs485_read_chunk", lua_rs485_read_chunk);
    lua_register(lua_state, "rs485_sleep", lua_rs485_sleep);

    lua_register(lua_state, "util_crc8", lua_util_crc8);
    lua_register(lua_state, "i2c_connect", lua_i2c_connect);
    lua_register(lua_state, "i2c_safe_close", lua_i2c_safe_close);
    lua_register(lua_state, "i2c_reset_rx_cursor", lua_i2c_reset_rx_cursor);
    lua_register(lua_state, "i2c_set_rx_size", lua_i2c_set_rx_size);
    lua_register(lua_state, "i2c_write", lua_i2c_write);
    lua_register(lua_state, "i2c_read_chunk", lua_i2c_read_chunk);
    lua_register(lua_state, "i2c_sleep", lua_i2c_sleep);

    int load_rc = luaL_loadbuffer(lua_state,
                                  (const char *)script,
                                  script_len,
                                  "sample_script");
    if (load_rc != LUA_OK) {
        const char *msg = lua_tostring(lua_state, -1);
        ESP_LOGE(TAG, "Lua load error: %s", msg ? msg : "unknown");
        build_default_sample_payload(out_payload, out_payload_size, "lua_load_error");
        if (s_script_error_print_budget > 0) {
            print_lua_script_dump("load_error");
            s_script_error_print_budget--;
        }
        lua_pop(lua_state, 1);
        lua_close(lua_state);
        return ESP_FAIL;
    }

    int exec_rc = lua_pcall(lua_state, 0, LUA_MULTRET, 0);
    if (exec_rc != LUA_OK) {
        const char *msg = lua_tostring(lua_state, -1);
        ESP_LOGE(TAG, "Lua runtime error: %s", msg ? msg : "unknown");
        build_default_sample_payload(out_payload, out_payload_size, "lua_runtime_error");
        if (s_script_error_print_budget > 0) {
            print_lua_script_dump("runtime_error");
            s_script_error_print_budget--;
        }
        lua_pop(lua_state, 1);
        lua_close(lua_state);
        return ESP_FAIL;
    }

    int result_count = lua_gettop(lua_state);
    lua_log_stack_snapshot(lua_state, "after_pcall");
    if (result_count > 0) {
        if (lua_istable(lua_state, -1)) {
            lua_log_table_shape(lua_state, -1, "script_return_top");
            lua_log_result_table_entries(lua_state, -1);
            size_t queued_lines = lua_enqueue_result_table_lines(lua_state, -1);
            if (queued_lines > 0) {
                ESP_LOGI(TAG,
                         "Lua result report lines queued: %u",
                         (unsigned)queued_lines);
            }

            if (!lua_serialize_result_table(lua_state, -1, out_payload, out_payload_size)) {
                out_payload[0] = '\0';
                ESP_LOGW(TAG,
                         "Lua table result serialization failed; output payload cleared");
            } else {
                ESP_LOGI(TAG,
                         "Lua table result serialized for /31025/*/10 while per-record SenML lines remain enabled");
            }
        } else {
            size_t lua_text_len = 0;
            const char *lua_text = luaL_tolstring(lua_state, -1, &lua_text_len);
            if (lua_text && lua_text_len > 0) {
                size_t copy_len = lua_text_len;
                if (copy_len > out_payload_size - 1) {
                    copy_len = out_payload_size - 1;
                }
                memset(out_payload, 0, out_payload_size);
                memcpy(out_payload, lua_text, copy_len);
                out_payload[copy_len] = '\0';
                ESP_LOGI(TAG,
                         "Lua non-table return copied to output: type=%s len=%u",
                         luaL_typename(lua_state, -2),
                         (unsigned)copy_len);
            }
            lua_pop(lua_state, 1);
        }
    } else {
        ESP_LOGW(TAG, "Lua script returned no values");
    }

    ESP_LOGI(TAG, "Lua script executed successfully");
    lua_close(lua_state);
    return ESP_OK;
}

static esp_err_t sample_script_cache_set_capacity(size_t capacity)
{
    if (capacity == 0 || capacity > SAMPLE_SCRIPT_CACHE_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_sample_script_buf != NULL && g_sample_script_buf_capacity == capacity) {
        if (g_sample_script_len > capacity) {
            g_sample_script_len = (uint32_t)capacity;
        }
        return ESP_OK;
    }

    uint8_t *next = NULL;
    bool using_psram = false;
    if (g_sample_script_buf == NULL) {
        next = (uint8_t *)heap_caps_malloc(capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (next != NULL) {
            using_psram = true;
        }
        if (next == NULL) {
            next = (uint8_t *)heap_caps_malloc(capacity, MALLOC_CAP_8BIT);
        }
    } else {
        next = (uint8_t *)heap_caps_realloc(g_sample_script_buf,
                                            capacity,
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (next != NULL) {
            using_psram = true;
        }
        if (next == NULL) {
            next = (uint8_t *)heap_caps_realloc(g_sample_script_buf,
                                                capacity,
                                                MALLOC_CAP_8BIT);
        }
    }

    if (!next) {
        return ESP_ERR_NO_MEM;
    }

    if (!using_psram) {
        ESP_LOGW(TAG,
                 "sample script cache using internal RAM fallback: capacity=%u",
                 (unsigned)capacity);
    } else {
        ESP_LOGI(TAG,
                 "sample script cache allocated in PSRAM: capacity=%u",
                 (unsigned)capacity);
    }

    g_sample_script_buf = next;
    g_sample_script_buf_capacity = capacity;
    if (g_sample_script_len > capacity) {
        g_sample_script_len = (uint32_t)capacity;
    }
    return ESP_OK;
}

static esp_err_t sample_script_cache_ensure_capacity(size_t min_capacity)
{
    if (min_capacity == 0) {
        min_capacity = SAMPLE_SCRIPT_CACHE_DEFAULT_SIZE;
    }

    if (min_capacity > SAMPLE_SCRIPT_CACHE_MAX_SIZE) {
        return ESP_ERR_NO_MEM;
    }

    if (g_sample_script_buf != NULL && g_sample_script_buf_capacity >= min_capacity) {
        return ESP_OK;
    }

    size_t target = g_sample_script_buf_capacity;
    if (target == 0) {
        target = SAMPLE_SCRIPT_CACHE_DEFAULT_SIZE;
    }
    while (target < min_capacity) {
        if (target >= SAMPLE_SCRIPT_CACHE_MAX_SIZE) {
            target = SAMPLE_SCRIPT_CACHE_MAX_SIZE;
            break;
        }
        size_t next = target * 2;
        if (next < target || next > SAMPLE_SCRIPT_CACHE_MAX_SIZE) {
            next = SAMPLE_SCRIPT_CACHE_MAX_SIZE;
        }
        target = next;
    }

    return sample_script_cache_set_capacity(target);
}

RTC_DATA_ATTR uint32_t g_sample_config_version = 1;
RTC_DATA_ATTR bool g_sample_sampling_enabled = true;
RTC_DATA_ATTR uint8_t g_sample_sleep_mode = SAMPLE_SLEEP_MODE_NO;
RTC_DATA_ATTR uint32_t g_sample_rate = 60;
RTC_DATA_ATTR uint32_t g_sample_report_rate = 300;
RTC_DATA_ATTR uint32_t g_sample_send_ack_timeout_ms = CONFIG_SAMPLE_SEND_ACK_TIMEOUT_MS;
RTC_DATA_ATTR uint32_t g_sample_send_retry_delay_ms = CONFIG_SAMPLE_SEND_RETRY_DELAY_MS;
RTC_DATA_ATTR uint32_t g_sample_send_retry_count = SAMPLE_SEND_MAX_ATTEMPTS;
RTC_DATA_ATTR char g_sample_sensor_uart_i2c[32] = {0};
RTC_DATA_ATTR char g_sample_sensor_rs485[32] = {0};
uint32_t g_sample_script_len = 0;
bool g_sample_script_truncated = false;
uint8_t *g_sample_script_buf = NULL;
size_t g_sample_script_buf_capacity = 0;
RTC_DATA_ATTR bool s_sample_rtc_cache_ready = false;
RTC_DATA_ATTR uint64_t s_next_sample_due_us = 0;

typedef struct {
    uint32_t len;
    uint8_t *payload;
} sample_ring_record_t;

static sample_ring_record_t s_sample_ring[SAMPLE_RING_MAX_RECORDS] = {0};
static uint8_t s_sample_ring_head = 0;
static uint8_t s_sample_ring_tail = 0;
static uint8_t s_sample_ring_count = 0;

static esp_err_t sample_ring_ensure_slot_storage(uint8_t slot_index)
{
    if (slot_index >= SAMPLE_RING_MAX_RECORDS) {
        return ESP_ERR_INVALID_ARG;
    }

    sample_ring_record_t *slot = &s_sample_ring[slot_index];
    if (slot->payload) {
        return ESP_OK;
    }

    slot->payload = (uint8_t *)heap_caps_malloc(SAMPLE_RING_RECORD_MAX_LEN,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!slot->payload) {
        ESP_LOGE(TAG,
                 "sample ring PSRAM allocation failed slot=%u size=%u",
                 (unsigned)slot_index,
                 (unsigned)SAMPLE_RING_RECORD_MAX_LEN);
        return ESP_ERR_NO_MEM;
    }

    memset(slot->payload, 0, SAMPLE_RING_RECORD_MAX_LEN);
    return ESP_OK;
}

static void sample_ring_init_if_needed(void)
{
    portENTER_CRITICAL(&s_sample_ring_lock);
    if (s_sample_ring_head >= SAMPLE_RING_MAX_RECORDS ||
        s_sample_ring_tail >= SAMPLE_RING_MAX_RECORDS ||
        s_sample_ring_count > SAMPLE_RING_MAX_RECORDS) {
        s_sample_ring_head = 0;
        s_sample_ring_tail = 0;
        s_sample_ring_count = 0;
        for (size_t i = 0; i < SAMPLE_RING_MAX_RECORDS; i++) {
            s_sample_ring[i].len = 0;
        }
    }
    portEXIT_CRITICAL(&s_sample_ring_lock);
}

static void sample_ring_push_payload(const char *payload, size_t len)
{
    if (!payload || len == 0) {
        return;
    }

#ifdef CONFIG_ENABLE_MM_HALOW
    if (halow_interface_app_beacon_only()) {
        if (!s_beacon_report_cache_skip_logged) {
            ESP_LOGI(TAG,
                     "Beacon-only mode retains latest SensorData; report ring disabled");
            s_beacon_report_cache_skip_logged = true;
        }
        return;
    }
#endif

    size_t original_len = len;
    bool line_truncated = false;
    bool dropped_oldest = false;

    if (len > SAMPLE_DATA_FIELD_LINE_MAX_LEN) {
        line_truncated = true;
        len = SAMPLE_DATA_FIELD_LINE_MAX_LEN;
    }

    if (len > SAMPLE_RING_RECORD_MAX_LEN) {
        len = SAMPLE_RING_RECORD_MAX_LEN;
    }

    while (true) {
        uint8_t alloc_slot = 0;
        bool need_alloc = false;

        portENTER_CRITICAL(&s_sample_ring_lock);

        if (s_sample_ring_head >= SAMPLE_RING_MAX_RECORDS ||
            s_sample_ring_tail >= SAMPLE_RING_MAX_RECORDS ||
            s_sample_ring_count > SAMPLE_RING_MAX_RECORDS) {
            s_sample_ring_head = 0;
            s_sample_ring_tail = 0;
            s_sample_ring_count = 0;
            for (size_t i = 0; i < SAMPLE_RING_MAX_RECORDS; i++) {
                s_sample_ring[i].len = 0;
            }
        }

        if (s_sample_ring_count == SAMPLE_RING_MAX_RECORDS) {
            s_sample_ring_head = (uint8_t)((s_sample_ring_head + 1) % SAMPLE_RING_MAX_RECORDS);
            s_sample_ring_count--;
            dropped_oldest = true;
        }

        alloc_slot = s_sample_ring_tail;
        sample_ring_record_t *slot = &s_sample_ring[alloc_slot];
        if (!slot->payload) {
            need_alloc = true;
            portEXIT_CRITICAL(&s_sample_ring_lock);
        } else {
            memcpy(slot->payload, payload, len);
            slot->len = (uint32_t)len;

            s_sample_ring_tail = (uint8_t)((s_sample_ring_tail + 1) % SAMPLE_RING_MAX_RECORDS);
            s_sample_ring_count++;

            portEXIT_CRITICAL(&s_sample_ring_lock);
            break;
        }

        if (need_alloc) {
            if (sample_ring_ensure_slot_storage(alloc_slot) != ESP_OK) {
                ESP_LOGW(TAG,
                         "Failed to allocate sample ring slot storage (idx=%u, payload_len=%u)",
                         (unsigned)alloc_slot,
                         (unsigned)len);
                return;
            }
        }
    }

    if (line_truncated) {
        ESP_LOGW(TAG,
                 "Sample report line exceeds transport limit (%u > %u), truncating",
                 (unsigned)original_len,
                 (unsigned)SAMPLE_DATA_FIELD_LINE_MAX_LEN);
    }

    if (dropped_oldest) {
        ESP_LOGW(TAG, "sample ring full, dropping oldest record");
    }
}

static void sample_ring_pop(void)
{
    portENTER_CRITICAL(&s_sample_ring_lock);
    if (s_sample_ring_count == 0) {
        portEXIT_CRITICAL(&s_sample_ring_lock);
        return;
    }

    sample_ring_record_t *head = &s_sample_ring[s_sample_ring_head];
    if (head->payload && head->len > 0) {
        size_t clear_len = head->len;
        if (clear_len > SAMPLE_RING_RECORD_MAX_LEN) {
            clear_len = SAMPLE_RING_RECORD_MAX_LEN;
        }
        memset(head->payload, 0, clear_len);
    }
    head->len = 0;
    s_sample_ring_head = (uint8_t)((s_sample_ring_head + 1) % SAMPLE_RING_MAX_RECORDS);
    s_sample_ring_count--;
    portEXIT_CRITICAL(&s_sample_ring_lock);
}

static uint8_t sample_ring_drop_all(void)
{
    sample_ring_init_if_needed();

    portENTER_CRITICAL(&s_sample_ring_lock);
    uint8_t dropped = s_sample_ring_count;
    portEXIT_CRITICAL(&s_sample_ring_lock);

    for (uint8_t i = 0; i < dropped; i++) {
        sample_ring_pop();
    }

    return dropped;
}

static void build_default_sample_payload(char *out, size_t out_size, const char *status)
{
    if (!out || out_size == 0) {
        return;
    }

    uint64_t now_us = (uint64_t)esp_timer_get_time();
    snprintf(out,
             out_size,
             "{\"ts_us\":%llu,\"status\":\"%s\"}",
             (unsigned long long)now_us,
             status ? status : "ok");
}

static void sample_mark_finish_uptime(void)
{
#ifdef ESP_PLATFORM
    s_last_sample_finish_uptime_ms = (int64_t)esp_log_timestamp();
#else
    {
        time_t now_sec = time(NULL);
        if (now_sec >= 0) {
            s_last_sample_finish_uptime_ms = ((int64_t)now_sec) * 1000;
        }
    }
#endif
}

static bool flush_sample_ring_with_ack(void)
{
    sample_ring_init_if_needed();

    uint8_t dropped = sample_ring_drop_all();
    if (dropped > 0) {
        ESP_LOGI(TAG,
                 "Local sampling completed; retained latest SensorData for beacon and discarded %u cloud-report copy/copies",
                 (unsigned)dropped);
    }
    return true;
}

static uint64_t interval_sample_us(void)
{
#ifdef CONFIG_ENABLE_MM_HALOW
    return (uint64_t)halow_sync_bridge_beacon_interval_seconds() * 1000000ULL;
#else
    uint32_t sec = g_sample_rate > 0 ? g_sample_rate : 60;
    return (uint64_t)sec * 1000000ULL;
#endif
}

static void init_due_times_if_needed(void)
{
    uint64_t now = (uint64_t)esp_timer_get_time();
    if (s_next_sample_due_us == 0) s_next_sample_due_us = now;
}

static bool sample_is_provisioning_successful(void)
{
    return true;
}

static bool sample_sampling_gate_allows_run(uint64_t now_us)
{
    if (ble_control_is_connected()) {
        if (now_us - s_sampling_gate_last_log_us >= 5000000ULL) {
            printf("[sampling] BLE editor connected; skip firmware sampling\n");
            s_sampling_gate_last_log_us = now_us;
        }
        return false;
    }

#ifdef CONFIG_ENABLE_MM_HALOW
    if (!halow_interface_app_ready()) {
        if (now_us - s_sampling_gate_last_log_us >= 5000000ULL) {
            printf("[sampling] HaLow interface not ready; sampling remains stopped\n");
            s_sampling_gate_last_log_us = now_us;
        }
        return false;
    }
#endif

    if (sample_is_provisioning_successful()) {
        return true;
    }

    if (now_us - s_sampling_gate_last_log_us >= 5000000ULL) {
        printf("[sampling] Provisioning not successful yet; skip sampling until connection is ready\n");
        s_sampling_gate_last_log_us = now_us;
    }

    return false;
}

static bool trigger_report_after_sample(void)
{
    return flush_sample_ring_with_ack();
}

static bool sample_due_and_run(uint64_t now)
{
    if (now < s_next_sample_due_us) return false;

    if (!sample_sampling_gate_allows_run(now)) {
        uint64_t step = interval_sample_us();
        while (s_next_sample_due_us <= now) s_next_sample_due_us += step;
        return false;
    }

    sample_handle_sampling();

    bool report_ok = trigger_report_after_sample();
    if (!report_ok) {
        printf("[sampling] Sample report deferred (will retry after next sample)\n");
    }

    uint64_t now_after_work = (uint64_t)esp_timer_get_time();
    uint64_t step = interval_sample_us();
    while (s_next_sample_due_us <= now_after_work) s_next_sample_due_us += step;
    return true;
}

static bool work_in_progress(void)
{
    return s_sample_in_progress;
}

static void recover_network_for_pending_sample(uint64_t now)
{
    if (now < s_next_sample_due_us) {
        return;
    }

#ifdef CONFIG_ENABLE_MM_HALOW
    esp_err_t wake_recover_err = wifi_prov_exit_halow_light_sleep();
    if (wake_recover_err != ESP_OK) {
        printf("[sampling] HaLow wake recovery failed: %s\n", esp_err_to_name(wake_recover_err));
        return;
    }
#endif

}

static bool wait_halow_ready_before_light_sleep(void)
{
#ifdef CONFIG_ENABLE_MM_HALOW
    const TickType_t wait_step = pdMS_TO_TICKS(CONFIG_SAMPLE_SLEEP_IDLE_WAIT_STEP_MS);
    const TickType_t wait_timeout = pdMS_TO_TICKS(CONFIG_SAMPLE_HALOW_SLEEP_READY_TIMEOUT_MS);
    TickType_t waited = 0;
    bool logged = false;

    while (!wifi_prov_halow_ready_for_light_sleep() && waited < wait_timeout) {
        if (!logged) {
            printf("[sampling] Waiting for HaLow reconnect to settle before sleep\n");
            logged = true;
        }
        vTaskDelay(wait_step);
        waited += wait_step;
    }

    if (!wifi_prov_halow_ready_for_light_sleep()) {
        printf("[sampling] HaLow still reconnecting after %d ms, skip sleep this cycle\n",
               CONFIG_SAMPLE_HALOW_SLEEP_READY_TIMEOUT_MS);
        return false;
    }
#endif

    return true;
}

static void sample_task_loop(void *arg)
{
    (void)arg;
    init_due_times_if_needed();

    while (true) {
        uint64_t now = (uint64_t)esp_timer_get_time();
        bool did_work = sample_due_and_run(now);
        if (did_work) {
            UBaseType_t stack_words = uxTaskGetStackHighWaterMark(NULL);
            if (stack_words < CONFIG_SAMPLE_TASK_STACK_WATERMARK_WARN_WORDS &&
                s_sample_task_stack_warn_budget > 0) {
                ESP_LOGW(TAG,
                         "sample_task low stack watermark: %u words (~%u bytes) remaining",
                         (unsigned)stack_words,
                         (unsigned)(stack_words * sizeof(StackType_t)));
                s_sample_task_stack_warn_budget--;
            }
            continue;
        }

        uint64_t wait_us = (s_next_sample_due_us > now) ? (s_next_sample_due_us - now) : 1000ULL;
        if (wait_us < 1000ULL) wait_us = 1000ULL;
        uint32_t delay_ms = (uint32_t)((wait_us + 999ULL) / 1000ULL);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

static bool is_valid_sleep_mode(sample_sleep_mode_t mode)
{
    return mode == SAMPLE_SLEEP_MODE_NO || mode == SAMPLE_SLEEP_MODE_LIGHT || mode == SAMPLE_SLEEP_MODE_DEEP;
}

const char *sample_sleep_mode_to_string(sample_sleep_mode_t mode)
{
    switch (mode) {
        case SAMPLE_SLEEP_MODE_NO:
            return "no";
        case SAMPLE_SLEEP_MODE_LIGHT:
            return "light";
        case SAMPLE_SLEEP_MODE_DEEP:
            return "deep";
        default:
            return "unknown";
    }
}

static bool parse_legacy_sleep_string(const char *text, sample_sleep_mode_t *out_mode)
{
    if (!text || !out_mode) return false;

    if (strcmp(text, "no") == 0) {
        *out_mode = SAMPLE_SLEEP_MODE_NO;
        return true;
    }
    if (strcmp(text, "yes") == 0 || strcmp(text, "light") == 0) {
        *out_mode = SAMPLE_SLEEP_MODE_LIGHT;
        return true;
    }
    if (strcmp(text, "deep") == 0) {
        *out_mode = SAMPLE_SLEEP_MODE_DEEP;
        return true;
    }
    return false;
}

void sample_settings_load_from_nvs_to_rtc(void)
{
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(SAMPLE_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        if (err != ESP_ERR_NVS_NOT_FOUND) {
            printf("[sampling] Failed to open sample NVS namespace: %s\n", esp_err_to_name(err));
        }
        printf("[sampling] sample NVS missing/unavailable, using RTC/default config and still refreshing script cache\n");
        goto finalize;
    }

    uint32_t val = 0;
    if (nvs_get_u32(nvs, SAMPLE_NVS_KEY_VER, &val) == ESP_OK && val > 0) {
        g_sample_config_version = val;
    }

    uint8_t en = 0;
    if (nvs_get_u8(nvs, SAMPLE_NVS_KEY_EN, &en) == ESP_OK) {
        g_sample_sampling_enabled = (en != 0);
    }

    uint8_t sleep_mode = 0;
    if (nvs_get_u8(nvs, SAMPLE_NVS_KEY_SLEEP_MODE, &sleep_mode) == ESP_OK) {
        if (is_valid_sleep_mode((sample_sleep_mode_t)sleep_mode)) {
            g_sample_sleep_mode = sleep_mode;
        }
    } else {
        char sleep_buf[8] = {0};
        size_t sleep_len = sizeof(sleep_buf);
        if (nvs_get_str(nvs, SAMPLE_NVS_KEY_SLEEP, sleep_buf, &sleep_len) == ESP_OK) {
            sample_sleep_mode_t parsed = SAMPLE_SLEEP_MODE_NO;
            if (parse_legacy_sleep_string(sleep_buf, &parsed)) {
                g_sample_sleep_mode = (uint8_t)parsed;
            }
        }
    }

    if (nvs_get_u32(nvs, SAMPLE_NVS_KEY_S_RATE, &val) == ESP_OK && val > 0) {
        g_sample_rate = val;
    }

    if (nvs_get_u32(nvs, SAMPLE_NVS_KEY_R_RATE, &val) == ESP_OK && val > 0) {
        g_sample_report_rate = val;
    }

    if (nvs_get_u32(nvs, SAMPLE_NVS_KEY_SEND_ACK_TO, &val) == ESP_OK && val > 0) {
        g_sample_send_ack_timeout_ms = val;
    }

    if (nvs_get_u32(nvs, SAMPLE_NVS_KEY_SEND_RETRY_DELAY, &val) == ESP_OK && val > 0) {
        g_sample_send_retry_delay_ms = val;
    }

    if (nvs_get_u32(nvs, SAMPLE_NVS_KEY_SEND_RETRY_CNT, &val) == ESP_OK && val > 0) {
        g_sample_send_retry_count = val;
    }

    size_t u2c_len = sizeof(g_sample_sensor_uart_i2c);
    if (nvs_get_str(nvs, SAMPLE_NVS_KEY_S_U2C, g_sample_sensor_uart_i2c, &u2c_len) != ESP_OK) {
        g_sample_sensor_uart_i2c[0] = '\0';
    }

    size_t rs_len = sizeof(g_sample_sensor_rs485);
    if (nvs_get_str(nvs, SAMPLE_NVS_KEY_S_RS, g_sample_sensor_rs485, &rs_len) != ESP_OK) {
        g_sample_sensor_rs485[0] = '\0';
    }

    g_sample_script_len = 0;
    g_sample_script_truncated = false;
    if (g_sample_script_buf && g_sample_script_buf_capacity > 0) {
        memset(g_sample_script_buf, 0, g_sample_script_buf_capacity);
    }

    nvs_close(nvs);

    finalize:
        printf("[sampling] Refreshing aggregated script cache from selector ids at bootstrap/config-load\n");
    sample_refresh_script_cache_from_selectors();
        printf("[sampling] Aggregated script cache refresh finished: script_len=%lu truncated=%d\n",
            (unsigned long)g_sample_script_len,
            g_sample_script_truncated ? 1 : 0);

    s_sample_rtc_cache_ready = true;
    sample_ring_init_if_needed();

        printf("[sampling] Sample RTC cache: ver=%lu en=%d sleep=%u(%s) s_rate=%lu r_rate=%lu(legacy_ignored) ack_to=%lu retry_delay=%lu retry_cnt=%lu script_len=%lu truncated=%d\n",
            (unsigned long)g_sample_config_version,
            g_sample_sampling_enabled ? 1 : 0,
            (unsigned)g_sample_sleep_mode,
            sample_sleep_mode_to_string((sample_sleep_mode_t)g_sample_sleep_mode),
            (unsigned long)g_sample_rate,
            (unsigned long)g_sample_report_rate,
            (unsigned long)g_sample_send_ack_timeout_ms,
            (unsigned long)g_sample_send_retry_delay_ms,
            (unsigned long)g_sample_send_retry_count,
            (unsigned long)g_sample_script_len,
            g_sample_script_truncated ? 1 : 0);
}

esp_err_t sample_set_sensor_selectors(const char *uart_i2c, const char *rs485)
{
    const char *u2c = uart_i2c ? uart_i2c : "";
    const char *rs = rs485 ? rs485 : "";
    if (strnlen(u2c, sizeof(g_sample_sensor_uart_i2c)) >= sizeof(g_sample_sensor_uart_i2c) ||
        strnlen(rs, sizeof(g_sample_sensor_rs485)) >= sizeof(g_sample_sensor_rs485)) {
        return ESP_ERR_INVALID_SIZE;
    }

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(SAMPLE_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_str(nvs, SAMPLE_NVS_KEY_S_U2C, u2c);
    if (err == ESP_OK) err = nvs_set_str(nvs, SAMPLE_NVS_KEY_S_RS, rs);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err != ESP_OK) return err;

    strlcpy(g_sample_sensor_uart_i2c, u2c, sizeof(g_sample_sensor_uart_i2c));
    strlcpy(g_sample_sensor_rs485, rs, sizeof(g_sample_sensor_rs485));
    sample_refresh_script_cache_from_selectors();
    return ESP_OK;
}

bool sample_is_rtc_cache_ready(void)
{
    return s_sample_rtc_cache_ready;
}

bool sample_is_sampling_enabled(void)
{
    return g_sample_sampling_enabled;
}

void sample_set_startup_network_guard(bool enabled)
{
    s_startup_network_guard_enabled = enabled;
    s_startup_network_guard_logged = false;
    s_startup_network_guard_start_us = enabled ? (uint64_t)esp_timer_get_time() : 0;
}

bool sample_is_report_due_now(void)
{
    init_due_times_if_needed();
    uint64_t now = (uint64_t)esp_timer_get_time();
    return now >= s_next_sample_due_us;
}

bool sample_is_deep_report_due_now(void)
{
    return true;
}

sample_sleep_mode_t sample_get_sleep_mode(void)
{
    if (!is_valid_sleep_mode((sample_sleep_mode_t)g_sample_sleep_mode)) {
        return SAMPLE_SLEEP_MODE_NO;
    }
    return (sample_sleep_mode_t)g_sample_sleep_mode;
}

static bool sample_has_sensor_script_selection(void)
{
    return !sample_sensor_selector_is_empty(g_sample_sensor_uart_i2c) ||
           !sample_sensor_selector_is_empty(g_sample_sensor_rs485);
}

static void sample_close_lua_sensor_interfaces(void)
{
    (void)lua_i2c_safe_close(NULL);
    (void)lua_rs485_safe_close(NULL);
    (void)lua_uart_safe_close(NULL);
}

static bool sample_run_lua_sensor_sampling(void)
{
    if (!sample_has_sensor_script_selection()) {
        ESP_LOGI(TAG, "Skipping Lua sensor sample: no sensor configured");
        return false;
    }

    if (!g_sample_script_buf || g_sample_script_len == 0) {
        ESP_LOGI(TAG, "Lua sensor script cache empty; refreshing from selected sensor ids");
        sample_refresh_script_cache_from_selectors();
    }

    if (g_sample_script_truncated) {
        ESP_LOGW(TAG,
                 "Skipping Lua sensor sample: aggregated script cache truncated len=%lu capacity=%u",
                 (unsigned long)g_sample_script_len,
                 (unsigned)g_sample_script_buf_capacity);
        return false;
    }

    if (!g_sample_script_buf || g_sample_script_len == 0) {
        ESP_LOGW(TAG,
                 "Skipping Lua sensor sample: no executable script for uart_i2c='%s' rs485='%s'",
                 sample_sensor_selector_is_empty(g_sample_sensor_uart_i2c) ? "none" : g_sample_sensor_uart_i2c,
                 sample_sensor_selector_is_empty(g_sample_sensor_rs485) ? "none" : g_sample_sensor_rs485);
        return false;
    }

    char script_payload[SAMPLE_SCRIPT_RESULT_PAYLOAD_MAX_LEN] = {0};
    if (!sample_script_global_buffer_mutex_ensure() ||
        xSemaphoreTake(s_script_global_buffer_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Skipping Lua sensor sample: global buffer lock unavailable");
        return false;
    }
    esp_err_t err = sample_execute_script_and_log_results(g_sample_script_buf,
                                                          g_sample_script_len,
                                                          script_payload,
                                                          sizeof(script_payload));
    if (err == ESP_OK) {
        esp_err_t publish_err = sample_script_global_buffer_publish_for_serving();
        if (publish_err != ESP_OK) {
            ESP_LOGW(TAG,
                     "Global buffer sample publish failed: %s",
                     esp_err_to_name(publish_err));
        }
    }
    xSemaphoreGive(s_script_global_buffer_mutex);
    sample_close_lua_sensor_interfaces();

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Lua sensor sample failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG,
             "Lua sensor sample ok: script_len=%lu payload_len=%u",
             (unsigned long)g_sample_script_len,
             (unsigned)strnlen(script_payload, sizeof(script_payload)));
    return true;
}

void sample_handle_sampling(void)
{
    s_sample_in_progress = true;
    if (s_script_global_buffer_capacity == 0 ||
        s_script_global_buffer == NULL ||
        s_script_serving_buffer == NULL) {
        esp_err_t buffer_err = sample_script_global_buffer_set_capacity(
            SAMPLE_SCRIPT_GLOBAL_BUFFER_DEFAULT_SIZE);
        if (buffer_err != ESP_OK) {
            ESP_LOGW(TAG,
                     "Skipping sensor sample: global PSRAM buffers unavailable: %s",
                     esp_err_to_name(buffer_err));
            s_sample_in_progress = false;
            return;
        }
    }
    if (s_startup_network_guard_enabled) {
        s_startup_network_guard_enabled = false;
    }

#ifdef CONFIG_ENABLE_MM_HALOW
    printf("[sampling] Sampling handler: version=%lu enabled=%d sampling_interval=%lus(source=beacon_interval) report_rate=unused sleep=%u(%s) uart_i2c_sensor=%s\n",
           (unsigned long)g_sample_config_version,
           g_sample_sampling_enabled ? 1 : 0,
           (unsigned long)halow_sync_bridge_beacon_interval_seconds(),
           (unsigned)sample_get_sleep_mode(),
           sample_sleep_mode_to_string(sample_get_sleep_mode()),
           sample_sensor_selector_is_empty(g_sample_sensor_uart_i2c) ? "none" : g_sample_sensor_uart_i2c);
#else
    printf("[sampling] Sampling handler: version=%lu enabled=%d sample_rate=%lus report_rate=%lus(legacy_ignored) sleep=%u(%s) uart_i2c_sensor=%s\n",
           (unsigned long)g_sample_config_version,
           g_sample_sampling_enabled ? 1 : 0,
           (unsigned long)g_sample_rate,
           (unsigned long)g_sample_report_rate,
           (unsigned)sample_get_sleep_mode(),
           sample_sleep_mode_to_string(sample_get_sleep_mode()),
           sample_sensor_selector_is_empty(g_sample_sensor_uart_i2c) ? "none" : g_sample_sensor_uart_i2c);
#endif

    sample_mark_finish_uptime();

    sample_begin_sensor_data_update();

    if (!sample_run_lua_sensor_sampling()) {
        sample_finish_sensor_data_update(false);
        s_sample_in_progress = false;
        return;
    }
    sample_finish_sensor_data_update(true);

    s_sample_in_progress = false;
}

static void sample_start_no_sleep_task_now(void)
{
    interface_bridge_register();
    if (xTaskCreateWithCaps(sample_task_loop,
                            "sample_task",
                            CONFIG_SAMPLE_TASK_STACK_SIZE,
                            NULL,
                            5,
                            NULL,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sampling task after HaLow mesh initialization");
    }
}

#ifdef CONFIG_ENABLE_MM_HALOW
static void sample_startup_launcher_task(void *arg)
{
    (void)arg;
    uint64_t last_log_us = 0;

    while (!halow_interface_app_ready()) {
        uint64_t now_us = (uint64_t)esp_timer_get_time();
        if (now_us - last_log_us >= 5000000ULL) {
            ESP_LOGI(TAG,
                     "Waiting for HaLow interface readiness before allocating sampling task");
            last_log_us = now_us;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    s_startup_network_guard_enabled = false;
    s_startup_network_guard_logged = false;
    ESP_LOGI(TAG, "HaLow interface ready; starting local sampling task");
    sample_start_no_sleep_task_now();
    vTaskDelete(NULL);
}
#endif

void sample_start_no_sleep_tasks(void)
{
#ifdef CONFIG_ENABLE_MM_HALOW
    if (s_startup_network_guard_enabled) {
        if (xTaskCreate(sample_startup_launcher_task,
                        "sample_wait_halow",
                        CONFIG_SAMPLE_STARTUP_LAUNCHER_STACK_SIZE,
                        NULL,
                        5,
                        NULL) != pdPASS) {
            ESP_LOGE(TAG,
                     "Failed to create HaLow sampling launcher; sampling remains stopped");
        }
        return;
    }
#endif
    sample_start_no_sleep_task_now();
}

void sample_run_light_sleep_mode_loop(void)
{
    init_due_times_if_needed();

    const gpio_config_t wake_button_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_SAMPLE_WAKE_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t wake_btn_cfg_err = gpio_config(&wake_button_cfg);
    if (wake_btn_cfg_err != ESP_OK) {
        ESP_LOGW(TAG,
                 "Failed to configure wake button GPIO%u for light sleep: %s",
                 (unsigned)CONFIG_SAMPLE_WAKE_BUTTON_GPIO,
                 esp_err_to_name(wake_btn_cfg_err));
    }

    while (true) {
        uint64_t now = (uint64_t)esp_timer_get_time();

        recover_network_for_pending_sample(now);

        bool did_work = false;
        did_work |= sample_due_and_run(now);

        if (did_work) continue;

        uint64_t next_due = s_next_sample_due_us;
        uint64_t sleep_us = (next_due > now) ? (next_due - now) : 1000000ULL;
        if (sleep_us < 100000ULL) sleep_us = 100000ULL;

        if (work_in_progress()) {
            vTaskDelay(pdMS_TO_TICKS(CONFIG_SAMPLE_SLEEP_IDLE_WAIT_STEP_MS));
            continue;
        }

        if (!wait_halow_ready_before_light_sleep()) {
            vTaskDelay(pdMS_TO_TICKS(CONFIG_SAMPLE_SLEEP_IDLE_WAIT_STEP_MS));
            continue;
        }

    #ifdef CONFIG_ENABLE_MM_HALOW
        wifi_prov_enter_halow_light_sleep();
    #endif

        esp_sleep_enable_timer_wakeup(sleep_us);
        printf("[sampling] Entering light sleep for %llu ms\n", (unsigned long long)(sleep_us / 1000ULL));
        uint64_t sleep_start_us = (uint64_t)esp_timer_get_time();
        esp_err_t sleep_err = esp_light_sleep_start();
        uint64_t slept_us = (uint64_t)esp_timer_get_time() - sleep_start_us;

        if (sleep_err != ESP_OK) {
            printf("[sampling] esp_light_sleep_start failed: %s\n", esp_err_to_name(sleep_err));
        } else {
            esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
            printf("[sampling] Woke from light sleep: slept=%llu ms requested=%llu ms cause=%d\n",
                   (unsigned long long)(slept_us / 1000ULL),
                   (unsigned long long)(sleep_us / 1000ULL),
                   (int)wakeup_cause);

            if (slept_us + 1000ULL < sleep_us) {
                printf("[sampling] Light sleep woke earlier than timer target\n");
            }
        }

        uint64_t now_after_wake = (uint64_t)esp_timer_get_time();
        if (now_after_wake >= s_next_sample_due_us) {
            recover_network_for_pending_sample(now_after_wake);
        } else {
            printf("[sampling] Wake before next sample due; skip network recovery\n");
        }
    }
}

static void sample_set_power_pins_low_for_sleep(void)
{
    esp_err_t halow_dir_err = gpio_set_direction(GPIO_NUM_48, GPIO_MODE_OUTPUT);
    if (halow_dir_err != ESP_OK) {
        ESP_LOGW(TAG,
                 "Failed configuring HaLow power GPIO48 as output before deep sleep: %s",
                 esp_err_to_name(halow_dir_err));
    }

    esp_err_t halow_power_err = gpio_set_level(GPIO_NUM_48, 0);
    if (halow_power_err != ESP_OK) {
        ESP_LOGW(TAG,
                 "Failed setting HaLow power GPIO48 low before deep sleep: %s",
                 esp_err_to_name(halow_power_err));
    }

    esp_err_t camera_dir_err = gpio_set_direction(GPIO_NUM_3, GPIO_MODE_OUTPUT);
    if (camera_dir_err != ESP_OK) {
        ESP_LOGW(TAG,
                 "Failed configuring camera power GPIO3 as output before deep sleep: %s",
                 esp_err_to_name(camera_dir_err));
    }

    esp_err_t camera_power_err = gpio_set_level(GPIO_NUM_3, 0);
    if (camera_power_err != ESP_OK) {
        ESP_LOGW(TAG,
                 "Failed setting camera power GPIO3 low before deep sleep: %s",
                 esp_err_to_name(camera_power_err));
    }
}

void sample_run_deep_sleep_mode_cycle(void)
{
    uint64_t cycle_start_us = (uint64_t)esp_timer_get_time();

    if (!sample_sampling_gate_allows_run(cycle_start_us)) {
        printf("[sampling] Deep sleep mode: provisioning not successful yet, skip sampling/report this cycle\n");

        uint64_t sample_interval_us = interval_sample_us();
        uint64_t sleep_us = (sample_interval_us < 100000ULL) ? 100000ULL : sample_interval_us;

        printf("[sampling] Deep sleep mode: sleeping for %llu ms while waiting provisioning success\n",
               (unsigned long long)(sleep_us / 1000ULL));
        esp_err_t prep_err = wifi_prov_prepare_for_deep_sleep();
        if (prep_err != ESP_OK) {
            printf("[sampling] Deep sleep prep warning: %s\n", esp_err_to_name(prep_err));
        }
        sample_set_power_pins_low_for_sleep();
        esp_sleep_enable_timer_wakeup(sleep_us);
        esp_deep_sleep_start();
    }

    sample_handle_sampling();

    bool report_ok = trigger_report_after_sample();
    if (!report_ok) {
        uint8_t dropped = sample_ring_drop_all();
        printf("[sampling] Deep sleep mode: local report failed, dropped %u buffered sample record(s)\n",
               (unsigned)dropped);
    }

    uint64_t now = (uint64_t)esp_timer_get_time();
    uint64_t cycle_elapsed_us = now - cycle_start_us;
    uint64_t sample_interval_us = interval_sample_us();
    uint64_t sleep_us = (cycle_elapsed_us < sample_interval_us) ? (sample_interval_us - cycle_elapsed_us) : 100000ULL;
    if (sleep_us < 100000ULL) sleep_us = 100000ULL;

    esp_err_t prep_err = wifi_prov_prepare_for_deep_sleep();
    if (prep_err != ESP_OK) {
        printf("[sampling] Deep sleep prep warning: %s\n", esp_err_to_name(prep_err));
    }

    sample_set_power_pins_low_for_sleep();
    printf("[sampling] Deep sleep mode: sleeping for %llu ms then full init on wake\n",
           (unsigned long long)(sleep_us / 1000ULL));
    esp_sleep_enable_timer_wakeup(sleep_us);
    esp_deep_sleep_start();
}
