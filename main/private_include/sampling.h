#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SAMPLE_SCRIPT_RTC_MAX_SIZE 4096
#define SAMPLE_SCRIPT_RESULT_PAYLOAD_MAX_LEN 1024

typedef enum {
	SAMPLE_SLEEP_MODE_NO = 0,
	SAMPLE_SLEEP_MODE_LIGHT = 1,
	SAMPLE_SLEEP_MODE_DEEP = 2,
} sample_sleep_mode_t;

typedef struct {
	bool has_latitude;
	bool has_longitude;
	bool has_altitude;
	bool has_temperature;
	bool has_humidity;
	bool has_pressure;
	bool has_vibration_average;
	bool has_length;
	bool has_accel_x;
	bool has_accel_y;
	bool has_accel_z;
	bool has_gyro_x;
	bool has_gyro_y;
	bool has_gyro_z;
	float latitude;
	float longitude;
	float altitude;
	float temperature;
	float humidity;
	float pressure;
	float vibration_average;
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	int32_t length;
} sample_sensor_data_t;

extern uint32_t g_sample_config_version;
extern bool g_sample_sampling_enabled;
extern uint8_t g_sample_sleep_mode;
extern uint32_t g_sample_rate;
extern uint32_t g_sample_report_rate;
extern uint32_t g_sample_send_ack_timeout_ms;
extern uint32_t g_sample_send_retry_delay_ms;
extern uint32_t g_sample_send_retry_count;
extern char g_sample_sensor_uart_i2c[32];
extern char g_sample_sensor_rs485[32];
extern uint32_t g_sample_script_len;
extern bool g_sample_script_truncated;
extern uint8_t *g_sample_script_buf;
extern size_t g_sample_script_buf_capacity;

void sample_settings_load_from_nvs_to_rtc(void);
bool sample_is_rtc_cache_ready(void);
void sample_set_startup_network_guard(bool enabled);
bool sample_is_sampling_enabled(void);
bool sample_is_report_due_now(void);
bool sample_is_deep_report_due_now(void);
sample_sleep_mode_t sample_get_sleep_mode(void);
const char *sample_sleep_mode_to_string(sample_sleep_mode_t mode);
void sample_refresh_script_cache_from_selectors(void);
esp_err_t sample_set_sensor_selectors(const char *uart_i2c, const char *rs485);
void sample_handle_sampling(void);
bool sample_get_latest_sensor_data(sample_sensor_data_t *out);
esp_err_t sample_execute_script_and_log_results(const uint8_t *script,
												size_t script_len,
												char *out_payload,
												size_t out_payload_size);
esp_err_t sample_script_global_buffer_set_capacity(size_t capacity);
size_t sample_script_global_buffer_get_capacity(void);
size_t sample_script_global_buffer_get_length(void);
size_t sample_script_global_buffer_get_serving_length(void);
const uint8_t *sample_script_global_buffer_get_data(void);
esp_err_t sample_script_global_buffer_publish_for_serving(void);
esp_err_t sample_script_global_buffer_acquire_serving(const uint8_t **data,
										 size_t *length);
void sample_script_global_buffer_release_serving(void);
void sample_script_global_buffer_reset(void);
esp_err_t sample_script_global_buffer_append(const uint8_t *data, size_t len);
esp_err_t sample_script_global_buffer_write_at(size_t pos, const uint8_t *data, size_t len);
void sample_start_no_sleep_tasks(void);
void sample_run_light_sleep_mode_loop(void);
void sample_run_deep_sleep_mode_cycle(void);

#ifdef __cplusplus
}
#endif
