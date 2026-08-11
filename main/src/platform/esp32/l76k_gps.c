#include "l76k_gps.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "l76k_gps";

#ifdef CONFIG_EDGEZ_L76K_GPS

#ifdef CONFIG_EDGEZ_L76K_POWER_ACTIVE_HIGH
#define L76K_POWER_ACTIVE_LEVEL 1
#else
#define L76K_POWER_ACTIVE_LEVEL 0
#endif

typedef struct {
    bool valid;
    float latitude;
    float longitude;
    uint64_t timestamp_ms;
} l76k_fix_t;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_task;
static bool s_enabled;
static uint32_t s_update_interval_seconds = CONFIG_EDGEZ_L76K_MIN_UPDATE_SECONDS;
static l76k_fix_t s_latest;
static bool s_receiver_initialized;

static int hex_value(char value)
{
    if (value >= '0' && value <= '9') return value - '0';
    value = (char)toupper((unsigned char)value);
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    return -1;
}

static bool nmea_checksum_valid(const char *line)
{
    if (!line || line[0] != '$') return false;
    const char *star = strchr(line, '*');
    if (!star || star[1] == '\0' || star[2] == '\0') return false;
    uint8_t checksum = 0;
    for (const char *cursor = line + 1; cursor < star; ++cursor) {
        checksum ^= (uint8_t)*cursor;
    }
    int high = hex_value(star[1]);
    int low = hex_value(star[2]);
    return high >= 0 && low >= 0 && checksum == (uint8_t)((high << 4) | low);
}

static bool nmea_coordinate(const char *value, const char *hemisphere,
                            int degree_digits, float *out)
{
    if (!value || !hemisphere || !out || strlen(value) <= (size_t)degree_digits) {
        return false;
    }
    char degrees_text[4] = {0};
    if (degree_digits >= (int)sizeof(degrees_text)) return false;
    memcpy(degrees_text, value, (size_t)degree_digits);
    char *end = NULL;
    double degrees = strtod(degrees_text, &end);
    if (!end || *end != '\0') return false;
    double minutes = strtod(value + degree_digits, &end);
    if (!end || (*end != '\0' && *end != '*') || minutes < 0.0 || minutes >= 60.0) {
        return false;
    }
    double coordinate = degrees + minutes / 60.0;
    char direction = (char)toupper((unsigned char)hemisphere[0]);
    if (direction == 'S' || direction == 'W') coordinate = -coordinate;
    if ((degree_digits == 2 && direction != 'N' && direction != 'S') ||
        (degree_digits == 3 && direction != 'E' && direction != 'W')) {
        return false;
    }
    *out = (float)coordinate;
    return isfinite(*out);
}

static bool parse_rmc(char *line, float *latitude, float *longitude)
{
    if (!nmea_checksum_valid(line)) return false;
    char *star = strchr(line, '*');
    if (star) *star = '\0';

    char *fields[8] = {0};
    size_t count = 1;
    fields[0] = line;
    for (char *cursor = line; *cursor != '\0'; ++cursor) {
        if (*cursor == ',') {
            *cursor = '\0';
            if (count < sizeof(fields) / sizeof(fields[0])) {
                fields[count++] = cursor + 1;
            }
        }
    }
    if (count < 7 ||
        (strcmp(fields[0], "$GNRMC") != 0 && strcmp(fields[0], "$GPRMC") != 0) ||
        strcmp(fields[2], "A") != 0) {
        return false;
    }
    return nmea_coordinate(fields[3], fields[4], 2, latitude) &&
           nmea_coordinate(fields[5], fields[6], 3, longitude) &&
           *latitude >= -90.0f && *latitude <= 90.0f &&
           *longitude >= -180.0f && *longitude <= 180.0f &&
           (*latitude != 0.0f || *longitude != 0.0f);
}

static bool nmea_is_rmc(const char *line)
{
    return line &&
           (strncmp(line, "$GNRMC,", 7) == 0 ||
            strncmp(line, "$GPRMC,", 7) == 0);
}

static bool rmc_has_active_fix(const char *line)
{
    if (!nmea_is_rmc(line)) return false;
    const char *time_separator = strchr(line, ',');
    if (!time_separator) return false;
    const char *status_separator = strchr(time_separator + 1, ',');
    return status_separator && status_separator[1] == 'A' &&
           status_separator[2] == ',';
}

static void gps_power(bool enabled)
{
    if (CONFIG_EDGEZ_L76K_POWER_GPIO < 0) return;
    int active = L76K_POWER_ACTIVE_LEVEL;
    (void)gpio_set_level((gpio_num_t)CONFIG_EDGEZ_L76K_POWER_GPIO,
                         enabled ? active : !active);
}

/* RESET_N and WAKE_UP have internal pull-ups. Drive them low only while
 * asserted and otherwise release them to high impedance, matching the HC33
 * reference wiring and avoiding an externally driven high level. */
static void gps_release_pin(int pin)
{
    if (pin < 0) return;
    (void)gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
}

static void gps_pull_pin_low(int pin)
{
    if (pin < 0) return;
    (void)gpio_set_level((gpio_num_t)pin, 0);
    (void)gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
}

static void gps_enter_standby(void)
{
    gps_pull_pin_low(CONFIG_EDGEZ_L76K_WAKE_GPIO);
    ESP_LOGI(TAG, "L76K entering standby (WAKE_UP GPIO%d low)",
             CONFIG_EDGEZ_L76K_WAKE_GPIO);
}

static void gps_wake_receiver(bool reset_receiver)
{
    gps_release_pin(CONFIG_EDGEZ_L76K_WAKE_GPIO);
    ESP_LOGI(TAG, "L76K waking (WAKE_UP GPIO%d released)",
             CONFIG_EDGEZ_L76K_WAKE_GPIO);

    if (reset_receiver && CONFIG_EDGEZ_L76K_RESET_GPIO >= 0) {
        ESP_LOGI(TAG, "L76K cold initialization (RESET_N GPIO%d)",
                 CONFIG_EDGEZ_L76K_RESET_GPIO);
        gps_pull_pin_low(CONFIG_EDGEZ_L76K_RESET_GPIO);
        vTaskDelay(pdMS_TO_TICKS(200));
        gps_release_pin(CONFIG_EDGEZ_L76K_RESET_GPIO);
        vTaskDelay(pdMS_TO_TICKS(100));

        /* Brief standby pulse followed by release selects Continuous mode. */
        gps_pull_pin_low(CONFIG_EDGEZ_L76K_WAKE_GPIO);
        vTaskDelay(pdMS_TO_TICKS(20));
        gps_release_pin(CONFIG_EDGEZ_L76K_WAKE_GPIO);
        vTaskDelay(pdMS_TO_TICKS(100));
    } else {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static esp_err_t gps_uart_open(void)
{
    uart_port_t port = (uart_port_t)CONFIG_EDGEZ_L76K_UART_PORT;
    uart_config_t config = {
        .baud_rate = CONFIG_EDGEZ_L76K_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_param_config(port, &config);
    if (err == ESP_OK) {
        err = uart_set_pin(port, CONFIG_EDGEZ_L76K_UART_TX_PIN,
                           CONFIG_EDGEZ_L76K_UART_RX_PIN,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err == ESP_OK) {
        err = uart_driver_install(port, 2048, 0, 0, NULL, 0);
    }
    return err;
}

static bool acquire_fix(l76k_fix_t *fix)
{
    if (!fix) return false;
    gps_power(true);
    if (CONFIG_EDGEZ_L76K_POWER_GPIO >= 0) {
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    gps_wake_receiver(!s_receiver_initialized);
    esp_err_t err = gps_uart_open();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "L76K unavailable; UART open failed: %s", esp_err_to_name(err));
        gps_enter_standby();
        gps_power(false);
        return false;
    }

    uart_port_t port = (uart_port_t)CONFIG_EDGEZ_L76K_UART_PORT;
    (void)uart_flush_input(port);
    ESP_LOGI(TAG,
             "L76K acquisition started uart=%d baud=%d ESP_TX=GPIO%d ESP_RX=GPIO%d timeout=%ds",
             (int)port, CONFIG_EDGEZ_L76K_BAUD_RATE,
             CONFIG_EDGEZ_L76K_UART_TX_PIN, CONFIG_EDGEZ_L76K_UART_RX_PIN,
             CONFIG_EDGEZ_L76K_FIX_TIMEOUT_SECONDS);
    char line[128] = {0};
    size_t line_length = 0;
    bool found = false;
    size_t bytes_received = 0;
    uint32_t sentence_count = 0;
    uint32_t valid_checksum_count = 0;
    uint32_t rmc_count = 0;
    uint32_t active_rmc_count = 0;
    uint32_t overlong_line_count = 0;
    TickType_t deadline = xTaskGetTickCount() +
                          pdMS_TO_TICKS(CONFIG_EDGEZ_L76K_FIX_TIMEOUT_SECONDS * 1000UL);
    while (!found && (int32_t)(deadline - xTaskGetTickCount()) > 0) {
        portENTER_CRITICAL(&s_lock);
        bool still_enabled = s_enabled;
        portEXIT_CRITICAL(&s_lock);
        if (!still_enabled) break;

        uint8_t input[96];
        int length = uart_read_bytes(port, input, sizeof(input), pdMS_TO_TICKS(250));
        if (length < 0) {
            ESP_LOGW(TAG, "L76K UART read failed; location skipped");
            break;
        }
        if (length > 0) bytes_received += (size_t)length;
        for (int index = 0; index < length && !found; ++index) {
            char value = (char)input[index];
            if (value == '\r') continue;
            if (value == '\n') {
                if (line_length > 0) {
                    line[line_length] = '\0';
                    sentence_count++;
                    bool checksum_valid = nmea_checksum_valid(line);
                    if (checksum_valid) valid_checksum_count++;
                    if (nmea_is_rmc(line)) {
                        rmc_count++;
                        if (rmc_has_active_fix(line)) active_rmc_count++;
                    }
                    float latitude = 0.0f;
                    float longitude = 0.0f;
                    if (parse_rmc(line, &latitude, &longitude)) {
                        fix->valid = true;
                        fix->latitude = latitude;
                        fix->longitude = longitude;
                        fix->timestamp_ms = (uint64_t)(esp_timer_get_time() / 1000);
                        found = true;
                    }
                }
                line_length = 0;
            } else if (line_length + 1 < sizeof(line)) {
                line[line_length++] = value;
            } else {
                overlong_line_count++;
                line_length = 0;
            }
        }
    }

    (void)uart_driver_delete(port);
    s_receiver_initialized = valid_checksum_count > 0;
    if (found) {
        ESP_LOGI(TAG,
                 "L76K valid fix received bytes=%u sentences=%lu checksum_ok=%lu rmc=%lu active_rmc=%lu",
                 (unsigned)bytes_received, (unsigned long)sentence_count,
                 (unsigned long)valid_checksum_count, (unsigned long)rmc_count,
                 (unsigned long)active_rmc_count);
    } else if (bytes_received == 0) {
        ESP_LOGW(TAG,
                 "L76K timeout: no UART bytes; check 3.3V/GND, WAKE_UP GPIO%d, GPS TXD->GPIO%d, and baud=%d",
                 CONFIG_EDGEZ_L76K_WAKE_GPIO, CONFIG_EDGEZ_L76K_UART_RX_PIN,
                 CONFIG_EDGEZ_L76K_BAUD_RATE);
    } else if (valid_checksum_count == 0) {
        ESP_LOGW(TAG,
                 "L76K timeout: UART data but no checksum-valid NMEA bytes=%u sentences=%lu overlong=%lu; check baud/wiring/noise",
                 (unsigned)bytes_received, (unsigned long)sentence_count,
                 (unsigned long)overlong_line_count);
    } else if (rmc_count == 0) {
        ESP_LOGW(TAG,
                 "L76K timeout: valid NMEA received but no GNRMC/GPRMC sentence checksum_ok=%lu",
                 (unsigned long)valid_checksum_count);
    } else if (active_rmc_count == 0) {
        ESP_LOGW(TAG,
                 "L76K timeout: RMC received with status V (receiver responding, no satellite fix yet) rmc=%lu checksum_ok=%lu",
                 (unsigned long)rmc_count, (unsigned long)valid_checksum_count);
    } else {
        ESP_LOGW(TAG,
                 "L76K timeout: active RMC received but coordinates were invalid rmc=%lu active=%lu",
                 (unsigned long)rmc_count, (unsigned long)active_rmc_count);
    }
    gps_enter_standby();
    gps_power(false);
    return found;
}

static void gps_task(void *arg)
{
    (void)arg;
    while (true) {
        portENTER_CRITICAL(&s_lock);
        bool enabled = s_enabled;
        uint32_t interval = s_update_interval_seconds;
        portEXIT_CRITICAL(&s_lock);
        if (!enabled) {
            gps_enter_standby();
            gps_power(false);
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }

        l76k_fix_t fix = {0};
        uint32_t wait_seconds = interval;
        if (acquire_fix(&fix)) {
            portENTER_CRITICAL(&s_lock);
            s_latest = fix;
            portEXIT_CRITICAL(&s_lock);
            ESP_LOGI(TAG, "L76K fix lat=%.6f lon=%.6f; receiver powered down",
                     (double)fix.latitude, (double)fix.longitude);
        } else {
            portENTER_CRITICAL(&s_lock);
            s_latest.valid = false;
            portEXIT_CRITICAL(&s_lock);
            ESP_LOGW(TAG, "No L76K fix; optional location skipped");
            wait_seconds = interval > 900U ? 3600U : interval * 4U;
        }
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_seconds * 1000UL));
    }
}

bool l76k_gps_supported(void)
{
    return true;
}

esp_err_t l76k_gps_configure(bool enabled, uint32_t update_interval_seconds)
{
    uint32_t interval = update_interval_seconds;
    if (interval < CONFIG_EDGEZ_L76K_MIN_UPDATE_SECONDS) {
        interval = CONFIG_EDGEZ_L76K_MIN_UPDATE_SECONDS;
    }

    if (!enabled && s_task == NULL) {
        return ESP_OK;
    }

    if (s_task != NULL) {
        portENTER_CRITICAL(&s_lock);
        bool unchanged = s_enabled == enabled &&
                         s_update_interval_seconds == interval;
        portEXIT_CRITICAL(&s_lock);
        if (unchanged) {
            return ESP_OK;
        }
    }

    if (enabled && s_task == NULL) {
#if CONFIG_EDGEZ_L76K_POWER_GPIO >= 0
        gpio_config_t config = {
            .pin_bit_mask = 1ULL << CONFIG_EDGEZ_L76K_POWER_GPIO,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t err = gpio_config(&config);
        if (err != ESP_OK) return err;
        gps_power(false);
#endif
        gps_release_pin(CONFIG_EDGEZ_L76K_RESET_GPIO);
        gps_enter_standby();
        ESP_LOGI(TAG,
                 "L76K HC33 pins RESET_N=GPIO%d WAKE_UP=GPIO%d GPS_RXD<-ESP_TX=GPIO%d GPS_TXD->ESP_RX=GPIO%d power=%s",
                 CONFIG_EDGEZ_L76K_RESET_GPIO, CONFIG_EDGEZ_L76K_WAKE_GPIO,
                 CONFIG_EDGEZ_L76K_UART_TX_PIN, CONFIG_EDGEZ_L76K_UART_RX_PIN,
                 CONFIG_EDGEZ_L76K_POWER_GPIO < 0 ? "direct 3.3V" : "switched GPIO");
        BaseType_t created = xTaskCreate(gps_task, "l76k_gps", 4096, NULL, 3, &s_task);
        if (created != pdPASS) {
            s_task = NULL;
            return ESP_ERR_NO_MEM;
        }
    }

    portENTER_CRITICAL(&s_lock);
    s_enabled = enabled;
    s_update_interval_seconds = interval;
    if (!enabled) s_latest.valid = false;
    portEXIT_CRITICAL(&s_lock);
    xTaskNotifyGive(s_task);
    ESP_LOGI(TAG, "L76K %s update_interval=%lu seconds",
             enabled ? "enabled" : "disabled", (unsigned long)interval);
    return ESP_OK;
}

bool l76k_gps_get_latest(float *latitude, float *longitude,
                         uint64_t *timestamp_ms)
{
    l76k_fix_t fix;
    portENTER_CRITICAL(&s_lock);
    fix = s_latest;
    portEXIT_CRITICAL(&s_lock);
    if (!fix.valid) return false;
    if (latitude) *latitude = fix.latitude;
    if (longitude) *longitude = fix.longitude;
    if (timestamp_ms) *timestamp_ms = fix.timestamp_ms;
    return true;
}

#else

bool l76k_gps_supported(void) { return false; }

esp_err_t l76k_gps_configure(bool enabled, uint32_t update_interval_seconds)
{
    (void)update_interval_seconds;
    return enabled ? ESP_ERR_NOT_SUPPORTED : ESP_OK;
}

bool l76k_gps_get_latest(float *latitude, float *longitude,
                         uint64_t *timestamp_ms)
{
    (void)latitude;
    (void)longitude;
    (void)timestamp_ms;
    return false;
}

#endif
