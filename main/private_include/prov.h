/**
 * @file prov.h
 * @brief Platform-agnostic Wi-Fi/Network connection interface
 * 
 * This header provides a common interface for network connection handling
 * across different platforms.
 */

#ifndef DRIVER_PROV_H
#define DRIVER_PROV_H

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "network_transport.h"
#include "usb_control.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Event bit to signal Wi-Fi connection */
#define WIFI_CONNECTED_EVENT BIT0
/* Event bit to signal network stack is ready for sockets (valid IP acquired) */
#define WIFI_NETWORK_READY_EVENT BIT1

/**
 * @brief Initialize network connection state
 * 
 * This function initializes event handling and waits for runtime code to
 * supply HaLow credentials with wifi_prov_connect_halow().
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t wifi_prov_init_and_start(void);

/**
 * @brief Connect HaLow with runtime-supplied credentials
 *
 * Intended to be called by the BLE input layer after it receives parameters.
 * Credentials are kept in RAM for this boot and are not persisted to NVS.
 *
 * @param ssid Non-empty HaLow SSID
 * @param password Optional passphrase, or NULL/empty for open networks
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t wifi_prov_connect_halow(const char *ssid, const char *password);
esp_err_t wifi_prov_start_halow_beacon_only(const char *ssid, const char *password);

/**
 * @brief Connect the ESP native Wi-Fi STA as an upstream network.
 *
 * This is separate from wifi_prov_set_wifi_credentials() so HaLow builds can
 * keep using runtime credentials for the HaLow mesh while optionally joining a
 * normal Wi-Fi AP in device mode.
 */
esp_err_t wifi_prov_connect_upstream_wifi(const char *ssid, const char *passphrase);

/**
 * @brief Set local user identity advertised in EdgeZ HaLow discovery IEs
 *
 * The identity is kept in RAM for this boot and included in the encrypted
 * discovery protobuf carried by the EdgeZ vendor IE.
 */
esp_err_t wifi_prov_set_halow_user_identity(uint64_t user_id_high,
                                            uint64_t user_id_low,
                                            const char *user_name,
                                            const uint8_t *user_public_key,
                                            size_t user_public_key_len);
esp_err_t wifi_prov_set_halow_beacon_profile(uint32_t marker,
                                             bool has_location,
                                             float latitude,
                                             float longitude);
esp_err_t wifi_prov_set_halow_beacon_sensor_data(
    const ai_edgez_halow_SensorData *sensor_data,
    size_t sensor_data_count);
esp_err_t wifi_prov_set_halow_beacon_device_type(uint32_t device_type);
esp_err_t wifi_prov_refresh_halow_mesh_vendor_ie(const char *mesh_id,
                                                 const char *passphrase);
bool halow_edgez_mesh_peer_admission_allowed(const uint8_t *ies,
                                             size_t ies_len,
                                             uint32_t *device_type);

/**
 * @brief Set the maximum EdgeZ HaLow broadcast relay hop count
 */
esp_err_t wifi_prov_set_halow_max_hop(uint32_t max_hop);

/**
 * @brief Select the HaLow mesh bootstrap channel from a country-validated
 * frequency and bandwidth supplied by the mobile configuration.
 */
esp_err_t wifi_prov_set_halow_mesh_radio(uint32_t frequency_khz,
                                         uint32_t bandwidth_mhz);

/**
 * @brief Persist ESP Wi-Fi STA credentials from a local control channel
 *
 * Stores credentials in the ESP Wi-Fi driver/NVS config when that driver is
 * active. On HaLow builds where ESP Wi-Fi is not initialized, applies the same
 * credentials to the runtime HaLow connection path.
 *
 * @param ssid Non-empty 2.4 GHz Wi-Fi SSID
 * @param passphrase Optional passphrase, or NULL/empty for open networks
 * @param connect_after_set Request an immediate reconnect when possible
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t wifi_prov_set_wifi_credentials(const char *ssid,
                                         const char *passphrase,
                                         bool connect_after_set);

/**
 * @brief Wait for Wi-Fi connection
 * 
 * Blocks until the Wi-Fi connection is established
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t wifi_prov_wait_connected(void);

/**
 * @brief Wait for Wi-Fi connection with timeout
 * 
 * Blocks until the Wi-Fi connection is established or timeout occurs
 * 
 * @param timeout_ms Timeout in milliseconds (0 = no wait, portMAX_DELAY = wait forever)
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on timeout, error code otherwise
 */
esp_err_t wifi_prov_wait_connected_timeout(uint32_t timeout_ms);

/**
 * @brief Wait until network is ready for sockets (valid IP acquired)
 *
 * For Wi-Fi, this is equivalent to IP_EVENT_STA_GOT_IP.
 * For HaLow, this waits until the real HaLow IP is available.
 *
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on timeout, error code otherwise
 */
esp_err_t wifi_prov_wait_network_ready_timeout(uint32_t timeout_ms);

/**
 * @brief Get active IPv4 gateway address for current transport
 *
 * In HaLow mode this prefers MMIPAL/cached HaLow IP state, which can be ready
 * before esp_netif reflects the gateway. In Wi-Fi mode this falls back to
 * esp_netif gateway information.
 *
 * @param out_gateway Output buffer for IPv4 gateway string (e.g. "192.168.12.1")
 * @param out_gateway_len Output buffer length
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_FOUND when unavailable, error otherwise
 */
esp_err_t wifi_prov_get_active_gateway_ipv4(char *out_gateway, size_t out_gateway_len);

/**
 * @brief Prepare provisioning/network stacks for deep sleep
 *
 * Ensures active transport stacks are shut down before deep sleep:
 * - HaLow transceiver (when active)
 * - Wi-Fi STA stack
 *
 * Safe to call repeatedly; treats already-stopped states as success.
 *
 * @return esp_err_t ESP_OK on success, or first non-fatal teardown error
 */
esp_err_t wifi_prov_prepare_for_deep_sleep(void);

/**
 * @brief Get the Wi-Fi event group handle
 * 
 * @return EventGroupHandle_t Event group handle for Wi-Fi events
 */
EventGroupHandle_t wifi_prov_get_event_group(void);

/**
 * @brief Check if Wi-Fi has runtime credentials
 * 
 * @return true if runtime credentials are available, false otherwise
 */
bool wifi_is_provisioned(void);

/**
 * @brief Legacy compatibility hook; Wi-Fi provisioning has been removed
 *
 * @return false
 */
bool wifi_prov_ble_session_active(void);

/**
 * @brief Legacy compatibility hook; Wi-Fi provisioning has been removed
 *
 * @return false
 */
bool wifi_prov_service_active(void);

/**
 * @brief Legacy compatibility hook; Wi-Fi provisioning has been removed
 *
 * @param window_ms Time window in milliseconds
 * @return false
 */
bool wifi_prov_recent_success(uint32_t window_ms);

/**
 * @brief Check whether this boot has usable runtime credentials
 *
 * Stored factory/NVS network credentials are intentionally ignored by the
 * startup path.
 */
bool wifi_prov_has_bootstrap_credentials(void);

/**
 * @brief Legacy compatibility hook; Wi-Fi provisioning has been removed
 *
 * @return ESP_ERR_NOT_SUPPORTED
 */
esp_err_t wifi_prov_start_ble_reprovisioning(void);

/**
 * @brief Get the current connection type (WiFi or HaLow)
 * 
 * @return edgez_transport_type_t Transport used for the current connection
 */
edgez_transport_type_t wifi_prov_get_connection_type(void);

#ifdef CONFIG_ENABLE_MM_HALOW
/**
 * @brief Global flag to track if lwIP was already initialized by esp_netif_init()
 * 
 * This flag prevents double initialization when halow_init() is called
 * after WiFi stack has already initialized lwIP.
 */
extern bool g_lwip_initialized_by_esp_netif;

/**
 * @brief Shut down the HaLow transceiver before entering sleep
 *
 * Cleanly disconnects from the AP, shuts down the WLAN driver, de-initialises
 * the HAL (pulling RESET_N low), and holds the GPIO state so the module stays
 * powered off during deep sleep.  Safe to call when HaLow was never started.
 */
void wifi_prov_shutdown_halow_for_sleep(void);

/**
 * @brief Shut down the HaLow transceiver before warm reboot
 *
 * Same clean disconnect flow as sleep shutdown, but does NOT leave RESET_N
 * pin hold enabled across restart. Use this before esp_restart().
 */
void wifi_prov_shutdown_halow_for_restart(void);

/**
 * @brief Enter HaLow low-power mode for host light sleep windows
 *
 * Performs a full HaLow shutdown before host light sleep to reduce power draw.
 * The radio is restarted on wake via wifi_prov_exit_halow_light_sleep().
 * Safe to call repeatedly; no-op when HaLow is not the active connection.
 */
void wifi_prov_enter_halow_light_sleep(void);

/**
 * @brief Exit HaLow low-power mode after host wakes from light sleep
 *
 * Re-initializes and starts HaLow using runtime credentials after a prior
 * full light-sleep shutdown.
 * Safe to call when light sleep mode was not entered.
 */
esp_err_t wifi_prov_exit_halow_light_sleep(void);

/**
 * @brief Check if HaLow state is ready for entering host light sleep
 *
 * Returns false while HaLow STA reconnect/association is still in progress,
 * so callers can avoid immediately shutting it down again.
 *
 * @return true if safe to enter light sleep now, false if reconnect is active
 */
bool wifi_prov_halow_ready_for_light_sleep(void);

/**
 * @brief Check if HaLow link/network is recovered enough for reporting
 *
 * For HaLow mode this requires STA to be connected and network-ready
 * (valid IP event observed). For non-HaLow modes this returns true.
 *
 * @return true if reporting can proceed, false otherwise
 */
bool wifi_prov_halow_ready_for_report(void);

/**
 * @brief Override HaLow regulatory country code at runtime
 *
 * Must be called before HaLow initialization for deterministic behavior.
 * Expected format is a 2-character ISO code, e.g. "US", "CA", "JP", "AU", "NZ", "EU".
 */
esp_err_t wifi_prov_set_halow_country_code(const char *country_code);

/**
 * @brief Configure startup HaLow mesh profile from factory data
 *
 * Applies only to HaLow startup path when factory credentials are used.
 * When mesh_mode is false, channel/bandwidth hints are ignored.
 */
esp_err_t wifi_prov_set_startup_halow_mesh_profile(bool mesh_mode,
												   uint16_t channel,
												   uint8_t bandwidth_mhz);

/**
 * @brief Snapshot HaLow state for local mobile/USB synchronization.
 */
typedef struct {
	bool supported;
	bool stack_initialized;
	bool mesh_mode;
	bool link_up;
	bool route_ready;
	bool ready_for_report;
	char mesh_id[33];
	char ip_addr[16];
	char gateway[16];
} wifi_prov_halow_status_t;

void wifi_prov_get_halow_status(wifi_prov_halow_status_t *out_status);

/**
 * @brief Example function to start HaLow iperf test after connection
 * 
 * This is a demonstration of how to use the halow_iperf API.
 * Call this after HaLow link is up to start throughput testing.
 * 
 * @param is_server true to start server mode, false for client mode
 * @param use_udp true to use UDP, false for TCP
 * @param server_ip Server IP address (for client mode only)
 */
void halow_start_iperf_example(bool is_server, bool use_udp, const char *server_ip);
#endif

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_PROV_H */
