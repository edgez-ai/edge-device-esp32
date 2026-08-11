#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "network_transport.h"
#include "prov.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char country_code[3];
    char mesh_id[33];
    char passphrase[65];
    uint64_t user_id_high;
    uint64_t user_id_low;
    char user_name[65];
    uint8_t user_public_key[32];
    size_t user_public_key_len;
    uint32_t max_hop;
    uint32_t mesh_frequency_khz;
    uint8_t mesh_bandwidth_mhz;
    uint32_t device_type;
    bool beacon_only;
} halow_interface_init_config_t;

esp_err_t halow_interface_app_init(void);
esp_err_t halow_interface_app_start(const halow_interface_init_config_t *config);
esp_err_t halow_interface_app_set_user_identity(uint64_t user_id_high,
                                                uint64_t user_id_low,
                                                const char *user_name,
                                                const uint8_t *user_public_key,
                                                size_t user_public_key_len);
esp_err_t halow_interface_app_request_beacon(const char *beacon,
                                             uint32_t max_hop,
                                             uint64_t message_id_high,
                                             uint64_t message_id_low,
                                             uint32_t sequence,
                                             uint64_t next_hop);
esp_err_t halow_interface_app_send_mesh_payload(const uint8_t *payload,
                                                size_t payload_len,
                                                uint64_t from,
                                                uint64_t to,
                                                uint64_t message_id_high,
                                                uint64_t message_id_low,
                                                uint32_t max_hop,
                                                uint32_t sequence);
esp_err_t halow_interface_app_send_peer_independent_beacon(const uint8_t *payload,
                                                           size_t payload_len,
                                                           uint64_t from,
                                                           uint64_t to,
                                                           uint64_t message_id_high,
                                                           uint64_t message_id_low,
                                                           uint32_t max_hop,
                                                           uint32_t sequence);
esp_err_t halow_interface_app_send_mesh_payload_via(const uint8_t *payload,
                                                    size_t payload_len,
                                                    uint64_t from,
                                                    uint64_t to,
                                                    uint64_t next_hop,
                                                    uint64_t message_id_high,
                                                    uint64_t message_id_low,
                                                    uint32_t max_hop,
                                                    uint32_t sequence);
esp_err_t halow_interface_app_forward_mesh_payload(const uint8_t *payload,
                                                   size_t payload_len,
                                                   uint64_t from,
                                                   uint64_t to,
                                                   uint64_t next_hop,
                                                   uint64_t message_id_high,
                                                   uint64_t message_id_low,
                                                   uint32_t max_hop,
                                                   uint32_t sequence,
                                                   uint32_t hop);
bool halow_interface_app_get_self_mac(uint8_t mac[6]);
void halow_interface_app_get_status(wifi_prov_halow_status_t *out_status);
bool halow_interface_app_ready(void);
bool halow_interface_app_mesh_initialized(void);
bool halow_interface_app_beacon_only(void);
esp_err_t halow_interface_app_wait_ready(uint32_t timeout_ms);
esp_err_t halow_interface_app_shutdown(bool hold_reset_n);
edgez_transport_type_t halow_interface_app_connection_type(void);

#ifdef __cplusplus
}
#endif
