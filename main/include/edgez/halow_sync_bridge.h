#ifndef EDGEZ_HALOW_SYNC_BRIDGE_H
#define EDGEZ_HALOW_SYNC_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "usb_control.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HALOW_SYNC_ETHERTYPE 0x88B5
#define HALOW_SYNC_REPORT_ETHERTYPE 0x88B6

/* Public-channel IDs are OpenMANET RTP talkgroup ports. */
#define EDGEZ_PUBLIC_CHANNEL_COUNT 5U
#define EDGEZ_PUBLIC_CHANNEL_FIRST_ID UINT64_C(38801)
#define EDGEZ_PUBLIC_CHANNEL_LAST_ID UINT64_C(38809)
#define EDGEZ_PUBLIC_CHANNEL_ID_STEP UINT64_C(2)
#define EDGEZ_PUBLIC_CHANNEL_ALL_MASK ((1U << EDGEZ_PUBLIC_CHANNEL_COUNT) - 1U)

static inline bool halow_sync_is_public_channel(uint64_t id)
{
    return id >= EDGEZ_PUBLIC_CHANNEL_FIRST_ID &&
           id <= EDGEZ_PUBLIC_CHANNEL_LAST_ID &&
           ((id - EDGEZ_PUBLIC_CHANNEL_FIRST_ID) %
            EDGEZ_PUBLIC_CHANNEL_ID_STEP) == 0;
}

bool halow_sync_public_channel_enabled(uint64_t id);

typedef enum {
    HALOW_SYNC_ACTIVE_INTERFACE_NONE = 0,
    HALOW_SYNC_ACTIVE_INTERFACE_BLE,
    HALOW_SYNC_ACTIVE_INTERFACE_USB,
} halow_sync_active_interface_t;

void halow_sync_bridge_init(void);
void halow_sync_bridge_note_active_interface(halow_sync_active_interface_t active_interface);
void halow_sync_bridge_note_ble_connected(bool connected);
void halow_sync_bridge_note_usb_connected(bool connected);
void halow_sync_bridge_request_status_report(void);
void halow_sync_bridge_fill_status(ai_edgez_halow_HaLowInterfaceStatus *status);
void halow_sync_bridge_fill_report_peers(ai_edgez_halow_Report *report);
void halow_sync_bridge_notify_beacon(const ai_edgez_halow_Beacon *beacon,
                                     const uint8_t peer_mac[6],
                                     int32_t rssi_dbm);
void halow_sync_bridge_notify_batman_peer(const uint8_t originator[6],
                                          const uint8_t last_sender[6],
                                          int32_t rssi_dbm,
                                          uint32_t sequence,
                                          bool direct);
bool halow_sync_bridge_forwarding_enabled(void);
uint32_t halow_sync_bridge_beacon_interval_seconds(void);
bool halow_sync_bridge_queue_log_frame(const uint8_t *payload,
                                       uint16_t payload_len);
bool halow_sync_bridge_log_delivery_active(void);
void halow_sync_bridge_set_log_stream_enabled(bool enabled);
void halow_sync_bridge_request_log_level_test(void);
esp_err_t halow_sync_bridge_handle_to_radio(const uint8_t *payload,
                                                  size_t payload_len,
                                                  uint32_t request_id,
                                                  bool *sent_status_response,
                                                  bool via_forward_interface);
esp_err_t halow_sync_bridge_handle_voice_to_radio(const uint8_t *payload,
                                                   size_t payload_len);
esp_err_t halow_sync_bridge_handle_speed_to_radio(const uint8_t *payload,
                                                   size_t payload_len);
void halow_sync_bridge_send_realtime_frame(const uint8_t *payload,
                                            uint16_t payload_len);
bool halow_sync_bridge_handle_rx_frame(uint8_t *header,
                                             unsigned header_len,
                                             uint8_t *payload,
                                             unsigned payload_len,
                                             const uint8_t *remote_bssid,
                                             bool batman_delivered);
void halow_sync_bridge_send_status_frame(uint16_t seq);

#ifdef __cplusplus
}
#endif

#endif
