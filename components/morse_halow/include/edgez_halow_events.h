#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EDGEZ_HALOW_EVENT_HEADER_MAX 32U
#define EDGEZ_HALOW_EVENT_PAYLOAD_MAX 1600U
#define EDGEZ_HALOW_EVENT_IES_MAX 512U

typedef enum {
    EDGEZ_HALOW_EVENT_RX_FRAME = 1,
    EDGEZ_HALOW_EVENT_BEACON,
    EDGEZ_HALOW_EVENT_PEER_ADMISSION,
} edgez_halow_event_type_t;

typedef struct {
    edgez_halow_event_type_t type;
    uint32_t request_id;
    union {
        struct {
            uint16_t header_len;
            uint16_t payload_len;
            uint8_t remote_bssid[6];
            uint8_t header[EDGEZ_HALOW_EVENT_HEADER_MAX];
            uint8_t payload[EDGEZ_HALOW_EVENT_PAYLOAD_MAX];
        } rx_frame;
        struct {
            uint16_t ies_len;
            int32_t rssi_dbm;
            uint8_t bssid[6];
            char mesh_id[33];
            char passphrase[65];
            uint8_t ies[EDGEZ_HALOW_EVENT_IES_MAX];
        } beacon;
        struct {
            uint16_t ies_len;
            uint8_t ies[EDGEZ_HALOW_EVENT_IES_MAX];
        } peer_admission;
    } data;
} edgez_halow_event_t;

/** Initialize the mm-iot-owned application event and response queues. */
esp_err_t edgez_halow_event_queue_init(void);

/** Receive a copied radio event from application task context. */
bool edgez_halow_event_receive(edgez_halow_event_t *event, TickType_t wait);

/** Complete a synchronous peer-admission event. */
esp_err_t edgez_halow_event_respond_peer_admission(uint32_t request_id,
                                                   bool allow,
                                                   uint32_t device_type);

#ifdef __cplusplus
}
#endif
