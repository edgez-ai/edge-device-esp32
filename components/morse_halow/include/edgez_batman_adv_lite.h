#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EDGEZ_BATADV_ETHERTYPE 0x4305U
#define EDGEZ_BATADV_COMPAT_VERSION 15U
#define EDGEZ_BATADV_IV_OGM 0x00U
#define EDGEZ_BATADV_BCAST 0x01U
#define EDGEZ_BATADV_UNICAST 0x40U
#define EDGEZ_BATADV_OGM_LEN 24U
#define EDGEZ_BATADV_UNICAST_HEADER_LEN 10U
#define EDGEZ_BATADV_BCAST_HEADER_LEN 14U
#define EDGEZ_BATADV_DEFAULT_TTL 50U
#define EDGEZ_BATADV_MAX_PACKET_LEN 512U
#define EDGEZ_BATADV_MAX_PEERS 16U
#define EDGEZ_BATADV_MAX_ROUTES 16U
#define EDGEZ_BATADV_MAX_TT_CLIENTS 32U

typedef struct {
    bool occupied;
    uint8_t originator[6];
    uint8_t last_sender[6];
    uint32_t last_sequence;
    uint64_t sequence_window;
    uint32_t newest_bcast_sequence;
    uint64_t bcast_sequence_window;
    uint32_t last_seen_ms;
    uint32_t last_notified_ms;
} edgez_batadv_peer_t;

typedef struct {
    bool occupied;
    bool selected;
    uint8_t originator[6];
    uint8_t next_hop[6];
    uint32_t newest_sequence;
    uint64_t sequence_window;
    uint8_t sequence_span;
    uint32_t last_seen_ms;
    uint8_t advertised_tq;
    uint8_t tq;
    uint8_t hops;
    uint8_t ttvn;
    bool gateway;
    uint32_t gateway_down;
    uint32_t gateway_up;
} edgez_batadv_route_t;

typedef struct {
    uint8_t originator[6];
    uint8_t next_hop[6];
    uint8_t tq;
    uint8_t hops;
    uint32_t age_ms;
} edgez_batadv_route_snapshot_t;

typedef struct {
    bool occupied;
    uint8_t client[6];
    uint8_t originator[6];
    uint32_t last_seen_ms;
    uint8_t ttvn;
} edgez_batadv_tt_client_t;

typedef struct {
    bool initialized;
    bool connected;
    uint8_t local_mac[6];
    uint32_t next_sequence;
    uint32_t next_bcast_sequence;
    uint32_t next_ogm_ms;
    uint32_t interval_ms;
    uint32_t jitter_state;
    edgez_batadv_peer_t peers[EDGEZ_BATADV_MAX_PEERS];
    edgez_batadv_route_t routes[EDGEZ_BATADV_MAX_ROUTES];
    edgez_batadv_tt_client_t tt_clients[EDGEZ_BATADV_MAX_TT_CLIENTS];
    uint8_t selected_gateway[6];
} edgez_batadv_lite_t;

typedef struct {
    bool valid;
    bool peer_valid;
    bool new_peer;
    bool notify_peer;
    uint8_t packet_type;
    uint8_t originator[6];
    uint8_t last_sender[6];
    uint32_t sequence;

    /* At most one forwarding action is returned per input Ethernet frame.
     * All aggregated OGMs still update routing/duplicate state. */
    bool forward_ready;
    bool forward_directed;
    bool forward_exclude_valid;
    uint8_t forward_next_hop[6];
    uint8_t forward_exclude[6];
    uint8_t forward[EDGEZ_BATADV_MAX_PACKET_LEN];
    size_t forward_len;

    /* A locally addressed BATMAN data packet is copied here so callers can
     * deliver it after the BATMAN state lock has been released. */
    bool deliver_ready;
    /* BATMAN originator which emitted a delivered broadcast. This can differ
     * from the source MAC of the encapsulated Ethernet frame. */
    uint8_t deliver_originator[6];
    uint8_t deliver[EDGEZ_BATADV_MAX_PACKET_LEN];
    size_t deliver_len;
} edgez_batadv_rx_result_t;

void edgez_batadv_lite_init(edgez_batadv_lite_t *state,
                            const uint8_t local_mac[6],
                            uint32_t initial_sequence,
                            uint32_t interval_ms,
                            uint32_t now_ms);
void edgez_batadv_lite_set_connected(edgez_batadv_lite_t *state,
                                     bool connected,
                                     uint32_t now_ms);
bool edgez_batadv_lite_build_periodic_ogm(edgez_batadv_lite_t *state,
                                          uint32_t now_ms,
                                          uint8_t out[EDGEZ_BATADV_OGM_LEN]);
edgez_batadv_rx_result_t edgez_batadv_lite_receive(edgez_batadv_lite_t *state,
                                                   const uint8_t ethernet_source[6],
                                                   const uint8_t *payload,
                                                   size_t payload_len,
                                                   uint32_t now_ms);
bool edgez_batadv_lite_lookup_route(edgez_batadv_lite_t *state,
                                    const uint8_t destination[6],
                                    uint32_t now_ms,
                                    uint8_t next_hop[6]);
bool edgez_batadv_lite_lookup_route_info(edgez_batadv_lite_t *state,
                                         const uint8_t destination[6],
                                         uint32_t now_ms,
                                         uint8_t next_hop[6],
                                         uint8_t *hop_count,
                                         uint8_t *tq,
                                         uint32_t *route_age_ms);
size_t edgez_batadv_lite_snapshot_routes(
    edgez_batadv_lite_t *state, uint32_t now_ms,
    edgez_batadv_route_snapshot_t *routes, size_t capacity);
bool edgez_batadv_lite_select_direct_peer(edgez_batadv_lite_t *state,
                                           const uint8_t exclude_a[6],
                                           const uint8_t exclude_b[6],
                                           const uint8_t exclude_c[6],
                                           uint64_t selection_key,
                                           uint32_t now_ms,
                                           uint8_t peer[6]);
bool edgez_batadv_lite_build_unicast(edgez_batadv_lite_t *state,
                                     const uint8_t destination[6],
                                     const uint8_t *payload,
                                     size_t payload_len,
                                     uint32_t now_ms,
                                     uint8_t out[EDGEZ_BATADV_MAX_PACKET_LEN],
                                     size_t *out_len,
                                     uint8_t next_hop[6]);
bool edgez_batadv_lite_build_broadcast(edgez_batadv_lite_t *state,
                                       const uint8_t *payload,
                                       size_t payload_len,
                                       uint8_t out[EDGEZ_BATADV_MAX_PACKET_LEN],
                                       size_t *out_len);
bool edgez_batadv_lite_selected_gateway(edgez_batadv_lite_t *state,
                                        uint32_t now_ms,
                                        uint8_t gateway[6]);
size_t edgez_batadv_lite_invalidate_next_hop(
    edgez_batadv_lite_t *state, const uint8_t next_hop[6], uint32_t now_ms);
void edgez_batadv_lite_expire(edgez_batadv_lite_t *state, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
