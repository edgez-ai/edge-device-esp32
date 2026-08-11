#pragma once

#include <stddef.h>
#include <stdint.h>

// IEEE 802 Local Experimental EtherType 1, valid on private LANs. Carries
// EdgeZ protobuf/vendor payloads directly.
static constexpr uint16_t ETHERTYPE_EDGEZ_HALOW = 0x88B5;
// Peer-independent public beacon/report frames. Kept separate so only this
// traffic may bypass SAE data protection before a mesh peer is authorized.
static constexpr uint16_t ETHERTYPE_EDGEZ_BEACON_REPORT = 0x88B6;
// Peer-independent targeted sensor transfer control/data. Frames use a
// multicast radio bearer, while the NetworkPacket `to` field selects the one
// device that may consume them.

static constexpr uint8_t HALOW_BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct __attribute__((packed)) HaLowEthFrameHeader {
    uint8_t dst[6];
    uint8_t src[6];
    uint16_t ethertype; // network byte order on the wire
};
static_assert(sizeof(HaLowEthFrameHeader) == 14, "HaLow ethernet header must be 14 bytes");
