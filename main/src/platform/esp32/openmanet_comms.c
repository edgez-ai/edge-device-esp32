#include "openmanet_comms.h"

#include <string.h>

#include "esp_random.h"
#include "halow_interface_app.h"
#include "halow_sync_bridge.h"

enum {
    ETH_LEN = 14,
    IPV4_LEN = 20,
    UDP_LEN = 8,
    RTP_LEN = 12,
    ETHERTYPE_IPV4 = 0x0800,
    IP_PROTOCOL_UDP = 17,
    RTP_PAYLOAD_TYPE_OPUS = 111,
    RTP_FRAME_SAMPLES = 960,
    PHONE_TX_HEADER_LEN = 6,
    PHONE_RX_HEADER_LEN = 12,
    /* 512-byte BATMAN packet - 14-byte BCAST header - 54-byte inner
     * Ethernet/IPv4/UDP/RTP headers. */
    MAX_OPUS_PAYLOAD = 444,
};

static const uint8_t s_magic[OPENMANET_COMMS_MAGIC_LEN] = {'O', 'M', 'C', 1};
static const uint8_t s_group_ip[4] = {239, 192, 41, 1};
static const uint8_t s_group_mac[6] = {0x01, 0x00, 0x5e, 0x40, 0x29, 0x01};
static uint16_t s_sequence;
static uint32_t s_timestamp;
static uint32_t s_ssrc;

static uint16_t read_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t read_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

static void write_be16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

static void write_be32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)value;
}

static bool valid_talkgroup(uint16_t port)
{
    return halow_sync_is_public_channel(port);
}

static uint16_t ipv4_checksum(const uint8_t *data, size_t len)
{
    uint32_t sum = 0;
    while (len >= 2) {
        sum += read_be16(data);
        data += 2;
        len -= 2;
    }
    while (sum >> 16) sum = (sum & 0xffffU) + (sum >> 16);
    return (uint16_t)~sum;
}

esp_err_t openmanet_comms_send_phone_frame(const uint8_t *payload,
                                            size_t payload_len)
{
    if (!payload || payload_len <= PHONE_TX_HEADER_LEN ||
        payload_len > PHONE_TX_HEADER_LEN + MAX_OPUS_PAYLOAD ||
        memcmp(payload, s_magic, sizeof(s_magic)) != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t port = read_be16(payload + OPENMANET_COMMS_MAGIC_LEN);
    if (!valid_talkgroup(port)) return ESP_ERR_INVALID_ARG;
    if (!halow_sync_public_channel_enabled(port)) return ESP_ERR_INVALID_STATE;

    uint8_t self_mac[6];
    if (!halow_interface_app_get_self_mac(self_mac)) return ESP_ERR_INVALID_STATE;
    if (s_ssrc == 0) {
        s_ssrc = ((uint32_t)self_mac[2] << 24) |
                 ((uint32_t)self_mac[3] << 16) |
                 ((uint32_t)self_mac[4] << 8) | self_mac[5];
        if (s_ssrc == 0) s_ssrc = esp_random();
        s_sequence = (uint16_t)esp_random();
        s_timestamp = esp_random();
    }

    const size_t opus_len = payload_len - PHONE_TX_HEADER_LEN;
    uint8_t frame[ETH_LEN + IPV4_LEN + UDP_LEN + RTP_LEN + MAX_OPUS_PAYLOAD] = {0};
    memcpy(frame, s_group_mac, sizeof(s_group_mac));
    memcpy(frame + 6, self_mac, 6);
    write_be16(frame + 12, ETHERTYPE_IPV4);

    uint8_t *ip = frame + ETH_LEN;
    ip[0] = 0x45;
    write_be16(ip + 2, (uint16_t)(IPV4_LEN + UDP_LEN + RTP_LEN + opus_len));
    write_be16(ip + 4, ++s_sequence);
    ip[8] = 1;
    ip[9] = IP_PROTOCOL_UDP;
    ip[12] = 169;
    ip[13] = 254;
    ip[14] = self_mac[4];
    ip[15] = self_mac[5];
    memcpy(ip + 16, s_group_ip, sizeof(s_group_ip));
    write_be16(ip + 10, ipv4_checksum(ip, IPV4_LEN));

    uint8_t *udp = ip + IPV4_LEN;
    write_be16(udp, port);
    write_be16(udp + 2, port);
    write_be16(udp + 4, (uint16_t)(UDP_LEN + RTP_LEN + opus_len));
    /* A zero UDP checksum is valid for IPv4 and matches ordinary UDP stacks. */

    uint8_t *rtp = udp + UDP_LEN;
    rtp[0] = 0x80;
    rtp[1] = RTP_PAYLOAD_TYPE_OPUS;
    write_be16(rtp + 2, s_sequence);
    write_be32(rtp + 4, s_timestamp);
    write_be32(rtp + 8, s_ssrc);
    memcpy(rtp + RTP_LEN, payload + PHONE_TX_HEADER_LEN, opus_len);
    s_timestamp += RTP_FRAME_SAMPLES;

    return halow_interface_app_send_batman_broadcast_payload(
        frame, ETH_LEN + IPV4_LEN + UDP_LEN + RTP_LEN + opus_len);
}

bool openmanet_comms_handle_frame(const uint8_t *frame, size_t frame_len)
{
    if (!frame || frame_len < ETH_LEN + IPV4_LEN + UDP_LEN + RTP_LEN + 1 ||
        read_be16(frame + 12) != ETHERTYPE_IPV4) return false;
    const uint8_t *ip = frame + ETH_LEN;
    const size_t ihl = (size_t)(ip[0] & 0x0fU) * 4U;
    if ((ip[0] >> 4) != 4 || ihl < IPV4_LEN || frame_len < ETH_LEN + ihl + UDP_LEN ||
        ip[9] != IP_PROTOCOL_UDP || memcmp(ip + 16, s_group_ip, sizeof(s_group_ip)) != 0) {
        return false;
    }
    const uint8_t *udp = ip + ihl;
    uint16_t port = read_be16(udp + 2);
    uint16_t udp_len = read_be16(udp + 4);
    if (!valid_talkgroup(port) || udp_len < UDP_LEN + RTP_LEN + 1 ||
        ETH_LEN + ihl + udp_len > frame_len) return false;
    if (!halow_sync_public_channel_enabled(port)) return true;
    const uint8_t *rtp = udp + UDP_LEN;
    if ((rtp[0] >> 6) != 2 || (rtp[1] & 0x7fU) != RTP_PAYLOAD_TYPE_OPUS) return true;
    size_t rtp_header_len = RTP_LEN + (size_t)(rtp[0] & 0x0fU) * 4U;
    if ((rtp[0] & 0x10U) != 0) {
        if (udp_len < UDP_LEN + rtp_header_len + 4) return true;
        rtp_header_len += 4U + (size_t)read_be16(rtp + rtp_header_len + 2) * 4U;
    }
    if (udp_len <= UDP_LEN + rtp_header_len) return true;
    size_t opus_len = udp_len - UDP_LEN - rtp_header_len;
    if (opus_len > MAX_OPUS_PAYLOAD) return true;
    uint8_t notify[PHONE_RX_HEADER_LEN + MAX_OPUS_PAYLOAD];
    memcpy(notify, s_magic, sizeof(s_magic));
    write_be16(notify + 4, port);
    write_be32(notify + 6, read_be32(rtp + 8));
    write_be16(notify + 10, read_be16(rtp + 2));
    memcpy(notify + PHONE_RX_HEADER_LEN, rtp + rtp_header_len, opus_len);
    halow_sync_bridge_send_realtime_frame(notify,
                                          (uint16_t)(PHONE_RX_HEADER_LEN + opus_len));
    return true;
}
