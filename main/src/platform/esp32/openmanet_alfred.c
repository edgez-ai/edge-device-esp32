#include "openmanet_alfred.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "halow_interface_app.h"
#include "l76k_gps.h"
#include "openmanet/network/v1/node.pb.h"
#include "pb_encode.h"

enum {
    ETHERNET_HEADER_LEN = 14,
    IPV6_HEADER_LEN = 40,
    UDP_HEADER_LEN = 8,
    ALFRED_TLV_LEN = 4,
    ALFRED_PUSH_HEADER_LEN = 8,
    ALFRED_DATA_HEADER_LEN = 10,
    ALFRED_PORT = 0x4242,
    ALFRED_PUSH_DATA = 0,
    ALFRED_ANNOUNCE_PRIMARY = 1,
    ALFRED_STATUS_TXEND = 3,
    ALFRED_VERSION = 0,
    ALFRED_GPSD_TYPE = 2,
    ALFRED_GPSD_VERSION = 1,
    ALFRED_BAT_HOSTS_TYPE = 64,
    OPENMANET_NODE_TYPE = 102,
    OPENMANET_NODE_VERSION = 1,
    PUBLISH_INTERVAL_MS = 60000,
    IP_PROTOCOL_UDP = 17,
    ETHERTYPE_IPV6 = 0x86dd,
};

static const char *TAG = "openmanet_alfred";
static char s_hostname[65] = "EdgeZ-HaLow";
static uint32_t s_last_publish_ms;
static bool s_has_published;
static uint16_t s_transaction_id;

static uint16_t read_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
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

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void mac_to_link_local(const uint8_t mac[6], uint8_t address[16])
{
    memset(address, 0, 16);
    address[0] = 0xfe;
    address[1] = 0x80;
    address[8] = mac[0] ^ 0x02U;
    address[9] = mac[1];
    address[10] = mac[2];
    address[11] = 0xff;
    address[12] = 0xfe;
    address[13] = mac[3];
    address[14] = mac[4];
    address[15] = mac[5];
}

static uint32_t checksum_add(uint32_t sum, const uint8_t *data, size_t len)
{
    while (len >= 2) {
        sum += read_be16(data);
        data += 2;
        len -= 2;
    }
    if (len) {
        sum += (uint16_t)data[0] << 8;
    }
    return sum;
}

static uint16_t checksum_finish(uint32_t sum)
{
    while (sum >> 16) {
        sum = (sum & 0xffffU) + (sum >> 16);
    }
    uint16_t result = (uint16_t)~sum;
    return result ? result : 0xffffU;
}

static uint16_t udp6_checksum(const uint8_t source[16],
                              const uint8_t destination[16],
                              const uint8_t *udp, size_t udp_len)
{
    uint32_t sum = 0;
    uint8_t pseudo_tail[8] = {0};
    write_be32(pseudo_tail, (uint32_t)udp_len);
    pseudo_tail[7] = IP_PROTOCOL_UDP;
    sum = checksum_add(sum, source, 16);
    sum = checksum_add(sum, destination, 16);
    sum = checksum_add(sum, pseudo_tail, sizeof(pseudo_tail));
    sum = checksum_add(sum, udp, udp_len);
    return checksum_finish(sum);
}

static esp_err_t send_udp(const uint8_t route_mac[6],
                          const uint8_t peer_mac[6],
                          const uint8_t peer_ip[16],
                          const uint8_t *payload, size_t payload_len)
{
    uint8_t self_mac[6] = {0};
    if (!halow_interface_app_get_self_mac(self_mac)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t frame[ETHERNET_HEADER_LEN + IPV6_HEADER_LEN + UDP_HEADER_LEN + 480] = {0};
    if (payload_len > sizeof(frame) - ETHERNET_HEADER_LEN - IPV6_HEADER_LEN - UDP_HEADER_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t self_ip[16];
    mac_to_link_local(self_mac, self_ip);
    memcpy(frame, peer_mac, 6);
    memcpy(frame + 6, self_mac, 6);
    write_be16(frame + 12, ETHERTYPE_IPV6);

    uint8_t *ipv6 = frame + ETHERNET_HEADER_LEN;
    ipv6[0] = 0x60;
    write_be16(ipv6 + 4, (uint16_t)(UDP_HEADER_LEN + payload_len));
    ipv6[6] = IP_PROTOCOL_UDP;
    ipv6[7] = 1;
    memcpy(ipv6 + 8, self_ip, 16);
    memcpy(ipv6 + 24, peer_ip, 16);

    uint8_t *udp = ipv6 + IPV6_HEADER_LEN;
    write_be16(udp, ALFRED_PORT);
    write_be16(udp + 2, ALFRED_PORT);
    write_be16(udp + 4, (uint16_t)(UDP_HEADER_LEN + payload_len));
    memcpy(udp + UDP_HEADER_LEN, payload, payload_len);
    write_be16(udp + 6, udp6_checksum(self_ip, peer_ip, udp,
                                      UDP_HEADER_LEN + payload_len));

    return halow_interface_app_send_batman_payload(
        route_mac, frame, ETHERNET_HEADER_LEN + IPV6_HEADER_LEN +
                         UDP_HEADER_LEN + payload_len);
}

static esp_err_t publish_record(const uint8_t route_mac[6],
                                const uint8_t peer_mac[6],
                                const uint8_t peer_ip[16], uint8_t data_type,
                                uint8_t data_version, const uint8_t *data,
                                size_t data_len)
{
    uint8_t self_mac[6] = {0};
    uint8_t packet[ALFRED_PUSH_HEADER_LEN + ALFRED_DATA_HEADER_LEN + 384] = {0};
    if (!data || data_len > sizeof(packet) - ALFRED_PUSH_HEADER_LEN -
                                      ALFRED_DATA_HEADER_LEN ||
        !halow_interface_app_get_self_mac(self_mac)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t tx_id = ++s_transaction_id;
    if (tx_id == 0) {
        tx_id = ++s_transaction_id;
    }

    packet[0] = ALFRED_PUSH_DATA;
    packet[1] = ALFRED_VERSION;
    write_be16(packet + 2, (uint16_t)(4 + ALFRED_DATA_HEADER_LEN + data_len));
    write_be16(packet + 4, tx_id);
    write_be16(packet + 6, 0);
    memcpy(packet + 8, self_mac, 6);
    packet[14] = data_type;
    packet[15] = data_version;
    write_be16(packet + 16, (uint16_t)data_len);
    memcpy(packet + 18, data, data_len);

    esp_err_t err = send_udp(route_mac, peer_mac, peer_ip, packet,
                             ALFRED_PUSH_HEADER_LEN + ALFRED_DATA_HEADER_LEN +
                                 data_len);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t tx_end[8] = {ALFRED_STATUS_TXEND, ALFRED_VERSION, 0, 4};
    write_be16(tx_end + 4, tx_id);
    write_be16(tx_end + 6, 1);
    return send_udp(route_mac, peer_mac, peer_ip, tx_end, sizeof(tx_end));
}

static esp_err_t publish_bat_hosts(const uint8_t route_mac[6],
                                   const uint8_t peer_mac[6],
                                   const uint8_t peer_ip[16],
                                   const uint8_t self_mac[6])
{
    char host_label[sizeof(s_hostname)];
    strlcpy(host_label, s_hostname, sizeof(host_label));
    for (size_t i = 0; host_label[i]; ++i) {
        if (host_label[i] == ' ' || host_label[i] == '\t' ||
            host_label[i] == '\r' || host_label[i] == '\n') {
            host_label[i] = '_';
        }
    }
    char value[128];
    int len = snprintf(value, sizeof(value),
                       "%02x:%02x:%02x:%02x:%02x:%02x %s_halow0\n",
                       self_mac[0], self_mac[1], self_mac[2], self_mac[3],
                       self_mac[4], self_mac[5], host_label);
    if (len <= 0 || (size_t)len >= sizeof(value)) {
        return ESP_ERR_INVALID_SIZE;
    }
    return publish_record(route_mac, peer_mac, peer_ip,
                          ALFRED_BAT_HOSTS_TYPE, 0,
                          (const uint8_t *)value, (size_t)len);
}

static bool read_location(float *latitude, float *longitude)
{
    uint64_t timestamp_ms = 0;
    return l76k_gps_get_latest(latitude, longitude, &timestamp_ms);
}

static esp_err_t publish_node(const uint8_t route_mac[6],
                              const uint8_t peer_mac[6],
                              const uint8_t peer_ip[16],
                              const uint8_t self_mac[6], bool has_location,
                              float latitude, float longitude)
{
    openmanet_network_v1_Node node = openmanet_network_v1_Node_init_zero;
    snprintf(node.mac, sizeof(node.mac),
             "%02x:%02x:%02x:%02x:%02x:%02x", self_mac[0], self_mac[1],
             self_mac[2], self_mac[3], self_mac[4], self_mac[5]);
    strlcpy(node.hostname, s_hostname, sizeof(node.hostname));
    if (has_location) {
        node.has_position = true;
        node.position.latitude = latitude;
        node.position.longitude = longitude;
    }

    uint8_t encoded[openmanet_network_v1_Node_size] = {0};
    pb_ostream_t stream = pb_ostream_from_buffer(encoded, sizeof(encoded));
    if (!pb_encode(&stream, openmanet_network_v1_Node_fields, &node)) {
        ESP_LOGW(TAG, "Node protobuf encode failed: %s", PB_GET_ERROR(&stream));
        return ESP_FAIL;
    }
    return publish_record(route_mac, peer_mac, peer_ip, OPENMANET_NODE_TYPE,
                          OPENMANET_NODE_VERSION, encoded,
                          stream.bytes_written);
}

static esp_err_t publish_gps(const uint8_t route_mac[6],
                             const uint8_t peer_mac[6],
                             const uint8_t peer_ip[16], float latitude,
                             float longitude)
{
    uint8_t value[192] = {0};
    char *json = (char *)(value + 4);
    int json_len = snprintf(json, sizeof(value) - 4,
                            "{\"class\":\"TPV\",\"device\":\"L76K\","
                            "\"lat\":%.7f,\"lon\":%.7f,\"mode\":3}",
                            (double)latitude, (double)longitude);
    if (json_len <= 0 || (size_t)json_len + 5 > sizeof(value)) {
        return ESP_ERR_INVALID_SIZE;
    }
    write_be32(value, (uint32_t)json_len + 1U);
    return publish_record(route_mac, peer_mac, peer_ip, ALFRED_GPSD_TYPE,
                          ALFRED_GPSD_VERSION, value,
                          (size_t)json_len + 5U);
}

static void publish_all(const uint8_t route_mac[6],
                        const uint8_t peer_mac[6], const uint8_t peer_ip[16])
{
    uint8_t self_mac[6] = {0};
    if (!halow_interface_app_get_self_mac(self_mac)) {
        return;
    }
    float latitude = 0.0f;
    float longitude = 0.0f;
    bool has_location = read_location(&latitude, &longitude);

    esp_err_t host_err = publish_bat_hosts(route_mac, peer_mac, peer_ip,
                                           self_mac);
    esp_err_t node_err = publish_node(route_mac, peer_mac, peer_ip, self_mac,
                                      has_location, latitude, longitude);
    esp_err_t gps_err = has_location
                            ? publish_gps(route_mac, peer_mac, peer_ip,
                                          latitude, longitude)
                            : ESP_OK;
    if (host_err == ESP_OK && node_err == ESP_OK && gps_err == ESP_OK) {
        ESP_LOGI(TAG,
                 "Published Alfred hostname='%s' GPS=%u to %02x:%02x:%02x:%02x:%02x:%02x",
                 s_hostname, has_location ? 1U : 0U, route_mac[0], route_mac[1],
                 route_mac[2], route_mac[3], route_mac[4], route_mac[5]);
    } else {
        ESP_LOGW(TAG, "Alfred publish failed hosts=%s node=%s gps=%s",
                 esp_err_to_name(host_err), esp_err_to_name(node_err),
                 esp_err_to_name(gps_err));
    }
}

void openmanet_alfred_set_hostname(const char *hostname)
{
    if (hostname && hostname[0]) {
        strlcpy(s_hostname, hostname, sizeof(s_hostname));
    }
}

void openmanet_alfred_handle_frame(const uint8_t originator[6],
                                   const uint8_t *frame, size_t frame_len)
{
    if (!originator || !frame || frame_len < ETHERNET_HEADER_LEN + IPV6_HEADER_LEN +
                                  UDP_HEADER_LEN + ALFRED_TLV_LEN ||
        read_be16(frame + 12) != ETHERTYPE_IPV6) {
        return;
    }
    const uint8_t *ipv6 = frame + ETHERNET_HEADER_LEN;
    if ((ipv6[0] >> 4) != 6 || ipv6[6] != IP_PROTOCOL_UDP ||
        ipv6[8] != 0xfe || (ipv6[9] & 0xc0U) != 0x80U) {
        return;
    }
    size_t ipv6_payload_len = read_be16(ipv6 + 4);
    if (ipv6_payload_len < UDP_HEADER_LEN + ALFRED_TLV_LEN ||
        ipv6_payload_len > frame_len - ETHERNET_HEADER_LEN - IPV6_HEADER_LEN) {
        return;
    }
    const uint8_t *udp = ipv6 + IPV6_HEADER_LEN;
    size_t udp_len = read_be16(udp + 4);
    if (read_be16(udp + 2) != ALFRED_PORT ||
        udp_len != ipv6_payload_len || udp_len < UDP_HEADER_LEN + ALFRED_TLV_LEN) {
        return;
    }
    const uint8_t *alfred = udp + UDP_HEADER_LEN;
    if (alfred[0] != ALFRED_ANNOUNCE_PRIMARY ||
        alfred[1] != ALFRED_VERSION || read_be16(alfred + 2) != 0) {
        return;
    }

    uint32_t now = now_ms();
    if (s_has_published &&
        (uint32_t)(now - s_last_publish_ms) < PUBLISH_INTERVAL_MS) {
        return;
    }
    s_last_publish_ms = now;
    s_has_published = true;
    if (s_transaction_id == 0) {
        s_transaction_id = (uint16_t)esp_random();
    }
    /* The encapsulated Ethernet source is br-ahwlan, while BATMAN routes to
     * the broadcaster's bat0 originator. Keep both identities distinct. */
    publish_all(originator, frame + 6, ipv6 + 8);
}
