#pragma once

#include <stddef.h>
#include <stdint.h>

extern "C" {
#include "mmwlan.h"
}
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "edgez_batman_adv_lite.h"

/**
 * EdgeZ HaLow (802.11ah) transport implemented directly on the mmwlan
 * frame-level API.
 *
 * Frames go out as 802.3 payloads (broadcast MAC, EtherType 0x88B5) carrying
 * EdgeZ protobuf/vendor payloads directly.
 *
 * Mesh discovery uses standard 802.11 mesh advertisement IEs. The AP path is
 * intentionally not used for this transport.
 */
typedef int EdgezRadioError;

enum {
    EDGEZ_RADIO_OK = 0,
    EDGEZ_RADIO_ERROR = 32,
    EDGEZ_RADIO_RETRY = 33,
    EDGEZ_RADIO_DISABLED = 34,
};

class HaLowInterface
{
  public:
    HaLowInterface() = default;

    bool init();
    bool reconfigure();
    bool sleep();
    bool shutdown();
    void setCountryCode(const char *country_code);
    void setMeshId(const char *mesh_id);
    void setMeshSaePassphrase(const char *passphrase);
    void setMeshRadio(uint32_t frequency_khz, uint8_t bandwidth_mhz);
    void setBeaconOnly(bool enabled);
    void setProactiveJoinEnabled(bool enabled);
    void setRelayModeEnabled(bool enabled);
    bool setMeshVendorIes(const uint8_t *ies, size_t ies_len);
    void noteForegroundTraffic();
    bool foregroundTrafficActive() const;

    EdgezRadioError sendVendorPayload(const uint8_t *payload, size_t payload_len);
    EdgezRadioError sendVendorPayloadTo(const uint8_t dest_mac[6], const uint8_t *payload, size_t payload_len);
    EdgezRadioError sendVendorPayloadToVia(const uint8_t dest_mac[6],
                                           const uint8_t next_hop_mac[6],
                                           const uint8_t *payload,
                                           size_t payload_len);
    EdgezRadioError sendPeerIndependentBeacon(const uint8_t *payload, size_t payload_len);
    EdgezRadioError sendBatmanPayloadTo(const uint8_t destination[6],
                                        const uint8_t *payload,
                                        size_t payload_len,
                                        const uint8_t forced_next_hop[6] = nullptr);
    EdgezRadioError sendBatmanBroadcastPayload(const uint8_t *payload,
                                                size_t payload_len);
    void receiveBatmanAdv(const uint8_t ethernet_source[6],
                          const uint8_t *payload,
                          size_t payload_len);
    bool lookupBatmanRoute(const uint8_t destination[6],
                           uint8_t next_hop[6],
                           uint8_t *hop_count = nullptr,
                           uint8_t *tq = nullptr,
                           uint32_t *route_age_ms = nullptr);
    size_t snapshotBatmanRoutes(edgez_batadv_route_snapshot_t *routes,
                                size_t capacity);
    bool selectBatmanDirectPeer(const uint8_t exclude_a[6],
                                const uint8_t exclude_b[6],
                                const uint8_t exclude_c[6],
                                uint64_t selection_key,
                                uint8_t peer[6]);

  private:
    static constexpr uint8_t WLAN_IE_ID_MESH_CONFIG = 113;
    static constexpr uint8_t WLAN_IE_ID_MESH_ID = 114;

    volatile bool linkUp = false;
    volatile bool scanInProgress = false;
    volatile bool meshPeerSeen = false;
    volatile bool meshControlPortOpen = false;
    bool wlanReady = false;
    bool meshEnabled = false;
    bool beaconOnly = false;
    bool proactiveJoinEnabled = false;
    bool relayModeEnabled = false;
    bool vendorIeFilterActive = false;
    uint32_t lastScanMs = 0;
    uint32_t lastMeshInfoMs = 0;
    uint32_t lastMeshStatusLogMs = 0;
    uint32_t nextProactiveJoinScanMs = 0;
    uint32_t proactiveJoinScanAttempts = 0;
    volatile uint32_t lastForegroundTrafficMs = 0;
    uint32_t staEventCount = 0;
    uint32_t staScanCount = 0;
    uint32_t staScanResultCount = 0;
    uint32_t staMeshAdvSeenCount = 0;
    uint32_t staTargetIdHitCount = 0;
    uint32_t staAuthReqCount = 0;
    uint32_t staAssocReqCount = 0;
    uint32_t staCtrlPortOpenCount = 0;
    int16_t bestMeshRssi = -32768;
    uint8_t bestMeshBssid[6] = {0};
    char bestMeshId[33] = {0};
    char meshId[33] = {0};
    char meshKey[65] = {0};
    char countryCode[3] = {0};
    uint32_t meshFrequencyKHz = 0;
    uint8_t meshBandwidthMHz = 0;

    bool startMeshInfoRequest();
    bool loadMeshProfile();
    EdgezRadioError sendVendorPayloadInternal(const uint8_t dest_mac[6],
                                              const uint8_t next_hop_mac[6],
                                              const uint8_t *payload,
                                              size_t payload_len,
                                              uint16_t ethertype,
                                              bool peer_independent,
                                              uint32_t tx_ready_timeout_ms);
    EdgezRadioError sendBatmanAdvPayload(const uint8_t *payload, size_t payload_len,
                                         const uint8_t exclude_peer[6] = nullptr,
                                         uint32_t tx_ready_timeout_ms = 50);
    void runBatmanAdv(uint32_t now_ms);
    int32_t runOnce();

    bool applyChannelList();
    bool startMeshStation();
    void startBackgroundTask();
    void stopBackgroundTask();

    uint8_t meshScanIes[2 + MMWLAN_SSID_MAXLEN] = {0};
    struct mmwlan_scan_req meshScanReq = MMWLAN_SCAN_REQ_INIT;
    TaskHandle_t backgroundTask = nullptr;
    volatile bool backgroundTaskStopRequested = false;
    volatile bool txPathReady = true;
    static constexpr uint32_t FOREGROUND_TX_READY_TIMEOUT_MS = 50;

#if defined(CONFIG_MM_BATMAN_ADV_LITE) && CONFIG_MM_BATMAN_ADV_LITE
    static constexpr size_t BATMAN_FORWARD_QUEUE_DEPTH = 48;
    static constexpr uint8_t BATMAN_FORWARD_MAX_ATTEMPTS = 3;
    static constexpr uint8_t BATMAN_FORWARD_BATCH_SIZE = 10;
    static constexpr uint32_t BATMAN_FORWARD_TX_READY_TIMEOUT_MS = 20;
    static constexpr uint32_t BATMAN_FORWARD_RETRY_BACKOFF_MS = 10;
    struct BatmanForwardFrame {
        uint8_t data[EDGEZ_BATADV_MAX_PACKET_LEN] = {0};
        size_t len = 0;
        bool directed = false;
        bool exclude_valid = false;
        uint8_t next_hop[6] = {0};
        uint8_t exclude_peer[6] = {0};
        uint8_t attempts = 0;
    };
    edgez_batadv_lite_t batmanAdv = {};
    portMUX_TYPE batmanLock = portMUX_INITIALIZER_UNLOCKED;
    /* Payload storage is allocated in PSRAM during init. Keeping this array
     * inside the global HaLowInterface object consumed about 25 KiB of scarce
     * internal BSS at the 48-frame depth and could starve SAE/AES and NimBLE. */
    BatmanForwardFrame *batmanForwardQueue = nullptr;
    size_t batmanForwardHead = 0;
    size_t batmanForwardTail = 0;
    size_t batmanForwardCount = 0;
    size_t batmanForwardHighWater = 0;
    uint32_t batmanTxAccepted = 0;
    uint32_t batmanTxFailed = 0;
    uint32_t batmanTxRetryAttempts = 0;
    uint32_t batmanTxRecovered = 0;
    uint32_t batmanRxValid = 0;
    uint32_t batmanForwardDropped = 0;
    uint8_t batmanAuthorizedPeers[EDGEZ_BATADV_MAX_PEERS][6] = {{0}};
    uint8_t batmanAuthorizedPeerCount = 0;
#endif

    void onMeshScanResult(const struct mmwlan_scan_result *result);
    void onMeshScanComplete(enum mmwlan_scan_state scan_state);
    void onStaEvent(const struct mmwlan_sta_event_cb_args *sta_event);
    bool onMeshPeerAdmission(const struct mmwlan_scan_result *result);

    // Legacy RX trampoline. The extended packet callback below is used in
    // mesh mode because BATMAN route learning needs the actual 802.11 TA.
    static void rxTrampoline(uint8_t *header, unsigned header_len, uint8_t *payload, unsigned payload_len, void *arg);
    static void rxPktExtTrampoline(struct mmpkt *packet,
                                   const struct mmwlan_rx_metadata *metadata,
                                   void *arg);
    void onRxFrame(uint8_t *header, unsigned header_len, uint8_t *payload,
                   unsigned payload_len, const uint8_t transmitter[6]);
    static void linkStateTrampoline(enum mmwlan_link_state link_state, void *arg);
    static void scanRxTrampoline(const struct mmwlan_scan_result *result, void *arg);
    static void vendorIeFilterTrampoline(const uint8_t *ies,
                                         uint32_t ies_len,
                                         const uint8_t *bssid,
                                         void *arg);
    static bool meshPeerAdmissionTrampoline(const struct mmwlan_scan_result *result,
                                            void *arg);
    static void scanCompleteTrampoline(enum mmwlan_scan_state scan_state, void *arg);
    static void staEventTrampoline(const struct mmwlan_sta_event_cb_args *sta_event, void *arg);
    static void txFlowControlTrampoline(enum mmwlan_tx_flow_control_state state,
                                        void *arg);
    static void backgroundTaskTrampoline(void *arg);

    static constexpr uint32_t MESH_STATUS_LOG_INTERVAL_MS = 10000;
    static constexpr uint32_t PROACTIVE_JOIN_SCAN_RETRY_MS = 10000;
    static constexpr uint16_t MESH_CONNECT_SCAN_BASE_S = 60;
    static constexpr uint16_t MESH_CONNECT_SCAN_LIMIT_S = 600;
    // Approximate bytes-per-millisecond at the configured channel width / MCS.
    // HaLow is 150 kbps to 32.5 Mbps depending on configuration — picking a
    // single value is fiction, but airtime accounting needs *something*, and
    // duty cycle isn't the constraint on HaLow that it is on LoRa.
    static constexpr uint32_t HALOW_NOMINAL_KBPS = 1000; // 1 Mbps, 2 MHz MCS3 ballpark
};
