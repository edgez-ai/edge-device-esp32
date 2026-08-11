#pragma once

#include <stddef.h>
#include <stdint.h>

extern "C" {
#include "mmwlan.h"
}
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
    bool setMeshVendorIes(const uint8_t *ies, size_t ies_len);

    EdgezRadioError sendVendorPayload(const uint8_t *payload, size_t payload_len);
    EdgezRadioError sendVendorPayloadTo(const uint8_t dest_mac[6], const uint8_t *payload, size_t payload_len);
    EdgezRadioError sendVendorPayloadToVia(const uint8_t dest_mac[6],
                                           const uint8_t next_hop_mac[6],
                                           const uint8_t *payload,
                                           size_t payload_len);
    EdgezRadioError sendPeerIndependentBeacon(const uint8_t *payload, size_t payload_len);

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
    bool vendorIeFilterActive = false;
    uint32_t lastScanMs = 0;
    uint32_t lastMeshInfoMs = 0;
    uint32_t lastMeshStatusLogMs = 0;
    uint32_t nextProactiveJoinScanMs = 0;
    uint32_t proactiveJoinScanAttempts = 0;
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
                                              uint16_t peer_independent_ethertype);
    int32_t runOnce();

    bool applyChannelList();
    bool startMeshStation();
    void startBackgroundTask();
    void stopBackgroundTask();

    uint8_t meshScanIes[2 + MMWLAN_SSID_MAXLEN] = {0};
    struct mmwlan_scan_req meshScanReq = MMWLAN_SCAN_REQ_INIT;
    TaskHandle_t backgroundTask = nullptr;
    volatile bool backgroundTaskStopRequested = false;

    void onMeshScanResult(const struct mmwlan_scan_result *result);
    void onMeshScanComplete(enum mmwlan_scan_state scan_state);
    void onStaEvent(const struct mmwlan_sta_event_cb_args *sta_event);
    bool onMeshPeerAdmission(const struct mmwlan_scan_result *result);

    // Trampoline registered with mmwlan_register_rx_cb. The callback hands us
    // the 802.3 header and payload separately.
    static void rxTrampoline(uint8_t *header, unsigned header_len, uint8_t *payload, unsigned payload_len, void *arg);
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
