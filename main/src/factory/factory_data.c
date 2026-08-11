#include "factory_data.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_app_desc.h"
#include "esp_mac.h"
#include "esp_partition.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/sha256.h"
#include "nvs.h"
#include "pb_decode.h"
#include "factory_data.pb.h"

static const char *TAG = "factory_data";
static const char s_edgez_license_public_key_hex[] =
    "04A3D2F7675C99D6263A0C8E4CC077DE26B7E19F2D3D7E62486EA59232FCD977"
    "0046A9AE2F3A988C3E6071E3255A8BE7BE5376ED175266A0ADA344828E3B5A837A";

#define FACTORY_DATA_NVS_NAMESPACE "factory_data"
#define FACTORY_DATA_NVS_BLOB_KEY "pb_blob"
#define FACTORY_DATA_PARTITION_LABEL "factory_data"
#define FACTORY_DATA_READ_MAX 512
#define EDGEZ_LICENSE_SIGNATURE_SIZE 64
#define EDGEZ_LICENSE_PUBLIC_KEY_SIZE 65
#define EDGEZ_SDK_RELEASE_PREFIX "EDGEZ-FLUTTER-SDK-RELEASE-V1:"
#define EDGEZ_SDK_RELEASE_ID_MAX 32
#define EDGEZ_SDK_COMPATIBILITY_MAX 32

static int s_license_valid = -1;
static uint8_t s_license_bound_mac[6];
static uint8_t s_license_digest[32];
static uint32_t s_license_capability_tags[5];
static edgez_sdk_release_auth_state_t s_sdk_release_auth_state =
    EDGEZ_SDK_RELEASE_AUTHORIZED;

typedef struct {
    uint32_t major;
    uint32_t minor;
    uint32_t patch;
} edgez_semver_t;

static bool sdk_parse_uint32(const char **cursor, uint32_t *value)
{
    if (!cursor || !*cursor || !value || **cursor < '0' || **cursor > '9') {
        return false;
    }
    uint32_t parsed = 0;
    do {
        const uint32_t digit = (uint32_t)(**cursor - '0');
        if (parsed > (UINT32_MAX - digit) / 10U) {
            return false;
        }
        parsed = parsed * 10U + digit;
        ++(*cursor);
    } while (**cursor >= '0' && **cursor <= '9');
    *value = parsed;
    return true;
}

static bool sdk_parse_semver(const char **cursor, edgez_semver_t *version)
{
    if (!cursor || !*cursor || !version ||
        !sdk_parse_uint32(cursor, &version->major) || **cursor != '.') {
        return false;
    }
    ++(*cursor);
    if (!sdk_parse_uint32(cursor, &version->minor) || **cursor != '.') {
        return false;
    }
    ++(*cursor);
    return sdk_parse_uint32(cursor, &version->patch);
}

static int sdk_compare_semver(const edgez_semver_t *left,
                              const edgez_semver_t *right)
{
    if (left->major != right->major) return left->major < right->major ? -1 : 1;
    if (left->minor != right->minor) return left->minor < right->minor ? -1 : 1;
    if (left->patch != right->patch) return left->patch < right->patch ? -1 : 1;
    return 0;
}

static bool sdk_semver_upper_bound(const edgez_semver_t *minimum,
                                   bool caret,
                                   edgez_semver_t *maximum)
{
    *maximum = *minimum;
    if (!caret) {
        if (maximum->minor == UINT32_MAX) return false;
        ++maximum->minor;
        maximum->patch = 0;
        return true;
    }
    if (maximum->major > 0) {
        if (maximum->major == UINT32_MAX) return false;
        ++maximum->major;
        maximum->minor = 0;
        maximum->patch = 0;
    } else if (maximum->minor > 0) {
        if (maximum->minor == UINT32_MAX) return false;
        ++maximum->minor;
        maximum->patch = 0;
    } else {
        if (maximum->patch == UINT32_MAX) return false;
        ++maximum->patch;
    }
    return true;
}

static bool sdk_semver_matches_comparators(const edgez_semver_t *firmware,
                                           const char *range)
{
    const char *cursor = range;
    bool saw_comparator = false;
    while (*cursor != '\0') {
        enum { OP_EQ, OP_LT, OP_LTE, OP_GT, OP_GTE } operation = OP_EQ;
        if (strncmp(cursor, ">=", 2) == 0) {
            operation = OP_GTE;
            cursor += 2;
        } else if (strncmp(cursor, "<=", 2) == 0) {
            operation = OP_LTE;
            cursor += 2;
        } else if (*cursor == '>') {
            operation = OP_GT;
            ++cursor;
        } else if (*cursor == '<') {
            operation = OP_LT;
            ++cursor;
        } else if (*cursor == '=') {
            ++cursor;
        }

        edgez_semver_t required = {0};
        if (!sdk_parse_semver(&cursor, &required) ||
            (*cursor != '\0' && *cursor != ' ')) {
            return false;
        }
        const int comparison = sdk_compare_semver(firmware, &required);
        const bool matches =
            (operation == OP_EQ && comparison == 0) ||
            (operation == OP_LT && comparison < 0) ||
            (operation == OP_LTE && comparison <= 0) ||
            (operation == OP_GT && comparison > 0) ||
            (operation == OP_GTE && comparison >= 0);
        if (!matches) return false;
        saw_comparator = true;
        if (*cursor == ' ') {
            ++cursor;
            if (*cursor == '\0' || *cursor == ' ') return false;
        }
    }
    return saw_comparator;
}

static bool sdk_firmware_is_compatible(const char *compatibility)
{
    if (!compatibility || *compatibility == '\0') {
        return false;
    }

    const esp_app_desc_t *app = esp_app_get_description();
    if (!app || app->version[0] == '\0') {
        return false;
    }
    const char *firmware_cursor = app->version;
    if (*firmware_cursor == 'v' || *firmware_cursor == 'V') {
        ++firmware_cursor;
    }
    edgez_semver_t firmware = {0};
    if (!sdk_parse_semver(&firmware_cursor, &firmware)) {
        return false;
    }

    if (*compatibility == '^' || *compatibility == '~') {
        const bool caret = *compatibility == '^';
        const char *range_cursor = compatibility + 1;
        edgez_semver_t minimum = {0};
        edgez_semver_t maximum = {0};
        if (!sdk_parse_semver(&range_cursor, &minimum) ||
            *range_cursor != '\0' ||
            !sdk_semver_upper_bound(&minimum, caret, &maximum)) {
            return false;
        }
        return sdk_compare_semver(&firmware, &minimum) >= 0 &&
               sdk_compare_semver(&firmware, &maximum) < 0;
    }

    return sdk_semver_matches_comparators(&firmware, compatibility);
}

static int license_capability_index(edgez_license_capability_t capability)
{
    switch (capability) {
    case EDGEZ_LICENSE_CAP_RADIO_INIT: return 0;
    case EDGEZ_LICENSE_CAP_MESH_TX: return 1;
    case EDGEZ_LICENSE_CAP_MESH_RX: return 2;
    case EDGEZ_LICENSE_CAP_PROVISION: return 3;
    case EDGEZ_LICENSE_CAP_MOBILE_CONTROL: return 4;
    default: return -1;
    }
}

static uint32_t license_capability_tag(edgez_license_capability_t capability,
                                       const uint8_t digest[32],
                                       const uint8_t mac[6])
{
    uint32_t tag = 2166136261U ^ (uint32_t)capability;
    for (size_t i = 0; i < 32; ++i) {
        tag = (tag ^ digest[i]) * 16777619U;
    }
    for (size_t i = 0; i < 6; ++i) {
        tag = (tag ^ mac[i]) * 16777619U;
    }
    return tag ^ 0xE63A91D5U;
}

static esp_err_t decode_factory_data_blob(const uint8_t *blob,
                                          size_t blob_len,
                                          factory_data_config_t *out)
{
    if (!blob || blob_len == 0 || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    edge_device_factory_FactoryData pb = edge_device_factory_FactoryData_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(blob, blob_len);
    if (!pb_decode(&stream, edge_device_factory_FactoryData_fields, &pb)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    out->present = true;
    out->mode = (uint32_t)pb.mode;
    out->country = (uint32_t)pb.country;
    if (pb.serial_number[0] != '\0') {
        strncpy(out->serial_number, pb.serial_number, sizeof(out->serial_number) - 1);
        out->serial_number[sizeof(out->serial_number) - 1] = '\0';
    }
    if (pb.device_private_key.size > sizeof(out->device_private_key)) {
        return ESP_ERR_INVALID_SIZE;
    }
    out->device_private_key_len = pb.device_private_key.size;
    if (out->device_private_key_len > 0) {
        memcpy(out->device_private_key,
               pb.device_private_key.bytes,
               out->device_private_key_len);
    }

    if (pb.credential_signature.size > sizeof(out->credential_signature)) {
        return ESP_ERR_INVALID_SIZE;
    }
    out->credential_signature_len = pb.credential_signature.size;
    if (out->credential_signature_len > 0) {
        memcpy(out->credential_signature,
               pb.credential_signature.bytes,
               out->credential_signature_len);
    }

    if (pb.ssid[0] != '\0') {
        strncpy(out->ssid, pb.ssid, sizeof(out->ssid) - 1);
        out->ssid[sizeof(out->ssid) - 1] = '\0';
    }

    if (pb.passphrase[0] != '\0') {
        strncpy(out->passphrase, pb.passphrase, sizeof(out->passphrase) - 1);
        out->passphrase[sizeof(out->passphrase) - 1] = '\0';
    }

    if (pb.ble_pin_code[0] != '\0') {
        strncpy(out->ble_pin_code, pb.ble_pin_code, sizeof(out->ble_pin_code) - 1);
        out->ble_pin_code[sizeof(out->ble_pin_code) - 1] = '\0';
    }

    return ESP_OK;
}

static esp_err_t load_factory_data_from_partition(factory_data_config_t *out)
{
    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_ANY,
        FACTORY_DATA_PARTITION_LABEL);
    if (!partition) {
        return ESP_ERR_NOT_FOUND;
    }

    size_t read_len = partition->size;
    if (read_len > FACTORY_DATA_READ_MAX) {
        read_len = FACTORY_DATA_READ_MAX;
    }

    uint8_t raw[FACTORY_DATA_READ_MAX] = {0};
    esp_err_t err = esp_partition_read(partition, 0, raw, read_len);
    if (err != ESP_OK) {
        return err;
    }

    ssize_t last_non_ff = -1;
    for (size_t i = 0; i < read_len; i++) {
        if (raw[i] != 0xFF) {
            last_non_ff = (ssize_t)i;
        }
    }

    if (last_non_ff < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    size_t blob_len = (size_t)last_non_ff + 1;
    return decode_factory_data_blob(raw, blob_len, out);
}

static esp_err_t load_factory_data_from_nvs(factory_data_config_t *out)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(FACTORY_DATA_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    size_t blob_len = 0;
    err = nvs_get_blob(nvs_handle, FACTORY_DATA_NVS_BLOB_KEY, NULL, &blob_len);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return err;
    }

    if (blob_len == 0 || blob_len > FACTORY_DATA_READ_MAX) {
        nvs_close(nvs_handle);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t blob[FACTORY_DATA_READ_MAX] = {0};
    err = nvs_get_blob(nvs_handle, FACTORY_DATA_NVS_BLOB_KEY, blob, &blob_len);
    nvs_close(nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    return decode_factory_data_blob(blob, blob_len, out);
}

esp_err_t factory_data_load(factory_data_config_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));

    esp_err_t err = load_factory_data_from_partition(out);
    if (err == ESP_OK) {
        ESP_LOGI(TAG,
                 "Loaded factory data from partition (mode=%u country=%u)",
                 (unsigned)out->mode,
                 (unsigned)out->country);
        return ESP_OK;
    }

    // Backward-compatible fallback for older provisioning paths that persisted in NVS.
    err = load_factory_data_from_nvs(out);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "Factory data not found in partition or NVS");
        } else {
            ESP_LOGW(TAG, "Failed to load factory data: %s", esp_err_to_name(err));
        }
        return err;
    }

    ESP_LOGI(TAG,
             "Loaded factory data from NVS (mode=%u country=%u)",
             (unsigned)out->mode,
             (unsigned)out->country);

    return ESP_OK;
}

esp_err_t factory_data_country_to_code(const factory_data_config_t *cfg,
                                       char out_code[3])
{
    if (!cfg || !cfg->present || !out_code) {
        return ESP_ERR_INVALID_ARG;
    }

    switch ((edge_device_factory_CountryCode)cfg->country) {
    case edge_device_factory_CountryCode_COUNTRY_US:
        out_code[0] = 'U'; out_code[1] = 'S'; out_code[2] = '\0';
        return ESP_OK;
    case edge_device_factory_CountryCode_COUNTRY_CA:
        out_code[0] = 'C'; out_code[1] = 'A'; out_code[2] = '\0';
        return ESP_OK;
    case edge_device_factory_CountryCode_COUNTRY_JP:
        out_code[0] = 'J'; out_code[1] = 'P'; out_code[2] = '\0';
        return ESP_OK;
    case edge_device_factory_CountryCode_COUNTRY_AU:
        out_code[0] = 'A'; out_code[1] = 'U'; out_code[2] = '\0';
        return ESP_OK;
    case edge_device_factory_CountryCode_COUNTRY_NZ:
        out_code[0] = 'N'; out_code[1] = 'Z'; out_code[2] = '\0';
        return ESP_OK;
    case edge_device_factory_CountryCode_COUNTRY_EU:
        out_code[0] = 'E'; out_code[1] = 'U'; out_code[2] = '\0';
        return ESP_OK;
    case edge_device_factory_CountryCode_COUNTRY_UNSPECIFIED:
    default:
        return ESP_ERR_NOT_FOUND;
    }
}

bool factory_data_is_wifi_only(const factory_data_config_t *cfg)
{
    if (!cfg || !cfg->present) {
        return false;
    }

    return (cfg->mode == edge_device_factory_WifiMode_WIFI_MODE_WIFI);
}

bool factory_data_is_mesh_mode(const factory_data_config_t *cfg)
{
    if (!cfg || !cfg->present) {
        return false;
    }

    return (cfg->mode == edge_device_factory_WifiMode_WIFI_MODE_MESH);
}

static bool license_hex_to_bytes(const char *hex, uint8_t *out, size_t out_len)
{
    if (!hex || !out || strlen(hex) != out_len * 2U) {
        return false;
    }
    for (size_t i = 0; i < out_len; ++i) {
        unsigned value = 0;
        if (sscanf(&hex[i * 2U], "%2x", &value) != 1) {
            return false;
        }
        out[i] = (uint8_t)value;
    }
    return true;
}

bool factory_data_license_is_valid(void)
{
    /* Licensing is disabled. Keep this API for protocol compatibility. */
    return true;

    if (s_license_valid >= 0) {
        return s_license_valid == 1;
    }

    s_license_valid = 0;

    factory_data_config_t cfg = {0};
    uint8_t ble_mac[6] = {0};
    uint8_t serial_identity[6] = {0};
    uint8_t public_key[EDGEZ_LICENSE_PUBLIC_KEY_SIZE] = {0};
    uint8_t digest[32] = {0};
    static const uint8_t prefix[] = "EDGEZ-HALOW-LICENSE-V1";

    if (factory_data_load(&cfg) != ESP_OK ||
        cfg.credential_signature_len != EDGEZ_LICENSE_SIGNATURE_SIZE ||
        strlen(cfg.serial_number) != 12 ||
        !license_hex_to_bytes(cfg.serial_number, serial_identity, sizeof(serial_identity)) ||
        esp_read_mac(ble_mac, ESP_MAC_BT) != ESP_OK ||
        memcmp(&serial_identity[1], &ble_mac[1], 5) != 0 ||
        !license_hex_to_bytes(s_edgez_license_public_key_hex,
                              public_key,
                              sizeof(public_key))) {
        ESP_LOGW(TAG, "License unavailable: invalid signature, serial identity, BLE MAC, or public key");
        return false;
    }

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    int ret = mbedtls_sha256_starts(&sha, 0);
    if (ret == 0) {
        ret = mbedtls_sha256_update(&sha, prefix, sizeof(prefix));
    }
    if (ret == 0) {
        ret = mbedtls_sha256_update(&sha, serial_identity, sizeof(serial_identity));
    }
    if (ret == 0) {
        ret = mbedtls_sha256_finish(&sha, digest);
    }
    mbedtls_sha256_free(&sha);

    mbedtls_ecp_group group;
    mbedtls_ecp_point point;
    mbedtls_mpi r;
    mbedtls_mpi s;
    mbedtls_ecp_group_init(&group);
    mbedtls_ecp_point_init(&point);
    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);
    if (ret == 0) {
        ret = mbedtls_ecp_group_load(&group, MBEDTLS_ECP_DP_SECP256R1);
    }
    if (ret == 0) {
        ret = mbedtls_ecp_point_read_binary(&group, &point,
                                            public_key, sizeof(public_key));
    }
    if (ret == 0) {
        ret = mbedtls_mpi_read_binary(&r, cfg.credential_signature, 32);
    }
    if (ret == 0) {
        ret = mbedtls_mpi_read_binary(&s, cfg.credential_signature + 32, 32);
    }
    if (ret == 0) {
        ret = mbedtls_ecdsa_verify(&group, digest, sizeof(digest), &point, &r, &s);
    }
    mbedtls_mpi_free(&s);
    mbedtls_mpi_free(&r);
    mbedtls_ecp_point_free(&point);
    mbedtls_ecp_group_free(&group);

    s_license_valid = (ret == 0) ? 1 : 0;
    if (s_license_valid == 1) {
        memcpy(s_license_bound_mac, ble_mac, sizeof(s_license_bound_mac));
        memcpy(s_license_digest, digest, sizeof(s_license_digest));
        static const edgez_license_capability_t capabilities[] = {
            EDGEZ_LICENSE_CAP_RADIO_INIT,
            EDGEZ_LICENSE_CAP_MESH_TX,
            EDGEZ_LICENSE_CAP_MESH_RX,
            EDGEZ_LICENSE_CAP_PROVISION,
            EDGEZ_LICENSE_CAP_MOBILE_CONTROL,
        };
        for (size_t i = 0; i < sizeof(capabilities) / sizeof(capabilities[0]); ++i) {
            s_license_capability_tags[i] =
                license_capability_tag(capabilities[i], digest, ble_mac);
        }
    }
    ESP_LOGI(TAG,
             "Serial license licensed=%u serial=%s mac=%02x:%02x:%02x:%02x:%02x:%02x verify=%d",
             s_license_valid == 1 ? 1U : 0U,
             cfg.serial_number,
             ble_mac[0], ble_mac[1], ble_mac[2],
             ble_mac[3], ble_mac[4], ble_mac[5],
             ret);
    return s_license_valid == 1;
}

bool factory_data_license_authorize(edgez_license_capability_t capability)
{
    /* All firmware capabilities are authorized without factory credentials. */
    (void)capability;
    return true;

    const int capability_index = license_capability_index(capability);
    uint8_t current_mac[6] = {0};
    if (capability_index < 0 ||
        !factory_data_license_is_valid() ||
        esp_read_mac(current_mac, ESP_MAC_BT) != ESP_OK ||
        memcmp(current_mac, s_license_bound_mac, sizeof(current_mac)) != 0) {
        return false;
    }

    const uint32_t actual_tag =
        license_capability_tag(capability, s_license_digest, current_mac);
    return actual_tag == s_license_capability_tags[capability_index];
}

bool factory_data_sdk_release_authorize(const char *compatibility,
                                        const char *release_id,
                                        const uint8_t *signature,
                                        size_t signature_len)
{
    /* Accept empty or legacy SDK credentials and retain the wire handshake. */
    (void)compatibility;
    (void)release_id;
    (void)signature;
    (void)signature_len;
    s_sdk_release_auth_state = EDGEZ_SDK_RELEASE_AUTHORIZED;
    return true;

    s_sdk_release_auth_state = EDGEZ_SDK_RELEASE_AUTH_INVALID;
    if (!compatibility || !release_id || !signature ||
        signature_len != EDGEZ_LICENSE_SIGNATURE_SIZE ||
        strnlen(compatibility, EDGEZ_SDK_COMPATIBILITY_MAX + 1U) >
            EDGEZ_SDK_COMPATIBILITY_MAX ||
        strnlen(release_id, EDGEZ_SDK_RELEASE_ID_MAX + 1U) >
            EDGEZ_SDK_RELEASE_ID_MAX ||
        compatibility[0] == '\0' || release_id[0] == '\0') {
        return false;
    }
    if (!sdk_firmware_is_compatible(compatibility)) {
        s_sdk_release_auth_state = EDGEZ_SDK_RELEASE_AUTH_INCOMPATIBLE;
        return false;
    }

    char payload[sizeof(EDGEZ_SDK_RELEASE_PREFIX) +
                 EDGEZ_SDK_COMPATIBILITY_MAX + 1U +
                 EDGEZ_SDK_RELEASE_ID_MAX] = {0};
    const int payload_len = snprintf(payload,
                                     sizeof(payload),
                                     "%s%s:%s",
                                     EDGEZ_SDK_RELEASE_PREFIX,
                                     compatibility,
                                     release_id);
    if (payload_len <= 0 || (size_t)payload_len >= sizeof(payload)) {
        return false;
    }

    uint8_t digest[32] = {0};
    uint8_t public_key[EDGEZ_LICENSE_PUBLIC_KEY_SIZE] = {0};
    if (!license_hex_to_bytes(s_edgez_license_public_key_hex,
                              public_key,
                              sizeof(public_key))) {
        return false;
    }

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    int ret = mbedtls_sha256_starts(&sha, 0);
    if (ret == 0) {
        ret = mbedtls_sha256_update(&sha,
                                    (const uint8_t *)payload,
                                    (size_t)payload_len);
    }
    if (ret == 0) {
        ret = mbedtls_sha256_finish(&sha, digest);
    }
    mbedtls_sha256_free(&sha);

    mbedtls_ecp_group group;
    mbedtls_ecp_point point;
    mbedtls_mpi r;
    mbedtls_mpi s;
    mbedtls_ecp_group_init(&group);
    mbedtls_ecp_point_init(&point);
    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);
    if (ret == 0) {
        ret = mbedtls_ecp_group_load(&group, MBEDTLS_ECP_DP_SECP256R1);
    }
    if (ret == 0) {
        ret = mbedtls_ecp_point_read_binary(&group,
                                            &point,
                                            public_key,
                                            sizeof(public_key));
    }
    if (ret == 0) {
        ret = mbedtls_mpi_read_binary(&r, signature, 32);
    }
    if (ret == 0) {
        ret = mbedtls_mpi_read_binary(&s, signature + 32, 32);
    }
    if (ret == 0) {
        ret = mbedtls_ecdsa_verify(&group,
                                   digest,
                                   sizeof(digest),
                                   &point,
                                   &r,
                                   &s);
    }
    mbedtls_mpi_free(&s);
    mbedtls_mpi_free(&r);
    mbedtls_ecp_point_free(&point);
    mbedtls_ecp_group_free(&group);

    s_sdk_release_auth_state = ret == 0
        ? EDGEZ_SDK_RELEASE_AUTHORIZED
        : EDGEZ_SDK_RELEASE_AUTH_INVALID;
    return s_sdk_release_auth_state == EDGEZ_SDK_RELEASE_AUTHORIZED;
}

bool factory_data_sdk_release_is_authorized(void)
{
    return true;
}

edgez_sdk_release_auth_state_t factory_data_sdk_release_auth_state(void)
{
    return EDGEZ_SDK_RELEASE_AUTHORIZED;
}

void factory_data_sdk_release_reset(void)
{
    s_sdk_release_auth_state = EDGEZ_SDK_RELEASE_AUTHORIZED;
}
