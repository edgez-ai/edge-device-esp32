# ESP-IDF evaluates component dependencies in an isolated CMake scope, before
# the generated CONFIG_* variables exist. Read the persisted choice here too;
# relying on a top-level variable would silently select morse_halow in that
# dependency-scanning scope even when source mode was requested.
set(edgez_component_source_mode OFF)
if(DEFINED SDKCONFIG AND EXISTS "${SDKCONFIG}")
    file(STRINGS "${SDKCONFIG}" edgez_source_setting
        REGEX "^CONFIG_BUILD_EDGEZ_FROM_SOURCE=y$")
    file(STRINGS "${SDKCONFIG}" edgez_prebuilt_setting
        REGEX "^CONFIG_USE_PREBUILT_EDGEZ=y$")
endif()
if(NOT edgez_source_setting AND NOT edgez_prebuilt_setting)
    file(STRINGS
        "${CMAKE_CURRENT_LIST_DIR}/../sdkconfig.defaults.esp32s3"
        edgez_source_setting
        REGEX "^CONFIG_BUILD_EDGEZ_FROM_SOURCE=y$")
endif()
if(edgez_source_setting AND NOT edgez_prebuilt_setting)
    set(edgez_component_source_mode ON)
endif()

if(edgez_component_source_mode)
    set(edgez_halow_requires
        edgez_mesh
        mm_shims
        mmipal
        mmregdb
        mmutils
        morselib)
else()
    set(edgez_halow_requires morse_halow)
endif()

set(edgez_requires
    app_update
    bt
    driver
    esp_adc
    esp_app_format
    esp_driver_rmt
    esp_hw_support
    esp_netif
    esp_psram
    esp_timer
    esp_wifi
    freertos
    heap
    log
    lua
    mbedtls
    nanopb
    nvs_flash
    ${edgez_halow_requires})

set(edgez_sources
    "main.c"
    "src/app/edgez_app.c"
    "src/factory/factory_data.c"
    "src/factory/factory_data.pb.c"
    "src/mesh/halow_sync_bridge.c"
    "src/platform/edgez_platform.c"
    "src/platform/esp32/ble_control.c"
    "src/platform/esp32/edgez_platform_adapter.c"
    "src/platform/esp32/halow_interface_app.cpp"
    "src/platform/esp32/prov.c"
    "src/platform/esp32/status_led.c"
    "src/platform/esp32/usb_control_transport.c"
    "src/protocol/edgez_frame_protocol.c"
    "src/protocol/edgez_lua_rpc.c"
    "src/protocol/usb_control.pb.c")

if(CONFIG_EDGEZ_SENSOR_SCRIPTING)
    list(APPEND edgez_sources
        "src/platform/esp32/interface_bridge.c"
        "src/platform/esp32/l76k_gps.c"
        "src/platform/esp32/script_interface_runtime.c"
        "src/sampling/sampling.c"
        "src/scripting/local_interface.c"
        "src/scripting/local_script_store.c")
else()
    list(APPEND edgez_sources
        "src/platform/esp32/l76k_gps.c"
        "src/sampling/sampling_disabled.c")
endif()

idf_component_register(
    SRCS ${edgez_sources}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS
        "include/edgez"
        "private_include"
    REQUIRES ${edgez_requires})

# Preserve the source-build compile settings from the working baseline.
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Os
    -g0
    -fvisibility=hidden
    -ffunction-sections
    -fdata-sections)
