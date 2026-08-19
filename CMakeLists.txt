# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly.
cmake_minimum_required(VERSION 3.16)

set(SDKCONFIG_DEFAULTS
    "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults.esp32s3")

# Component discovery happens before ESP-IDF evaluates Kconfig. Read the saved
# build-mode choice early so the same CONFIG_BUILD_EDGEZ_FROM_SOURCE switch
# used by the original working setup can select source components or the
# packaged morse_halow component.
function(edgez_read_library_mode config_file output_variable)
    set(edgez_mode "")
    if(EXISTS "${config_file}")
        file(STRINGS "${config_file}" edgez_source_setting
            REGEX "^CONFIG_BUILD_EDGEZ_FROM_SOURCE=y$")
        file(STRINGS "${config_file}" edgez_prebuilt_setting
            REGEX "^CONFIG_USE_PREBUILT_EDGEZ=y$")
        if(edgez_source_setting AND edgez_prebuilt_setting)
            message(FATAL_ERROR "Conflicting EdgeZ build modes in ${config_file}")
        elseif(edgez_source_setting)
            set(edgez_mode "SOURCE")
        elseif(edgez_prebuilt_setting)
            set(edgez_mode "PREBUILT")
        endif()
    endif()
    set(${output_variable} "${edgez_mode}" PARENT_SCOPE)
endfunction()

if(DEFINED SDKCONFIG AND NOT SDKCONFIG STREQUAL "")
    set(edgez_sdkconfig "${SDKCONFIG}")
else()
    set(edgez_sdkconfig "${CMAKE_CURRENT_LIST_DIR}/sdkconfig")
endif()

edgez_read_library_mode("${edgez_sdkconfig}" edgez_library_mode)
if(NOT edgez_library_mode)
    edgez_read_library_mode(
        "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults.esp32s3"
        edgez_library_mode)
endif()

if(edgez_library_mode STREQUAL "SOURCE")
    list(APPEND SDKCONFIG_DEFAULTS
        "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.edgez-source.defaults")
    list(APPEND EXTRA_COMPONENT_DIRS
        "${CMAKE_CURRENT_LIST_DIR}/third_party/mm-iot-esp32/framework/morselib"
        "${CMAKE_CURRENT_LIST_DIR}/third_party/mm-iot-esp32/framework/mm_shims"
        "${CMAKE_CURRENT_LIST_DIR}/third_party/mm-iot-esp32/framework/src/edgez_mesh"
        "${CMAKE_CURRENT_LIST_DIR}/third_party/mm-iot-esp32/framework/src/hostap"
        "${CMAKE_CURRENT_LIST_DIR}/third_party/mm-iot-esp32/framework/src/mmipal"
        "${CMAKE_CURRENT_LIST_DIR}/third_party/mm-iot-esp32/framework/src/mmpktmem"
        "${CMAKE_CURRENT_LIST_DIR}/third_party/mm-iot-esp32/framework/src/mmregdb"
        "${CMAKE_CURRENT_LIST_DIR}/third_party/mm-iot-esp32/framework/src/mmutils")
    message(STATUS "EdgeZ HaLow build mode: source")
else()
    message(STATUS "EdgeZ HaLow build mode: packaged libmorse.a")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Keep the project build trimmed to main and its declared dependencies.
idf_build_set_property(MINIMAL_BUILD ON)

# Ensure lwIP exposes netif callback APIs used by mmipal
idf_build_set_property(COMPILE_DEFINITIONS "LWIP_NETIF_LINK_CALLBACK=1" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "LWIP_NETIF_STATUS_CALLBACK=1" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "CONFIG_MMHAL_CHIP_TYPE_MM6108=1" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "CONFIG_MMHAL_CHIP_TYPE_MM8108=0" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "CONFIG_MM_HALOW_MESH_JOIN_ONLY=1" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "MESH_PREQ_INTERVAL_MS=4000" APPEND)
# The speed-test logs show simultaneous BLE ingress, local radio TX/RX and
# BATMAN forwarding exhausting the original packet bounds. Double the bounds
# so short bursts do not force avoidable flow-control pauses or RX drops.
idf_build_set_property(COMPILE_DEFINITIONS "MMPKTMEM_TX_POOL_N_BLOCKS=40" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "MMPKTMEM_RX_POOL_N_BLOCKS=46" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "MMLOG_LEVEL_DEFAULT=1" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "NEED_AP_MLME" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "ON_DEMAND_TIMERS_ENABLED=0" APPEND)

project(edge-device-esp32)

# The Morse transport uses the ESP-IDF SPI polling API from its driver task.
# ESP-IDF marks that hot path IRAM-safe, but when verbose logging is compiled
# in it still enters the cross-core logging lock after every transaction. A
# simultaneous hardware-AES CCMP operation can then hold the other core in the
# scheduler long enough to trip the interrupt watchdog. Keep INFO/WARN/ERROR SPI
# diagnostics while compiling the polling-path DEBUG/VERBOSE calls out.
idf_component_get_property(edgez_spi_component esp_driver_spi COMPONENT_LIB)
target_compile_definitions(${edgez_spi_component} PRIVATE LOG_LOCAL_LEVEL=3)

# spi_bus_lock.c belongs to esp_hw_support rather than esp_driver_spi. Apply the
# same ceiling only to that source so DEBUG/VERBOSE remains available elsewhere.
idf_component_get_property(edgez_hw_support_component esp_hw_support COMPONENT_LIB)
set_source_files_properties(
    "$ENV{IDF_PATH}/components/esp_hw_support/spi_bus_lock.c"
    TARGET_DIRECTORY ${edgez_hw_support_component}
    PROPERTIES COMPILE_DEFINITIONS "LOG_LOCAL_LEVEL=3")
