if(NOT IDF_TARGET STREQUAL "esp32s3")
    message(FATAL_ERROR "The packaged Morse Micro HaLow driver supports esp32s3 only")
endif()

set(morse_archive
    "${CMAKE_CURRENT_LIST_DIR}/lib/${IDF_TARGET}/libmorse.a")
set(morse_bcf
    "${CMAKE_CURRENT_LIST_DIR}/bcf/bcf_mf08551.mbin")
if(NOT EXISTS "${morse_archive}")
    message(FATAL_ERROR
        "Packaged Morse driver not found: ${morse_archive}. "
        "Run third_party/mm-iot-esp32/tools/build_libmorse.sh "
        "components/morse_halow first.")
endif()
if(NOT EXISTS "${morse_bcf}")
    message(FATAL_ERROR
        "Packaged Morse BCF not found: ${morse_bcf}. "
        "Run third_party/mm-iot-esp32/tools/build_libmorse.sh "
        "components/morse_halow first.")
endif()

idf_component_register(
    INCLUDE_DIRS "include"
    REQUIRES
        driver
        esp_app_format
        esp_driver_spi
        esp_event
        esp_hw_support
        esp_netif
        esp_timer
        esp_wifi
        freertos
        heap
        log
        lwip
        mbedtls
        nvs_flash)

add_prebuilt_library(morse_prebuilt "${morse_archive}"
    PRIV_REQUIRES
        driver
        esp_app_format
        esp_driver_spi
        esp_event
        esp_hw_support
        esp_netif
        esp_timer
        esp_wifi
        freertos
        heap
        log
        lwip
        mbedtls
        nvs_flash)

# Keep the board-specific BCF outside libmorse.a so it can be replaced without
# rebuilding the driver archive. The MM shim expects these two linker symbols.
set(morse_bcf_object "${CMAKE_CURRENT_BINARY_DIR}/bcf_mf08551.mbin.o")
string(MAKE_C_IDENTIFIER "${morse_bcf}" morse_bcf_identifier)
add_custom_command(
    OUTPUT "${morse_bcf_object}"
    COMMAND "${CMAKE_OBJCOPY}"
        -I binary -O elf32-xtensa-le -B xtensa
        "${morse_bcf}" "${morse_bcf_object}"
        --redefine-sym
            "_binary_${morse_bcf_identifier}_start=bcf_binary_start"
        --redefine-sym
            "_binary_${morse_bcf_identifier}_end=bcf_binary_end"
        --rename-section
            .data=.rodata._bcf_mbin,contents,alloc,load,readonly,data
    DEPENDS "${morse_bcf}"
    VERBATIM)
add_custom_target(morse_bcf_blob ALL DEPENDS "${morse_bcf_object}")
add_dependencies(${COMPONENT_LIB} morse_bcf_blob)

# The original MorseLib and MM shim components are registered with
# WHOLE_ARCHIVE when built from source. Preserve that behavior after merging
# them into libmorse.a; several stack entry points are reached through weak or
# callback-only references and would otherwise be discarded by the linker.
# Pass the archive and both state changes in one linker argument so CMake
# cannot place morse_prebuilt's ESP-IDF dependencies inside the region. The
# ordinary morse_prebuilt entry that follows supplies those dependencies in
# link order; it does not extract the already-loaded archive a second time.
# Keep the replaceable board BCF outside the whole-archive region.
target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-Wl,--whole-archive,${morse_archive},--no-whole-archive"
    morse_prebuilt
    "${morse_bcf_object}")
