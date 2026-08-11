# EdgeZ ESP32-S3 Device

This ESP-IDF project keeps the EdgeZ device application in `main/`. The
baseline build compiles MorseLib, HostAP, and the MM platform components from
the pinned `mm-iot-esp32` submodule.

## Project layout

```text
main/
  include/edgez/        Public EdgeZ application and protocol headers
  private_include/      Application-internal headers
  proto/                USB control protocol definitions
  src/                  EdgeZ application, mesh, platform, and sampling code
  main.c                ESP-IDF entry point
third_party/mm-iot-esp32/  (internal repository only)
  framework/            Pinned MorseLib, HostAP, and MM platform sources
```

The application entry point calls `edgez_app_run()`. EdgeZ business logic is
compiled as part of the project `main` component. The HaLow driver and its BCF
are compiled and embedded through the original source-build component graph.

## Build

For the baseline application build, initialize the pinned driver submodule,
activate ESP-IDF 5.5, and build:

```sh
. "$IDF_PATH/export.sh"
git submodule update --init --recursive third_party/mm-iot-esp32
rm -f sdkconfig sdkconfig.old
idf.py -DIDF_TARGET=esp32s3 build
```

After changing or updating the Morse Micro driver, initialize the pinned
submodule and regenerate the packaged archive and headers before rebuilding the
application:

```sh
git submodule update --init --recursive third_party/mm-iot-esp32
. "$IDF_PATH/export.sh"
third_party/mm-iot-esp32/tools/build_libmorse.sh components/morse_halow
```

The OTA application image is `build/edge-device-esp32.bin`. To also create a
single image containing the bootloader, partition table, OTA metadata, and
application:

```sh
./make_merged_bin.sh \
  --skip-build \
  --output build/edge-device-esp32_merged.bin
```

The `Build and publish ESP32S3 release` GitHub Actions workflow regenerates
`libmorse.a`, builds the application against that archive, and uploads both
images. It then build-tests a clean source snapshot without the private
`mm-iot-esp32` submodule and publishes that snapshot to the `main` branch of
`edgez-ai/edge-device-esp32`. The published project builds directly from the
packaged `components/morse_halow` component.

Because the `mm-iot-esp32-mesh` submodule is private, configure
`GH_PERSONAL_ACCESS_TOKEN` with read access to that repository and Contents:
Write access to the target repository. The target repository's `.github/`
directory is preserved during the source sync.
