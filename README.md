# EdgeZ ESP32-S3 Device

This repository contains the public ESP-IDF firmware project for the EdgeZ
ESP32-S3 device. It builds the EdgeZ application and links it with the supplied
prebuilt Morse Micro HaLow library.

## Prebuilt Morse Micro library

`components/morse_halow/lib/esp32s3/libmorse.a` is a required, predefined build
artifact included in this repository. Public builds consume this archive
directly; they do not require the private `mm-iot-esp32` source repository and
do not rebuild MorseLib or HostAP.

The packaged `components/morse_halow` component contains:

- `lib/esp32s3/libmorse.a`: prebuilt Morse Micro, HostAP, and EdgeZ HaLow
  integration code.
- `include/`: headers exposed to the application build.
- `bcf/bcf_mf08551.mbin`: the board-specific BCF, kept outside `libmorse.a` so
  it can be selected and embedded by the final firmware build.

Release maintainers generate and validate these packaged files before syncing
the public source. Users of this repository should treat them as supplied
dependencies.

The bundled `libmorse.a` is an EdgeZeron AI AB modified integration and
contains custom EdgeZeron code. EdgeZeron's modifications and custom code are
provided only for non-commercial use under this repository's license. Using
those portions in a commercial project requires a separate prior written
license from EdgeZeron AI AB.

## Project layout

```text
components/morse_halow/  Prebuilt HaLow component, headers, and BCF
main/
  include/edgez/         Public EdgeZ application and protocol headers
  private_include/       Application-internal headers
  proto/                 USB control protocol definitions
  src/                   EdgeZ application, mesh, platform, and sampling code
  main.c                 ESP-IDF entry point
```

The application entry point calls `edgez_app_run()`. EdgeZ application logic is
compiled in the `main` component, while the HaLow implementation is linked from
the packaged `libmorse.a` archive.

## Build

Install and activate ESP-IDF 5.5.4, then build the project:

```sh
. "$IDF_PATH/export.sh"
rm -f sdkconfig sdkconfig.old
idf.py -DIDF_TARGET=esp32s3 \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults.esp32s3" \
  build
```

The OTA application image is generated at `build/edge-device-esp32.bin`. To
create a single flashable image containing the bootloader, partition table, OTA
metadata, and application, run:

```sh
./make_merged_bin.sh \
  --skip-build \
  --output build/edge-device-esp32_merged.bin
```

## Releases

The internal release workflow rebuilds and validates `libmorse.a`, verifies a
clean public build without private MM-IoT sources, and syncs this source package
to the `internal` branch of `edgez-ai/edge-device-esp32`. Release binaries are
published alongside that source snapshot.

## License

The EdgeZ project source is available under the
[PolyForm Noncommercial License 1.0.0](LICENSE). Commercial use is not granted
by that license; contact the project maintainers to discuss separate commercial
terms.

Third-party portions of `libmorse.a`, as well as Morse Micro headers, firmware
data, and BCF data, retain their own copyright notices and remain subject to
their applicable third-party terms. The EdgeZeron license does not replace or
override those terms.
