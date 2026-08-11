# Packaged Morse Micro HaLow driver

This component is the only Morse Micro dependency exposed to the EdgeZ
application. It contains the generated `lib/esp32s3/libmorse.a`, the public
headers required by `main`, and the board-specific BCF in `bcf/`.

Regenerate both from the pinned `third_party/mm-iot-esp32` source with:

```sh
third_party/mm-iot-esp32/tools/build_libmorse.sh components/morse_halow
```

The generated archive combines MorseLib, HostAP, MM shims, MM IP adaptation,
packet memory, regulatory database, utilities, and firmware. The BCF remains a
separate component asset and is embedded into the application during its final
link.
