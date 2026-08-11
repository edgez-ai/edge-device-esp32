# Packaged Morse Micro HaLow driver

This component is the only Morse Micro dependency exposed to the public EdgeZ
application. It contains the predefined `lib/esp32s3/libmorse.a`, the public
headers required by `main`, and the board-specific BCF in `bcf/`. Public builds
consume these supplied files directly and do not require MM-IoT source code.

Release maintainers regenerate and validate the package in the internal source
repository before publishing it. Public users should treat `libmorse.a` and the
BCF as prebuilt dependencies.

## License notice

This `libmorse.a` is an EdgeZeron AI AB modified integration containing custom
EdgeZeron code. EdgeZeron's modifications and custom code are provided only for
non-commercial use under the repository's PolyForm Noncommercial License 1.0.0.
Commercial use requires a separate prior written license from EdgeZeron AI AB.

The archive also contains third-party portions. Those portions retain their
respective copyrights and applicable license terms; the EdgeZeron license does
not replace or override them.

The generated archive combines MorseLib, HostAP, MM shims, MM IP adaptation,
packet memory, regulatory database, utilities, and firmware. The BCF remains a
separate component asset and is embedded into the application during its final
link.
