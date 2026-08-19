# OpenMANET protobuf definitions

The files under `network/v1` are vendored from
[`OpenMANET/protobufs`](https://github.com/OpenMANET/protobufs) commit
`ae3e237214be178a114e564f84f3c233a51f8f01`.

Upstream is licensed under GPL-3.0. The local `openmanet.options` file is an
ESP32/Nanopb configuration overlay; it changes only generated in-memory bounds,
not protobuf field numbers or wire encoding.

Generated Nanopb bindings are committed under `main/generated` so normal
firmware builds do not need Python generator packages. To regenerate them,
use Nanopb 0.4.9.1 and apply `network/v1/openmanet.options`.
