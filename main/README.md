# EdgeZ application component

This ESP-IDF component owns and compiles the EdgeZ application, mesh protocol,
and product logic. The Morse Micro HaLow implementation is linked separately
from `components/morse_halow/lib/<target>/libmorse.a`; firmware builds do not
compile or discover the private `third_party/mm-iot-esp32` source tree.

The packaged archive can be refreshed from the pinned private source with:

```sh
third_party/mm-iot-esp32/tools/build_libmorse.sh components/morse_halow
```

The board BCF remains outside the archive at
`components/morse_halow/bcf/bcf_mf08551.mbin`.

It currently contains:

- BATMAN-IV next-hop routing plus EdgeZ forwarding, hop limiting and duplicate
  suppression;
- reliable delivery, acknowledgement and retry state;
- beacon encoding, decoding and peer topology tracking;
- voice and global-buffer routing;
- the EdgeZ nanopb wire schema, control-frame dispatch and Lua RPC framing;
- device and mesh settings protocol handling;
- factory-data decoding and storage access.

The host firmware remains responsible for hardware and product services such
as BLE transport, Morse/ESP32 radio initialization, provisioning, sampling,
script storage, status LEDs and power transitions. EdgeZ consumes those
services only through the SDK-owned `edgez_platform_api_t` contract. A host
registers its application adapter once with `edgez_platform_register()` before
starting EdgeZ; the SDK does not include host headers or resolve host paths.

## Consuming from an ESP-IDF project

Add the component to the application's `idf_component.yml`:

```yaml
dependencies:
  edgez:
    path: path/to/mm-iot-esp32/framework/edgez
```

Then add `edgez` to the consuming component's `REQUIRES` list and select one
of the library modes above.

The public headers are `halow_sync_bridge.h`, `edgez_frame_protocol.h`,
`edgez_platform.h`, and `usb_control.pb.h`.

## UART and BLE logging

The CP2102 UART runs at 921600 baud and starts in plain-text log mode. Its RX
parser remains active, but protobuf, voice, and speed frames are ignored until
the host completes the existing legacy ping/pong handshake. A valid ping is an
outer stream frame with magic `94 C3`, a big-endian payload length, and this
legacy EZ payload:

| Offset | Size | Encoding | Value |
| --- | ---: | --- | --- |
| 0 | 2 | bytes | `E Z` |
| 2 | 1 | unsigned | protocol version `1` |
| 3 | 1 | unsigned | message type |
| 4 | 2 | little-endian | sequence |
| 6 | 2 | little-endian | payload length |
| 8 | variable | bytes | payload |

The ping uses type `1` and carries a host-generated nonce in its payload. The
device drains pending text logs, returns type `2` with the same sequence and
nonce, and only then enters framed control mode.

| Type | Direction | Sequence field | Payload |
| ---: | --- | --- | --- |
| 1 | host to device | request sequence | nonce |
| 2 | device to host | echoed sequence | echoed nonce |
| 8 | host to device | request sequence | empty |
| 9 | device to host | echoed sequence | empty |

Logs and log-level control use a dedicated `LG2` payload, independent of the
legacy EZ control handshake, `VC2` voice packets, and `ST2` speed-test packets:

| Offset | Size | Encoding | Value |
| --- | ---: | --- | --- |
| 0 | 3 | bytes | `L G 2` |
| 3 | 1 | unsigned | message type |
| 4 | 1 | unsigned | log level |
| 5 | variable | bytes | UTF-8 text or tag |

| Type | Direction | Level | Payload |
| ---: | --- | --- | --- |
| 1 | device to host | reserved | UTF-8 log text, up to 256 bytes |
| 2 | host to device | requested level | optional UTF-8 tag; empty means `*` |
| 3 | device to host | effective level, or `255` on error | effective tag |

On USB, the complete `LG2` payload is carried inside the normal `94 C3` outer
stream frame and is emitted only after the nonce handshake has selected framed
control mode. On BLE, the complete `LG2` payload is sent directly over the
realtime stream characteristic used by `VC2` and `ST2`; it is not wrapped in a
voice or speed-test tag.

ESP log levels are `0=none`, `1=error`, `2=warn`, `3=info`, `4=debug`, and
`5=verbose`. The effective level is capped by `CONFIG_LOG_MAXIMUM_LEVEL`.
After the type-9 exit response is transmitted, the UART returns to plain-text
log mode.

The default runtime level is `WARN`. `LG2` records use a separate,
lowest-priority diagnostic lane and must be excluded from voice/speed
throughput and loss statistics. Client applications are responsible for any
log history they want to retain.
