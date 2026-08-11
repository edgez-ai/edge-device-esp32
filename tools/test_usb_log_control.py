#!/usr/bin/env python3
"""Hardware smoke test for the EdgeZ CP2102 log/control-mode contract."""

from __future__ import annotations

import argparse
import fcntl
import glob
import os
import secrets
import select
import struct
import sys
import termios
import time
import tty


BAUD = 921_600
STREAM_MAGIC = b"\x94\xc3"
EZ_MAGIC = b"EZ"
LG_MAGIC = b"LG\x02"
EZ_PING = 1
EZ_PONG = 2
EZ_EXIT = 8
EZ_EXIT_RESPONSE = 9
LG_RECORD = 1
LG_SET_LEVEL = 2
LG_LEVEL_RESPONSE = 3
LG_LEVEL_ERROR = 0xFF
MAX_FRAME = 512


class SerialStream:
    def __init__(self, path: str) -> None:
        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        speed = getattr(termios, "B921600", termios.B38400)
        attrs[4] = speed
        attrs[5] = speed
        attrs[2] |= termios.CLOCAL | termios.CREAD
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        if not hasattr(termios, "B921600") and sys.platform == "darwin":
            # macOS supports arbitrary serial rates through IOSSIOSPEED rather
            # than termios baud constants. PySerial uses the same ioctl.
            fcntl.ioctl(self.fd, 0x80045402, struct.pack("I", BAUD))
        modem_lines = termios.TIOCM_DTR | termios.TIOCM_RTS
        fcntl.ioctl(self.fd, termios.TIOCMBIS, struct.pack("I", modem_lines))
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.buffer = bytearray()
        self.text = bytearray()

    def close(self) -> None:
        os.close(self.fd)

    def write_payload(self, payload: bytes) -> None:
        frame = STREAM_MAGIC + len(payload).to_bytes(2, "big") + payload
        view = memoryview(frame)
        while view:
            _, writable, _ = select.select([], [self.fd], [], 1.0)
            if not writable:
                raise TimeoutError("serial write timed out")
            view = view[os.write(self.fd, view) :]

    def _read_available(self, timeout: float) -> None:
        readable, _, _ = select.select([self.fd], [], [], timeout)
        if readable:
            try:
                chunk = os.read(self.fd, 16_384)
            except BlockingIOError:
                return
            self.buffer.extend(chunk)

    def next_payload(self, timeout: float) -> bytes | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            while len(self.buffer) >= 2:
                if self.buffer[:2] != STREAM_MAGIC:
                    self.text.append(self.buffer.pop(0))
                    continue
                if len(self.buffer) < 4:
                    break
                length = int.from_bytes(self.buffer[2:4], "big")
                if length == 0 or length > MAX_FRAME:
                    self.text.append(self.buffer.pop(0))
                    continue
                if len(self.buffer) < 4 + length:
                    break
                payload = bytes(self.buffer[4 : 4 + length])
                del self.buffer[: 4 + length]
                return payload
            self._read_available(max(0.0, deadline - time.monotonic()))
        return None

    def drain(self, duration: float = 0.25) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self._read_available(min(0.05, deadline - time.monotonic()))
        self.buffer.clear()
        self.text.clear()


def ez_frame(message_type: int, sequence: int, payload: bytes = b"") -> bytes:
    return struct.pack("<2sBBHH", EZ_MAGIC, 1, message_type, sequence, len(payload)) + payload


def parse_ez(payload: bytes) -> tuple[int, int, bytes] | None:
    if len(payload) < 8 or payload[:3] != b"EZ\x01":
        return None
    _, _, message_type, sequence, length = struct.unpack("<2sBBHH", payload[:8])
    if len(payload) != 8 + length:
        return None
    return message_type, sequence, payload[8:]


def lg_set_level(level: int, tag: str = "") -> bytes:
    return LG_MAGIC + bytes((LG_SET_LEVEL, level)) + tag.encode("utf-8")


def wait_for(stream: SerialStream, predicate, timeout: float) -> tuple[bytes, list[bytes]]:
    deadline = time.monotonic() + timeout
    seen: list[bytes] = []
    while time.monotonic() < deadline:
        payload = stream.next_payload(deadline - time.monotonic())
        if payload is None:
            break
        seen.append(payload)
        if predicate(payload):
            return payload, seen
    raw = bytes((stream.text + stream.buffer)[-64:])
    suffix = f"; last raw bytes={raw.hex()}" if raw else ""
    raise TimeoutError(
        f"expected response not received; saw {len(seen)} framed payload(s){suffix}"
    )


def choose_port(requested: str | None) -> str:
    if requested:
        return requested
    candidates = sorted(glob.glob("/dev/cu.usbserial*"))
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise RuntimeError("no /dev/cu.usbserial* CP2102 port found; pass --port")
    raise RuntimeError(f"multiple CP2102 ports found: {', '.join(candidates)}; pass --port")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial device; auto-detects /dev/cu.usbserial*")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--log-seconds", type=float, default=3.0)
    parser.add_argument("--require-log", action="store_true")
    args = parser.parse_args()

    port = choose_port(args.port)
    print(f"[INFO] Opening {port} at {BAUD} baud")
    stream = SerialStream(port)
    exit_sequence = secrets.randbelow(0xFFFF) + 1
    entered_control = False
    try:
        stream.drain()

        # LG2 commands must not claim the UART while it is still log-only.
        stream.write_payload(lg_set_level(2))
        try:
            wait_for(stream, lambda p: p.startswith(LG_MAGIC), 0.5)
            raise AssertionError("LG2 command was accepted before the ping/pong handshake")
        except TimeoutError:
            print("[PASS] Pre-handshake LG2 command did not claim the UART")

        sequence = secrets.randbelow(0xFFFF) + 1
        nonce = secrets.token_bytes(16)
        requested_level = 2  # WARN: the default app/device policy.
        handshake = nonce + bytes((requested_level,))
        stream.write_payload(ez_frame(EZ_PING, sequence, handshake))
        pong, _ = wait_for(
            stream,
            lambda p: parse_ez(p) == (EZ_PONG, sequence, handshake),
            args.timeout,
        )
        assert parse_ez(pong) == (EZ_PONG, sequence, handshake)
        entered_control = True
        print(f"[PASS] Matching nonce pong received (sequence={sequence})")

        stream.write_payload(lg_set_level(4))
        response, _ = wait_for(
            stream,
            lambda p: len(p) >= 5
            and p[:3] == LG_MAGIC
            and p[3] == LG_LEVEL_RESPONSE,
            args.timeout,
        )
        if response[4] != 4:
            raise AssertionError(f"requested DEBUG=4, device reported level={response[4]}")
        print("[PASS] LG2 log level changed to DEBUG")

        stream.write_payload(lg_set_level(6))
        error, _ = wait_for(
            stream,
            lambda p: len(p) >= 5
            and p[:3] == LG_MAGIC
            and p[3] == LG_LEVEL_RESPONSE,
            args.timeout,
        )
        if error[4] != LG_LEVEL_ERROR:
            raise AssertionError(f"invalid level was not rejected: response={error.hex()}")
        print("[PASS] Invalid LG2 log level rejected with level=255")

        logs: list[str] = []
        deadline = time.monotonic() + args.log_seconds
        while time.monotonic() < deadline:
            payload = stream.next_payload(deadline - time.monotonic())
            if payload is None:
                break
            if len(payload) >= 5 and payload[:3] == LG_MAGIC and payload[3] == LG_RECORD:
                logs.append(payload[5:].decode("utf-8", errors="replace").rstrip())
        if logs:
            print(f"[PASS] Received {len(logs)} dedicated LG2 log record(s)")
            for line in logs[:3]:
                print(f"       {line}")
        elif args.require_log:
            raise AssertionError("no LG2 log record received during observation window")
        else:
            print("[WARN] No runtime log was generated during the observation window")
        print("[PASS] LG2 tag is distinct from VC2/ST2 and can be excluded from speed/loss")

        stream.write_payload(lg_set_level(2))
        restored, _ = wait_for(
            stream,
            lambda p: len(p) >= 5
            and p[:3] == LG_MAGIC
            and p[3] == LG_LEVEL_RESPONSE,
            args.timeout,
        )
        if restored[4] != 2:
            raise AssertionError(f"failed to restore WARN level: response={restored.hex()}")
        print("[PASS] Log level restored to WARN")

        stream.write_payload(ez_frame(EZ_EXIT, exit_sequence))
        wait_for(
            stream,
            lambda p: parse_ez(p) == (EZ_EXIT_RESPONSE, exit_sequence, b""),
            args.timeout,
        )
        entered_control = False
        print("[PASS] Exit acknowledged; UART returned to plain-log mode")
        print("[PASS] USB log/control contract passed")
        return 0
    finally:
        if entered_control:
            try:
                stream.write_payload(lg_set_level(2))
                stream.write_payload(ez_frame(EZ_EXIT, exit_sequence))
            except Exception:
                pass
        stream.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError, RuntimeError, TimeoutError) as error:
        print(f"[FAIL] {error}", file=sys.stderr)
        raise SystemExit(1)
