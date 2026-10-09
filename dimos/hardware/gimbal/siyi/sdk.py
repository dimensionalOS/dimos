# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""SIYI gimbal-camera Ethernet SDK: the UDP protocol on port 37260, here the zoom query.

Packet building and parsing are pure; :class:`SiyiSdk` owns the socket. Aim commands do
not go through this SDK: the gimbal is aimed over MAVLink.
"""

from __future__ import annotations

import binascii
import socket
import struct
import time

A8_SDK_PORT = 37260  # fixed by the SIYI SDK protocol

CMD_ZOOM_MULTIPLE = 0x18


def build_packet(cmd: int, payload: bytes = b"", seq: int = 0) -> bytes:
    pkt = b"\x55\x66\x01" + struct.pack("<HHB", len(payload), seq, cmd) + payload
    return pkt + struct.pack("<H", binascii.crc_hqx(pkt, 0))  # CRC-16/XMODEM


def parse_packet(data: bytes) -> tuple[int, bytes] | None:
    """``(cmd, body)`` of a well-formed reply, else None (bad header, length or CRC)."""
    if len(data) < 10 or data[:2] != b"\x55\x66":
        return None
    n = int.from_bytes(data[3:5], "little")
    if len(data) < 8 + n + 2:
        return None
    if struct.unpack("<H", data[8 + n : 10 + n])[0] != binascii.crc_hqx(data[: 8 + n], 0):
        return None
    return data[7], data[8 : 8 + n]


def parse_zoom(body: bytes) -> float:
    return float(body[0]) + float(body[1]) / 10.0


class SiyiSdk:
    """Minimal SDK client. ``open()`` connects the UDP socket; every query is bounded."""

    def __init__(self, ip: str, port: int = A8_SDK_PORT, timeout_s: float = 0.15) -> None:
        self._addr = (ip, port)
        self._timeout_s = timeout_s
        self._sock: socket.socket | None = None
        self._seq = 0

    def open(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(self._timeout_s)
        self._sock.connect(self._addr)

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def _query(self, cmd: int, min_len: int, deadline_s: float = 0.3) -> bytes | None:
        sock = self._sock
        if sock is None:
            raise RuntimeError("SiyiSdk not open")
        seq, self._seq = self._seq, (self._seq + 1) & 0xFFFF
        end = time.monotonic() + deadline_s
        try:
            sock.send(build_packet(cmd, seq=seq))
            while time.monotonic() < end:
                try:
                    data = sock.recv(1024)
                except TimeoutError:
                    continue
                parsed = parse_packet(data)
                if parsed is None or parsed[0] != cmd:
                    continue
                if len(parsed[1]) >= min_len:
                    return parsed[1]
        except OSError:
            pass  # ICMP unreachable on the connected socket: the camera is down or booting
        return None

    def query_zoom(self) -> float | None:
        body = self._query(CMD_ZOOM_MULTIPLE, 2)
        return None if body is None else parse_zoom(body)
