# Copyright 2025-2026 Dimensional Inc.
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

"""Explicit UDP echo probes with Linux receive timestamps; never infer Wi-Fi airtime."""

from __future__ import annotations

import platform
import socket
import struct
import time
from typing import Any
from uuid import uuid4

from dimos.diagnostics.clock import clock_domain

# Linux UAPI SO_TIMESTAMPNS_OLD uses the native timespec ABI. Only enabled on
# 64-bit Linux; other platforms retain an explicit unavailable capability.
_SO_TIMESTAMPNS = 35
_MAGIC = b"DIMOS_PROBE_V1:"


def socket_probe(address: str, port: int, timeout: float = 2) -> dict[str, Any]:
    if not 0 < port < 65536 or not 0 < timeout <= 10:
        raise ValueError("invalid probe destination or timeout")
    packet = _MAGIC + uuid4().bytes
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(timeout)
        kernel = False
        if platform.system() == "Linux" and struct.calcsize("@l") == 8:
            try:
                sock.setsockopt(socket.SOL_SOCKET, _SO_TIMESTAMPNS, 1)
                kernel = True
            except OSError:
                pass
        sock.connect((address, port))
        begin = time.monotonic_ns()
        offset_before = time.time_ns() - time.monotonic_ns()
        try:
            sock.send(packet)
            data, ancillary, _, _ = sock.recvmsg(1024, 256)
            received = time.monotonic_ns()
            wall = time.time_ns()
            offset_after = wall - received
            if data != packet:
                raise ValueError("echo identity mismatch")
            timestamp = None
            for level, kind, raw in ancillary:
                if (
                    level == socket.SOL_SOCKET
                    and kind == _SO_TIMESTAMPNS
                    and len(raw) >= struct.calcsize("@ll")
                ):
                    seconds, nanos = struct.unpack("@ll", raw[: struct.calcsize("@ll")])
                    timestamp = seconds * 1_000_000_000 + nanos
            delta = wall - timestamp if timestamp is not None else None
            valid = (
                delta is not None and delta >= 0 and abs(offset_after - offset_before) < 1_000_000
            )
            return {
                "schema": "dimos.trace/v1",
                "event": "network.socket_probe",
                "status": "ok",
                "mono_ns": received,
                "clock_domain": clock_domain(),
                "target": f"{address}:{port}",
                "rtt_ns": received - begin,
                "kernel_rx_realtime_ns": timestamp,
                "kernel_rx_to_userspace_ns": delta if valid else None,
                "kernel_timestamp_enabled": kernel,
                "scope": "dedicated echo socket; not application delivery or over-air timing",
            }
        except (OSError, ValueError) as error:
            return {
                "schema": "dimos.trace/v1",
                "event": "network.socket_probe",
                "status": "error",
                "reason": type(error).__name__,
                "mono_ns": time.monotonic_ns(),
                "clock_domain": clock_domain(),
                "kernel_timestamp_enabled": kernel,
            }


def echo(sock: socket.socket, seconds: float) -> None:
    """Serve only fixed-size probe nonces on an explicitly supplied test socket."""
    end = time.monotonic() + seconds
    sock.settimeout(min(seconds, 0.25))
    while time.monotonic() < end:
        try:
            data, address = sock.recvfrom(1024)
            if len(data) == len(_MAGIC) + 16 and data.startswith(_MAGIC):
                sock.sendto(data, address)
        except TimeoutError:
            continue
