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

"""Linux iw acquisition; cached observations retain their original age."""

from __future__ import annotations

import asyncio
import math
import re
import subprocess
import time

from dimos.msgs.sensor_msgs.WifiScan import WifiAccessPoint, WifiScan

_MAC = r"[0-9a-fA-F]{2}(?::[0-9a-fA-F]{2}){5}"


def _number(text: str, pattern: str) -> float | None:
    match = re.search(pattern, text, re.MULTILINE)
    if match is None:
        return None
    value = float(match[1])
    return value if math.isfinite(value) else None


def link_fields(text: str) -> dict[str, float | None]:
    """Fields shared by connection observations and diagnostic station parsing."""
    return {
        "signal_dbm": _number(text, r"^\s*signal:\s*(-?[\d.]+) dBm"),
        "frequency_mhz": _number(text, r"^\s*freq:\s*(\d+)"),
        "tx_bitrate_mbps": _number(text, r"^\s*tx bitrate:\s*([\d.]+) MBit/s"),
        "rx_bitrate_mbps": _number(text, r"^\s*rx bitrate:\s*([\d.]+) MBit/s"),
    }


def _access_point(text: str, bssid: str, ts: float, connected: bool) -> WifiAccessPoint | None:
    fields = link_fields(text)
    signal, frequency = fields["signal_dbm"], fields["frequency_mhz"]
    if signal is None or frequency is None:
        return None
    ssid = re.search(r"^\s*SSID: ?(.*)$", text, re.MULTILINE)
    return WifiAccessPoint(
        ts=ts,
        bssid=bssid.lower(),
        ssid=ssid[1] if ssid else "",
        frequency_mhz=int(frequency),
        signal_dbm=signal,
        connected=connected,
        tx_bitrate_mbps=fields["tx_bitrate_mbps"],
        rx_bitrate_mbps=fields["rx_bitrate_mbps"],
    )


def parse_link(text: str, ts: float) -> WifiAccessPoint | None:
    match = re.search(rf"^Connected to ({_MAC})", text, re.MULTILINE)
    return _access_point(text, match[1], ts, True) if match else None


def parse_scan(text: str, ts: float, max_age_s: float) -> list[WifiAccessPoint]:
    """Ignore records without an observation age; they cannot safely be mapped."""
    points: list[WifiAccessPoint] = []
    for block in re.split(r"(?m)^BSS ", text)[1:]:
        match = re.match(_MAC, block)
        age_ms = _number(block, r"^\s*last seen:\s*(\d+) ms ago")
        if match is None or age_ms is None or age_ms > max_age_s * 1000:
            continue
        point = _access_point(block, match[0], ts - age_ms / 1000, False)
        if point is not None:
            points.append(point)
    return points


async def run_iw(interface: str, *arguments: str) -> str:
    command = ["iw", "dev", interface, *arguments]
    process = await asyncio.create_subprocess_exec(
        *command, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE
    )
    try:
        stdout, stderr = await asyncio.wait_for(process.communicate(), timeout=3)
        if process.returncode:
            raise subprocess.CalledProcessError(process.returncode, command, stdout, stderr)
        return stdout.decode()
    finally:
        if process.returncode is None:
            process.kill()
            await process.wait()


async def read_scan(interface: str, max_age_s: float) -> WifiScan:
    cached = await run_iw(interface, "scan", "dump")
    points = parse_scan(cached, time.time(), max_age_s)
    link = await run_iw(interface, "link")
    ts = time.time()
    connected = parse_link(link, ts)
    if connected is not None:
        points = [point for point in points if point.bssid != connected.bssid]
        points.append(connected)
    return WifiScan(ts=ts, interface=interface, access_points=points)
