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

"""Measured radio/link health, with explicit counter provenance and missing coverage."""

from __future__ import annotations

from functools import lru_cache
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
from typing import Any

from dimos.constants import CACHE_DIR
from dimos.hardware.sensors.wifi.iw import link_fields

COUNTERS = (
    "tx_packets",
    "rx_packets",
    "tx_bytes",
    "rx_bytes",
    "tx_retries",
    "tx_failed",
    "tx_dropped",
    "rx_dropped",
)


@lru_cache(maxsize=1)
def macos_helper() -> str | None:
    """Build the tiny OS observer once; never invoke scanning system_profiler in the loop."""
    compiler = shutil.which("swiftc")
    if compiler is None:
        return None
    source = Path(__file__).with_name("wifi_macos.swift")
    digest = hashlib.sha256(source.read_bytes()).hexdigest()[:16]
    directory = CACHE_DIR / "wifi"
    destination = directory / f"wifi-macos-{digest}"
    if not destination.exists():
        directory.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_suffix(f".{os.getpid()}.tmp")
        try:
            subprocess.run(
                [compiler, "-O", str(source), "-o", str(temporary)],
                capture_output=True,
                timeout=30,
                check=True,
            )
            temporary.replace(destination)
        except (OSError, subprocess.SubprocessError):
            temporary.unlink(missing_ok=True)
            return None
    return str(destination)


def linux_radio(probes: list[dict[str, Any]], interface: str) -> dict[str, Any]:
    """Parse nl80211 station/link/survey and qdisc evidence from the selected interface."""
    radio: dict[str, Any] = {"interface": interface, "source": "Linux iw/tc", "counters": {}}
    texts = {}
    for probe in probes:
        command = probe.get("command", [])
        if interface not in command or probe.get("returncode") != 0:
            continue
        if command and command[0] in ("iw", "tc", "ethtool"):
            texts[" ".join(command)] = probe.get("stdout", "")
    link = texts.get(f"iw dev {interface} link", "")
    station = texts.get(f"iw dev {interface} station dump", "")
    match = re.search(r"(?:Connected to|Station) ([0-9a-fA-F:]{17})", link or station)
    radio["associated"] = bool(match) if link or station else None
    radio["bssid"] = match[1].lower() if match else None
    fields = link_fields(station + "\n" + link)
    for name, key in (
        ("signal_dbm", "signal_dbm"),
        ("frequency_mhz", "frequency_mhz"),
        ("tx_rate_mbps", "tx_bitrate_mbps"),
        ("rx_rate_mbps", "rx_bitrate_mbps"),
    ):
        radio[name] = fields[key]
    for name in COUNTERS:
        found = re.search(r"^\s*" + name.replace("_", " ") + r":\s*(\d+)", station, re.M)
        if found:
            radio["counters"][name] = int(found[1])
    for direction in ("tx", "rx"):
        found = re.search(direction.upper() + r":\s*(\d+) bytes \((\d+) packets\)", link)
        if found:
            radio["counters"].setdefault(direction + "_bytes", int(found[1]))
            radio["counters"].setdefault(direction + "_packets", int(found[2]))
    driver = texts.get(f"ethtool -S {interface}", "")
    for name in COUNTERS:
        found = re.search(r"^\s*" + name + r":\s*(\d+)", driver, re.M)
        if found:
            radio["counters"].setdefault(name, int(found[1]))
    survey = texts.get(f"iw dev {interface} survey dump", "")
    for block in re.split(r"Survey data from", survey):
        if "[in use]" not in block:
            continue
        radio["survey_ms"] = {}
        for name, field in (
            ("active", "active"),
            ("busy", "busy"),
            ("rx", "receive"),
            ("tx", "transmit"),
        ):
            found = re.search(r"channel " + field + r" time:\s*(\d+) ms", block)
            if found:
                radio["survey_ms"][name] = int(found[1])
    qdisc = texts.get(f"tc -s qdisc show dev {interface}", "")
    # Root counters already include children; do not sum and double count them.
    root = re.search(r"qdisc [^\n]* root[^\n]*\n(.*?)(?=\nqdisc|\Z)", qdisc, re.S)
    if root:
        dropped = re.search(r"dropped (\d+)", root[1])
        backlog = re.search(r"backlog (\d+)([KMG]?)b (\d+)p", root[1])
        radio["qdisc"] = {
            "drops": int(dropped[1]) if dropped else None,
            "backlog_bytes": int(backlog[1])
            * {"": 1, "K": 1024, "M": 1024**2, "G": 1024**3}[backlog[2]]
            if backlog
            else None,
            "backlog_packets": int(backlog[3]) if backlog else None,
        }
    radio["coverage"] = {
        "association": bool(link or station),
        "radio_signal": radio["signal_dbm"] is not None,
        "retry_counters": "tx_retries" in radio["counters"],
        "channel_airtime": bool(radio.get("survey_ms")),
        "ap_queues": False,
        "note": "Station driver counters; missing values are unavailable, never zero. Channel busy time is local radio observation, not AP queue delay.",
    }
    return radio


def darwin_radio(probes: list[dict[str, Any]], interface: str) -> dict[str, Any]:
    """Use the OS wireless inventory; macOS privacy may withhold network identity."""
    radio: dict[str, Any] = {
        "interface": interface,
        "source": "macOS system_profiler",
        "counters": {},
        "bssid": None,
        "associated": None,
    }
    for probe in probes:
        if probe.get("command", [""])[0] == "corewlan" and probe.get("returncode") == 0:
            try:
                measured = json.loads(probe.get("stdout", ""))
                if measured.get("interface") == interface:
                    radio.update(measured)
            except ValueError:
                pass
        if "SPAirPortDataType" not in probe.get("command", []) or probe.get("returncode") != 0:
            continue
        try:
            data = json.loads(probe.get("stdout", ""))
        except ValueError:
            continue
        for adapter in data.get("SPAirPortDataType", []):
            for item in adapter.get("spairport_airport_interfaces", []):
                if item.get("_name") != interface:
                    continue
                network = item.get("spairport_current_network_information", {})
                signal = re.search(
                    r"(-?\d+) dBm / (-?\d+) dBm", network.get("spairport_signal_noise", "")
                )
                radio.update(
                    associated=bool(network.get("spairport_network_channel")),
                    channel=network.get("spairport_network_channel"),
                    tx_rate_mbps=network.get("spairport_network_rate"),
                    signal_dbm=int(signal[1]) if signal else None,
                    noise_dbm=int(signal[2]) if signal else None,
                )
    radio["coverage"] = {
        "association": radio["associated"] is not None,
        "association_identity": radio.get("bssid") is not None,
        "radio_signal": radio.get("signal_dbm") is not None,
        "retry_counters": False,
        "channel_airtime": False,
        "ap_queues": False,
        "note": "OS radio measurement; privacy permissions may withhold BSSID. Retry and AP queue counters require infrastructure evidence.",
    }
    return radio


class RadioHistory:
    """Compute interval counters only within the same association and clock sequence."""

    def __init__(self) -> None:
        self.previous: dict[str, dict[str, Any]] = {}

    def observe(self, radio: dict[str, Any], at_ns: int) -> dict[str, Any]:
        key = str(radio["interface"])
        old = self.previous.get(key)
        self.previous[key] = {**radio, "at_ns": at_ns}
        result = {**radio, "mono_ns": at_ns, "association_changed": False, "delta": {}}
        if old is None or at_ns <= old["at_ns"]:
            result["interval_status"] = "baseline"
            return result
        if old.get("bssid") != radio.get("bssid"):
            result["association_changed"] = True
            result["previous_bssid"] = old.get("bssid")
            result["interval_status"] = "association_changed"
            return result
        elapsed = (at_ns - old["at_ns"]) / 1e9
        result["interval_s"] = elapsed
        result["interval_status"] = "measured"
        for name, value in radio.get("counters", {}).items():
            previous = old.get("counters", {}).get(name)
            if previous is None:
                continue
            if value < previous:
                result["interval_status"] = "counter_reset"
                result["delta"] = {}
                return result
            result["delta"][name] = value - previous
        packets = result["delta"].get("tx_packets", 0)
        if packets:
            for name in ("tx_retries", "tx_failed"):
                if name in result["delta"]:
                    # Retry count may exceed packet count: this is not a loss probability.
                    result[name + "_per_packet"] = result["delta"][name] / packets
        for name in ("tx_bytes", "rx_bytes"):
            if name in result["delta"]:
                result[name + "_per_s"] = result["delta"][name] / elapsed
        survey, previous = radio.get("survey_ms", {}), old.get("survey_ms", {})
        active = survey.get("active", 0) - previous.get("active", 0)
        busy = survey.get("busy", 0) - previous.get("busy", 0)
        if (
            {"active", "busy"} <= survey.keys() & previous.keys()
            and active > 0
            and 0 <= busy <= active
        ):
            result["channel_busy_fraction"] = busy / active
        return result
