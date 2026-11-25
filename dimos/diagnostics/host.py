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

"""Read-only host snapshots; unsupported counters remain absent."""

from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import socket
import subprocess
import time
from typing import Any

import psutil

from dimos.core.resource_monitor.stats import collect_process_stats
from dimos.diagnostics.clock import clock_domain
from dimos.hardware.sensors.wifi.radio import macos_helper


def process_snapshot() -> dict[str, Any]:
    return asdict(collect_process_stats(os.getpid()))


def snapshot(interface: str | None = None) -> dict[str, Any]:
    result: dict[str, Any] = {
        "at_ns": time.time_ns(),
        "mono_ns": time.monotonic_ns(),
        "clock_domain": clock_domain(),
        "host": socket.gethostname(),
        "system": platform.system(),
        "cpu_times": psutil.cpu_times()._asdict(),
        "memory": psutil.virtual_memory()._asdict(),
        "process": process_snapshot(),
        "interfaces": {
            key: value._asdict() for key, value in psutil.net_io_counters(pernic=True).items()
        },
        "note": "Endpoint counters do not isolate AP airtime or over-air transmission.",
    }
    commands = []
    if platform.system() == "Linux":
        commands.append(["ip", "-json", "route", "show"])
        if interface:
            commands += [
                ["iw", "dev", interface, "link"],
                ["iw", "dev", interface, "station", "dump"],
                ["iw", "dev", interface, "survey", "dump"],
                ["tc", "-s", "qdisc", "show", "dev", interface],
                ["ethtool", "-S", interface],
            ]
    elif platform.system() == "Darwin":
        commands.append(["route", "-n", "get", "default"])
        if interface:
            commands += [
                ["networksetup", "-getairportnetwork", interface],
                ["ifconfig", interface],
            ]
    result["capabilities"] = {
        "interface_counters": True,
        "route": platform.system() in ("Linux", "Darwin"),
        "wifi_station": "iw" if platform.system() == "Linux" else "OS permission dependent",
        "ap_airtime": "requires infrastructure telemetry",
        "socket_timestamps": "separate probe required",
    }
    result["probes"] = []
    if platform.system() == "Darwin" and interface:
        helper = macos_helper()
        if helper:
            try:
                response = subprocess.run(
                    [helper, interface], capture_output=True, text=True, timeout=2, check=False
                )
                result["probes"].append(
                    {
                        "command": ["corewlan", interface],
                        "returncode": response.returncode,
                        "stdout": response.stdout,
                        "stderr": response.stderr[:1024],
                    }
                )
            except subprocess.TimeoutExpired:
                result["probes"].append({"command": ["corewlan", interface], "status": "timeout"})
        else:
            result["probes"].append(
                {
                    "command": ["corewlan", interface],
                    "status": "unavailable",
                    "reason": "Swift compiler/helper unavailable",
                }
            )
    for command in commands:
        if not shutil.which(command[0]):
            result["probes"].append({"command": command, "status": "unavailable"})
            continue
        try:
            response = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=5 if command[0] == "system_profiler" else 2,
                check=False,
            )
            result["probes"].append(
                {
                    "command": command,
                    "returncode": response.returncode,
                    "stdout": response.stdout[:65536],
                    "stderr": response.stderr[:4096],
                }
            )
        except subprocess.TimeoutExpired:
            result["probes"].append({"command": command, "status": "timeout"})
    return result


def infrastructure_snapshot(path: Path, *, maximum_age_s: float = 5.0) -> dict[str, Any]:
    """Read the normalized AP/controller adapter contract, never infer physical hops.

    Controllers write this atomically beside their own capture. This boundary accepts
    proprietary controller exporters without embedding their credentials in DimOS.
    Timestamp age is evaluated on the exporter's monotonic clock, supplied in the file.
    """
    if path.stat().st_size > 1024 * 1024:
        raise ValueError("infrastructure snapshot exceeds limit")
    value: dict[str, Any] = json.loads(path.read_text())
    if value.get("schema") != "dimos.infrastructure/v1" or not isinstance(value.get("links"), list):
        raise ValueError("invalid infrastructure snapshot")
    if len(value["links"]) > 1024 or not isinstance(value.get("clock_domain"), str):
        raise ValueError("invalid infrastructure topology")
    now = value.get("exported_ns")
    for link in value["links"]:
        if not all(
            isinstance(link.get(k), str) and link[k] for k in ("source", "target", "evidence")
        ):
            raise ValueError("links require endpoint IDs and evidence provenance")
        if (
            not isinstance(now, int)
            or not isinstance(link.get("observed_ns"), int)
            or now < link["observed_ns"]
        ):
            raise ValueError("invalid infrastructure clock")
        link["freshness_at_export"] = (
            "fresh" if now - link["observed_ns"] <= maximum_age_s * 1e9 else "stale"
        )
        link["local_import_age_ns"] = max(0, time.time_ns() - path.stat().st_mtime_ns)
    value["generation"] = hashlib.sha256(
        json.dumps(
            [(l["source"], l["target"], l.get("association")) for l in value["links"]],
            sort_keys=True,
        ).encode()
    ).hexdigest()[:16]
    return value
