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

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import os
import re
import time

import psutil

from dimos.msgs.sensor_msgs.SurveyMetrics import SurveySample


async def probe(interface: str, name: str, target: str) -> SurveySample:
    """One Linux ICMP probe, bound to the surveyed interface; no throughput load."""
    started, monotonic = time.time(), time.monotonic()
    process = await asyncio.create_subprocess_exec(
        "ping",
        "-n",
        "-I",
        interface,
        "-c",
        "1",
        "-W",
        "1",
        target,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
        env={**os.environ, "LC_ALL": "C"},
    )
    try:
        stdout, stderr = await asyncio.wait_for(process.communicate(), timeout=2)
    finally:
        if process.returncode is None:
            process.kill()
            await process.wait()
    elapsed = time.monotonic() - monotonic
    labels = {"target": target, "interface": interface}
    values: dict[str, float] = {}
    match = re.search(rb"time[=<]([\d.]+)\s*ms", stdout)
    if process.returncode == 0 and match:
        values = {"rtt_ms": float(match[1]), "probe_loss_pct": 0.0}
    elif process.returncode == 1:
        values = {"probe_loss_pct": 100.0}
    else:
        labels["error"] = (stderr or stdout).decode(errors="replace")[-300:]
    return SurveySample(started + elapsed, f"ping/{name}", values, elapsed, labels=labels)


@dataclass(frozen=True)
class HostCounters:
    ts: float
    monotonic: float
    busy: float
    total: float
    tx_bytes: int
    rx_bytes: int

    @classmethod
    def read(cls, interface: str) -> HostCounters:
        cpu = psutil.cpu_times()
        busy = sum(
            getattr(cpu, name, 0.0)
            for name in ("user", "nice", "system", "irq", "softirq", "steal")
        )
        total = busy + cpu.idle + getattr(cpu, "iowait", 0.0)
        net = psutil.net_io_counters(pernic=True, nowrap=False)[interface]
        return cls(time.time(), time.monotonic(), busy, total, net.bytes_sent, net.bytes_recv)

    def since(self, before: HostCounters, interface: str) -> SurveySample:
        elapsed = self.monotonic - before.monotonic
        values: dict[str, float] = {}
        if elapsed > 0:
            for name, delta in (
                ("tx_mbps", self.tx_bytes - before.tx_bytes),
                ("rx_mbps", self.rx_bytes - before.rx_bytes),
            ):
                if delta >= 0:
                    values[name] = delta * 8 / elapsed / 1e6
        total, busy = self.total - before.total, self.busy - before.busy
        if total > 0 and 0 <= busy <= total:
            values["cpu_pct"] = 100 * busy / total
        return SurveySample(
            self.ts, "host", values, max(0, elapsed), labels={"interface": interface}
        )
