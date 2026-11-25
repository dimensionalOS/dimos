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

"""Retain existing survey observations in the run's bounded incident capture.

These functions consume measurements already made by modules. They never create
subscriptions, poll hardware, or start another collection loop.
"""

from __future__ import annotations

from dataclasses import asdict
import time

from dimos.diagnostics.runtime import get_runtime
from dimos.msgs.sensor_msgs.SurveyMetrics import SurveyMetrics
from dimos.msgs.sensor_msgs.WifiScan import WifiScan


def record_survey(batch: SurveyMetrics) -> None:
    runtime = get_runtime()
    if runtime is None:
        return
    for sample in batch.samples:
        runtime.event(
            "survey.measurement",
            interface=batch.interface,
            observation=asdict(sample),
        )
        if sample.source.startswith("ping/"):
            error = sample.labels.get("error")
            status = (
                "error"
                if error
                else "ok"
                if "rtt_ms" in sample.values
                else "no_reply"
                if sample.values.get("probe_loss_pct") == 100.0
                else "unavailable"
            )
            runtime.event(
                "network.path_probe",
                host=runtime.host,
                measurement={
                    "address": sample.labels.get("target"),
                    "role": sample.source.removeprefix("ping/"),
                    "status": status,
                    "reason": error,
                    "rtt_ms": sample.values.get("rtt_ms"),
                    "observed_wall_s": sample.ts,
                    "duration_s": sample.duration_s,
                    "scope": "ICMP round trip including scheduling; not isolated radio airtime",
                },
            )


def record_wifi(scan: WifiScan) -> None:
    runtime = get_runtime()
    if runtime is None:
        return
    # Observations retain their acquisition time, including old scan-cache rows.
    # A missing association remains missing; it is not a zero-strength radio.
    for point in scan.access_points:
        runtime.event("survey.wifi", interface=scan.interface, observation=asdict(point))
    connected = next((point for point in scan.access_points if point.connected), None)
    runtime.event(
        "network.wifi",
        host=runtime.host,
        radio={
            "interface": scan.interface,
            "source": "WifiMonitor / Linux iw",
            "bssid": connected.bssid if connected else None,
            "signal_dbm": connected.signal_dbm if connected else None,
            "frequency_mhz": connected.frequency_mhz if connected else None,
            "tx_rate_mbps": connected.tx_bitrate_mbps if connected else None,
            "rx_rate_mbps": connected.rx_bitrate_mbps if connected else None,
            "associated": connected is not None,
            "observed_wall_s": scan.ts,
            "mono_ns": time.monotonic_ns(),
            "coverage": {"retry_counters": False, "channel_airtime": False},
        },
    )
