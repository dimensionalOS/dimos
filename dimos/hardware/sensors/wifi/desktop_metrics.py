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

"""Optional adapter for existing Spatial Desktop telemetry; no media subscriptions."""

from __future__ import annotations

import math
from typing import Any

from dimos.msgs.sensor_msgs.SurveyMetrics import SurveySample


def number(value: Any) -> float | None:
    if isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value):
        return float(value)
    return None


def metrics(value: dict[str, Any], fields: dict[str, str]) -> dict[str, float]:
    return {
        name: n for field, name in fields.items() if (n := number(value.get(field))) is not None
    }


class DesktopMetrics:
    """Translate browser time using the gateway clock estimate, retaining its uncertainty."""

    def __init__(self) -> None:
        self._previous: dict[str, float] = {}

    def read(self, document: dict[str, Any], now: float) -> list[SurveySample]:
        output = []
        clients = document.get("clients")
        if not isinstance(clients, list):
            raise ValueError("Expected a Spatial Desktop diagnostics response with a clients list")
        for client in clients[:16]:
            if not isinstance(client, dict):
                continue
            identity, at = client.get("clientId"), number(client.get("at"))
            if not isinstance(identity, str) or at is None:
                continue
            received = number(client.get("receivedAt"))
            if received is not None and received <= now * 1000:
                output.append(
                    SurveySample(
                        now,
                        f"viewer/{identity}/telemetry",
                        {"report_arrival_age_ms": now * 1000 - received},
                    )
                )
            previous = self._previous.get(identity)
            if previous is not None and at <= previous:
                continue
            self._previous[identity] = at
            while len(self._previous) > 32:
                del self._previous[next(iter(self._previous))]
            link = mapping(client.get("link"))
            gateway = mapping(link.get("gateway"))
            clock = mapping(gateway.get("clock"))
            offset, rtt = number(clock.get("offsetMs")), number(clock.get("rttMs"))
            source = f"viewer/{identity}"
            labels = {
                "visibility": str(client.get("visibility", "unknown")),
                "run_id": str(client.get("runId", "")),
            }
            if offset is None or rtt is None or rtt < 0:
                output.append(
                    SurveySample(now, source, {}, labels={**labels, "error": "clock_unavailable"})
                )
                continue
            ts = (at + offset) / 1000  # gateway clock minus browser clock
            if client.get("stale") or not -rtt / 2000 <= now - ts <= 5:
                output.append(
                    SurveySample(now, source, {}, labels={**labels, "error": "stale_report"})
                )
                continue
            labels["connection"] = str(client.get("connection", "unknown"))
            duration = (at - previous) / 1000 if previous is not None else 0.0
            values = metrics(link, {"rttMs": "application_rtt_ms"})
            pair = mapping(link.get("pair"))
            if (rtc := number(pair.get("currentRoundTripTime"))) is not None:
                values["rtc_rtt_ms"] = rtc * 1000
            if client.get("connection") != "connected":
                values = {}  # retained gateway stats may belong to a dead connection
                labels["error"] = "viewer_disconnected"
            output.append(SurveySample(ts, source, values, 0, rtt / 2000, labels))
            if client.get("connection") != "connected":
                continue
            cameras = client.get("cameras")
            if not isinstance(cameras, list):
                continue
            for camera in cameras[:32]:
                if not isinstance(camera, dict):
                    continue
                key = camera.get("key")
                if not isinstance(key, str):
                    continue
                # Rates need two reports; large gaps cannot be assigned to a single location.
                rates = (
                    metrics(
                        camera,
                        {
                            "sourceFps": "source_fps",
                            "sentFps": "sent_fps",
                            "receivedFps": "received_fps",
                            "decodedFps": "decoded_fps",
                            "displayedFps": "displayed_fps",
                        },
                    )
                    if previous is not None
                    else {}
                )
                output.append(
                    SurveySample(ts, f"{source}/{key}", rates, duration, rtt / 2000, labels)
                )
                if (age := number(camera.get("lastDisplayedAgoMs"))) is not None:
                    output.append(
                        SurveySample(
                            ts,
                            f"{source}/{key}/freshness",
                            {"last_displayed_ago_ms": age},
                            0,
                            rtt / 2000,
                            labels,
                        )
                    )
        return output


def mapping(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}
