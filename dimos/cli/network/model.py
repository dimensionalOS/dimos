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

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import math
from typing import Any

PROTOCOL_VERSION = 1


@dataclass(frozen=True)
class Settings:
    max_mbps: float = 50.0
    step_seconds: float = 5.0
    idle_seconds: float = 5.0
    warmup_seconds: float = 0.5
    drain_seconds: float = 1.0
    max_seconds: float = 90.0
    max_bytes: int = 256 * 1024 * 1024
    payload_bytes: int = 64 * 1024
    probe_hz: float = 50.0
    probe_timeout: float = 0.5
    min_goodput_mbps: float | None = None
    max_rtt_p95_ms: float | None = None
    max_missing_pct: float | None = None

    def validate(self) -> None:
        for name in (
            "max_mbps",
            "step_seconds",
            "idle_seconds",
            "max_seconds",
            "probe_hz",
            "probe_timeout",
            "drain_seconds",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if not math.isfinite(self.warmup_seconds) or self.warmup_seconds < 0:
            raise ValueError("warmup_seconds must be finite and nonnegative")
        if not 1024 <= self.payload_bytes <= 1024 * 1024:
            raise ValueError("payload_bytes must be between 1024 and 1048576")
        if not self.payload_bytes <= self.max_bytes <= 1024 * 1024 * 1024:
            raise ValueError("max_bytes must cover one message and be at most 1 GiB")
        if self.max_seconds > 600 or self.probe_hz > 200 or self.max_mbps > 1000:
            raise ValueError("safety limits: max_seconds <= 600, probe_hz <= 200, max_mbps <= 1000")
        for name in ("min_goodput_mbps", "max_rtt_p95_ms", "max_missing_pct"):
            value = getattr(self, name)
            if value is not None and (not math.isfinite(value) or value < 0):
                raise ValueError(f"{name} must be finite and nonnegative")
        if self.min_goodput_mbps is not None and self.min_goodput_mbps > self.max_mbps:
            raise ValueError("min_goodput_mbps cannot exceed max_mbps")
        if self.max_missing_pct is not None and self.max_missing_pct > 100:
            raise ValueError("max_missing_pct cannot exceed 100")

    @property
    def rates(self) -> list[float]:
        return [self.max_mbps / 8, self.max_mbps / 4, self.max_mbps / 2, self.max_mbps]

    @property
    def has_thresholds(self) -> bool:
        return any(
            v is not None
            for v in (
                self.min_goodput_mbps,
                self.max_rtt_p95_ms,
                self.max_missing_pct,
            )
        )


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    rank = (len(ordered) - 1) * fraction
    lo = math.floor(rank)
    hi = math.ceil(rank)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (rank - lo)


@dataclass
class RTT:
    samples_ms: list[float] = field(default_factory=list)
    timeouts: int = 0

    def summary(self) -> dict[str, Any]:
        return {
            "p50_ms": percentile(self.samples_ms, 0.50),
            "p95_ms": percentile(self.samples_ms, 0.95),
            "p99_ms": percentile(self.samples_ms, 0.99),
            "replies": len(self.samples_ms),
            "timeouts": self.timeouts,
            "samples_ms": self.samples_ms,
        }


@dataclass
class Report:
    session_id: str
    remote: str
    remote_dimos: str
    settings: Settings
    local_zenoh: str
    remote_zenoh: str = "unknown"
    endpoint: str = "unknown"
    status: str = "running"
    cleanup: str = "pending"
    baseline: dict[str, Any] = field(default_factory=dict)
    directions: dict[str, list[dict[str, Any]]] = field(
        default_factory=lambda: {
            "remote_to_local": [],
            "local_to_remote": [],
        }
    )
    stop_reasons: dict[str, str] = field(default_factory=dict)
    verdict: str = "not_requested"
    error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": PROTOCOL_VERSION,
            **asdict(self),
            "transport": "zenoh/tcp",
            "qos": "reliable/drop; probes express, same default priority",
        }


def meets_thresholds(step: dict[str, Any], settings: Settings) -> bool:
    if not settings.has_thresholds:
        return False
    rx, rtt = step["receiver"], step["rtt"]
    # Timeouts cannot silently turn a missed latency deadline into a passing p95.
    return (
        (settings.min_goodput_mbps is None or rx["goodput_mbps"] >= settings.min_goodput_mbps)
        and (
            settings.max_rtt_p95_ms is None
            or (
                rtt["p95_ms"] is not None
                and rtt["p95_ms"] <= settings.max_rtt_p95_ms
                and rtt["timeouts"] == 0
            )
        )
        and (settings.max_missing_pct is None or rx["missing_pct"] <= settings.max_missing_pct)
    )
