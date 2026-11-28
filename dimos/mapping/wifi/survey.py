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

"""Position-aligned survey evidence and selectable performance layers."""

from __future__ import annotations

from collections import OrderedDict, deque
from dataclasses import asdict
import math
from typing import Any
from uuid import uuid4

import numpy as np

from dimos.mapping.wifi.coverage import CoverageMap, Position
from dimos.msgs.sensor_msgs.SurveyMetrics import SurveyMetrics
from dimos.msgs.sensor_msgs.WifiScan import WifiScan


class SurveyMap:
    def __init__(
        self, coverage: CoverageMap, max_samples: int = 20000, max_clock_error_s: float = 0.5
    ) -> None:
        self.coverage = coverage
        self.max_clock_error_s = max_clock_error_s
        self.samples: deque[dict[str, Any]] = deque(maxlen=max_samples)
        self._wifi: deque[tuple[float, str, str | None]] = deque(maxlen=2048)
        self._seen: OrderedDict[str, float] = OrderedDict()
        self.epoch = uuid4().hex

    def clear(self) -> None:
        self.samples.clear()
        self._wifi.clear()
        self._seen.clear()
        self.epoch = uuid4().hex

    def add_wifi(self, scan: WifiScan) -> None:
        bssid = next((ap.bssid for ap in scan.access_points if ap.connected), None)
        self._wifi.append((scan.ts, scan.interface, bssid))

    def cell(self, position: Position) -> tuple[int, int, int]:
        x, y, z = (math.floor(v / self.coverage.cell_size) for v in position)
        return x, y, z

    def add(self, batch: SurveyMetrics) -> list[dict[str, Any]]:
        rows = []
        for sample in batch.samples:
            # Retransmitted or repeatedly polled telemetry must not bias cell statistics.
            identity = f"{batch.interface}/{sample.source}"
            if math.isfinite(sample.ts) and sample.ts <= self._seen.get(identity, -math.inf):
                continue
            values = {k: v for k, v in sample.values.items() if math.isfinite(v)}
            reason: str | None = None
            position: Position | None = None
            start = sample.ts - sample.duration_s
            if (
                not all(
                    math.isfinite(v)
                    for v in (batch.ts, sample.ts, sample.duration_s, sample.time_uncertainty_s)
                )
                or sample.duration_s < 0
                or sample.time_uncertainty_s < 0
            ):
                reason = "invalid_timestamp"
            elif sample.labels.get("error"):
                reason = sample.labels["error"]
            elif not values:
                reason = "no_measurement"
            elif sample.time_uncertainty_s > self.max_clock_error_s:
                reason = "clock_uncertain"
            elif (
                not -sample.time_uncertainty_s
                <= batch.ts - sample.ts
                <= self.coverage.max_sample_age_s
            ):
                reason = "stale_measurement"
            else:
                position = self.coverage.position_at((start + sample.ts) / 2)
                before = self.coverage.position_at(start - sample.time_uncertainty_s)
                after = self.coverage.position_at(sample.ts + sample.time_uncertainty_s)
                if position is None or before is None or after is None:
                    reason = "missing_pose"
                elif self.cell(before) != self.cell(after) or self.cell(before) != self.cell(
                    position
                ):
                    reason = "measurement_spans_cells"
            if math.isfinite(sample.ts):
                self._seen[identity] = sample.ts
                self._seen.move_to_end(identity)
                if len(self._seen) > 2048:
                    self._seen.popitem(last=False)
            radios = [
                entry
                for entry in self._wifi
                if entry[1] == batch.interface and 0 <= sample.ts - entry[0] <= 2
            ]
            radio = max(radios, default=None, key=lambda entry: entry[0])
            data = asdict(sample)
            # Invalid timestamps remain explicit, and the archive remains strict JSON.
            for field in ("ts", "duration_s", "time_uncertainty_s"):
                if not math.isfinite(data[field]):
                    data[field] = None
            row = {
                **data,
                "values": values,
                "epoch": self.epoch,
                "interface": batch.interface,
                "frame_id": self.coverage.frame_id,
                "position": position,
                "cell": self.cell(position) if position is not None else None,
                "bssid": radio[2] if radio else None,
                "reason": reason,
            }
            self.samples.append(row)
            rows.append(row)
        return rows

    def layers(self) -> list[dict[str, str]]:
        layers = {
            (row["source"], key)
            for row in self.samples
            if row["reason"] is None
            for key in row["values"]
        }
        return [{"source": source, "metric": metric} for source, metric in sorted(layers)]

    def cells(self, metric: str, source: str, bssid: str = "") -> list[dict[str, Any]]:
        groups: dict[tuple[int, ...], list[dict[str, Any]]] = {}
        for row in self.samples:
            if (
                row["reason"] is not None
                or row["source"] != source
                or metric not in row["values"]
                or (bssid and row["bssid"] != bssid)
            ):
                continue
            groups.setdefault(row["cell"], []).append(row)
        result = []
        for cell, rows in groups.items():
            values = [row["values"][metric] for row in rows]
            result.append(
                {
                    "cell": cell,
                    "position": tuple((v + 0.5) * self.coverage.cell_size for v in cell),
                    "metric": metric,
                    "source": source,
                    "count": len(values),
                    "mean": float(np.mean(values)),
                    "p50": float(np.percentile(values, 50)),
                    "p95": float(np.percentile(values, 95)),
                    "min": min(values),
                    "max": max(values),
                    "first_seen": min(row["ts"] for row in rows),
                    "last_seen": max(row["ts"] for row in rows),
                    "bssids": sorted({row["bssid"] for row in rows if row["bssid"]}),
                }
            )
        return result
