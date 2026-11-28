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

"""Bounded, timestamp-aligned receiver coverage; independent of acquisition and transport."""

from __future__ import annotations

from bisect import bisect_left
from collections import OrderedDict
from dataclasses import dataclass
import math

from dimos.msgs.sensor_msgs.WifiScan import WifiAccessPoint, WifiScan

Position = tuple[float, float, float]
CellKey = tuple[str, int, int, int]


@dataclass
class CoverageCell:
    bssid: str
    ssid: str
    frequency_mhz: int
    position: Position
    count: int
    mean_dbm: float
    min_dbm: float
    max_dbm: float
    last_seen: float

    def update(self, sample: WifiAccessPoint) -> None:
        self.count += 1
        self.mean_dbm += (sample.signal_dbm - self.mean_dbm) / self.count
        self.min_dbm = min(self.min_dbm, sample.signal_dbm)
        self.max_dbm = max(self.max_dbm, sample.signal_dbm)
        self.last_seen = sample.ts
        self.ssid = sample.ssid
        self.frequency_mhz = sample.frequency_mhz


class CoverageMap:
    def __init__(
        self,
        cell_size: float = 0.5,
        max_cells: int = 10000,
        max_pose_age_s: float = 0.5,
        max_sample_age_s: float = 5.0,
    ) -> None:
        if not math.isfinite(cell_size) or cell_size <= 0 or max_cells < 1:
            raise ValueError("cell_size and max_cells must be positive")
        if not all(math.isfinite(v) and v > 0 for v in (max_pose_age_s, max_sample_age_s)):
            raise ValueError("observation and pose age limits must be positive")
        self.cell_size = cell_size
        self.max_cells = max_cells
        self.max_pose_age_s = max_pose_age_s
        self.max_sample_age_s = max_sample_age_s
        self.frame_id = ""
        self.cells: OrderedDict[CellKey, CoverageCell] = OrderedDict()
        self._poses: list[tuple[float, Position]] = []
        self._seen: OrderedDict[tuple[str, str], float] = OrderedDict()

    def clear(self) -> None:
        self.cells.clear()
        self._poses.clear()
        self._seen.clear()
        self.frame_id = ""

    def add_pose(self, ts: float, frame_id: str, position: Position) -> None:
        if not frame_id or not all(math.isfinite(value) for value in (ts, *position)):
            return
        if self.frame_id and frame_id != self.frame_id:
            self.clear()
        self.frame_id = frame_id
        index = bisect_left(self._poses, ts, key=lambda pose: pose[0])
        if index < len(self._poses) and self._poses[index][0] == ts:
            self._poses[index] = (ts, position)
        else:
            self._poses.insert(index, (ts, position))
        del self._poses[:-2048]

    def position_at(self, ts: float) -> Position | None:
        if not self._poses:
            return None
        index = bisect_left(self._poses, ts, key=lambda pose: pose[0])
        if 0 < index < len(self._poses):
            before, after = self._poses[index - 1], self._poses[index]
            if max(ts - before[0], after[0] - ts) <= self.max_pose_age_s:
                fraction = (ts - before[0]) / (after[0] - before[0])
                x, y, z = (a + fraction * (b - a) for a, b in zip(before[1], after[1], strict=True))
                return x, y, z
        nearest = min(
            self._poses[max(0, index - 1) : index + 1], key=lambda pose: abs(pose[0] - ts)
        )
        return nearest[1] if abs(nearest[0] - ts) <= self.max_pose_age_s else None

    def add_scan(self, scan: WifiScan) -> int:
        added = 0
        for sample in scan.access_points:
            if not all(math.isfinite(v) for v in (scan.ts, sample.ts, sample.signal_dbm)):
                continue
            if not 0 <= scan.ts - sample.ts <= self.max_sample_age_s:
                continue
            identity = (scan.interface, sample.bssid)
            # iw's cached age has millisecond timing jitter between reads.
            if sample.ts <= self._seen.get(identity, -math.inf) + 0.1:
                continue
            position = self.position_at(sample.ts)
            if position is None:
                continue
            self._seen[identity] = sample.ts
            self._seen.move_to_end(identity)
            if len(self._seen) > self.max_cells:
                self._seen.popitem(last=False)
            x, y, z = (math.floor(value / self.cell_size) for value in position)
            key = (sample.bssid, x, y, z)
            cell = self.cells.get(key)
            if cell is None:
                center = (
                    (x + 0.5) * self.cell_size,
                    (y + 0.5) * self.cell_size,
                    (z + 0.5) * self.cell_size,
                )
                cell = CoverageCell(
                    sample.bssid,
                    sample.ssid,
                    sample.frequency_mhz,
                    center,
                    1,
                    sample.signal_dbm,
                    sample.signal_dbm,
                    sample.signal_dbm,
                    sample.ts,
                )
                self.cells[key] = cell
            else:
                cell.update(sample)
            self.cells.move_to_end(key)
            if len(self.cells) > self.max_cells:
                self.cells.popitem(last=False)
            added += 1
        return added

    def selected_cells(self, bssid: str = "") -> list[CoverageCell]:
        if bssid:
            return [cell for cell in self.cells.values() if cell.bssid == bssid]
        strongest: dict[Position, CoverageCell] = {}
        for cell in self.cells.values():
            previous = strongest.get(cell.position)
            if previous is None or cell.mean_dbm > previous.mean_dbm:
                strongest[cell.position] = cell
        return list(strongest.values())
