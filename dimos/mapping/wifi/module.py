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

from collections.abc import AsyncIterator
from copy import deepcopy
from dataclasses import asdict
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, TextIO

import numpy as np
import open3d as o3d
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.wifi.coverage import CoverageMap
from dimos.mapping.wifi.survey import SurveyMap
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.sensor_msgs.SurveyMetrics import SurveyMetrics
from dimos.msgs.sensor_msgs.WifiScan import WifiScan


class WifiCoverageConfig(ModuleConfig):
    cell_size: float = Field(default=0.5, gt=0)
    max_cells: int = Field(default=10000, ge=1)
    max_pose_age_s: float = Field(default=0.5, gt=0)
    max_sample_age_s: float = Field(default=5.0, gt=0)
    # Empty selects the strongest observed AP in each cell.
    bssid: str = ""
    max_samples: int = Field(default=20000, ge=1)
    max_clock_error_s: float = Field(default=0.5, ge=0)
    # A new file, never overwrite a previous survey. Contains metadata, no images.
    record_path: Path | None = None


class WifiCoverage(Module):
    """Map radio and measured system performance at the receiver's locations."""

    config: WifiCoverageConfig
    odometry: In[Odometry]
    wifi_scan: In[WifiScan]
    survey_metrics: In[SurveyMetrics]
    wifi_coverage: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.RLock()
        self._map = CoverageMap(
            self.config.cell_size,
            self.config.max_cells,
            self.config.max_pose_age_s,
            self.config.max_sample_age_s,
        )
        self._bssid = self.config.bssid.lower()
        self._survey = SurveyMap(self._map, self.config.max_samples, self.config.max_clock_error_s)
        self._layer = ("rssi_dbm", "", -40.0, -90.0)
        self._statistic = "mean"
        self._recording: TextIO | None = None

    async def main(self) -> AsyncIterator[None]:
        if self.config.record_path:
            self.config.record_path.parent.mkdir(parents=True, exist_ok=True)
            self._recording = self.config.record_path.open("x", buffering=1)
        try:
            if self._recording is not None:
                self._recording.write(
                    json.dumps(
                        {
                            "kind": "survey",
                            "version": 1,
                            "started_at": time.time(),
                            "epoch": self._survey.epoch,
                            "cell_size": self.config.cell_size,
                            "max_pose_age_s": self.config.max_pose_age_s,
                            "max_sample_age_s": self.config.max_sample_age_s,
                            "max_clock_error_s": self.config.max_clock_error_s,
                        }
                    )
                    + "\n"
                )
            yield
        finally:
            with self._lock:
                if self._recording is not None:
                    self._recording.close()
                    self._recording = None

    async def handle_odometry(self, msg: Odometry) -> None:
        p = msg.pose.position
        with self._lock:
            previous_frame = self._map.frame_id
            self._map.add_pose(msg.ts, msg.frame_id, (p.x, p.y, p.z))
            if previous_frame and previous_frame != self._map.frame_id:
                self._survey.clear()
                self._publish(msg.ts)
            if self._recording is not None and all(
                math.isfinite(v) for v in (msg.ts, p.x, p.y, p.z)
            ):
                self._recording.write(
                    json.dumps(
                        {
                            "kind": "pose",
                            "epoch": self._survey.epoch,
                            "ts": msg.ts,
                            "frame_id": msg.frame_id,
                            "position": [p.x, p.y, p.z],
                        },
                        allow_nan=False,
                    )
                    + "\n"
                )

    async def handle_wifi_scan(self, msg: WifiScan) -> None:
        with self._lock:
            self._survey.add_wifi(msg)
            if self._recording is not None:
                self._recording.write(
                    json.dumps(
                        {"kind": "wifi", "epoch": self._survey.epoch, **asdict(msg)},
                        allow_nan=False,
                    )
                    + "\n"
                )
            if not self._map.add_scan(msg):
                return
            if self._layer[0] == "rssi_dbm":
                self._publish(msg.ts)

    async def handle_survey_metrics(self, msg: SurveyMetrics) -> None:
        with self._lock:
            rows = self._survey.add(msg)
            if self._recording is not None:
                for row in rows:
                    self._recording.write(
                        json.dumps({"kind": "performance", **row}, allow_nan=False) + "\n"
                    )
            if rows and self._layer[0] != "rssi_dbm":
                self._publish(msg.ts)

    def _publish(self, ts: float) -> None:
        metric, _, good, bad = self._layer
        cells = self.get_cells()
        points = np.asarray([cell["position"] for cell in cells], dtype=np.float64).reshape(-1, 3)
        signal = np.asarray(
            [cell["mean_dbm"] if metric == "rssi_dbm" else cell[self._statistic] for cell in cells]
        )
        strength = np.clip((signal - bad) / (good - bad), 0, 1)
        colors = np.column_stack(
            (np.minimum(2 * (1 - strength), 1), np.minimum(2 * strength, 1), np.zeros(len(cells)))
        )
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        cloud.colors = o3d.utility.Vector3dVector(colors)
        self.wifi_coverage.publish(PointCloud2(cloud, frame_id=self._map.frame_id, ts=ts))

    @rpc
    def select_access_point(self, bssid: str = "") -> None:
        """Filter by BSSID; empty combines performance observations or shows strongest RSSI."""
        with self._lock:
            self._bssid = bssid.lower()
            self._publish(time.time())

    @rpc
    def get_cells(self) -> list[dict[str, Any]]:
        """Return selected cells with sample counts, value ranges and observation timestamps."""
        with self._lock:
            metric, source, _, _ = self._layer
            if metric != "rssi_dbm":
                return self._survey.cells(metric, source, self._bssid)
            return [asdict(cell) for cell in self._map.selected_cells(self._bssid)]

    @rpc
    def get_layers(self) -> list[dict[str, str]]:
        """List available metric/source pairs; different viewers and targets stay separate."""
        with self._lock:
            return [{"metric": "rssi_dbm", "source": ""}, *self._survey.layers()]

    @rpc
    def select_metric(
        self, metric: str, source: str, good: float, bad: float, statistic: str = "mean"
    ) -> None:
        """Select a map layer and statistic, with explicit green/red scale endpoints."""
        if not all(math.isfinite(v) for v in (good, bad)) or good == bad:
            raise ValueError("good and bad must be different finite values")
        if statistic not in ("mean", "p50", "p95", "min", "max") or (
            metric == "rssi_dbm" and statistic != "mean"
        ):
            raise ValueError("Use mean, p50, p95, min or max; RSSI supports mean only")
        with self._lock:
            if {"metric": metric, "source": source} not in self.get_layers():
                raise ValueError("Unknown layer; use get_layers()")
            self._layer = (metric, source, good, bad)
            self._statistic = statistic
            self._publish(time.time())

    @rpc
    def get_status(self) -> dict[str, Any]:
        """Describe the selected layer, survey epoch, retention and archive destination."""
        with self._lock:
            metric, source, good, bad = self._layer
            return {
                "metric": metric,
                "source": source,
                "statistic": self._statistic,
                "good": good,
                "bad": bad,
                "bssid": self._bssid,
                "frame_id": self._map.frame_id,
                "epoch": self._survey.epoch,
                "retained_samples": len(self._survey.samples),
                "recording": self._recording is not None,
                "record_path": str(self.config.record_path) if self.config.record_path else None,
            }

    @rpc
    def get_samples(self, cell: list[int] | None = None, limit: int = 200) -> list[dict[str, Any]]:
        """Inspect recent raw samples and missing-data reasons, optionally at one grid cell."""
        if not 1 <= limit <= 2000 or (cell is not None and len(cell) != 3):
            raise ValueError("Use a three-coordinate cell and a limit between 1 and 2000")
        with self._lock:
            return deepcopy(
                [row for row in self._survey.samples if cell is None or row["cell"] == tuple(cell)][
                    -limit:
                ]
            )

    @rpc
    def get_access_points(self) -> list[dict[str, Any]]:
        """List observed BSSIDs without merging radios that share an SSID."""
        with self._lock:
            latest = {cell.bssid: cell for cell in self._map.cells.values()}
            return [
                dict(
                    bssid=cell.bssid,
                    ssid=cell.ssid,
                    frequency_mhz=cell.frequency_mhz,
                    last_seen=cell.last_seen,
                )
                for cell in latest.values()
            ]

    @rpc
    def reset(self) -> None:
        """Clear coverage and pose history before a new localization/map epoch."""
        with self._lock:
            frame_id = self._map.frame_id
            self._map.clear()
            self._survey.clear()
            self.wifi_coverage.publish(PointCloud2(frame_id=frame_id, ts=time.time()))
