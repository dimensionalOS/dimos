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

"""Rolling lidar acquisition from pose history and generic ray queries."""

from collections import deque
from collections.abc import Callable
import math
from typing import Any

from dimos_lcm.sim_msgs import WorldState
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.assets.model import RobotModel, UsdModel
from dimos.sim2.geometry.poses import pose_array, poses_by_id
from dimos.sim2.publication import PublicationRate
from dimos.sim2.sensors.lidar.config import LidarConfig, Raycaster
from dimos.sim2.sensors.poses import PoseSensorConfig, _PoseSensorModule
from dimos.sim2.sensors.sampling import Acquisition


class LidarModuleConfig(PoseSensorConfig):
    sensor: LidarConfig
    raycaster: Callable[[], Raycaster]
    model: UsdModel | None = None


class SimLidarModule(_PoseSensorModule):
    config: LidarModuleConfig
    lidar: Out[PointCloud2]
    # Privileged, truth-registered hits for inspection/mapping without SLAM.
    # Never connect this stream to a lidar-inertial estimator.
    pointcloud: Out[PointCloud2]

    @rpc
    def get_usd(self) -> list[str]:
        model = (
            self.config.model
            or RobotModel.from_root_link(self.config.sensor.frame_id, name="lidar").to_usd()
        )
        if self.config.sensor.frame_id not in model.frame_names():
            raise ValueError("lidar USD must describe its measurement frame")
        return [str(model.source_path.parent) if model.source_path else "", model.usd]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        c = self.config.sensor
        if (
            not all(
                map(math.isfinite, (c.scan_hz, c.motion_hz, c.min_range, c.max_range, c.range_std))
            )
            or not 0 < c.scan_hz <= c.motion_hz
            or not 0 <= c.min_range < c.max_range
            or c.range_std < 0
        ):
            raise ValueError("invalid lidar timing/range model")
        self._pattern = c.pattern()
        self._response = c.response() if c.response is not None else None
        self._history: deque[WorldState] = deque()
        self._sampling = Acquisition(c.scan_hz)
        self._publication_rate = PublicationRate(self, "lidar", c.scan_hz)
        self._epoch = -1

    def open_sensor(self, scene: UsdModel) -> None:
        self.raycaster = self.config.raycaster()
        self.raycaster.load(scene, self.config.sensor.excluded_bodies)

    def measure(self, sample: WorldState) -> tuple[PointCloud2, PointCloud2] | None:
        c = self.config.sensor
        # Check the clock even when a scan is not due.
        due = self._sampling.due(sample.header)
        if sample.header.epoch < self._epoch:
            return None
        if sample.header.epoch != self._epoch:
            self._history.clear()
            self._epoch = sample.header.epoch
        if self._history and sample.header.time_ns <= self._history[-1].header.time_ns:
            return None
        self._history.append(sample)
        cutoff = sample.header.time_ns - round(2e9 / c.scan_hz)
        while len(self._history) > 2 and self._history[1].header.time_ns < cutoff:
            self._history.popleft()
        if len(self._history) > 4096:
            raise ValueError("lidar history exceeds the bounded input budget")
        if not due:
            return None
        duration = round(1e9 / c.scan_hz)
        start = sample.header.time_ns - duration
        if start < self._history[0].header.time_ns:
            self.count("gaps")
            return None
        directions, offsets, lines = self._pattern.rays(start, duration)
        # Piecewise-held geometry at a bounded physical sample interval.
        # Keep each point's actual acquisition time, never scan-publication time.
        interval = round(1e9 / c.motion_hz)
        buckets = offsets // interval
        raw_points, world_points, times, channels = [], [], [], []
        for bucket in np.unique(buckets):
            indices = np.flatnonzero(buckets == bucket)
            stamp = start + int(offsets[indices[0]])
            state = next((s for s in reversed(self._history) if s.header.time_ns <= stamp), None)
            if state is None or stamp - state.header.time_ns > c.max_pose_gap_ns:
                self.count("gaps")
                return None
            self.raycaster.set_poses(state)
            pose = pose_array(poses_by_id(state)[c.frame_id])
            rotation = Rotation.from_quat(pose[3:]).as_matrix()
            world_rays = directions[indices] @ rotation.T
            origins = np.broadcast_to(pose[:3], world_rays.shape).copy()
            distance, normals = self.raycaster.cast(origins, world_rays, c.frame_id)
            rng = np.random.default_rng([c.seed, sample.header.epoch, stamp])
            hit = distance >= 0
            if self._response is not None:
                distance = self._response.measure(
                    distance, np.einsum("ij,ij->i", world_rays, normals), rng
                )
                hit &= distance >= 0
            distance = np.where(hit, distance + rng.normal(0, c.range_std, len(distance)), -1)
            keep = (distance >= c.min_range) & (distance <= c.max_range)
            raw_points.append(directions[indices[keep]] * distance[keep, None])
            world_points.append(origins[keep] + world_rays[keep] * distance[keep, None])
            times.append(offsets[indices[keep]])
            channels.append(lines[indices[keep]])
        # Each raw point is in the sensor axes at its own firing time. No
        # simulator pose is used to deskew these estimator inputs.
        offsets_out = np.concatenate(times).astype(np.uint32)
        raw = PointCloud2.from_numpy(
            np.concatenate(raw_points).astype(np.float32),
            frame_id=c.frame_id,
            timestamp=start / 1e9,
            offset_times=offsets_out,
            lines=np.concatenate(channels),
            tags=np.zeros(len(offsets_out), dtype=np.uint8),
        )
        world = PointCloud2.from_numpy(
            np.concatenate(world_points).astype(np.float32),
            frame_id="world",
            timestamp=start / 1e9,
            offset_times=offsets_out,
        )
        return raw, world

    def acquire(self, sample: WorldState, *, latest: bool = True) -> None:
        self._publication_rate.begin()
        clouds = self.measure(sample)
        if clouds is not None:
            raw, world = clouds
            self.lidar.publish(raw)
            self._publication_rate.published()
            if self.config.sensor.publish_truth:
                self.pointcloud.publish(world)
            self.count("acquisitions")

    def close_sensor(self) -> None:
        self.raycaster.close()
