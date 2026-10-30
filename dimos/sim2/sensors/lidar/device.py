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

"""Independent instantaneous or rolling measurement acquisition, without a Module."""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from functools import partial
import math
import threading
import time
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.sensors.device import SensorDevice
from dimos.sim2.sensors.lidar.raycast import Raycaster
from dimos.sim2.sensors.lidar.spec import (
    Lidar,
    RangeResponse,
    RayPattern,
    TimedRayPattern,
    TimedRays,
)

_RAY_WORKERS = 4


@dataclass
class _RayWorker:
    data: mujoco.MjData
    scratch: mujoco.MjData


@dataclass(frozen=True)
class _RayResult:
    indices: NDArray[np.int64]
    ranges: NDArray[np.float64]
    directions: NDArray[np.float64]
    points: NDArray[np.float64]
    cos_incidence: NDArray[np.float64]


class LidarDevice(SensorDevice[Lidar]):
    def open(self) -> None:
        # Initialize PointCloud2's lazy Open3D dependency before high-rate acquisition.
        PointCloud2()
        prefix = self.model_prefix
        self._site = self.reader.model.site(prefix + self.sensor.model_name).id
        self._frame = self.sensor.frame_id or self.frame_prefix + self.sensor.name
        self._model = self.sensor.model(**self.sensor.model_kwargs)
        self._rays = self._model.directions() if isinstance(self._model, RayPattern) else None
        if isinstance(self._model, TimedRayPattern):
            # Load the pattern before acquisition, not halfway through a scan.
            self._model.scan(0.0, 1 / self.sensor.rate_hz)
        self._episode = -1
        self._last_scan = 0
        self._clock_origin = 0.0
        self._scan_status: dict[str, Any] = {
            "fidelity": "rolling" if isinstance(self._model, TimedRayPattern) else "instantaneous",
            "scans": 0,
            "dropped_scans": 0,
            "history_pending": False,
        }
        self._raycaster = Raycaster(
            self.reader.model,
            self.reader.model.body(prefix + self.root_body).id
            if self.root_body is not None
            else None,
            self_occlusion=self.sensor.self_occlusion,
            excluded_meshes=tuple(prefix + mesh for mesh in self.sensor.excluded_meshes),
        )
        self._worker_state = threading.local()
        self._ray_pool = (
            ThreadPoolExecutor(
                max_workers=_RAY_WORKERS,
                thread_name_prefix="sim2-lidar-ray",
                initializer=self._open_ray_worker,
            )
            if isinstance(self._model, TimedRayPattern)
            else None
        )

    def _open_ray_worker(self) -> None:
        self._worker_state.value = _RayWorker(
            mujoco.MjData(self.reader.model), mujoco.MjData(self.reader.model)
        )

    def close(self) -> None:
        if self._ray_pool is not None:
            self._ray_pool.shutdown(wait=True, cancel_futures=True)
            self._ray_pool = None
        super().close()

    def reset(self) -> None:
        self._clock_origin = self.reader.timestamp - float(self.reader.data.time)
        self._last_scan = 0

    def capture(self) -> None:
        self._clock_origin = self.reader.timestamp - float(self.reader.data.time)
        if isinstance(self._model, TimedRayPattern):
            self._capture_rolling(self._model)
            return
        if self.sample_time(self.sensor.rate_hz) is None:
            return
        assert self._rays is not None
        data = self.reader.data
        origin = data.site_xpos[self._site]
        rotation = data.site_xmat[self._site].reshape(3, 3)
        rays = self._rays @ rotation.T
        elevation_limit = self.sensor.maximum_world_elevation
        if elevation_limit is not None:
            rays = rays[rays[:, 2] <= np.sin(np.deg2rad(elevation_limit)) + 1e-12]
        points = self._raycaster.cast(
            data,
            origin,
            rays,
            self._model.min_range,
            self._model.max_range,
        )
        self._publish_cloud(points, origin, rotation)

    def _publish_cloud(
        self,
        points: NDArray[np.float64],
        origin: NDArray[np.float64],
        rotation: NDArray[np.float64],
    ) -> None:
        if not self.sensor.truth_outputs:
            return
        frame = "world"
        if self.sensor.output_frame == "sensor":
            points = (points - origin) @ rotation
            frame = self._frame
        if frame == self._frame or isinstance(self._model, TimedRayPattern):
            self.emit(
                "tf",
                TFMessage(
                    Transform(
                        translation=Vector3(*origin),
                        rotation=Quaternion(*Rotation.from_matrix(rotation).as_quat()),
                        frame_id="world",
                        child_frame_id=self._frame,
                        ts=self.reader.timestamp,
                    )
                ),
            )
        self.emit(
            "pointcloud",
            PointCloud2.from_numpy(
                points.astype(np.float32),
                frame_id=frame,
                timestamp=self.reader.timestamp,
            ),
        )

    def _capture_rolling(self, pattern: TimedRayPattern) -> None:
        rate = self.sensor.rate_hz
        index = math.floor((float(self.reader.data.time) + 1e-9) * rate)
        if index <= self._last_scan:
            return
        end, duration = index / rate, 1 / rate
        start = end - duration
        if not self.reader.history(start, end):
            with self._error_lock:
                self._scan_status["history_pending"] = True
            return
        before = time.monotonic()
        rays = pattern.scan(start, duration)
        bins = np.floor(rays.offsets * pattern.motion_sample_rate_hz).astype(np.int64)
        distances = np.full(len(rays.offsets), -1.0)
        world_points = np.empty_like(rays.directions)
        world_rays = np.empty_like(rays.directions)
        cos_incidence = np.zeros(len(rays.offsets))
        mapping_visible = np.ones(len(rays.offsets), dtype=bool)
        elevation = self.sensor.maximum_world_elevation
        assert self._ray_pool is not None
        # The interval remains pinned until every bin completes. Workers only
        # read that history/model and each owns its MuJoCo stack and transforms.
        cast = partial(self._cast_motion_bin, rays=rays, start=start, max_range=pattern.max_range)
        groups = [np.flatnonzero(bins == motion_bin) for motion_bin in np.unique(bins)]
        for result in self._ray_pool.map(cast, groups):
            indices = result.indices
            cos_incidence[indices] = result.cos_incidence
            distances[indices] = result.ranges
            world_rays[indices] = result.directions
            world_points[indices] = result.points
            if elevation is not None:
                mapping_visible[indices] = (
                    result.directions[:, 2] <= np.sin(np.deg2rad(elevation)) + 1e-12
                )
        if isinstance(pattern, RangeResponse):
            measured = pattern.measure(distances, cos_incidence, start)
            world_points += world_rays * (measured - distances)[:, None]
            distances = measured
        self.reader.restore_at(end)
        self.reader.timestamp = self._clock_origin + end
        if self.reader.latest_episode != self._episode:
            return
        hit = (distances >= pattern.min_range) & (distances <= pattern.max_range)
        raw = rays.directions[hit] * distances[hit, None]
        self.emit(
            "lidar",
            PointCloud2.from_numpy(
                raw.astype(np.float32),
                frame_id=self._frame,
                timestamp=self.reader.timestamp - duration,
                offset_times=np.rint(rays.offsets[hit] * 1e9).astype(np.uint32),
                lines=rays.lines[hit],
            ),
            start=start,
            end=end,
        )
        self._publish_cloud(
            world_points[hit & mapping_visible],
            self.reader.data.site_xpos[self._site],
            self.reader.data.site_xmat[self._site].reshape(3, 3),
        )
        with self._error_lock:
            self._scan_status.update(
                scans=self._scan_status["scans"] + 1,
                dropped_scans=self._scan_status["dropped_scans"]
                + (max(0, index - self._last_scan - 1) if self._last_scan else 0),
                history_pending=False,
                rays=len(rays.offsets),
                returns=int(hit.sum()),
                capture_seconds=time.monotonic() - before,
                scan_start=start,
                scan_end=end,
                episode=self._episode,
                # Privileged diagnostic only; never connected to estimator inputs.
                truth_pose={
                    "timestamp": self.reader.timestamp,
                    "position": self.reader.data.site_xpos[self._site].tolist(),
                    "orientation": Rotation.from_matrix(
                        self.reader.data.site_xmat[self._site].reshape(3, 3)
                    )
                    .as_quat()
                    .tolist(),
                },
            )
        self._last_scan = index

    def _cast_motion_bin(
        self,
        indices: NDArray[np.int64],
        *,
        rays: TimedRays,
        start: float,
        max_range: float,
    ) -> _RayResult:
        worker: _RayWorker = self._worker_state.value
        self.reader.restore_into(
            start + float(np.mean(rays.offsets[indices])),
            worker.data,
            worker.scratch,
            kinematics_only=True,
        )
        origin = worker.data.site_xpos[self._site]
        rotation = worker.data.site_xmat[self._site].reshape(3, 3)
        directions = rays.directions[indices] @ rotation.T
        ranges, normals = self._raycaster.hits(worker.data, origin, directions, max_range)
        return _RayResult(
            indices,
            ranges,
            directions,
            origin + directions * ranges[:, None],
            np.einsum("ij,ij->i", directions, normals),
        )

    def status(self) -> dict[str, Any]:
        with self._error_lock:
            return {**super().status(), **self._scan_status}
