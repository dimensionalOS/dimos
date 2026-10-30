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

"""Typed stream/lifecycle wrapper for lidar measurement acquisition."""

from collections import deque
import math
import threading
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.messages import FrameMetadata, StateFrame
from dimos.sim2.sensors.imu.messages import imu_message
from dimos.sim2.sensors.lidar.device import LidarDevice
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.sensors.module import SensorModule, SensorModuleConfig
from dimos.sim2.sensors.observations import ObservationFrame


class LidarModuleConfig(SensorModuleConfig):
    sensor: Lidar
    root_body: str | None = None


class LidarModule(SensorModule):
    """A lidar worker with a real driver's ports.

    ``lidar`` is the sensor-frame scan with per-point offsets and channels, and
    ``imu`` the device's own IMU, exactly what a Mid360 driver publishes.
    ``pointcloud`` is the deskewed world-frame cloud and ``tf`` the true sensor
    pose, both ground truth a real device cannot give; ``Lidar.truth_outputs``
    turns them off when an estimator is under test.
    """

    config: LidarModuleConfig
    observations: In[ObservationFrame]
    lidar: Out[PointCloud2]
    imu: Out[Imu]
    pointcloud: Out[PointCloud2]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._imu_lock = threading.Lock()
        self._imu_identity = (-1, -1)
        self._imu_timestamp: float | None = None
        self._imu_samples = 0
        self._dropped_imu_samples = 0
        self._imu_sim_time = -math.inf
        self._pending_imu: deque[tuple[float, ObservationFrame]] = deque()

    @rpc
    def start(self) -> None:
        # IMU observations are acquired by physics and delivered independently
        # of the expensive rolling ray query on the sensor worker thread.
        self.register_disposable(Disposable(self.observations.subscribe(self._on_observation)))
        if self.config.sensor.imu is not None and self.config.sensor.imu.latency > 0:
            # A slow IMU still releases delayed samples at snapshot cadence,
            # without waiting for its next acquisition or the ray worker.
            self.register_disposable(Disposable(self.snapshots.subscribe(self._on_snapshot)))
        super().start()

    def _advance_imu_clock(self, metadata: FrameMetadata) -> bool:
        if metadata.episode_id < self._imu_identity[0]:
            return False
        if metadata.episode_id != self._imu_identity[0]:
            self._imu_identity = (metadata.episode_id, -1)
            self._imu_timestamp = None
            self._imu_sim_time = -math.inf
            self._pending_imu.clear()
        self._imu_sim_time = max(self._imu_sim_time, metadata.sim_time)
        return True

    def _on_snapshot(self, frame: StateFrame) -> None:
        with self._imu_lock:
            if self._advance_imu_clock(frame.metadata):
                self._publish_ready_imu()

    def _on_observation(self, frame: ObservationFrame) -> None:
        imu = self.config.sensor.imu
        if imu is None or frame.robot_id != self.config.robot_id or frame.sensor_name != imu.name:
            return
        identity = frame.metadata.episode_id, frame.metadata.sequence
        with self._imu_lock:
            if not self._advance_imu_clock(frame.metadata) or identity <= self._imu_identity:
                return
            self._imu_identity = identity
            acquired_at = float(frame.values["metadata"][0])
            self._pending_imu.append((acquired_at + imu.latency, frame))
            self._publish_ready_imu()

    def _publish_ready_imu(self) -> None:
        imu = self.config.sensor.imu
        assert imu is not None
        while self._pending_imu and self._pending_imu[0][0] <= self._imu_sim_time + 1e-9:
            _, frame = self._pending_imu.popleft()
            if self._imu_timestamp is not None:
                self._dropped_imu_samples += max(
                    0, round((frame.timestamp - self._imu_timestamp) * imu.rate_hz) - 1
                )
            self._imu_timestamp = frame.timestamp
            message = imu_message(frame)
            if self.config.sensor.imu_frame_id is not None:
                message.frame_id = self.config.sensor.imu_frame_id
            self.imu.publish(message)
            self._imu_samples += 1

    @rpc
    def sensor_status(self) -> dict[str, Any]:
        status = super().sensor_status()
        with self._imu_lock:
            return {
                **status,
                "imu_samples": self._imu_samples,
                "dropped_imu_samples": self._dropped_imu_samples,
                "pending_imu_samples": len(self._pending_imu),
            }

    @rpc
    def stop(self) -> None:
        super().stop()
        with self._imu_lock:
            self._pending_imu.clear()

    def open(self) -> None:
        self.device = LidarDevice(
            self.reader,
            self.config.sensor,
            self.config.robot_id,
            self.config.root_body or self.reader.root_body(self.config.robot_id),
        )
