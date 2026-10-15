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

from threading import Event, Thread
import time
from typing import Any

from dimos_generated.geometry_msgs.msg import PoseStamped, Vector3
from dimos_generated.sensor_msgs.msg import PointCloud2

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.time import to_seconds
from dimos.robot.unitree.type.lidar import RawLidarMsg, pointcloud2_from_webrtc_lidar
from dimos.robot.unitree.type.odometry import RawOdometryMessage, pose_from_webrtc_odometry
from dimos.types.timestamped import TimestampedData
from dimos.utils.testing.replay import SensorReplay


def _replay_lidar(raw: RawLidarMsg) -> TimestampedData[PointCloud2]:
    cloud = pointcloud2_from_webrtc_lidar(raw)
    return TimestampedData(cloud, to_seconds(cloud.header.stamp))


def _replay_odometry(raw: RawOdometryMessage) -> TimestampedData[PoseStamped]:
    pose = pose_from_webrtc_odometry(raw)
    return TimestampedData(pose, to_seconds(pose.header.stamp))


class MockRobotClient(Module):
    odometry: Out[PoseStamped]
    timed_odometry: Out[TimestampedData[PoseStamped]]
    lidar: Out[PointCloud2]
    mov: In[Vector3]

    mov_msg_count = 0

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = Event()
        self._thread = None

    def mov_callback(self, msg) -> None:  # type: ignore[no-untyped-def]
        self.mov_msg_count += 1

    @rpc
    def start(self) -> None:
        super().start()

        self._thread = Thread(target=self.odomloop)  # type: ignore[assignment]
        self._thread.start()  # type: ignore[attr-defined]
        self.mov.subscribe(self.mov_callback)

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        super().stop()

    def odomloop(self) -> None:
        odomdata = SensorReplay("raw_odometry_rotate_walk", autocast=_replay_odometry)
        lidardata = SensorReplay("office_lidar", autocast=_replay_lidar)

        lidariter = lidardata.iterate()
        self._stop_event.clear()
        while not self._stop_event.is_set():
            for odom in odomdata.iterate():
                if self._stop_event.is_set():
                    return
                print(odom)
                # Benchmark publication time belongs to the Python-object stream,
                # keeping the generated pose's exact source header untouched.
                self.timed_odometry.publish(TimestampedData(odom.value, time.perf_counter()))
                self.odometry.publish(odom.value)

                lidarmsg = next(lidariter)
                self.lidar.publish(lidarmsg.value)
                time.sleep(0.1)
