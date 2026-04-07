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

"""SensorScanGeneration: transforms registered (world-frame) point cloud to sensor frame.

Ported from sensorScanGeneration.cpp. Takes Odometry + PointCloud2 (world-frame),
computes inverse transform, outputs sensor-frame point cloud.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class SensorScanGeneration(Module):
    """Transform registered world-frame point cloud into sensor frame.

    Ports:
        registered_scan (In[PointCloud2]): World-frame registered point cloud from SLAM.
        odometry (In[Odometry]): Vehicle state estimation.
        sensor_scan (Out[PointCloud2]): Sensor-frame point cloud.
        odometry_at_scan (Out[Odometry]): Odometry republished with scan timestamp.
    """

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    sensor_scan: Out[PointCloud2]
    odometry_at_scan: Out[Odometry]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._latest_odom: Odometry | None = None
        self._lock = threading.Lock()

    def __getstate__(self) -> dict[str, Any]:
        state = super().__getstate__()
        state.pop("_lock", None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        super().__setstate__(state)
        self._lock = threading.Lock()

    @rpc
    def start(self) -> None:
        self.odometry.subscribe(self._on_odometry)
        self.registered_scan.subscribe(self._on_scan)

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_odometry(self, odom: Odometry) -> None:
        with self._lock:
            self._latest_odom = odom

    def _on_scan(self, cloud: PointCloud2) -> None:
        with self._lock:
            odom = self._latest_odom

        if odom is None:
            return

        try:
            # Build transform from odometry (map -> sensor)
            tf_map_to_sensor = Transform(
                translation=Vector3(odom.x, odom.y, odom.z),
                rotation=odom.orientation,
                frame_id="map",
                child_frame_id="sensor",
            )

            # Inverse transform: sensor -> map (transforms world points into sensor frame)
            tf_sensor_to_map = tf_map_to_sensor.inverse()

            # Transform the point cloud into sensor frame
            sensor_cloud = cloud.transform(tf_sensor_to_map)
            sensor_cloud.frame_id = "sensor_at_scan"

            # Publish sensor-frame cloud
            self.sensor_scan.publish(sensor_cloud)

            # Republish odometry with scan timestamp
            odom_at_scan = Odometry(
                ts=cloud.ts if cloud.ts is not None else time.time(),
                frame_id="map",
                child_frame_id="sensor_at_scan",
                pose=odom.pose,
                twist=odom.twist,
            )
            self.odometry_at_scan.publish(odom_at_scan)
        except Exception:
            pass  # Skip malformed messages silently
