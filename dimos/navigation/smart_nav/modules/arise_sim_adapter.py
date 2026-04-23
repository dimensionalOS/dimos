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

"""AriseSimAdapter: adapts Unity sim data for AriseSLAM input.

AriseSLAM expects body-frame lidar (raw_points) and IMU data.
Unity provides world-frame registered_scan and ground-truth odometry.
This adapter:
  1. Transforms registered_scan from world-frame → body-frame using odom
  2. Synthesizes IMU (orientation + angular velocity + gravity) from odom

This lets AriseSLAM run in simulation without real hardware.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class AriseSimAdapterConfig(ModuleConfig):
    gravity: float = 9.80511
    imu_rate: float = 200.0  # Hz — AriseSLAM expects high-rate IMU


class AriseSimAdapter(Module):
    """Adapts sim data (world-frame scans + odom) → AriseSLAM inputs (body-frame + IMU).

    NOTE: using this is basically doing "1+1-1", its useful for sim or robots that do not provide raw-scans
          but beyond those two edgecases THIS MODULE SHOULD NOT BE USED
    Ports:
        registered_scan (In[PointCloud2]): World-frame scan from simulator.
        odometry (In[Odometry]): Ground-truth odom from simulator.
        raw_points (Out[PointCloud2]): Body-frame scan for AriseSLAM.
        imu (Out[Imu]): Synthetic IMU for AriseSLAM.
    """

    config: AriseSimAdapterConfig
    default_config = AriseSimAdapterConfig

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    raw_points: Out[PointCloud2]
    imu: Out[Imu]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._latest_odom: Odometry | None = None

    def __getstate__(self) -> dict[str, Any]:
        state = super().__getstate__()
        state.pop("_lock", None)
        state.pop("_thread", None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        super().__setstate__(state)
        self._lock = threading.Lock()
        self._thread = None

    def start(self) -> None:
        self.odometry._transport.subscribe(self._on_odom)
        self.registered_scan._transport.subscribe(self._on_scan)
        self._running = True
        self._thread = threading.Thread(target=self._imu_loop, daemon=True)
        self._thread.start()
        print("[AriseSimAdapter] Started — converting sim data for AriseSLAM")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        super().stop()

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._latest_odom = msg

    def _on_scan(self, cloud: PointCloud2) -> None:
        """Transform world-frame scan → body-frame using latest odom."""
        with self._lock:
            odom = self._latest_odom
        if odom is None:
            return

        try:
            tf_map_to_sensor = Transform(
                translation=Vector3(odom.x, odom.y, odom.z),
                rotation=odom.orientation,
                frame_id="map",
                child_frame_id="sensor",
            )
            tf_sensor_to_map = tf_map_to_sensor.inverse()
            body_cloud = cloud.transform(tf_sensor_to_map)
            body_cloud.frame_id = "sensor"
            self.raw_points._transport.publish(body_cloud)
        except Exception:
            import traceback

            print(f"[AriseSimAdapter] scan transform failed: {traceback.format_exc()}")

    def _imu_loop(self) -> None:
        """Publish synthetic IMU at high rate from latest odom."""
        dt = 1.0 / self.config.imu_rate
        g = self.config.gravity

        while self._running:
            t0 = time.monotonic()

            with self._lock:
                odom = self._latest_odom

            if odom is not None:
                q = odom.pose.orientation
                ang_vel = Vector3(0.0, 0.0, 0.0)
                if odom.twist is not None:
                    ang_vel = Vector3(
                        odom.twist.angular.x,
                        odom.twist.angular.y,
                        odom.twist.angular.z,
                    )

                # Rotate gravity [0, 0, g] into body frame
                gx, gy, gz = _rotate_vec_by_quat_inv(0.0, 0.0, g, q.x, q.y, q.z, q.w)

                self.imu._transport.publish(
                    Imu(
                        angular_velocity=ang_vel,
                        linear_acceleration=Vector3(gx, gy, gz),
                        orientation=Quaternion(q.x, q.y, q.z, q.w),
                        ts=time.time(),
                        frame_id="sensor",
                    )
                )

            elapsed = time.monotonic() - t0
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)


def _rotate_vec_by_quat_inv(
    vx: float,
    vy: float,
    vz: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> tuple[float, float, float]:
    """Rotate vector by the inverse of a unit quaternion."""
    nqx, nqy, nqz = -qx, -qy, -qz
    tx = 2.0 * (nqy * vz - nqz * vy)
    ty = 2.0 * (nqz * vx - nqx * vz)
    tz = 2.0 * (nqx * vy - nqy * vx)
    return (
        vx + qw * tx + (nqy * tz - nqz * ty),
        vy + qw * ty + (nqz * tx - nqx * tz),
        vz + qw * tz + (nqx * ty - nqy * tx),
    )
