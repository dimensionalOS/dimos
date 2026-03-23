"""AriseSimAdapter: synthesizes IMU data from sim odometry for AriseSLAM.

In simulation, the Unity bridge provides ground-truth odometry but no IMU.
AriseSLAM needs IMU for motion prediction. This adapter derives synthetic
IMU (orientation + angular velocity + gravity-aligned acceleration) from
consecutive odometry messages.

Use with SensorScanGeneration which provides body-frame scans from
world-frame registered_scan. Wire via remappings:

    (SensorScanGeneration, "sensor_scan", "raw_points")   # → AriseSLAM
    (AriseSimAdapter, "imu", "imu")                       # → AriseSLAM (autoconnect)
"""

from __future__ import annotations

import threading
import time

import numpy as np

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu


class AriseSimAdapterConfig(ModuleConfig):
    gravity: float = 9.80511
    publish_rate: float = 200.0  # Hz — AriseSLAM expects high-rate IMU


class AriseSimAdapter(Module[AriseSimAdapterConfig]):
    """Synthesizes IMU from odometry for testing AriseSLAM in simulation.

    Ports:
        odometry (In[Odometry]): Ground-truth odom from simulator.
        imu (Out[Imu]): Synthetic IMU (orientation + angular vel + gravity).
    """

    default_config = AriseSimAdapterConfig

    odometry: In[Odometry]
    imu: Out[Imu]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._latest_odom: Odometry | None = None
        self._prev_odom: Odometry | None = None

    def __getstate__(self) -> dict:
        state = super().__getstate__()
        state.pop("_lock", None)
        state.pop("_thread", None)
        return state

    def __setstate__(self, state: dict) -> None:
        super().__setstate__(state)
        self._lock = threading.Lock()
        self._thread = None

    def start(self) -> None:
        self.odometry._transport.subscribe(self._on_odom)
        self._running = True
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        print("[AriseSimAdapter] Started — synthesizing IMU from odom")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        super().stop()

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._prev_odom = self._latest_odom
            self._latest_odom = msg

    def _publish_loop(self) -> None:
        dt = 1.0 / self.config.publish_rate
        g = self.config.gravity

        while self._running:
            t0 = time.monotonic()

            with self._lock:
                odom = self._latest_odom
                prev = self._prev_odom

            if odom is not None:
                # Orientation directly from odom
                orientation = Quaternion(
                    odom.pose.orientation.x,
                    odom.pose.orientation.y,
                    odom.pose.orientation.z,
                    odom.pose.orientation.w,
                )

                # Angular velocity from odom twist (if available)
                ang_vel = Vector3(0.0, 0.0, 0.0)
                if odom.twist is not None:
                    ang_vel = Vector3(
                        odom.twist.angular.x,
                        odom.twist.angular.y,
                        odom.twist.angular.z,
                    )

                # Linear acceleration: gravity in body frame + odom acceleration
                # Rotate gravity [0, 0, g] into body frame using inverse of orientation
                q = orientation
                # Quaternion rotation of gravity vector into body frame
                # Using q^{-1} * [0,0,g] * q
                gx, gy, gz = _rotate_vec_by_quat_inv(0.0, 0.0, g, q.x, q.y, q.z, q.w)
                lin_accel = Vector3(gx, gy, gz)

                now = time.time()
                imu_msg = Imu(
                    angular_velocity=ang_vel,
                    linear_acceleration=lin_accel,
                    orientation=orientation,
                    ts=now,
                    frame_id="sensor",
                )
                self.imu._transport.publish(imu_msg)

            elapsed = time.monotonic() - t0
            sleep_time = max(0.0, dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)


def _rotate_vec_by_quat_inv(
    vx: float, vy: float, vz: float,
    qx: float, qy: float, qz: float, qw: float,
) -> tuple[float, float, float]:
    """Rotate vector [vx,vy,vz] by the inverse of quaternion [qx,qy,qz,qw]."""
    # q_inv = [-qx, -qy, -qz, qw] for unit quaternion
    # result = q_inv * v * q
    # Using the formula: v' = v + 2*w*(w x v) + 2*(q x (q x v))
    # where q = [-qx,-qy,-qz], w = qw
    nqx, nqy, nqz = -qx, -qy, -qz
    # t = 2 * cross(q, v)
    tx = 2.0 * (nqy * vz - nqz * vy)
    ty = 2.0 * (nqz * vx - nqx * vz)
    tz = 2.0 * (nqx * vy - nqy * vx)
    # result = v + qw*t + cross(q, t)
    rx = vx + qw * tx + (nqy * tz - nqz * ty)
    ry = vy + qw * ty + (nqz * tx - nqx * tz)
    rz = vz + qw * tz + (nqx * ty - nqy * tx)
    return rx, ry, rz
