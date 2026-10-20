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

"""GO2DDS: the Go2 over CycloneDDS, as a native module on the robot side.

The rust binary (``rust/``) terminates ``cmd_vel`` as sport ``Move`` and ``command`` verbs
as sport requests, and streams the robot's own odometry (``odom -> base_link``, with the
tf edge), the mount tree, the head L1 cloud, the front camera and its intrinsics.
GO2L1Lidar places the raw L1 scans into Point-LIO's odom, deskewed and filtered.
"""

from __future__ import annotations

from collections import deque
import threading
import time
from typing import TYPE_CHECKING, Literal

from dimos_lcm.control_msgs import DynamicJointState
import numpy as np
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation, Slerp

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.go2.base import Go2Base, Go2BaseConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GO2DDSConfig(NativeModuleConfig, Go2BaseConfig):
    source_dir: str | None = "dimos/robot/unitree/go2/dds/rust"
    executable: str = "result/bin/go2_dds"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    # Every field below crosses to the rust `Config` verbatim (test_module.py).
    # eth0 on the Go2 itself, the Go2 link on the Jetson.
    iface: str = "eth0"
    domain_id: int = 0
    # Spin the head L1 up at start (park it otherwise) and stream its deskewed cloud.
    lidar_on: bool = True
    # Also the undeskewed sensor-frame cloud, the L1's own IMU and its height map.
    lidar_raw_on: bool = False
    # The robot's built-in obstacle avoidance, applied at start.
    obstacle_avoidance: bool = True
    # Body joint and motor states, IMU and battery off rt/lowstate, decimated to lowstate_hz.
    lowstate_on: bool = True
    lowstate_hz: float = 50.0
    # Also whatever drives the motors on rt/lowcmd (a low-level policy), at lowstate_hz.
    lowcmd_on: bool = False
    # h264 off the RTP multicast onto `video`, or jpeg polled at `video_fps` onto `image`.
    video_on: bool = True
    video_encoding: Literal["h264", "jpeg"] = "jpeg"
    video_fps: float = 15.0
    video_group: str = "230.1.1.1"
    video_port: int = 1720
    # StopMove once cmd_vel has been silent this long.
    deadman_ms: int = 500


class GO2DDS(NativeModule, Go2Base):
    """The Go2 over DDS: sport control in; odometry, clouds, camera and body state out."""

    config: GO2DDSConfig

    image: Out[CompressedImage]  # front camera JPEG, `video_encoding="jpeg"` only
    lidar_raw: Out[PointCloud2]  # undeskewed L1 cloud in its sensor frame, `lidar_raw_on`
    lidar_imu: Out[Imu]  # the L1's own IMU, `lidar_raw_on`
    joint_state: Out[JointState]  # 12 leg joints, Unitree order
    motor_state: Out[DynamicJointState]  # the 12 motors' full readings + 4 foot forces
    joint_command: Out[DynamicJointState]  # rt/lowcmd targets and PD gains, `lowcmd_on`
    height_map: Out[PointCloud2]  # the L1's elevation grid, known cells, `lidar_raw_on`
    imu: Out[Imu]  # the body IMU, base_link
    battery: Out[BatteryState]  # ~1 Hz
    joy: Out[Joy]  # the handheld remote: 4 stick axes, 16 key bits

    @rpc
    def stop(self) -> None:
        self.liedown()
        time.sleep(0.5)  # let the verb reach the process before it is killed
        super().stop()


class GO2L1LidarConfig(ModuleConfig):
    # L1 returns past this (sensor frame) are too noisy to keep.
    max_range: float = 2.0
    # Returns within this horizontal radius of base_link are the robot's own legs and body.
    self_radius: float = 0.75
    # Scans delivered later than this after their stamp are dropped (stalls replay old scans).
    max_lag: float = 1.0
    # A knock to the head: when the 0.5 s RMS of the L1-vs-body gyro residual crosses
    # knock_rad_s, the next knock_hold seconds of scans are dropped.
    knock_rad_s: float = 0.3
    knock_window: float = 0.5
    knock_hold: float = 4.0
    # L1 imu <- body imu, fixed-axis rpy in degrees (gyro Kabsch on two recordings).
    knock_imu_rpy: tuple[float, float, float] = (-179.3, -14.2, -56.9)
    sensor_frame: str = "utlidar_lidar"
    body_frame: str = "base_link"


def _vec(v: Vector3) -> np.ndarray:
    return np.array([v.x, v.y, v.z], dtype=np.float64)


class L1Deskew:
    """Causal L1 scan placement: hold each scan until odometry covers it, then deskew."""

    def __init__(
        self, config: GO2L1LidarConfig, odom_from_l1: Transform, body_from_l1: Transform
    ) -> None:
        """odom_from_l1 is the odometry child frame <- the L1, body_from_l1 base_link <- the L1."""
        self.config = config
        self._mount = odom_from_l1.to_matrix()
        self._body = body_from_l1.to_matrix()
        self._to_l1 = Rotation.from_euler("xyz", config.knock_imu_rpy, degrees=True)
        self._poses: deque[Odometry] = deque()
        self._pending: deque[tuple[PointCloud2, float]] = deque()
        self._body_gyro: np.ndarray | None = None
        self._residual: deque[tuple[float, float]] = deque()
        self._knocked = False
        self.knocks: list[tuple[float, float]] = []
        self.dropped_late = 0
        self.dropped_knock = 0

    def on_body_imu(self, imu: Imu) -> None:
        self._body_gyro = self._to_l1.apply(_vec(imu.angular_velocity))

    def on_l1_imu(self, imu: Imu) -> None:
        if self._body_gyro is None:
            return
        res2 = float(np.sum((_vec(imu.angular_velocity) - self._body_gyro) ** 2))
        self._residual.append((imu.ts, res2))
        while imu.ts - self._residual[0][0] > self.config.knock_window:
            self._residual.popleft()
        rms = np.sqrt(np.mean([r for _, r in self._residual]))
        knocked = bool(rms >= self.config.knock_rad_s)
        if knocked and not self._knocked:
            end = imu.ts + self.config.knock_hold
            if self.knocks and imu.ts < self.knocks[-1][1]:
                self.knocks[-1] = (self.knocks[-1][0], end)
            else:
                self.knocks.append((imu.ts, end))
            logger.info(f"L1 knock at {imu.ts:.3f}, holding scans for {self.config.knock_hold} s")
        self._knocked = knocked

    def on_cloud(self, cloud: PointCloud2, arrival: float) -> list[PointCloud2]:
        if arrival - cloud.ts > self.config.max_lag:
            self.dropped_late += 1
            return []
        offsets = cloud.offset_times_u32()
        end = cloud.ts + (0.0 if offsets is None or len(offsets) == 0 else offsets.max() * 1e-9)
        self._pending.append((cloud, end))
        return self._release()

    def on_odometry(self, odom: Odometry) -> list[PointCloud2]:
        self._poses.append(odom)
        return self._release()

    def _release(self) -> list[PointCloud2]:
        out: list[PointCloud2] = []
        newest = self._poses[-1].ts if self._poses else -np.inf
        while self._pending and (
            self._pending[0][1] <= newest
            or self._pending[0][0].ts < self._pending[-1][0].ts - self.config.max_lag
        ):
            cloud, end = self._pending.popleft()
            if end > newest:
                continue  # odometry never caught up
            if any(a <= cloud.ts < b for a, b in self.knocks):
                self.dropped_knock += 1
                continue
            placed = self._place(cloud)
            if placed is not None:
                out.append(placed)
        # A scan is stamped at most max_lag before it arrives, and odometry trails arrival.
        while len(self._poses) > 2 and self._poses[1].ts < newest - 2 * self.config.max_lag:
            self._poses.popleft()
        return out

    def _place(self, cloud: PointCloud2) -> PointCloud2 | None:
        poses = list(self._poses)
        ts = np.array([p.ts for p in poses])
        pts = cloud.points_f32().astype(np.float64)
        offsets = cloud.offset_times_u32()
        tp = cloud.ts + (np.zeros(len(pts)) if offsets is None else offsets * 1e-9)
        body = pts @ self._body[:3, :3].T + self._body[:3, 3]
        keep = (np.linalg.norm(pts, axis=1) < self.config.max_range) & (
            np.linalg.norm(body[:, :2], axis=1) > self.config.self_radius
        )
        keep &= (tp >= ts[0]) & (tp <= ts[-1])
        if len(ts) < 2 or not keep.any():
            return None
        pts, tp = pts[keep] @ self._mount[:3, :3].T + self._mount[:3, 3], tp[keep]
        quats = [
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w] for p in poses
        ]
        pos = np.array([_vec(p.position) for p in poses])
        world = Slerp(ts, Rotation.from_quat(quats))(tp).apply(pts) + np.stack(
            [np.interp(tp, ts, pos[:, k]) for k in range(3)], axis=1
        )
        inten = cloud.intensities_f32()
        return PointCloud2.from_numpy(
            world.astype(np.float32),
            frame_id=poses[-1].frame_id,
            timestamp=cloud.ts,
            intensities=None if inten is None else inten[keep],
        )


class GO2L1Lidar(Module):
    """The head L1's raw scans in Point-LIO's odom: deskewed, range/self filtered, knocks held off."""

    config: GO2L1LidarConfig

    l1_lidar_raw: In[PointCloud2]  # GO2DDS `lidar_raw`, utlidar_lidar
    l1_lidar_imu: In[Imu]  # GO2DDS `lidar_imu`
    go2_imu: In[Imu]  # GO2DDS `imu`, the body IMU in base_link
    odometry: In[Odometry]  # Point-LIO, odom -> mid360_link
    tf: In[TFMessage]
    l1_lidar: Out[PointCloud2]  # the deskewed L1 cloud, in odom

    _core: L1Deskew | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._lock = threading.Lock()
        for unsubscribe in (
            self.l1_lidar_raw.subscribe(self._on_cloud),
            self.l1_lidar_imu.subscribe(self._on_l1_imu),
            self.go2_imu.subscribe(self._on_body_imu),
            self.odometry.subscribe(self._on_odometry),
        ):
            self.register_disposable(Disposable(unsubscribe))

    def _deskew(self, child_frame: str) -> L1Deskew | None:
        """Built once tf reaches the L1 from the odometry's moving frame."""
        if self._core is None:
            c = self.config
            mount = self.tfbuffer.get(child_frame, c.sensor_frame, warn=False)
            body = self.tfbuffer.get(c.body_frame, c.sensor_frame, warn=False)
            if mount is None or body is None:
                return None
            self._core = L1Deskew(c, mount, body)
        return self._core

    def _on_odometry(self, msg: Odometry) -> None:
        with self._lock:
            core = self._deskew(msg.child_frame_id)
            out = [] if core is None else core.on_odometry(msg)
        for cloud in out:
            self.l1_lidar.publish(cloud)

    def _on_cloud(self, msg: PointCloud2) -> None:
        arrival = time.time()
        with self._lock:
            out = [] if self._core is None else self._core.on_cloud(msg, arrival)
        for cloud in out:
            self.l1_lidar.publish(cloud)

    def _on_l1_imu(self, msg: Imu) -> None:
        with self._lock:
            if self._core is not None:
                self._core.on_l1_imu(msg)

    def _on_body_imu(self, msg: Imu) -> None:
        with self._lock:
            if self._core is not None:
                self._core.on_body_imu(msg)


if TYPE_CHECKING:
    GO2DDS()
    GO2L1Lidar()
