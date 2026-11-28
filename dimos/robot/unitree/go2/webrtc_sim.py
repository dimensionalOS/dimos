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

"""A simulated Go2 as GO2Connection presents one over WebRTC: cmd_vel in; odom, its L1's voxel map and its front camera out, from one view of the world."""

import asyncio
from collections.abc import Coroutine
from functools import cached_property
import math
from pathlib import Path
import time
from typing import Any

from dimos_lcm.world_msgs import WorldAction
import numpy as np
from numpy.typing import NDArray
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.frames import usd_frame_pose
from dimos.robot.unitree.go2.connection import _camera_info_static
from dimos.robot.unitree.go2.constants import CMD_VEL_TIMEOUT
from dimos.robot.unitree.go2.go2_mid360_static_transforms import (
    CAMERA_XYZ,
    L1_MOUNT,
    L1_XYZ,
    OPTICAL_RPY,
)
from dimos.sim2.actions import JointCommand, world_action
from dimos.sim2.l1 import L1Voxels, SimL1Config
from dimos.sim2.mujoco.drives import usd_drives
from dimos.sim2.mujoco.renders import RenderQuality
from dimos.sim2.mujoco.sensor_mount import SensorMount
from dimos.sim2.mujoco.sensor_world import WorldView, rotation_matrix
from dimos.sim2.sensor import SimSensorConfig
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.view_sensor import ViewSensor
from dimos.simulation.sensors.lidar import Raycaster
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Its body: the prim the real Go2's base_link is, which its x, y and yaw servos move off its mount.
BODY = "go2/Geometry/base_link"
# The servos that move its body, in the order cmd_vel's (vx, vy, wz) drives them.
SERVOS = ("x", "y", "yaw")


def go2_usd() -> UsdModel:
    """The kinematic Go2, with the frames the real one has on tf: its front camera (and its optical frame) and its L1."""
    return UsdModel.from_file(Path(__file__).parent / "assets" / "go2_kinematic.usda").with_frames(
        {
            "camera_link": (BODY, CAMERA_XYZ, (0.0, 0.0, 0.0)),
            "camera_optical": (f"{BODY}/camera_link", (0.0, 0.0, 0.0), OPTICAL_RPY),
            "l1": (
                BODY,
                L1_XYZ,
                (math.radians(L1_MOUNT[0]), math.radians(L1_MOUNT[1]), math.radians(L1_MOUNT[2])),
            ),
        }
    )


class Go2WebRtcSimConfig(SimSensorConfig):
    usd: UsdModel = Field(default_factory=go2_usd)
    # Its frames by the names the real Go2 gives them on tf.
    tf_frames_local: dict[str, str] = {
        "base_link": BODY,
        "camera_link": f"{BODY}/camera_link",
        "camera_optical": f"{BODY}/camera_link/camera_optical",
        "l1": f"{BODY}/l1",
    }
    # The frame its odometry (and so its L1's map) is in: the mount it started on holds it, and it holds base_link.
    odom_frame: str = "odom"
    # How often it reads cmd_vel into its servos and sends its odometry.
    control_rate_hz: float = 50.0
    # Its camera: the real one's lens (front_camera_720.yaml) at ``scale`` of its 1280x720.
    camera_rate_hz: float = 60.0
    camera_scale: float = 0.5
    # Its frames are drawn on this many views at once (each its own render process on macOS), so one draws while another is tone mapped.
    camera_views: int = 4
    render: RenderQuality = RenderQuality()
    l1: SimL1Config = Field(default_factory=lambda: SimL1Config(stamp_tolerance=None))


class Go2WebRtcSim(ViewSensor):
    """A Go2 the world moves: cmd_vel sets the velocity its body slides and turns at (until it times out), and walls stop it."""

    config: Go2WebRtcSimConfig
    cmd_vel: In[Twist]
    world_actions: Out[WorldAction]
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._desired = np.zeros(3)
        self._desired_expires = 0.0

    @property
    def attach_frame(self) -> str:
        """Its odometry frame: the mount it started on holds odom, and odom holds its body."""
        return self.odom_frame_id

    @property
    def odom_frame_id(self) -> str:
        return (
            f"{self.namespace}/{self.config.odom_frame}"
            if self.namespace
            else self.config.odom_frame
        )

    @cached_property
    def _static_edges(self) -> list[Transform]:
        """The edges below its body, and the mount: odometry holds the body, so the frames its servos move go out on no edge."""
        below = f"{self.tf_frames[BODY]}/"
        mount = (self.namespace, self.attach_frame)
        return [
            t
            for t in super()._static_edges
            if t.child_frame_id.startswith(below)
            or t.frame_id == self.tf_frames[BODY]
            or (t.frame_id, t.child_frame_id) == mount
        ]

    @cached_property
    def tf_edge_ids(self) -> frozenset[tuple[str, str]]:
        return frozenset((t.frame_id, t.child_frame_id) for t in self._static_edges)

    @rpc
    def simulated_robot(self) -> SimulatedRobot:
        return SimulatedRobot(
            self.namespace, self.config.usd, odom_frame=self.odom_frame_id, free=False
        )

    def loops(self) -> list[Coroutine[Any, Any, None]]:
        return [self._drive(), self._sense()]

    async def handle_cmd_vel(self, twist: Twist) -> None:
        desired = np.array([twist.linear.x, twist.linear.y, twist.angular.z])
        if not np.isfinite(desired).all():
            logger.warning(f"go2 sim: ignoring a non-finite cmd_vel (vx, vy, wz) = {desired}")
            return
        self._desired = desired
        self._desired_expires = time.time() + CMD_VEL_TIMEOUT

    @cached_property
    def _gains(self) -> tuple[float, ...]:
        """Each servo's damping: a velocity servo, so all its gain is on velocity."""
        drives = usd_drives(self.config.usd.usd, self.config.usd.asset_root)
        return tuple(drives[name][1] for name in SERVOS)

    @cached_property
    def _standing(self) -> NDArray[np.float64]:
        """Its body in its mount's frame at its servos' zero: where its odometry starts."""
        return usd_frame_pose(self.config.usd.usd, BODY, self.config.usd.asset_root)

    async def _drive(self) -> None:
        robot = self.namespace
        async for _ in self._world.ticks(self.config.control_rate_hz):
            stamp = time.time()
            x, y, yaw = self._servo_positions(robot)
            vx, vy, wz = self._desired if stamp < self._desired_expires else np.zeros(3)
            # cmd_vel is in its body's frame; the x and y servos slide along its mount's.
            along = (
                vx * math.cos(yaw) - vy * math.sin(yaw),
                vx * math.sin(yaw) + vy * math.cos(yaw),
                wz,
            )
            command = JointCommand(
                robot, SERVOS, velocity=along, kp=(0.0,) * len(SERVOS), kd=self._gains
            )
            self.world_actions.publish(world_action(self._world, command))
            self.odom.publish(self._odometry(x, y, yaw, self._world.time))

    def _servo_positions(self, robot: str) -> tuple[float, float, float]:
        with self._world.lock:
            names, rows = self._world.joints(robot)
        positions = {name: float(row[0]) for name, row in zip(names, rows, strict=True)}
        return positions.get("x", 0.0), positions.get("y", 0.0), positions.get("yaw", 0.0)

    def _odometry(self, x: float, y: float, yaw: float, stamp: float) -> PoseStamped:
        return PoseStamped(
            ts=stamp,
            frame_id=self.odom_frame_id,
            position=Vector3(x, y, float(self._standing[2, 3])),
            orientation=Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)),
        )

    def _site(self, frame: str) -> str:
        """A frame of its USD as the world names its prim: its namespace and its path, whatever tf calls it."""
        return f"{self.namespace}/{frame}" if self.namespace else frame

    async def _sense(self) -> None:
        config = self.config
        camera, l1 = self._site(f"{BODY}/camera_link/camera_optical"), self._site(f"{BODY}/l1")
        info = self._camera_info()
        fovy = math.degrees(2 * math.atan(info.height / 2 / info.K[4]))
        mount = SensorMount("camera", camera, "go2_front_camera", fovy)
        # The world it views is composed once the sim runs, which its first tick says.
        async for _ in self._world.ticks(config.camera_rate_hz):
            break
        view = await asyncio.to_thread(self.open_view, mount, config.render)
        raycaster = view.raycaster(
            config.l1.geom_groups, mounted_at=l1, blind_to_robots=(self.namespace,)
        )
        # Its odometry starts on its mount, which holds its anchor.
        anchor = _matrix(*view.pose(self._site(self.config.usd.root_link)))
        from_world = np.asarray(np.linalg.inv(anchor), dtype=np.float64)
        # Each sensor on its own clock, and the camera on views of its own, so a frame being drawn never holds up the L1.
        drawers = [view.twin() for _ in range(config.camera_views)]
        try:
            filming = self.film(
                drawers,
                "go2_front_camera",
                info.width,
                info.height,
                config.camera_rate_hz,
                config.stamp_tolerance,
                lambda rgb, stamp: self._publish_frame(info, rgb, stamp),
            )
            await asyncio.gather(filming, self._map(view, raycaster, from_world))
        finally:
            for drawer in drawers:
                drawer.close()

    async def _map(
        self, view: WorldView, raycaster: Raycaster, from_world: NDArray[np.float64]
    ) -> None:
        """Its L1's voxel map, a scan each period."""
        config = self.config
        voxels = L1Voxels(config.l1)
        async for _ in self._world.ticks(1 / config.l1.period):
            # Read off the event loop, so a scan never holds up cmd_vel and odometry.
            points, stamp = await asyncio.to_thread(self._scan, view, raycaster, voxels, from_world)
            if self.on_time("lidar", stamp, config.l1.period, config.l1.stamp_tolerance):
                self.lidar.publish(PointCloud2.from_numpy(points, self.odom_frame_id, stamp))

    def _publish_frame(self, info: CameraInfo, rgb: NDArray[np.uint8], stamp: float) -> None:
        self.color_image.publish(
            Image(data=rgb, format=ImageFormat.RGB, frame_id=info.frame_id, ts=stamp)
        )
        self.camera_info.publish(info.with_ts(stamp))

    def _scan(
        self,
        view: WorldView,
        raycaster: Raycaster,
        voxels: L1Voxels,
        from_world: NDArray[np.float64],
    ) -> tuple[NDArray[np.float64], float]:
        """Its L1's map off the world as it is now, and when that was."""
        view.update(self._world)
        origin, orientation = view.pose(self._site(f"{BODY}/l1"))
        body, _ = view.pose(self._site(BODY))
        return voxels.scan(raycaster, origin, orientation, from_world, body), view.time

    def _camera_info(self) -> CameraInfo:
        """The real front camera's lens, scaled to the images this one renders: a pinhole, so no distortion."""
        real, scale = _camera_info_static(), self.config.camera_scale
        k = [value * scale for value in real.K[:6]] + list(real.K[6:])
        return CameraInfo(
            height=round(real.height * scale),
            width=round(real.width * scale),
            distortion_model="plumb_bob",
            D=[0.0] * 5,
            K=k,
            R=list(real.R),
            P=[k[0], 0.0, k[2], 0.0, 0.0, k[4], k[5], 0.0, 0.0, 0.0, 1.0, 0.0],
            frame_id=self.tf_frames[f"{BODY}/camera_link/camera_optical"],
        )


def _matrix(position: NDArray[np.float64], orientation: NDArray[np.float64]) -> NDArray[np.float64]:
    pose = np.eye(4)
    pose[:3, :3] = rotation_matrix(orientation)
    pose[:3, 3] = position
    return pose
