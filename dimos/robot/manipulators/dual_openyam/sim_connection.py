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

"""A Damiao robot in MuJoCo, behind the ports of its Damiao connection and its cameras.

The simulation lives in :mod:`dimos.robot.manipulators.dual_openyam.sim_rig`.
``DamiaoSimConnection`` only translates streams into targets, publishes
readings at a fixed rate, and publishes each rendered frame on the port and
under the frame names the rig's RealSense modules use.
``DualOpenYamSimConnection`` is the Dual OpenYAM's.
"""

from __future__ import annotations

from collections.abc import Mapping
import threading
import time
from typing import Any, ClassVar

import numpy as np
from pydantic import Field, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule, Values
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import movable_joints
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.hardware.sensors.camera.spec import OPTICAL_ROTATION
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.manipulators.damiao.robot import DamiaoRobot
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM
from dimos.robot.manipulators.dual_openyam.sim_rig import DamiaoSimRig, SimCameraFrame
from dimos.robot.manipulators.dual_openyam.sim_scene import (
    CAMERA_FOVY,
    DUAL_OPENYAM_WORKCELL,
    OVERHEAD_CAMERA,
    WRIST_CAMERAS,
    SimWorkcell,
)

# The frame every camera pose is published in: the planner's world frame, where
# the workcell's base link sits at the origin.
_WORLD_FRAME = "world"
# A camera_link (x forward, y left, z up) as seen from its optical frame.
_LINK_IN_OPTICAL = OPTICAL_ROTATION.to_rotation_matrix().T


def _frame(prefix: str | None, name: str) -> str:
    return f"{prefix}/{name}" if prefix else name


def _optical_frame(prefix: str | None) -> str:
    """The frame a camera's images are stamped with; depth is aligned to colour."""
    return _frame(prefix, "camera_color_optical_frame")


def _camera_transforms(
    prefix: str | None, frame: SimCameraFrame, imagers: tuple[str, ...], mount: bool = True
) -> list[Transform]:
    """A camera's tf tree for one frame: its mount, world to camera_link, then each imager.

    A RealSense publishes the imagers only. ``mount`` adds where the simulated
    camera is, for a stack in which nothing else says so.
    """
    link = _frame(prefix, "camera_link")
    transforms = []
    if mount:
        transforms.append(
            Transform(
                translation=Vector3(*frame.position),
                rotation=Quaternion.from_rotation_matrix(frame.rotation @ _LINK_IN_OPTICAL),
                frame_id=_WORLD_FRAME,
                child_frame_id=link,
                ts=frame.stamp,
            )
        )
    for imager in imagers:
        body = _frame(prefix, f"camera_{imager}_frame")
        transforms.append(Transform(frame_id=link, child_frame_id=body, ts=frame.stamp))
        transforms.append(
            Transform(
                rotation=OPTICAL_ROTATION,
                frame_id=body,
                child_frame_id=_frame(prefix, f"camera_{imager}_optical_frame"),
                ts=frame.stamp,
            )
        )
    return transforms


class DamiaoSimConnectionConfig(ConnectionConfig):
    """Settings of a simulated Damiao robot.

    Attributes:
        state_rate_hz: How often joint readings are published, in Hz.
        headless: Run without MuJoCo's viewer window.
        cameras: Render and publish the cameras; needs an OpenGL context.
        width: Width of every camera image, in pixels.
        height: Height of every camera image, in pixels.
        fps: How often the cameras are rendered, in frames per second.
        camera_info_fps: How often each camera's intrinsics are published, in Hz.
        enable_pointcloud: Publish the overhead camera's view as a coloured
            point cloud too, as its RealSense does with this option.
        pointcloud_fps: How often the cloud is published, in Hz.
        pointcloud_decimation: Every n-th pixel of a row and of a column goes
            into the cloud.
        depth_trunc_m: Cloud points farther than this from the camera are
            dropped, in metres.
        publish_overhead_mount: Publish where the overhead camera is, ``world``
            to ``camera_link``, on tf. Off where a planner publishes that edge
            itself, as a static transform.
    """

    state_rate_hz: float = Field(default=100.0, gt=0, allow_inf_nan=False)
    headless: bool = True
    cameras: bool = True
    width: int = Field(default=640, gt=0)
    height: int = Field(default=480, gt=0)
    fps: float = Field(default=30.0, gt=0, allow_inf_nan=False)
    camera_info_fps: float = Field(default=1.0, gt=0, allow_inf_nan=False)
    enable_pointcloud: bool = False
    pointcloud_fps: float = Field(default=5.0, gt=0, allow_inf_nan=False)
    pointcloud_decimation: int = Field(default=2, gt=0)
    depth_trunc_m: float = Field(default=5.0, gt=0, allow_inf_nan=False)
    publish_overhead_mount: bool = True

    @model_validator(mode="after")
    def _position_only(self) -> Self:
        if any(interfaces != (Interface.POSITION,) for interfaces in self.joint_commands.values()):
            raise ValueError("the simulated robot accepts position commands only")
        return self


#: A wrist camera's ports and frames: its image port, its intrinsics port and
#: the frame namespace its RealSense publishes in.
WristPorts = tuple[Out[Image], Out[CameraInfo], str]


class DamiaoSimConnection(ConnectionModule):
    """A Damiao robot in MuJoCo, where its Damiao connection drives the robot.

    Joint commands and readings use that connection's ports, names and units.
    The overhead D405 publishes on ``color_image``, ``depth_image``,
    ``camera_info``, ``depth_camera_info`` and, if asked to, ``pointcloud``;
    each wrist camera on the ports ``_wrist_ports`` names. Every camera's
    frames are on ``tf`` under the world frame, named as a RealSense names them.
    """

    #: The robot this connection simulates: joints, gains, grippers and models.
    robot: ClassVar[DamiaoRobot]
    #: Where the robot's arms are in the URDF the simulation is built from.
    workcell: ClassVar[SimWorkcell]
    config: DamiaoSimConnectionConfig
    position_command: In[JointState]
    joint_state: Out[JointState]
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    pointcloud: Out[PointCloud2]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        kwargs.setdefault(
            "joint_commands", {name: (Interface.POSITION,) for name in self.robot.joint_names}
        )
        kwargs.setdefault("model", self.robot.model)
        super().__init__(**kwargs)
        config = self.config
        self._rig = DamiaoSimRig(
            self.robot,
            self.workcell,
            headless=config.headless,
            camera_size=(config.width, config.height) if config.cameras else None,
            camera_fps=config.fps,
            on_frames=self._publish_frames,
        )
        self._state_frame = ""
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None
        # When the intrinsics are next due, on the monotonic clock, and the
        # stamp of the first frame the next cloud may be built from; only the
        # rendering thread reads them.
        self._camera_info_due = 0.0
        self._pointcloud_due = 0.0

    def connect(self) -> None:
        """Check the deployed model against the robot's joints, then start the rig."""
        assert self.config.model is not None
        joints = {joint.name for joint in movable_joints(self.config.model)}
        if joints != set(self.robot.joint_names):
            raise ValueError("connection model joints must match the robot's joints")
        self._state_frame = self.frame(self.config.model.load().root_link)
        self._rig.connect()

    @rpc
    def start(self) -> None:
        try:
            super().start()
            self.register_disposable(Disposable(self.position_command.subscribe(self._on_position)))
            self._reader = threading.Thread(
                target=self._read_loop, daemon=True, name=f"{self.config.source}-read"
            )
            self._reader.start()
        except Exception:
            self.stop()
            raise

    @rpc
    def stop(self) -> None:
        self._stop_reading.set()
        if (
            self._reader is not None
            and self._reader.is_alive()
            and self._reader is not threading.current_thread()
        ):
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        try:
            # The rig goes first: its rendering thread publishes on these ports.
            self._rig.disconnect()
        finally:
            super().stop()

    @rpc
    def body_position(self, name: str) -> list[float]:
        """Where a body of the simulated workcell is, by the simulation itself.

        Args:
            name: A body of the workcell, e.g. "rubiks_cube", "cup" or "bottle".

        Returns its frame's position, metres in the world frame.
        """
        return [float(value) for value in self._rig.body_position(name)]

    @rpc
    def place_body(self, name: str, position: list[float], yaw: float = 0.0) -> None:
        """Stand one of the simulated objects somewhere, upright and at rest.

        Args:
            name: An object on the table, e.g. "rubiks_cube", "cup" or "bottle".
            position: Where its frame goes, metres in the world frame. The cup
                and the bottle have theirs at the foot, the cube at its centre.
            yaw: How far it is turned about the vertical, rad.
        """
        self._rig.place_body(name, position, yaw)

    def _on_position(self, message: JointState) -> None:
        try:
            values = joint_state_to_values(message, (Interface.POSITION,))
        except ValueError as error:
            self._report(str(error), rejection=True)
            return
        self.command(values)

    def write(self, values: Values) -> None:
        prefix = f"{self.config.source}/"
        self._rig.command(
            {
                key.removeprefix(prefix).removesuffix("/position"): value
                for key, value in values.items()
            }
        )

    def halt(self) -> None:
        self._rig.hold()

    def _read_loop(self) -> None:
        period = 1.0 / self.config.state_rate_hz
        names = [f"{self.config.source}/{name}" for name in self.robot.joint_names]
        while not self._stop_reading.is_set():
            started = time.monotonic()
            try:
                reading = self._rig.read()
            except RuntimeError as error:
                # Without readings the coordinator stops commanding the robot.
                self._report(f"feedback failed: {error}")
            else:
                self.joint_state.publish(
                    JointState(
                        ts=time.time(),
                        frame_id=self._state_frame,
                        name=names,
                        position=reading.position,
                        velocity=reading.velocity,
                        effort=reading.effort,
                    )
                )
            self._stop_reading.wait(max(0.0, period - (time.monotonic() - started)))

    def _publish_frames(self, frames: Mapping[str, SimCameraFrame]) -> None:
        """Runs on the rig's rendering thread with every camera's new frame."""
        now = time.monotonic()
        with_info = now >= self._camera_info_due
        if with_info:
            self._camera_info_due = now + 1.0 / self.config.camera_info_fps

        overhead = frames[OVERHEAD_CAMERA]
        optical = _optical_frame(None)
        color = Image(
            data=overhead.color, format=ImageFormat.RGB, frame_id=optical, ts=overhead.stamp
        )
        self.color_image.publish(color)
        assert overhead.depth is not None, "the overhead camera is RGB-D"
        depth = Image(
            data=overhead.depth, format=ImageFormat.DEPTH, frame_id=optical, ts=overhead.stamp
        )
        self.depth_image.publish(depth)
        if with_info:
            info = self._camera_info(optical, overhead.stamp)
            self.camera_info.publish(info)
            self.depth_camera_info.publish(info)
        if self.config.enable_pointcloud and overhead.stamp >= self._pointcloud_due:
            self._pointcloud_due = overhead.stamp + 1.0 / self.config.pointcloud_fps
            self.pointcloud.publish(self._point_cloud(color, depth))
        transforms = _camera_transforms(
            None, overhead, ("color", "depth"), mount=self.config.publish_overhead_mount
        )

        for camera, (image_port, info_port, prefix) in self._wrist_ports().items():
            frame = frames[camera]
            optical = _optical_frame(prefix)
            image_port.publish(
                Image(data=frame.color, format=ImageFormat.RGB, frame_id=optical, ts=frame.stamp)
            )
            if with_info:
                info_port.publish(self._camera_info(optical, frame.stamp))
            transforms.extend(_camera_transforms(prefix, frame, ("color",)))

        self.tf.publish(TFMessage(*transforms))

    def _wrist_ports(self) -> dict[str, WristPorts]:
        """Each wrist camera of the workcell, by name, and where its frames go."""
        raise NotImplementedError

    def _camera_info(self, frame_id: str, stamp: float) -> CameraInfo:
        """Pinhole intrinsics of a rendered image: square pixels, no distortion."""
        return CameraInfo.from_fov(
            CAMERA_FOVY, self.config.width, self.config.height, frame_id=frame_id
        ).with_ts(stamp)

    def _point_cloud(self, color: Image, depth: Image) -> PointCloud2:
        """An RGB-D frame as coloured points in its optical frame, as a RealSense builds them.

        Every n-th pixel is put back through the pinhole. The cloud has the
        depth image's frame and stamp, and no point past ``depth_trunc_m``.
        """
        step = self.config.pointcloud_decimation
        thinned_color, thinned_depth = (
            Image(
                data=np.ascontiguousarray(image.data[::step, ::step]),
                format=image.format,
                frame_id=image.frame_id,
                ts=image.ts,
            )
            for image in (color, depth)
        )
        full = self._camera_info(depth.frame_id, depth.ts)
        # Pixel n of a thinned image is pixel n * step of the full one.
        thinned = CameraInfo.from_intrinsics(
            fx=full.K[0] / step,
            fy=full.K[4] / step,
            cx=full.K[2] / step,
            cy=full.K[5] / step,
            width=thinned_depth.width,
            height=thinned_depth.height,
            frame_id=depth.frame_id,
        )
        return PointCloud2.from_rgbd(
            thinned_color, thinned_depth, thinned, depth_trunc=self.config.depth_trunc_m
        )


class DualOpenYamSimConnectionConfig(DamiaoSimConnectionConfig):
    """Settings of the simulated Dual OpenYAM."""

    source: str = "dual_openyam"


class DualOpenYamSimConnection(DamiaoSimConnection):
    """Both OpenYAM arms and jaws in MuJoCo, where ``DualOpenYamConnection`` drives the robot.

    Each wrist D405 publishes on ``{side}_wrist_color_image`` and
    ``{side}_wrist_camera_info``, in the frames of the "{side}_wrist" RealSense.
    """

    robot = DUAL_OPENYAM
    workcell = DUAL_OPENYAM_WORKCELL
    config: DualOpenYamSimConnectionConfig
    left_wrist_color_image: Out[Image]
    left_wrist_camera_info: Out[CameraInfo]
    right_wrist_color_image: Out[Image]
    right_wrist_camera_info: Out[CameraInfo]

    def _wrist_ports(self) -> dict[str, WristPorts]:
        return {
            WRIST_CAMERAS["left"]: (
                self.left_wrist_color_image,
                self.left_wrist_camera_info,
                "left_wrist",
            ),
            WRIST_CAMERAS["right"]: (
                self.right_wrist_color_image,
                self.right_wrist_camera_info,
                "right_wrist",
            ),
        }
