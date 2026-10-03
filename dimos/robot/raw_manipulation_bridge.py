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


"""Plain agent inputs to the existing EE-twist and gripper streams, with robot observations.

Wired like keyboard and hosted teleop: the bridge publishes ``ee_twist_command`` and
``gripper_command`` for the coordinator's eef_twist and gripper tasks. No robot model,
IK, command IDs or replies. A twist is held for its ``t`` seconds (a deadman, like the
raw navigation bridge's cmd_vel), then a single zero twist is sent.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Annotated, Any, Literal

import numpy as np
from numpy.typing import NDArray
from pydantic import BaseModel, ConfigDict, Field, TypeAdapter
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.evals.constants import (
    RAW_ENDPOINT,
    RAW_JPEG_QUALITY,
    RAW_MAX_CMD_S,
    RAW_MAX_EE_ANGULAR_RPS,
    RAW_MAX_EE_LINEAR_MPS,
    RAW_TOPIC_PREFIX,
)
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.raw_robot_bridge import RawTopics, jpeg_bytes
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

Vec3 = tuple[float, float, float]


class _Command(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, strict=True)


class TwistCommand(_Command):
    kind: Literal["twist"]
    linear: Vec3 = (0.0, 0.0, 0.0)
    angular: Vec3 = (0.0, 0.0, 0.0)
    t: float = Field(ge=0)


class GripperCommand(_Command):
    kind: Literal["gripper"]
    opening: float = Field(ge=0, le=1)


COMMAND: TypeAdapter[TwistCommand | GripperCommand] = TypeAdapter(
    Annotated[TwistCommand | GripperCommand, Field(discriminator="kind")]
)


def pose_json(matrix: NDArray[np.float64]) -> dict[str, Any]:
    return {
        "xyz": matrix[:3, 3].tolist(),
        "quaternion_xyzw": Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist(),
    }


def depth_f32(image: Image) -> bytes:
    """Metric optical-axis depth without quantization or image compression."""
    depth = image.as_numpy()
    if image.format != ImageFormat.DEPTH or depth.ndim != 2 or depth.dtype.kind != "f":
        raise ValueError("Raw depth requires a 2D floating-point DEPTH image in metres")
    return np.ascontiguousarray(depth, dtype="<f4").tobytes()


class RawManipulationBridgeConfig(ModuleConfig):
    endpoint: str = RAW_ENDPOINT
    prefix: str = RAW_TOPIC_PREFIX
    gripper_joint: str = "arm/gripper"
    gripper_range: tuple[float, float] = (0.0, 0.85)  # native (closed, open)
    camera_optical_frame: str = "wrist_camera_color_optical_frame"
    overview_optical_frame: str = "env_camera_color_optical_frame"
    max_cmd_s: float = RAW_MAX_CMD_S
    max_linear_mps: float = RAW_MAX_EE_LINEAR_MPS
    max_angular_rps: float = RAW_MAX_EE_ANGULAR_RPS
    drive_hz: float = Field(default=20, gt=0, le=100, allow_inf_nan=False)
    state_hz: float = Field(default=20, gt=0, le=100, allow_inf_nan=False)
    stale_s: float = Field(default=1, gt=0, allow_inf_nan=False)
    jpeg_quality: int = RAW_JPEG_QUALITY


class RawManipulationBridge(Module):
    """Another input source for the eef_twist and gripper tasks, like keyboard teleop."""

    config: RawManipulationBridgeConfig
    ee_twist_command: Out[TwistStamped]
    gripper_command: Out[Float32]
    coordinator_joint_state: In[JointState]
    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    overview_image: In[Image]
    overview_camera_info: In[CameraInfo]
    tf: In[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._topics: RawTopics | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._twist = TwistStamped()
        self._until = 0.0
        self._moving = False
        self._last_state = float("-inf")
        self._last_info = float("-inf")

    @rpc
    def start(self) -> None:
        super().start()
        self._topics = RawTopics(self.config.endpoint, self.config.prefix, listen=True)
        self._subscriber = self._topics.subscribe("arm/command/json", self._on_command)
        self.coordinator_joint_state.subscribe(self._on_joint_state)
        self.color_image.subscribe(self._on_image)
        self.depth_image.subscribe(self._on_depth)
        self.camera_info.subscribe(self._on_camera_info)
        self.overview_image.subscribe(self._on_overview_image)
        self.overview_camera_info.subscribe(self._on_overview_camera_info)
        self.tf.subscribe(self._on_tf)
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._drive, name="raw-manipulation-drive", daemon=True
        )
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)
        if self._topics is not None:
            self.ee_twist_command.publish(TwistStamped())
            self._topics.close()
            self._topics = None
        super().stop()

    def _on_command(self, payload: bytes, _ts: float | None) -> None:
        try:
            command = COMMAND.validate_json(payload)
        except ValueError as exc:
            logger.warning("Raw command rejected", error=str(exc))
            return
        if isinstance(command, GripperCommand):
            self.gripper_command.publish(Float32(data=command.opening))
            return
        lin, ang = self.config.max_linear_mps, self.config.max_angular_rps
        twist = TwistStamped(
            linear=[float(np.clip(v, -lin, lin)) for v in command.linear],
            angular=[float(np.clip(v, -ang, ang)) for v in command.angular],
        )
        with self._lock:
            self._twist = twist
            self._until = time.monotonic() + min(command.t, self.config.max_cmd_s)

    def _drive(self) -> None:
        """Republish the held twist until its deadline, then one zero (the eef_twist task's
        own command timeout stops the arm if this loop dies)."""
        while not self._stop.wait(1 / self.config.drive_hz):
            with self._lock:
                twist = self._twist if time.monotonic() < self._until else None
            if twist is not None:
                self.ee_twist_command.publish(twist)
                self._moving = True
            elif self._moving:
                self.ee_twist_command.publish(TwistStamped())
                self._moving = False
            now = time.monotonic()
            if now - self._last_info >= 1:
                self._last_info = now
                self._put("arm/info/json", self._info())

    def _info(self) -> dict[str, Any]:
        return {
            "commands": ["twist", "gripper"],
            "twist_frame": "world",
            "max_linear_mps": self.config.max_linear_mps,
            "max_angular_rps": self.config.max_angular_rps,
            "max_cmd_s": self.config.max_cmd_s,
            "gripper": {"unit": "normalized", "closed": 0.0, "open": 1.0},
        }

    def _on_joint_state(self, state: JointState) -> None:
        now = time.monotonic()
        if now - self._last_state < 1 / self.config.state_hz:
            return
        self._last_state = now
        names = list(state.name)
        positions = dict(zip(names, state.position, strict=False))
        velocities = dict(zip(names, state.velocity, strict=False))
        arm = [n for n in names if n != self.config.gripper_joint]
        closed, opened = self.config.gripper_range
        gripper = positions.get(self.config.gripper_joint)
        self._put(
            "arm/state/json",
            {
                "t": state.ts,
                "joint_names": arm,
                "positions": [positions[n] for n in arm],
                "velocities": [velocities.get(n, 0.0) for n in arm],
                "gripper_opening": None
                if gripper is None
                else float(np.clip((gripper - closed) / (opened - closed), 0, 1)),
            },
            state.ts,
        )

    def _put(self, key: str, value: Any, ts: float | None = None) -> None:
        if self._topics is not None:
            self._topics.put(key, json.dumps(value, allow_nan=False), ts)

    def _on_tf(self, message: TFMessage) -> None:
        topics = {
            self.config.camera_optical_frame: "camera_pose/json",
            self.config.overview_optical_frame: "overview/camera_pose/json",
        }
        for transform in message.transforms:
            topic = topics.get(transform.child_frame_id)
            if topic is None:
                continue
            matrix = np.eye(4)
            matrix[:3, 3] = transform.translation.to_numpy()
            matrix[:3, :3] = Rotation.from_quat(transform.rotation.to_numpy()).as_matrix()
            self._put(
                topic,
                {"t": transform.ts, "frame": transform.frame_id, **pose_json(matrix)},
                transform.ts,
            )

    def _on_image(self, image: Image) -> None:
        if self._topics is not None:
            self._topics.put("camera/jpeg", jpeg_bytes(image, self.config.jpeg_quality), image.ts)

    def _on_depth(self, image: Image) -> None:
        if self._topics is None:
            return
        try:
            payload = depth_f32(image)
        except ValueError as exc:
            logger.warning("Raw depth frame rejected", error=str(exc))
            return
        self._put(
            "camera/depth_info/json",
            {
                "t": image.ts,
                "width": image.width,
                "height": image.height,
                "dtype": "<f4",
                "unit": "metres",
                "frame_id": image.frame_id,
            },
            image.ts,
        )
        self._topics.put("camera/depth_f32", payload, image.ts)

    def _on_camera_info(self, info: CameraInfo) -> None:
        self._put(
            "camera_info/json", {"width": info.width, "height": info.height, "K": info.K}, info.ts
        )

    def _on_overview_image(self, image: Image) -> None:
        if self._topics is not None:
            self._topics.put("overview/jpeg", jpeg_bytes(image, self.config.jpeg_quality), image.ts)

    def _on_overview_camera_info(self, info: CameraInfo) -> None:
        self._put(
            "overview/camera_info/json",
            {"width": info.width, "height": info.height, "K": info.K},
            info.ts,
        )
