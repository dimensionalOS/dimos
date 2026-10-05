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


"""The robot's streams as plain Zenoh topics, for agents that have no dimOS.

One bridge for any robot: every input it declares is optional, so a stream the
blueprint does not provide simply never publishes. Camera frames go out as JPEG,
depth and lidar as float32, odometry, joint state and poses as JSON. Velocity
commands come back as JSON with a deadman, for the base (``cmd_vel``) and for an
arm's end effector (``ee_twist_command``); gripper openings go straight through.
That is the surface a vendor SDK exposes, and nothing dimOS builds on top of it.
The bridge opens its own Zenoh session on a fixed endpoint with scouting off, so
a subscriber sees these keys and none of dimOS's own topics.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import io
import json
import math
import threading
import time
from typing import Any

import numpy as np
from PIL import Image as PILImage
from scipy.spatial.transform import Rotation
import zenoh

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.evals.constants import (
    RAW_DRIVE_HZ,
    RAW_ENDPOINT,
    RAW_JPEG_QUALITY,
    RAW_MAX_ANGULAR_RPS,
    RAW_MAX_CMD_S,
    RAW_MAX_EE_ANGULAR_RPS,
    RAW_MAX_EE_LINEAR_MPS,
    RAW_MAX_LINEAR_MPS,
    RAW_TOPIC_PREFIX,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def zenoh_config(endpoint: str, *, listen: bool) -> zenoh.Config:
    """A peer that talks only to *endpoint*: no multicast or gossip scouting."""
    config = zenoh.Config()
    config.insert_json5("mode", '"peer"')
    config.insert_json5(
        "listen/endpoints" if listen else "connect/endpoints", json.dumps([endpoint])
    )
    config.insert_json5("scouting/multicast/enabled", "false")
    config.insert_json5("scouting/gossip/enabled", "false")
    return config


class RawTopics:
    """Publish and subscribe on the bridge's key space; shared by the module and its clients."""

    def __init__(
        self, endpoint: str = RAW_ENDPOINT, prefix: str = RAW_TOPIC_PREFIX, *, listen: bool
    ) -> None:
        self.prefix = prefix
        self.session = zenoh.open(zenoh_config(endpoint, listen=listen))
        self._publishers: dict[str, zenoh.Publisher] = {}

    def put(self, key: str, payload: bytes | str, ts: float | None = None) -> None:
        publisher = self._publishers.get(key)
        if publisher is None:
            publisher = self._publishers[key] = self.session.declare_publisher(
                f"{self.prefix}/{key}"
            )
        publisher.put(payload, attachment=None if ts is None else json.dumps({"t": ts}))

    def subscribe(
        self, key: str, handler: Callable[[bytes, float | None], None]
    ) -> zenoh.Subscriber[Any]:
        def on_sample(sample: zenoh.Sample) -> None:
            ts = (
                json.loads(sample.attachment.to_bytes())["t"]
                if sample.attachment is not None
                else None
            )
            handler(sample.payload.to_bytes(), ts)

        return self.session.declare_subscriber(f"{self.prefix}/{key}", on_sample)

    def close(self) -> None:
        self.session.close()


def jpeg_bytes(image: Image, quality: int = 90) -> bytes:
    buf = io.BytesIO()
    PILImage.fromarray(image.to_rgb().as_numpy()).save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


def depth_f32(image: Image) -> bytes:
    """Metric optical-axis depth without quantization or image compression."""
    depth = image.as_numpy()
    if image.format != ImageFormat.DEPTH or depth.ndim != 2 or depth.dtype.kind != "f":
        raise ValueError("Raw depth requires a 2D floating-point DEPTH image in metres")
    return np.ascontiguousarray(depth, dtype="<f4").tobytes()


def xyz_f32(cloud: PointCloud2) -> bytes:
    points, _ = cloud.as_numpy()
    return np.ascontiguousarray(points, dtype="<f4").tobytes()


def odom_json(pose: PoseStamped) -> str:
    p, q = pose.position, pose.orientation
    return json.dumps(
        {"t": pose.ts, "x": p.x, "y": p.y, "z": p.z, "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w}
    )


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@dataclass
class Deadman:
    """The latest velocity command, valid until its deadline passes.

    ``linear`` and ``angular`` name the JSON fields it reads; the defaults are a
    planar base's ``{"vx", "vy", "wz", "t"}``.
    """

    max_s: float = RAW_MAX_CMD_S
    max_linear: float = RAW_MAX_LINEAR_MPS
    max_angular: float = RAW_MAX_ANGULAR_RPS
    linear: tuple[str, ...] = ("vx", "vy")
    angular: tuple[str, ...] = ("wz",)
    values: tuple[float, ...] = ()
    until: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def __post_init__(self) -> None:
        for name in ("max_s", "max_linear", "max_angular"):
            limit = getattr(self, name)
            if not (math.isfinite(limit) and limit > 0):
                raise ValueError(f"{name} must be a finite positive limit, got {limit!r}")
        self.values = self._zero

    @property
    def _zero(self) -> tuple[float, ...]:
        return (0.0,) * (len(self.linear) + len(self.angular))

    def set(self, command: bytes | str) -> None:
        c = json.loads(command)
        raw = [float(c.get(k, 0.0)) for k in (*self.linear, *self.angular, "t")]
        if not all(math.isfinite(v) for v in raw):
            raise ValueError("velocity command must be finite")
        *velocity, hold = raw
        limits = (self.max_linear,) * len(self.linear) + (self.max_angular,) * len(self.angular)
        with self.lock:
            self.values = tuple(_clamp(v, lim) for v, lim in zip(velocity, limits, strict=True))
            self.until = time.monotonic() + max(0.0, min(hold, self.max_s))

    def current(self) -> tuple[float, ...]:
        with self.lock:
            return self.values if time.monotonic() < self.until else self._zero

    def clear(self) -> None:
        with self.lock:
            self.values, self.until = self._zero, 0.0


class RawRobotBridgeConfig(ModuleConfig):
    endpoint: str = RAW_ENDPOINT
    prefix: str = RAW_TOPIC_PREFIX
    max_cmd_s: float = RAW_MAX_CMD_S
    max_linear_mps: float = RAW_MAX_LINEAR_MPS
    max_angular_rps: float = RAW_MAX_ANGULAR_RPS
    max_ee_linear_mps: float = RAW_MAX_EE_LINEAR_MPS
    max_ee_angular_rps: float = RAW_MAX_EE_ANGULAR_RPS
    jpeg_quality: int = RAW_JPEG_QUALITY
    drive_hz: float = RAW_DRIVE_HZ
    state_hz: float = 20.0
    stale_s: float = 1.0
    camera_frame: str | None = None
    overview_frame: str | None = None
    ee_frame: str | None = None
    gripper_joint: str | None = None
    gripper_range: tuple[float, float] = (0.0, 1.0)


class RawRobotBridge(Module):
    """Vendor-shaped topics from whatever streams the robot provides; JSON commands back in."""

    config: RawRobotBridgeConfig

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    overview_image: In[Image]
    overview_camera_info: In[CameraInfo]
    lidar: In[PointCloud2]
    odom: In[PoseStamped]
    coordinator_joint_state: In[JointState]
    tf: In[TFMessage]
    cmd_vel: Out[Twist]
    ee_twist_command: Out[TwistStamped]
    gripper_command: Out[Float32]

    _topics: RawTopics | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        cfg = self.config
        self._base = Deadman(cfg.max_cmd_s, cfg.max_linear_mps, cfg.max_angular_rps)
        self._arm = Deadman(
            cfg.max_cmd_s,
            cfg.max_ee_linear_mps,
            cfg.max_ee_angular_rps,
            linear=("vx", "vy", "vz"),
            angular=("wx", "wy", "wz"),
        )
        self._lock = threading.Lock()
        self._ee_pose: dict[str, Any] | None = None
        self._ee_seen = float("-inf")
        self._last_state = float("-inf")
        self._stop = threading.Event()
        self._moving = {"base": False, "arm": False}
        self._drive_thread: threading.Thread | None = None
        self._depth_warned = False

    @rpc
    def start(self) -> None:
        super().start()
        cfg = self.config
        self._stop.clear()  # a restarted bridge drives again
        self._topics = RawTopics(cfg.endpoint, cfg.prefix, listen=True)
        q = cfg.jpeg_quality
        self.color_image.subscribe(lambda img: self._put("camera/jpeg", jpeg_bytes(img, q), img.ts))
        self.depth_image.subscribe(self._on_depth)
        self.overview_image.subscribe(
            lambda img: self._put("overview/jpeg", jpeg_bytes(img, q), img.ts)
        )
        self.overview_camera_info.subscribe(
            lambda info: self._put(
                "overview/camera_info/json",
                json.dumps({"width": info.width, "height": info.height, "K": info.K}),
                info.ts,
            )
        )
        self.lidar.subscribe(lambda cloud: self._put("lidar/xyz_f32", xyz_f32(cloud), cloud.ts))
        self.odom.subscribe(lambda pose: self._put("odom/json", odom_json(pose)))
        self.camera_info.subscribe(
            lambda info: self._put(
                "camera_info/json",
                json.dumps({"width": info.width, "height": info.height, "K": info.K}),
            )
        )
        self.coordinator_joint_state.subscribe(self._on_joint_state)
        self.tf.subscribe(self._on_tf)
        self._subs = [
            self._topics.subscribe("cmd_vel/json", self._command(self._base.set)),
            self._topics.subscribe("arm/twist/json", self._command(self._arm.set)),
            self._topics.subscribe("arm/gripper/json", self._command(self._on_gripper)),
        ]
        self._drive_thread = threading.Thread(
            target=self._drive, daemon=True, name="raw-robot-drive"
        )
        self._drive_thread.start()

    @rpc
    def stop(self) -> None:
        if self._topics is not None:
            self._stop.set()
            if self._drive_thread is not None:
                self._drive_thread.join(timeout=1.0)  # one drive loop across restarts
                self._drive_thread = None
            self._base.clear()  # a restart must not resume a held command
            self._arm.clear()
            self._moving = {"base": False, "arm": False}
            self.cmd_vel.publish(Twist())
            self.ee_twist_command.publish(TwistStamped())
            self._topics.close()
            self._topics = None
        super().stop()

    def _put(self, key: str, payload: bytes | str, ts: float | None = None) -> None:
        if self._topics is not None:
            self._topics.put(key, payload, ts)

    @staticmethod
    def _command(handle: Callable[[bytes], None]) -> Callable[[bytes, float | None], None]:
        def on_command(payload: bytes, _ts: float | None) -> None:
            try:
                handle(payload)
            except (ValueError, TypeError, AttributeError, KeyError):
                return  # a malformed packet is dropped, as an SDK would

        return on_command

    def _on_gripper(self, payload: bytes) -> None:
        opening = float(json.loads(payload)["opening"])
        if not 0.0 <= opening <= 1.0:  # also rejects NaN
            raise ValueError("gripper opening must be within 0..1")
        self.gripper_command.publish(Float32(data=opening))

    def _drive(self) -> None:
        """Republish each held velocity until its deadman expires, then one zero."""
        moving = self._moving
        while not self._stop.wait(1.0 / self.config.drive_hz):
            vx, vy, wz = self._base.current()
            if (vx, vy, wz) != (0.0, 0.0, 0.0):
                self.cmd_vel.publish(Twist(linear=(vx, vy, 0.0), angular=(0.0, 0.0, wz)))
                moving["base"] = True
            elif moving["base"]:
                self.cmd_vel.publish(Twist())
                moving["base"] = False
            arm = self._arm.current()
            if any(arm):
                self.ee_twist_command.publish(TwistStamped(linear=arm[:3], angular=arm[3:]))
                moving["arm"] = True
            elif moving["arm"]:
                self.ee_twist_command.publish(TwistStamped())
                moving["arm"] = False

    def _on_depth(self, image: Image) -> None:
        try:
            payload = depth_f32(image)
        except ValueError as exc:
            if not self._depth_warned:
                self._depth_warned = True
                logger.warning("Raw depth frames rejected", error=str(exc))
            return
        info = {
            "t": image.ts,
            "width": image.width,
            "height": image.height,
            "dtype": "<f4",
            "unit": "metres",
            "frame_id": image.frame_id,
        }
        self._put("camera/depth_info/json", json.dumps(info), image.ts)
        self._put("camera/depth_f32", payload, image.ts)

    def _on_joint_state(self, state: JointState) -> None:
        now = time.monotonic()
        if now - self._last_state < 1.0 / self.config.state_hz:
            return
        self._last_state = now
        gripper_joint = self.config.gripper_joint
        positions = dict(zip(state.name, state.position, strict=False))
        velocities = dict(zip(state.name, state.velocity, strict=False))
        joints = [n for n in state.name if n != gripper_joint]
        message: dict[str, Any] = {
            "t": state.ts,
            "joint_names": joints,
            "positions": [positions[n] for n in joints],
            "velocities": [velocities.get(n, 0.0) for n in joints],
        }
        if self.config.ee_frame is not None:
            # Unstamped: the 30 Hz tf pose trails `t` by at most ~33 ms; add its stamp if needed.
            with self._lock:
                fresh = now - self._ee_seen <= self.config.stale_s
                message["ee_pose"] = self._ee_pose if fresh else None
        if gripper_joint is not None and gripper_joint in positions:
            closed, opened = self.config.gripper_range
            opening = (positions[gripper_joint] - closed) / (opened - closed)
            message["gripper_opening"] = float(np.clip(opening, 0.0, 1.0))
        try:
            payload = json.dumps(message, allow_nan=False)
        except ValueError:
            return  # a non-finite joint value would make invalid JSON
        self._put("arm/state/json", payload, state.ts)

    def _on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            child = transform.child_frame_id
            cameras = {
                self.config.camera_frame: "camera_pose/json",
                self.config.overview_frame: "overview/camera_pose/json",
            }
            if child not in (*cameras, self.config.ee_frame) or child is None:
                continue
            q = transform.rotation.to_numpy()
            pose = {
                "frame": transform.frame_id,
                "xyz": transform.translation.to_numpy().tolist(),
                "quaternion_xyzw": Rotation.from_quat(q).as_quat().tolist(),
            }
            if child == self.config.ee_frame:
                with self._lock:
                    self._ee_pose = pose
                    self._ee_seen = time.monotonic()
            else:
                self._put(cameras[child], json.dumps({"t": transform.ts, **pose}), transform.ts)
