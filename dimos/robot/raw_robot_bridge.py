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

"""The robot connection's streams as plain Zenoh topics, for agents that have no dimOS.

Camera frames go out as JPEG, lidar as float32 xyz, odometry and intrinsics as JSON;
velocity commands come back as JSON with a deadman. That is the surface a vendor SDK
exposes, and nothing dimOS builds on top of it. The bridge opens its own Zenoh session
on a fixed endpoint with scouting off, so a subscriber sees these keys and none of
dimOS's own topics.

``topics`` selects the keys. ``world_state`` is the text-only surface: the JSON the
TypeSafe agent reads (pose, objects with bearing and distance, room sectors), for
benchmarks where every arm sees text and nothing else. ``finished`` lets the agent
declare the task done; it is republished on the ``finished`` stream.
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

from dimos_lcm.std_msgs import Bool
import numpy as np
from PIL import Image as PILImage
import zenoh

from dimos.agents.typesafe.world_state import build_world_state
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.evals.constants import (
    RAW_DRIVE_HZ,
    RAW_ENDPOINT,
    RAW_JPEG_QUALITY,
    RAW_MAX_ANGULAR_RPS,
    RAW_MAX_CMD_S,
    RAW_MAX_LINEAR_MPS,
    RAW_TOPIC_PREFIX,
    RAW_TOPICS,
    RAW_WORLD_STATE_HZ,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray


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
    """The latest velocity command, valid until its deadline passes."""

    max_s: float = RAW_MAX_CMD_S
    max_linear: float = RAW_MAX_LINEAR_MPS
    max_angular: float = RAW_MAX_ANGULAR_RPS
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    until: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def __post_init__(self) -> None:
        for name in ("max_s", "max_linear", "max_angular"):
            limit = getattr(self, name)
            if not (math.isfinite(limit) and limit > 0):
                raise ValueError(f"{name} must be a finite positive limit, got {limit!r}")

    def set(self, command: bytes | str) -> None:
        c = json.loads(command)
        vx, vy, wz, hold = (float(c.get(k, 0.0)) for k in ("vx", "vy", "wz", "t"))
        if not all(math.isfinite(v) for v in (vx, vy, wz, hold)):
            raise ValueError("velocity command must be finite")
        with self.lock:
            self.vx, self.vy = _clamp(vx, self.max_linear), _clamp(vy, self.max_linear)
            self.wz = _clamp(wz, self.max_angular)
            self.until = time.monotonic() + max(0.0, min(hold, self.max_s))

    def current(self) -> tuple[float, float, float]:
        with self.lock:
            return (self.vx, self.vy, self.wz) if time.monotonic() < self.until else (0.0, 0.0, 0.0)


class RawRobotBridgeConfig(ModuleConfig):
    endpoint: str = RAW_ENDPOINT
    prefix: str = RAW_TOPIC_PREFIX
    max_cmd_s: float = RAW_MAX_CMD_S
    max_linear_mps: float = RAW_MAX_LINEAR_MPS
    max_angular_rps: float = RAW_MAX_ANGULAR_RPS
    jpeg_quality: int = RAW_JPEG_QUALITY
    drive_hz: float = RAW_DRIVE_HZ
    topics: tuple[str, ...] = RAW_TOPICS
    world_state_hz: float = RAW_WORLD_STATE_HZ
    stale_s: float = 2.0  # inputs older than this leave the world state
    lidar_z_min: float = -0.2
    lidar_z_max: float = 0.8
    lidar_max_range: float = 5.0


class RawRobotBridge(Module):
    """Vendor-shaped topics from the connection's streams; JSON velocity commands back in."""

    config: RawRobotBridgeConfig

    color_image: In[Image]
    lidar: In[PointCloud2]
    odom: In[PoseStamped]
    camera_info: In[CameraInfo]
    detections_3d: In[Detection3DArray]
    cmd_vel: Out[Twist]
    finished: Out[Bool]

    _topics: RawTopics | None = None

    @rpc
    def start(self) -> None:
        super().start()
        unknown = set(self.config.topics) - set(RAW_TOPICS)
        if unknown:
            raise ValueError(f"unknown raw topics {sorted(unknown)}; choose from {RAW_TOPICS}")
        self._deadman = Deadman(
            self.config.max_cmd_s, self.config.max_linear_mps, self.config.max_angular_rps
        )
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: dict[str, tuple[float, Any]] = {}
        self._topics = RawTopics(self.config.endpoint, self.config.prefix, listen=True)
        on = set(self.config.topics)
        q = self.config.jpeg_quality
        if "camera" in on:
            self.color_image.subscribe(
                lambda img: self._put("camera/jpeg", jpeg_bytes(img, q), img.ts)
            )
        if "lidar" in on:
            self.lidar.subscribe(lambda cloud: self._put("lidar/xyz_f32", xyz_f32(cloud), cloud.ts))
        if "odom" in on:
            self.odom.subscribe(lambda pose: self._put("odom/json", odom_json(pose)))
        if "camera_info" in on:
            self.camera_info.subscribe(
                lambda info: self._put(
                    "camera_info/json",
                    json.dumps({"width": info.width, "height": info.height, "K": info.K}),
                )
            )
        if "world_state" in on:
            for name in ("odom", "detections_3d", "lidar"):
                getattr(self, name).subscribe(self._keep(name))
            threading.Thread(target=self._world_state, daemon=True, name="raw-world-state").start()
        if "cmd_vel" in on:
            self._cmd_sub = self._topics.subscribe("cmd_vel/json", self._on_command)
            threading.Thread(target=self._drive, daemon=True, name="raw-robot-drive").start()
        if "finished" in on:
            self._finished_sub = self._topics.subscribe("finished/json", self._on_finished)

    @rpc
    def stop(self) -> None:
        if self._topics is not None:
            self._stop.set()
            self.cmd_vel.publish(Twist())
            self._topics.close()
            self._topics = None
        super().stop()

    def _put(self, key: str, payload: bytes | str, ts: float | None = None) -> None:
        if self._topics is not None:
            self._topics.put(key, payload, ts)

    def _on_command(self, payload: bytes, _ts: float | None) -> None:
        try:
            self._deadman.set(payload)
        except (ValueError, TypeError, AttributeError):
            return  # a malformed packet is dropped, as an SDK would

    def _on_finished(self, payload: bytes, _ts: float | None) -> None:
        try:
            done = bool(json.loads(payload).get("done", True))
        except (ValueError, TypeError, AttributeError):
            return
        if done:
            self.finished.publish(Bool(True))

    def _keep(self, name: str) -> Callable[[Any], None]:
        def on_msg(msg: Any) -> None:
            with self._lock:
                self._latest[name] = (time.monotonic(), msg)

        return on_msg

    def _fresh(self, name: str) -> Any:
        with self._lock:
            item = self._latest.get(name)
        return None if item is None or time.monotonic() - item[0] > self.config.stale_s else item[1]

    def _world_state(self) -> None:
        while not self._stop.wait(1.0 / self.config.world_state_hz):
            pose = self._fresh("odom")
            if pose is None:
                continue
            state = build_world_state(
                "",
                pose,
                detections_3d=self._fresh("detections_3d"),
                detections_2d=None,
                lidar=self._fresh("lidar"),
                robot={},
                lidar_band=(
                    self.config.lidar_z_min,
                    self.config.lidar_z_max,
                    self.config.lidar_max_range,
                ),
            )
            self._put("world_state/json", json.dumps({**state, "goal": None}), pose.ts)

    def _drive(self) -> None:
        """Republish the held velocity until the deadman expires, then one zero."""
        active = False
        while not self._stop.wait(1.0 / self.config.drive_hz):
            vx, vy, wz = self._deadman.current()
            if (vx, vy, wz) != (0.0, 0.0, 0.0):
                self.cmd_vel.publish(Twist(linear=(vx, vy, 0.0), angular=(0.0, 0.0, wz)))
                active = True
            elif active:
                self.cmd_vel.publish(Twist())
                active = False
