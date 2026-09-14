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
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import io
import json
import threading
import time
from typing import Any

import numpy as np
from PIL import Image as PILImage
import zenoh

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

RAW_ENDPOINT = "tcp/127.0.0.1:7448"


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
        self, endpoint: str = RAW_ENDPOINT, prefix: str = "robot", *, listen: bool
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


@dataclass
class Deadman:
    """The latest velocity command, valid until its deadline passes."""

    max_s: float = 2.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    until: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def set(self, command: bytes | str) -> None:
        c = json.loads(command)
        hold = min(float(c.get("t", 0.0)), self.max_s)
        with self.lock:
            self.vx, self.vy, self.wz = (
                float(c.get("vx", 0)),
                float(c.get("vy", 0)),
                float(c.get("wz", 0)),
            )
            self.until = time.monotonic() + max(0.0, hold)

    def current(self) -> tuple[float, float, float]:
        with self.lock:
            return (self.vx, self.vy, self.wz) if time.monotonic() < self.until else (0.0, 0.0, 0.0)


class RawRobotBridgeConfig(ModuleConfig):
    endpoint: str = RAW_ENDPOINT
    prefix: str = "robot"
    max_cmd_s: float = 2.0
    jpeg_quality: int = 90
    drive_hz: float = 10.0


class RawRobotBridge(Module):
    """Vendor-shaped topics from the connection's streams; JSON velocity commands back in."""

    config: RawRobotBridgeConfig

    color_image: In[Image]
    lidar: In[PointCloud2]
    odom: In[PoseStamped]
    camera_info: In[CameraInfo]
    cmd_vel: Out[Twist]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._topics: RawTopics | None = None
        self._deadman = Deadman(max_s=self.config.max_cmd_s)
        self._stop = threading.Event()

    def start(self) -> None:
        super().start()
        self._topics = RawTopics(self.config.endpoint, self.config.prefix, listen=True)
        q = self.config.jpeg_quality
        self.color_image.subscribe(lambda img: self._put("camera/jpeg", jpeg_bytes(img, q), img.ts))
        self.lidar.subscribe(lambda cloud: self._put("lidar/xyz_f32", xyz_f32(cloud), cloud.ts))
        self.odom.subscribe(lambda pose: self._put("odom/json", odom_json(pose)))
        self.camera_info.subscribe(
            lambda info: self._put(
                "camera_info/json",
                json.dumps({"width": info.width, "height": info.height, "K": info.K}),
            )
        )
        self._cmd_sub = self._topics.subscribe("cmd_vel/json", self._on_command)
        threading.Thread(target=self._drive, daemon=True, name="raw-robot-drive").start()

    def stop(self) -> None:
        self._stop.set()
        self.cmd_vel.publish(Twist())
        if self._topics is not None:
            self._topics.close()
        super().stop()

    def _put(self, key: str, payload: bytes | str, ts: float | None = None) -> None:
        if self._topics is not None:
            self._topics.put(key, payload, ts)

    def _on_command(self, payload: bytes, _ts: float | None) -> None:
        try:
            self._deadman.set(payload)
        except (ValueError, TypeError, AttributeError):
            return  # a malformed packet is dropped, as an SDK would

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
