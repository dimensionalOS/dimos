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

"""Calibrated RGB-D from the existing head capture and vendor wrist drivers.

Adapted from the r1pro-spatial stack (ef8a0548fa). Head cameras have one owner:
this module consumes their compressed streams rather than reopening V4L2.
"""

from collections import deque
from collections.abc import Callable
import importlib
import threading
import time
from typing import Any, ClassVar

import cv2
import numpy as np
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.galaxea.r1pro.connection import _make_qos, _stamp_secs
from dimos.robot.galaxea.r1pro.stereo import HeadStereo, rectify_wrist
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class R1SpatialSensorsConfig(ModuleConfig):
    calibration_path: str = "/opt/galaxea/body/stereo.yaml"
    publish_hz: float = Field(default=5.0, gt=0, le=15)
    stereo_width: int = 480
    max_pair_skew_s: float = 0.02
    max_age_s: float = 0.5
    enable_wrists: bool = True


class R1SpatialSensors(Module):
    config: R1SpatialSensorsConfig
    dedicated_worker: ClassVar[bool] = True
    head_left_color: In[CompressedImage]
    head_right_color: In[CompressedImage]
    color_image: Out[CompressedImage]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    left_wrist: Out[CompressedImage]
    left_wrist_depth: Out[Image]
    left_wrist_info: Out[CameraInfo]
    right_wrist: Out[CompressedImage]
    right_wrist_depth: Out[Image]
    right_wrist_info: Out[CameraInfo]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._ros: Any = None
        self._unsubs: list[Callable[[], None]] = []
        self._lock = threading.Lock()
        self._frames: dict[str, deque[Any]] = {}
        self._infos: dict[str, Any] = {}
        self._last: dict[str, float] = {}
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._stereo: HeadStereo | None = None
        self._stats: dict[str, int] = {"head": 0, "left": 0, "right": 0, "errors": 0}
        self._published: dict[str, float] = {}

    @rpc
    def start(self) -> None:
        super().start()
        cv2.setNumThreads(2)
        self._stereo = HeadStereo(self.config.calibration_path, self.config.stereo_width)
        if self.config.enable_wrists:
            self._start_wrists()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _start_wrists(self) -> None:
        # ROS is optional at package import; the hardware launcher supplies its environment.
        ros = importlib.import_module("dimos.protocol.pubsub.impl.rospubsub")
        msgs = importlib.import_module("sensor_msgs.msg")
        self._ros = ros.RawROS(node_name="r1_spatial_cameras")
        self._ros.start()
        topics = []
        for side in ("left", "right"):
            base = f"/hdas/camera_wrist_{side}"
            topics += [
                (side + "_color", base + "/color/image_raw/compressed", msgs.CompressedImage),
                (side + "_depth", base + "/aligned_depth_to_color/image_raw", msgs.Image),
                (side + "_info", base + "/color/camera_info", msgs.CameraInfo),
            ]
        for key, topic, kind in topics:
            self._unsubs.append(
                self._ros.subscribe(
                    ros.RawROSTopic(topic, kind, qos=_make_qos()),
                    lambda msg, _, key=key: self._receive(key, msg),
                )
            )

    async def handle_head_left_color(self, image: CompressedImage) -> None:
        self._receive("head_left", image)

    async def handle_head_right_color(self, image: CompressedImage) -> None:
        self._receive("head_right", image)

    @staticmethod
    def _stamp(msg: Any) -> float:
        return msg.ts if isinstance(msg, CompressedImage) else _stamp_secs(msg)

    def _receive(self, key: str, msg: Any) -> None:
        with self._lock:
            if key.endswith("_info"):
                self._infos[key] = msg
            else:
                self._frames.setdefault(key, deque(maxlen=6)).append(msg)

    def _pair(self, a: str, b: str) -> tuple[Any, Any] | None:
        with self._lock:
            pairs = [
                (x, y)
                for x in self._frames.get(a, [])
                for y in self._frames.get(b, [])
                if abs(self._stamp(x) - self._stamp(y)) <= self.config.max_pair_skew_s
                and self._stamp(x) > self._last.get(a, 0)
                and self._stamp(y) > self._last.get(b, 0)
            ]
            if not pairs:
                return None
            x, y = max(pairs, key=lambda pair: self._stamp(pair[0]))
            self._last[a], self._last[b] = self._stamp(x), self._stamp(y)
            if not -0.05 <= time.time() - self._stamp(x) <= self.config.max_age_s:
                return None
            return x, y

    @staticmethod
    def _color(msg: Any, *, reduced: bool = False) -> np.ndarray:
        # Head stereo is processed at quarter resolution: downsample in the
        # JPEG decoder instead of decoding 3 MP only to discard 15/16 pixels.
        mode = cv2.IMREAD_REDUCED_COLOR_4 if reduced else cv2.IMREAD_COLOR
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), mode)
        if image is None:
            raise ValueError("Invalid camera JPEG")
        return image

    def capture(self) -> None:
        assert self._stereo is not None
        pair = self._pair("head_left", "head_right")
        if pair:
            ts = self._stamp(pair[0])
            matrix = np.eye(4)
            matrix[:3, :3] = self._stereo.rotation
            self.tf.publish(
                TFMessage(
                    Transform.from_matrix(
                        matrix,
                        frame_id="camera_head_left_link",
                        child_frame_id="head_stereo_optical",
                        ts=ts,
                    )
                )
            )
            color, depth = self._stereo.process(*(self._color(msg, reduced=True) for msg in pair))
            published = self._publish(
                (self.color_image, self.depth_image, self.camera_info),
                color,
                depth,
                self._stereo.info,
                "head_stereo_optical",
                ts,
            )
            self._stats["head"] += int(published)
        for side in ("left", "right"):
            with self._lock:
                info = self._infos.get(side + "_info")
            if info is None:
                continue
            pair = self._pair(side + "_color", side + "_depth")
            if pair is None:
                continue
            rgb, raw = pair
            if raw.encoding != "16UC1":
                raise ValueError(f"Expected RealSense millimetre depth, got {raw.encoding}")
            # Respect ROS row padding and byte order. RealSense ROS converts device units to mm.
            dtype = np.dtype(">u2" if raw.is_bigendian else "<u2")
            depth = (
                np.ndarray(
                    (raw.height, raw.width),
                    dtype=dtype,
                    buffer=bytes(raw.data),
                    strides=(raw.step, 2),
                ).astype(np.float32)
                * 0.001
            )
            color = self._color(rgb)
            if color.shape[:2] != depth.shape or depth.shape != (info.height, info.width):
                raise ValueError("Wrist color/depth/calibration dimensions differ")
            color, depth, calibration = rectify_wrist(color, depth, info)
            ports = (
                (self.left_wrist, self.left_wrist_depth, self.left_wrist_info)
                if side == "left"
                else (self.right_wrist, self.right_wrist_depth, self.right_wrist_info)
            )
            published = self._publish(
                ports, color, depth, calibration, f"{side}_d405_link", _stamp_secs(rgb)
            )
            self._stats[side] += int(published)

    def _publish(
        self,
        ports: tuple[Out[CompressedImage], Out[Image], Out[CameraInfo]],
        color: np.ndarray,
        depth: np.ndarray,
        info: CameraInfo,
        frame: str,
        ts: float,
    ) -> bool:
        if time.time() - ts > self.config.max_age_s:
            return False
        info.frame_id, info.ts = frame, ts
        ports[2].publish(info)
        ports[0].publish(
            CompressedImage.from_image(Image.from_numpy(color, ImageFormat.BGR, frame, ts))
        )
        ports[1].publish(Image.from_numpy(depth, ImageFormat.DEPTH, frame, ts))
        with self._lock:
            self._published[frame] = ts
        return True

    def _run(self) -> None:
        period = 1 / self.config.publish_hz
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                self.capture()
            except (ValueError, RuntimeError, cv2.error):
                self._stats["errors"] += 1
                logger.exception("R1 spatial camera capture failed")
            # Processing is part of the period, not an extra delay on top of it.
            self._stop.wait(max(0.0, period - (time.monotonic() - started)))

    @rpc
    def sensor_stats(self) -> dict[str, int]:
        return dict(self._stats)

    @rpc
    def get_status(self) -> dict[str, Any]:
        """Read camera output counts and capture ages without changing acquisition."""
        with self._lock:
            published = dict(self._published)
        return {
            **self._stats,
            "age_s": {frame: time.time() - stamp for frame, stamp in published.items()},
            "waiting_for": [side for side in ("head", "left", "right") if not self._stats[side]],
        }

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3)
            if self._thread.is_alive():
                raise RuntimeError("RGB-D capture thread did not stop")
            self._thread = None
        for unsub in self._unsubs:
            unsub()
        self._unsubs.clear()
        if self._ros is not None:
            self._ros.stop()
            self._ros = None
        super().stop()


class R1SpatialCloudConfig(ModuleConfig):
    fixed_frame: str = "odom"


class R1SpatialCloud(Module):
    """Put Point-LIO's sensor-local cloud into the spatial viewer's world frame."""

    config: R1SpatialCloudConfig
    lidar: In[PointCloud2]
    pointcloud: Out[PointCloud2]
    tf: IO[TFMessage]

    @rpc
    def start(self) -> None:
        self.tfbuffer  # noqa: B018
        super().start()

    async def handle_lidar(self, cloud: PointCloud2) -> None:
        transform = self.tfbuffer.get(
            self.config.fixed_frame, cloud.frame_id, cloud.ts, 0.2, warn=False
        )
        if transform is not None:
            self.pointcloud.publish(cloud.transform(transform))
