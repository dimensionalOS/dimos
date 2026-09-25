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

"""What PX4 SITL lacks: the A8 gimbal. ``FakeA8`` stands in for it in ``px4-sitl``.

It consumes ``gimbal_target`` and reports ``gimbal_attitude`` the way the connection does
from the real A8 (10 Hz, radians, follow-mode flags), slewing toward the target at the
A8's rate. On the aircraft the same two ports are the connection's, so SiyiA8Gimbal and
the perception bridge cannot tell the difference. The camera has its own stand-in
(``RtspCamera`` with ``url="synthetic"``), and ``BrightBlobDetector`` finds the square it
draws, so ``px4-sitl-follow`` needs no model.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.gimbal.siyi.replay import AttitudeSample
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.perception.detection.detectors.base import Detector
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


class FakeA8Config(ModuleConfig):
    rate_hz: float = Field(default=10.0)
    # The real A8 slews about 90 deg/s.
    slew_dps: float = Field(default=90.0)
    frame_id: str = Field(default="gimbal_base")
    # Where the fake gimbal points before any request arrives (degrees).
    initial_pitch_deg: float = Field(default=0.0)
    initial_yaw_deg: float = Field(default=0.0)


class FakeA8(Module):
    config: FakeA8Config

    gimbal_target: In[JointState]
    gimbal_attitude: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._pitch = self.config.initial_pitch_deg
        self._yaw = self.config.initial_yaw_deg
        self._target = (self._pitch, self._yaw)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.gimbal_target.subscribe(self._on_target)))
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="fake-a8", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        super().stop()

    def _on_target(self, msg: JointState) -> None:
        angles = dict(zip(msg.name, msg.position, strict=False))
        with self._lock:
            self._target = (
                math.degrees(angles.get("gimbal_pitch", math.radians(self._target[0]))),
                math.degrees(angles.get("gimbal_yaw", math.radians(self._target[1]))),
            )

    def _run(self) -> None:
        period = 1.0 / self.config.rate_hz
        step = self.config.slew_dps * period
        while not self._stop_event.wait(period):
            with self._lock:
                tp, ty = self._target
                self._pitch += max(-step, min(step, tp - self._pitch))
                self._yaw += max(-step, min(step, ty - self._yaw))
                sample = AttitudeSample(time.time(), 0.0, self._pitch, self._yaw)
            self.gimbal_attitude.publish(sample.joint_state(self.config.frame_id))

    @rpc
    def attitude(self) -> dict[str, float]:
        """Current fake pitch and yaw in degrees."""
        with self._lock:
            return {"pitch": self._pitch, "yaw": self._yaw}


class BrightBlobDetector(Detector):
    """Everything brighter than ``threshold`` is one ``person`` with track id 1."""

    def __init__(self, threshold: int = 200, confidence: float = 0.9, min_pixels: int = 16) -> None:
        self._threshold = threshold
        self._confidence = confidence
        self._min_pixels = min_pixels

    def process_image(self, image: Image) -> ImageDetections2D:
        rgb = image.data if image.format is ImageFormat.RGB else image.to_rgb().data
        ys, xs = np.nonzero(rgb.mean(axis=2) > self._threshold)
        if xs.size < self._min_pixels:
            return ImageDetections2D(image)
        box = (float(xs.min()), float(ys.min()), float(xs.max()) + 1.0, float(ys.max()) + 1.0)
        return ImageDetections2D(
            image, [Detection2DBBox(box, 1, 0, self._confidence, "person", image.ts, image)]
        )
