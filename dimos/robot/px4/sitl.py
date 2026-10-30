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

"""What PX4 SITL lacks: the A8. ``FakeA8`` and ``FakeA8Camera`` stand in for it in
``px4-sitl-follow``.

``FakeA8`` consumes ``gimbal_target`` and reports ``gimbal_attitude`` the way the connection
does from the real A8 (10 Hz, radians, follow-mode flags), slewing toward the target at the
A8's rate. On the aircraft the same two ports are the connection's, so SiyiA8Gimbal and
the perception bridge cannot tell the difference. ``FakeA8Camera`` publishes
``color_image`` in place of ``RtspCamera``: a white square in the middle of a grey frame,
which ``BrightBlobDetector`` finds, so ``px4-sitl-follow`` needs no model.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray
from pydantic import Field
import reactivex as rx
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.gimbal.siyi.frame import (
    GIMBAL_TARGET_JOINTS,
    GimbalDeviceFlags,
    attitude_msg,
    joints,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.perception.detection.detectors.base import Detector
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.spec import perception

# The real A8: attitude at 10 Hz, slewing about 90 deg/s, in follow mode.
_RATE_HZ = 10.0
_SLEW_DPS = 90.0
_A8_FLAGS = GimbalDeviceFlags.YAW_LOCK | GimbalDeviceFlags.YAW_IN_VEHICLE_FRAME
# BrightBlobDetector: a pixel brighter than this, and at least this many of them.
_BLOB_THRESHOLD = 200
_BLOB_MIN_PIXELS = 16
# FakeA8Camera's target: a white square this many pixels wide on grey.
_TARGET_PX = 24


def target_frame(width: int, height: int) -> NDArray[np.uint8]:
    """FakeA8Camera's picture: a white square in the middle of a grey RGB frame."""
    frame = np.full((height, width, 3), 96, dtype=np.uint8)
    x, y = (width - _TARGET_PX) // 2, (height - _TARGET_PX) // 2
    frame[y : y + _TARGET_PX, x : x + _TARGET_PX] = 255
    return frame


class FakeA8Config(ModuleConfig):
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

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.gimbal_target.subscribe(self._on_target)))
        self.register_disposable(rx.interval(1.0 / _RATE_HZ).subscribe(lambda _: self._step()))

    def _on_target(self, msg: JointState) -> None:
        angles = joints(msg, GIMBAL_TARGET_JOINTS)
        if angles is not None:
            with self._lock:
                self._target = (math.degrees(angles[0]), math.degrees(angles[1]))

    def _step(self) -> None:
        step = _SLEW_DPS / _RATE_HZ
        with self._lock:
            tp, ty = self._target
            self._pitch += max(-step, min(step, tp - self._pitch))
            self._yaw += max(-step, min(step, ty - self._yaw))
            pitch, yaw = math.radians(self._pitch), math.radians(self._yaw)
        self.gimbal_attitude.publish(attitude_msg(time.time(), 0.0, pitch, yaw, _A8_FLAGS))


class FakeA8CameraConfig(ModuleConfig):
    frame_id: str = Field(default="a8_optical")
    # The follow blueprint pins the CameraInfo of this size.
    image_size: tuple[int, int] = Field(default=(320, 180))
    # RtspCamera's color_hz.
    fps: float = Field(default=10.0, gt=0)


class FakeA8Camera(Module, perception.Image):
    config: FakeA8CameraConfig

    color_image: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._frame = target_frame(*self.config.image_size)

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            rx.interval(1.0 / self.config.fps).subscribe(lambda _: self._publish())
        )

    def _publish(self) -> None:
        self.color_image.publish(
            Image(
                data=self._frame,
                format=ImageFormat.RGB,
                frame_id=self.config.frame_id,
                ts=time.time(),
            )
        )


class BrightBlobDetector(Detector):
    """Everything bright is one ``person`` with track id 1."""

    def process_image(self, image: Image) -> ImageDetections2D:
        rgb = image.data if image.format is ImageFormat.RGB else image.to_rgb().data
        ys, xs = np.nonzero(rgb.mean(axis=2) > _BLOB_THRESHOLD)
        if xs.size < _BLOB_MIN_PIXELS:
            return ImageDetections2D(image)
        box = (float(xs.min()), float(ys.min()), float(xs.max()) + 1.0, float(ys.max()) + 1.0)
        return ImageDetections2D(
            image, [Detection2DBBox(box, 1, 0, 0.9, "person", image.ts, image)]
        )
