# Copyright 2025-2026 Dimensional Inc.
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

"""Bounded stereo pairing and metric RGB-D, with no robot control or base-frame TF."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any, Literal

import cv2
import numpy as np
from pydantic import Field, model_validator
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.sensors.camera.stereo.backends import OpenCvStereo, StereoBackend, VpiCudaStereo
from dimos.hardware.sensors.camera.stereo.calibration import StereoCalibration, metric_depth
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class StereoDepthConfig(ModuleConfig):
    calibration_path: str
    left_frame: str
    right_frame: str
    frame_id: str = "stereo_left_rectified_optical"
    width: int = Field(default=960, gt=0)
    height: int = Field(default=768, gt=0)
    backend: Literal["vpi_cuda", "opencv_sgbm"] = "vpi_cuda"
    translation_scale_to_m: float = Field(default=1.0, gt=0, allow_inf_nan=False)
    max_disparity: int = Field(default=256, ge=16, le=256)
    confidence_threshold: int = Field(default=32767, ge=0, le=65535)
    min_depth_m: float = Field(default=0.3, gt=0, allow_inf_nan=False)
    max_depth_m: float = Field(default=3.0, gt=0, allow_inf_nan=False)
    max_pair_skew_s: float = Field(default=0.002, gt=0, allow_inf_nan=False)
    max_age_s: float = Field(default=0.5, gt=0, allow_inf_nan=False)
    max_hz: float = Field(default=5.0, gt=0, allow_inf_nan=False)

    @model_validator(mode="after")
    def _validate_geometry(self) -> StereoDepthConfig:
        if self.max_disparity % 16 or self.width <= self.max_disparity:
            raise ValueError("width must exceed max_disparity, which must be a multiple of 16")
        if self.max_depth_m <= self.min_depth_m:
            raise ValueError("max_depth_m must exceed min_depth_m")
        if (
            not self.left_frame
            or not self.right_frame
            or self.left_frame == self.right_frame
            or not self.frame_id
            or self.frame_id in (self.left_frame, self.right_frame)
        ):
            raise ValueError(
                "non-empty distinct left/right frames and an output frame are required"
            )
        return self


@dataclass(frozen=True)
class StereoPair:
    left: CompressedImage
    right: CompressedImage


class StereoPairer:
    """One pending image per eye; never reuse a frame or pair across capture skew."""

    def __init__(self, left_frame: str, right_frame: str, max_skew: float) -> None:
        self._frames = {"left": left_frame, "right": right_frame}
        self._skew = max_skew
        self._pending: dict[str, CompressedImage] = {}
        self._last: dict[str, float] = {"left": -math.inf, "right": -math.inf}
        self.dropped = 0

    def push(self, side: str, image: CompressedImage) -> StereoPair | None:
        if (
            image.frame_id != self._frames[side]
            or not math.isfinite(image.ts)
            or image.ts <= self._last[side]
        ):
            self.dropped += 1
            return None
        self._last[side] = image.ts
        if side in self._pending:
            self.dropped += 1
        self._pending[side] = image
        if len(self._pending) != 2:
            return None
        left, right = self._pending["left"], self._pending["right"]
        if abs(left.ts - right.ts) > self._skew:
            self._pending.pop("left" if left.ts < right.ts else "right")
            self.dropped += 1
            return None
        self._pending.clear()
        return StereoPair(left, right)


class StereoDepth(Module):
    config: StereoDepthConfig
    left: In[CompressedImage]
    right: In[CompressedImage]
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    confidence: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._condition = threading.Condition()
        self._stop = False
        self._thread: threading.Thread | None = None
        self._pairer = StereoPairer(
            self.config.left_frame, self.config.right_frame, self.config.max_pair_skew_s
        )
        self._pending: StereoPair | None = None
        self._calibration: StereoCalibration | None = None
        self._backend: StereoBackend | None = None
        self._processed = 0
        self._replaced = 0
        self._expired = 0
        self._last_ts: float | None = None
        self._last_skew: float | None = None
        self._valid_fraction: float | None = None
        self._latest_depth: Image | None = None
        self._error: str | None = None

    @rpc
    def start(self) -> None:
        """Initialize only stereo processing. The blueprint's camera modules own capture."""
        if self._thread is not None:
            return
        calibration = StereoCalibration.load(
            self.config.calibration_path,
            (self.config.width, self.config.height),
            self.config.translation_scale_to_m,
        )
        backend: StereoBackend = (
            VpiCudaStereo(self.config.max_disparity, self.config.confidence_threshold)
            if self.config.backend == "vpi_cuda"
            else OpenCvStereo(self.config.max_disparity)
        )
        super().start()
        with self._condition:
            self._calibration = calibration
            self._backend = backend
            self._stop = False
            self._pairer = StereoPairer(
                self.config.left_frame, self.config.right_frame, self.config.max_pair_skew_s
            )
            self._pending = None
        self.register_disposable(
            Disposable(self.left.subscribe(lambda image: self._receive("left", image)))
        )
        self.register_disposable(
            Disposable(self.right.subscribe(lambda image: self._receive("right", image)))
        )
        self._thread = threading.Thread(target=self._run, name="StereoDepth", daemon=True)
        self._thread.start()

    def _receive(self, side: str, image: CompressedImage) -> None:
        with self._condition:
            if self._stop:
                return
            pair = self._pairer.push(side, image)
            if pair is not None:
                if self._pending is not None:
                    self._replaced += 1
                self._pending = pair
                self._condition.notify_all()

    def _run(self) -> None:
        next_due = 0.0
        while True:
            with self._condition:
                while not self._stop:
                    delay = max(0.0, next_due - time.monotonic())
                    if self._pending is not None and delay == 0:
                        break
                    self._condition.wait(timeout=delay if self._pending is not None else None)
                if self._stop:
                    return
                pair, self._pending = self._pending, None
            assert pair is not None
            next_due = time.monotonic() + 1.0 / self.config.max_hz
            try:
                self._process_pair(pair)
            except Exception as error:
                with self._condition:
                    self._error = str(error)
                logger.exception("Stereo depth pair failed")

    def _process_pair(self, pair: StereoPair) -> bool:
        assert self._calibration is not None and self._backend is not None
        stamp = pair.left.ts
        age = time.time() - min(pair.left.ts, pair.right.ts)
        if age < 0 or age > self.config.max_age_s:
            with self._condition:
                self._expired += 1
            return False
        left, right = self._calibration.rectify(
            pair.left.decode().to_rgb().data, pair.right.decode().to_rgb().data
        )
        disparity, confidence = self._backend.estimate(
            np.asarray(cv2.cvtColor(left, cv2.COLOR_RGB2GRAY), dtype=np.uint8),
            np.asarray(cv2.cvtColor(right, cv2.COLOR_RGB2GRAY), dtype=np.uint8),
        )
        if (
            disparity.shape != (self.config.height, self.config.width)
            or confidence.shape != disparity.shape
        ):
            raise ValueError("stereo backend output does not match rectified image size")
        depth = metric_depth(
            disparity,
            self._calibration.q,
            confidence,
            self.config.confidence_threshold,
            self.config.min_depth_m,
            self.config.max_depth_m,
        )
        with self._condition:
            final_age = time.time() - min(pair.left.ts, pair.right.ts)
            if self._stop or final_age < 0 or final_age > self.config.max_age_s:
                self._expired += 1
                return False
            frame = self.config.frame_id
            self.color_image.publish(Image(left, ImageFormat.RGB, frame_id=frame, ts=stamp))
            self.depth_image.publish(Image(depth, ImageFormat.DEPTH, frame_id=frame, ts=stamp))
            self.camera_info.publish(self._calibration.camera_info(frame, stamp))
            self.confidence.publish(Image(confidence, ImageFormat.GRAY16, frame_id=frame, ts=stamp))
            self._processed += 1
            self._last_ts = stamp
            self._last_skew = abs(pair.left.ts - pair.right.ts)
            self._valid_fraction = float(np.mean(depth > 0))
            self._latest_depth = Image(depth, ImageFormat.DEPTH, frame_id=frame, ts=stamp)
            self._error = None
        return True

    @rpc
    def get_status(self) -> dict[str, Any]:
        """Read processing health. No base-frame transform or grasp accuracy is implied."""
        with self._condition:
            return {
                "backend": self.config.backend,
                "width": self.config.width,
                "height": self.config.height,
                "running": self._thread is not None and self._thread.is_alive(),
                "last_pair_skew_s": self._last_skew,
                "valid_depth_fraction": self._valid_fraction,
                "processed_pairs": self._processed,
                "dropped_pairs": self._replaced + self._expired,
                "backpressure_replacements": self._replaced,
                "expired_pairs": self._expired,
                "dropped_images": self._pairer.dropped,
                "last_capture_ts": self._last_ts,
                "age_s": None if self._last_ts is None else time.time() - self._last_ts,
                "error": self._error,
                "frame_id": self.config.frame_id,
                "baseline_m": None if self._calibration is None else self._calibration.baseline_m,
                "robot_mount_verified": False,
            }

    @rpc
    def sample_depth(self, u: int, v: int) -> dict[str, Any]:
        """Median valid depth in a 5x5 RECTIFIED pixel patch; never a robot-base pose."""
        if not 0 <= u < self.config.width or not 0 <= v < self.config.height:
            raise ValueError("pixel lies outside rectified image")
        with self._condition:
            image = self._latest_depth
            if image is None or not 0 <= time.time() - image.ts <= self.config.max_age_s:
                return {"available": False, "depth_m": None, "reason": "no fresh depth"}
            patch = image.data[max(0, v - 2) : v + 3, max(0, u - 2) : u + 3]
            valid = patch[patch > 0]
            return {
                "available": bool(valid.size),
                "depth_m": float(np.median(valid)) if valid.size else None,
                "valid_pixels": int(valid.size),
                "capture_ts": image.ts,
                "frame_id": image.frame_id,
            }

    @rpc
    def stop(self) -> None:
        with self._condition:
            self._stop = True
            self._pending = None
            self._latest_depth = None
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=self.config.max_age_s + 2.0)
            if self._thread.is_alive():
                raise RuntimeError("Stereo worker did not stop; do not restart it")
            self._thread = None
        self._backend = None
        super().stop()
