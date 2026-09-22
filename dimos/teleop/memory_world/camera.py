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

"""The recording's camera: its intrinsics, and frames rectified to a pinhole."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from typing import Any, Literal

import cv2
import numpy as np

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo


@dataclass(frozen=True)
class CameraModel:
    """Intrinsics at the recorded resolution."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple[float, ...]
    model: Literal["equidistant", "plumb_bob"] = "equidistant"

    @classmethod
    def from_camera_info(cls, info: Any) -> CameraModel:
        k = [float(v) for v in info.K]
        model = "equidistant" if str(info.distortion_model) == "equidistant" else "plumb_bob"
        return cls(
            width=int(info.width),
            height=int(info.height),
            fx=k[0],
            fy=k[4],
            cx=k[2],
            cy=k[5],
            distortion=tuple(float(v) for v in info.D),
            model=model,
        )

    def fingerprint(self) -> str:
        blob = json.dumps(self.__dict__, sort_keys=True).encode()
        return hashlib.sha1(blob).hexdigest()[:12]

    def matrix(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], dtype=np.float64)

    def pinhole_info(self, frame_id: str = "") -> CameraInfo:
        """The same focal length and center with no distortion, for rectified frames."""
        return CameraInfo.from_intrinsics(
            self.fx, self.fy, self.cx, self.cy, self.width, self.height, frame_id=frame_id
        )


class Rectifier:
    """Undistorts frames so the pinhole model of ``camera`` holds exactly."""

    def __init__(self, camera: CameraModel) -> None:
        self.camera = camera
        self._maps: tuple[np.ndarray, np.ndarray] | None = None
        if any(camera.distortion):
            k = camera.matrix()
            d = np.asarray(camera.distortion, dtype=np.float64)
            size = (camera.width, camera.height)
            if camera.model == "equidistant":
                self._maps = cv2.fisheye.initUndistortRectifyMap(
                    k, d, np.eye(3), k, size, cv2.CV_16SC2
                )
            else:
                self._maps = cv2.initUndistortRectifyMap(k, d, np.eye(3), k, size, cv2.CV_16SC2)

    def __call__(self, image_bgr: np.ndarray) -> np.ndarray:
        if self._maps is None:
            return image_bgr
        return cv2.remap(image_bgr, self._maps[0], self._maps[1], cv2.INTER_LINEAR)
