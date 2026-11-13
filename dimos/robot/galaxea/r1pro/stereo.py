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

"""Factory-calibrated head stereo and rectified wrist RGB-D for spatial teleop."""

from pathlib import Path
from typing import Any

import cv2
import numpy as np

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo


class HeadStereo:
    """Rectify both eyes at reduced resolution before metric SGBM depth."""

    def __init__(self, calibration: str | Path, width: int = 480) -> None:
        storage = cv2.FileStorage(str(calibration), cv2.FILE_STORAGE_READ)
        if not storage.isOpened():
            raise FileNotFoundError(calibration)
        try:
            original = (
                int(storage.getNode("image_width").real()),
                int(storage.getNode("image_height").real()),
            )
            self.size = (width, round(original[1] * width / original[0]))
            scale = width / original[0]
            k1, k2 = [
                np.asarray(storage.getNode(f"K_{s}").mat(), dtype=np.float64)
                for s in ("left", "right")
            ]
            d1, d2 = [storage.getNode(f"D_{s}").mat() for s in ("left", "right")]
            rotation, translation = [storage.getNode(s).mat() for s in ("R", "T")]
        finally:
            storage.release()
        k1[:2] *= scale
        k2[:2] *= scale
        r1, r2, p1, p2, _, roi1, roi2 = cv2.stereoRectify(
            k1,
            d1,
            k2,
            d2,
            self.size,
            rotation,
            translation,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0,
        )
        self.rotation = r1.T  # rectified optical -> raw left optical
        self.maps = [
            cv2.initUndistortRectifyMap(k, d, r, p[:3, :3], self.size, cv2.CV_32FC1)
            for k, d, r, p in ((k1, d1, r1, p1), (k2, d2, r2, p2))
        ]
        self.fx_baseline = abs(float(p2[0, 3]))
        self.info = CameraInfo.from_intrinsics(
            p1[0, 0], p1[1, 1], p1[0, 2], p1[1, 2], *self.size, frame_id="head_stereo_optical"
        )
        self.matcher = cv2.StereoSGBM.create(
            minDisparity=0,
            numDisparities=96,
            blockSize=5,
            P1=8 * 25,
            P2=32 * 25,
            disp12MaxDiff=1,
            uniquenessRatio=12,
            speckleWindowSize=100,
            speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        self.roi = cv2.getValidDisparityROI(roi1, roi2, 0, 96, 5)

    def process(self, left: np.ndarray, right: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        rectified = [
            cv2.remap(
                cv2.resize(im, self.size, interpolation=cv2.INTER_AREA), *maps, cv2.INTER_LINEAR
            )
            for im, maps in zip((left, right), self.maps, strict=True)
        ]
        gray = [cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) for im in rectified]
        disparity = self.matcher.compute(*gray).astype(np.float32) / 16
        depth = disparity_depth(disparity, self.fx_baseline)
        x, y, w, h = self.roi
        mask = np.zeros(depth.shape, dtype=bool)
        mask[y : y + h, x : x + w] = True
        depth[~mask] = 0
        return rectified[0], depth


def disparity_depth(disparity: np.ndarray, fx_baseline: float) -> np.ndarray:
    """Invalid/unreliable disparities stay holes, never infinite triangles."""
    depth = np.zeros(disparity.shape, dtype=np.float32)
    np.divide(fx_baseline, disparity, out=depth, where=disparity > 0)
    depth[(depth < 0.15) | (depth > 8) | ~np.isfinite(depth)] = 0
    return depth


def rectify_wrist(
    color: np.ndarray, depth: np.ndarray, calibration: Any
) -> tuple[np.ndarray, np.ndarray, CameraInfo]:
    """Aligned depth shares distorted color pixels; rectify both with the same map."""
    k = np.asarray(calibration.k).reshape(3, 3)
    size = (calibration.width, calibration.height)
    maps = cv2.initUndistortRectifyMap(
        k, np.asarray(calibration.d), np.eye(3), k, size, cv2.CV_32FC1
    )
    info = CameraInfo.from_intrinsics(k[0, 0], k[1, 1], k[0, 2], k[1, 2], *size)
    return (
        cv2.remap(color, *maps, cv2.INTER_LINEAR),
        cv2.remap(depth, *maps, cv2.INTER_NEAREST),
        info,
    )
