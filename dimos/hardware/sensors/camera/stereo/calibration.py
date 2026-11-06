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

"""Metric horizontal stereo geometry, independent of camera capture and robot TF."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from numpy.typing import NDArray

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo


@dataclass(frozen=True)
class StereoCalibration:
    input_size: tuple[int, int]
    output_size: tuple[int, int]
    left_maps: tuple[NDArray[np.float32], NDArray[np.float32]]
    right_maps: tuple[NDArray[np.float32], NDArray[np.float32]]
    projection: NDArray[np.float64]
    q: NDArray[np.float64]
    rotation: NDArray[np.float64]
    baseline_m: float

    @classmethod
    def load(
        cls, path: str | Path, output_size: tuple[int, int], translation_scale_to_m: float = 1.0
    ) -> StereoCalibration:
        if not np.isfinite(translation_scale_to_m) or translation_scale_to_m <= 0:
            raise ValueError("translation_scale_to_m must be finite and positive")
        if min(output_size) < 1:
            raise ValueError("output size must be positive")
        f = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        if not f.isOpened():
            raise FileNotFoundError(path)
        try:
            size = (int(f.getNode("image_width").real()), int(f.getNode("image_height").real()))
            if min(size) < 1:
                raise ValueError("calibration has no image dimensions")

            def matrix(key: str) -> NDArray[np.float64]:
                value = f.getNode(key).mat()
                if value is None or not np.isfinite(value).all():
                    raise ValueError(f"missing or non-finite calibration matrix: {key}")
                return np.asarray(value, dtype=np.float64)

            kl, kr, dl, dr = [matrix(k) for k in ("K_left", "K_right", "D_left", "D_right")]
            rotation, translation = matrix("R"), matrix("T").reshape(-1) * translation_scale_to_m
        finally:
            f.release()
        if (
            kl.shape != (3, 3)
            or kr.shape != (3, 3)
            or rotation.shape != (3, 3)
            or translation.shape != (3,)
        ):
            raise ValueError("invalid stereo matrix dimensions")
        if min(kl[0, 0], kl[1, 1], kr[0, 0], kr[1, 1]) <= 0:
            raise ValueError("focal lengths must be positive")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3) or not np.isclose(
            np.linalg.det(rotation), 1.0, atol=1e-3
        ):
            raise ValueError("stereo R must be a proper rotation")
        if translation[0] >= 0 or abs(translation[0]) <= abs(translation[1]):
            raise ValueError("expected horizontal left-to-right baseline with negative Tx")
        baseline = float(np.linalg.norm(translation))
        r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(
            kl,
            dl,
            kr,
            dr,
            size,
            rotation,
            translation,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0,
            newImageSize=output_size,
        )
        left = cv2.initUndistortRectifyMap(kl, dl, r1, p1, output_size, cv2.CV_32FC1)
        right = cv2.initUndistortRectifyMap(kr, dr, r2, p2, output_size, cv2.CV_32FC1)
        return cls(
            size,
            output_size,
            (np.asarray(left[0], dtype=np.float32), np.asarray(left[1], dtype=np.float32)),
            (np.asarray(right[0], dtype=np.float32), np.asarray(right[1], dtype=np.float32)),
            np.asarray(p1, dtype=np.float64),
            np.asarray(q, dtype=np.float64),
            np.asarray(r1, dtype=np.float64),
            baseline,
        )

    def rectify(
        self, left: NDArray[np.uint8], right: NDArray[np.uint8]
    ) -> tuple[NDArray[np.uint8], NDArray[np.uint8]]:
        expected = (self.input_size[1], self.input_size[0], 3)
        if (
            left.shape != expected
            or right.shape != expected
            or left.dtype != np.uint8
            or right.dtype != np.uint8
        ):
            raise ValueError(f"decoded RGB frames must match calibration shape {expected}")
        return (
            np.asarray(cv2.remap(left, *self.left_maps, cv2.INTER_LINEAR), dtype=np.uint8),
            np.asarray(cv2.remap(right, *self.right_maps, cv2.INTER_LINEAR), dtype=np.uint8),
        )

    def camera_info(self, frame_id: str, ts: float) -> CameraInfo:
        return CameraInfo(
            width=self.output_size[0],
            height=self.output_size[1],
            K=self.projection[:, :3].ravel().tolist(),
            D=[0.0] * 5,
            R=np.eye(3).ravel().tolist(),
            P=self.projection.ravel().tolist(),
            distortion_model="plumb_bob",
            frame_id=frame_id,
            ts=ts,
        )


def metric_depth(
    disparity: NDArray[np.float32],
    q: NDArray[np.float64],
    confidence: NDArray[np.uint16],
    threshold: int,
    min_depth: float,
    max_depth: float,
) -> NDArray[np.float32]:
    """Reproject disparity in PIXELS; invalid or uncertain depth is zero, never filled."""
    if disparity.ndim != 2 or confidence.shape != disparity.shape:
        raise ValueError("disparity and confidence must have matching 2D shapes")
    with np.errstate(divide="ignore", invalid="ignore"):
        depth = q[2, 3] / (q[3, 2] * disparity + q[3, 3])
    valid = (
        np.isfinite(disparity)
        & (disparity > 0)
        & np.isfinite(depth)
        & (depth >= min_depth)
        & (depth <= max_depth)
        & (confidence >= threshold)
    )
    return np.where(valid, depth, 0.0).astype(np.float32)
