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

"""Stereo matching backends; importing this module never initializes CUDA."""

from __future__ import annotations

import importlib
from typing import Any, Protocol

import cv2
import numpy as np
from numpy.typing import NDArray


class StereoBackend(Protocol):
    def estimate(
        self, left: NDArray[np.uint8], right: NDArray[np.uint8]
    ) -> tuple[NDArray[np.float32], NDArray[np.uint16]]: ...


class VpiCudaStereo:
    """VPI 3.1 CUDA SGM, following the installed NVIDIA stereo sample."""

    def __init__(self, max_disparity: int, confidence_threshold: int) -> None:
        self._vpi: Any = importlib.import_module("vpi")
        self._max_disparity = max_disparity
        self._threshold = confidence_threshold

    def estimate(
        self, left: NDArray[np.uint8], right: NDArray[np.uint8]
    ) -> tuple[NDArray[np.float32], NDArray[np.uint16]]:
        vpi = self._vpi
        with vpi.Backend.CUDA:
            l = vpi.asimage(np.ascontiguousarray(left)).convert(vpi.Format.Y16_ER, scale=1)
            r = vpi.asimage(np.ascontiguousarray(right)).convert(vpi.Format.Y16_ER, scale=1)
            confidence = vpi.Image((left.shape[1], left.shape[0]), vpi.Format.U16)
            disparity = vpi.stereodisp(
                l,
                r,
                downscale=1,
                out_confmap=confidence,
                maxdisp=self._max_disparity,
                confthreshold=self._threshold,
                conftype=vpi.ConfidenceType.ABSOLUTE,
            )
            # VPI disparity is signed Q10.5, NOT pixels or a visualization-scaled U8.
            return (
                np.asarray(disparity.cpu(), dtype=np.float32).copy() / 32.0,
                np.asarray(confidence.cpu(), dtype=np.uint16).copy(),
            )


class OpenCvStereo:
    """Portable reference backend. Confidence is a binary validity mask, not probability."""

    def __init__(self, max_disparity: int) -> None:
        self._matcher = cv2.StereoSGBM_create(  # type: ignore[attr-defined]
            minDisparity=0,
            numDisparities=max_disparity,
            blockSize=5,
            P1=8 * 25,
            P2=32 * 25,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

    def estimate(
        self, left: NDArray[np.uint8], right: NDArray[np.uint8]
    ) -> tuple[NDArray[np.float32], NDArray[np.uint16]]:
        disparity = self._matcher.compute(left, right).astype(np.float32) / 16.0
        return disparity, np.where(disparity > 0, 65535, 0).astype(np.uint16)
