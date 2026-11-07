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

"""Bounded, sensor-only RGB-D frames for the spatial viewer.

Wire format: uint32 LE JSON length, padded UTF-8 JSON, uint16 LE millimetre
depth, JPEG. RGB and depth must be registered to the same optical frame.
Rectification uses the supplied calibration, never simulator geometry.
"""

from dataclasses import dataclass
import json
import math
import struct
from typing import Any

import cv2
import numpy as np

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


@dataclass(frozen=True)
class SurfaceFrame:
    ts: float
    payload: bytes
    points: np.ndarray


class RGBDEncoder:
    def __init__(
        self,
        *,
        max_pixels: int = 60000,
        max_depth_m: float = 2.0,
        depth_unit_m: float = 0.001,
        sync_tolerance_s: float = 0.03,
        max_edge_m: float = 0.035,
    ) -> None:
        self.max_pixels = max_pixels
        self.max_depth_m = max_depth_m
        self.depth_unit_m = depth_unit_m
        self.sync_tolerance_s = sync_tolerance_s
        self.max_edge_m = max_edge_m
        self._calibration: tuple[Any, ...] = ()
        self._maps: tuple[np.ndarray, np.ndarray] | None = None

    def encode(
        self, color: Image, depth: Image, info: CameraInfo, transform: Transform
    ) -> SurfaceFrame:
        if (
            not np.isfinite([color.ts, depth.ts]).all()
            or abs(color.ts - depth.ts) > self.sync_tolerance_s
        ):
            raise ValueError("RGB and depth timestamps are not synchronized")
        if (
            not depth.frame_id
            or color.frame_id != depth.frame_id
            or info.frame_id != depth.frame_id
            or color.shape[:2] != depth.shape
            or depth.shape != (info.height, info.width)
            or transform.child_frame_id != depth.frame_id
            or transform.frame_id != "world"
        ):
            raise ValueError("RGB, depth and calibration must share the registered optical frame")
        if depth.format == ImageFormat.DEPTH and depth.dtype.kind == "f":
            metres = depth.data
        elif depth.format == ImageFormat.DEPTH16 and depth.dtype == np.uint16:
            metres = depth.data.astype(np.float32) * self.depth_unit_m
        else:
            raise ValueError("Depth must be float metres or uint16 with a configured unit scale")
        k = info.get_K_matrix()
        if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
            raise ValueError("Camera intrinsics are invalid")
        rgb = color.to_rgb().data
        key = (info.width, info.height, tuple(info.K), tuple(info.D), info.distortion_model)
        if key != self._calibration:
            self._maps = None
            if np.any(info.D):
                if info.distortion_model not in ("plumb_bob", "rational_polynomial"):
                    raise ValueError("Rectified RGB-D or supported OpenCV calibration is required")
                self._maps = cv2.initUndistortRectifyMap(
                    k, np.array(info.D), None, k, (info.width, info.height), cv2.CV_32FC1
                )
            self._calibration = key
        if self._maps is not None:
            # Nearest depth preserves invalid samples and never averages across an edge.
            metres = cv2.remap(metres, *self._maps, cv2.INTER_NEAREST)
            rgb = cv2.remap(rgb, *self._maps, cv2.INTER_LINEAR)
        step = max(1, math.ceil(math.sqrt(metres.size / self.max_pixels)))
        while math.ceil(info.width / step) * math.ceil(info.height / step) > self.max_pixels:
            step += 1
        sampled = metres[::step, ::step]
        valid = np.isfinite(sampled) & (sampled > 0) & (sampled <= self.max_depth_m)
        millimetres = np.rint(np.where(valid, sampled, 0) * 1000).astype("<u2")
        if not np.any(millimetres):
            raise ValueError("No valid workspace depth observations")
        height, width = sampled.shape
        v, u = np.mgrid[:height, :width]
        z = millimetres.astype(np.float32) * 0.001
        points = np.stack(
            ((u * step - k[0, 2]) * z / k[0, 0], (v * step - k[1, 2]) * z / k[1, 1], z), axis=-1
        )
        matrix = transform.to_matrix()
        points = points[millimetres > 0] @ matrix[:3, :3].T + matrix[:3, 3]
        header = {
            "version": 1,
            "ts": min(color.ts, depth.ts),
            "width": width,
            "height": height,
            "step": step,
            "image_width": info.width,
            "image_height": info.height,
            "K": info.K,
            "world_from_optical": matrix.T.ravel().tolist(),
            "max_edge_m": self.max_edge_m,
        }
        metadata = json.dumps(header, separators=(",", ":"), allow_nan=False).encode()
        metadata += b" " * (-len(metadata) % 4)
        jpeg = Image(rgb, ImageFormat.RGB).to_jpeg_bytes(quality=88)
        return SurfaceFrame(
            ts=header["ts"],
            payload=struct.pack("<I", len(metadata)) + metadata + millimetres.tobytes() + jpeg,
            points=points,
        )
