# Copyright 2025 Dimensional Inc.
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

from __future__ import annotations

from dataclasses import dataclass, field
import time

import cv2
import numpy as np

from dimos.msgs.sensor_msgs.image_impls.AbstractImage import (
    AbstractImage,
    ImageFormat,
)


@dataclass
class NumpyImage(AbstractImage):
    data: np.ndarray  # type: ignore[type-arg]
    format: ImageFormat = field(default=ImageFormat.BGR)
    frame_id: str = field(default="")
    ts: float = field(default_factory=time.time)

    def __post_init__(self):  # type: ignore[no-untyped-def]
        if not isinstance(self.data, np.ndarray) or self.data.ndim < 2:
            raise ValueError("NumpyImage requires a 2D/3D NumPy array")

    @property
    def is_cuda(self) -> bool:
        return False

    def to_opencv(self) -> np.ndarray:  # type: ignore[type-arg]
        arr = self.data
        if self.format == ImageFormat.BGR:
            return arr
        if self.format == ImageFormat.RGB:
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if self.format == ImageFormat.RGBA:
            return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        if self.format == ImageFormat.BGRA:
            return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        if self.format in (
            ImageFormat.GRAY,
            ImageFormat.GRAY16,
            ImageFormat.DEPTH,
            ImageFormat.DEPTH16,
        ):
            return arr
        raise ValueError(f"Unsupported format: {self.format}")

    def to_rgb(self) -> NumpyImage:
        if self.format == ImageFormat.RGB:
            return self.copy()  # type: ignore
        arr = self.data
        if self.format == ImageFormat.BGR:
            return NumpyImage(
                cv2.cvtColor(arr, cv2.COLOR_BGR2RGB), ImageFormat.RGB, self.frame_id, self.ts
            )
        if self.format == ImageFormat.RGBA:
            return self.copy()  # type: ignore[return-value]  # RGBA contains RGB + alpha
        if self.format == ImageFormat.BGRA:
            rgba = cv2.cvtColor(arr, cv2.COLOR_BGRA2RGBA)
            return NumpyImage(rgba, ImageFormat.RGBA, self.frame_id, self.ts)
        if self.format in (ImageFormat.GRAY, ImageFormat.GRAY16, ImageFormat.DEPTH16):
            gray8 = (arr / 256).astype(np.uint8) if self.format != ImageFormat.GRAY else arr
            rgb = cv2.cvtColor(gray8, cv2.COLOR_GRAY2RGB)
            return NumpyImage(rgb, ImageFormat.RGB, self.frame_id, self.ts)
        return self.copy()  # type: ignore

    def to_bgr(self) -> NumpyImage:
        if self.format == ImageFormat.BGR:
            return self.copy()  # type: ignore
        arr = self.data
        if self.format == ImageFormat.RGB:
            return NumpyImage(
                cv2.cvtColor(arr, cv2.COLOR_RGB2BGR), ImageFormat.BGR, self.frame_id, self.ts
            )
        if self.format == ImageFormat.RGBA:
            return NumpyImage(
                cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR), ImageFormat.BGR, self.frame_id, self.ts
            )
        if self.format == ImageFormat.BGRA:
            return NumpyImage(
                cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR), ImageFormat.BGR, self.frame_id, self.ts
            )
        if self.format in (ImageFormat.GRAY, ImageFormat.GRAY16, ImageFormat.DEPTH16):
            gray8 = (arr / 256).astype(np.uint8) if self.format != ImageFormat.GRAY else arr
            return NumpyImage(
                cv2.cvtColor(gray8, cv2.COLOR_GRAY2BGR), ImageFormat.BGR, self.frame_id, self.ts
            )
        return self.copy()  # type: ignore

    def to_grayscale(self) -> NumpyImage:
        if self.format in (ImageFormat.GRAY, ImageFormat.GRAY16, ImageFormat.DEPTH):
            return self.copy()  # type: ignore
        if self.format == ImageFormat.BGR:
            return NumpyImage(
                cv2.cvtColor(self.data, cv2.COLOR_BGR2GRAY),
                ImageFormat.GRAY,
                self.frame_id,
                self.ts,
            )
        if self.format == ImageFormat.RGB:
            return NumpyImage(
                cv2.cvtColor(self.data, cv2.COLOR_RGB2GRAY),
                ImageFormat.GRAY,
                self.frame_id,
                self.ts,
            )
        if self.format in (ImageFormat.RGBA, ImageFormat.BGRA):
            code = cv2.COLOR_RGBA2GRAY if self.format == ImageFormat.RGBA else cv2.COLOR_BGRA2GRAY
            return NumpyImage(
                cv2.cvtColor(self.data, code), ImageFormat.GRAY, self.frame_id, self.ts
            )
        raise ValueError(f"Unsupported format: {self.format}")

    def resize(self, width: int, height: int, interpolation: int = cv2.INTER_LINEAR) -> NumpyImage:
        return NumpyImage(
            cv2.resize(self.data, (width, height), interpolation=interpolation),
            self.format,
            self.frame_id,
            self.ts,
        )

    def crop(self, x: int, y: int, width: int, height: int) -> NumpyImage:
        """Crop the image to the specified region.

        Args:
            x: Starting x coordinate (left edge)
            y: Starting y coordinate (top edge)
            width: Width of the cropped region
            height: Height of the cropped region

        Returns:
            A new NumpyImage containing the cropped region
        """
        # Get current image dimensions
        img_height, img_width = self.data.shape[:2]

        # Clamp the crop region to image bounds
        x = max(0, min(x, img_width))
        y = max(0, min(y, img_height))
        x_end = min(x + width, img_width)
        y_end = min(y + height, img_height)

        # Perform the crop using array slicing
        if self.data.ndim == 2:
            # Grayscale image
            cropped_data = self.data[y:y_end, x:x_end]
        else:
            # Color image (HxWxC)
            cropped_data = self.data[y:y_end, x:x_end, :]

        # Return a new NumpyImage with the cropped data
        return NumpyImage(cropped_data, self.format, self.frame_id, self.ts)

    def sharpness(self) -> float:
        gray = self.to_grayscale()
        sx = cv2.Sobel(gray.data, cv2.CV_32F, 1, 0, ksize=5)
        sy = cv2.Sobel(gray.data, cv2.CV_32F, 0, 1, ksize=5)
        magnitude = cv2.magnitude(sx, sy)
        mean_mag = float(magnitude.mean())
        if mean_mag <= 0:
            return 0.0
        return float(np.clip((np.log10(mean_mag + 1) - 1.7) / 2.0, 0.0, 1.0))

    # PnP wrappers
    def solve_pnp(
        self,
        object_points: np.ndarray,  # type: ignore[type-arg]
        image_points: np.ndarray,  # type: ignore[type-arg]
        camera_matrix: np.ndarray,  # type: ignore[type-arg]
        dist_coeffs: np.ndarray | None = None,  # type: ignore[type-arg]
        flags: int = cv2.SOLVEPNP_ITERATIVE,
    ) -> tuple[bool, np.ndarray, np.ndarray]:  # type: ignore[type-arg]
        obj = np.asarray(object_points, dtype=np.float32).reshape(-1, 3)
        img = np.asarray(image_points, dtype=np.float32).reshape(-1, 2)
        K = np.asarray(camera_matrix, dtype=np.float64)
        dist = None if dist_coeffs is None else np.asarray(dist_coeffs, dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=flags)  # type: ignore[arg-type]
        return bool(ok), rvec.astype(np.float64), tvec.astype(np.float64)

    def create_csrt_tracker(self, bbox: tuple[int, int, int, int]):  # type: ignore[no-untyped-def]
        tracker = None
        if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
            tracker = cv2.legacy.TrackerCSRT_create()
        elif hasattr(cv2, "TrackerCSRT_create"):
            tracker = cv2.TrackerCSRT_create()
        else:
            raise RuntimeError("OpenCV CSRT tracker not available")
        ok = tracker.init(self.to_bgr().to_opencv(), tuple(map(int, bbox)))
        if not ok:
            raise RuntimeError("Failed to initialize CSRT tracker")
        return tracker

    def csrt_update(self, tracker) -> tuple[bool, tuple[int, int, int, int]]:  # type: ignore[no-untyped-def]
        ok, box = tracker.update(self.to_bgr().to_opencv())
        if not ok:
            return False, (0, 0, 0, 0)
        x, y, w, h = map(int, box)
        return True, (x, y, w, h)

    def solve_pnp_ransac(
        self,
        object_points: np.ndarray,  # type: ignore[type-arg]
        image_points: np.ndarray,  # type: ignore[type-arg]
        camera_matrix: np.ndarray,  # type: ignore[type-arg]
        dist_coeffs: np.ndarray | None = None,  # type: ignore[type-arg]
        iterations_count: int = 100,
        reprojection_error: float = 3.0,
        confidence: float = 0.99,
        min_sample: int = 6,
    ) -> tuple[bool, np.ndarray, np.ndarray, np.ndarray]:  # type: ignore[type-arg]
        obj = np.asarray(object_points, dtype=np.float32).reshape(-1, 3)
        img = np.asarray(image_points, dtype=np.float32).reshape(-1, 2)
        K = np.asarray(camera_matrix, dtype=np.float64)
        dist = None if dist_coeffs is None else np.asarray(dist_coeffs, dtype=np.float64)
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            obj,
            img,
            K,
            dist,  # type: ignore[arg-type]
            iterationsCount=int(iterations_count),
            reprojectionError=float(reprojection_error),
            confidence=float(confidence),
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        mask = np.zeros((obj.shape[0],), dtype=np.uint8)
        if inliers is not None and len(inliers) > 0:
            mask[inliers.flatten()] = 1
        return bool(ok), rvec.astype(np.float64), tvec.astype(np.float64), mask
