# Copyright 2026 Dimensional Inc.
"""Small, strict object cropper for the recorded demonstration scene."""

from dataclasses import dataclass

import numpy as np

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True)
class AxisAlignedROI:
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]

    def validate(self) -> None:
        low, high = np.asarray(self.minimum), np.asarray(self.maximum)
        if low.shape != (3,) or high.shape != (3,) or not np.all(np.isfinite([low, high])):
            raise ValueError("ROI bounds must be finite 3-vectors")
        if not np.all(low < high):
            raise ValueError("ROI minimum must be less than maximum")


class ObjectPointCloudExtractor:
    """Extract an object using a configurable axis-aligned 3D ROI."""

    def __init__(self, roi: AxisAlignedROI, minimum_points: int = 4) -> None:
        roi.validate()
        if minimum_points < 1:
            raise ValueError("minimum_points must be positive")
        self.roi, self.minimum_points = roi, minimum_points

    def extract(self, scene: PointCloud2) -> PointCloud2:
        points, _ = scene.as_numpy()
        if scene.frame_id != "world":
            raise ValueError(f"scene must be in world, got {scene.frame_id!r}")
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < self.minimum_points:
            raise ValueError("scene has insufficient XYZ points")
        if not np.all(np.isfinite(points)):
            raise ValueError("scene contains non-finite points")
        low, high = np.asarray(self.roi.minimum), np.asarray(self.roi.maximum)
        mask = np.all((points >= low) & (points <= high), axis=1)
        cropped = points[mask]
        if len(cropped) < self.minimum_points:
            raise ValueError("ROI contains insufficient object points")
        return PointCloud2.from_numpy(cropped.astype(np.float32), scene.frame_id, scene.ts)
