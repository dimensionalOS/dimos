# Copyright 2026 Dimensional Inc.
"""Deterministic point-cloud measurements for private VQA oracle tools."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from dimos.benchmark.vqa.generation.geometry import project_visible_points
from dimos.benchmark.vqa.models import CalibratedFrame, GroundPlaneEstimate


@dataclass(frozen=True)
class PlaneFitResult:
    """Accepted plane or explicit quality-gated rejection."""

    estimate: GroundPlaneEstimate | None
    quality_flags: tuple[str, ...]
    rejection_reason: str | None = None


def estimate_ground_plane(frame: CalibratedFrame) -> PlaneFitResult:
    """Fit a robust plane to visible points in the lower image ground band."""
    projected = project_visible_points(frame)
    candidates = np.asarray(
        [
            point
            for point, (_, y) in zip(projected.camera_points, projected.pixels, strict=True)
            if y >= int(frame.image.height * 0.6)
        ],
        dtype=np.float64,
    )
    min_points, min_inliers, threshold = 12, 10, 0.06
    if len(candidates) < min_points:
        return PlaneFitResult(None, ("insufficient_ground_band_points",), "insufficient_support")

    # Open3D owns the randomized consensus search; seed its process-global RNG
    # so repeated generation runs are stable for a fixed Open3D release.
    import open3d as o3d

    o3d.utility.random.seed(0)
    point_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(candidates))
    _, inlier_indices = point_cloud.segment_plane(
        distance_threshold=threshold,
        ransac_n=3,
        num_iterations=816,
    )
    if len(inlier_indices) < min_inliers:
        return PlaneFitResult(None, ("insufficient_plane_inliers",), "insufficient_inliers")

    inlier_points = candidates[np.asarray(inlier_indices, dtype=int)]
    center = np.mean(inlier_points, axis=0)
    _, _, right = np.linalg.svd(inlier_points - center, full_matrices=False)
    normal = right[-1]
    offset = -float(normal @ center)
    # The camera is normally above the support plane; this fixes signed heights.
    if offset < 0:
        normal, offset = -normal, -offset
    residuals = np.abs(candidates @ normal + offset)
    inliers = residuals <= threshold
    residual_m = (
        float(np.sqrt(np.mean(np.square(residuals[inliers])))) if inliers.any() else float("inf")
    )
    if int(inliers.sum()) < min_inliers:
        return PlaneFitResult(None, ("insufficient_refined_inliers",), "insufficient_inliers")
    if residual_m > 0.035:
        return PlaneFitResult(None, ("high_plane_residual",), "residual_too_high")
    return PlaneFitResult(
        GroundPlaneEstimate(
            tuple(float(value) for value in normal),
            float(offset),
            len(candidates),
            int(inliers.sum()),
            residual_m,
        ),
        ("ground_band_visible", "ransac_inliers_accepted"),
    )


def points_in_mask(frame: CalibratedFrame, mask: np.ndarray) -> np.ndarray:
    """Return nearest visible camera points covered by one foreground mask."""
    if mask.shape != (frame.image.height, frame.image.width):
        raise ValueError("segmentation mask dimensions must match the image")
    projected = project_visible_points(frame)
    return np.asarray(
        [
            point
            for point, (x, y) in zip(projected.camera_points, projected.pixels, strict=True)
            if mask[y, x] > 0
        ],
        dtype=np.float64,
    )
