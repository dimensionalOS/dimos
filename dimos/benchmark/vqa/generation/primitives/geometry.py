"""Deterministic point-cloud geometry helpers for private VQA primitives."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
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


def points_around_mask(frame: CalibratedFrame, mask: np.ndarray, radius_px: int = 12) -> np.ndarray:
    """Return visible points in an annulus surrounding one foreground mask."""
    if radius_px < 1:
        raise ValueError("mask ring radius must be positive")
    if mask.shape != (frame.image.height, frame.image.width):
        raise ValueError("segmentation mask dimensions must match the image")
    foreground = mask > 0
    expanded = cv2.dilate(
        foreground.astype(np.uint8),
        np.ones((radius_px * 2 + 1, radius_px * 2 + 1), dtype=np.uint8),
    ).astype(bool)
    return points_in_mask(frame, expanded & ~foreground)


def fit_surface_plane(points: np.ndarray) -> PlaneFitResult:
    """Fit a robust plane to private surface points without assuming ground orientation."""
    min_points, min_inliers, threshold = 12, 10, 0.06
    if len(points) < min_points:
        return PlaneFitResult(None, ("insufficient_surface_points",), "insufficient_support")

    import open3d as o3d

    o3d.utility.random.seed(0)
    point_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    _, inlier_indices = point_cloud.segment_plane(
        distance_threshold=threshold,
        ransac_n=3,
        num_iterations=816,
    )
    if len(inlier_indices) < min_inliers:
        return PlaneFitResult(None, ("insufficient_surface_inliers",), "insufficient_inliers")

    inlier_points = points[np.asarray(inlier_indices, dtype=int)]
    center = np.mean(inlier_points, axis=0)
    _, _, right = np.linalg.svd(inlier_points - center, full_matrices=False)
    normal = right[-1]
    offset = -float(normal @ center)
    residuals = np.abs(points @ normal + offset)
    inliers = residuals <= threshold
    residual_m = (
        float(np.sqrt(np.mean(np.square(residuals[inliers])))) if inliers.any() else float("inf")
    )
    if int(inliers.sum()) < min_inliers:
        return PlaneFitResult(
            None, ("insufficient_refined_surface_inliers",), "insufficient_inliers"
        )
    if residual_m > 0.035:
        return PlaneFitResult(None, ("high_surface_residual",), "residual_too_high")
    return PlaneFitResult(
        GroundPlaneEstimate(
            tuple(float(value) for value in normal),
            float(offset),
            len(points),
            int(inliers.sum()),
            residual_m,
        ),
        ("surface_plane_accepted",),
    )


def classify_door_plane_angle(
    door: GroundPlaneEstimate, surrounding: GroundPlaneEstimate
) -> tuple[str | None, str | None, float]:
    """Classify only clearly coplanar or rotated door and surrounding planes."""
    alignment = abs(float(np.dot(door.normal, surrounding.normal)))
    angle_deg = float(np.degrees(np.arccos(np.clip(alignment, -1.0, 1.0))))
    if angle_deg <= 12.0:
        return "closed", None, angle_deg
    if angle_deg >= 25.0:
        return "open", None, angle_deg
    return None, "ambiguous_door_angle", angle_deg


def classify_forward_corridor(
    points: np.ndarray, ground: GroundPlaneEstimate
) -> tuple[str | None, tuple[str, ...], str | None]:
    """Classify a visible camera-forward corridor as clear or blocked."""
    if len(points) == 0:
        return None, (), "insufficient_forward_support"
    depth = points[:, 2]
    lateral_limit = depth * np.tan(np.radians(20.0))
    corridor = points[(depth >= 0.5) & (depth <= 3.0) & (np.abs(points[:, 0]) <= lateral_limit)]
    if len(corridor) < 12:
        return None, (), "insufficient_forward_support"
    elevation = corridor @ np.asarray(ground.normal) + ground.offset_m
    ground_points = corridor[np.abs(elevation) <= 0.08]
    for start, stop in ((0.5, 1.33), (1.33, 2.16), (2.16, 3.0)):
        if int(((ground_points[:, 2] >= start) & (ground_points[:, 2] < stop)).sum()) < 3:
            return None, (), "incomplete_forward_ground_support"
    obstacle_count = int((elevation > 0.15).sum())
    if obstacle_count >= 4:
        return "blocked", ("visible_forward_obstacle", "forward_ground_supported"), None
    if obstacle_count:
        return None, (), "ambiguous_forward_obstacle"
    return "clear", ("forward_ground_supported", "no_supported_forward_obstacle"), None
