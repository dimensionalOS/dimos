"""Deterministic point-cloud geometry helpers for private VQA primitives."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

from dimos.benchmark.vqa.contracts import CalibratedFrame, GroundPlaneEstimate, OracleMeasurement

from .projection import project_visible_points


@dataclass(frozen=True)
class PlaneFitResult:
    """Accepted plane or explicit quality-gated rejection."""

    estimate: GroundPlaneEstimate | None
    quality_flags: tuple[str, ...]
    rejection_reason: str | None = None


@dataclass(frozen=True)
class OpeningGeometryResult:
    """Opening width from a mask and surrounding structural plane."""

    width_m: float | None
    tolerance_m: float | None
    quality_flags: tuple[str, ...]
    rejection_reason: str | None = None


@dataclass(frozen=True)
class ForwardCorridorMeasurement:
    """Private point-support metrics for the camera-forward corridor."""

    ground_band_counts: tuple[int, int, int]
    obstacle_count: int
    largest_obstacle_cluster_count: int
    point_count: int
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
            (float(normal[0]), float(normal[1]), float(normal[2])),
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
    import cv2

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
            (float(normal[0]), float(normal[1]), float(normal[2])),
            float(offset),
            len(points),
            int(inliers.sum()),
            residual_m,
        ),
        ("surface_plane_accepted",),
    )


def measure_relative_plane_angle(
    first: GroundPlaneEstimate, second: GroundPlaneEstimate
) -> OracleMeasurement:
    """Measure the unsigned angle between two accepted plane normals."""
    alignment = abs(float(np.dot(first.normal, second.normal)))
    return OracleMeasurement(
        float(np.degrees(np.arccos(np.clip(alignment, -1.0, 1.0)))),
        "deg",
        2.0,
        ("accepted_plane_normals",),
        (),
    )


def classify_forward_corridor(
    points: np.ndarray, ground: GroundPlaneEstimate
) -> tuple[Literal["clear", "blocked"] | None, tuple[str, ...], str | None]:
    """Classify a visible camera-forward corridor as clear or blocked."""
    measured = measure_forward_corridor(points, ground)
    if measured.rejection_reason is not None:
        return None, (), measured.rejection_reason
    if measured.largest_obstacle_cluster_count >= 4:
        return "blocked", ("visible_forward_obstacle", "forward_ground_supported"), None
    if measured.obstacle_count:
        return None, (), "ambiguous_forward_obstacle"
    return "clear", ("forward_ground_supported", "no_supported_forward_obstacle"), None


def measure_forward_corridor(
    points: np.ndarray, ground: GroundPlaneEstimate
) -> ForwardCorridorMeasurement:
    """Measure private floor and elevated-obstacle support in the camera-forward corridor."""
    if len(points) == 0:
        return ForwardCorridorMeasurement((0, 0, 0), 0, 0, 0, "insufficient_forward_support")
    depth = points[:, 2]
    corridor = points[(depth > 0.0) & (depth <= 2.0) & (np.abs(points[:, 0]) <= 0.5)]
    if len(corridor) < 12:
        return ForwardCorridorMeasurement(
            (0, 0, 0), 0, 0, len(corridor), "insufficient_forward_support"
        )
    elevation = corridor @ np.asarray(ground.normal) + ground.offset_m
    ground_points = corridor[np.abs(elevation) <= 0.08]
    band_counts = [
        int(((ground_points[:, 2] >= start) & (ground_points[:, 2] < stop)).sum())
        for start, stop in ((0.0, 2 / 3), (2 / 3, 4 / 3), (4 / 3, 2.01))
    ]
    bands = (band_counts[0], band_counts[1], band_counts[2])
    if any(count < 3 for count in bands):
        return ForwardCorridorMeasurement(
            bands, 0, 0, len(corridor), "incomplete_forward_ground_support"
        )
    obstacles = corridor[(elevation > 0.15) & (elevation <= 2.0)]
    obstacle_count = len(obstacles)
    largest_cluster = _largest_radius_cluster(obstacles, radius_m=0.2)
    return ForwardCorridorMeasurement(bands, obstacle_count, largest_cluster, len(corridor))


def _largest_radius_cluster(points: np.ndarray, radius_m: float) -> int:
    if len(points) == 0:
        return 0
    distances = np.linalg.norm(points[:, np.newaxis, :] - points[np.newaxis, :, :], axis=2)
    unvisited = set(range(len(points)))
    largest = 0
    while unvisited:
        pending = [unvisited.pop()]
        size = 0
        while pending:
            index = pending.pop()
            size += 1
            neighbors = {item for item in unvisited if distances[index, item] <= radius_m}
            unvisited.difference_update(neighbors)
            pending.extend(neighbors)
        largest = max(largest, size)
    return largest


def measure_opening_width(
    frame: CalibratedFrame, mask: np.ndarray, ground: GroundPlaneEstimate
) -> OpeningGeometryResult:
    """Measure a ground-connected vertical aperture from its silhouette and surrounding wall plane."""
    import cv2

    if mask.shape != (frame.image.height, frame.image.width):
        raise ValueError("segmentation mask dimensions must match the image")
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        (mask > 0).astype(np.uint8)
    )
    components = [
        index for index in range(1, component_count) if stats[index, cv2.CC_STAT_AREA] >= 128
    ]
    if len(components) != 1:
        return OpeningGeometryResult(None, None, (), "ambiguous_opening_component")
    component = components[0]
    x, y, width_px, height_px, _ = stats[component]
    if x == 0 or y == 0 or x + width_px >= frame.image.width or y + height_px >= frame.image.height:
        return OpeningGeometryResult(None, None, (), "opening_touches_image_edge")
    structure = fit_surface_plane(points_around_mask(frame, (labels == component).astype(np.uint8)))
    if structure.estimate is None:
        return OpeningGeometryResult(
            None,
            None,
            structure.quality_flags,
            structure.rejection_reason or "structure_plane_rejected",
        )
    normal = np.asarray(structure.estimate.normal)
    ground_normal = np.asarray(ground.normal)
    if abs(float(normal @ ground_normal)) > np.sin(np.radians(20.0)):
        return OpeningGeometryResult(
            None, None, structure.quality_flags, "opening_structure_not_vertical"
        )
    direction = np.cross(ground_normal, normal)
    direction_norm = float(np.linalg.norm(direction))
    if direction_norm == 0:
        return OpeningGeometryResult(
            None, None, structure.quality_flags, "opening_width_axis_rejected"
        )
    direction /= direction_norm
    component_mask = labels == component
    rows = range(y + int(height_px * 0.3), y + max(int(height_px * 0.7), 1))
    widths: list[float] = []
    for row in rows:
        columns = np.flatnonzero(component_mask[row])
        if len(columns) < 2:
            continue
        left = _intersect_pixel_with_plane(frame, int(columns[0]), row, structure.estimate)
        right = _intersect_pixel_with_plane(frame, int(columns[-1]), row, structure.estimate)
        if left is not None and right is not None:
            widths.append(abs(float((right - left) @ direction)))
    if len(widths) < 5:
        return OpeningGeometryResult(
            None, None, structure.quality_flags, "insufficient_opening_scanlines"
        )
    median_width = float(np.median(widths))
    mad = float(np.median(np.abs(np.asarray(widths) - median_width)))
    if median_width < 0.4 or max(widths) - min(widths) > max(0.1, median_width * 0.1):
        return OpeningGeometryResult(None, None, structure.quality_flags, "ambiguous_opening_width")
    bottom_columns = np.flatnonzero(component_mask[y + height_px - 1])
    if not len(bottom_columns):
        return OpeningGeometryResult(
            None, None, structure.quality_flags, "opening_not_ground_connected"
        )
    bottom = _intersect_pixel_with_plane(
        frame, int(np.median(bottom_columns)), y + height_px - 1, structure.estimate
    )
    if bottom is None or abs(float(bottom @ ground_normal + ground.offset_m)) > 0.08:
        return OpeningGeometryResult(
            None, None, structure.quality_flags, "opening_not_ground_connected"
        )
    return OpeningGeometryResult(
        median_width,
        max(0.05, structure.estimate.residual_m + 1.4826 * mad),
        (*structure.quality_flags, "ground_connected_vertical_opening", "stable_opening_scanlines"),
    )


def _intersect_pixel_with_plane(
    frame: CalibratedFrame, x: int, y: int, plane: GroundPlaneEstimate
) -> np.ndarray | None:
    fx, fy, cx, cy = (
        frame.camera_info.K[0],
        frame.camera_info.K[4],
        frame.camera_info.K[2],
        frame.camera_info.K[5],
    )
    if fx <= 0 or fy <= 0:
        return None
    ray = np.asarray(((x - cx) / fx, (y - cy) / fy, 1.0))
    denominator = float(np.asarray(plane.normal) @ ray)
    if abs(denominator) < 1e-6:
        return None
    distance = -plane.offset_m / denominator
    return ray * distance if distance > 0 else None
