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

"""Fixture verification helpers for the checked-in AprilTag board images."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from reportlab.lib.pagesizes import A4, LETTER
from reportlab.lib.units import mm
import yaml

from dimos.utils.cli.apriltag import _grid_layout

PAGE_SIZES_PT = {
    "a4": A4,
    "letter": LETTER,
}


@dataclass(frozen=True)
class TagLayout:
    """A single printed tag in board coordinates."""

    tag_id: int
    col: int
    row: int
    bottom_left_m: np.ndarray
    center_m: np.ndarray
    corners_m: np.ndarray


@dataclass(frozen=True)
class BoardLayout:
    """Generated board layout in the page frame: x right, y up, z out of page."""

    cols: int
    rows: int
    marker_length_m: float
    tags: dict[int, TagLayout]

    @property
    def ids(self) -> list[int]:
        return sorted(self.tags)


@dataclass(frozen=True)
class DetectionResult:
    corners_by_id: dict[int, np.ndarray]
    image_width_px: int
    image_height_px: int

    @property
    def ids(self) -> list[int]:
        return sorted(self.corners_by_id)


@dataclass(frozen=True)
class BoardLayoutGeometryResult:
    ok: bool
    layout_errors_px: list[float]
    median_tag_edge_px: float

    @property
    def layout_error_px_p50(self) -> float:
        return _percentile_or_zero(self.layout_errors_px, 50)

    @property
    def layout_error_px_p95(self) -> float:
        return _percentile_or_zero(self.layout_errors_px, 95)


@dataclass(frozen=True)
class FrameMetrics:
    image_width_px: int
    image_height_px: int
    median_tag_edge_percent: float
    visible_image_hull_area_percent: float
    visible_board_layout_area_percent: float
    board_layout_error_px_p50: float
    board_layout_error_px_p95: float


@dataclass(frozen=True)
class FrameVerificationResult:
    frame_id: str
    detected_ids: list[int]
    computed_class: str
    apparent_scale: str
    image_footprint: str
    metrics: FrameMetrics
    accepted: bool
    reject_reasons: list[str]
    detection: DetectionResult
    board_layout_geometry: BoardLayoutGeometryResult | None


def load_manifest(path: Path) -> dict[str, Any]:
    with path.open() as f:
        manifest = yaml.safe_load(f)
    if not isinstance(manifest, dict):
        raise ValueError(f"Manifest {path} did not contain a YAML mapping")
    return manifest


def generated_apriltag_board_layout(
    ids: list[int],
    *,
    marker_length_m: float = 0.05,
    page_size: str = "a4",
) -> BoardLayout:
    """Return the packed DimOS generator layout for the fixture board."""
    if page_size not in PAGE_SIZES_PT:
        raise ValueError(f"Unsupported page size: {page_size}")

    size_mm = marker_length_m * 1000.0
    page_w_pt, page_h_pt = PAGE_SIZES_PT[page_size]
    cols, rows, x0_pt, y_top_pt, tile_w_pt, tile_h_pt = _grid_layout(
        page_w_pt,
        page_h_pt,
        size_mm,
    )

    tag_layouts: dict[int, TagLayout] = {}
    n = len(ids)
    last_row_count = n - (n // cols) * cols or cols
    last_row_idx = (n - 1) // cols
    last_row_offset = (cols - last_row_count) * tile_w_pt / 2

    for idx, tag_id in enumerate(ids):
        row = idx // cols
        col = idx % cols
        x_pt = x0_pt + col * tile_w_pt + (last_row_offset if row == last_row_idx else 0.0)
        y_pt = y_top_pt - row * tile_h_pt - size_mm * mm
        bottom_left_m = np.array([x_pt / mm / 1000.0, y_pt / mm / 1000.0, 0.0])
        center_m = bottom_left_m + np.array([marker_length_m / 2, marker_length_m / 2, 0.0])
        half = marker_length_m / 2
        corners_m = np.array(
            [
                [center_m[0] - half, center_m[1] + half, 0.0],
                [center_m[0] + half, center_m[1] + half, 0.0],
                [center_m[0] + half, center_m[1] - half, 0.0],
                [center_m[0] - half, center_m[1] - half, 0.0],
            ],
            dtype=np.float64,
        )
        tag_layouts[int(tag_id)] = TagLayout(
            tag_id=int(tag_id),
            col=col,
            row=row,
            bottom_left_m=bottom_left_m,
            center_m=center_m,
            corners_m=corners_m,
        )

    return BoardLayout(
        cols=cols,
        rows=rows,
        marker_length_m=marker_length_m,
        tags=tag_layouts,
    )


def detect_apriltag_frame(image_path: Path, dictionary_name: str) -> DetectionResult:
    """Run OpenCV AprilTag detection on a fixture image."""
    if not hasattr(cv2.aruco, dictionary_name):
        raise ValueError(f"Unknown ArUco dictionary {dictionary_name!r}")
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"Could not load image: {image_path}")
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name)),
        cv2.aruco.DetectorParameters(),
    )
    corners, ids, _rejected = detector.detectMarkers(image)
    corners_by_id: dict[int, np.ndarray] = {}
    if ids is not None:
        for corner_set, id_arr in zip(corners, ids, strict=True):
            corners_by_id[int(id_arr[0])] = np.asarray(corner_set, dtype=np.float32).reshape(4, 2)
    height, width = image.shape[:2]
    return DetectionResult(corners_by_id, width, height)


def median_tag_edge_percent(
    corners_by_id: dict[int, np.ndarray], image_size: tuple[int, int]
) -> float:
    """Median detected marker edge length divided by the short image side, in percent."""
    width, height = image_size
    edges_px: list[float] = []
    for corners in corners_by_id.values():
        for idx in range(4):
            edges_px.append(float(np.linalg.norm(corners[(idx + 1) % 4] - corners[idx])))
    if not edges_px:
        return 0.0
    return float(np.median(edges_px) / min(width, height) * 100.0)


def visible_image_hull_area_percent(
    corners_by_id: dict[int, np.ndarray],
    image_size: tuple[int, int],
) -> float:
    """Convex hull area of detected marker corners divided by image area, in percent."""
    width, height = image_size
    if not corners_by_id:
        return 0.0
    points = np.concatenate(list(corners_by_id.values()), axis=0).astype(np.float32)
    return _convex_hull_area(points) / (width * height) * 100.0


def visible_board_layout_area_percent(layout: BoardLayout, visible_ids: list[int]) -> float:
    """Convex hull area of generated visible tag corners divided by the full layout hull.

    IDs not present in ``layout.tags`` are ignored so detector false positives cannot
    raise ``KeyError`` when building ``visible_points``.
    """
    visible_layout_ids = [tag_id for tag_id in visible_ids if tag_id in layout.tags]
    if not visible_layout_ids:
        return 0.0
    full_points = np.concatenate([tag.corners_m[:, :2] for tag in layout.tags.values()], axis=0)
    visible_points = np.concatenate(
        [layout.tags[tag_id].corners_m[:, :2] for tag_id in visible_layout_ids], axis=0
    )
    full_area = _convex_hull_area(full_points.astype(np.float32))
    return _convex_hull_area(visible_points.astype(np.float32)) / full_area * 100.0


def apparent_scale_bin(median_edge_percent: float) -> str:
    if 4.0 <= median_edge_percent < 8.0:
        return "small_tag"
    if 8.0 <= median_edge_percent < 18.0:
        return "medium_tag"
    if 18.0 <= median_edge_percent <= 35.0:
        return "large_tag"
    return "reject"


def image_footprint_bin(visible_hull_area_percent: float) -> str:
    if 1.0 <= visible_hull_area_percent < 8.0:
        return "low_image_footprint"
    if 8.0 <= visible_hull_area_percent < 25.0:
        return "medium_image_footprint"
    if 25.0 <= visible_hull_area_percent <= 60.0:
        return "high_image_footprint"
    return "reject"


def board_completeness_class(layout: BoardLayout, visible_ids: list[int]) -> str:
    visible_layout_ids = [tag_id for tag_id in visible_ids if tag_id in layout.tags]
    area_percent = visible_board_layout_area_percent(layout, visible_layout_ids)
    visible_count = len(visible_layout_ids)
    if visible_count == 0:
        return "no_board"
    if visible_count == len(layout.tags) and area_percent >= 95.0:
        return "full_board"
    if 6 <= visible_count <= 11 and 55.0 <= area_percent < 95.0:
        return "partial_board_large"
    if 4 <= visible_count <= 8 and 30.0 <= area_percent < 55.0:
        return "partial_board_medium"
    if 2 <= visible_count <= 5 and 10.0 <= area_percent < 30.0:
        return "partial_board_small"
    return "insufficient_board"


def validate_detection_expectation(frame: dict[str, Any], detected_ids: list[int]) -> None:
    """Validate detected IDs against manifest visible/absent expectations."""
    expected_visible = set(frame["expected_visible_ids"])
    expected_absent = set(frame["expected_absent_ids"])
    allowed_extra = set(frame.get("allowed_extra_ids", []))
    allowed_missing = set(frame.get("allowed_missing_ids", []))
    allowed_missing_max = int(frame.get("allowed_missing_max", 0) or 0)
    detected = set(detected_ids)

    unexpected = detected - expected_visible - allowed_extra
    if unexpected:
        raise ValueError(f"Unexpected detected IDs: {sorted(unexpected)}")
    absent_detected = detected & expected_absent
    if absent_detected:
        raise ValueError(f"Expected-absent IDs were detected: {sorted(absent_detected)}")

    missing = expected_visible - detected
    disallowed_missing = missing - allowed_missing
    if len(disallowed_missing) > allowed_missing_max:
        raise ValueError(f"Too many missing expected IDs: {sorted(disallowed_missing)}")


def verify_board_layout_geometry(
    corners_by_id: dict[int, np.ndarray],
    layout: BoardLayout,
    *,
    max_layout_error_px_p95: float = 3.0,
) -> BoardLayoutGeometryResult:
    """Verify detections are consistent with the generated PDF board plane.

    This is a 2D PDF-layout check: it fits a homography from generated board
    corners to detected image corners and measures residuals.
    """
    source_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    edges_px: list[float] = []
    for tag_id, corners in corners_by_id.items():
        if tag_id not in layout.tags:
            continue
        source_points.extend(layout.tags[tag_id].corners_m[:, :2].astype(np.float32))
        image_points.extend(corners.astype(np.float32))
        for idx in range(4):
            edges_px.append(float(np.linalg.norm(corners[(idx + 1) % 4] - corners[idx])))

    if len(source_points) < 4:
        return BoardLayoutGeometryResult(False, [], _percentile_or_zero(edges_px, 50))

    source = np.asarray(source_points, dtype=np.float32)
    image = np.asarray(image_points, dtype=np.float32)
    homography, _mask = cv2.findHomography(source, image, method=0)
    if homography is None:
        return BoardLayoutGeometryResult(False, [], _percentile_or_zero(edges_px, 50))

    projected = cv2.perspectiveTransform(source.reshape(-1, 1, 2), homography).reshape(-1, 2)
    residuals = projected - image
    errors_px = [float(np.linalg.norm(residual)) for residual in residuals]
    return BoardLayoutGeometryResult(
        _percentile_or_zero(errors_px, 95) <= max_layout_error_px_p95,
        errors_px,
        _percentile_or_zero(edges_px, 50),
    )


def verify_fixture_frame(
    frame: dict[str, Any],
    *,
    repo_root: Path,
    manifest: dict[str, Any],
    layout: BoardLayout,
    max_board_layout_error_px_p95: float = 3.0,
) -> FrameVerificationResult:
    """Run detector, classification, and PDF-layout checks for one manifest row."""
    detection = detect_apriltag_frame(
        repo_root / frame["image_path"],
        manifest["fixture"]["opencv_dictionary"],
    )
    image_size = (detection.image_width_px, detection.image_height_px)
    board_layout_geometry = (
        verify_board_layout_geometry(
            detection.corners_by_id,
            layout,
            max_layout_error_px_p95=max_board_layout_error_px_p95,
        )
        if detection.ids
        else None
    )
    # AprilTag may report IDs outside the PDF layout - board metrics must not KeyError.
    detected_ids_in_layout = [tag_id for tag_id in detection.ids if tag_id in layout.tags]
    metrics = FrameMetrics(
        image_width_px=detection.image_width_px,
        image_height_px=detection.image_height_px,
        median_tag_edge_percent=median_tag_edge_percent(detection.corners_by_id, image_size),
        visible_image_hull_area_percent=visible_image_hull_area_percent(
            detection.corners_by_id,
            image_size,
        ),
        visible_board_layout_area_percent=visible_board_layout_area_percent(
            layout, detected_ids_in_layout
        ),
        board_layout_error_px_p50=(
            board_layout_geometry.layout_error_px_p50 if board_layout_geometry else 0.0
        ),
        board_layout_error_px_p95=(
            board_layout_geometry.layout_error_px_p95 if board_layout_geometry else 0.0
        ),
    )

    reject_reasons: list[str] = []
    try:
        validate_detection_expectation(frame, detection.ids)
    except ValueError as exc:
        reject_reasons.append(str(exc))

    is_positive = frame["operator_planned_class"] != "none"
    computed_class = board_completeness_class(layout, detected_ids_in_layout)
    if is_positive:
        if board_layout_geometry is None or not board_layout_geometry.ok:
            reject_reasons.append(
                "Board layout homography exceeds p95 threshold: "
                f"{metrics.board_layout_error_px_p95:.2f}px"
            )
    elif computed_class not in ("no_board", "insufficient_board"):
        reject_reasons.append(f"Negative frame computed as {computed_class}")

    return FrameVerificationResult(
        frame_id=str(frame["frame_id"]),
        detected_ids=detection.ids,
        computed_class=computed_class,
        apparent_scale=apparent_scale_bin(metrics.median_tag_edge_percent),
        image_footprint=image_footprint_bin(metrics.visible_image_hull_area_percent),
        metrics=metrics,
        accepted=is_positive and not reject_reasons,
        reject_reasons=reject_reasons,
        detection=detection,
        board_layout_geometry=board_layout_geometry,
    )


def _convex_hull_area(points: np.ndarray) -> float:
    if len(points) < 3:
        return 0.0
    return float(cv2.contourArea(cv2.convexHull(points.reshape(-1, 1, 2))))


def _percentile_or_zero(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    return float(np.percentile(np.asarray(values, dtype=np.float64), percentile))
