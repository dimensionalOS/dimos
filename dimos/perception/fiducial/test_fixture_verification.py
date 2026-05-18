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

from __future__ import annotations

import cv2
import numpy as np
import pytest

pytest.importorskip("cv2.aruco")

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.fiducial.fixture_verification import (
    BoardLayout,
    apparent_scale_bin,
    board_completeness_class,
    generated_apriltag_board_layout,
    image_footprint_bin,
    median_tag_edge_percent,
    verify_board_layout_geometry,
    visible_board_layout_area_percent,
    visible_image_hull_area_percent,
)
from dimos.perception.fiducial.marker_tf_module import MarkerTfModule


@pytest.fixture(scope="module")
def layout() -> BoardLayout:
    return generated_apriltag_board_layout(list(range(12)), marker_length_m=0.05, page_size="a4")


def _synthetic_packed_apriltag_board_bgr(
    layout: BoardLayout,
    *,
    width: int,
    height: int,
    marker_inner_px: int = 220,
    margin_frac: float = 0.06,
) -> np.ndarray:
    """Render the packed DimOS A4 layout in pixels (no checked-in PNG fixtures)."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    all_xy = np.concatenate(
        [layout.tags[i].corners_m[:, :2] for i in sorted(layout.tags)],
        axis=0,
    )
    min_xy = all_xy.min(axis=0)
    max_xy = all_xy.max(axis=0)
    span_x = float(max_xy[0] - min_xy[0])
    span_y = float(max_xy[1] - min_xy[1])
    margin = margin_frac * min(width, height)
    usable_w = width - 2.0 * margin
    usable_h = height - 2.0 * margin
    scale = min(usable_w / span_x, usable_h / span_y)

    inner = marker_inner_px
    src = np.array(
        [[0.0, 0.0], [inner - 1.0, 0.0], [inner - 1.0, inner - 1.0], [0.0, inner - 1.0]],
        dtype=np.float32,
    )

    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    for tag_id in sorted(layout.tags):
        tile = np.zeros((inner, inner), dtype=np.uint8)
        cv2.aruco.generateImageMarker(dictionary, tag_id, inner, tile)
        tile_bgr = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)
        corners_xy = layout.tags[tag_id].corners_m[:, :2].astype(np.float64)
        dst = np.empty((4, 2), dtype=np.float32)
        for k in range(4):
            xm, ym = corners_xy[k]
            dst[k, 0] = margin + (xm - min_xy[0]) * scale
            dst[k, 1] = margin + (max_xy[1] - ym) * scale
        H, _ = cv2.findHomography(src, dst, method=0)
        if H is None:
            raise RuntimeError("homography failed for synthetic marker tile")
        warped = cv2.warpPerspective(
            tile_bgr,
            H,
            (width, height),
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(255, 255, 255),
        )
        canvas = np.minimum(canvas, warped)
    return canvas


def test_fixture_layout_matches_dimos_apriltag_generator_for_a4_50mm_3x4(
    layout: BoardLayout,
) -> None:
    assert (layout.cols, layout.rows) == (3, 4)
    expected_bottom_left_mm = {
        0: (20.0, 218.8),
        1: (80.0, 218.8),
        2: (140.0, 218.8),
        3: (20.0, 159.6),
        4: (80.0, 159.6),
        5: (140.0, 159.6),
        6: (20.0, 100.4),
        7: (80.0, 100.4),
        8: (140.0, 100.4),
        9: (20.0, 41.2),
        10: (80.0, 41.2),
        11: (140.0, 41.2),
    }
    for tag_id, (x_mm, y_mm) in expected_bottom_left_mm.items():
        tag = layout.tags[tag_id]
        assert (tag.col, tag.row) == (tag_id % 3, tag_id // 3)
        np.testing.assert_allclose(tag.bottom_left_m[:2] * 1000.0, [x_mm, y_mm], atol=0.05)
        np.testing.assert_allclose(
            tag.center_m[:2] * 1000.0,
            [x_mm + 25.0, y_mm + 25.0],
            atol=0.05,
        )
        assert tag.corners_m.shape == (4, 3)


def test_synthetic_packed_board_detects_all_twelve_ids_and_layout_homography(
    layout: BoardLayout,
) -> None:
    width, height = 1920, 1080
    bgr = _synthetic_packed_apriltag_board_bgr(layout, width=width, height=height)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
        cv2.aruco.DetectorParameters(),
    )
    corners, ids, _ = det.detectMarkers(gray)
    assert ids is not None and len(ids) == 12
    detected = sorted(int(np.asarray(i).reshape(-1)[0]) for i in ids)
    assert detected == list(range(12))

    corners_by_id: dict[int, np.ndarray] = {}
    for corner_set, id_arr in zip(corners, ids, strict=True):
        corners_by_id[int(id_arr[0])] = np.asarray(corner_set, dtype=np.float32).reshape(4, 2)

    geom = verify_board_layout_geometry(corners_by_id, layout)
    assert geom.ok, f"layout homography failed p95={geom.layout_error_px_p95}"
    assert geom.layout_error_px_p95 <= 3.0


def test_marker_tf_replay_synthetic_packed_board_publishes_twelve_markers(
    layout: BoardLayout,
) -> None:
    width, height = 1920, 1080
    bgr = _synthetic_packed_apriltag_board_bgr(layout, width=width, height=height)
    ts = 10_000.0
    cam_info = CameraInfo.from_intrinsics(
        1400.0,
        1400.0,
        width / 2.0,
        height / 2.0,
        width,
        height,
        frame_id="camera_optical",
    )
    cam_info.ts = ts

    mod = MarkerTfModule(
        marker_length_m=0.05,
        marker_namespace_prefix="fixture",
        max_freq=60.0,
    )
    try:
        mod.tf.publish(
            Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="world",
                child_frame_id="base_link",
                ts=ts,
            ),
            Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="base_link",
                child_frame_id="camera_optical",
                ts=ts,
            ),
        )
        mod._latest_camera_info = cam_info
        image = Image.from_opencv(bgr, frame_id="camera_optical", ts=ts)
        mod._process_color_image(image)

        marker_parent = "fixture/markers"
        assert mod.tf.get("world", marker_parent, ts, 0.1) is not None

        for tag_id in range(12):
            tr = mod.tf.get(marker_parent, f"fixture/marker_{tag_id}", ts, 0.1)
            assert tr is not None, f"missing marker {tag_id}"
            assert np.all(np.isfinite(tr.to_matrix()))
    finally:
        mod.stop()


def test_apparent_scale_bins_use_normalized_tag_edge_percent() -> None:
    corners_by_id = {0: _square_corners(10.0, 10.0, 80.0)}
    assert median_tag_edge_percent(corners_by_id, (1000, 500)) == pytest.approx(16.0)
    assert apparent_scale_bin(5.0) == "small_tag"
    assert apparent_scale_bin(10.0) == "medium_tag"
    assert apparent_scale_bin(20.0) == "large_tag"
    assert apparent_scale_bin(3.99) == "reject"
    assert apparent_scale_bin(35.01) == "reject"


def test_image_footprint_bins_use_visible_image_hull_area_percent() -> None:
    low = {0: _rect_corners(100.0, 100.0, 300.0, 200.0)}
    medium = {0: _rect_corners(100.0, 100.0, 450.0, 450.0)}
    high = {0: _rect_corners(100.0, 100.0, 700.0, 700.0)}
    assert visible_image_hull_area_percent(low, (1000, 1000)) == pytest.approx(2.0)
    assert image_footprint_bin(visible_image_hull_area_percent(low, (1000, 1000))) == (
        "low_image_footprint"
    )
    assert image_footprint_bin(visible_image_hull_area_percent(medium, (1000, 1000))) == (
        "medium_image_footprint"
    )
    assert image_footprint_bin(visible_image_hull_area_percent(high, (1000, 1000))) == (
        "high_image_footprint"
    )
    assert image_footprint_bin(0.5) == "reject"


def test_board_completeness_uses_generated_layout_area_percent(layout: BoardLayout) -> None:
    assert visible_board_layout_area_percent(layout, list(range(12))) == pytest.approx(100.0)
    assert board_completeness_class(layout, list(range(12))) == "full_board"
    assert board_completeness_class(layout, list(range(9))) == "partial_board_large"
    assert board_completeness_class(layout, list(range(6))) == "partial_board_medium"
    assert board_completeness_class(layout, [0, 3]) == "partial_board_small"
    assert board_completeness_class(layout, [9]) == "insufficient_board"
    assert board_completeness_class(layout, []) == "no_board"


def test_board_completeness_ignores_spurious_detected_ids(layout: BoardLayout) -> None:
    assert visible_board_layout_area_percent(layout, [999]) == 0.0
    assert visible_board_layout_area_percent(layout, [0, 1, 999]) == pytest.approx(
        visible_board_layout_area_percent(layout, [0, 1])
    )
    assert board_completeness_class(layout, [*range(12), 999]) == "full_board"
    assert board_completeness_class(layout, [999]) == "no_board"


def test_board_layout_geometry_rejects_swapped_detected_ids(layout: BoardLayout) -> None:
    corners_by_id = _layout_image_corners(layout, list(range(12)))
    corners_by_id[0], corners_by_id[1] = corners_by_id[1], corners_by_id[0]
    result = verify_board_layout_geometry(corners_by_id, layout)
    assert not result.ok
    assert result.layout_error_px_p95 > 3.0


def _square_corners(x: float, y: float, edge: float) -> np.ndarray:
    return _rect_corners(x, y, x + edge, y + edge)


def _rect_corners(x0: float, y0: float, x1: float, y1: float) -> np.ndarray:
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=np.float32)


def _layout_image_corners(layout: BoardLayout, visible_ids: list[int]) -> dict[int, np.ndarray]:
    return {
        tag_id: (layout.tags[tag_id].corners_m[:, :2] * 1000.0 + np.array([100.0, 50.0])).astype(
            np.float32
        )
        for tag_id in visible_ids
    }
