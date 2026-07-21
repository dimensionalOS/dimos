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

"""Coverage-guided capture: pure coverage/diversity functions + one guided-loop smoke test.

Every coverage function is pure and deterministic, so these tests fabricate board corner arrays
directly (no camera, no display) and assert exact behaviour. The single loop test drives the real
``_interactive_capture`` with synthetic chessboard frames so the guided auto-accept path is
exercised end to end.
"""

from __future__ import annotations

from itertools import pairwise

import cv2
import numpy as np
import pytest

from dimos.utils.cli.cameracalibrate.cameracalibrate import (
    _interactive_capture,
    board_coverage_params,
    compute_coverage,
    coverage_gate_accepts,
)

_IMAGE_WH = (640, 480)


def _rect_board_corners(
    x0: float, y0: float, w: float, h: float, *, nx: int = 3, ny: int = 3
) -> np.ndarray:
    """Axis-aligned ``nx`` x ``ny`` grid of board corners spanning [x0, x0+w] x [y0, y0+h].

    Axis-aligned means the outer quad is a rectangle, so the skew proxy is ~0 -- useful for
    isolating the skew axis as the weakest one.
    """
    xs = np.linspace(x0, x0 + w, nx)
    ys = np.linspace(y0, y0 + h, ny)
    pts = [[float(x), float(y)] for y in ys for x in xs]
    return np.asarray(pts, dtype=np.float32).reshape(-1, 1, 2)


def test_board_coverage_params_axis_aligned_rectangle_has_zero_skew_and_centroid() -> None:
    """A fronto-parallel rectangle: x/y at its centroid, size in (0,1), skew ~0."""
    corners = _rect_board_corners(220.0, 140.0, 200.0, 200.0)  # centroid (320, 240)
    x, y, size, skew = board_coverage_params(corners, _IMAGE_WH)

    assert x == pytest.approx(320.0 / 640.0, abs=1e-6)
    assert y == pytest.approx(240.0 / 480.0, abs=1e-6)
    assert 0.0 < size < 1.0
    assert skew == pytest.approx(0.0, abs=1e-6)


def test_compute_coverage_increases_monotonically_as_new_cells_are_filled() -> None:
    """Overall coverage % never decreases as views fill new grid cells, and strictly grows overall."""
    # Each view sits in a distinct quadrant / size, so it lights new grid cells and widens the
    # x/y/size sweep -> every axis is monotonically non-decreasing.
    views = [
        _rect_board_corners(40.0, 40.0, 90.0, 90.0),
        _rect_board_corners(430.0, 50.0, 110.0, 110.0),
        _rect_board_corners(50.0, 330.0, 100.0, 100.0),
        _rect_board_corners(410.0, 320.0, 200.0, 140.0),
    ]

    overalls = [
        compute_coverage(views[: i + 1], _IMAGE_WH).overall for i in range(len(views))
    ]

    # Monotonic non-decreasing across every prefix.
    assert all(b >= a - 1e-12 for a, b in pairwise(overalls)), overalls
    # And genuinely climbs from the first view to the last.
    assert overalls[-1] > overalls[0]


def test_compute_coverage_empty_is_zero() -> None:
    progress = compute_coverage([], _IMAGE_WH)
    assert progress.overall == 0.0
    assert progress.fill_fraction == 0.0


def test_gate_rejects_near_duplicate_and_accepts_new_region() -> None:
    """The acceptance gate drops a near-identical view but takes one that opens a new region."""
    first = _rect_board_corners(100.0, 100.0, 120.0, 120.0)
    accepted = [first]

    # Near-duplicate: same corners -> below the min corner-motion threshold -> rejected.
    near_duplicate = first.copy()
    assert coverage_gate_accepts(near_duplicate, accepted, _IMAGE_WH) is False

    # New region: shifted far into fresh grid cells -> accepted.
    new_region = _rect_board_corners(420.0, 330.0, 120.0, 120.0)
    assert coverage_gate_accepts(new_region, accepted, _IMAGE_WH) is True

    # First view is always accepted against an empty history.
    assert coverage_gate_accepts(first, [], _IMAGE_WH) is True


def test_guidance_names_the_weakest_axis_skew() -> None:
    """Views spread in x/y/size but all fronto-parallel -> skew is weakest and the hint says so."""
    views = [
        _rect_board_corners(50.0, 50.0, 100.0, 100.0),
        _rect_board_corners(430.0, 60.0, 120.0, 120.0),
        _rect_board_corners(60.0, 320.0, 110.0, 110.0),
        _rect_board_corners(400.0, 330.0, 200.0, 140.0),
    ]

    progress = compute_coverage(views, _IMAGE_WH)

    assert progress.skew_spread == pytest.approx(0.0, abs=1e-6)
    assert progress.weakest_axis == "skew"
    guidance = progress.guidance.lower()
    assert "skew" in guidance or "tilt" in guidance


def test_guidance_names_the_weakest_axis_fill_points_at_a_region() -> None:
    """A single small view leaves most of the frame empty -> the hint points somewhere concrete."""
    views = [_rect_board_corners(40.0, 40.0, 60.0, 60.0)]  # top-left corner only
    progress = compute_coverage(views, _IMAGE_WH)
    # x/y/skew spread are all 0 from one view; fill is small too -- guidance must name a direction.
    assert "move the board" in progress.guidance.lower()


# --- guided capture loop (real detector, synthetic frames, no display) --------


def _chessboard_tile(cols: int, rows: int, square_px: int) -> np.ndarray:
    """Binary chessboard tile with ``cols`` x ``rows`` inner corners ((cols+1)x(rows+1) squares)."""
    h = (rows + 1) * square_px
    w = (cols + 1) * square_px
    img = np.full((h, w), 255, dtype=np.uint8)
    for yi in range(rows + 1):
        for xi in range(cols + 1):
            if (xi + yi) % 2 == 0:
                img[yi * square_px : (yi + 1) * square_px, xi * square_px : (xi + 1) * square_px] = 0
    return img


def _frame_with_board_at(canvas_w: int, canvas_h: int, tile: np.ndarray, ox: int, oy: int) -> np.ndarray:
    canvas = np.full((canvas_h, canvas_w), 255, dtype=np.uint8)
    th, tw = tile.shape
    canvas[oy : oy + th, ox : ox + tw] = tile
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def test_interactive_capture_guided_auto_accepts_distinct_frames_without_space(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """Guided mode accepts diverse frames with NO SPACE key, and prints coverage under no_display."""
    cols, rows = 5, 4
    tile = _chessboard_tile(cols, rows, square_px=40)
    offsets = [(30, 30), (340, 30), (30, 250), (340, 250)]
    frames = [_frame_with_board_at(640, 480, tile, ox, oy) for ox, oy in offsets]

    # waitKey never returns SPACE/q; acceptance must come purely from the coverage gate.
    monkeypatch.setattr(cv2, "waitKey", lambda _delay=0: 0)

    frame_iter = iter(frames)

    def _next() -> np.ndarray | None:
        return next(frame_iter, None)

    result = _interactive_capture(
        _next, len(frames), cols, rows, no_display=True, coverage_guided=True
    )

    assert len(result.frames) == len(frames)
    assert len(result.image_points) == len(frames)
    # Guided no_display path prints a coverage/guidance line per accepted frame.
    out = capsys.readouterr().out
    assert "coverage" in out


def test_interactive_capture_guided_min_coverage_finishes_before_target(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """--min-coverage lets guided capture stop before target_count without raising."""
    cols, rows = 5, 4
    tile = _chessboard_tile(cols, rows, square_px=40)
    offsets = [(30, 30), (340, 30), (30, 250), (340, 250)]
    frames = [_frame_with_board_at(640, 480, tile, ox, oy) for ox, oy in offsets]

    monkeypatch.setattr(cv2, "waitKey", lambda _delay=0: 0)
    frame_iter = iter(frames)

    def _next() -> np.ndarray | None:
        return next(frame_iter, None)

    # target_count is a high hard cap; min_coverage=0.0 auto-finishes at the floor (>=3 frames).
    result = _interactive_capture(
        _next,
        50,
        cols,
        rows,
        no_display=True,
        coverage_guided=True,
        min_coverage=0.0,
    )

    assert 3 <= len(result.frames) < 50
