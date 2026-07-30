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

"""How a review crop's frame is chosen, without the replay.

Rendering an image needs a recording; deciding *which frame* to render needs
nothing but numbers, and that decision is the whole content of the change these
cover. Each of the picker's parts is here as a separate claim: which frames are
candidates at all, how the window is sized to the object, what the LiDAR sweep
has to say about the marker before a frame may be shown, and what happens when
no frame qualifies -- because the answer there is "ship one crop and say why",
not "ship the best of a bad set silently".

The crops themselves are not asserted anywhere: they are presentation, and the
reference table is byte-identical without them (see ``crops`` module docstring).
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.agents.evals.crops import (
    CROP_DEFAULT_WINDOW_PX,
    CROP_MAX_WINDOW_PX,
    CROP_MIN_GRAY,
    CROP_MIN_GRAY_FALLBACK,
    CROP_MIN_MARKER_POINTS,
    CROP_MIN_SUPPORT_POINTS,
    CROP_MIN_WINDOW_PX,
    CropCandidate,
    MarkerSupport,
    laplacian_sharpness,
    marker_support,
    merge_windows,
    pick_identification_frame,
    sharpness_window_px,
    window_region,
    yaw_rates,
)


def projection(
    rows: list[tuple[tuple[float, float], float, tuple[float, float, float]]],
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Build the index-aligned ``(uv, depth, world)`` triple a projection returns."""
    uv = np.array([row[0] for row in rows], dtype=np.float64)
    depth = np.array([row[1] for row in rows], dtype=np.float64)
    world = np.array([row[2] for row in rows], dtype=np.float64)
    return uv, depth, world


def pose_at(yaw: float) -> tuple[float, ...]:
    """A recorded pose ``(x, y, z, qx, qy, qz, qw)`` with only yaw applied."""
    return (0.0, 0.0, 0.0, 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_candidate(
    ts: float = 100.0,
    sharpness: float = 50.0,
    mean_gray: float = 80.0,
    n_support: int = CROP_MIN_SUPPORT_POINTS,
    occluded: bool | None = False,
    yaw_rate: float | None = 0.1,
) -> CropCandidate:
    """One fully measured candidate frame, i.e. an input to the ranking."""
    return CropCandidate(
        ts=ts,
        marker=(640, 360),
        marker_depth_m=2.0,
        mean_gray=mean_gray,
        sharpness=sharpness,
        window_px=300,
        region=(490, 210, 790, 510),
        support=MarkerSupport(
            n_support=n_support,
            n_at_marker=5,
            seen_depth_m=2.0,
            occluded=occluded,
            support_uv=(),
        ),
        dt_to_detection_s=0.2,
        yaw_rate=yaw_rate,
    )


def test_overlapping_candidate_windows_are_merged_into_one_span() -> None:
    """Two detections less than a window apart share frames, and must be queried once.

    Querying both windows separately would measure -- and could rank -- the same
    frame twice.
    """
    assert merge_windows([10.0, 10.5, 30.0], 1.0) == [(9.0, 11.5), (29.0, 31.0)]
    assert merge_windows([10.0], 1.0) == [(9.0, 11.0)]
    assert merge_windows([], 1.0) == []


def test_the_sharpness_window_follows_the_objects_apparent_size() -> None:
    """Twice as far away is half as wide, and the fallback is used when it cannot be."""
    bbox = (0.0, 0.0, 600.0, 400.0)  # 600 px across, measured at 2 m
    assert sharpness_window_px(bbox, 2.0, 2.0) == 600
    assert sharpness_window_px(bbox, 2.0, 4.0) == 600 // 2
    # Clamped at both ends rather than tracking the ratio to absurdity.
    assert sharpness_window_px(bbox, 2.0, 20.0) == CROP_MIN_WINDOW_PX
    assert sharpness_window_px(bbox, 2.0, 0.5) == CROP_MAX_WINDOW_PX
    # No usable depth, or a degenerate box: the documented fixed fallback.
    assert sharpness_window_px(bbox, 0.0, 2.0) == CROP_DEFAULT_WINDOW_PX
    assert sharpness_window_px(bbox, 2.0, -1.0) == CROP_DEFAULT_WINDOW_PX
    assert sharpness_window_px((10.0, 10.0, 10.0, 10.0), 2.0, 2.0) == CROP_DEFAULT_WINDOW_PX


def test_a_window_near_the_rim_is_shifted_rather_than_clipped() -> None:
    """A marker at the edge still gets a full-size window, so it is ranked on the object."""
    assert window_region((640, 360), 300, 1280, 720) == (490, 210, 790, 510)
    assert window_region((20, 20), 300, 1280, 720) == (0, 0, 300, 300)
    assert window_region((1270, 700), 300, 1280, 720) == (980, 420, 1280, 720)
    # Wider than the image: the biggest square that fits, never a region outside it.
    assert window_region((640, 360), 2000, 1280, 720) == (280, 0, 1000, 720)


def test_yaw_rate_ignores_a_stale_pose_and_a_gap_between_windows() -> None:
    """The tiebreak has to measure the camera turning, not the pose stream lagging.

    Adjacent colour frames arrive faster than the recorded pose updates, so the
    nearest neighbour at all is often the *same* pose a moment later, which reads
    as a perfectly steady camera. Differencing over at least
    ``CROP_POSE_MIN_DT_S`` is what stops that; ``CROP_POSE_MAX_DT_S`` stops a
    difference from spanning two candidate windows.
    """
    samples = [
        (0.00, pose_at(0.0)),
        (0.05, pose_at(0.0)),  # the same pose again, 0.05 s later
        (0.20, pose_at(0.4)),  # 0.15 s after that stale sample
        (5.00, pose_at(0.0)),  # next window: no usable neighbour
    ]
    rates = yaw_rates(samples)
    assert rates[0] is None
    # Not 0.0 rad/s over 0.05 s: that interval is refused, so nothing is reported.
    assert rates[1] is None
    assert rates[2] == pytest.approx(0.4 / 0.15, abs=1e-6)
    assert rates[3] is None


def test_yaw_rate_is_small_across_the_pi_branch_cut() -> None:
    """A robot facing backwards is not turning at 6 rad/s."""
    before = pose_at(math.pi - 0.05)
    after = pose_at(-math.pi + 0.05)
    rates = yaw_rates([(0.0, before), (0.5, after)])
    assert rates[1] == pytest.approx(0.1 / 0.5, abs=1e-6)


def test_marker_support_counts_only_points_that_are_both_near_and_visible() -> None:
    """Support is 3-D proximity to the reference *and* presence in the shipped window."""
    region = (0, 0, 100, 100)
    reference = (1.0, 0.0, 0.5)
    uv, depth, world = projection(
        [
            ((50.0, 50.0), 2.0, (1.0, 0.0, 0.5)),  # on the reference, in the window
            ((50.0, 60.0), 2.0, (1.1, 0.0, 0.5)),  # near it, in the window
            ((500.0, 50.0), 2.0, (1.0, 0.05, 0.5)),  # near it, outside the window
            ((60.0, 60.0), 2.0, (4.0, 0.0, 0.5)),  # in the window, far from it
        ]
    )
    support = marker_support(uv, depth, world, reference, (50, 50), region, 2.0)
    assert support.n_support == 2
    assert support.support_uv == ((50.0, 50.0), (50.0, 60.0))


@pytest.mark.parametrize(
    ("seen_depth_m", "expected"),
    [
        (2.0, False),  # the sweep sees the object itself
        (2.2, False),  # behind the reference: not something in front of it
        (1.0, True),  # a metre of something in between
    ],
)
def test_the_occlusion_guard_reads_the_depth_at_the_marker(
    seen_depth_m: float, expected: bool
) -> None:
    uv, depth, world = projection(
        [((50.0, 50.0), seen_depth_m, (1.0, 0.0, 0.5))] * CROP_MIN_MARKER_POINTS
    )
    support = marker_support(uv, depth, world, (1.0, 0.0, 0.5), (50, 50), (0, 0, 100, 100), 2.0)
    assert support.seen_depth_m == pytest.approx(seen_depth_m)
    assert support.occluded is expected


def test_too_few_points_at_the_marker_is_unknown_rather_than_clear() -> None:
    """One stray return is not a depth measurement, and must not read as one."""
    uv, depth, world = projection([((50.0, 50.0), 0.5, (1.0, 0.0, 0.5))])
    support = marker_support(uv, depth, world, (1.0, 0.0, 0.5), (50, 50), (0, 0, 100, 100), 2.0)
    assert support.n_at_marker == 1
    assert support.seen_depth_m is None
    assert support.occluded is None


def test_an_empty_sweep_supports_nothing() -> None:
    """The shapes ``Projector.project`` documents for a frame with nothing in view."""
    uv, depth, world = np.zeros((0, 2)), np.zeros(0), np.zeros((0, 3))
    support = marker_support(uv, depth, world, (1.0, 0.0, 0.5), (50, 50), (0, 0, 100, 100), 2.0)
    assert (support.n_support, support.n_at_marker, support.occluded) == (0, 0, None)
    assert support.support_uv == ()


def test_the_sharpest_confirmed_and_bright_candidate_wins() -> None:
    """Sharpness ranks, but only among frames that are confirmed *and* bright.

    The two rejected candidates are the two failure modes the picker exists for:
    a frame whose marker no LiDAR return vouches for, and a dark frame whose
    Laplacian variance is streaks rather than detail.
    """
    unconfirmed = make_candidate(ts=1.0, sharpness=900.0, n_support=CROP_MIN_SUPPORT_POINTS - 1)
    dark = make_candidate(ts=2.0, sharpness=800.0, mean_gray=CROP_MIN_GRAY - 1)
    occluded = make_candidate(ts=3.0, sharpness=700.0, occluded=True)
    winner = make_candidate(ts=4.0, sharpness=120.0)
    best, note = pick_identification_frame([unconfirmed, dark, occluded, winner, make_candidate()])
    assert best is winner
    assert note == ""


def test_a_tie_on_sharpness_is_broken_by_the_steadier_frame() -> None:
    """Equal measured sharpness must not be resolved by iteration order."""
    steady = make_candidate(ts=9.0, yaw_rate=0.01)
    turning = make_candidate(ts=2.0, yaw_rate=1.5)
    unknown = make_candidate(ts=1.0, yaw_rate=None)
    assert pick_identification_frame([turning, steady, unknown])[0] is steady
    assert pick_identification_frame([unknown, steady, turning])[0] is steady
    # No yaw rate at all anywhere: the earliest frame, not whichever came first.
    late, early = make_candidate(ts=9.0, yaw_rate=None), make_candidate(ts=1.0, yaw_rate=None)
    assert pick_identification_frame([late, early])[0] is early


def test_a_dim_pick_is_reported_rather_than_passed_off_as_a_normal_one() -> None:
    """Nothing bright enough: the pipeline floor is used, and the crop is flagged."""
    dim = make_candidate(mean_gray=CROP_MIN_GRAY - 10.0)
    best, note = pick_identification_frame([dim])
    assert best is dim
    assert "pipeline floor" in note


@pytest.mark.parametrize(
    ("candidates", "expected_note"),
    [
        ([], "no candidate frame has LiDAR support"),
        ([make_candidate(n_support=0)], "no candidate frame has LiDAR support"),
        ([make_candidate(mean_gray=CROP_MIN_GRAY_FALLBACK - 1)], "brightness floor"),
    ],
)
def test_no_identification_crop_is_written_rather_than_a_meaningless_one(
    candidates: list[CropCandidate], expected_note: str
) -> None:
    """A reference with no checkable frame ships one crop, and says why.

    The absence is itself the review signal: nothing in the recording shows that
    marker with geometry behind it.
    """
    best, note = pick_identification_frame(candidates)
    assert best is None
    assert expected_note in note


def test_sharpness_is_measured_where_the_object_is() -> None:
    """A frame sharp in one corner and smeared in the other is not one number."""
    detailed = np.tile(np.array([[0, 255], [255, 0]], dtype=np.uint8), (50, 50))
    frame = np.zeros((200, 200), dtype=np.uint8)
    frame[:100, :100] = detailed
    frame[100:, 100:] = 128  # flat: no detail at all
    assert laplacian_sharpness(frame, (0, 0, 100, 100)) > 1000.0
    assert laplacian_sharpness(frame, (100, 100, 200, 200)) == 0.0
    assert laplacian_sharpness(frame, (0, 0, 0, 0)) == 0.0
