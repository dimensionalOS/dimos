from itertools import pairwise
import math
from unittest.mock import patch

import numpy as np

from dimos.mapping.occupancy.path_resampling import (
    ConstrainedPathSmoothingConfig,
    _path_cost_validation,
    _select_backtracked_path,
    constrained_smooth_resample_path,
    simple_resample_path,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path


def _path(points: list[tuple[float, float]]) -> Path:
    return Path(poses=[PoseStamped(position=[x, y, 0.0]) for x, y in points])


def _total_turn(path: Path) -> float:
    headings = [
        math.atan2(following.y - current.y, following.x - current.x)
        for current, following in pairwise(path.poses)
    ]
    return sum(abs(math.atan2(math.sin(b - a), math.cos(b - a))) for a, b in pairwise(headings))


def _costmap() -> OccupancyGrid:
    return OccupancyGrid(np.zeros((80, 80), dtype=np.int8), resolution=0.05)


def _assert_continuously_traversable(path: Path, costmap: OccupancyGrid) -> None:
    for start, end in pairwise(path.poses):
        length = start.position.distance(end.position)
        sample_count = max(1, math.ceil(length / (costmap.resolution / 2)))
        for index in range(sample_count + 1):
            ratio = index / sample_count
            point = start.position + ratio * (end.position - start.position)
            assert costmap.cell_value(point) < CostValues.OCCUPIED


def test_constrained_smoothing_reduces_local_zigzag() -> None:
    raw = _path([(0.2 + i * 0.1, 1.0 + (0.04 if i % 2 else -0.04)) for i in range(25)])
    smoothed = constrained_smooth_resample_path(
        raw,
        Pose(position=raw.poses[-1].position),
        _costmap(),
        ConstrainedPathSmoothingConfig(spacing_m=0.05),
    )

    assert _total_turn(smoothed) < _total_turn(raw) * 0.25
    assert smoothed.poses[0].position.distance(raw.poses[0].position) < 1e-9
    assert smoothed.poses[-1].position.distance(raw.poses[-1].position) < 1e-9


def test_constrained_smoothing_does_not_cut_through_lethal_cell() -> None:
    costmap = _costmap()
    costmap.grid[18:23, 28:33] = CostValues.OCCUPIED
    raw = _path([(0.5, 1.0), (1.1, 1.0), (1.1, 1.3), (1.9, 1.3), (1.9, 1.0), (2.5, 1.0)])
    smoothed = constrained_smooth_resample_path(
        raw,
        Pose(position=raw.poses[-1].position),
        costmap,
        ConstrainedPathSmoothingConfig(spacing_m=0.05, max_deviation_m=0.1),
    )

    _assert_continuously_traversable(smoothed, costmap)


def test_zero_iterations_preserves_raw_geometry() -> None:
    raw = _path([(0.2, 0.2), (0.3, 0.25), (0.4, 0.2)])
    result = constrained_smooth_resample_path(
        raw,
        Pose(position=raw.poses[-1].position),
        _costmap(),
        ConstrainedPathSmoothingConfig(spacing_m=0.05, max_iterations=0),
    )

    assert result.poses[0].position.distance(raw.poses[0].position) < 1e-9
    assert result.poses[-1].position.distance(raw.poses[-1].position) < 1e-9


def test_path_cost_validation_reports_rejection_reason() -> None:
    costmap = _costmap()
    costmap.grid[10, 10] = CostValues.OCCUPIED

    lethal_cost, lethal_reason = _path_cost_validation(
        np.array([[0.0, 0.0], [0.5, 0.5]]),
        costmap,
        0.025,
    )
    outside_cost, outside_reason = _path_cost_validation(
        np.array([[0.0, 0.0], [-0.1, 0.0]]),
        costmap,
        0.025,
    )

    assert lethal_cost is None
    assert lethal_reason == "lethal_cell"
    assert outside_cost is None
    assert outside_reason == "out_of_bounds"


def test_backtracking_selects_largest_valid_fraction() -> None:
    raw = _path([(0.2, 0.2), (0.3, 0.3), (0.4, 0.2)])
    original = np.array([[pose.x, pose.y] for pose in raw.poses])
    smoothed = original.copy()
    smoothed[1, 1] = 0.2
    goal = Pose(position=raw.poses[-1].position)
    config = ConstrainedPathSmoothingConfig(
        spacing_m=0.05,
        backtracking_factor=0.5,
        max_backtracking_steps=3,
    )

    with patch(
        "dimos.mapping.occupancy.path_resampling._path_cost_validation",
        side_effect=[(10.0, None), (13.0, None), (11.0, None)],
    ):
        result = _select_backtracked_path(raw, original, smoothed, goal, _costmap(), config)

    expected_points = original + 0.5 * (smoothed - original)
    expected = simple_resample_path(_path(expected_points.tolist()), goal, config.spacing_m)
    assert np.allclose(
        [[pose.x, pose.y] for pose in result.poses],
        [[pose.x, pose.y] for pose in expected.poses],
    )


def test_backtracking_uses_aligned_raw_baseline_after_all_fractions_fail() -> None:
    raw = _path([(0.2, 0.2), (0.3, 0.3), (0.4, 0.2)])
    original = np.array([[pose.x, pose.y] for pose in raw.poses])
    smoothed = original.copy()
    smoothed[1, 1] = 0.2
    goal = Pose(position=raw.poses[-1].position)
    config = ConstrainedPathSmoothingConfig(spacing_m=0.05, max_backtracking_steps=2)

    with patch(
        "dimos.mapping.occupancy.path_resampling._path_cost_validation",
        side_effect=[(10.0, None), (13.0, None), (12.5, None), (12.1, None)],
    ) as validate:
        result = _select_backtracked_path(raw, original, smoothed, goal, _costmap(), config)

    expected = simple_resample_path(raw, goal, config.spacing_m)
    assert np.allclose(
        [[pose.x, pose.y] for pose in result.poses],
        [[pose.x, pose.y] for pose in expected.poses],
    )
    baseline_points = validate.call_args_list[0].args[0]
    assert np.allclose(baseline_points, [[pose.x, pose.y] for pose in expected.poses])
