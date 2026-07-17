from itertools import pairwise
import math
from unittest.mock import patch

import numpy as np
import pytest

from dimos.mapping.occupancy.path_resampling import (
    ConstrainedPathSmoothingConfig,
    _effective_path_cost,
    _local_triple_path_cost,
    _path_cost_validation,
    _path_from_xy,
    _resample_xy,
    _resample_xy_array,
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


def _legacy_path_cost_validation(
    points: np.ndarray,
    costmap: OccupancyGrid,
    sample_spacing_m: float,
) -> tuple[float | None, str | None]:
    values: list[float] = []
    for segment_index, (start, end) in enumerate(pairwise(points)):
        length = float(np.linalg.norm(end - start))
        sample_count = max(1, math.ceil(length / sample_spacing_m))
        first_sample = 0 if segment_index == 0 else 1
        for sample_index in range(first_sample, sample_count + 1):
            ratio = sample_index / sample_count
            point = start + ratio * (end - start)
            grid_point = costmap.world_to_grid((float(point[0]), float(point[1]), 0.0))
            grid_x = math.floor(grid_point.x)
            grid_y = math.floor(grid_point.y)
            if not (0 <= grid_x < costmap.width and 0 <= grid_y < costmap.height):
                return None, "out_of_bounds"
            value = int(costmap.grid[grid_y, grid_x])
            if value >= CostValues.OCCUPIED:
                return None, "lethal_cell"
            values.append(80.0 if value == CostValues.UNKNOWN else max(0.0, float(value)))
    return (float(np.mean(values)) if values else 0.0), None


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


def test_constrained_smoothing_records_complete_phase_timing() -> None:
    raw = _path([(0.2 + i * 0.1, 1.0 + (0.04 if i % 2 else -0.04)) for i in range(25)])
    timing: dict[str, float | int] = {}

    constrained_smooth_resample_path(
        raw,
        Pose(position=raw.poses[-1].position),
        _costmap(),
        ConstrainedPathSmoothingConfig(spacing_m=0.05),
        timing,
    )

    assert timing.keys() == {
        "raw_path_points",
        "raw_path_length_m",
        "costmap_width",
        "costmap_height",
        "raw_validation_ms",
        "reference_costs_ms",
        "smoothing_loop_ms",
        "smoothing_iterations",
        "raw_resample_metrics_ms",
        "candidate_1_0_ms",
        "candidate_0_5_ms",
        "candidate_0_25_ms",
        "candidate_0_125_ms",
        "final_path_message_ms",
        "optimizer_total_ms",
        "path_publish_ms",
        "local_planner_handoff_ms",
    }
    assert timing["raw_path_points"] == len(raw.poses)
    assert timing["raw_path_length_m"] > 0
    assert timing["smoothing_iterations"] > 0
    assert timing["optimizer_total_ms"] > 0


@pytest.mark.parametrize(
    ("points", "spacing"),
    [
        ([(0.0, 0.0)], 0.1),
        ([(0.0, 0.0), (0.05, 0.0)], 0.1),
        ([(0.0, 0.0), (0.0, 0.3)], 0.1),
        ([(0.0, 0.0), (0.3, 0.3)], 0.07),
        ([(0.0, 0.0), (0.2, 0.0), (0.2, 0.0), (0.35, 0.1), (0.7, 0.1)], 0.1),
        ([(0.2, 0.3), (0.2, 0.3), (0.2, 0.3)], 0.1),
        ([(-0.7, -0.3), (-0.4, -0.1), (-0.2, -0.5), (0.1, -0.2)], 0.08),
    ],
)
def test_array_resampler_matches_message_resampler(points, spacing) -> None:
    source = _path(points)
    goal = Pose(position=source.poses[-1].position)
    expected = simple_resample_path(source, goal, spacing)

    actual = _resample_xy_array(np.asarray(points, dtype=np.float64), spacing)
    actual_path = _resample_xy(source, np.asarray(points, dtype=np.float64), goal, spacing)

    np.testing.assert_allclose(actual, [[pose.x, pose.y] for pose in expected.poses], atol=1e-12)
    np.testing.assert_allclose(
        [[pose.x, pose.y] for pose in actual_path.poses],
        [[pose.x, pose.y] for pose in expected.poses],
        atol=1e-12,
    )
    np.testing.assert_allclose(
        [
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            for pose in actual_path.poses
        ],
        [
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            for pose in expected.poses
        ],
        atol=1e-12,
    )


def test_array_resampler_matches_message_resampler_for_random_polylines() -> None:
    rng = np.random.default_rng(20260717)
    for point_count in (2, 3, 10, 100):
        points = np.cumsum(rng.normal(0.0, 0.1, size=(point_count, 2)), axis=0)
        points[point_count // 2] = points[max(0, point_count // 2 - 1)]
        source = _path([tuple(point) for point in points])
        expected = simple_resample_path(
            source,
            Pose(position=source.poses[-1].position),
            0.07,
        )

        actual = _resample_xy_array(points, 0.07)

        np.testing.assert_allclose(
            actual,
            [[pose.x, pose.y] for pose in expected.poses],
            atol=1e-12,
        )


def test_constrained_smoothing_constructs_one_final_path_message() -> None:
    raw = _path([(0.2 + i * 0.1, 1.0 + (0.04 if i % 2 else -0.04)) for i in range(25)])

    with patch(
        "dimos.mapping.occupancy.path_resampling._path_from_xy",
        wraps=_path_from_xy,
    ) as build_path:
        constrained_smooth_resample_path(
            raw,
            Pose(position=raw.poses[-1].position),
            _costmap(),
            ConstrainedPathSmoothingConfig(),
        )

    assert build_path.call_count == 1


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


def test_direct_path_cost_matches_legacy_on_seeded_grids_and_paths() -> None:
    rng = np.random.default_rng(20260717)
    for origin_xy in ((0.0, 0.0), (-2.35, 1.15), (4.2, -3.8)):
        grid = rng.integers(-1, 100, size=(30, 40), dtype=np.int8)
        grid[rng.random(grid.shape) < 0.08] = CostValues.OCCUPIED
        costmap = OccupancyGrid(
            grid=grid,
            resolution=0.1,
            origin=Pose(position=[origin_xy[0], origin_xy[1], 0.0]),
        )
        for point_count in (1, 2, 3, 12):
            for _ in range(25):
                points = rng.uniform((-0.15, -0.15), (4.05, 3.05), size=(point_count, 2))
                points += np.asarray(origin_xy)
                expected = _legacy_path_cost_validation(points, costmap, 0.05)
                actual = _path_cost_validation(points, costmap, 0.05)
                assert actual[1] == expected[1]
                if expected[0] is None:
                    assert actual[0] is None
                else:
                    assert actual[0] == pytest.approx(expected[0], abs=1e-12)


def test_direct_path_cost_matches_legacy_at_cell_boundaries() -> None:
    origin_xy = (-3.7, 2.4)
    costmap = OccupancyGrid(
        grid=np.zeros((8, 9), dtype=np.int8),
        resolution=0.1,
        origin=Pose(position=[origin_xy[0], origin_xy[1], 0.0]),
    )
    offsets = (
        -0.1,
        np.nextafter(0.0, -1.0),
        0.0,
        np.nextafter(0.1, 0.0),
        0.1,
        np.nextafter(0.1, 1.0),
        0.8,
        0.9,
    )
    for x_offset in offsets:
        for y_offset in offsets:
            point = np.asarray(
                [origin_xy[0] + x_offset, origin_xy[1] + y_offset],
                dtype=np.float64,
            )
            points = np.stack((point, point))
            assert _path_cost_validation(points, costmap, 0.05) == _legacy_path_cost_validation(
                points,
                costmap,
                0.05,
            )


def test_local_triple_cost_matches_general_validation() -> None:
    rng = np.random.default_rng(20260717)
    grid = rng.integers(-1, 100, size=(50, 60), dtype=np.int8)
    grid[rng.random(grid.shape) < 0.05] = CostValues.OCCUPIED
    costmap = OccupancyGrid(
        grid=grid,
        resolution=0.1,
        origin=Pose(position=[-2.5, 1.7, 0.0]),
    )
    context = (
        costmap.grid,
        costmap.origin.position.x,
        costmap.origin.position.y,
        costmap.resolution,
        costmap.width,
        costmap.height,
    )
    for _ in range(250):
        points = rng.uniform((-2.7, 1.5), (3.7, 6.9), size=(3, 2))
        expected = _effective_path_cost(points, costmap, 0.05)
        actual = _local_triple_path_cost(
            points[0],
            float(points[1, 0]),
            float(points[1, 1]),
            points[2],
            context,
            0.05,
        )
        if expected is None:
            assert actual is None
        else:
            assert actual == pytest.approx(expected, abs=1e-12)


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
