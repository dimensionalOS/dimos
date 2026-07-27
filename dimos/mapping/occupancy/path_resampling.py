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


from dataclasses import dataclass
from itertools import pairwise
import math
from time import perf_counter

import numpy as np
from scipy.ndimage import uniform_filter1d

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import euler_to_quaternion

logger = setup_logger()

_PATH_SMOOTHING_TIMING_FIELDS = (
    "raw_validation_ms",
    "reference_costs_ms",
    "smoothing_loop_ms",
    "raw_resample_metrics_ms",
    "candidate_1_0_ms",
    "candidate_0_5_ms",
    "candidate_0_25_ms",
    "candidate_0_125_ms",
    "final_path_message_ms",
    "optimizer_total_ms",
    "path_publish_ms",
    "local_planner_handoff_ms",
)


def _initialize_smoothing_timing(
    timing: dict[str, float | int] | None,
    path: Path,
    costmap: OccupancyGrid,
) -> None:
    if timing is None:
        return
    timing.clear()
    timing.update({field: 0.0 for field in _PATH_SMOOTHING_TIMING_FIELDS})
    timing["raw_path_points"] = len(path.poses)
    timing["raw_path_length_m"] = sum(
        math.hypot(following.x - current.x, following.y - current.y)
        for current, following in pairwise(path.poses)
    )
    timing["costmap_width"] = costmap.width
    timing["costmap_height"] = costmap.height
    timing["smoothing_iterations"] = 0


def _record_elapsed(
    timing: dict[str, float | int] | None,
    field: str,
    started: float,
) -> float:
    elapsed_ms = (perf_counter() - started) * 1000
    if timing is not None:
        timing[field] = elapsed_ms
    return elapsed_ms


@dataclass(frozen=True)
class ConstrainedPathSmoothingConfig:
    spacing_m: float = 0.1
    max_iterations: int = 40
    data_weight: float = 0.02
    smoothness_weight: float = 0.45
    max_deviation_m: float = 0.1
    collision_sample_spacing_m: float = 0.05
    max_cost_increase: float = 2.0
    backtracking_factor: float = 0.5
    max_backtracking_steps: int = 3

    def __post_init__(self) -> None:
        if self.spacing_m <= 0 or self.collision_sample_spacing_m <= 0:
            raise ValueError("Path smoothing sample spacing must be positive")
        if self.max_iterations < 0 or self.max_deviation_m < 0:
            raise ValueError("Path smoothing limits must be non-negative")
        if not 0 <= self.data_weight <= 1:
            raise ValueError("data_weight must be between 0 and 1")
        if not 0 <= self.smoothness_weight <= 0.5:
            raise ValueError("smoothness_weight must be between 0 and 0.5")
        if self.max_cost_increase < 0:
            raise ValueError("max_cost_increase must be non-negative")
        if not 0 < self.backtracking_factor < 1:
            raise ValueError("backtracking_factor must be between 0 and 1")
        if self.max_backtracking_steps < 0:
            raise ValueError("max_backtracking_steps must be non-negative")


def _add_orientations_to_path(path: Path, goal_orientation: Quaternion) -> None:
    """Add orientations to path poses based on direction of movement.

    Args:
        path: Path with poses to add orientations to
        goal_orientation: Desired orientation for the final pose

    Returns:
        Path with orientations added to all poses
    """
    if not path.poses or len(path.poses) < 2:
        return

    # Calculate orientations for all poses except the last one
    for i in range(len(path.poses) - 1):
        current_pose = path.poses[i]
        next_pose = path.poses[i + 1]

        # Calculate direction to next point
        dx = next_pose.position.x - current_pose.position.x
        dy = next_pose.position.y - current_pose.position.y

        # Calculate yaw angle
        yaw = math.atan2(dy, dx)

        # Convert to quaternion (roll=0, pitch=0, yaw)
        orientation = euler_to_quaternion(Vector3(0, 0, yaw))
        current_pose.orientation = orientation

    # Set last pose orientation
    identity_quat = Quaternion(0, 0, 0, 1)
    if goal_orientation != identity_quat:
        # Use the provided goal orientation if it's not the identity
        path.poses[-1].orientation = goal_orientation
    elif len(path.poses) > 1:
        # Use the previous pose's orientation
        path.poses[-1].orientation = path.poses[-2].orientation
    else:
        # Single pose with identity goal orientation
        path.poses[-1].orientation = identity_quat


# TODO: replace goal_pose with just goal_orientation
def simple_resample_path(path: Path, goal_pose: Pose, spacing: float) -> Path:
    """Resample a path to have approximately uniform spacing between poses.

    Args:
        path: The original Path
        spacing: Desired distance between consecutive poses

    Returns:
        A new Path with resampled poses
    """
    if len(path) < 2 or spacing <= 0:
        return path

    resampled = []
    resampled.append(path.poses[0])

    accumulated_distance = 0.0

    for i in range(1, len(path.poses)):
        current = path.poses[i]
        prev = path.poses[i - 1]

        # Calculate segment distance
        dx = current.x - prev.x
        dy = current.y - prev.y
        segment_length = (dx**2 + dy**2) ** 0.5

        if segment_length < 1e-10:
            continue

        # Direction vector
        dir_x = dx / segment_length
        dir_y = dy / segment_length

        # Add points along this segment
        while accumulated_distance + segment_length >= spacing:
            # Distance along segment for next point
            dist_along = spacing - accumulated_distance
            if dist_along < 0:
                break

            # Create new pose
            new_x = prev.x + dir_x * dist_along
            new_y = prev.y + dir_y * dist_along
            new_pose = PoseStamped(
                frame_id=path.frame_id,
                position=[new_x, new_y, 0.0],
                orientation=prev.orientation,  # Keep same orientation
            )
            resampled.append(new_pose)

            # Update for next iteration
            accumulated_distance = 0
            segment_length -= dist_along
            prev = new_pose

        accumulated_distance += segment_length

    # Add last pose if not already there
    if len(path.poses) > 1:
        last = path.poses[-1]
        if not resampled or (resampled[-1].x != last.x or resampled[-1].y != last.y):
            resampled.append(last)

    ret = Path(frame_id=path.frame_id, poses=resampled)

    _add_orientations_to_path(ret, goal_pose.orientation)

    return ret


def _path_cost_validation(
    points: np.ndarray,
    costmap: OccupancyGrid,
    sample_spacing_m: float,
) -> tuple[float | None, str | None]:
    """Return mean traversable cost and a failure reason when invalid."""
    if len(points) < 2:
        return 0.0, None

    origin_x = costmap.origin.position.x
    origin_y = costmap.origin.position.y
    resolution = costmap.resolution
    width = costmap.width
    height = costmap.height
    grid = costmap.grid
    total_cost = 0.0
    value_count = 0
    for segment_index in range(len(points) - 1):
        start_x = float(points[segment_index, 0])
        start_y = float(points[segment_index, 1])
        delta_x = float(points[segment_index + 1, 0]) - start_x
        delta_y = float(points[segment_index + 1, 1]) - start_y
        length = (delta_x**2 + delta_y**2) ** 0.5
        sample_count = max(1, math.ceil(length / sample_spacing_m))
        first_sample = 0 if segment_index == 0 else 1
        for sample_index in range(first_sample, sample_count + 1):
            ratio = sample_index / sample_count
            point_x = start_x + ratio * delta_x
            point_y = start_y + ratio * delta_y
            grid_x = math.floor((point_x - origin_x) / resolution)
            grid_y = math.floor((point_y - origin_y) / resolution)
            if not (0 <= grid_x < width and 0 <= grid_y < height):
                return None, "out_of_bounds"

            value = int(grid[grid_y, grid_x])
            if value >= CostValues.OCCUPIED:
                return None, "lethal_cell"
            # Match min_cost_astar's default unknown penalty: 0.8 * 100.
            total_cost += 80.0 if value == CostValues.UNKNOWN else max(0.0, float(value))
            value_count += 1

    return (total_cost / value_count if value_count else 0.0), None


def _effective_path_cost(
    points: np.ndarray,
    costmap: OccupancyGrid,
    sample_spacing_m: float,
) -> float | None:
    return _path_cost_validation(points, costmap, sample_spacing_m)[0]


def _local_triple_path_cost(
    previous: np.ndarray,
    candidate_x: float,
    candidate_y: float,
    following: np.ndarray,
    grid_context: tuple[np.ndarray, float, float, float, int, int],
    sample_spacing_m: float,
) -> float | None:
    grid, origin_x, origin_y, resolution, width, height = grid_context
    total_cost = 0.0
    value_count = 0
    for segment_index in range(2):
        if segment_index == 0:
            start_x = float(previous[0])
            start_y = float(previous[1])
            end_x = candidate_x
            end_y = candidate_y
        else:
            start_x = candidate_x
            start_y = candidate_y
            end_x = float(following[0])
            end_y = float(following[1])
        delta_x = end_x - start_x
        delta_y = end_y - start_y
        length = (delta_x**2 + delta_y**2) ** 0.5
        sample_count = max(1, math.ceil(length / sample_spacing_m))
        first_sample = 0 if segment_index == 0 else 1
        for sample_index in range(first_sample, sample_count + 1):
            ratio = sample_index / sample_count
            point_x = start_x + ratio * delta_x
            point_y = start_y + ratio * delta_y
            grid_x = math.floor((point_x - origin_x) / resolution)
            grid_y = math.floor((point_y - origin_y) / resolution)
            if not (0 <= grid_x < width and 0 <= grid_y < height):
                return None
            value = int(grid[grid_y, grid_x])
            if value >= CostValues.OCCUPIED:
                return None
            total_cost += 80.0 if value == CostValues.UNKNOWN else max(0.0, float(value))
            value_count += 1
    return total_cost / value_count if value_count else 0.0


def _path_from_xy(path: Path, points: np.ndarray) -> Path:
    return Path(
        frame_id=path.frame_id,
        poses=[
            PoseStamped(
                frame_id=path.frame_id,
                position=[float(point[0]), float(point[1]), 0.0],
                orientation=Quaternion(0, 0, 0, 1),
            )
            for point in points
        ],
    )


def _resample_xy_array(points: np.ndarray, spacing_m: float) -> np.ndarray:
    """Resample XY geometry without constructing message objects."""
    if len(points) < 2 or spacing_m <= 0:
        return points.copy()

    # Preserve legacy arithmetic order so boundary-cell decisions stay exact.
    resampled = [(float(points[0, 0]), float(points[0, 1]))]
    accumulated_distance = 0.0
    for index in range(1, len(points)):
        current_x = float(points[index, 0])
        current_y = float(points[index, 1])
        previous_x = float(points[index - 1, 0])
        previous_y = float(points[index - 1, 1])
        dx = current_x - previous_x
        dy = current_y - previous_y
        segment_length = (dx**2 + dy**2) ** 0.5
        if segment_length < 1e-10:
            continue

        direction_x = dx / segment_length
        direction_y = dy / segment_length
        while accumulated_distance + segment_length >= spacing_m:
            distance_along = spacing_m - accumulated_distance
            if distance_along < 0:
                break
            previous_x += direction_x * distance_along
            previous_y += direction_y * distance_along
            resampled.append((previous_x, previous_y))
            accumulated_distance = 0.0
            segment_length -= distance_along

        accumulated_distance += segment_length

    final_point = (float(points[-1, 0]), float(points[-1, 1]))
    if resampled[-1] != final_point:
        resampled.append(final_point)
    return np.asarray(resampled, dtype=np.float64)


def _finalize_xy_path(
    source_path: Path,
    points: np.ndarray,
    goal_pose: Pose,
    timing: dict[str, float | int] | None,
) -> Path:
    started = perf_counter()
    result = _path_from_xy(source_path, points)
    _add_orientations_to_path(result, goal_pose.orientation)
    _record_elapsed(timing, "final_path_message_ms", started)
    return result


def _resample_xy(
    source_path: Path,
    points: np.ndarray,
    goal_pose: Pose,
    spacing_m: float,
) -> Path:
    return _finalize_xy_path(
        source_path,
        _resample_xy_array(points, spacing_m),
        goal_pose,
        None,
    )


def _select_backtracked_path(
    source_path: Path,
    original: np.ndarray,
    smoothed: np.ndarray,
    goal_pose: Pose,
    costmap: OccupancyGrid,
    config: ConstrainedPathSmoothingConfig,
    timing: dict[str, float | int] | None = None,
) -> Path:
    raw_metrics_started = perf_counter()
    raw_resampled_points = _resample_xy_array(original, config.spacing_m)
    baseline_cost, baseline_failure_reason = _path_cost_validation(
        raw_resampled_points,
        costmap,
        config.collision_sample_spacing_m,
    )
    _record_elapsed(timing, "raw_resample_metrics_ms", raw_metrics_started)
    if baseline_cost is None:
        logger.warning(
            "Raw-resampled baseline failed path validation; using raw-resampled A* path.",
            reason=baseline_failure_reason,
            raw_points=len(original),
            baseline_points=len(raw_resampled_points),
        )
        return _finalize_xy_path(source_path, raw_resampled_points, goal_pose, timing)

    max_allowed_cost = baseline_cost + config.max_cost_increase
    full_offset = smoothed - original
    fractions = [
        config.backtracking_factor**step for step in range(config.max_backtracking_steps + 1)
    ]
    rejected_fractions: list[float] = []

    for fraction in fractions:
        candidate_started = perf_counter()
        blended = original + fraction * full_offset
        candidate_points = _resample_xy_array(blended, config.spacing_m)
        candidate_cost, failure_reason = _path_cost_validation(
            candidate_points,
            costmap,
            config.collision_sample_spacing_m,
        )
        rejection_reason = failure_reason
        if candidate_cost is not None and candidate_cost > max_allowed_cost:
            rejection_reason = "cost_increase"
        if timing is not None:
            timing[f"candidate_{str(fraction).replace('.', '_')}_ms"] = (
                perf_counter() - candidate_started
            ) * 1000

        if rejection_reason is None:
            assert candidate_cost is not None
            logger.info(
                "Constrained path smoothing accepted.",
                selected_fraction=round(fraction, 4),
                baseline_cost=round(baseline_cost, 3),
                candidate_cost=round(candidate_cost, 3),
                max_allowed_cost=round(max_allowed_cost, 3),
                rejected_fractions=rejected_fractions,
                raw_points=len(original),
                candidate_points=len(candidate_points),
                max_deviation_m=round(
                    float(np.max(np.linalg.norm(blended - original, axis=1))),
                    3,
                ),
            )
            return _finalize_xy_path(source_path, candidate_points, goal_pose, timing)

        rejected_fractions.append(round(fraction, 4))
        logger.info(
            "Constrained path smoothing fraction rejected.",
            fraction=round(fraction, 4),
            reason=rejection_reason,
            baseline_cost=round(baseline_cost, 3),
            candidate_cost=None if candidate_cost is None else round(candidate_cost, 3),
            max_allowed_cost=round(max_allowed_cost, 3),
        )

    logger.warning(
        "All constrained smoothing fractions failed; using raw-resampled A* path.",
        rejected_fractions=rejected_fractions,
        baseline_cost=round(baseline_cost, 3),
        max_allowed_cost=round(max_allowed_cost, 3),
        raw_points=len(original),
    )
    return _finalize_xy_path(source_path, raw_resampled_points, goal_pose, timing)


def constrained_smooth_resample_path(
    path: Path,
    goal_pose: Pose,
    costmap: OccupancyGrid,
    config: ConstrainedPathSmoothingConfig,
    timing: dict[str, float | int] | None = None,
) -> Path:
    """Locally smooth a grid path while preserving its costmap corridor."""
    optimizer_started = perf_counter()
    _initialize_smoothing_timing(timing, path, costmap)
    try:
        if len(path) < 3 or config.max_iterations == 0 or config.max_deviation_m == 0:
            started = perf_counter()
            result = simple_resample_path(path, goal_pose, config.spacing_m)
            _record_elapsed(timing, "final_path_message_ms", started)
            return result

        original = np.array([[pose.x, pose.y] for pose in path.poses], dtype=np.float64)
        duplicate = np.linalg.norm(np.diff(original, axis=0), axis=1) <= 1e-10
        original = original[np.concatenate(([True], ~duplicate))]
        if len(original) < 3:
            started = perf_counter()
            result = simple_resample_path(path, goal_pose, config.spacing_m)
            _record_elapsed(timing, "final_path_message_ms", started)
            return result

        started = perf_counter()
        raw_cost, raw_failure_reason = _path_cost_validation(
            original,
            costmap,
            config.collision_sample_spacing_m,
        )
        _record_elapsed(timing, "raw_validation_ms", started)
        if raw_cost is None:
            started = perf_counter()
            result = simple_resample_path(path, goal_pose, config.spacing_m)
            _record_elapsed(timing, "final_path_message_ms", started)
            logger.warning(
                "Raw A* path failed constrained-smoothing validation; skipping smoothing.",
                reason=raw_failure_reason,
                raw_points=len(original),
            )
            return result

        smoothed = original.copy()
        started = perf_counter()
        reference_costs = [
            _effective_path_cost(
                original[index - 1 : index + 2],
                costmap,
                config.collision_sample_spacing_m,
            )
            for index in range(1, len(original) - 1)
        ]
        _record_elapsed(timing, "reference_costs_ms", started)
        grid_context = (
            costmap.grid,
            costmap.origin.position.x,
            costmap.origin.position.y,
            costmap.resolution,
            costmap.width,
            costmap.height,
        )

        started = perf_counter()
        for iteration in range(config.max_iterations):
            if timing is not None:
                timing["smoothing_iterations"] = iteration + 1
            max_change = 0.0
            for index in range(1, len(smoothed) - 1):
                current_x = float(smoothed[index, 0])
                current_y = float(smoothed[index, 1])
                candidate_x = current_x + config.data_weight * (
                    float(original[index, 0]) - current_x
                )
                candidate_y = current_y + config.data_weight * (
                    float(original[index, 1]) - current_y
                )
                candidate_x += config.smoothness_weight * (
                    float(smoothed[index - 1, 0]) + float(smoothed[index + 1, 0]) - 2 * current_x
                )
                candidate_y += config.smoothness_weight * (
                    float(smoothed[index - 1, 1]) + float(smoothed[index + 1, 1]) - 2 * current_y
                )

                offset_x = candidate_x - float(original[index, 0])
                offset_y = candidate_y - float(original[index, 1])
                offset_length = (offset_x**2 + offset_y**2) ** 0.5
                if offset_length > config.max_deviation_m:
                    scale = config.max_deviation_m / offset_length
                    candidate_x = float(original[index, 0]) + offset_x * scale
                    candidate_y = float(original[index, 1]) + offset_y * scale

                reference_cost = reference_costs[index - 1]
                candidate_cost = _local_triple_path_cost(
                    smoothed[index - 1],
                    candidate_x,
                    candidate_y,
                    smoothed[index + 1],
                    grid_context,
                    config.collision_sample_spacing_m,
                )
                if (
                    reference_cost is None
                    or candidate_cost is None
                    or candidate_cost > reference_cost + config.max_cost_increase
                ):
                    continue

                change = ((candidate_x - current_x) ** 2 + (candidate_y - current_y) ** 2) ** 0.5
                smoothed[index, 0] = candidate_x
                smoothed[index, 1] = candidate_y
                max_change = max(max_change, change)

            if max_change < 1e-4:
                break
        _record_elapsed(timing, "smoothing_loop_ms", started)

        return _select_backtracked_path(
            path,
            original,
            smoothed,
            goal_pose,
            costmap,
            config,
            timing,
        )
    finally:
        _record_elapsed(timing, "optimizer_total_ms", optimizer_started)


def smooth_resample_path(
    path: Path, goal_pose: Pose, spacing: float, smoothing_window: int = 100
) -> Path:
    """Resample a path with smoothing to reduce jagged corners and abrupt turns.

    This produces smoother paths than simple_resample_path by:
    - First upsampling the path to have many points
    - Applying a moving average filter to smooth the coordinates
    - Resampling at the desired spacing
    - Keeping start and end points fixed

    Args:
        path: The original Path
        goal_pose: Goal pose with desired final orientation
        spacing: Desired approximate distance between consecutive poses
        smoothing_window: Size of the smoothing window (larger = smoother)

    Returns:
        A new Path with smoothly resampled poses
    """

    if len(path.poses) == 1:
        p = path.poses[0].position
        o = goal_pose.orientation
        new_pose = PoseStamped(
            frame_id=path.frame_id,
            position=[p.x, p.y, p.z],
            orientation=[o.x, o.y, o.z, o.w],
        )
        return Path(frame_id=path.frame_id, poses=[new_pose])

    if len(path) < 2 or spacing <= 0:
        return path

    # Extract x, y coordinates from path
    xs = np.array([p.x for p in path.poses])
    ys = np.array([p.y for p in path.poses])

    # Remove duplicate consecutive points
    diffs = np.sqrt(np.diff(xs) ** 2 + np.diff(ys) ** 2)
    valid_mask = np.concatenate([[True], diffs > 1e-10])
    xs = xs[valid_mask]
    ys = ys[valid_mask]

    if len(xs) < 2:
        return path

    # Calculate total path length
    dx = np.diff(xs)
    dy = np.diff(ys)
    segment_lengths = np.sqrt(dx**2 + dy**2)
    total_length = np.sum(segment_lengths)

    if total_length < spacing:
        return path

    # Upsample: create many points along the original path using linear interpolation
    # This gives us enough points for effective smoothing
    upsample_factor = 10
    num_upsampled = max(len(xs) * upsample_factor, 100)

    arc_length = np.concatenate([[0], np.cumsum(segment_lengths)])
    upsample_distances = np.linspace(0, total_length, num_upsampled)

    # Linear interpolation along arc length
    xs_upsampled = np.interp(upsample_distances, arc_length, xs)
    ys_upsampled = np.interp(upsample_distances, arc_length, ys)

    # Apply moving average smoothing
    # Use 'nearest' mode to avoid shrinking at boundaries
    window = min(smoothing_window, len(xs_upsampled) // 3)
    if window >= 3:
        xs_smooth = uniform_filter1d(xs_upsampled, size=window, mode="nearest")
        ys_smooth = uniform_filter1d(ys_upsampled, size=window, mode="nearest")
    else:
        xs_smooth = xs_upsampled
        ys_smooth = ys_upsampled

    # Keep start and end points exactly as original
    xs_smooth[0] = xs[0]
    ys_smooth[0] = ys[0]
    xs_smooth[-1] = xs[-1]
    ys_smooth[-1] = ys[-1]

    # Recalculate arc length on smoothed path
    dx_smooth = np.diff(xs_smooth)
    dy_smooth = np.diff(ys_smooth)
    segment_lengths_smooth = np.sqrt(dx_smooth**2 + dy_smooth**2)
    arc_length_smooth = np.concatenate([[0], np.cumsum(segment_lengths_smooth)])
    total_length_smooth = arc_length_smooth[-1]

    # Resample at desired spacing
    num_samples = max(2, int(np.ceil(total_length_smooth / spacing)) + 1)
    sample_distances = np.linspace(0, total_length_smooth, num_samples)

    # Interpolate to get final points
    sampled_x = np.interp(sample_distances, arc_length_smooth, xs_smooth)
    sampled_y = np.interp(sample_distances, arc_length_smooth, ys_smooth)

    # Create resampled poses
    resampled = []
    for i in range(len(sampled_x)):
        new_pose = PoseStamped(
            frame_id=path.frame_id,
            position=[float(sampled_x[i]), float(sampled_y[i]), 0.0],
            orientation=Quaternion(0, 0, 0, 1),
        )
        resampled.append(new_pose)

    ret = Path(frame_id=path.frame_id, poses=resampled)

    _add_orientations_to_path(ret, goal_pose.orientation)

    return ret
