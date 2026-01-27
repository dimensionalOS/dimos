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

"""
Path Utilities

Standalone utility functions for path manipulation and post-processing.
These functions are stateless and can be used by any planner implementation.

## Functions

- interpolate_path(): Interpolate path to uniform resolution
- interpolate_segment(): Interpolate between two configurations
- simplify_path(): Remove unnecessary waypoints (requires WorldSpec)
- compute_path_length(): Compute total path length in joint space
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec import JointPath, WorldRobotID, WorldSpec


def interpolate_path(
    path: JointPath,
    resolution: float = 0.05,
) -> JointPath:
    """Interpolate path to have uniform resolution.

    Adds intermediate waypoints so that the maximum joint-space distance
    between consecutive waypoints is at most `resolution`.

    Args:
        path: Original path (list of joint configurations)
        resolution: Maximum distance between waypoints (radians)

    Returns:
        Interpolated path with more waypoints

    Example:
        # After planning, interpolate for smoother execution
        raw_path = planner.plan_joint_path(world, robot_id, q_start, q_goal).path
        smooth_path = interpolate_path(raw_path, resolution=0.02)
    """
    if len(path) <= 1:
        return path

    interpolated = [path[0]]

    for i in range(len(path) - 1):
        start = path[i]
        end = path[i + 1]

        diff = end - start
        max_diff = float(np.max(np.abs(diff)))

        if max_diff <= resolution:
            interpolated.append(end)
        else:
            num_steps = int(np.ceil(max_diff / resolution))
            for step in range(1, num_steps + 1):
                alpha = step / num_steps
                interpolated.append(start + alpha * diff)

    return interpolated


def interpolate_segment(
    start: NDArray[np.float64],
    end: NDArray[np.float64],
    step_size: float,
) -> list[NDArray[np.float64]]:
    """Interpolate between two configurations.

    Returns a list of configurations from start to end (inclusive)
    with at most `step_size` distance between consecutive points.

    Args:
        start: Start joint configuration
        end: End joint configuration
        step_size: Maximum step size (radians)

    Returns:
        List of interpolated configurations [start, ..., end]

    Example:
        # Check collision along a segment
        segment = interpolate_segment(q1, q2, step_size=0.02)
        for q in segment:
            if not world.is_collision_free(ctx, robot_id):
                return False
    """
    diff = end - start
    distance = float(np.linalg.norm(diff))

    if distance <= step_size:
        return [start, end]

    num_steps = int(np.ceil(distance / step_size))
    segment = []

    for i in range(num_steps + 1):
        alpha = i / num_steps
        segment.append(start + alpha * diff)

    return segment


def simplify_path(
    world: WorldSpec,
    robot_id: WorldRobotID,
    path: JointPath,
    max_iterations: int = 100,
    collision_step_size: float = 0.02,
) -> JointPath:
    """Simplify path by removing unnecessary waypoints.

    Uses random shortcutting: randomly select two points and check if
    the direct connection is collision-free. If so, remove intermediate
    waypoints.

    Args:
        world: World for collision checking
        robot_id: Which robot
        path: Original path
        max_iterations: Maximum shortcutting attempts
        collision_step_size: Step size for collision checking along shortcuts

    Returns:
        Simplified path with fewer waypoints

    Example:
        raw_path = planner.plan_joint_path(world, robot_id, q_start, q_goal).path
        simplified = simplify_path(world, robot_id, raw_path)
    """
    if len(path) <= 2:
        return path

    simplified = list(path)

    for _ in range(max_iterations):
        if len(simplified) <= 2:
            break

        # Pick two random indices (at least 2 apart)
        i = np.random.randint(0, len(simplified) - 2)
        j = np.random.randint(i + 2, len(simplified))

        # Check if direct connection is valid
        with world.scratch_context() as ctx:
            segment = interpolate_segment(simplified[i], simplified[j], collision_step_size)
            valid = True

            for q in segment:
                world.set_positions(ctx, robot_id, q)
                if not world.is_collision_free(ctx, robot_id):
                    valid = False
                    break

            if valid:
                # Remove intermediate waypoints
                simplified = simplified[: i + 1] + simplified[j:]

    return simplified


def compute_path_length(path: JointPath) -> float:
    """Compute total path length in joint space.

    Sums the Euclidean distances between consecutive waypoints.

    Args:
        path: Path to measure

    Returns:
        Total length in radians

    Example:
        length = compute_path_length(path)
        print(f"Path length: {length:.2f} rad")
    """
    if len(path) <= 1:
        return 0.0

    length = 0.0
    for i in range(len(path) - 1):
        length += float(np.linalg.norm(path[i + 1] - path[i]))

    return length


def is_path_within_limits(
    path: JointPath,
    lower_limits: NDArray[np.float64],
    upper_limits: NDArray[np.float64],
) -> bool:
    """Check if all waypoints in path are within joint limits.

    Args:
        path: Path to check
        lower_limits: Lower joint limits (radians)
        upper_limits: Upper joint limits (radians)

    Returns:
        True if all waypoints are within limits
    """
    for q in path:
        if np.any(q < lower_limits) or np.any(q > upper_limits):
            return False
    return True


def clip_path_to_limits(
    path: JointPath,
    lower_limits: NDArray[np.float64],
    upper_limits: NDArray[np.float64],
) -> JointPath:
    """Clip all waypoints in path to joint limits.

    Args:
        path: Path to clip
        lower_limits: Lower joint limits (radians)
        upper_limits: Upper joint limits (radians)

    Returns:
        Path with all waypoints clipped to limits
    """
    return [np.clip(q, lower_limits, upper_limits) for q in path]


def reverse_path(path: JointPath) -> JointPath:
    """Reverse a path (for returning to start, etc.).

    Args:
        path: Path to reverse

    Returns:
        Reversed path
    """
    return list(reversed(path))


def concatenate_paths(
    *paths: JointPath,
    remove_duplicates: bool = True,
) -> JointPath:
    """Concatenate multiple paths into one.

    Args:
        *paths: Paths to concatenate
        remove_duplicates: If True, remove duplicate waypoints at junctions

    Returns:
        Single concatenated path
    """
    result: list[NDArray[np.float64]] = []

    for path in paths:
        if not path:
            continue

        if remove_duplicates and result:
            # Check if last point matches first point (tight tolerance for joint space)
            if np.allclose(result[-1], path[0], atol=1e-6, rtol=0):
                result.extend(path[1:])
            else:
                result.extend(path)
        else:
            result.extend(path)

    return result
