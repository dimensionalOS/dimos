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

from dimos.msgs.sensor_msgs import JointState

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
        path: Original path (list of JointState waypoints)
        resolution: Maximum distance between waypoints (radians)

    Returns:
        Interpolated path with more waypoints

    Example:
        # After planning, interpolate for smoother execution
        raw_path = planner.plan_joint_path(world, robot_id, start, goal).path
        smooth_path = interpolate_path(raw_path, resolution=0.02)
    """
    if len(path) <= 1:
        return list(path)

    interpolated: list[JointState] = [path[0]]
    joint_names = path[0].name

    for i in range(len(path) - 1):
        q_start = np.array(path[i].position, dtype=np.float64)
        q_end = np.array(path[i + 1].position, dtype=np.float64)

        diff = q_end - q_start
        max_diff = float(np.max(np.abs(diff)))

        if max_diff <= resolution:
            interpolated.append(path[i + 1])
        else:
            num_steps = int(np.ceil(max_diff / resolution))
            for step in range(1, num_steps + 1):
                alpha = step / num_steps
                q_interp = q_start + alpha * diff
                interpolated.append(JointState(name=joint_names, position=q_interp.tolist()))

    return interpolated


def interpolate_segment(
    start: JointState,
    end: JointState,
    step_size: float,
) -> JointPath:
    """Interpolate between two configurations.

    Returns a list of configurations from start to end (inclusive)
    with at most `step_size` distance between consecutive points.

    Args:
        start: Start joint configuration
        end: End joint configuration
        step_size: Maximum step size (radians)

    Returns:
        List of interpolated JointState waypoints [start, ..., end]

    Example:
        # Check collision along a segment
        segment = interpolate_segment(start_state, end_state, step_size=0.02)
        for state in segment:
            if not world.check_config_collision_free(robot_id, state):
                return False
    """
    q_start = np.array(start.position, dtype=np.float64)
    q_end = np.array(end.position, dtype=np.float64)
    joint_names = start.name

    diff = q_end - q_start
    distance = float(np.linalg.norm(diff))

    if distance <= step_size:
        return [start, end]

    num_steps = int(np.ceil(distance / step_size))
    segment: JointPath = []

    for i in range(num_steps + 1):
        alpha = i / num_steps
        q_interp = q_start + alpha * diff
        segment.append(JointState(name=joint_names, position=q_interp.tolist()))

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
        path: Original path (list of JointState waypoints)
        max_iterations: Maximum shortcutting attempts
        collision_step_size: Step size for collision checking along shortcuts

    Returns:
        Simplified path with fewer waypoints

    Example:
        raw_path = planner.plan_joint_path(world, robot_id, start, goal).path
        simplified = simplify_path(world, robot_id, raw_path)
    """
    if len(path) <= 2:
        return list(path)

    simplified = list(path)

    for _ in range(max_iterations):
        if len(simplified) <= 2:
            break

        # Pick two random indices (at least 2 apart)
        i = np.random.randint(0, len(simplified) - 2)
        j = np.random.randint(i + 2, len(simplified))

        # Check if direct connection is valid using context-free API
        if world.check_edge_collision_free(
            robot_id, simplified[i], simplified[j], collision_step_size
        ):
            # Remove intermediate waypoints
            simplified = simplified[: i + 1] + simplified[j:]

    return simplified


def compute_path_length(path: JointPath) -> float:
    """Compute total path length in joint space.

    Sums the Euclidean distances between consecutive waypoints.

    Args:
        path: Path to measure (list of JointState waypoints)

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
        q_curr = np.array(path[i].position, dtype=np.float64)
        q_next = np.array(path[i + 1].position, dtype=np.float64)
        length += float(np.linalg.norm(q_next - q_curr))

    return length


def is_path_within_limits(
    path: JointPath,
    lower_limits: NDArray[np.float64],
    upper_limits: NDArray[np.float64],
) -> bool:
    """Check if all waypoints in path are within joint limits.

    Args:
        path: Path to check (list of JointState waypoints)
        lower_limits: Lower joint limits (radians)
        upper_limits: Upper joint limits (radians)

    Returns:
        True if all waypoints are within limits
    """
    for state in path:
        q = np.array(state.position, dtype=np.float64)
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
        path: Path to clip (list of JointState waypoints)
        lower_limits: Lower joint limits (radians)
        upper_limits: Upper joint limits (radians)

    Returns:
        Path with all waypoints clipped to limits
    """
    clipped: list[JointState] = []
    for state in path:
        q = np.array(state.position, dtype=np.float64)
        q_clipped = np.clip(q, lower_limits, upper_limits)
        clipped.append(JointState(name=state.name, position=q_clipped.tolist()))
    return clipped


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
        *paths: Paths to concatenate (each is a list of JointState waypoints)
        remove_duplicates: If True, remove duplicate waypoints at junctions

    Returns:
        Single concatenated path
    """
    result: list[JointState] = []

    for path in paths:
        if not path:
            continue

        if remove_duplicates and result:
            # Check if last point matches first point (tight tolerance for joint space)
            q_last = np.array(result[-1].position, dtype=np.float64)
            q_first = np.array(path[0].position, dtype=np.float64)
            if np.allclose(q_last, q_first, atol=1e-6, rtol=0):
                result.extend(path[1:])
            else:
                result.extend(path)
        else:
            result.extend(path)

    return result
