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


from copy import deepcopy
import math
from typing import TypeVar, cast

import numpy as np
from scipy.ndimage import uniform_filter1d

from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose as CdrPose,
    PoseStamped as CdrPoseStamped,
    Quaternion as CdrQuaternion,
)
from dimos.messages.nav_msgs.msg import Path as CdrPath
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


_PathT = TypeVar("_PathT", Path, CdrPath)
_Stamped = PoseStamped | CdrPoseStamped
_Quaternion = Quaternion | CdrQuaternion


def _pose(stamped: _Stamped) -> PoseStamped | CdrPose:
    return stamped.pose if isinstance(stamped, CdrPoseStamped) else stamped


def _new_pose(
    path: Path | CdrPath, position: tuple[float, float, float], orientation: _Quaternion
) -> _Stamped:
    if isinstance(path, CdrPath):
        x, y, z = position
        return CdrPoseStamped(
            header=deepcopy(path.header),
            pose=CdrPose(
                position=Point(x=float(x), y=float(y), z=float(z)),
                orientation=CdrQuaternion(
                    x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w
                ),
            ),
        )
    return PoseStamped(
        frame_id=path.frame_id,
        position=position,
        orientation=Quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
    )


def _new_path(path: _PathT, poses: list[_Stamped]) -> _PathT:
    if isinstance(path, CdrPath):
        return CdrPath(header=deepcopy(path.header), poses=cast("list[CdrPoseStamped]", poses))
    return Path(frame_id=path.frame_id, poses=cast("list[PoseStamped]", poses))


def _set_orientation(stamped: _Stamped, orientation: _Quaternion) -> None:
    if isinstance(stamped, CdrPoseStamped):
        stamped.pose.orientation = CdrQuaternion(
            x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w
        )
    else:
        stamped.orientation = Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)


def _add_orientations_to_path(path: Path | CdrPath, goal_orientation: _Quaternion) -> None:
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
        current_pose = _pose(path.poses[i])
        next_pose = _pose(path.poses[i + 1])

        # Calculate direction to next point
        dx = next_pose.position.x - current_pose.position.x
        dy = next_pose.position.y - current_pose.position.y

        # Calculate yaw angle
        yaw = math.atan2(dy, dx)

        # Convert to quaternion (roll=0, pitch=0, yaw)
        orientation = Quaternion(0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
        _set_orientation(path.poses[i], orientation)

    # Set last pose orientation
    identity_quat = Quaternion(0, 0, 0, 1)
    if (goal_orientation.x, goal_orientation.y, goal_orientation.z, goal_orientation.w) != (
        0,
        0,
        0,
        1,
    ):
        # Use the provided goal orientation if it's not the identity
        _set_orientation(path.poses[-1], goal_orientation)
    elif len(path.poses) > 1:
        # Use the previous pose's orientation
        _set_orientation(path.poses[-1], _pose(path.poses[-2]).orientation)
    else:
        # Single pose with identity goal orientation
        _set_orientation(path.poses[-1], identity_quat)


# TODO: replace goal_pose with just goal_orientation
def simple_resample_path(path: _PathT, goal_pose: Pose | CdrPose, spacing: float) -> _PathT:
    """Resample a path to have approximately uniform spacing between poses.

    Args:
        path: The original Path
        spacing: Desired distance between consecutive poses

    Returns:
        A new Path with resampled poses
    """
    if len(path.poses) < 2 or spacing <= 0:
        return path

    resampled: list[_Stamped] = []
    resampled.append(deepcopy(path.poses[0]))

    accumulated_distance = 0.0

    for i in range(1, len(path.poses)):
        current = path.poses[i]
        prev: _Stamped = path.poses[i - 1]

        # Calculate segment distance
        dx = _pose(current).position.x - _pose(prev).position.x
        dy = _pose(current).position.y - _pose(prev).position.y
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
            new_x = _pose(prev).position.x + dir_x * dist_along
            new_y = _pose(prev).position.y + dir_y * dist_along
            new_pose = _new_pose(path, (new_x, new_y, 0.0), _pose(prev).orientation)
            resampled.append(new_pose)

            # Update for next iteration
            accumulated_distance = 0
            segment_length -= dist_along
            prev = new_pose

        accumulated_distance += segment_length

    # Add last pose if not already there
    if len(path.poses) > 1:
        last = path.poses[-1]
        if not resampled or (
            _pose(resampled[-1]).position.x != _pose(last).position.x
            or _pose(resampled[-1]).position.y != _pose(last).position.y
        ):
            resampled.append(deepcopy(last))

    ret = _new_path(path, resampled)

    _add_orientations_to_path(ret, goal_pose.orientation)

    return ret


def smooth_resample_path(
    path: _PathT, goal_pose: Pose | CdrPose, spacing: float, smoothing_window: int = 100
) -> _PathT:
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
        p = _pose(path.poses[0]).position
        o = goal_pose.orientation
        new_pose = _new_pose(path, (p.x, p.y, p.z), o)
        return _new_path(path, [new_pose])

    if len(path.poses) < 2 or spacing <= 0:
        return path

    # Extract x, y coordinates from path
    xs = np.array([_pose(p).position.x for p in path.poses])
    ys = np.array([_pose(p).position.y for p in path.poses])

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
    resampled: list[_Stamped] = []
    for i in range(len(sampled_x)):
        new_pose = _new_pose(
            path, (float(sampled_x[i]), float(sampled_y[i]), 0.0), Quaternion(0, 0, 0, 1)
        )
        resampled.append(new_pose)

    ret = _new_path(path, resampled)

    _add_orientations_to_path(ret, goal_pose.orientation)

    return ret
