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


import math

from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.nav_msgs.msg import Path
import numpy as np
from scipy.ndimage import uniform_filter1d

from dimos.msgs.geometry import quaternion_from_euler
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


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
        dx = next_pose.pose.position.x - current_pose.pose.position.x
        dy = next_pose.pose.position.y - current_pose.pose.position.y

        # Calculate yaw angle
        yaw = math.atan2(dy, dx)

        # Convert to quaternion (roll=0, pitch=0, yaw)
        orientation = quaternion_from_euler(0, 0, yaw)
        current_pose.pose.orientation = orientation
        path.poses[i] = current_pose

    # Set last pose orientation
    identity_quat = Quaternion(w=1)
    last_pose = path.poses[-1]
    if goal_orientation != identity_quat:
        # Use the provided goal orientation if it's not the identity
        last_pose.pose.orientation = goal_orientation
    elif len(path.poses) > 1:
        # Use the previous pose's orientation
        last_pose.pose.orientation = path.poses[-2].pose.orientation
    else:
        # Single pose with identity goal orientation
        last_pose.pose.orientation = identity_quat
    path.poses[-1] = last_pose


# TODO: replace goal_pose with just goal_orientation
def simple_resample_path(path: Path, goal_pose: Pose, spacing: float) -> Path:
    """Resample a path to have approximately uniform spacing between poses.

    Args:
        path: The original Path
        spacing: Desired distance between consecutive poses

    Returns:
        A new Path with resampled poses
    """
    if len(path.poses) < 2 or spacing <= 0:
        return path

    resampled = []
    resampled.append(path.poses[0])

    accumulated_distance = 0.0

    for i in range(1, len(path.poses)):
        current = path.poses[i]
        prev = path.poses[i - 1]

        # Calculate segment distance
        dx = current.pose.position.x - prev.pose.position.x
        dy = current.pose.position.y - prev.pose.position.y
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
            new_x = prev.pose.position.x + dir_x * dist_along
            new_y = prev.pose.position.y + dir_y * dist_along
            new_pose = PoseStamped(
                header=path.header,
                pose=Pose(position=Point(x=new_x, y=new_y), orientation=prev.pose.orientation),
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
        if (
            math.hypot(
                resampled[-1].pose.position.x - last.pose.position.x,
                resampled[-1].pose.position.y - last.pose.position.y,
            )
            <= 1e-10
        ):
            resampled[-1] = last
        else:
            resampled.append(last)

    ret = Path(header=path.header, poses=resampled)

    _add_orientations_to_path(ret, goal_pose.orientation)

    return ret


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
        p = path.poses[0].pose.position
        o = goal_pose.orientation
        new_pose = PoseStamped(
            header=path.header,
            pose=Pose(position=p, orientation=o),
        )
        return Path(header=path.header, poses=[new_pose])

    if len(path.poses) < 2 or spacing <= 0:
        return path

    # Extract x, y coordinates from path
    xs = np.array([p.pose.position.x for p in path.poses])
    ys = np.array([p.pose.position.y for p in path.poses])

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
            header=path.header,
            pose=Pose(
                position=Point(x=float(sampled_x[i]), y=float(sampled_y[i])),
                orientation=Quaternion(w=1),
            ),
        )
        resampled.append(new_pose)

    ret = Path(header=path.header, poses=resampled)

    _add_orientations_to_path(ret, goal_pose.orientation)

    return ret
