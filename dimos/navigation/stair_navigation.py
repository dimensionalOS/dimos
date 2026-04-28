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

"""Helpers for generating 3D stair-climbing waypoint plans."""

import math

from dimos.msgs.geometry_msgs import PoseStamped, Quaternion
from dimos.msgs.geometry_msgs.Vector3 import make_vector3


def generate_stair_waypoints(
    *,
    start_x: float,
    start_y: float,
    start_z: float,
    goal_x: float,
    goal_y: float,
    total_height: float,
    steps: int,
) -> list[PoseStamped]:
    """Generate evenly spaced 3D waypoints for a staircase ascent.

    Args:
        start_x: Current position x in meters.
        start_y: Current position y in meters.
        start_z: Current position z in meters.
        goal_x: Goal x in meters.
        goal_y: Goal y in meters.
        total_height: Vertical displacement in meters. Positive climbs, negative descends.
        steps: Number of stair steps to split the path into.

    Returns:
        A non-empty list of map-frame PoseStamped waypoints.
    """
    if steps <= 0:
        raise ValueError("steps must be greater than 0")
    if total_height == 0:
        raise ValueError("total_height must be non-zero")

    waypoints: list[PoseStamped] = []
    yaw = 0.0
    if goal_x != start_x or goal_y != start_y:
        # Heading in the XY plane so the controller faces the stair direction.
        yaw = math.atan2(goal_y - start_y, goal_x - start_x)
    orientation = Quaternion.from_euler(make_vector3(0, 0, yaw))

    for step_idx in range(1, steps + 1):
        ratio = step_idx / steps
        waypoint_x = start_x + (goal_x - start_x) * ratio
        waypoint_y = start_y + (goal_y - start_y) * ratio
        waypoint_z = start_z + total_height * ratio
        waypoints.append(
            PoseStamped(
                position=make_vector3(waypoint_x, waypoint_y, waypoint_z),
                orientation=orientation,
                frame_id="map",
            )
        )

    return waypoints


__all__ = ["generate_stair_waypoints"]
