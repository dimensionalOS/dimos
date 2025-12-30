# Copyright 2025 Dimensional Inc.
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
Joint Trajectory Generator Specification

Generates time-parameterized joint trajectories from waypoints using
trapezoidal velocity profiles. Does NOT execute - just generates.

Input: List of joint positions (waypoints) without timing
Output: JointTrajectory with proper time parameterization

Trapezoidal Profile:
    velocity
       ^
       |    ____________________
       |   /                    \
       |  /                      \
       | /                        \
       |/                          \
       +------------------------------> time
        accel    cruise      decel
"""

from typing import Protocol

from dimos.msgs.trajectory_msgs import JointTrajectory


class JointTrajectoryGeneratorSpec(Protocol):
    """Protocol for joint trajectory generator.

    Generates time-parameterized trajectories from waypoints.
    """

    # Configuration
    max_velocity: list[float]  # rad/s per joint
    max_acceleration: list[float]  # rad/s^2 per joint

    def generate(self, waypoints: list[list[float]]) -> JointTrajectory:
        """
        Generate a trajectory through waypoints with trapezoidal velocity profile.

        Args:
            waypoints: List of joint positions [q1, q2, ..., qn] in radians
                       First waypoint is start, last is goal

        Returns:
            JointTrajectory with time-parameterized points
        """
        ...

    def set_limits(
        self,
        max_velocity: list[float] | float,
        max_acceleration: list[float] | float,
    ) -> None:
        """
        Set velocity and acceleration limits.

        Args:
            max_velocity: rad/s (single value applies to all joints, or per-joint)
            max_acceleration: rad/s^2 (single value or per-joint)
        """
        ...
