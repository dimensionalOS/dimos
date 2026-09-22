# Copyright 2026 Dimensional Inc.
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

"""Bounded non-driving trajectory timing for the classical simulator demo."""

import math

from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
    JointTrajectoryGenerator,
)
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS

CLASSICAL_MOTION_SPEED_SCALE = 2.0


def joint_trajectory(
    points: list[list[float]],
    *,
    carrying: bool,
    speed_scale: float = CLASSICAL_MOTION_SPEED_SCALE,
) -> JointTrajectory:
    """Retain every checked sample and stop, changing only non-base timing.

    At the maximum 2x scale, unloaded joints use 1.2 rad/s and 6 rad/s²;
    loaded torso joints use 0.16/0.6 and arms 0.5/2.0. Grippers use
    0.1 m/s and 0.8 m/s². These velocities remain below the coordinator's
    2 rad/s joint and 0.25 m/s gripper limits. Base joints are not accepted.
    """
    if not math.isfinite(speed_scale) or not 0 < speed_scale <= CLASSICAL_MOTION_SPEED_SCALE:
        raise ValueError("Non-driving speed_scale must be finite, positive and at most 2")
    joint_count = len(R1PRO_PICK_PLACE_JOINTS)
    if any(len(point) != joint_count for point in points):
        raise ValueError(
            "Non-driving waypoints must contain exactly the 20 arm/torso/gripper joints"
        )
    if any(not math.isfinite(value) for point in points for value in point):
        raise ValueError("Non-driving waypoints must be finite")
    generator = JointTrajectoryGenerator(
        num_joints=joint_count,
        max_velocity=([0.08] * 4 + [0.25] * 14 if carrying else [0.6] * 18) + [0.05, 0.05],
        max_acceleration=([0.15] * 4 + [0.5] * 14 if carrying else [1.5] * 18) + [0.2, 0.2],
        points_per_segment=8,
    )
    reference = generator.generate(points)
    return JointTrajectory(
        joint_names=list(R1PRO_PICK_PLACE_JOINTS),
        points=[
            TrajectoryPoint(
                positions=point.positions,
                velocities=[value * speed_scale for value in point.velocities],
                time_from_start=point.time_from_start / speed_scale,
            )
            for point in reference.points
        ],
    )
