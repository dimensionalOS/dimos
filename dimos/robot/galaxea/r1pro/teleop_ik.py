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

"""R1 Pro whole-upper-body Pink solver for Quest teleoperation."""

from __future__ import annotations

from collections.abc import Mapping

import numpy as np
import pink

from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS, coordinator_name
from dimos.robot.galaxea.r1pro.ready_pose import READY_POSE

_POSTURE_TASK = "posture/current"

# The tray pose the robot boots into, as a full-model configuration vector.
# ``UPPER_BODY_JOINTS`` is the order Pinocchio builds the R1 Pro model in
# (torso 1-4, then left arm 1-7, then right arm 1-7), so this indexes straight
# into ``configuration.q``. One definition serves both uses: the arms start at
# this pose and the solver keeps aiming back at it, so engaging teleoperation
# never steps the arms.
NOMINAL_POSTURE = np.array(
    [READY_POSE[coordinator_name(joint)] for joint in UPPER_BODY_JOINTS],
    dtype=np.float64,
)

# Per-joint shares of the posture cost, in the same order. The hands are what
# the operator is aiming, so the three wrist joints stay nearly free -- weight
# them and the gripper stops holding the orientation it is pointed at. What is
# worth holding is everything that decides which of the many arm shapes hits
# that hand pose: the torso, the shoulder, and the elbow, with upper-arm roll
# (joint 3) carrying the swivel that a 7-DOF arm is otherwise free to wander.
POSTURE_WEIGHTS = np.concatenate(
    [
        np.full(4, 4.0),  # torso 1-4
        np.tile(np.array([2.0, 3.0, 3.0, 3.0, 0.3, 0.3, 0.1]), 2),  # arm 1-7, per side
    ]
)

# How fast the posture task may drag the arms back toward the tray pose.
#
# This bound is what makes a strong posture weight safe, and it is not
# optional. Pink turns posture error straight into a demanded joint step, so a
# far-from-posture arm demands far more motion than the streaming velocity
# envelope will pass. The envelope clips that demand per joint, which rotates
# the whole QP solution rather than merely slowing it, and the hands settle
# ~100 mm off target for as long as the arms are away from the tray pose --
# measured, and independent of the posture cost, so no amount of retuning the
# weights escapes it. Clamping the error the task ever sees keeps the demand
# inside the envelope however far the operator has moved: the pull is a spring
# near the tray pose and a bounded drift beyond it. At 0.15 rad/s the measured
# cost is ~10 mm of hand tracking while the arms are off-posture, and none once
# they settle.
POSTURE_APPROACH_RAD_S = 0.15

# Frame weights matching the proven G1 and OpenArm bimanual tuning: a metre of
# position error and a radian of posture error have to be on comparable scales
# before a posture weight means anything.
R1PRO_TELEOP_PINK = PinkKinematicsConfig(
    position_cost=8.0,
    orientation_cost=2.0,
    posture_cost=0.2,
)


class R1ProPinkPoseTargetSolver(PinkPoseTargetSolver):
    """Hold the tray posture, and limit the headset target to what a torso can do."""

    HEAD_FRAME = "head_link"

    def _create_tasks(
        self,
        configuration: pink.Configuration,
        target_frames: tuple[str, ...],
    ) -> dict[str, pink.Task]:
        tasks = super()._create_tasks(configuration, target_frames)

        posture_task = tasks.get(_POSTURE_TASK)
        if posture_task is None:
            raise ValueError("R1ProPinkPoseTargetSolver requires a positive posture cost")
        if configuration.model.nq != len(NOMINAL_POSTURE):
            raise ValueError(
                f"R1 Pro nominal posture has {len(NOMINAL_POSTURE)} joints, "
                f"model has {configuration.model.nq}"
            )
        posture_task.cost = self.config.posture_cost * POSTURE_WEIGHTS

        # Arms-only teleoperation binds no head target, so there is nothing to
        # constrain; the shaping below applies only when the torso is driven.
        head_task = tasks.get(f"frame/{self.HEAD_FRAME}")
        if head_task is None:
            return tasks
        # Free x, hold y and z. The torso is four revolute joints with no
        # prismatic lift, so head_link cannot change height without swinging
        # forward -- measured, ~0.28 m of x across the full descent. Constrain
        # x as well and the solver fights itself and the torso barely moves.
        # y stays pinned, which is what keeps the descent from twisting out of
        # the sagittal plane, and orientation below keeps the head level.
        head_task.set_position_cost(
            self.config.position_cost * np.array([0.0, 1.0, 1.0], dtype=np.float64)
        )
        head_task.set_orientation_cost(
            self.config.orientation_cost * np.array([0.0, 1.0, 1.0], dtype=np.float64)
        )
        return tasks

    def _update_current_posture_target(
        self,
        tasks: Mapping[str, pink.Task],
        configuration: pink.Configuration,
    ) -> None:
        """Skip the shared current-configuration target.

        Aiming the posture task at the configuration the robot is already in
        makes its error zero by construction, which regularizes the QP but
        holds no posture. The tray target replaces it in ``_before_solve``,
        which is the hook that gets the ``dt`` the approach limit needs.
        """

    def _before_solve(
        self,
        tasks: Mapping[str, pink.Task],
        configuration: pink.Configuration,
        dt: float,
    ) -> None:
        super()._before_solve(tasks, configuration, dt)
        posture_task = tasks.get(_POSTURE_TASK)
        if not isinstance(posture_task, pink.tasks.PostureTask):
            raise ValueError("R1ProPinkPoseTargetSolver requires a posture task")
        # Pink asks for ``gain`` times the error in one solve, so the error the
        # task is allowed to see is the step budget divided back out by it.
        approach_limit = POSTURE_APPROACH_RAD_S * dt / posture_task.gain
        q = np.asarray(configuration.q, dtype=np.float64)
        posture_task.set_target(q + np.clip(NOMINAL_POSTURE - q, -approach_limit, approach_limit))
