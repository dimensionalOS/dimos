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

"""OpenArm Mini leader teleop blueprints for the bimanual OpenArm follower."""

from __future__ import annotations

import os

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.manipulators.openarm.blueprints.teleop import (
    OpenArmTeleopCoordinator,
    _OpenArmManipulationModule,
)
from dimos.robot.manipulators.openarm.config import (
    OPENARM_GRIPPER_JOINTS,
    OPENARM_SIDES,
    openarm_bimanual_model_config,
    openarm_urdf_joints,
)
from dimos.teleop.openarm_mini.calibration import OpenArmMiniSide
from dimos.teleop.openarm_mini.teleop_module import OpenArmMiniTeleopModule

# Leader-follower needs human arm speeds, well above the WebXR IK profile.
_OPENARM_ARM_VELOCITY_PROFILE_RAD_S = (3.0, 3.0, 3.0, 3.0, 6.0, 6.0, 6.0)

OPENARM_ROUTER = os.environ.get("DIMOS_OPENARM_ROUTER", "tcp/127.0.0.1:7447")


def _trajectory_task(sides: tuple[OpenArmMiniSide, ...]) -> TaskConfig:
    velocity_limits = {
        joint_name: velocity_limit
        for side in sides
        for joint_name, velocity_limit in zip(
            openarm_urdf_joints(side),
            _OPENARM_ARM_VELOCITY_PROFILE_RAD_S,
            strict=True,
        )
    }
    return TaskConfig(
        name=JOINT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=list(velocity_limits),
        priority=10,
        params={
            "start_position_tolerance": 0.05,
            "velocity_limits": velocity_limits,
        },
    )


def _gripper_tasks() -> list[TaskConfig]:
    return [
        TaskConfig(
            name=f"{side}_arm_gripper",
            type="gripper",
            joint_names=[joint_name],
            priority=20,
            stream_bind={"gripper_command": f"{side}_gripper_command"},
        )
        for side, joint_name in zip(OPENARM_SIDES, OPENARM_GRIPPER_JOINTS, strict=True)
    ]


def _follower_tasks() -> list[TaskConfig]:
    return [_trajectory_task(("left", "right")), *_gripper_tasks()]


teleop_openarm_mini = autoconnect(
    OpenArmMiniTeleopModule.blueprint(),
    OpenArmTeleopCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=_follower_tasks(),
    ),
    _OpenArmManipulationModule.blueprint(
        model=openarm_bimanual_model_config(),
        visualization={"backend": "viser"},
    ),
)

# Split deployment: leaders on an operator machine, follower on the robot.
# The robot stack owns the Coordinator name, so the leader half does not claim it.
teleop_openarm_mini_leader = OpenArmMiniTeleopModule.blueprint().global_config(
    zenoh_mode="client", serve_coordinator_rpc=False
)

teleop_openarm_mini_follower = autoconnect(
    OpenArmTeleopCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=_follower_tasks(),
    ),
    _OpenArmManipulationModule.blueprint(
        model=openarm_bimanual_model_config(),
        visualization=ViserVisualizationConfig(host="0.0.0.0"),
    ),
).global_config(zenoh_connect=OPENARM_ROUTER)
