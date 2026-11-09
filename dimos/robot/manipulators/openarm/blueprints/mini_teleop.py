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
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoCoordinator,
    arm_connection,
    arm_remaps,
)
from dimos.robot.manipulators.openarm.config import (
    OPENARM_SIDES,
    openarm_bimanual_model_config,
    openarm_urdf_joints,
)
from dimos.robot.manipulators.openarm.connection import OpenArmConnection
from dimos.teleop.openarm_mini.teleop_module import OpenArmMiniTeleopModule

# Leader-follower needs human arm speeds, well above the WebXR IK profile.
_OPENARM_ARM_VELOCITY_PROFILE_RAD_S = (3.0, 3.0, 3.0, 3.0, 6.0, 6.0, 6.0)

OPENARM_ROUTER = os.environ.get("DIMOS_OPENARM_ROUTER", "tcp/127.0.0.1:7447")

_MODEL = openarm_bimanual_model_config(robot="arm")
# Leader joint N drives the connection's joint N of the same side.
_TARGET_JOINTS_BY_SIDE = {
    side: tuple(f"arm/{joint}" for joint in openarm_urdf_joints(side)) for side in OPENARM_SIDES
}


def _trajectory_task() -> TaskConfig:
    velocity_limits = {
        joint_name: velocity_limit
        for side in OPENARM_SIDES
        for joint_name, velocity_limit in zip(
            _TARGET_JOINTS_BY_SIDE[side], _OPENARM_ARM_VELOCITY_PROFILE_RAD_S, strict=True
        )
    }
    return TaskConfig(
        name=JOINT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=list(_MODEL.joint_names),
        priority=10,
        params={
            # A streamed trajectory restarts from the measured pose after a reject;
            # the leader may be this far ahead of the arm by then.
            "start_position_tolerance": 0.3,
            "velocity_limits": velocity_limits,
        },
    )


def _gripper_tasks() -> list[TaskConfig]:
    """One gripper task per jaw, fed by the leader's trigger opening (1 = open)."""
    return [
        TaskConfig(
            name=f"arm_{gripper.name}",
            type="gripper",
            joint_names=[f"arm/{gripper.joint}"],
            priority=20,
            params={"open_at": gripper.open_at},
            stream_bind={"gripper_command": f"{gripper.name}_command"},
        )
        for gripper in OpenArmConnection.robot.grippers
    ]


def _leader() -> Blueprint:
    return OpenArmMiniTeleopModule.blueprint(target_joint_names_by_side=_TARGET_JOINTS_BY_SIDE)


def _follower(visualization: ViserVisualizationConfig | None = None) -> Blueprint:
    """Streamed joint targets over the OpenArm connection; --simulation swaps in a mock."""
    return autoconnect(
        DamiaoCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=_MODEL.model,
            tasks=[_trajectory_task(), *_gripper_tasks()],
            zero_stiffness_joints=[],
        ),
        arm_connection(OpenArmConnection),
        ManipulationModule.blueprint(
            model=_MODEL,
            gripper_tasks={f"{side}_arm": f"arm_{side}_gripper" for side in OPENARM_SIDES},
            visualization=visualization if visualization is not None else {"backend": "viser"},
        ),
    ).remappings(arm_remaps(OpenArmConnection, DamiaoCoordinator))


teleop_openarm_mini = autoconnect(_leader(), _follower())

# Split deployment: leaders on an operator machine, follower on the robot.
# The robot stack owns the Coordinator name, so the leader half does not claim it.
teleop_openarm_mini_leader = _leader().global_config(
    zenoh_mode="client", serve_coordinator_rpc=False
)

teleop_openarm_mini_follower = _follower(ViserVisualizationConfig(host="0.0.0.0")).global_config(
    zenoh_connect=OPENARM_ROUTER
)
