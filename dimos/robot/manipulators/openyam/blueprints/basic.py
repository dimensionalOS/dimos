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

"""OpenYAM coordinator and planning through its connection module."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.damiao.connection import zero_stiffness_joints
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    arm_connection,
    arm_declaration,
    arm_remaps,
)
from dimos.robot.manipulators.openyam.config import (
    OPENYAM,
    make_openyam_model_config,
)
from dimos.robot.manipulators.openyam.connection import OpenYamConnection
from dimos.robot.manipulators.openyam.coordinator import OpenYamCoordinator
from dimos.robot.manipulators.openyam.joints import OPENYAM_GRIPPER_JOINT
from dimos.robot.urdf_manager import UrdfManager

_openyam = make_openyam_model_config(robot="arm")


def _trajectory_task(*, priority: int = 10) -> TaskConfig:
    return joint_trajectory_task([*_openyam.joint_names, OPENYAM_GRIPPER_JOINT], priority=priority)


def _gripper_task() -> TaskConfig:
    # joint7 is open at its lower end.
    return TaskConfig(
        name="arm_gripper",
        type="gripper",
        joint_names=[OPENYAM_GRIPPER_JOINT],
        priority=20,
        params={"open_at": "lower"},
    )


def openyam_control(*, teach: bool = False) -> Blueprint:
    """Trajectories and the gripper, through the OpenYAM connection.

    Args:
        teach: Run with zero stiffness and teaching damping, gravity
            compensated, and the jaw passive so an operator can guide it.
    """
    return autoconnect(
        OpenYamCoordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[arm_declaration(OpenYamConnection)],
            tasks=[_trajectory_task(), _gripper_task()],
            zero_stiffness_joints=zero_stiffness_joints(OPENYAM, teach=teach),
        ),
        arm_connection(OpenYamConnection, teach=teach),
    ).remappings(arm_remaps(OpenYamConnection, OpenYamCoordinator))


coordinator_openyam = openyam_control().remappings([])
# UrdfManager serves the URDF to web 3D views. The planner already puts every
# link on tf, so the manager leaves joint tf off (on, links would get two parents).
openyam_planner_coordinator = autoconnect(
    planner(model=_openyam, gripper_tasks={"manipulator": "arm_gripper"}),
    openyam_control(),
    UrdfManager.blueprint(model=_openyam.model, publish_joint_tf=False),
)
