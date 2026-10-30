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

"""Basic xArm coordinator and planner blueprints."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators._modeling import joint_names
from dimos.robot.manipulators.common.blueprints import coordinator, planner, trajectory_task
from dimos.robot.manipulators.common.sim import mujoco_if_sim
from dimos.robot.manipulators.xarm.config import (
    XARM6_SIM_PATH,
    XARM7_SIM_PATH,
    XARM_ARMS,
    XArmType,
    lite6_hardware,
    make_dual_xarm6_model_config,
    make_lite6_model_config,
    make_xarm_hardware,
    make_xarm_model_config,
    xarm6_hardware,
    xarm7_hardware,
    xarm_model,
)
from dimos.robot.manipulators.xarm.connection import XArmConnection

_dual_xarm6_model = make_dual_xarm6_model_config()
_mock_left_xarm6_hw = make_xarm_hardware(
    "left_arm", 6, canonical_joint_names=list(_dual_xarm6_model.planning_groups[0].joint_names)
)
_mock_right_xarm6_hw = make_xarm_hardware(
    "right_arm", 6, canonical_joint_names=list(_dual_xarm6_model.planning_groups[1].joint_names)
)

dual_xarm6_planner_coordinator = autoconnect(
    planner(
        model=_dual_xarm6_model,
        visualization={"backend": "viser"},
    ),
    coordinator(
        hardware=[_mock_left_xarm6_hw, _mock_right_xarm6_hw],
        tasks=[trajectory_task(_mock_left_xarm6_hw, _mock_right_xarm6_hw)],
    ),
)


def _gripper_task() -> TaskConfig:
    return TaskConfig(
        name="arm_gripper",
        type="gripper",
        joint_names=["arm/gripper"],
        priority=20,
    )


class XArmCoordinator(ControlCoordinator):
    """Joint targets out to one xArm named "arm", and its joints in, through
    ``XArmConnection``."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


# The connection module's ports, onto the coordinator's ports for the robot named "arm".
XARM_REMAPS: list[tuple[type[ModuleBase], str, str]] = [
    (XArmConnection, "position_command", "arm_position_command"),
    (XArmCoordinator, "arm_joint_state", "joint_state"),
]


def xarm_coordinator(arm: XArmType) -> Blueprint:
    """Run planned joint trajectories on an xArm and open and close its xArm
    Gripper, through ``XArmConnection``.

    The coordinator names the joints ``arm/joint1`` ... and ``arm/drive_joint``;
    a planner driving it uses the model from ``make_xarm_model_config(...,
    robot="arm", gripper_hardware_id="arm")``, whose joints are named the same.
    Wire it up with ``.remappings(XARM_REMAPS)``. The arm's address is given at
    run time, after the blueprint name (``--xarmconnection.ip 192.168.1.185``)
    or as ``XARMCONNECTION__IP``.

    Args:
        arm: Which arm, e.g. "xarm7".
    """
    dof = XARM_ARMS[arm][0]
    return autoconnect(
        XArmCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=xarm_model(arm, gripper="xarm", robot="arm"),
            tasks=[
                joint_trajectory_task(joint_names(dof, prefix="arm/joint")),
                # drive_joint is open at 0 rad.
                TaskConfig(
                    name="arm_gripper",
                    type="gripper",
                    joint_names=["arm/drive_joint"],
                    priority=20,
                    params={"open_at": "lower"},
                ),
            ],
        ),
        XArmConnection.blueprint(arm=arm, gripper="xarm"),
    )


def _planner_coordinator(arm: XArmType) -> Blueprint:
    """Plan and run motions of an xArm with the xArm Gripper. Wire it up with
    ``.remappings(XARM_REMAPS)``; see ``xarm_coordinator``."""
    dof, robot_type = XARM_ARMS[arm]
    return autoconnect(
        planner(
            model=make_xarm_model_config(
                dof, robot_type=robot_type, robot="arm", gripper_hardware_id="arm"
            )
        ),
        xarm_coordinator(arm),
    )


# The blueprint registry finds a blueprint only by a line ending in a call like `.remappings`.
xarm7_planner_coordinator = _planner_coordinator("xarm7").remappings(XARM_REMAPS)
xarm6_planner_coordinator = _planner_coordinator("xarm6").remappings(XARM_REMAPS)

_coordinator_xarm7_hw = xarm7_hardware("arm")

coordinator_xarm7 = autoconnect(
    coordinator(
        hardware=[_coordinator_xarm7_hw],
        tasks=[trajectory_task(_coordinator_xarm7_hw)],
    ),
    *mujoco_if_sim(XARM7_SIM_PATH, len(_coordinator_xarm7_hw.joints)),
)

_coordinator_xarm6_hw = xarm6_hardware("arm", gripper=True)

coordinator_xarm6 = autoconnect(
    coordinator(
        hardware=[_coordinator_xarm6_hw],
        tasks=[trajectory_task(_coordinator_xarm6_hw), _gripper_task()],
    ),
    *mujoco_if_sim(XARM6_SIM_PATH, len(_coordinator_xarm6_hw.joints)),
)

_coordinator_lite6_hw = lite6_hardware("arm", gripper=True)

coordinator_lite6 = ControlCoordinator.blueprint(
    hardware=[_coordinator_lite6_hw],
    tasks=[trajectory_task(_coordinator_lite6_hw), _gripper_task()],
)

_lite6_hw = lite6_hardware("arm", gripper=True, mock_without_address=True)

lite6_planner_coordinator = autoconnect(
    planner(
        model=make_lite6_model_config(
            add_gripper=True,
            gripper_hardware_id="arm",
        ),
        visualization={"backend": "viser"},
    ),
    coordinator(
        hardware=[_lite6_hw],
        tasks=[trajectory_task(_lite6_hw), _gripper_task()],
    ),
)

_xarm7_left = xarm7_hardware(
    "left_arm", canonical_joint_names=[f"left_arm/joint{i}" for i in range(1, 8)]
)
_xarm6_right = xarm6_hardware(
    "right_arm", canonical_joint_names=[f"right_arm/joint{i}" for i in range(1, 7)]
)

coordinator_dual_xarm = ControlCoordinator.blueprint(
    hardware=[_xarm7_left, _xarm6_right],
    tasks=[trajectory_task(_xarm7_left, _xarm6_right)],
)
