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

from collections.abc import Sequence
from typing import Any

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.connection.mock_joint_connection import (
    MockJointConnection,
    MockJointConnectionConfig,
)
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import movable_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, ChoiceAtom, MetaBlueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators._modeling import joint_names
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.xarm.config import (
    XARM6_SIM_HOME,
    XARM6_SIM_PATH,
    XARM7_SIM_BASE_POSE,
    XARM7_SIM_HOME,
    XARM7_SIM_PATH,
    XARM_ARMS,
    XArmType,
    make_dual_xarm6_model_config,
    make_lite6_model_config,
    make_xarm_model_config,
    xarm_model,
)
from dimos.robot.manipulators.xarm.connection import XArmConnection
from dimos.simulation.engines.mujoco_arm_connection import MujocoArmConnection

SIMULATED = bool(global_config.simulation)

# The MuJoCo scene and start pose of each arm that has one; each has the xArm Gripper.
XARM_SIM_SCENES: dict[XArmType, Any] = {"xarm6": XARM6_SIM_PATH, "xarm7": XARM7_SIM_PATH}
XARM_SIM_HOMES: dict[XArmType, list[float]] = {"xarm6": XARM6_SIM_HOME, "xarm7": XARM7_SIM_HOME}


class XArmCoordinator(ControlCoordinator):
    """Joint targets and speeds out to one xArm named "arm", and its joints in."""

    arm_position_command: Out[JointState]
    arm_velocity_command: Out[JointState]
    arm_joint_state: In[JointState]


def xarm_gripper_task(**kwargs: Any) -> TaskConfig:
    """Open and close the xArm Gripper of the arm named "arm" (``arm/drive_joint``)."""
    # drive_joint is open at 0 rad.
    return TaskConfig(
        name="arm_gripper",
        type="gripper",
        joint_names=["arm/drive_joint"],
        priority=20,
        params={"open_at": "lower"},
        **kwargs,
    )


def xarm_connection(
    arm: XArmType,
    robot: str = "arm",
    *,
    gripper: bool = True,
    velocity: bool = False,
    simulated: bool = False,
    sim_module_kwargs: dict[str, Any] | None = None,
    instance_name: str | None = None,
) -> tuple[ConnectionConfig, Blueprint | MetaBlueprint]:
    """One xArm for a coordinator: its declaration, and the connection wired onto
    the coordinator ports ``<robot>_position_command``, ``<robot>_velocity_command``
    and ``<robot>_joint_state``. Joints are named ``<robot>/joint1`` ... and
    ``<robot>/drive_joint``.

    A real arm without an ``instance_name`` can be swapped at launch for a
    ``MockJointConnection`` with the same joints: ``--choice arm=mock``.

    Args:
        arm: Which arm, e.g. "xarm7".
        robot: The robot's name in front of each joint name.
        gripper: With the xArm Gripper. The MuJoCo scenes always have it.
        velocity: Drive the arm joints by speed (rad/s) instead of position (rad).
        simulated: Run the arm in its MuJoCo scene instead of over the SDK.
        sim_module_kwargs: When simulated, the ``MujocoSimModule`` settings
            (scene, camera, start pose); the arm's plain scene at its home pose if None.
        instance_name: The connection's name, for a blueprint with two xArms.
            The real arm's address flag follows it: ``--<instance_name>.ip``.
    """
    if simulated and arm not in XARM_SIM_SCENES:
        raise ValueError(f"there is no MuJoCo scene of the {arm}; only of {list(XARM_SIM_SCENES)}")
    if simulated and not gripper:
        raise ValueError("the MuJoCo xArm scenes have the xArm Gripper; simulate with gripper=True")
    model = xarm_model(arm, gripper="xarm" if gripper else None, robot=robot)
    arm_interface = Interface.VELOCITY if velocity else Interface.POSITION
    declaration = ConnectionConfig(
        source=robot,
        model=model,
        joint_commands={
            joint.name: (
                Interface.POSITION if joint.name.endswith("/drive_joint") else arm_interface,
            )
            for joint in movable_joints(model)
        },
    )
    fields = {
        "source": robot,
        "model": model,
        "joint_commands": declaration.joint_commands,
        **({"instance_name": instance_name} if instance_name else {}),
    }
    if simulated:
        # mujoco is optional; import it only when a blueprint runs the sim.
        from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

        if sim_module_kwargs is None:
            sim_module_kwargs = {
                "address": str(XARM_SIM_SCENES[arm]),
                "headless": False,
                "dof": XARM_ARMS[arm][0],
                "reset_joint_positions": list(XARM_SIM_HOMES[arm]),
            }
        sim = autoconnect(
            MujocoArmConnection.blueprint(**fields, gripper_joint=f"{robot}/drive_joint"),
            MujocoSimModule.blueprint(**sim_module_kwargs),
        )
        return declaration, sim.remappings(_arm_ports(instance_name or MujocoArmConnection, robot))
    real = XArmConnection.blueprint(
        arm=arm, gripper="xarm" if gripper else None, **fields
    ).remappings(_arm_ports(instance_name or XArmConnection, robot))
    if instance_name:
        return declaration, real
    mock = MockJointConnection.blueprint(**fields).remappings(
        _arm_ports(MockJointConnection, robot)
    )
    return declaration, autoconnect(
        ChoiceAtom(name="arm", default="real", choices={"real": real, "mock": mock})
    )


def _arm_ports(
    module: type[ModuleBase] | str, robot: str
) -> list[tuple[type[ModuleBase] | str, str, str]]:
    """A connection's ports, onto the coordinator's ports for the robot named ``robot``."""
    return [
        (module, "position_command", f"{robot}_position_command"),
        (module, "velocity_command", f"{robot}_velocity_command"),
        (module, "joint_state", f"{robot}_joint_state"),
    ]


def xarm_coordinator(
    arm: XArmType,
    *,
    cls: type[ControlCoordinator] = XArmCoordinator,
    tasks: Sequence[TaskConfig] | None = None,
    gripper: bool = True,
    velocity: bool = False,
    simulated: bool = False,
    sim_module_kwargs: dict[str, Any] | None = None,
    **coordinator_kwargs: Any,
) -> MetaBlueprint:
    """A coordinator driving one xArm named "arm", through ``XArmConnection``, or
    ``MujocoArmConnection`` when simulated.

    Joints are named ``arm/joint1`` ... and ``arm/drive_joint``; a planner or IK
    task uses a model from ``make_xarm_model_config(..., robot="arm")``. The
    arm's address is given at run time (``--xarmconnection.ip 192.168.1.185``
    or ``XARMCONNECTION__IP``); ``--choice arm=mock`` runs without an arm.

    Args:
        arm: Which arm, e.g. "xarm7".
        cls: The coordinator, with the ports of ``XArmCoordinator``.
        tasks: The coordinator's tasks; if None, joint trajectories over the arm
            and its gripper, and the gripper's task, when it has one.
        gripper, velocity, simulated, sim_module_kwargs: See ``xarm_connection``.
        coordinator_kwargs: More coordinator settings, e.g. ``tick_rate``.
    """
    declaration, connection = xarm_connection(
        arm,
        gripper=gripper,
        velocity=velocity,
        simulated=simulated,
        sim_module_kwargs=sim_module_kwargs,
    )
    if tasks is None:
        arm_joints = joint_names(XARM_ARMS[arm][0], prefix="arm/joint")
        if gripper:
            tasks = [
                joint_trajectory_task([*arm_joints, "arm/drive_joint"]),
                xarm_gripper_task(),
            ]
        else:
            tasks = [joint_trajectory_task(arm_joints)]
    return autoconnect(
        cls.blueprint(
            instance_name="ControlCoordinator",
            connections=[declaration],
            tasks=list(tasks),
            **coordinator_kwargs,
        ),
        connection,
    )


def xarm_planning_model(arm: XArmType, **kwargs: Any) -> RobotModelConfig:
    """Planning and IK model for a single arm in its default scene."""
    dof, robot_type = XARM_ARMS[arm]
    if SIMULATED and arm in XARM_SIM_HOMES:
        kwargs.setdefault("base_pose", XARM7_SIM_BASE_POSE)
        kwargs.setdefault("home_joints", XARM_SIM_HOMES[arm])
    return make_xarm_model_config(dof, robot_type=robot_type, robot="arm", **kwargs)


def _planner_coordinator(arm: XArmType) -> MetaBlueprint:
    """Plan and run motions of an xArm with the xArm Gripper; see ``xarm_coordinator``."""
    return autoconnect(
        planner(
            model=xarm_planning_model(arm),
            gripper_tasks={"manipulator": "arm_gripper"},
        ),
        xarm_coordinator(arm, simulated=SIMULATED),
    )


xarm7_planner_coordinator = autoconnect(_planner_coordinator("xarm7"))
xarm6_planner_coordinator = autoconnect(_planner_coordinator("xarm6"))

# The real arm has no gripper here; the MuJoCo scene's comes with the arm.
coordinator_xarm7 = autoconnect(xarm_coordinator("xarm7", gripper=SIMULATED, simulated=SIMULATED))
coordinator_xarm6 = autoconnect(xarm_coordinator("xarm6", simulated=SIMULATED))

# The Lite 6's gripper has no joint, so XArmConnection does not drive it.
coordinator_lite6 = autoconnect(xarm_coordinator("lite6", gripper=False))

lite6_planner_coordinator = autoconnect(
    planner(
        model=make_lite6_model_config(add_gripper=True, robot="arm"),
        visualization={"backend": "viser"},
    ),
    xarm_coordinator("lite6", gripper=False),
)


class _DualXArmCoordinator(ControlCoordinator):
    left_arm_position_command: Out[JointState]
    left_arm_velocity_command: Out[JointState]
    left_arm_joint_state: In[JointState]
    right_arm_position_command: Out[JointState]
    right_arm_velocity_command: Out[JointState]
    right_arm_joint_state: In[JointState]


_left_xarm7, _left_xarm7_connection = xarm_connection(
    "xarm7", "left_arm", gripper=False, instance_name="left_arm"
)
_right_xarm6, _right_xarm6_connection = xarm_connection(
    "xarm6", "right_arm", gripper=False, instance_name="right_arm"
)

# Each arm's address: --left_arm.ip and --right_arm.ip.
coordinator_dual_xarm = autoconnect(
    _DualXArmCoordinator.blueprint(
        instance_name="ControlCoordinator",
        connections=[_left_xarm7, _right_xarm6],
        tasks=[
            joint_trajectory_task(
                [
                    *joint_names(7, prefix="left_arm/joint"),
                    *joint_names(6, prefix="right_arm/joint"),
                ]
            )
        ],
    ),
    _left_xarm7_connection,
    _right_xarm6_connection,
)


class _DualXArm6MockCoordinator(ControlCoordinator):
    left_position_command: Out[JointState]
    left_joint_state: In[JointState]
    right_position_command: Out[JointState]
    right_joint_state: In[JointState]


_dual_xarm6_model = make_dual_xarm6_model_config()


def _mock_xarm6(robot: str) -> tuple[ConnectionConfig, Blueprint]:
    """A mock xArm6 named ``robot``, with the planning model's joints ``<robot>/joint1`` ..."""
    mock = MockJointConnectionConfig(source=robot, model=xarm_model("xarm6", robot=robot))
    connection = MockJointConnection.blueprint(
        **{**mock.model_dump(), "instance_name": robot}
    ).remappings(
        [
            (robot, "position_command", f"{robot}_position_command"),
            (robot, "joint_state", f"{robot}_joint_state"),
        ]
    )
    return (
        ConnectionConfig(source=robot, model=mock.model, joint_commands=mock.joint_commands),
        connection,
    )


_mock_left_xarm6, _mock_left_xarm6_connection = _mock_xarm6("left")
_mock_right_xarm6, _mock_right_xarm6_connection = _mock_xarm6("right")

dual_xarm6_planner_coordinator = autoconnect(
    planner(
        model=_dual_xarm6_model,
        visualization={"backend": "viser"},
    ),
    _DualXArm6MockCoordinator.blueprint(
        instance_name="ControlCoordinator",
        connections=[_mock_left_xarm6, _mock_right_xarm6],
        tasks=[
            joint_trajectory_task(
                [
                    *_dual_xarm6_model.planning_groups[0].joint_names,
                    *_dual_xarm6_model.planning_groups[1].joint_names,
                ]
            )
        ],
    ),
    _mock_left_xarm6_connection,
    _mock_right_xarm6_connection,
)
