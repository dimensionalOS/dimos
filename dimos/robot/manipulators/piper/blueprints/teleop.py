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

"""Piper teleop blueprints."""

from __future__ import annotations

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import movable_joints
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.stream import In, Out
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.manipulators._modeling import joint_names
from dimos.robot.manipulators.common.coordinators import (
    ArmPoseCoordinator,
    ArmTwistCoordinator,
)
from dimos.robot.manipulators.common.topics import CARTESIAN_IK_TASK_NAME, EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.piper.blueprints.basic import (
    PIPER_ARM_JOINTS,
    PIPER_JOINTS,
    SIMULATED,
    piper_arm,
    piper_declaration,
    piper_remaps,
)
from dimos.robot.manipulators.piper.config import (
    PIPER_DOF,
    make_piper_model_config,
    piper_model,
)
from dimos.robot.manipulators.piper.connection import PiperConnection
from dimos.robot.urdf_manager import UrdfManager
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

# The Piper named "arm", for planning and IK: joints arm/joint1 ... arm/joint6.
_piper_model = make_piper_model_config(robot="arm")


def _gripper_task(stream: str | None = None) -> TaskConfig:
    """Open and close arm/gripper, open at its upper end (the task's default),
    from the gripper openings on ``stream`` (0 closed, 1 open)."""
    return TaskConfig(
        name="arm_gripper",
        type="gripper",
        joint_names=["arm/gripper"],
        priority=20,
        stream_bind={"gripper_command": stream} if stream else {},
    )


class PiperKeyboardCoordinator(ArmTwistCoordinator):
    """End-effector twists and gripper openings in, from the keyboard; one Piper
    named "arm" out, driven by joint targets."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


class PiperPoseCoordinator(ArmPoseCoordinator):
    """End-effector poses in; one Piper named "arm" out, driven by joint targets."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


class PiperWebXRCoordinator(TeleopControlCoordinator):
    """VR controller poses and gripper openings in, from WebXR; one Piper named
    "arm" out, driven by joint targets."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


# The blueprint registry finds a blueprint only by a line ending in a call like `.remappings`.
# With --simulation the Piper runs in MuJoCo; otherwise the CAN port is given at
# run time (--piperconnection.can-port can0), or --choice arm=mock runs without one.
keyboard_teleop_piper = autoconnect(
    KeyboardTeleopModule.blueprint(),
    PiperKeyboardCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        connections=[piper_declaration()],
        tasks=[
            TaskConfig(
                name=EEF_TWIST_TASK_NAME,
                type="eef_twist",
                joint_names=PIPER_ARM_JOINTS,
                priority=10,
                params={
                    "robot_model": _piper_model,
                    "target_frame": "gripper_base",
                    "timeout": 0.3,
                    "max_joint_velocity_rad_s": 5.0,
                    "max_command_tracking_error_deg": 10.0,
                },
            ),
            _gripper_task(),
            joint_trajectory_task(PIPER_JOINTS),
        ],
    ),
    ManipulationModule.blueprint(
        gripper_tasks={"manipulator": "arm_gripper"},
        model=_piper_model,
        visualization={"backend": "viser"},
    ),
    piper_arm(simulated=SIMULATED),
).remappings(piper_remaps(PiperKeyboardCoordinator))


def _cartesian_ik_task() -> TaskConfig:
    """Move the Piper's gripper base to the poses that come in on ``cartesian_command``."""
    return TaskConfig(
        name=CARTESIAN_IK_TASK_NAME,
        type="cartesian_ik",
        joint_names=PIPER_ARM_JOINTS,
        priority=10,
        params={"robot_model": _piper_model, "target_frame": "gripper_base"},
    )


_mock_arm_model = piper_model(robot="arm")

# A Piper without its gripper on MockJointConnection: every joint target comes straight back.
coordinator_cartesian_ik_mock = autoconnect(
    PiperPoseCoordinator.blueprint(
        instance_name="ControlCoordinator",
        connections=[
            ConnectionConfig(
                source="arm",
                model=_mock_arm_model,
                joint_commands={
                    joint.name: (Interface.POSITION,) for joint in movable_joints(_mock_arm_model)
                },
            )
        ],
        tasks=[_cartesian_ik_task()],
    ),
    MockJointConnection.blueprint(source="arm", model=_mock_arm_model),
).remappings(
    [
        (MockJointConnection, "position_command", "arm_position_command"),
        (PiperPoseCoordinator, "arm_joint_state", "joint_state"),
    ]
)

# With --simulation the Piper runs in MuJoCo; otherwise the CAN port is given at
# run time (--piperconnection.can-port can0), or --choice arm=mock runs without one.
coordinator_teleop_piper = autoconnect(
    PiperWebXRCoordinator.blueprint(
        instance_name="ControlCoordinator",
        connections=[piper_declaration()],
        tasks=[
            TaskConfig(
                name="teleop_piper",
                type="teleop_ik",
                joint_names=PIPER_ARM_JOINTS,
                priority=10,
                params={
                    "robot_model": _piper_model,
                    "bindings": [{"hand": "left", "target_frame": "gripper_base"}],
                    "solver_type": PinkPoseTargetSolver,
                },
            ),
            _gripper_task("left_gripper_command"),
            joint_trajectory_task(PIPER_JOINTS),
        ],
    ),
    ManipulationModule.blueprint(
        gripper_tasks={"manipulator": "arm_gripper"},
        model=_piper_model,
        visualization={"backend": "viser"},
    ),
    piper_arm(simulated=SIMULATED),
).remappings(piper_remaps(PiperWebXRCoordinator))

# On the Piper at CAN port can0, unless given another (--piperconnection.can-port).
coordinator_cartesian_ik_piper = autoconnect(
    PiperPoseCoordinator.blueprint(
        instance_name="ControlCoordinator",
        connections=[piper_declaration()],
        tasks=[_cartesian_ik_task()],
    ),
    PiperConnection.blueprint(gripper=True, can_port="can0"),
).remappings(
    [
        (PiperConnection, "position_command", "arm_position_command"),
        *piper_remaps(PiperPoseCoordinator),
    ]
)


class PiperTwistCoordinator(ArmTwistCoordinator):
    """End-effector twists and gripper openings in, from the Desktop Manipulation app;
    one Piper named "arm" out, driven through ``PiperConnection`` by joint targets."""

    right_gripper_command: In[Float32]
    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


# The coordinator's readings of the robot named "arm", from the connection.
_ARM_REMAPS = piper_remaps(PiperTwistCoordinator)


def _teleop() -> Blueprint:
    """Jog a Piper's end and open and close its gripper from the Desktop
    Manipulation app, through ``PiperConnection``. ``UrdfManager`` serves the
    Piper's URDF and puts its joints on tf, so the app can draw it.

    Wire it up with ``.remappings(_ARM_REMAPS)``. The CAN port is given at run
    time, after the blueprint name (``--piperconnection.can-port can0``) or as
    ``PIPERCONNECTION__CAN_PORT``.
    """
    ik_model = RobotModelConfig(
        model=piper_model(robot="arm").with_default_joint_acceleration_limit(2.0),
        joint_names=joint_names(PIPER_DOF, prefix="arm/joint"),
    )
    model = piper_model(gripper=True, robot="arm")
    return autoconnect(
        PiperTwistCoordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[piper_declaration()],
            tasks=[
                TaskConfig(
                    name=EEF_TWIST_TASK_NAME,
                    type="eef_twist",
                    joint_names=list(ik_model.joint_names),
                    priority=10,
                    params={
                        "robot_model": ik_model,
                        "target_frame": "link6",
                        # The app sends twists only while a jog is held: the
                        # end stops 0.3 s after the last one.
                        "timeout": 0.3,
                    },
                ),
                _gripper_task("right_gripper_command"),
            ],
        ),
        piper_arm(simulated=False),
        UrdfManager.blueprint(model=model),
    )


# The blueprint registry finds a blueprint only by a line ending in a call like `.remappings`.
piper_teleop = _teleop().remappings(_ARM_REMAPS)
