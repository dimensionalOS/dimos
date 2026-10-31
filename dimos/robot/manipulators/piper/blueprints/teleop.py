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

from dimos.control.coordinator import TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.manipulators._modeling import joint_names
from dimos.robot.manipulators.common.blueprints import (
    cartesian_ik_task,
    eef_twist_task,
    teleop_ik_task,
    trajectory_task,
)
from dimos.robot.manipulators.common.coordinators import (
    ArmPoseCoordinator,
    ArmTwistCoordinator,
)
from dimos.robot.manipulators.common.sim import mujoco_if_sim
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.piper.config import (
    PIPER_DOF,
    PIPER_SIM_PATH,
    make_piper_hardware,
    make_piper_model_config,
    piper_hardware,
    piper_model,
)
from dimos.robot.manipulators.piper.connection import PiperConnection
from dimos.robot.urdf_manager import UrdfManager
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_piper_keyboard_hw = make_piper_hardware(
    "arm",
    adapter_type="piper",
    gripper=True,
    mock_without_address=True,
)
_piper_model = make_piper_model_config()

keyboard_teleop_piper = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ArmTwistCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_piper_keyboard_hw],
        tasks=[
            eef_twist_task(
                _piper_keyboard_hw,
                robot_model=_piper_model,
                target_frame="gripper_base",
            ),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
            ),
            trajectory_task(_piper_keyboard_hw),
        ],
    ),
    ManipulationModule.blueprint(
        model=_piper_model,
        visualization={"backend": "viser"},
    ),
)

_piper_mock_cartesian_hw = make_piper_hardware(
    "arm",
    gripper=False,
)

coordinator_cartesian_ik_mock = ArmPoseCoordinator.blueprint(
    instance_name="ControlCoordinator",
    hardware=[_piper_mock_cartesian_hw],
    tasks=[
        cartesian_ik_task(
            _piper_mock_cartesian_hw,
            robot_model=_piper_model,
            target_frame="gripper_base",
        )
    ],
)

_piper_teleop_hw = piper_hardware("arm")


coordinator_teleop_piper = autoconnect(
    TeleopControlCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[_piper_teleop_hw],
        tasks=[
            teleop_ik_task(
                _piper_teleop_hw,
                robot_model=_piper_model,
                bindings=[
                    {
                        "hand": "left",
                        "target_frame": "gripper_base",
                    }
                ],
                name="teleop_piper",
            ),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
                stream_bind={"gripper_command": "left_gripper_command"},
            ),
            trajectory_task(_piper_teleop_hw),
        ],
    ),
    ManipulationModule.blueprint(
        model=_piper_model,
        visualization={"backend": "viser"},
    ),
    *mujoco_if_sim(PIPER_SIM_PATH, len(_piper_teleop_hw.joints)),
)

_piper_cartesian_hw = make_piper_hardware(
    "arm",
    adapter_type="piper",
    address="can0",
    gripper=True,
)

coordinator_cartesian_ik_piper = ArmPoseCoordinator.blueprint(
    instance_name="ControlCoordinator",
    hardware=[_piper_cartesian_hw],
    tasks=[
        cartesian_ik_task(
            _piper_cartesian_hw,
            robot_model=_piper_model,
            target_frame="gripper_base",
        )
    ],
)


class PiperTwistCoordinator(ArmTwistCoordinator):
    """End-effector twists and gripper openings in, from the Desktop Manipulation app;
    one Piper named "arm" out, driven through ``PiperConnection`` by joint targets."""

    right_gripper_command: In[Float32]
    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


# The connection module's ports, onto the coordinator's ports for the robot named "arm".
_ARM_REMAPS: list[tuple[type[ModuleBase], str, str]] = [
    (PiperConnection, "position_command", "arm_position_command"),
    (PiperTwistCoordinator, "arm_joint_state", "joint_state"),
]


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
            model=model,
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
                # arm/gripper is open at its upper end, the task's default.
                TaskConfig(
                    name="arm_gripper",
                    type="gripper",
                    joint_names=["arm/gripper"],
                    priority=20,
                    stream_bind={"gripper_command": "right_gripper_command"},
                ),
            ],
        ),
        PiperConnection.blueprint(gripper=True),
        UrdfManager.blueprint(model=model),
    )


# The blueprint registry finds a blueprint only by a line ending in a call like `.remappings`.
piper_teleop = _teleop().remappings(_ARM_REMAPS)
