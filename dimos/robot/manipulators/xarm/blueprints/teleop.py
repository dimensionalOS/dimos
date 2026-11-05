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

"""Keyboard teleop blueprints for xArm6 and xArm7."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.common.blueprints import (
    eef_twist_task,
    teleop_ik_task,
    trajectory_task,
)
from dimos.robot.manipulators.common.coordinators import (
    ArmTwistCoordinator,
)
from dimos.robot.manipulators.common.sim import mujoco_if_sim
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.xarm.config import (
    XARM6_SIM_PATH,
    XARM7_SIM_PATH,
    XARM_ARMS,
    XArmType,
    lite6_hardware,
    make_lite6_model_config,
    make_xarm6_model_config,
    make_xarm7_model_config,
    make_xarm_hardware,
    make_xarm_model_config,
    xarm6_hardware,
    xarm7_hardware,
    xarm_model,
)
from dimos.robot.manipulators.xarm.connection import XArmConnection
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_xarm6_hw = xarm6_hardware("arm", gripper=True, mock_without_address=True)
_xarm7_hw = xarm7_hardware("arm", gripper=True, mock_without_address=True)
_xarm6_control_model = make_xarm6_model_config(add_gripper=False)
_xarm7_control_model = make_xarm7_model_config(add_gripper=False)

keyboard_teleop_xarm6 = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ArmTwistCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm6_hw],
        tasks=[
            eef_twist_task(
                _xarm6_hw,
                robot_model=_xarm6_control_model,
                target_frame="link6",
                timeout=0.0,
            ),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
            ),
        ],
    ),
    ManipulationModule.blueprint(
        model=make_xarm6_model_config(
            add_gripper=True,
            gripper_hardware_id="arm",
        ),
        visualization={"backend": "viser"},
    ),
)

keyboard_teleop_xarm7 = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ArmTwistCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm7_hw],
        tasks=[
            eef_twist_task(
                _xarm7_hw,
                robot_model=_xarm7_control_model,
                target_frame="link7",
                timeout=0.0,
            ),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
            ),
        ],
    ),
    ManipulationModule.blueprint(
        model=make_xarm7_model_config(
            add_gripper=True,
            gripper_hardware_id="arm",
        ),
        visualization={"backend": "viser"},
    ),
)

_lite6_hw = lite6_hardware("arm", gripper=True, mock_without_address=True)

keyboard_teleop_lite6 = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ArmTwistCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_lite6_hw],
        tasks=[
            eef_twist_task(
                _lite6_hw,
                robot_model=make_lite6_model_config(add_gripper=False),
                target_frame="link6",
                timeout=0.0,
            ),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
            ),
        ],
    ),
    ManipulationModule.blueprint(
        model=make_lite6_model_config(
            add_gripper=True,
            gripper_hardware_id="arm",
        ),
        visualization={"backend": "viser"},
    ),
)

_xarm6_control_hw = make_xarm_hardware(
    "arm",
    6,
    adapter_type="xarm",
    gripper=True,
)

coordinator_trajectory_xarm6 = ControlCoordinator.blueprint(
    hardware=[_xarm6_control_hw],
    tasks=[
        trajectory_task(_xarm6_control_hw),
    ],
)

coordinator_velocity_xarm6 = ControlCoordinator.blueprint(
    hardware=[_xarm6_control_hw],
    tasks=[
        TaskConfig(
            name="velocity_arm",
            type="velocity",
            joint_names=_xarm6_control_hw.joints,
            priority=10,
        ),
    ],
)

coordinator_combined_xarm6 = ControlCoordinator.blueprint(
    hardware=[_xarm6_control_hw],
    tasks=[
        trajectory_task(_xarm6_control_hw),
        TaskConfig(
            name="velocity_arm",
            type="velocity",
            joint_names=_xarm6_control_hw.joints,
            priority=10,
        ),
    ],
)

_xarm7_teleop_hw = xarm7_hardware(
    "arm",
    gripper=True,
    mock_without_address=True,
)
_xarm6_teleop_hw = xarm6_hardware(
    "arm",
    gripper=True,
    mock_without_address=True,
)
_xarm6_teleop_model = make_xarm6_model_config(
    add_gripper=True,
    gripper_hardware_id="arm",
)
_xarm7_teleop_model = make_xarm7_model_config(
    add_gripper=True,
    gripper_hardware_id="arm",
)

# Dual-input arm: VR (teleop_ik) preempts browser keyboard (eef_twist) via
# higher priority; when VR is idle the always-active eef_twist holds/drives.
# The dedicated gripper task consumes normalized trigger commands independently.


coordinator_teleop_xarm7 = autoconnect(
    TeleopControlCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[_xarm7_teleop_hw],
        tasks=[
            teleop_ik_task(
                _xarm7_teleop_hw,
                robot_model=_xarm7_teleop_model,
                bindings=[
                    {
                        "hand": "right",
                        "target_frame": "link_tcp",
                    }
                ],
                name="teleop_xarm",
                priority=20,
            ),
            eef_twist_task(
                _xarm7_teleop_hw,
                robot_model=_xarm7_control_model,
                target_frame="link7",
                priority=10,
                timeout=0.0,
            ),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
                stream_bind={"gripper_command": "right_gripper_command"},
            ),
            trajectory_task(_xarm7_teleop_hw),
        ],
    ),
    ManipulationModule.blueprint(
        model=_xarm7_teleop_model,
        visualization={"backend": "viser"},
    ),
    *mujoco_if_sim(XARM7_SIM_PATH, len(_xarm7_teleop_hw.joints)),
)

coordinator_teleop_xarm6 = autoconnect(
    TeleopControlCoordinator.blueprint(
        hardware=[_xarm6_teleop_hw],
        tasks=[
            teleop_ik_task(
                _xarm6_teleop_hw,
                robot_model=_xarm6_teleop_model,
                bindings=[
                    {
                        "hand": "right",
                        "target_frame": "link_tcp",
                    }
                ],
                name="teleop_xarm",
                priority=20,
            ),
            eef_twist_task(
                _xarm6_teleop_hw,
                robot_model=_xarm6_control_model,
                target_frame="link6",
                priority=10,
                timeout=0.0,
            ),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
                stream_bind={"gripper_command": "right_gripper_command"},
            ),
            trajectory_task(_xarm6_teleop_hw),
        ],
    ),
    ManipulationModule.blueprint(
        model=_xarm6_teleop_model,
        visualization={"backend": "viser"},
    ),
    *mujoco_if_sim(XARM6_SIM_PATH, len(_xarm6_teleop_hw.joints)),
)


class XArmTwistCoordinator(ArmTwistCoordinator):
    """End-effector twists in, from the keyboard or a browser; one xArm named "arm" out,
    driven through ``XArmConnection``."""

    arm_position_command: Out[JointState]
    arm_velocity_command: Out[JointState]
    arm_joint_state: In[JointState]


# The connection module's ports, onto the coordinator's ports for the robot named "arm".
_ARM_REMAPS: list[tuple[type[ModuleBase], str, str]] = [
    (XArmConnection, "position_command", "arm_position_command"),
    (XArmConnection, "velocity_command", "arm_velocity_command"),
    (XArmTwistCoordinator, "arm_joint_state", "joint_state"),
]


def _keyboard_teleop(arm: XArmType) -> Blueprint:
    """Jog an xArm's end from the keyboard, through ``XArmConnection``. Arm only, no gripper.

    Wire it up with ``.remappings(_ARM_REMAPS)``. The arm's address is given at
    run time, after the blueprint name (``--xarmconnection.ip 192.168.1.185``)
    or as ``XARMCONNECTION__IP``.

    Args:
        arm: Which arm, e.g. "xarm7". It picks the coordinator's model, the IK
            model and the connection's URDF, so they agree.
    """
    dof, robot_type = XARM_ARMS[arm]
    ik_model = make_xarm_model_config(dof, robot_type=robot_type, robot="arm", add_gripper=False)
    return autoconnect(
        KeyboardTeleopModule.blueprint(),
        XArmTwistCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=xarm_model(arm, robot="arm"),
            tasks=[
                TaskConfig(
                    name=EEF_TWIST_TASK_NAME,
                    type="eef_twist",
                    joint_names=list(ik_model.joint_names),
                    priority=10,
                    params={
                        "robot_model": ik_model,
                        "target_frame": f"link{dof}",
                        # The end stops 0.3 s after the last twist.
                        "timeout": 0.3,
                    },
                ),
            ],
        ),
        XArmConnection.blueprint(arm=arm),
    )


# The blueprint registry finds a blueprint only by a line ending in a call like `.remappings`.
xarm7_keyboard_teleop = _keyboard_teleop("xarm7").remappings(_ARM_REMAPS)
xarm6_keyboard_teleop = _keyboard_teleop("xarm6").remappings(_ARM_REMAPS)
