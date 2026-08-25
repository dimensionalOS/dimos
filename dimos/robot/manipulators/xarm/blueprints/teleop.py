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
from dimos.control.port_coordinator import PortControlCoordinator, PortTeleopControlCoordinator
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.robot.manipulators.common.blueprints import (
    eef_twist_task,
    teleop_ik_task,
    trajectory_task,
)
from dimos.robot.manipulators.common.coordinators import (
    ArmTwistCoordinator,
)
from dimos.robot.manipulators.common.sim import mujoco_if_sim
from dimos.robot.manipulators.xarm.config import (
    XARM6_SIM_PATH,
    XARM7_SIM_PATH,
    make_xarm6_model_config,
    make_xarm7_model_config,
    make_xarm_hardware,
    xarm6_hardware,
    xarm7_hardware,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule


class _PortArmTwistCoordinator(PortControlCoordinator):
    """Arm-twist coordinator connected to a robot connection module."""

    ee_twist_command: In[TwistStamped]


_xarm6_hw = (
    make_xarm_hardware("arm", 6, adapter_type="module", gripper=True)
    if global_config.simulation
    else xarm6_hardware("arm", gripper=True, mock_without_address=True)
)
_xarm7_hw = (
    make_xarm_hardware("arm", 7, adapter_type="module", gripper=True)
    if global_config.simulation
    else xarm7_hardware("arm", gripper=True, mock_without_address=True)
)
_xarm6_control_model = make_xarm6_model_config(add_gripper=False)
_xarm7_control_model = make_xarm7_model_config(add_gripper=False)
_keyboard_coordinator = (
    _PortArmTwistCoordinator if global_config.simulation else ArmTwistCoordinator
)

keyboard_teleop_xarm6 = autoconnect(
    KeyboardTeleopModule.blueprint(),
    _keyboard_coordinator.blueprint(
        instance_name="ControlCoordinator",
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm6_hw],
        tasks=[
            eef_twist_task(
                _xarm6_hw,
                robot_model=_xarm6_control_model,
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
        robots=[make_xarm6_model_config(add_gripper=True)],
        visualization={"backend": "viser"},
    ),
    *mujoco_if_sim(XARM6_SIM_PATH, 6),
)

keyboard_teleop_xarm7 = autoconnect(
    KeyboardTeleopModule.blueprint(),
    _keyboard_coordinator.blueprint(
        instance_name="ControlCoordinator",
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm7_hw],
        tasks=[
            eef_twist_task(
                _xarm7_hw,
                robot_model=_xarm7_control_model,
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
        robots=[make_xarm7_model_config(add_gripper=True)],
        visualization={"backend": "viser"},
    ),
    *mujoco_if_sim(XARM7_SIM_PATH, 7),
)

_xarm6_control_hw = make_xarm_hardware(
    "arm",
    6,
    adapter_type="xarm",
    address=global_config.xarm6_ip,
    gripper=True,
)

coordinator_servo_xarm6 = ControlCoordinator.blueprint(
    hardware=[_xarm6_control_hw],
    tasks=[
        TaskConfig(
            name="servo_arm",
            type="servo",
            joint_names=_xarm6_control_hw.joints,
            priority=10,
        ),
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
        TaskConfig(
            name="servo_arm",
            type="servo",
            joint_names=_xarm6_control_hw.joints,
            priority=10,
        ),
        TaskConfig(
            name="velocity_arm",
            type="velocity",
            joint_names=_xarm6_control_hw.joints,
            priority=10,
        ),
    ],
)

_xarm7_teleop_hw = xarm7_hardware("arm", gripper=True, mock_without_address=True)
_xarm6_teleop_hw = xarm6_hardware("arm", gripper=True, mock_without_address=True)
if global_config.simulation:
    _xarm7_teleop_hw = make_xarm_hardware("arm", 7, adapter_type="module", gripper=True)
    _xarm6_teleop_hw = make_xarm_hardware("arm", 6, adapter_type="module", gripper=True)
_xarm6_teleop_model = make_xarm6_model_config(add_gripper=True)
_xarm7_teleop_model = make_xarm7_model_config(add_gripper=True)
_teleop_coordinator = (
    PortTeleopControlCoordinator if global_config.simulation else TeleopControlCoordinator
)

# Dual-input arm: VR (teleop_ik) preempts browser keyboard (eef_twist) via
# higher priority; when VR is idle the always-active eef_twist holds/drives.
# The dedicated gripper task consumes normalized trigger commands independently.


coordinator_teleop_xarm7 = autoconnect(
    _teleop_coordinator.blueprint(
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
        robots=[_xarm7_teleop_model],
        visualization={"backend": "viser"},
    ),
    *mujoco_if_sim(XARM7_SIM_PATH, 7),
)

coordinator_teleop_xarm6 = autoconnect(
    _teleop_coordinator.blueprint(
        instance_name="ControlCoordinator",
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
        robots=[_xarm6_teleop_model],
        visualization={"backend": "viser"},
    ),
    *mujoco_if_sim(XARM6_SIM_PATH, 6),
)
