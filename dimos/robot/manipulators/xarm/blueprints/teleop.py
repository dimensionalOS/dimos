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

from dimos.control.components import make_gripper_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.common.blueprints import (
    eef_twist_task,
    teleop_ik_task,
)
from dimos.robot.manipulators.common.sim import mujoco_if_sim
from dimos.robot.manipulators.xarm.config import (
    XARM6_FK_MODEL,
    XARM6_SIM_PATH,
    XARM7_FK_MODEL,
    XARM7_SIM_PATH,
    make_xarm6_model_config,
    make_xarm7_model_config,
    make_xarm_hardware,
    xarm6_hardware,
    xarm7_hardware,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_xarm6_hw = make_xarm_hardware(
    "arm",
    6,
    adapter_type="xarm" if global_config.xarm6_ip else "mock",
    address=global_config.xarm6_ip,
)
_xarm7_hw = make_xarm_hardware(
    "arm",
    7,
    adapter_type="xarm" if global_config.xarm7_ip else "mock",
    address=global_config.xarm7_ip,
)

_xarm6_model = make_xarm6_model_config(add_gripper=False)
_xarm7_model = make_xarm7_model_config(add_gripper=False)

keyboard_teleop_xarm6 = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm6_hw],
        tasks=[
            eef_twist_task(
                _xarm6_hw,
                robot_model=_xarm6_model,
            )
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_xarm6_model],
        visualization={"backend": "meshcat"},
    ),
)

keyboard_teleop_xarm7 = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm7_hw],
        tasks=[
            eef_twist_task(
                _xarm7_hw,
                robot_model=_xarm7_model,
            )
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_xarm7_model],
        visualization={"backend": "meshcat"},
    ),
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

_xarm7_teleop_hw = xarm7_hardware("arm", gripper=True)
_xarm6_teleop_hw = xarm6_hardware("arm", gripper=True)

coordinator_teleop_xarm7 = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_xarm7_teleop_hw],
        tasks=[
            teleop_ik_task(
                _xarm7_teleop_hw,
                model_path=XARM7_FK_MODEL,
                ee_joint_id=7,
                hand="right",
                name="teleop_xarm",
                params={
                    "gripper_joint": make_gripper_joints("arm")[0],
                    "gripper_open_pos": 0.85,
                    "gripper_closed_pos": 0.0,
                },
            ),
        ],
    ),
    *mujoco_if_sim(XARM7_SIM_PATH, len(_xarm7_teleop_hw.joints)),
)

coordinator_teleop_xarm6 = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_xarm6_teleop_hw],
        tasks=[
            teleop_ik_task(
                _xarm6_teleop_hw,
                model_path=XARM6_FK_MODEL,
                ee_joint_id=6,
                hand="right",
                name="teleop_xarm",
                params={
                    "gripper_joint": make_gripper_joints("arm")[0],
                    "gripper_open_pos": 0.85,
                    "gripper_closed_pos": 0.0,
                },
            ),
        ],
    ),
    *mujoco_if_sim(XARM6_SIM_PATH, len(_xarm6_teleop_hw.joints)),
)
