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

from dimos.control.components import make_gripper_joints
from dimos.control.coordinator import TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule
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
from dimos.robot.manipulators.piper.config import (
    PIPER_SIM_PATH,
    make_piper_hardware,
    make_piper_model_config,
    piper_hardware,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_piper_keyboard_hw = make_piper_hardware(
    "arm",
    adapter_type="piper" if global_config.can_port else "mock",
    address=global_config.can_port or "can0",
    gripper=True,
    gripper_open_position=0.07,
    gripper_closed_position=0.0,
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
            eef_twist_task(_piper_keyboard_hw, robot_model=_piper_model),
            TaskConfig(
                name="servo_gripper",
                type="servo",
                joint_names=["arm/gripper"],
                priority=20,
                params={"timeout": 0.0, "default_positions": [0.0]},
            ),
            trajectory_task(_piper_keyboard_hw),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_piper_model],
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

_piper_teleop_hw = piper_hardware("arm", gripper_open_position=0.07, gripper_closed_position=0.0)


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
                        "gripper_joint": make_gripper_joints("arm")[0],
                        "gripper_open_position": 1.0,
                        "gripper_closed_position": 0.0,
                    }
                ],
                name="teleop_piper",
            ),
            trajectory_task(_piper_teleop_hw),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_piper_model],
        visualization={"backend": "viser"},
    ),
    *mujoco_if_sim(PIPER_SIM_PATH, len(_piper_teleop_hw.joints)),
)

_piper_cartesian_hw = make_piper_hardware(
    "arm",
    adapter_type="piper",
    address=global_config.can_port or "can0",
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
