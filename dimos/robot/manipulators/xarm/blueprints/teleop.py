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
    XARM_GRIPPER_PARAMS,
    make_xarm6_model_config,
    make_xarm7_model_config,
    make_xarm_hardware,
    xarm6_hardware,
    xarm7_hardware,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_xarm6_hw = xarm6_hardware("arm", gripper=True, mock_without_address=True)
_xarm7_hw = xarm7_hardware("arm", gripper=True, mock_without_address=True)

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
                model_path=XARM6_FK_MODEL,
                ee_joint_id=6,
                params=XARM_GRIPPER_PARAMS,
            )
        ],
    ),
    ManipulationModule.blueprint(
        robots=[make_xarm6_model_config(add_gripper=True)],
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
                model_path=XARM7_FK_MODEL,
                ee_joint_id=7,
                params=XARM_GRIPPER_PARAMS,
            )
        ],
    ),
    ManipulationModule.blueprint(
        robots=[make_xarm7_model_config(add_gripper=True)],
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

# Dual-input arm: VR (teleop_ik) preempts browser keyboard (eef_twist) via
# higher priority; when VR is idle the always-active eef_twist holds/drives.
# While engaged, VR also owns the gripper joint (trigger), so the browser
# gripper toggle only takes effect when VR is disengaged.

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
                priority=20,
                params={**XARM_GRIPPER_PARAMS, "max_joint_delta_deg": 50.0},
            ),
            eef_twist_task(
                _xarm7_teleop_hw,
                model_path=XARM7_FK_MODEL,
                ee_joint_id=7,
                priority=10,
                params={**XARM_GRIPPER_PARAMS, "max_joint_delta_deg": 50.0},
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
                priority=20,
                params={**XARM_GRIPPER_PARAMS, "max_joint_delta_deg": 50.0},
            ),
            eef_twist_task(
                _xarm6_teleop_hw,
                model_path=XARM6_FK_MODEL,
                ee_joint_id=6,
                priority=10,
                params={**XARM_GRIPPER_PARAMS, "max_joint_delta_deg": 50.0},
            ),
        ],
    ),
    *mujoco_if_sim(XARM6_SIM_PATH, len(_xarm6_teleop_hw.joints)),
)
