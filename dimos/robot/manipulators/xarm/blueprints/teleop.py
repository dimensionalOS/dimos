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

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.common.blueprints import cartesian_ik_task
from dimos.robot.manipulators.xarm.config import (
    XARM6_FK_MODEL,
    XARM7_FK_MODEL,
    make_xarm6_model_config,
    make_xarm7_model_config,
    make_xarm_hardware,
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

keyboard_teleop_xarm6 = autoconnect(
    KeyboardTeleopModule.blueprint(
        model_path=XARM6_FK_MODEL,
        ee_joint_id=6,
        joint_names=_xarm6_hw.joints,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm6_hw],
        tasks=[cartesian_ik_task(_xarm6_hw, model_path=XARM6_FK_MODEL, ee_joint_id=6)],
    ),
    ManipulationModule.blueprint(
        robots=[make_xarm6_model_config(add_gripper=False)],
        visualization={"backend": "meshcat"},
    ),
)

keyboard_teleop_xarm7 = autoconnect(
    KeyboardTeleopModule.blueprint(
        model_path=XARM7_FK_MODEL,
        ee_joint_id=7,
        joint_names=_xarm7_hw.joints,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm7_hw],
        tasks=[cartesian_ik_task(_xarm7_hw, model_path=XARM7_FK_MODEL, ee_joint_id=7)],
    ),
    ManipulationModule.blueprint(
        robots=[make_xarm7_model_config(add_gripper=False)],
        visualization={"backend": "meshcat"},
    ),
)
