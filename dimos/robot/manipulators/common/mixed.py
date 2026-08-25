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

"""Mixed-manipulator coordinator blueprints."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.global_config import global_config
from dimos.robot.manipulators.common.blueprints import teleop_ik_task
from dimos.robot.manipulators.piper.config import (
    make_piper_hardware,
    make_piper_model_config,
)
from dimos.robot.manipulators.xarm.config import (
    make_xarm6_model_config,
    make_xarm_hardware,
)

_xarm6_dual = make_xarm_hardware(
    "xarm_arm",
    6,
    adapter_type="xarm",
    address=global_config.xarm6_ip,
)
_piper_dual = make_piper_hardware(
    "piper_arm",
    adapter_type="piper",
    address=global_config.can_port or "can0",
    gripper=True,
)

coordinator_piper_xarm = ControlCoordinator.blueprint(
    hardware=[_xarm6_dual, _piper_dual],
    tasks=[
        joint_trajectory_task([*_xarm6_dual.joints, *_piper_dual.joints]),
    ],
)

_xarm6_teleop_hw = make_xarm_hardware(
    "xarm_arm",
    6,
    adapter_type="xarm",
    address=global_config.xarm6_ip,
    gripper=True,
)
_piper_teleop_hw = make_piper_hardware(
    "piper_arm",
    adapter_type="piper",
    address=global_config.can_port or "can0",
    gripper=True,
)

coordinator_teleop_dual = TeleopControlCoordinator.blueprint(
    hardware=[_xarm6_teleop_hw, _piper_teleop_hw],
    tasks=[
        teleop_ik_task(
            _xarm6_teleop_hw,
            name="teleop_xarm",
            robot_model=make_xarm6_model_config("xarm_arm"),
            bindings=[{"hand": "left", "target_frame": "link_tcp"}],
            priority=10,
        ),
        teleop_ik_task(
            _piper_teleop_hw,
            name="teleop_piper",
            robot_model=make_piper_model_config("piper_arm"),
            bindings=[{"hand": "right", "target_frame": "gripper_base"}],
            priority=10,
        ),
    ],
)
