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

from dimos.control.coordinator import ControlCoordinator, TaskConfig
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
        TaskConfig(
            name="traj_arm",
            type="trajectory",
            joint_names=[*_xarm6_dual.joints, *_piper_dual.joints],
            priority=10,
        ),
    ],
)

_xarm6_teleop_hw = make_xarm_hardware(
    "xarm_arm",
    6,
    adapter_type="xarm",
    address=global_config.xarm6_ip,
    gripper=True,
    canonical_joint_names=[f"xarm_arm/joint{i}" for i in range(1, 7)],
)
_piper_teleop_hw = make_piper_hardware(
    "piper_arm",
    adapter_type="piper",
    address=global_config.can_port or "can0",
    gripper=True,
)
_xarm6_teleop_model = make_xarm6_model_config(add_gripper=False, prefix="xarm_arm/")
_piper_teleop_model = make_piper_model_config()

coordinator_teleop_dual = ControlCoordinator.blueprint(
    hardware=[_xarm6_teleop_hw, _piper_teleop_hw],
    tasks=[
        teleop_ik_task(
            _xarm6_teleop_hw,
            name="teleop_xarm",
            bindings=[{"hand": "left", "target_frame": "xarm_arm/link6"}],
            robot_model=_xarm6_teleop_model,
            priority=10,
        ),
        teleop_ik_task(
            _piper_teleop_hw,
            name="teleop_piper",
            bindings=[{"hand": "right", "target_frame": "gripper_base"}],
            robot_model=_piper_teleop_model,
            priority=10,
        ),
        TaskConfig(
            name="xarm_arm_gripper",
            type="gripper",
            joint_names=["xarm_arm/gripper"],
            priority=20,
            stream_bind={"gripper_command": "left_gripper_command"},
        ),
        TaskConfig(
            name="piper_arm_gripper",
            type="gripper",
            joint_names=["piper_arm/gripper"],
            priority=20,
            stream_bind={"gripper_command": "right_gripper_command"},
        ),
    ],
)
