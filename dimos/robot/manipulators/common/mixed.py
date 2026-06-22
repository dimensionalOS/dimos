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
from dimos.robot.manipulators.piper.config import PIPER_FK_MODEL, make_piper_hardware
from dimos.robot.manipulators.xarm.config import XARM6_FK_MODEL, make_xarm_hardware

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
            name="traj_xarm", type="trajectory", joint_names=_xarm6_dual.joints, priority=10
        ),
        TaskConfig(
            name="traj_piper", type="trajectory", joint_names=_piper_dual.joints, priority=10
        ),
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

coordinator_teleop_dual = ControlCoordinator.blueprint(
    hardware=[_xarm6_teleop_hw, _piper_teleop_hw],
    tasks=[
        TaskConfig(
            name="teleop_xarm",
            type="teleop_ik",
            joint_names=_xarm6_teleop_hw.joints,
            priority=10,
            params={"model_path": XARM6_FK_MODEL, "ee_joint_id": 6, "hand": "left"},
        ),
        TaskConfig(
            name="teleop_piper",
            type="teleop_ik",
            joint_names=_piper_teleop_hw.joints,
            priority=10,
            params={"model_path": PIPER_FK_MODEL, "ee_joint_id": 6, "hand": "right"},
        ),
    ],
)
