# Copyright 2025-2026 Dimensional Inc.
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

"""Keyboard teleop blueprints for XArm6 and XArm7.

Launches the ControlCoordinator (mock adapter + CartesianIK), the
ManipulationModule (Drake/Meshcat visualization), and a pygame keyboard
teleop UI — all wired together via autoconnect.

Usage:
    dimos run keyboard-teleop-xarm6
    dimos run keyboard-teleop-xarm7
"""

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.catalog.ufactory import (
    XARM6_FK_MODEL,
    XARM7_FK_MODEL,
    xarm6 as _catalog_xarm6,
    xarm7 as _catalog_xarm7,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_xarm6_cfg = _catalog_xarm6(
    name="arm",
    add_gripper=False,
    adapter_type="xarm" if global_config.xarm6_ip else "mock",
    address=global_config.xarm6_ip or None,
)
_xarm7_cfg = _catalog_xarm7(
    name="arm",
    add_gripper=False,
    adapter_type="xarm" if global_config.xarm7_ip else "mock",
    address=global_config.xarm7_ip or None,
)

# XArm6 mock sim + keyboard teleop + Drake visualization
keyboard_teleop_xarm6 = autoconnect(
    KeyboardTeleopModule.blueprint(model_path=XARM6_FK_MODEL, ee_joint_id=_xarm6_cfg.dof),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm6_cfg.to_hardware_component()],
        tasks=[
            _xarm6_cfg.to_task_config(
                task_type="cartesian_ik",
                task_name="cartesian_ik_arm",
                model_path=XARM6_FK_MODEL,
                ee_joint_id=_xarm6_cfg.dof,
            ),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_xarm6_cfg.to_robot_model_config()],
        enable_viz=True,
    ),
).transports(
    {
        ("cartesian_command", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# XArm7 mock sim + keyboard teleop + Drake visualization
keyboard_teleop_xarm7 = autoconnect(
    KeyboardTeleopModule.blueprint(model_path=XARM7_FK_MODEL, ee_joint_id=_xarm7_cfg.dof),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm7_cfg.to_hardware_component()],
        tasks=[
            _xarm7_cfg.to_task_config(
                task_type="cartesian_ik",
                task_name="cartesian_ik_arm",
                model_path=XARM7_FK_MODEL,
                ee_joint_id=_xarm7_cfg.dof,
            ),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_xarm7_cfg.to_robot_model_config()],
        enable_viz=True,
    ),
).transports(
    {
        ("cartesian_command", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

__all__ = ["keyboard_teleop_xarm6", "keyboard_teleop_xarm7"]
