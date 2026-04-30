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

"""Keyboard teleop blueprint for the A-750 arm.

Launches the ControlCoordinator (mock adapter + CartesianIK), the
ManipulationModule (Drake/Meshcat visualization), and a pygame keyboard
teleop UI — all wired together via autoconnect.

Usage:
    dimos run keyboard-teleop-a750
"""

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.catalog.a750 import A750_FK_MODEL, a750 as _catalog_a750
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_a750_cfg = _catalog_a750(
    name="arm",
    adapter_type="a750" if global_config.device_path else "mock",
    device_path=global_config.device_path or "/dev/ttyACM0",
)

# Piper 6-DOF mock sim + keyboard teleop + Drake visualization
keyboard_teleop_a750 = autoconnect(
    KeyboardTeleopModule.blueprint(
        model_path=str(A750_FK_MODEL),
        ee_joint_id=_a750_cfg.dof,
        home_joints=_a750_cfg.home_joints,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_a750_cfg.to_hardware_component()],
        tasks=[
            _a750_cfg.to_task_config(
                task_type="cartesian_ik",
                task_name="cartesian_ik_arm",
                model_path=A750_FK_MODEL,
                ee_joint_id=_a750_cfg.dof,
            ),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_a750_cfg.to_robot_model_config()],
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

__all__ = ["keyboard_teleop_a750"]
