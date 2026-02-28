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

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import TaskConfig, control_coordinator
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.manipulation.manipulation_blueprints import (
    _make_xarm6_config,
    _make_xarm7_config,
)
from dimos.manipulation.manipulation_module import manipulation_module
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import JointState
from dimos.teleop.keyboard.keyboard_teleop_module import keyboard_teleop_module
from dimos.utils.data import LfsPath

_XARM6_MODEL_PATH = LfsPath("xarm_description/urdf/xarm6/xarm6.urdf")
_XARM7_MODEL_PATH = LfsPath("xarm_description/urdf/xarm7/xarm7.urdf")

# XArm6 mock sim + keyboard teleop + Drake visualization
keyboard_teleop_xarm6 = autoconnect(
    keyboard_teleop_module(model_path=_XARM6_MODEL_PATH, ee_joint_id=6),
    control_coordinator(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[
            HardwareComponent(
                hardware_id="arm",
                hardware_type=HardwareType.MANIPULATOR,
                joints=make_joints("arm", 6),
                adapter_type="mock",
            ),
        ],
        tasks=[
            TaskConfig(
                name="cartesian_ik_arm",
                type="cartesian_ik",
                joint_names=[f"arm_joint{i + 1}" for i in range(6)],
                priority=10,
                model_path=_XARM6_MODEL_PATH,
                ee_joint_id=6,
            ),
        ],
    ),
    manipulation_module(
        robots=[_make_xarm6_config(name="arm", joint_prefix="arm_", add_gripper=False)],
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
    keyboard_teleop_module(model_path=_XARM7_MODEL_PATH, ee_joint_id=7),
    control_coordinator(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[
            HardwareComponent(
                hardware_id="arm",
                hardware_type=HardwareType.MANIPULATOR,
                joints=make_joints("arm", 7),
                adapter_type="mock",
            ),
        ],
        tasks=[
            TaskConfig(
                name="cartesian_ik_arm",
                type="cartesian_ik",
                joint_names=[f"arm_joint{i + 1}" for i in range(7)],
                priority=10,
                model_path=_XARM7_MODEL_PATH,
                ee_joint_id=7,
            ),
        ],
    ),
    manipulation_module(
        robots=[_make_xarm7_config(name="arm", joint_prefix="arm_", add_gripper=False)],
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
