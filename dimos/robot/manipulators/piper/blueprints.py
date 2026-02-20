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

"""Keyboard teleop blueprint for the Piper arm.

Launches the ControlCoordinator (mock adapter + CartesianIK), the
ManipulationModule (Drake/Meshcat visualization), and a pygame keyboard
teleop UI — all wired together via autoconnect.

Usage:
    dimos run keyboard-teleop-piper
"""

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import TaskConfig, control_coordinator
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.manipulation.manipulation_module import manipulation_module
from dimos.manipulation.planning.spec import RobotModelConfig
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.msgs.sensor_msgs import JointState
from dimos.teleop.keyboard.keyboard_teleop_module import keyboard_teleop_module
from dimos.utils.data import LfsPath, get_data

_PIPER_MODEL_PATH = LfsPath("piper_description/mujoco_model/piper_no_gripper_description.xml")
_PIPER_DATA = get_data("piper_description")

# Piper 6-DOF mock sim + keyboard teleop + Drake visualization
keyboard_teleop_piper = autoconnect(
    keyboard_teleop_module(model_path=_PIPER_MODEL_PATH, ee_joint_id=6),
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
                model_path=_PIPER_MODEL_PATH,
                ee_joint_id=6,
            ),
        ],
    ),
    manipulation_module(
        robots=[
            RobotModelConfig(
                name="arm",
                urdf_path=_PIPER_DATA / "urdf" / "piper_description.xacro",
                base_pose=PoseStamped(
                    position=Vector3(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ),
                joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                end_effector_link="gripper_base",
                base_link="base_link",
                package_paths={
                    "piper_description": _PIPER_DATA,
                    "piper_gazebo": _PIPER_DATA,  # xacro refs $(find piper_gazebo); unused by Drake
                },
                joint_name_mapping={f"arm_joint{i}": f"joint{i}" for i in range(1, 7)},
                auto_convert_meshes=True,
                home_joints=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ),
        ],
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

__all__ = ["keyboard_teleop_piper"]
