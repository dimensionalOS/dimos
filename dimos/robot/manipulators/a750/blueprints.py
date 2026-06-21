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

import math
from pathlib import Path

from dimos.control.blueprints._hardware import A750_FK_MODEL, a750
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule
from dimos.utils.data import LfsPath

A750_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("base_link", "link1"),
    ("base_link", "link2"),
    ("left_finger_link", "link3"),
    ("left_finger_link", "link4"),
    ("left_finger_link", "link5"),
    ("left_finger_link", "link6"),
    ("left_finger_link", "right_finger_link"),
    ("link1", "link2"),
    ("link2", "link3"),
    ("link2", "link4"),
    ("link3", "link4"),
    ("link3", "link5"),
    ("link3", "right_finger_link"),
    ("link4", "link5"),
    ("link4", "link6"),
    ("link4", "right_finger_link"),
    ("link5", "link6"),
    ("link5", "right_finger_link"),
    ("link6", "right_finger_link"),
]

_A750_MODEL_PATH = LfsPath("a750_description") / "urdf/a750_rev1.urdf"
_A750_HOME_JOINTS = [0.0, 0.0, -math.radians(90), 0.0, 0.0, 0.0]
_A750_PACKAGE_PATHS: dict[str, Path] = {
    "a750_description": LfsPath("a750_description"),
    "a750_gazebo": LfsPath("a750_description"),
}


def _a750_model_config() -> RobotModelConfig:
    return RobotModelConfig(
        name="arm",
        model_path=_A750_MODEL_PATH,
        base_pose=PoseStamped(
            position=Vector3(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        joint_names=[f"joint{i}" for i in range(1, 7)],
        end_effector_link="gripper_base",
        base_link="base_link",
        package_paths=_A750_PACKAGE_PATHS,
        auto_convert_meshes=True,
        collision_exclusion_pairs=A750_GRIPPER_COLLISION_EXCLUSIONS,
        joint_name_mapping={f"arm/joint{i}": f"joint{i}" for i in range(1, 7)},
        coordinator_task_name="traj_arm",
        gripper_hardware_id="arm",
        home_joints=_A750_HOME_JOINTS,
    )


_a750_hw = a750("arm", mock_without_address=True)

# A-750 6-DOF mock sim + keyboard teleop + Drake visualization
keyboard_teleop_a750 = autoconnect(
    KeyboardTeleopModule.blueprint(
        model_path=A750_FK_MODEL,
        ee_joint_id=6,
        home_joints=_A750_HOME_JOINTS,
        joint_names=_a750_hw.joints,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_a750_hw],
        tasks=[
            TaskConfig(
                name="cartesian_ik_arm",
                type="cartesian_ik",
                joint_names=_a750_hw.joints,
                priority=10,
                params={"model_path": A750_FK_MODEL, "ee_joint_id": 6},
            ),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_a750_model_config()],
        visualization={"backend": "meshcat"},
    ),
)
