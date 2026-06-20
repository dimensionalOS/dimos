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

from pathlib import Path

from dimos.control.blueprints._hardware import PIPER_FK_MODEL, manipulator
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule
from dimos.utils.data import LfsPath

PIPER_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("gripper_base", "link7"),
    ("gripper_base", "link8"),
    ("link7", "link8"),
    ("link6", "gripper_base"),
]

_PIPER_MODEL_PATH = LfsPath("piper_description") / "urdf/piper_description.xacro"
_PIPER_PACKAGE_PATHS: dict[str, Path] = {
    "piper_description": LfsPath("piper_description"),
    "piper_gazebo": LfsPath("piper_description"),
}


def _piper_model_config() -> RobotModelConfig:
    return RobotModelConfig(
        name="arm",
        model_path=_PIPER_MODEL_PATH,
        base_pose=PoseStamped(
            position=Vector3(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        joint_names=[f"joint{i}" for i in range(1, 7)],
        end_effector_link="gripper_base",
        base_link="base_link",
        package_paths=_PIPER_PACKAGE_PATHS,
        auto_convert_meshes=True,
        collision_exclusion_pairs=PIPER_GRIPPER_COLLISION_EXCLUSIONS,
        joint_name_mapping={f"arm/joint{i}": f"joint{i}" for i in range(1, 7)},
        coordinator_task_name="traj_arm",
        gripper_hardware_id="arm",
        home_joints=[0.0] * 6,
    )


_piper_hw = manipulator(
    "arm",
    6,
    adapter_type="piper" if global_config.can_port else "mock",
    address=global_config.can_port or "can0",
    gripper=True,
)

# Piper 6-DOF mock sim + keyboard teleop + Drake visualization
keyboard_teleop_piper = autoconnect(
    KeyboardTeleopModule.blueprint(
        model_path=PIPER_FK_MODEL,
        ee_joint_id=6,
        joint_names=_piper_hw.joints,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_piper_hw],
        tasks=[
            TaskConfig(
                name="cartesian_ik_arm",
                type="cartesian_ik",
                joint_names=_piper_hw.joints,
                priority=10,
                params={"model_path": PIPER_FK_MODEL, "ee_joint_id": 6},
            ),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_piper_model_config()],
        visualization={"backend": "meshcat"},
    ),
)

__all__ = ["keyboard_teleop_piper"]
