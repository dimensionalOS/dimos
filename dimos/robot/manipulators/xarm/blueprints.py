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

from pathlib import Path

from dimos.control.blueprints._hardware import XARM6_FK_MODEL, XARM7_FK_MODEL, manipulator
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

_XARM_MODEL_PATH = LfsPath("xarm_description") / "urdf/xarm_device.urdf.xacro"
_XARM_PACKAGE_PATHS: dict[str, Path] = {"xarm_description": LfsPath("xarm_description")}

XARM_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("right_inner_knuckle", "right_outer_knuckle"),
    ("left_inner_knuckle", "left_outer_knuckle"),
    ("right_inner_knuckle", "right_finger"),
    ("left_inner_knuckle", "left_finger"),
    ("left_finger", "right_finger"),
    ("left_outer_knuckle", "right_outer_knuckle"),
    ("left_inner_knuckle", "right_inner_knuckle"),
    ("left_outer_knuckle", "right_finger"),
    ("right_outer_knuckle", "left_finger"),
    ("xarm_gripper_base_link", "left_inner_knuckle"),
    ("xarm_gripper_base_link", "right_inner_knuckle"),
    ("xarm_gripper_base_link", "left_finger"),
    ("xarm_gripper_base_link", "right_finger"),
    ("link6", "xarm_gripper_base_link"),
    ("link6", "left_outer_knuckle"),
    ("link6", "right_outer_knuckle"),
]


def _xarm_model_config(dof: int, *, add_gripper: bool = False) -> RobotModelConfig:
    xacro_args = {
        "dof": str(dof),
        "limited": "true",
        "attach_xyz": "0.0 0.0 0.0",
        "attach_rpy": "0 0.0 0",
    }
    if add_gripper:
        xacro_args["add_gripper"] = "true"

    return RobotModelConfig(
        name="arm",
        model_path=_XARM_MODEL_PATH,
        base_pose=PoseStamped(
            position=Vector3(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        joint_names=[f"joint{i}" for i in range(1, dof + 1)],
        end_effector_link="link_tcp" if add_gripper else f"link{dof}",
        base_link="link_base",
        package_paths=_XARM_PACKAGE_PATHS,
        xacro_args=xacro_args,
        auto_convert_meshes=True,
        collision_exclusion_pairs=(XARM_GRIPPER_COLLISION_EXCLUSIONS if add_gripper else []),
        joint_name_mapping={f"arm/joint{i}": f"joint{i}" for i in range(1, dof + 1)},
        coordinator_task_name="traj_arm",
        gripper_hardware_id="arm" if add_gripper else None,
        home_joints=[0.0] * dof,
    )


_xarm6_hw = manipulator(
    "arm",
    6,
    adapter_type="xarm" if global_config.xarm6_ip else "mock",
    address=global_config.xarm6_ip,
)
_xarm7_hw = manipulator(
    "arm",
    7,
    adapter_type="xarm" if global_config.xarm7_ip else "mock",
    address=global_config.xarm7_ip,
)

# XArm6 mock sim + keyboard teleop + Drake visualization
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
        tasks=[
            TaskConfig(
                name="cartesian_ik_arm",
                type="cartesian_ik",
                joint_names=_xarm6_hw.joints,
                priority=10,
                params={"model_path": XARM6_FK_MODEL, "ee_joint_id": 6},
            ),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_xarm_model_config(6)],
        visualization={"backend": "meshcat"},
    ),
)

# XArm7 mock sim + keyboard teleop + Drake visualization
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
        tasks=[
            TaskConfig(
                name="cartesian_ik_arm",
                type="cartesian_ik",
                joint_names=_xarm7_hw.joints,
                priority=10,
                params={"model_path": XARM7_FK_MODEL, "ee_joint_id": 7},
            ),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_xarm_model_config(7)],
        visualization={"backend": "meshcat"},
    ),
)

__all__ = ["keyboard_teleop_xarm6", "keyboard_teleop_xarm7"]
