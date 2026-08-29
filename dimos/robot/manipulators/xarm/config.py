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

"""xArm planning model configuration helpers."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
)
from dimos.core.global_config import global_config
from dimos.hardware.spec import JointLimits
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.manipulators._modeling import joint_names
from dimos.utils.data import LfsPath

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

XARM_ROS2_REPO = "https://github.com/xArm-Developer/xarm_ros2"
XARM_ROS2_REF = "5bb832f72ca665f1236a9d8ed1c3a82f308db489"
_XARM_REPO = RobotDescriptionSource(url=XARM_ROS2_REPO, ref=XARM_ROS2_REF)
XARM_MODEL_PATH = _XARM_REPO / "xarm_description" / "urdf" / "xarm_device.urdf.xacro"
XARM_DUAL_MODEL_PATH = _XARM_REPO / "xarm_description" / "urdf" / "dual_xarm_device.urdf.xacro"
XARM_PACKAGE_PATHS: dict[str, Path] = {"xarm_description": _XARM_REPO / "xarm_description"}
XARM6_SIM_PATH = LfsPath("xarm6/scene.xml")
XARM7_SIM_PATH = LfsPath("xarm7/scene.xml")
XARM7_SIM_HOME = [0.0, -0.247, 0.0, 0.909, 0.0, 1.15644, 0.0]
# The sim scene stands the arm on a pedestal: xarm7.xml mounts link_base at
# z=0.12. Place the planning model to match, or the planner solves poses 12cm
# below the arm it is driving and every grasp closes on air.
XARM7_SIM_BASE_POSE = PoseStamped(frame_id="world", position=Vector3(z=0.12))


def make_xarm7_sim_robot_config() -> RobotModelConfig:
    return make_xarm7_model_config(
        add_gripper=True,
        gripper_hardware_id="arm",
        base_pose=XARM7_SIM_BASE_POSE,
        tf_extra_links=["link7"],
        home_joints=XARM7_SIM_HOME,
        pre_grasp_offset=0.05,
    )


def make_dual_xarm6_model_config() -> RobotModelConfig:
    """Return one statically authored model containing two canonical xArm6 chains."""
    left_joints = joint_names(6, prefix="left/joint")
    right_joints = joint_names(6, prefix="right/joint")
    canonical_joints = [*left_joints, *right_joints]
    return RobotModelConfig(
        model=RobotModel.from_file(
            XARM_DUAL_MODEL_PATH,
            package_paths=XARM_PACKAGE_PATHS,
            xacro_args={
                "prefix_1": "left/",
                "prefix_2": "right/",
                "dof_1": "6",
                "dof_2": "6",
                "limited": "true",
                "add_gripper_1": "true",
                "add_gripper_2": "true",
            },
        ),
        joint_names=canonical_joints,
        base_link="world",
        planning_groups=[
            PlanningGroupDefinition(
                name="left_arm",
                joint_names=tuple(left_joints),
                base_link="left/link_base",
                tip_link="left/link_tcp",
            ),
            PlanningGroupDefinition(
                name="right_arm",
                joint_names=tuple(right_joints),
                base_link="right/link_base",
                tip_link="right/link_tcp",
            ),
            PlanningGroupDefinition(
                name="both_arms",
                joint_names=tuple(canonical_joints),
                base_link="world",
            ),
        ],
        auto_convert_meshes=True,
        collision_exclusion_pairs=[
            (f"{prefix}{left}", f"{prefix}{right}")
            for prefix in ("left/", "right/")
            for left, right in XARM_GRIPPER_COLLISION_EXCLUSIONS
        ],
        home_joints=[0.0] * 12,
    )


def make_xarm7_sim_hardware(address: str | Path) -> HardwareComponent:
    return make_xarm_hardware(
        "arm",
        7,
        adapter_type="sim_mujoco",
        address=address,
        gripper=True,
        home_joints=XARM7_SIM_HOME,
    )


def make_xarm7_sim_module_kwargs(address: str | Path) -> dict[str, Any]:
    return {
        "address": address,
        "headless": False,
        "dof": 7,
        "camera_name": "wrist_camera",
        "base_frame_id": "link7",
        "reset_joint_positions": XARM7_SIM_HOME,
    }


def _adapter_kwargs(home_joints: list[float] | None = None) -> dict[str, object]:
    if home_joints is None:
        return {}
    return {"initial_positions": home_joints}


def make_xarm_hardware(
    hw_id: str,
    dof: int,
    *,
    adapter_type: str = "mock",
    address: str | Path | None = None,
    gripper: bool = False,
    auto_enable: bool = True,
    adapter_kwargs: dict[str, object] | None = None,
    home_joints: list[float] | None = None,
    canonical_joint_names: list[str] | None = None,
) -> HardwareComponent:
    kwargs = _adapter_kwargs(home_joints)
    if adapter_type == "xarm":
        kwargs["arm_dof"] = dof
    if adapter_kwargs:
        kwargs.update(adapter_kwargs)
    gripper_joints = [f"{hw_id}/gripper"] if gripper else []
    initial_positions = kwargs.get("initial_positions")
    if gripper and isinstance(initial_positions, list):
        kwargs["initial_positions"] = [*initial_positions, 0.0]
    limits: JointLimits | None = None
    if adapter_type == "mock":
        limits = JointLimits(
            position_lower=[*([-2 * math.pi] * dof), *([0.0] * len(gripper_joints))],
            position_upper=[*([2 * math.pi] * dof), *([850.0] * len(gripper_joints))],
            velocity_max=[*([math.pi] * dof), *([0.0] * len(gripper_joints))],
        )
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=[*(canonical_joint_names or joint_names(dof)), *gripper_joints],
        adapter_type=adapter_type,
        address=address,
        auto_enable=auto_enable,
        limits=limits,
        adapter_kwargs=kwargs,
    )


def xarm7_hardware(
    hw_id: str = "arm",
    *,
    gripper: bool = False,
    mock_without_address: bool = False,
    home_joints: list[float] | None = None,
    canonical_joint_names: list[str] | None = None,
) -> HardwareComponent:
    if global_config.simulation:
        return make_xarm_hardware(
            hw_id,
            7,
            adapter_type="sim_mujoco",
            address=str(XARM7_SIM_PATH),
            gripper=gripper,
            home_joints=home_joints,
            canonical_joint_names=canonical_joint_names,
        )
    address = global_config.xarm7_ip
    if mock_without_address and not address:
        return make_xarm_hardware(
            hw_id,
            7,
            gripper=gripper,
            home_joints=home_joints,
            canonical_joint_names=canonical_joint_names,
        )
    return make_xarm_hardware(
        hw_id,
        7,
        adapter_type="xarm",
        address=address,
        gripper=gripper,
        home_joints=home_joints,
        canonical_joint_names=canonical_joint_names,
    )


def xarm6_hardware(
    hw_id: str = "arm",
    *,
    gripper: bool = False,
    mock_without_address: bool = False,
    home_joints: list[float] | None = None,
    canonical_joint_names: list[str] | None = None,
) -> HardwareComponent:
    if global_config.simulation:
        return make_xarm_hardware(
            hw_id,
            6,
            adapter_type="sim_mujoco",
            address=str(XARM6_SIM_PATH),
            gripper=gripper,
            home_joints=home_joints,
            canonical_joint_names=canonical_joint_names,
        )
    address = global_config.xarm6_ip
    if mock_without_address and not address:
        return make_xarm_hardware(
            hw_id,
            6,
            gripper=gripper,
            home_joints=home_joints,
            canonical_joint_names=canonical_joint_names,
        )
    return make_xarm_hardware(
        hw_id,
        6,
        adapter_type="xarm",
        address=address,
        gripper=gripper,
        home_joints=home_joints,
        canonical_joint_names=canonical_joint_names,
    )


def make_xarm_model_config(
    dof: int,
    *,
    prefix: str = "",
    add_gripper: bool = True,
    gripper_hardware_id: str | None = None,
    base_pose: PoseStamped | None = None,
    tf_extra_links: list[str] | None = None,
    home_joints: list[float] | None = None,
    pre_grasp_offset: float = 0.10,
) -> RobotModelConfig:
    xacro_args = {
        "dof": str(dof),
        "prefix": prefix,
        "limited": "true",
        "attach_xyz": "0 0 0",
        "attach_rpy": "0 0 0",
    }
    if add_gripper:
        xacro_args["add_gripper"] = "true"

    model_joint_names = joint_names(dof, prefix=f"{prefix}joint")
    tip_link = f"{prefix}link_tcp" if add_gripper else f"{prefix}link{dof}"
    collision_exclusions = [
        (f"{prefix}{left}", f"{prefix}{right}") for left, right in XARM_GRIPPER_COLLISION_EXCLUSIONS
    ]
    return RobotModelConfig(
        model=RobotModel.from_file(
            XARM_MODEL_PATH,
            package_paths=XARM_PACKAGE_PATHS,
            xacro_args=xacro_args,
        ),
        base_pose=base_pose if base_pose is not None else PoseStamped(),
        joint_names=model_joint_names,
        base_link=f"{prefix}link_base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(model_joint_names),
                base_link=f"{prefix}link_base",
                tip_link=tip_link,
            )
        ],
        auto_convert_meshes=True,
        collision_exclusion_pairs=collision_exclusions if add_gripper else [],
        gripper_hardware_id=gripper_hardware_id,
        tf_extra_links=[f"{prefix}{link}" for link in (tf_extra_links or [])],
        home_joints=home_joints or [0.0] * dof,
        pre_grasp_offset=pre_grasp_offset,
    )


def make_xarm6_model_config(
    **kwargs: Any,
) -> RobotModelConfig:
    return make_xarm_model_config(6, **kwargs)


def make_xarm7_model_config(
    **kwargs: Any,
) -> RobotModelConfig:
    return make_xarm_model_config(7, **kwargs)
