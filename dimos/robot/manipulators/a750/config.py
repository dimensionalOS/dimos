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

"""A-750 planning model configuration helpers."""

from __future__ import annotations

import math
from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import movable_joints
from dimos.hardware.spec import JointLimits
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.manipulators._modeling import joint_names, named_for_robot

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

A750_DOF = 6
# The gripper's joint "finger" is fully open at this many metres.
A750_GRIPPER_OPEN_M = 0.06
A750_HOME_JOINTS = [0.0, 0.0, -math.radians(90), 0.0, 0.0, 0.0]
A750_DESCRIPTION_REPO = "https://github.com/adob/a750_description"
A750_DESCRIPTION_REF = "3e4b7fe6ea0550e1f13d3dbd62f8d800ef348b14"
_A750_REPO = RobotDescriptionSource(
    url=A750_DESCRIPTION_REPO,
    ref=A750_DESCRIPTION_REF,
)
A750_MODEL_PATH = _A750_REPO / "urdf" / "a750_rev1.urdf"
A750_PACKAGE_PATHS: dict[str, Path] = {
    "a750_description": _A750_REPO / ".",
}


def make_a750_hardware(
    hw_id: str = "arm",
    *,
    adapter_type: str = "a750",
    address: str | None = None,
    gripper: bool = True,
    auto_enable: bool = True,
    home_joints: list[float] | None = None,
    mock_without_address: bool = False,
) -> HardwareComponent:
    gripper_joints = [f"{hw_id}/finger"] if gripper else []
    initial_positions = [*(home_joints or A750_HOME_JOINTS), *([0.0] * len(gripper_joints))]
    adapter_kwargs: dict[str, object] = {"initial_positions": initial_positions}
    mock_limits = JointLimits(
        position_lower=[*([-math.pi] * 6), *([0.0] * len(gripper_joints))],
        position_upper=[*([math.pi] * 6), *([0.06] * len(gripper_joints))],
        velocity_max=[*([math.pi] * 6), *([0.0] * len(gripper_joints))],
    )
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=[*joint_names(6), *gripper_joints],
        adapter_type=adapter_type,
        address=address,
        auto_enable=auto_enable,
        limits=mock_limits if adapter_type == "mock" else None,
        adapter_kwargs=adapter_kwargs,
        mock_limits=mock_limits if mock_without_address else None,
    )


def a750_hardware(
    hw_id: str = "arm", *, mock_without_address: bool = False, address: str | None = None
) -> HardwareComponent:
    """The A-750 on a serial port; with ``mock_without_address`` a mock stands in until one is configured."""
    return make_a750_hardware(
        hw_id,
        address=address if mock_without_address else address or "/dev/ttyACM0",
        mock_without_address=mock_without_address,
    )


def a750_model(*, robot: str | None = None) -> RobotModel:
    """The A-750's URDF with its gripper, the joint ``finger`` (0 to 0.06 m open).

    Args:
        robot: Name the moving joints under this robot, as the coordinator
            wants them, e.g. "arm" gives "arm/joint1". Links keep their names.
    """
    model = (
        RobotModel.from_file(A750_MODEL_PATH, package_paths=A750_PACKAGE_PATHS)
        .with_joint_position_limits("finger", lower=0.0, upper=A750_GRIPPER_OPEN_M)
        .with_joint_position_limits("finger_mimic", lower=0.0, upper=A750_GRIPPER_OPEN_M)
    )
    if not robot:
        return model
    moving = [*joint_names(A750_DOF), "finger"]
    return model.with_renamed_joints({joint: f"{robot}/{joint}" for joint in moving})


def a750_coordinator_connection(source: str = "arm") -> ConnectionConfig:
    """What a coordinator needs to know about the A-750 connection named ``source``:
    its model, with every joint (the finger too) taking positions."""
    model = a750_model(robot=source)
    return ConnectionConfig(
        source=source,
        model=model,
        joint_commands={joint.name: (Interface.POSITION,) for joint in movable_joints(model)},
    )


def make_a750_model_config(robot: str | None = None) -> RobotModelConfig:
    """The A-750's planning model; ``robot`` names its joints under that robot, e.g. "arm/joint1"."""
    model_joint_names = joint_names(A750_DOF)
    config = RobotModelConfig(
        model=a750_model().with_default_joint_acceleration_limit(2.0),
        joint_names=model_joint_names,
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(model_joint_names),
                base_link="base_link",
                tip_link="gripper_base",
            )
        ],
        auto_convert_meshes=True,
        collision_exclusion_pairs=A750_GRIPPER_COLLISION_EXCLUSIONS,
        home_joints=A750_HOME_JOINTS,
    )
    return named_for_robot(config, robot, fingers=("finger",))
