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

from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.manipulators.mock.adapter import MockAdapter
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.utils.mesh_utils import clear_cache, prepare_urdf_for_drake
from dimos.robot.manipulators.openyam.blueprints.basic import (
    coordinator_openyam_bare,
    coordinator_openyam_gripper,
    openyam_bare_planner_coordinator,
    openyam_gripper_planner_coordinator,
)
from dimos.robot.manipulators.openyam.blueprints.teleop import (
    keyboard_teleop_openyam_bare,
    keyboard_teleop_openyam_gripper,
)
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_DOF,
    OPENYAM_FLANGE_MODEL_PATH,
    OPENYAM_MODEL_PATH,
    OPENYAM_PACKAGE_PATHS,
    make_openyam_hardware,
    make_openyam_model_config,
)
from dimos.robot.model_parser import parse_model
from dimos.utils.ament_prefix import process_xacro


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _coordinator_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return _module_kwargs(blueprint, ControlCoordinator)


def _archive_model_root(config: RobotModelConfig) -> ET.Element:
    model_path = Path(config.model_path)
    if model_path.suffix == ".xacro":
        model_xml = process_xacro(model_path, config.package_paths, {})
        return ET.fromstring(model_xml)
    return ET.parse(model_path).getroot()


def _element_attributes(element: ET.Element | None) -> dict[str, str]:
    return {} if element is None else dict(element.attrib)


def _origin_attributes(element: ET.Element | None) -> dict[str, str]:
    origin = element.find("origin") if element is not None else None
    return _element_attributes(origin)


def _arm_link_name(prefix: str, link_number: int) -> str:
    return f"{prefix}link_{link_number}" if link_number else f"{prefix}base_link"


_EXPECTED_VISUAL_RPY = {
    0: (0.0, 0.0, 0.0),
    1: (0.0, 0.0, -1.5708),
    2: (1.5708, 0.0, 3.14159),
    3: (-1.5708, 0.0, -3.14159),
    4: (-1.5708, 0.0, -3.14159),
    5: (-3.14159, 0.0, 1.5708),
    6: (0.0, -1.5708, 0.0),
}


@pytest.mark.parametrize("has_gripper", [False, True], ids=["bare", "gripper"])
def test_openyam_model_config_has_expected_links_and_mapping(has_gripper: bool) -> None:
    config = make_openyam_model_config(name="arm", has_gripper=has_gripper)
    urdf_prefix = "yam_" if has_gripper else ""

    assert config.joint_names == [f"{urdf_prefix}joint{i}" for i in range(1, OPENYAM_DOF + 1)]
    assert config.joint_name_mapping == {
        f"arm/joint{i}": f"{urdf_prefix}joint{i}" for i in range(1, OPENYAM_DOF + 1)
    }
    assert config.base_link == ("yam_base_link" if has_gripper else "base_link")
    assert config.end_effector_link == ("yam_hand_tcp" if has_gripper else "link_6")
    assert list(config.package_paths) == list(OPENYAM_PACKAGE_PATHS)
    assert config.model_path is (OPENYAM_MODEL_PATH if has_gripper else OPENYAM_FLANGE_MODEL_PATH)
    assert config.gripper_hardware_id == ("arm" if has_gripper else None)


@pytest.mark.parametrize("has_gripper", [False, True], ids=["bare", "gripper"])
def test_openyam_assets_prepare_and_parse_from_lfs_archive(has_gripper: bool) -> None:
    config = make_openyam_model_config(name="arm", has_gripper=has_gripper)

    prepared_path = prepare_urdf_for_drake(
        config.model_path,
        package_paths=config.package_paths,
    )
    prepared_content = Path(prepared_path).read_text()
    model = parse_model(prepared_path)

    assert model.root_link == config.base_link
    assert config.end_effector_link in model.links
    assert "package://yam_description" not in prepared_content
    assert [joint for joint in config.joint_names if joint in model.actuated_joint_names] == (
        config.joint_names
    )
    assert len(config.joint_names) == OPENYAM_DOF
    assert len(config.joint_name_mapping) == OPENYAM_DOF


@pytest.mark.parametrize("has_gripper", [False, True], ids=["bare", "gripper"])
def test_openyam_visual_origins_match_mujoco_without_changing_arm_metadata(
    has_gripper: bool,
) -> None:
    config = make_openyam_model_config(name="arm", has_gripper=has_gripper)
    archive_root = _archive_model_root(config)
    clear_cache()
    prepared_path = prepare_urdf_for_drake(config.model_path, package_paths=config.package_paths)
    prepared_root = ET.parse(prepared_path).getroot()
    link_prefix = "yam_" if has_gripper else ""

    for link_number, expected_rpy in _EXPECTED_VISUAL_RPY.items():
        link_name = _arm_link_name(link_prefix, link_number)
        archive_link = archive_root.find(f"./link[@name='{link_name}']")
        prepared_link = prepared_root.find(f"./link[@name='{link_name}']")
        assert archive_link is not None
        assert prepared_link is not None

        visual_origin = _origin_attributes(prepared_link.find("visual"))
        collision_origin = _origin_attributes(prepared_link.find("collision"))
        for origin in (visual_origin, collision_origin):
            assert tuple(float(value) for value in origin["rpy"].split()) == pytest.approx(
                expected_rpy
            )
        assert visual_origin["xyz"] == _origin_attributes(archive_link.find("visual"))["xyz"]
        assert collision_origin["xyz"] == _origin_attributes(archive_link.find("collision"))["xyz"]

    for joint_name in config.joint_name_mapping.values():
        archive_joint = archive_root.find(f"./joint[@name='{joint_name}']")
        prepared_joint = prepared_root.find(f"./joint[@name='{joint_name}']")
        assert archive_joint is not None
        assert prepared_joint is not None
        assert _origin_attributes(prepared_joint) == _origin_attributes(archive_joint)
        assert _element_attributes(prepared_joint.find("axis")) == _element_attributes(
            archive_joint.find("axis")
        )


@pytest.mark.parametrize("has_gripper", [False, True], ids=["bare", "gripper"])
def test_openyam_mock_hardware_has_conditional_gripper(has_gripper: bool) -> None:
    hardware = make_openyam_hardware("arm", has_gripper=has_gripper)

    assert hardware.adapter_type == "mock"
    assert hardware.joints == [f"arm/joint{i}" for i in range(1, OPENYAM_DOF + 1)]
    assert hardware.gripper_joints == (["arm/gripper"] if has_gripper else [])


def test_openyam_mock_adapter_set_get_behavior() -> None:
    positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    adapter = MockAdapter(dof=OPENYAM_DOF, initial_positions=positions)

    assert adapter.read_joint_positions() == positions
    updated_positions = [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6]
    assert adapter.write_joint_positions(updated_positions)
    assert adapter.read_joint_positions() == updated_positions
    assert adapter.write_gripper_position(0.25)
    assert adapter.read_gripper_position() == 0.25


@pytest.mark.parametrize(
    "blueprint,has_gripper",
    [
        (openyam_bare_planner_coordinator, False),
        (openyam_gripper_planner_coordinator, True),
    ],
    ids=["bare", "gripper"],
)
def test_openyam_planner_blueprints_preserve_model_config(
    blueprint: Blueprint, has_gripper: bool
) -> None:
    kwargs = _module_kwargs(blueprint, ManipulationModule)
    config = ManipulationModuleConfig(**kwargs).robots[0]

    assert config == make_openyam_model_config(name="arm", has_gripper=has_gripper)
    task = _coordinator_kwargs(blueprint)["tasks"][0]
    assert task.type == "trajectory"
    assert task.joint_names == [f"arm/joint{i}" for i in range(1, OPENYAM_DOF + 1)]


@pytest.mark.parametrize(
    "blueprint",
    [coordinator_openyam_bare, coordinator_openyam_gripper],
    ids=["bare", "gripper"],
)
def test_openyam_coordinator_blueprints_use_six_arm_joints(blueprint: Blueprint) -> None:
    kwargs = _coordinator_kwargs(blueprint)
    assert len(kwargs["hardware"]) == 1
    assert len(kwargs["hardware"][0].joints) == OPENYAM_DOF
    assert kwargs["tasks"][0].joint_names == kwargs["hardware"][0].joints


@pytest.mark.parametrize(
    "blueprint",
    [keyboard_teleop_openyam_bare, keyboard_teleop_openyam_gripper],
    ids=["bare", "gripper"],
)
def test_openyam_teleop_blueprints_construct_with_eef_twist(blueprint: Blueprint) -> None:
    task = next(
        task for task in _coordinator_kwargs(blueprint)["tasks"] if task.type == "eef_twist"
    )

    assert task.joint_names == [f"arm/joint{i}" for i in range(1, OPENYAM_DOF + 1)]
    assert task.params["ee_joint_id"] == OPENYAM_DOF
    assert _module_kwargs(blueprint, ManipulationModule)["visualization"] == {"backend": "viser"}
