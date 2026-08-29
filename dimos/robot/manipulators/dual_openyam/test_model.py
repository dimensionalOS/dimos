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
import xml.etree.ElementTree as ET

import pinocchio
import pytest

from dimos.robot.manipulators.dual_openyam.model import (
    DUAL_OPENYAM_ARTIFACT,
    DUAL_OPENYAM_MODEL_PATH,
    DUAL_OPENYAM_PACKAGE,
    DUAL_OPENYAM_PACKAGE_PATHS,
)

pytestmark = pytest.mark.self_hosted


def _tree() -> ET.Element:
    return ET.parse(DUAL_OPENYAM_MODEL_PATH).getroot()


def _joint(root: ET.Element, name: str) -> ET.Element:
    result = root.find(f"./joint[@name='{name}']")
    assert result is not None
    return result


def _origin_xyz(joint: ET.Element) -> tuple[float, float, float]:
    origin = joint.find("origin")
    assert origin is not None
    return tuple(float(value) for value in origin.attrib["xyz"].split())


def test_model_package_is_versioned_and_self_contained() -> None:
    assert DUAL_OPENYAM_ARTIFACT == "dual_openyam_abc_box_v2"
    assert Path(DUAL_OPENYAM_PACKAGE).name == DUAL_OPENYAM_ARTIFACT
    assert DUAL_OPENYAM_MODEL_PATH.is_file()
    assert DUAL_OPENYAM_PACKAGE_PATHS == {
        "dual_openyam_abc_box": DUAL_OPENYAM_PACKAGE,
    }
    for mesh in _tree().findall("./link/visual/geometry/mesh"):
        uri = mesh.attrib["filename"]
        prefix = "package://dual_openyam_abc_box/"
        assert uri.startswith(prefix)
        assert (Path(DUAL_OPENYAM_PACKAGE) / uri.removeprefix(prefix)).is_file()


def test_model_has_one_geometry_free_root_and_only_preserves_arm_spacing() -> None:
    root = _tree()
    shared_base = root.find("./link[@name='dual_openyam_base']")
    assert shared_base is not None
    assert list(shared_base) == []

    link_names = {link.attrib["name"] for link in root.findall("link")}
    child_names = {
        child.attrib["link"]
        for joint in root.findall("joint")
        if (child := joint.find("child")) is not None
    }
    assert link_names - child_names == {"dual_openyam_base"}
    left = _origin_xyz(_joint(root, "left_arm_fixed_joint"))
    right = _origin_xyz(_joint(root, "right_arm_fixed_joint"))
    assert left == (0.0, 0.31, 0.0)
    assert right == (0.0, -0.31, 0.0)


def test_both_arm_chains_are_symmetric_and_use_two_rad_s_limits() -> None:
    root = _tree()
    for index in range(1, 7):
        left = _joint(root, f"left_joint{index}")
        right = _joint(root, f"right_joint{index}")
        assert left.attrib["type"] == right.attrib["type"] == "revolute"
        for element_name in ("origin", "axis", "limit"):
            left_element = left.find(element_name)
            right_element = right.find(element_name)
            assert left_element is not None and right_element is not None
            assert left_element.attrib == right_element.attrib
        limit = left.find("limit")
        assert limit is not None
        assert float(limit.attrib["velocity"]) == 2.0


def test_model_exposes_twelve_arm_joints_and_fixed_finger_geometry() -> None:
    model = pinocchio.buildModelFromUrdf(str(DUAL_OPENYAM_MODEL_PATH))
    expected = tuple(f"{side}_joint{index}" for side in ("left", "right") for index in range(1, 7))
    assert model.nq == 12
    assert model.nv == 12
    assert tuple(str(name) for name in model.names[1:]) == expected

    root = _tree()
    for side in ("left", "right"):
        for index in (7, 8):
            joint = _joint(root, f"{side}_joint{index}")
            assert joint.attrib["type"] == "fixed"
            assert joint.find("axis") is None
            assert joint.find("limit") is None
        assert _joint(root, f"{side}_grasp_frame_joint").attrib["type"] == "fixed"


def test_model_contains_no_workbench_camera_or_environment_geometry() -> None:
    root = _tree()
    assert len(root.findall("link")) == 21
    assert len(root.findall("joint")) == 20
    xml = ET.tostring(root, encoding="unicode").lower()
    for excluded in (
        "camera",
        "table",
        "cabinet",
        "enclosure",
        "gate",
        "wall",
        "bottle",
        "bin",
        "<collision",
    ):
        assert excluded not in xml
