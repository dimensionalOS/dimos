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


def _origin(joint: ET.Element, key: str) -> tuple[float, float, float]:
    origin = joint.find("origin")
    assert origin is not None
    x, y, z = (float(value) for value in origin.attrib[key].split())
    return (x, y, z)


def _origin_xyz(joint: ET.Element) -> tuple[float, float, float]:
    return _origin(joint, "xyz")


def test_model_package_is_versioned_and_self_contained() -> None:
    assert DUAL_OPENYAM_ARTIFACT == "dual_openyam_workcell_v2"
    assert Path(DUAL_OPENYAM_PACKAGE).name == DUAL_OPENYAM_ARTIFACT
    assert Path(DUAL_OPENYAM_MODEL_PATH).is_file()
    assert DUAL_OPENYAM_PACKAGE_PATHS == {
        "dual_openyam_workcell": DUAL_OPENYAM_PACKAGE,
    }
    for mesh in _tree().findall("./link/visual/geometry/mesh"):
        uri = mesh.attrib["filename"]
        prefix = "package://dual_openyam_workcell/"
        assert uri.startswith(prefix)
        assert (Path(DUAL_OPENYAM_PACKAGE) / uri.removeprefix(prefix)).is_file()


def test_model_has_one_geometry_free_root_and_the_calibrated_base_poses() -> None:
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
    left = _joint(root, "left_arm_fixed_joint")
    right = _joint(root, "right_arm_fixed_joint")
    assert _origin_xyz(left) == (-0.00862, 0.22219, -0.00559)
    assert _origin(left, "rpy") == (0.0, 0.0, 0.0)
    assert _origin_xyz(right) == (0.00862, -0.22219, 0.00559)
    assert _origin(right, "rpy") == pytest.approx((-0.064743, 0.029942, 0.020931))


def test_joint_three_origins_carry_the_calibrated_zero_offsets() -> None:
    # The offset lives in the joint origin, so the joint variable stays the encoder reading.
    root = _tree()
    assert _origin(_joint(root, "left_joint3"), "rpy") == pytest.approx((0.0, -0.04907, -3.14159))
    assert _origin(_joint(root, "right_joint3"), "rpy") == pytest.approx((0.0, -0.06793, -3.14159))


def test_joint_one_limits_sit_inside_the_measured_stops() -> None:
    root = _tree()
    for side in ("left", "right"):
        limit = _joint(root, f"{side}_joint1").find("limit")
        assert limit is not None
        assert float(limit.attrib["lower"]) == pytest.approx(-3.92699)
        assert float(limit.attrib["upper"]) == pytest.approx(1.5708)


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
            if element_name == "origin" and index == 3:
                assert left_element.attrib["xyz"] == right_element.attrib["xyz"]
                continue
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


def test_model_environment_is_only_the_workcell_link() -> None:
    root = _tree()
    assert len(root.findall("link")) == 22
    assert len(root.findall("joint")) == 21
    workcell = root.find("./link[@name='workcell']")
    assert workcell is not None
    assert [c.attrib["name"] for c in workcell.findall("collision")] == [
        "table",
        "camera_post",
        "overhead_camera",
    ]
    assert workcell.find("visual") is None and workcell.find("inertial") is None
    xml = ET.tostring(root, encoding="unicode").lower()
    for excluded in ("cabinet", "enclosure", "gate", "wall", "bottle", "bin"):
        assert excluded not in xml


def test_every_arm_link_has_a_convex_hull_and_each_gripper_its_camera() -> None:
    root = _tree()
    for link in root.findall("link"):
        name = link.attrib["name"]
        visual = link.find("visual/geometry/mesh")
        if visual is None:
            continue
        collision = link.find("collision/geometry/mesh")
        assert collision is not None, name
        assert collision.attrib["filename"].endswith(
            "/collision/" + visual.attrib["filename"].rsplit("/", 1)[1]
        )
    for side in ("left", "right"):
        gripper = root.find(f"./link[@name='{side}_gripper']")
        assert gripper is not None
        names = {c.attrib.get("name") for c in gripper.findall("collision")}
        assert {f"{side}_wrist_camera_bracket", f"{side}_wrist_camera"} <= names
