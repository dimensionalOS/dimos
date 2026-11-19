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

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys
from types import ModuleType
from typing import Any
import xml.etree.ElementTree as ET

import pytest

from dimos.manipulation.planning.spec.validation import PreparedRobotModel, prepare_robot_model
from dimos.manipulation.planning.world.collision_exclusions import collision_exclusion_pairs
from dimos.manipulation.planning.world.curobo_model import build_curobo_model
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.manipulators.xarm.config import (
    make_xarm7_model_config,
    make_xarm7_sim_robot_config,
)


@pytest.fixture(params=["hardware-names", "sim-names"], scope="module")
def xarm(request: pytest.FixtureRequest) -> PreparedRobotModel:
    base = PoseStamped(frame_id="world")
    if request.param == "hardware-names":
        config = make_xarm7_model_config(robot="arm", tf_extra_links=["link7"], base_pose=base)
    else:
        config = make_xarm7_sim_robot_config(base_pose=base, tf_extra_links=[])
    return prepare_robot_model(config)


def test_collision_exclusions_merge_and_validate(xarm: PreparedRobotModel, tmp_path: Path) -> None:
    srdf = tmp_path / "exclusions.srdf"
    srdf.write_text(
        '<robot name="robot"><disable_collisions link1="c" link2="a" />'
        '<disable_collisions link1="d" link2="a" />'
        '<disable_collisions link1="deleted_a" link2="deleted_b" /></robot>'
    )
    config = xarm.config.model_copy(
        update={"collision_exclusion_pairs": [("b", "a"), ("a", "c")], "srdf_path": srdf}
    )
    assert collision_exclusion_pairs(config, {"a", "b", "c", "d"}, [("a", "b"), ("b", "c")]) == (
        ("a", "b"),
        ("a", "c"),
        ("a", "d"),
        ("b", "c"),
    )
    with pytest.raises(ValueError, match="unknown links: d <-> a"):
        collision_exclusion_pairs(config, {"a", "b", "c"}, [("a", "b")])


def test_build_curobo_grasp_model(
    xarm: PreparedRobotModel, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = ET.fromstring(xarm.description.xml)
    expected_links = {
        link.attrib["name"] for link in root.findall("link") if link.find("collision") is not None
    }
    calls: list[Path] = []
    omit: set[str] = set()

    class FakeBuilder:
        def __init__(self, urdf_path: str, tool_frames: list[str]) -> None:
            assert tool_frames == ["link_tcp"]
            self.path = Path(urdf_path)

        def fit_collision_spheres(self, *, use_collision_mesh: bool) -> dict[str, Any]:
            assert use_collision_mesh
            calls.append(self.path)
            fitting = ET.parse(self.path).getroot()
            assert {
                link.attrib["name"]
                for link in fitting.findall("link")
                if link.find("visual") is not None
            } == expected_links
            return {
                link: [{"center": [0.0, 0.0, 0.0], "radius": 0.01}]
                for link in sorted(expected_links - omit)
            }

    builder_module = ModuleType("curobo.robot_builder")
    monkeypatch.setattr(builder_module, "RobotBuilder", FakeBuilder, raising=False)
    monkeypatch.setitem(sys.modules, "curobo.robot_builder", builder_module)
    kinematics = build_curobo_model(xarm)["kinematics"]
    assert len(calls) == 1 and not calls[0].exists()
    assert kinematics["base_link"] == "link_base"
    assert kinematics["tool_frames"] == ["link_tcp"]
    assert set(kinematics["collision_link_names"]) == expected_links
    assert kinematics["cspace"]["joint_names"] == list(xarm.joint_space.names)
    assert kinematics["cspace"]["max_acceleration"] == list(xarm.joint_space.acceleration_limits)
    assert xarm.config.gripper is not None
    driver = next(iter(xarm.config.gripper.finger_joints))
    assert kinematics["lock_joints"] == {driver: 0.0}

    urdf = ET.parse(kinematics["urdf_path"]).getroot()
    assert urdf.find("link[@name='world']") is None
    assert [m.attrib["joint"] for m in urdf.findall("joint/mimic")] == [driver] * 5
    assert all(
        Path(m.attrib["filename"]).is_file() for m in urdf.findall("link/collision/geometry/mesh")
    )
    expected_ignores = (
        {
            frozenset(pair)
            for pair in xarm.config.collision_exclusion_pairs
            if set(pair) <= expected_links
        }
        | {
            frozenset((joint.parent_link, joint.child_link))
            for joint in xarm.description.joints
            if {joint.parent_link, joint.child_link} <= expected_links
        }
        | {frozenset(("link7", "xarm_gripper_base_link"))}
    )
    assert {
        frozenset((first, second))
        for first, seconds in kinematics["self_collision_ignore"].items()
        for second in seconds
    } == expected_ignores

    omit.add("right_finger")
    with pytest.raises(ValueError, match="coverage mismatch.*right_finger"):
        build_curobo_model(xarm)
    assert not calls[-1].parent.exists()


def test_reject_tip_off_arm_chain_before_import(xarm: PreparedRobotModel) -> None:
    group = replace(xarm.planning_groups[0], tip_link="left_finger")
    with pytest.raises(ValueError, match="TCP chain"):
        build_curobo_model(replace(xarm, planning_groups=(group,)))
