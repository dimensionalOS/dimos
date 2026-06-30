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

"""Tests for planning group discovery."""

from __future__ import annotations

from pathlib import Path

import pytest

from dimos.manipulation.planning.groups.discovery import (
    FALLBACK_PLANNING_GROUP_NAME,
    PlanningGroupDiscoveryError,
    discover_planning_group_definitions,
    generate_fallback_planning_group,
    parse_srdf_planning_groups,
)
from dimos.robot.model_parser import JointDescription, ModelDescription


def _serial_model(*joint_types: str) -> ModelDescription:
    joints = [
        JointDescription(
            name=f"joint{i + 1}",
            type=joint_type,
            parent_link=f"link{i}",
            child_link=f"link{i + 1}",
        )
        for i, joint_type in enumerate(joint_types)
    ]
    return ModelDescription(
        joints=joints,
        root_link="link0",
        links=[f"link{i}" for i in range(len(joint_types) + 1)],
    )


def _branching_model() -> ModelDescription:
    return ModelDescription(
        joints=[
            JointDescription(
                name="left_joint",
                type="revolute",
                parent_link="base",
                child_link="left_link",
            ),
            JointDescription(
                name="right_joint",
                type="revolute",
                parent_link="base",
                child_link="right_link",
            ),
        ],
        root_link="base",
        links=["base", "left_link", "right_link"],
    )


def _write_srdf(tmp_path: Path, body: str) -> Path:
    srdf_path = tmp_path / "robot.srdf"
    srdf_path.write_text(f"<robot name='test'>{body}</robot>")
    return srdf_path


def test_parse_srdf_chain_group(tmp_path: Path) -> None:
    model = _serial_model("revolute", "revolute", "revolute")
    srdf_path = _write_srdf(
        tmp_path,
        "<group name='arm'><chain base_link='link0' tip_link='link3'/></group>",
    )

    groups = parse_srdf_planning_groups(
        srdf_path,
        model=model,
        controllable_joint_names=["joint1", "joint2", "joint3"],
    )

    assert len(groups) == 1
    assert groups[0].name == "arm"
    assert groups[0].joint_names == ("joint1", "joint2", "joint3")
    assert groups[0].base_link == "link0"
    assert groups[0].tip_link == "link3"
    assert groups[0].source == "srdf"


def test_parse_srdf_ordered_joint_list_group(tmp_path: Path) -> None:
    model = _serial_model("revolute", "prismatic", "revolute")
    srdf_path = _write_srdf(
        tmp_path,
        """
        <group name='arm'>
          <joint name='joint1'/>
          <joint name='joint2'/>
          <joint name='joint3'/>
        </group>
        """,
    )

    groups = parse_srdf_planning_groups(
        srdf_path,
        model=model,
        controllable_joint_names=["joint1", "joint2", "joint3"],
    )

    assert len(groups) == 1
    assert groups[0].joint_names == ("joint1", "joint2", "joint3")
    assert groups[0].base_link == "link0"
    assert groups[0].tip_link == "link3"


def test_parse_srdf_skips_unsupported_groups_and_ignores_end_effector(
    tmp_path: Path,
) -> None:
    model = _serial_model("revolute", "revolute")
    srdf_path = _write_srdf(
        tmp_path,
        """
        <group name='links'><link name='link1'/></group>
        <group name='nested'><group name='other'/></group>
        <group name='arm'><chain base_link='link0' tip_link='link2'/></group>
        <end_effector name='tool' group='gripper' parent_link='link2'/>
        """,
    )

    with pytest.warns(UserWarning) as warnings:
        groups = parse_srdf_planning_groups(
            srdf_path,
            model=model,
            controllable_joint_names=["joint1", "joint2"],
        )

    assert [group.name for group in groups] == ["arm"]
    warning_text = "\n".join(str(warning.message) for warning in warnings)
    assert "Skipping unsupported SRDF planning group links" in warning_text
    assert "Skipping unsupported SRDF planning group nested" in warning_text


def test_fallback_generates_manipulator_for_unambiguous_serial_chain() -> None:
    model = _serial_model("revolute", "prismatic", "revolute")

    group = generate_fallback_planning_group(
        model=model,
        controllable_joint_names=["joint2", "joint1", "joint3"],
    )

    assert group.name == FALLBACK_PLANNING_GROUP_NAME
    assert group.joint_names == ("joint1", "joint2", "joint3")
    assert group.base_link == "link0"
    assert group.tip_link == "link3"
    assert group.source == "fallback"


def test_fallback_strips_terminal_prismatic_joints() -> None:
    model = _serial_model("revolute", "revolute", "prismatic")

    group = generate_fallback_planning_group(
        model=model,
        controllable_joint_names=["joint1", "joint2", "joint3"],
    )

    assert group.joint_names == ("joint1", "joint2")
    assert group.tip_link == "link2"


def test_fallback_rejects_branching_model() -> None:
    with pytest.raises(PlanningGroupDiscoveryError, match="branch"):
        generate_fallback_planning_group(
            model=_branching_model(),
            controllable_joint_names=["left_joint", "right_joint"],
        )


def test_discovery_prefers_explicit_srdf_over_fallback(tmp_path: Path) -> None:
    model = _serial_model("revolute", "revolute")
    model_path = tmp_path / "robot.urdf"
    model_path.write_text("<robot name='test'/>")
    srdf_path = _write_srdf(
        tmp_path,
        "<group name='srdf_arm'><chain base_link='link0' tip_link='link2'/></group>",
    )

    groups = discover_planning_group_definitions(
        robot_name="robot",
        model_path=model_path,
        model=model,
        controllable_joint_names=["joint1", "joint2"],
        srdf_path=srdf_path,
    )

    assert [group.name for group in groups] == ["srdf_arm"]


def test_discovery_auto_discovers_srdf_with_warning(
    tmp_path: Path,
) -> None:
    model = _serial_model("revolute")
    model_path = tmp_path / "robot.urdf"
    model_path.write_text("<robot name='test'/>")
    _write_srdf(
        tmp_path,
        "<group name='auto_arm'><chain base_link='link0' tip_link='link1'/></group>",
    )

    with pytest.warns(UserWarning, match="Auto-discovered SRDF"):
        groups = discover_planning_group_definitions(
            robot_name="robot",
            model_path=model_path,
            model=model,
            controllable_joint_names=["joint1"],
        )

    assert [group.name for group in groups] == ["auto_arm"]
