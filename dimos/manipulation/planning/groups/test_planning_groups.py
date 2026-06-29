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

"""Tests for planning groups."""

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
from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.groups.utils import joint_target_to_global_names
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
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


def _make_group() -> PlanningGroup:
    return PlanningGroup(
        id="left/arm",
        robot_name="left",
        group_name="arm",
        joint_names=("left/j1", "left/j2", "left/j3"),
        local_joint_names=("j1", "j2", "j3"),
        base_link="base",
        tip_link="ee",
    )


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

    groups = parse_srdf_planning_groups(
        srdf_path,
        model=model,
        controllable_joint_names=["joint1", "joint2"],
    )

    assert [group.name for group in groups] == ["arm"]


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


def test_discovery_auto_discovers_srdf(tmp_path: Path) -> None:
    model = _serial_model("revolute")
    model_path = tmp_path / "robot.urdf"
    model_path.write_text("<robot name='test'/>")
    _write_srdf(
        tmp_path,
        "<group name='auto_arm'><chain base_link='link0' tip_link='link1'/></group>",
    )

    groups = discover_planning_group_definitions(
        robot_name="robot",
        model_path=model_path,
        model=model,
        controllable_joint_names=["joint1"],
    )

    assert [group.name for group in groups] == ["auto_arm"]


def test_primary_pose_group_id_for_robot_raises_when_ambiguous() -> None:
    registry = PlanningGroupRegistry(
        [
            RobotModelConfig(
                name="robot",
                model_path=Path("/tmp/robot.urdf"),
                base_pose=PoseStamped(),
                joint_names=["joint1", "joint2"],
                planning_groups=[
                    PlanningGroupDefinition(
                        name="left",
                        joint_names=("joint1",),
                        base_link="base",
                        tip_link="left_tool",
                    ),
                    PlanningGroupDefinition(
                        name="right",
                        joint_names=("joint2",),
                        base_link="base",
                        tip_link="right_tool",
                    ),
                ],
            )
        ]
    )

    with pytest.raises(ValueError, match="multiple|2 pose-targetable|explicit planning group"):
        registry.primary_pose_group_id_for_robot("robot")


def test_joint_target_to_global_names_accepts_named_global_targets_in_group_order() -> None:
    group = _make_group()
    target = JointState({"name": ["left/j3", "left/j1", "left/j2"], "position": [3.0, 1.0, 2.0]})

    normalized = joint_target_to_global_names(group, target)

    assert normalized.name == ["left/j1", "left/j2", "left/j3"]
    assert normalized.position == [1.0, 2.0, 3.0]


def test_joint_target_to_global_names_accepts_named_local_targets_in_group_order() -> None:
    group = _make_group()
    target = JointState({"name": ["j2", "j3", "j1"], "position": [2.0, 3.0, 1.0]})

    normalized = joint_target_to_global_names(group, target)

    assert normalized.name == ["left/j1", "left/j2", "left/j3"]
    assert normalized.position == [1.0, 2.0, 3.0]


def test_joint_target_to_global_names_rejects_mixed_global_and_local_target_names() -> None:
    group = _make_group()
    target = JointState({"name": ["left/j1", "j2", "left/j3"], "position": [1.0, 2.0, 3.0]})

    with pytest.raises(ValueError, match="mixes global and local joint names"):
        joint_target_to_global_names(group, target)


def test_robot_model_config_derives_legacy_end_effector_link_from_pose_group() -> None:
    config = RobotModelConfig(
        name="arm",
        model_path=Path("robot.urdf"),
        joint_names=["j1", "j2"],
        joint_name_mapping={"hw_j1": "j1", "hw_j2": "j2"},
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("j1", "j2"),
                base_link="base",
                tip_link="tool",
            )
        ],
    )

    assert config.end_effector_link == "tool"
    assert config.get_urdf_joint_name("hw_j1") == "j1"
    assert config.get_coordinator_joint_name("j2") == "hw_j2"
    assert config.get_coordinator_joint_names() == ["hw_j1", "hw_j2"]


def test_robot_model_config_end_effector_link_requires_pose_group() -> None:
    config = RobotModelConfig(
        name="arm",
        model_path=Path("robot.urdf"),
        joint_names=["j1"],
        planning_groups=[
            PlanningGroupDefinition(
                name="joint_only",
                joint_names=("j1",),
                base_link="base",
            )
        ],
    )

    with pytest.raises(ValueError, match="no pose-target planning group"):
        _ = config.end_effector_link


def test_robot_model_config_end_effector_link_rejects_ambiguous_pose_groups() -> None:
    config = RobotModelConfig(
        name="arm",
        model_path=Path("robot.urdf"),
        joint_names=["j1", "j2"],
        planning_groups=[
            PlanningGroupDefinition(
                name="left",
                joint_names=("j1",),
                base_link="base",
                tip_link="left_tool",
            ),
            PlanningGroupDefinition(
                name="right",
                joint_names=("j2",),
                base_link="base",
                tip_link="right_tool",
            ),
        ],
    )

    with pytest.raises(ValueError, match="multiple pose-target planning groups"):
        _ = config.end_effector_link
