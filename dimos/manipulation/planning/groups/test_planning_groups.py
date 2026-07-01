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

from dimos.manipulation.planning.groups import utils as planning_group_utils
from dimos.manipulation.planning.groups.discovery import (
    FALLBACK_PLANNING_GROUP_NAME,
    PlanningGroupDiscoveryError,
    discover_planning_group_definitions,
    generate_fallback_planning_group,
    parse_srdf_planning_groups,
)
from dimos.manipulation.planning.groups.identifiers import local_joint_name_from_global
from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.groups.utils import (
    filter_joint_state_to_selected_joints,
    joint_target_to_global_names,
    matching_global_joint_name,
    planning_group_id_from_selector,
)
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


def _robot_config(
    name: str = "robot",
    planning_groups: list[PlanningGroupDefinition] | None = None,
) -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model_path=Path("/tmp/robot.urdf"),
        base_pose=PoseStamped(),
        joint_names=["joint1", "joint2", "joint3"],
        planning_groups=planning_groups
        if planning_groups is not None
        else [
            PlanningGroupDefinition(
                name=FALLBACK_PLANNING_GROUP_NAME,
                joint_names=("joint1", "joint2"),
                base_link="base",
                tip_link="tool",
            )
        ],
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
    assert group.source == "fallback"


def test_fallback_strips_terminal_prismatic_joints() -> None:
    model = _serial_model("revolute", "revolute", "prismatic")

    group = generate_fallback_planning_group(
        model=model,
        controllable_joint_names=["joint1", "joint2", "joint3"],
    )

    assert group.joint_names == ("joint1", "joint2")
    assert group.tip_link == "link2"
    assert group.source == "fallback"


def test_fallback_rejects_branching_model() -> None:
    with pytest.raises(PlanningGroupDiscoveryError, match="branch"):
        generate_fallback_planning_group(
            model=_branching_model(),
            controllable_joint_names=["left_joint", "right_joint"],
        )


def test_fallback_rejects_all_terminal_prismatic_candidates() -> None:
    with pytest.raises(PlanningGroupDiscoveryError, match="removed all candidate joints"):
        generate_fallback_planning_group(
            model=_serial_model("prismatic", "prismatic"),
            controllable_joint_names=["joint1", "joint2"],
        )


def test_parse_srdf_skips_invalid_groups_and_keeps_valid_group(tmp_path: Path) -> None:
    model = _serial_model("revolute", "revolute")
    srdf_path = _write_srdf(
        tmp_path,
        """
        <group name='missing_tip'><chain base_link='link0'/></group>
        <group><chain base_link='link0' tip_link='link2'/></group>
        <group name='unknown_joint'><joint name='joint1'/><joint name='missing'/></group>
        <group name='arm'><joint name='joint1'/><joint name='joint2'/></group>
        """,
    )

    groups = parse_srdf_planning_groups(
        srdf_path,
        model=model,
        controllable_joint_names=["joint1", "joint2"],
    )

    assert [group.name for group in groups] == ["arm"]


def test_discovery_rejects_missing_explicit_srdf(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match="SRDF file not found"):
        discover_planning_group_definitions(
            robot_name="robot",
            model_path=tmp_path / "robot.urdf",
            model=_serial_model("revolute"),
            controllable_joint_names=["joint1"],
            srdf_path=tmp_path / "missing.srdf",
        )


def test_discovery_falls_back_when_srdf_has_no_supported_groups(tmp_path: Path) -> None:
    model_path = tmp_path / "robot.urdf.xacro"
    model_path.write_text("<robot name='test'/>")
    (tmp_path / "robot.srdf").write_text(
        "<robot name='test'><group name='links'><link name='link1'/></group></robot>"
    )

    groups = discover_planning_group_definitions(
        robot_name="robot",
        model_path=model_path,
        model=_serial_model("revolute"),
        controllable_joint_names=["joint1"],
    )

    assert [group.name for group in groups] == [FALLBACK_PLANNING_GROUP_NAME]
    assert [group.source for group in groups] == ["fallback"]


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


def test_registry_preserves_order_and_exposes_defaults() -> None:
    registry = PlanningGroupRegistry([_robot_config("left"), _robot_config("right")])

    assert [group.id for group in registry.list()] == ["left/manipulator", "right/manipulator"]
    assert registry.default_group_id_for_robot("left") == "left/manipulator"
    assert registry.primary_pose_group_id_for_robot("right") == "right/manipulator"
    assert registry.get("left/manipulator").source == "srdf"
    assert registry.groups_for_robot("missing") == ()
    assert registry.default_group_id_for_robot("missing") is None


def test_registry_rejects_duplicate_robot_and_unknown_group() -> None:
    registry = PlanningGroupRegistry([_robot_config()])

    with pytest.raises(ValueError, match="already registered"):
        registry.add_robot(_robot_config())
    with pytest.raises(KeyError, match="Unknown planning group ID"):
        registry.get("robot/missing")


def test_selection_preserves_group_order_and_rejects_overlapping_joints() -> None:
    registry = PlanningGroupRegistry(
        [
            _robot_config(
                planning_groups=[
                    PlanningGroupDefinition("arm", ("joint1", "joint2"), "base", "tool"),
                    PlanningGroupDefinition("gripper", ("joint3",), "tool"),
                ]
            )
        ]
    )

    selection = registry.select(["robot/gripper", "robot/arm"])

    assert selection.group_ids == ("robot/gripper", "robot/arm")
    assert selection.joint_names == ("robot/joint3", "robot/joint1", "robot/joint2")
    assert selection.robot_names == ("robot",)

    overlapping = (
        PlanningGroup("robot/first", "robot", "first", ("robot/joint1",), ("joint1",), "base"),
        PlanningGroup("robot/second", "robot", "second", ("robot/joint1",), ("joint1",), "base"),
    )
    with pytest.raises(ValueError, match="overlap"):
        type(selection).from_groups(overlapping)


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


def test_joint_target_to_global_names_rejects_bad_counts_missing_and_extra() -> None:
    group = _make_group()

    with pytest.raises(ValueError, match="2 positions, expected 3"):
        joint_target_to_global_names(group, JointState({"position": [1.0, 2.0]}))
    with pytest.raises(ValueError, match="2 names but 3 positions"):
        joint_target_to_global_names(
            group, JointState({"name": ["j1", "j2"], "position": [1.0, 2.0, 3.0]})
        )
    with pytest.raises(ValueError, match="missing joints"):
        joint_target_to_global_names(
            group, JointState({"name": ["j1", "j2"], "position": [1.0, 2.0]})
        )
    with pytest.raises(ValueError, match="extra joints"):
        joint_target_to_global_names(
            group, JointState({"name": ["j1", "j2", "j3", "j4"], "position": [1.0, 2.0, 3.0, 4.0]})
        )


def test_filter_joint_state_to_selected_joints_uses_local_fallbacks() -> None:
    joint_state = JointState({"name": ["j1", "robot/j2"], "position": [1.0, 2.0]})

    filtered = filter_joint_state_to_selected_joints(
        joint_state,
        ["robot/j1", "robot/j2"],
        ["j1", "j2"],
    )

    assert filtered.name == ["robot/j1", "robot/j2"]
    assert filtered.position == [1.0, 2.0]


def test_filter_joint_state_to_selected_joints_rejects_mismatched_and_missing_names() -> None:
    joint_state = JointState({"name": ["robot/j1"], "position": [1.0]})

    with pytest.raises(ValueError, match="same length"):
        filter_joint_state_to_selected_joints(joint_state, ["robot/j1", "robot/j2"], ["j1"])
    with pytest.raises(ValueError, match="missing selected joints"):
        filter_joint_state_to_selected_joints(joint_state, ["robot/j1", "robot/j2"])


def test_matching_global_joint_name_requires_unique_suffix_match() -> None:
    assert matching_global_joint_name({"left/j1": 1.0, "right/j2": 2.0}, "j1") == "left/j1"
    assert matching_global_joint_name({"left/j1": 1.0, "right/j1": 2.0}, "j1") is None
    assert matching_global_joint_name({"left/j1": 1.0}, "j2") is None


def test_matching_global_joint_name_warns_when_suffix_match_is_not_unique(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    messages: list[str] = []

    def capture_warning(message: str) -> None:
        messages.append(message)

    monkeypatch.setattr(planning_group_utils.logger, "warning", capture_warning)

    assert matching_global_joint_name({"left/j1": 1.0, "right/j1": 2.0}, "j1") is None
    assert messages == [
        "Expected exactly one global joint ending with local joint name 'j1', "
        "found 2 matches: ['left/j1', 'right/j1']"
    ]


def test_planning_group_id_from_selector_accepts_id_or_group() -> None:
    group = _make_group()

    assert planning_group_id_from_selector(group) == "left/arm"
    assert planning_group_id_from_selector("left/arm") == "left/arm"


def test_local_joint_name_from_global_validates_robot_prefix_and_local_shape() -> None:
    assert local_joint_name_from_global("robot", "robot/j1") == "j1"
    with pytest.raises(ValueError, match="does not belong"):
        local_joint_name_from_global("robot", "other/j1")
    with pytest.raises(ValueError, match="Invalid global joint name"):
        local_joint_name_from_global("robot", "robot/")


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
