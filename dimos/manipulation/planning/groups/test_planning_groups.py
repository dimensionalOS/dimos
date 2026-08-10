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

# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Tests for canonical, single-model planning groups."""

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
from dimos.manipulation.planning.groups.utils import (
    filter_joint_state_to_selected_joints,
    joint_state_to_ordered_positions,
    normalize_joint_target,
    planning_group_id_from_selector,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
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


def _config(groups: list[PlanningGroupDefinition] | None = None) -> RobotModelConfig:
    return RobotModelConfig(
        model_path=Path("/tmp/model.urdf"),
        joint_names=["left/j1", "left/j2", "right/j1"],
        planning_groups=groups
        or [
            PlanningGroupDefinition("left_arm", ("left/j1", "left/j2"), "base", "left/tool"),
            PlanningGroupDefinition("right_arm", ("right/j1",), "base", "right/tool"),
        ],
    )


def test_parse_srdf_chain_group(tmp_path: Path) -> None:
    path = tmp_path / "model.srdf"
    path.write_text(
        "<robot name='test'><group name='arm'>"
        "<chain base_link='link0' tip_link='link2'/></group></robot>"
    )
    groups = parse_srdf_planning_groups(
        path,
        model=_serial_model("revolute", "revolute"),
        controllable_joint_names=["joint1", "joint2"],
    )
    assert groups == [
        PlanningGroupDefinition("arm", ("joint1", "joint2"), "link0", "link2", "srdf")
    ]


def test_fallback_generation_and_branch_rejection() -> None:
    group = generate_fallback_planning_group(
        model=_serial_model("revolute", "revolute", "prismatic"),
        controllable_joint_names=["joint1", "joint2", "joint3"],
    )
    assert group.name == FALLBACK_PLANNING_GROUP_NAME
    assert group.joint_names == ("joint1", "joint2")

    branching = ModelDescription(
        joints=[
            JointDescription("left", "revolute", "base", "left_link"),
            JointDescription("right", "revolute", "base", "right_link"),
        ],
        root_link="base",
        links=["base", "left_link", "right_link"],
    )
    with pytest.raises(PlanningGroupDiscoveryError, match="branch"):
        generate_fallback_planning_group(
            model=branching, controllable_joint_names=["left", "right"]
        )


def test_discovery_rejects_missing_explicit_srdf(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match="SRDF file not found"):
        discover_planning_group_definitions(
            model_path=tmp_path / "model.urdf",
            model=_serial_model("revolute"),
            controllable_joint_names=["joint1"],
            srdf_path=tmp_path / "missing.srdf",
        )


def test_registry_uses_stable_unprefixed_ids_and_canonical_joints() -> None:
    registry = PlanningGroupRegistry([_config()])
    assert [group.id for group in registry.list()] == ["left_arm", "right_arm"]
    assert registry.get("left_arm").joint_names == ("left/j1", "left/j2")
    assert registry.default_group_id() is None
    with pytest.raises(ValueError, match="explicit planning group ID"):
        registry.primary_pose_group_id()


def test_registry_default_requires_exactly_one_compatible_group() -> None:
    registry = PlanningGroupRegistry(
        [_config([PlanningGroupDefinition("arm", ("left/j1",), "base", "tool")])]
    )
    assert registry.default_group_id() == "arm"
    assert registry.primary_pose_group_id() == "arm"


def test_selection_preserves_order_and_rejects_overlap() -> None:
    registry = PlanningGroupRegistry([_config()])
    selection = registry.select(("right_arm", "left_arm"))
    assert selection.group_ids == ("right_arm", "left_arm")
    assert selection.joint_names == ("right/j1", "left/j1", "left/j2")

    overlap = _config(
        [
            PlanningGroupDefinition("one", ("left/j1",), "base"),
            PlanningGroupDefinition("two", ("left/j1",), "base"),
        ]
    )
    with pytest.raises(ValueError, match="overlap"):
        PlanningGroupRegistry([overlap]).select(("one", "two"))


def test_normalize_joint_target_accepts_exact_or_unnamed_target() -> None:
    group = PlanningGroup("left_arm", ("left/j1", "left/j2"), "base", "tool")
    named = normalize_joint_target(
        group,
        JointState(name=["left/j1", "left/j2"], position=[1.0, 2.0]),
    )
    unnamed = normalize_joint_target(group, JointState(position=[3.0, 4.0]))
    assert named.name == ["left/j1", "left/j2"]
    assert unnamed.name == ["left/j1", "left/j2"]
    with pytest.raises(ValueError, match="missing joints"):
        normalize_joint_target(
            group,
            JointState(name=["j1", "j2"], position=[1.0, 2.0]),
        )


def test_state_projection_requires_exact_canonical_names() -> None:
    state = JointState(name=["right/j1", "left/j2", "left/j1"], position=[3.0, 2.0, 1.0])
    projected = filter_joint_state_to_selected_joints(state, ("left/j1", "right/j1"))
    assert projected.name == ["left/j1", "right/j1"]
    assert projected.position == [1.0, 3.0]
    assert joint_state_to_ordered_positions(
        state, joint_names=("left/j1", "left/j2", "right/j1")
    ).tolist() == [1.0, 2.0, 3.0]
    with pytest.raises(ValueError, match="missing"):
        filter_joint_state_to_selected_joints(state, ("missing",))


def test_selector_accepts_id_or_group() -> None:
    group = PlanningGroup("arm", ("left/j1",), "base", "tool")
    assert planning_group_id_from_selector("arm") == "arm"
    assert planning_group_id_from_selector(group) == "arm"


def test_model_config_rejects_obsolete_name_and_mapping_fields() -> None:
    base = {"model_path": Path("/tmp/model.urdf"), "joint_names": ["joint1"]}
    with pytest.raises(ValueError):
        RobotModelConfig(**base, name="arm")
    with pytest.raises(ValueError):
        RobotModelConfig(**base, joint_name_mapping={"arm/joint1": "joint1"})
