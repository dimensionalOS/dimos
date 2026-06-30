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

"""Tests for DrakeWorld planning group name/world resolution."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.world.drake_world import DrakeWorld, _RobotData
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState


def _pose() -> PoseStamped:
    return PoseStamped(position=[0, 0, 0], orientation=[0, 0, 0, 1])


def _config(
    name: str,
    joint_names: list[str],
    groups: list[PlanningGroupDefinition],
) -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model_path=Path("robot.urdf"),
        base_pose=_pose(),
        joint_names=joint_names,
        end_effector_link="tool0",
        base_link="base_link",
        planning_groups=groups,
    )


def _world(*configs: RobotModelConfig) -> DrakeWorld:
    world = DrakeWorld.__new__(DrakeWorld)
    world._robots = {
        f"robot_{index}": _RobotData(
            robot_id=f"robot_{index}",
            config=config,
            model_instance=None,
            joint_indices=[],
            ee_frame=None,
            base_frame=None,
        )
        for index, config in enumerate(configs, start=1)
    }
    world._planning_groups = PlanningGroupRegistry(configs)
    return world


def _arm_group(*joint_names: str) -> PlanningGroupDefinition:
    return PlanningGroupDefinition(
        name="arm",
        joint_names=joint_names,
        base_link="base_link",
        tip_link="tool0",
        source="srdf",
    )


def test_planning_group_registry_returns_stable_ids_and_global_joint_names() -> None:
    config = _config("left", ["joint1", "joint2"], [_arm_group("joint1", "joint2")])
    registry = PlanningGroupRegistry([config])

    groups = registry.list()

    assert len(groups) == 1
    assert groups[0].id == "left/arm"
    assert groups[0].robot_name == "left"
    assert groups[0].group_name == "arm"
    assert groups[0].joint_names == ("left/joint1", "left/joint2")
    assert groups[0].local_joint_names == ("joint1", "joint2")


def test_robot_model_config_allows_planning_groups_without_robot_scoped_ee() -> None:
    config = RobotModelConfig(
        name="left",
        model_path=Path("robot.urdf"),
        joint_names=["joint1"],
        planning_groups=[_arm_group("joint1")],
    )
    registry = PlanningGroupRegistry([config])

    groups = registry.list()

    assert config.end_effector_link is None
    assert groups[0].id == "left/arm"


def test_duplicate_local_joint_names_across_robots_are_disambiguated() -> None:
    registry = PlanningGroupRegistry(
        [
            _config("left", ["joint1"], [_arm_group("joint1")]),
            _config("right", ["joint1"], [_arm_group("joint1")]),
        ]
    )

    groups = registry.list()

    assert [group.id for group in groups] == ["left/arm", "right/arm"]
    assert [group.joint_names for group in groups] == [("left/joint1",), ("right/joint1",)]


def test_planning_group_selection_returns_ordered_global_joint_names() -> None:
    registry = PlanningGroupRegistry(
        [
            _config("left", ["joint1", "joint2"], [_arm_group("joint1", "joint2")]),
            _config("right", ["joint1", "joint2"], [_arm_group("joint2")]),
        ]
    )

    selection = registry.select(("left/arm", "right/arm"))

    assert list(selection.group_ids) == ["left/arm", "right/arm"]
    assert list(selection.robot_names) == ["left", "right"]
    assert [group.joint_names for group in selection.groups] == [
        ("left/joint1", "left/joint2"),
        ("right/joint2",),
    ]
    assert list(selection.joint_names) == ["left/joint1", "left/joint2", "right/joint2"]
    assert [group.local_joint_names for group in selection.groups] == [
        ("joint1", "joint2"),
        ("joint2",),
    ]


def test_planning_group_registry_unknown_group_raises_key_error() -> None:
    registry = PlanningGroupRegistry([_config("left", ["joint1"], [_arm_group("joint1")])])

    with pytest.raises(KeyError, match="Unknown planning group ID: left/gripper"):
        registry.select(("left/gripper",))


def test_planning_group_selection_overlapping_same_robot_groups_raise_value_error() -> None:
    registry = PlanningGroupRegistry(
        [
            _config(
                "left",
                ["joint1", "joint2"],
                [
                    _arm_group("joint1", "joint2"),
                    PlanningGroupDefinition(
                        name="wrist",
                        joint_names=("joint2",),
                        base_link="link1",
                        tip_link="tool0",
                    ),
                ],
            )
        ]
    )

    with pytest.raises(ValueError, match="overlap.*left/joint2"):
        registry.select(("left/arm", "left/wrist"))


def test_positions_for_robot_state_accepts_local_joint_names_in_config_order() -> None:
    world = _world(_config("left", ["joint1", "joint2"], [_arm_group("joint1", "joint2")]))
    joint_state = JointState({"name": ["joint2", "joint1"], "position": [2.0, 1.0]})

    positions = world._positions_for_robot_state("robot_1", joint_state)

    np.testing.assert_allclose(positions, np.array([1.0, 2.0]))


def test_positions_for_robot_state_rejects_global_joint_names() -> None:
    world = _world(_config("left", ["joint1", "joint2"], [_arm_group("joint1", "joint2")]))
    joint_state = JointState({"name": ["left/joint2", "left/joint1"], "position": [2.0, 1.0]})

    with pytest.raises(ValueError, match="Invalid local joint name: 'left/joint2'"):
        world._positions_for_robot_state("robot_1", joint_state)


def test_group_pose_rejects_group_without_target_frame() -> None:
    world = _world(
        _config(
            "left",
            ["joint1"],
            [
                PlanningGroupDefinition(
                    name="waist",
                    joint_names=("joint1",),
                    base_link="base_link",
                    tip_link=None,
                )
            ],
        )
    )
    world._finalized = True

    with pytest.raises(ValueError, match="left/waist.*no pose target frame"):
        world.get_group_ee_pose(None, "left/waist")
