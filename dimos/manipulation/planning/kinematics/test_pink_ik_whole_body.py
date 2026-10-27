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

"""Pink IK with groups free to help: the whole-body posture pull and the planar base's costs."""

from __future__ import annotations

from collections.abc import Mapping
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupDefinition
from dimos.manipulation.planning.kinematics.pink_ik import (
    _WHOLE_BODY_POSTURE_TASK,
    PinkIK,
    PinkIKConfig,
    _whole_body_posture_costs,
)
from dimos.manipulation.planning.kinematics.pink_solver import (
    _build_joint_mapping,
)
from dimos.manipulation.planning.kinematics.test_pink_ik import (
    _context,
    _FakePlanarModel,
    _FakeWorld,
    _install_fake_modules,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.joint_space import (
    CoordinateTopology,
    JointCoordinate,
    JointSpace,
)
from dimos.manipulation.planning.spec.models import IKResult
from dimos.manipulation.planning.spec.validation import PreparedRobotModel, prepare_robot_model
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import PlanarBaseDefinition, RobotModel
from dimos.utils.transform_utils import matrix_to_pose


def test_whole_body_posture_costs_tell_a_planar_base_from_the_arm() -> None:
    """A planar base's translation and yaw get their own costs; everything else the arm's."""
    planar = PlanarBaseDefinition(
        velocity_limits=(1.0, 1.0, 2.0),
        acceleration_limits=(2.0, 2.0, 4.0),
    )
    joint_space = JointSpace(
        (
            JointCoordinate(
                planar.joint_names[0], "prismatic", CoordinateTopology.LINE, None, None, 1.0, 2.0
            ),
            JointCoordinate(
                planar.joint_names[1], "prismatic", CoordinateTopology.LINE, None, None, 1.0, 2.0
            ),
            JointCoordinate(
                planar.joint_names[2], "continuous", CoordinateTopology.CIRCLE, None, None, 2.0, 4.0
            ),
        )
    )
    mapping = _build_joint_mapping(cast("Any", _FakePlanarModel()), joint_space)
    config = PinkIKConfig(
        whole_body_posture_cost=0.02,
        whole_body_base_translation_cost=0.03,
        whole_body_base_yaw_cost=0.4,
    )

    with_base = _whole_body_posture_costs(config, mapping, [0, 1, 2], planar.joint_names, nv=3)
    no_base = _whole_body_posture_costs(config, mapping, [0, 2], (), nv=3)

    assert with_base == pytest.approx([0.03, 0.03, 0.4])
    assert no_base == pytest.approx([0.02, 0.0, 0.02]), "an unselected joint costs nothing"


def _recorded_stacks(mocker: MockerFixture, ik: PinkIK) -> list[Mapping[str, Any]]:
    stacks: list[Mapping[str, Any]] = []
    original = ik._configuration_and_tasks

    def record(*args: Any, **kwargs: Any) -> tuple[Any, Mapping[str, Any]]:
        configuration, tasks = original(*args, **kwargs)
        stacks.append(tasks)
        return configuration, tasks

    mocker.patch.object(ik, "_configuration_and_tasks", side_effect=record)
    return stacks


def test_auxiliary_groups_add_one_posture_pull_toward_the_seed(mocker: MockerFixture) -> None:
    """With a group free to help, the stack gains a posture task on every selected joint
    whose target is the seed; the arm-only stack is what it always was."""
    _install_fake_modules(mocker)
    ik = PinkIK(PinkIKConfig(max_iterations=3, whole_body_posture_cost=0.02))
    context = _context()
    mocker.patch.object(ik, "_get_model_context", return_value=context)
    world = _FakeWorld()
    stacks = _recorded_stacks(mocker, ik)
    target = PoseStamped(
        position=Vector3(0.1, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
    )
    seed = JointState({"name": ["joint_a", "joint_b", "joint_c"], "position": [0.1, 0.2, 0.3]})

    whole_body = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={world.groups["manipulator"]: target},
        auxiliary_groups=[world.groups["no_tip"]],
        seed=seed,
        check_collision=False,
        max_attempts=1,
    )
    arm_only = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={world.groups["manipulator"]: target},
        seed=seed,
        check_collision=False,
        max_attempts=1,
    )

    assert whole_body.status == IKStatus.SUCCESS
    assert arm_only.status == IKStatus.SUCCESS
    with_help, alone = stacks
    posture = with_help[_WHOLE_BODY_POSTURE_TASK]
    # The model orders its joints b, a, c; every selected joint is pulled, in model order.
    assert posture.cost == pytest.approx([0.02, 0.02, 0.02])
    assert posture.target == pytest.approx([0.2, 0.1, 0.3])
    assert list(with_help) == [*alone, _WHOLE_BODY_POSTURE_TASK]
    assert _WHOLE_BODY_POSTURE_TASK not in alone


def test_zero_whole_body_costs_add_no_task(mocker: MockerFixture) -> None:
    _install_fake_modules(mocker)
    ik = PinkIK(
        PinkIKConfig(
            max_iterations=3,
            whole_body_posture_cost=0.0,
            whole_body_base_translation_cost=0.0,
            whole_body_base_yaw_cost=0.0,
        )
    )
    world = _FakeWorld()
    mocker.patch.object(ik, "_get_model_context", return_value=_context())
    stacks = _recorded_stacks(mocker, ik)

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["manipulator"]: PoseStamped(
                position=Vector3(0.1, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
        },
        auxiliary_groups=[world.groups["no_tip"]],
        seed=JointState({"name": ["joint_a", "joint_b", "joint_c"], "position": [0.1, 0.2, 0.3]}),
        check_collision=False,
        max_attempts=1,
    )

    assert result.status == IKStatus.SUCCESS
    assert _WHOLE_BODY_POSTURE_TASK not in stacks[0]


# A pitch arm on a planar base, its shoulder off the base's centre the way Alfred's right
# arm is: turning the base swings the hand forward and sideways, so a turn is a tempting
# way to reach that leaves the arm compensating for it.
_ARM_ON_A_BASE_URDF = """\
<robot name="arm_on_a_base">
  <link name="base"/><link name="shoulder_link"/><link name="upper_arm"/><link name="forearm"/>
  <link name="wrist"/><link name="tool"/>
  <joint name="shoulder_yaw" type="revolute"><origin xyz="0.1 -0.3 0.5"/><parent link="base"/><child link="shoulder_link"/><axis xyz="0 0 1"/><limit lower="-2.5" upper="2.5" effort="1" velocity="1"/></joint>
  <joint name="shoulder_pitch" type="revolute"><origin xyz="0 0 0"/><parent link="shoulder_link"/><child link="upper_arm"/><axis xyz="0 1 0"/><limit lower="-2.0" upper="2.0" effort="1" velocity="1"/></joint>
  <joint name="elbow" type="revolute"><origin xyz="0.3 0 0"/><parent link="upper_arm"/><child link="forearm"/><axis xyz="0 1 0"/><limit lower="-2.0" upper="2.0" effort="1" velocity="1"/></joint>
  <joint name="wrist_pitch" type="revolute"><origin xyz="0.3 0 0"/><parent link="forearm"/><child link="wrist"/><axis xyz="0 1 0"/><limit lower="-2.0" upper="2.0" effort="1" velocity="1"/></joint>
  <joint name="tool_fixed" type="fixed"><origin xyz="0.1 0 0"/><parent link="wrist"/><child link="tool"/></joint>
</robot>
"""
_ARM_ON_A_BASE_PLANAR = PlanarBaseDefinition(
    velocity_limits=(1.0, 1.0, 2.0), acceleration_limits=(2.0, 2.0, 4.0)
)
_ARM_ON_A_BASE_ARM = ("shoulder_yaw", "shoulder_pitch", "elbow", "wrist_pitch")


@dataclass(frozen=True)
class _ArmOnABase:
    prepared: PreparedRobotModel
    arm: PlanningGroup
    base: PlanningGroup
    seed: JointState
    world: Any


def _arm_on_a_base(tmp_path: Path) -> _ArmOnABase:
    planar = _ARM_ON_A_BASE_PLANAR
    model_path = tmp_path / "arm_on_a_base.urdf"
    model_path.write_text(_ARM_ON_A_BASE_URDF)
    joint_names = [*planar.joint_names, *_ARM_ON_A_BASE_ARM]
    config = RobotModelConfig(
        model=(
            RobotModel.from_file(model_path)
            .with_default_joint_acceleration_limit(2.0)
            .with_planar_base(planar)
        ),
        joint_names=joint_names,
        base_link=planar.root_link,
        planning_groups=[
            PlanningGroupDefinition(
                name="arm", joint_names=_ARM_ON_A_BASE_ARM, base_link="base", tip_link="tool"
            ),
            PlanningGroupDefinition(
                name="moving_base", joint_names=planar.joint_names, base_link=planar.root_link
            ),
        ],
    )
    # The base where it stands, the arm bent at the elbow with the hand level.
    seed = JointState(name=joint_names, position=[0.0, 0.0, 0.0, 0.0, -0.5, 1.0, -0.5])

    class World:
        is_finalized = True

        def __init__(self, prepared: PreparedRobotModel) -> None:
            self.prepared = prepared

        def get_prepared_model(self) -> PreparedRobotModel:
            return self.prepared

        def scratch_context(self) -> nullcontext[None]:
            return nullcontext(None)

        def get_joint_state(self, ctx: object) -> JointState:
            return seed

        def check_config_collision_free(self, joint_state: JointState) -> bool:
            return True

    prepared = prepare_robot_model(config)
    return _ArmOnABase(
        prepared=prepared,
        arm=PlanningGroup(
            id="arm", joint_names=_ARM_ON_A_BASE_ARM, base_link="base", tip_link="tool"
        ),
        base=PlanningGroup(
            id="moving_base",
            joint_names=planar.joint_names,
            base_link=planar.root_link,
            tip_link=None,
        ),
        seed=seed,
        world=World(prepared),
    )


def _solved_positions(result: IKResult) -> dict[str, float]:
    assert result.status == IKStatus.SUCCESS, result.message
    assert result.joint_state is not None
    return dict(zip(result.joint_state.name, result.joint_state.position, strict=True))


def _hand_pose(ik: PinkIK, robot: _ArmOnABase, dx: float = 0.0, yaw: float = 0.0) -> PoseStamped:
    """The seed's hand pose moved ``dx`` metres forward and turned ``yaw`` radians."""
    context = ik._build_robot_context(robot.prepared, "tool")
    hand = matrix_to_pose(
        ik._current_frame_matrix(
            context, ik._q_from_dimos_positions(context, np.asarray(robot.seed.position))
        )
    )
    return PoseStamped(
        frame_id="world",
        position=Vector3(hand.position.x + dx, hand.position.y, hand.position.z),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)) * hand.orientation,
    )


def test_a_reach_a_drive_forward_covers_does_not_turn_the_base(tmp_path: Path) -> None:
    robot = _arm_on_a_base(tmp_path)
    ik = PinkIK(PinkIKConfig())

    result = ik.solve_pose_targets(
        robot.world,
        {robot.arm: _hand_pose(ik, robot, dx=0.2)},
        auxiliary_groups=[robot.base],
        seed=robot.seed,
        check_collision=False,
        max_attempts=1,
    )

    positions = _solved_positions(result)
    x, y, yaw = (positions[name] for name in _ARM_ON_A_BASE_PLANAR.joint_names)
    assert abs(yaw) < 0.01
    assert x == pytest.approx(0.2, abs=0.02)
    assert abs(y) < 0.01
    seed = dict(zip(robot.seed.name, robot.seed.position, strict=True))
    for name in _ARM_ON_A_BASE_ARM:
        assert positions[name] == pytest.approx(seed[name], abs=0.1), name


def test_a_hand_turn_the_arm_can_make_itself_does_not_turn_the_base(tmp_path: Path) -> None:
    """Minimum-norm differential IK splits a turn between the base and the shoulder, both
    about z, and the base then has to drive sideways to keep the hand in place. With the
    pull toward the seed, the shoulder turns and the base stays put."""
    robot = _arm_on_a_base(tmp_path)
    free = PinkIKConfig(
        whole_body_posture_cost=0.0,
        whole_body_base_translation_cost=0.0,
        whole_body_base_yaw_cost=0.0,
    )
    turned_hand = _hand_pose(PinkIK(free), robot, yaw=0.5)

    outcomes = {
        label: _solved_positions(
            PinkIK(config).solve_pose_targets(
                robot.world,
                {robot.arm: turned_hand},
                auxiliary_groups=[robot.base],
                seed=robot.seed,
                check_collision=False,
                max_attempts=1,
            )
        )
        for label, config in (("free", free), ("regularized", PinkIKConfig()))
    }

    base_yaw = _ARM_ON_A_BASE_PLANAR.joint_names[2]
    assert outcomes["free"][base_yaw] > 0.1, "without the pull the base turns"
    assert abs(outcomes["regularized"][base_yaw]) < 0.01
    assert outcomes["regularized"]["shoulder_yaw"] == pytest.approx(0.5, abs=0.02)
