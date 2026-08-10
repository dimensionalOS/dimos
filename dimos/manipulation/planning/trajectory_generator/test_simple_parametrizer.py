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

"""Tests for the compatibility trajectory parametrizer Spec implementation."""

from pathlib import Path
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.planning.groups.models import (
    PlanningGroup,
    PlanningGroupSelection,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import PlanningResult
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.planning.trajectory_generator.config import (
    SimpleTrapezoidParametrizationConfig,
)
from dimos.manipulation.planning.trajectory_generator.parametrizer import (
    TrajectoryParametrizationError,
)
from dimos.manipulation.planning.trajectory_generator.simple_parametrizer import (
    SimpleTrapezoidParametrizer,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState


def _selection() -> PlanningGroupSelection:
    return PlanningGroupSelection.from_groups(
        (
            PlanningGroup(
                id="manipulator",
                joint_names=("arm/a", "arm/b"),
                base_link="base",
                tip_link="tip",
            ),
        )
    )


def _world(*, velocity: float = 2.0, acceleration: float = 6.0) -> WorldSpec:
    config = RobotModelConfig(
        model_path=Path("/robot.urdf"),
        base_pose=PoseStamped(),
        joint_names=["arm/a", "arm/b"],
        base_link="base",
        max_velocity=velocity,
        max_acceleration=acceleration,
    )
    world = MagicMock(spec=WorldSpec)
    world.get_model_config.return_value = config
    return world


def _result() -> PlanningResult:
    names = ["arm/a", "arm/b"]
    return PlanningResult(
        status=PlanningStatus.SUCCESS,
        path=[
            JointState(name=names, position=[0.0, 0.0]),
            JointState(name=names, position=[0.2, 0.1]),
            JointState(name=names, position=[0.4, 0.0]),
        ],
    )


def test_simple_parametrizer_materializes_segmented_trapezoid_plan() -> None:
    parametrizer = SimpleTrapezoidParametrizer(
        SimpleTrapezoidParametrizationConfig(
            velocity_scale=0.5,
            acceleration_scale=0.25,
            points_per_segment=4,
        )
    )
    result = _result()

    plan = parametrizer.materialize_plan(
        _world(),
        _selection(),
        result,
        speed_scale=0.5,
    )

    assert plan.group_ids == ("manipulator",)
    assert plan.trajectory.joint_names == ["arm/a", "arm/b"]
    assert len(plan.trajectory.points) == 9
    assert plan.trajectory.points[0].positions == [0.0, 0.0]
    assert plan.trajectory.points[4].positions == [0.2, 0.1]
    assert plan.trajectory.points[-1].positions == [0.4, 0.0]
    assert [state.position for state in result.path] == [
        [0.0, 0.0],
        [0.2, 0.1],
        [0.4, 0.0],
    ]


def test_simple_parametrizer_rejects_invalid_dimos_limits() -> None:
    parametrizer = SimpleTrapezoidParametrizer(SimpleTrapezoidParametrizationConfig())

    with pytest.raises(
        TrajectoryParametrizationError,
        match="Invalid velocity limit for 'arm/a'",
    ):
        parametrizer.materialize_plan(
            _world(velocity=0.0),
            _selection(),
            _result(),
        )


def test_simple_parametrizer_reports_generator_failure(mocker: MockerFixture) -> None:
    generator = mocker.patch(
        "dimos.manipulation.planning.trajectory_generator."
        "simple_parametrizer.JointTrajectoryGenerator"
    )
    generator.return_value.generate.side_effect = RuntimeError("boom")

    with pytest.raises(TrajectoryParametrizationError, match="failed: boom"):
        SimpleTrapezoidParametrizer(SimpleTrapezoidParametrizationConfig()).materialize_plan(
            _world(),
            _selection(),
            _result(),
        )


@pytest.mark.parametrize(
    "speed_scale",
    [0.0, 1.01, float("nan")],
)
def test_parametrizer_rejects_invalid_runtime_speed(speed_scale: float) -> None:
    with pytest.raises(TrajectoryParametrizationError, match="speed_scale"):
        SimpleTrapezoidParametrizer(SimpleTrapezoidParametrizationConfig()).materialize_plan(
            _world(),
            _selection(),
            _result(),
            speed_scale=speed_scale,
        )
