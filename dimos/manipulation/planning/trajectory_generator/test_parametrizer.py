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

"""Contract tests for PlanningResult-to-GeneratedPlan materialization."""

from unittest.mock import MagicMock

import pytest

from dimos.manipulation.planning.groups.models import (
    PlanningGroup,
    PlanningGroupSelection,
)
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import PlanningResult
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.planning.trajectory_generator.parametrizer import (
    BaseTrajectoryParametrizer,
    TrajectoryParametrizationError,
)
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


class _FixedParametrizer(BaseTrajectoryParametrizer):
    def __init__(self, output: JointTrajectory) -> None:
        self.output = output
        self.calls: list[float] = []

    def _parametrize_path(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        path: tuple[JointState, ...],
        speed_scale: float,
    ) -> JointTrajectory:
        self.calls.append(speed_scale)
        return self.output


def _selection() -> PlanningGroupSelection:
    return PlanningGroupSelection.from_groups(
        (
            PlanningGroup(
                id="group",
                joint_names=("arm/a", "arm/b"),
                base_link="base",
            ),
        )
    )


def _path() -> list[JointState]:
    names = ["arm/a", "arm/b"]
    return [
        JointState(name=names, position=[0.0, 0.0]),
        JointState(name=names, position=[0.2, 0.1]),
        JointState(name=names, position=[0.4, 0.0]),
    ]


def _output() -> JointTrajectory:
    return JointTrajectory(
        joint_names=["arm/a", "arm/b"],
        points=[
            TrajectoryPoint(
                time_from_start=0.0,
                positions=[0.0, 0.0],
                velocities=[0.0, 0.0],
            ),
            TrajectoryPoint(
                time_from_start=0.5,
                positions=[0.4, 0.0],
                velocities=[0.0, 0.0],
            ),
        ],
    )


def test_materializes_trajectory_and_preserves_source_path() -> None:
    parametrizer = _FixedParametrizer(_output())
    source_path = _path()

    plan = parametrizer.materialize_plan(
        MagicMock(spec=WorldSpec),
        _selection(),
        PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=source_path,
            planning_time=0.2,
            iterations=12,
        ),
        speed_scale=0.4,
    )

    assert [state.position for state in plan.path] == [
        [0.0, 0.0],
        [0.2, 0.1],
        [0.4, 0.0],
    ]
    assert plan.path is not source_path
    assert plan.trajectory is parametrizer.output
    assert plan.planning_time == 0.2
    assert plan.iterations == 12
    assert parametrizer.calls == [0.4]


def test_timed_planner_result_bypasses_backend_path_conversion() -> None:
    parametrizer = _FixedParametrizer(_output())
    path = [
        JointState(
            name=["arm/a", "arm/b"],
            position=[0.0, 0.0],
            velocity=[0.0, 0.0],
        ),
        JointState(
            name=["arm/a", "arm/b"],
            position=[0.4, 0.0],
            velocity=[0.3, 0.0],
        ),
    ]

    plan = parametrizer.materialize_plan(
        MagicMock(spec=WorldSpec),
        _selection(),
        PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=path,
            timestamps=[0.0, 0.75],
        ),
    )

    assert parametrizer.calls == []
    assert [point.time_from_start for point in plan.trajectory.points] == [
        0.0,
        0.75,
    ]
    assert plan.trajectory.points[-1].velocities == [0.3, 0.0]


def test_timed_planner_result_requires_velocity_for_each_joint() -> None:
    parametrizer = _FixedParametrizer(_output())
    path = _path()

    with pytest.raises(TrajectoryParametrizationError, match="velocity dimension"):
        parametrizer.materialize_plan(
            MagicMock(spec=WorldSpec),
            _selection(),
            PlanningResult(
                status=PlanningStatus.SUCCESS,
                path=path,
                timestamps=[0.0, 0.5, 1.0],
            ),
        )


def test_rejects_backend_trajectory_with_nonincreasing_time() -> None:
    output = _output()
    output.points[-1].time_from_start = 0.0
    parametrizer = _FixedParametrizer(output)

    with pytest.raises(TrajectoryParametrizationError, match="strictly increasing"):
        parametrizer.materialize_plan(
            MagicMock(spec=WorldSpec),
            _selection(),
            PlanningResult(status=PlanningStatus.SUCCESS, path=_path()),
        )


def test_rejects_backend_trajectory_that_changes_path_goal() -> None:
    output = _output()
    output.points[-1].positions = [0.3, 0.0]
    parametrizer = _FixedParametrizer(output)

    with pytest.raises(TrajectoryParametrizationError, match="path goal"):
        parametrizer.materialize_plan(
            MagicMock(spec=WorldSpec),
            _selection(),
            PlanningResult(status=PlanningStatus.SUCCESS, path=_path()),
        )


@pytest.mark.parametrize(
    ("path", "message"),
    [
        (
            [
                JointState(name=["arm/a", "arm/b"], position=[0.0, 0.0]),
                JointState(name=["wrong/a", "wrong/b"], position=[0.4, 0.0]),
            ],
            "joint names",
        ),
        (
            [
                JointState(name=["arm/a", "arm/b"], position=[0.0, 0.0]),
                JointState(name=["arm/a", "arm/b"], position=[0.4]),
            ],
            "dimension",
        ),
        (
            [
                JointState(name=["arm/a", "arm/b"], position=[0.0, 0.0]),
                JointState(name=["arm/a", "arm/b"], position=[float("nan"), 0.0]),
            ],
            "non-finite",
        ),
    ],
)
def test_rejects_malformed_path_before_invoking_backend(
    path: list[JointState],
    message: str,
) -> None:
    parametrizer = _FixedParametrizer(_output())

    with pytest.raises(TrajectoryParametrizationError, match=message):
        parametrizer.materialize_plan(
            MagicMock(spec=WorldSpec),
            _selection(),
            PlanningResult(status=PlanningStatus.SUCCESS, path=path),
        )

    assert parametrizer.calls == []
