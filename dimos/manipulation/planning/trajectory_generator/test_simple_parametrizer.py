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

"""Tests for the compatibility trajectory parametrizer."""

import pytest

from dimos.manipulation.planning.trajectory_generator.config import (
    SimpleTrapezoidParametrizationConfig,
)
from dimos.manipulation.planning.trajectory_generator.parametrizer import (
    TrajectoryParametrizationError,
    TrajectoryParametrizationRequest,
)
from dimos.manipulation.planning.trajectory_generator.simple_parametrizer import (
    SimpleTrapezoidParametrizer,
)
from dimos.msgs.sensor_msgs.JointState import JointState


def _request(*, speed_scale: float = 1.0) -> TrajectoryParametrizationRequest:
    names = ("arm/a", "arm/b")
    return TrajectoryParametrizationRequest(
        group_ids=("arm/manipulator",),
        joint_names=names,
        path=(
            JointState(name=list(names), position=[0.0, 0.0]),
            JointState(name=list(names), position=[0.2, 0.1]),
            JointState(name=list(names), position=[0.4, 0.0]),
        ),
        velocity_limits=(2.0, 4.0),
        acceleration_limits=(6.0, 8.0),
        speed_scale=speed_scale,
    )


def test_simple_parametrizer_preserves_segmented_trapezoid_behavior() -> None:
    request = _request(speed_scale=0.5)
    parametrizer = SimpleTrapezoidParametrizer(
        SimpleTrapezoidParametrizationConfig(
            velocity_scale=0.5,
            acceleration_scale=0.25,
            points_per_segment=4,
        )
    )

    result = parametrizer.parametrize(request)

    assert parametrizer.uses_request_limits
    assert result.velocity_limits == (0.5, 1.0)
    assert result.acceleration_limits == (0.75, 1.0)
    assert result.accelerations is None
    assert result.trajectory.joint_names == list(request.joint_names)
    assert len(result.trajectory.points) == 9
    assert result.trajectory.points[0].positions == [0.0, 0.0]
    assert result.trajectory.points[4].positions == [0.2, 0.1]
    assert result.trajectory.points[-1].positions == [0.4, 0.0]
    assert [state.position for state in request.path] == [
        [0.0, 0.0],
        [0.2, 0.1],
        [0.4, 0.0],
    ]


def test_simple_parametrizer_requires_dimos_limits() -> None:
    request = _request()
    request = TrajectoryParametrizationRequest(
        group_ids=request.group_ids,
        joint_names=request.joint_names,
        path=request.path,
    )

    with pytest.raises(
        TrajectoryParametrizationError,
        match="requires DimOS motion limits",
    ):
        SimpleTrapezoidParametrizer(SimpleTrapezoidParametrizationConfig()).parametrize(request)


@pytest.mark.parametrize("speed_scale", [0.0, -0.1, 1.01, float("inf"), float("nan")])
def test_parametrization_request_rejects_invalid_runtime_speed(speed_scale: float) -> None:
    with pytest.raises(ValueError, match="speed_scale"):
        _request(speed_scale=speed_scale)
