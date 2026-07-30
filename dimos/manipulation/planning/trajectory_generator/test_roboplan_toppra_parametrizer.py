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

"""Tests for the RoboPlan TOPP-RA trajectory parametrizer."""

from contextlib import contextmanager
from types import SimpleNamespace

import numpy as np
import pytest
from pytest_mock import MockerFixture

pytest.importorskip("roboplan.toppra")

from dimos.manipulation.planning.trajectory_generator.config import (
    RoboPlanTOPPRAParametrizationConfig,
)
from dimos.manipulation.planning.trajectory_generator.parametrizer import (
    TrajectoryParametrizationError,
    TrajectoryParametrizationRequest,
)
from dimos.manipulation.planning.trajectory_generator.roboplan_toppra_parametrizer import (
    RoboPlanTOPPRAParametrizer,
)
from dimos.manipulation.planning.world.roboplan_model import RoboPlanGroup, RoboPlanModel
from dimos.msgs.sensor_msgs.JointState import JointState

pytestmark = pytest.mark.self_hosted


class _Scene:
    def __init__(self, *, missing_acceleration: bool = False) -> None:
        self.missing_acceleration = missing_acceleration

    def getVelocityLimitVectors(self, group_name: str) -> tuple[np.ndarray, np.ndarray]:
        assert group_name == "composite"
        return np.asarray([-2.0, -1.0]), np.asarray([2.0, 1.0])

    def getAccelerationLimitVectors(self, group_name: str) -> tuple[np.ndarray, np.ndarray]:
        assert group_name == "composite"
        maximum = np.finfo(np.float64).max if self.missing_acceleration else 4.0
        return np.asarray([-6.0, -maximum]), np.asarray([6.0, maximum])


class _World:
    def __init__(self, model: RoboPlanModel) -> None:
        self.model = model

    @contextmanager
    def parametrization_model(self):
        yield self.model


def _model(*, missing_acceleration: bool = False) -> RoboPlanModel:
    group = RoboPlanGroup(
        group_ids=("left/arm", "right/arm"),
        name="composite",
        native_names=("native_b", "native_a"),
        public_names=("right/b", "left/a"),
    )
    return RoboPlanModel(
        scene=_Scene(missing_acceleration=missing_acceleration),
        groups={frozenset(group.group_ids): group},
        legacy_group_ids={},
        native_joint_by_global={},
        native_link_by_robot={},
        all_group=group,
    )


def _request(
    names: tuple[str, str] = ("left/a", "right/b"),
    *,
    speed_scale: float = 1.0,
) -> TrajectoryParametrizationRequest:
    positions_by_name = {
        "left/a": (0.0, 0.3),
        "right/b": (0.1, 0.4),
    }
    return TrajectoryParametrizationRequest(
        group_ids=("right/arm", "left/arm"),
        joint_names=names,
        path=(
            JointState(
                name=list(names),
                position=[positions_by_name[name][0] for name in names],
            ),
            JointState(
                name=list(names),
                position=[positions_by_name[name][1] for name in names],
            ),
        ),
        velocity_limits=(999.0, 999.0),
        acceleration_limits=(999.0, 999.0),
        speed_scale=speed_scale,
    )


@pytest.mark.parametrize(
    "fitting_mode",
    ["hermite", "cubic", "adaptive", "linear_blend"],
)
def test_roboplan_parametrizer_maps_composite_order_options_and_native_output(
    mocker: MockerFixture,
    fitting_mode: str,
) -> None:
    generated = SimpleNamespace(
        joint_names=["native_a", "native_b"],
        times=[0.0, 0.5],
        positions=[np.asarray([0.0, 0.1]), np.asarray([0.3, 0.4])],
        velocities=[np.asarray([0.0, 0.0]), np.asarray([0.6, 0.2])],
        accelerations=[np.asarray([0.0, 0.0]), np.asarray([1.2, 0.4])],
    )
    native = mocker.MagicMock()
    native.generate.return_value = generated
    constructor = mocker.patch(
        "dimos.manipulation.planning.trajectory_generator."
        "roboplan_toppra_parametrizer.roboplan_toppra.PathParameterizerTOPPRA",
        return_value=native,
    )
    parametrizer = RoboPlanTOPPRAParametrizer(
        _World(_model()),
        RoboPlanTOPPRAParametrizationConfig(
            fitting_mode=fitting_mode,
            output_period=0.02,
            velocity_scale=0.5,
            acceleration_scale=0.25,
        ),
    )
    request = _request(speed_scale=0.5)

    result = parametrizer.parametrize(request)

    assert not parametrizer.uses_request_limits
    constructor.assert_called_once()
    native_path, options = native.generate.call_args.args
    assert native_path.joint_names == ["native_b", "native_a"]
    assert [row.tolist() for row in native_path.positions] == [
        [0.1, 0.0],
        [0.4, 0.3],
    ]
    assert options.dt == 0.02
    assert options.mode.name.lower().replace("linearblend", "linear_blend") == fitting_mode
    assert options.velocity_scale == 0.25
    assert options.acceleration_scale == 0.125
    assert result.velocity_limits == (0.25, 0.5)
    assert result.acceleration_limits == (0.5, 0.75)
    assert result.trajectory.joint_names == ["left/a", "right/b"]
    assert [point.positions for point in result.trajectory.points] == [
        [0.0, 0.1],
        [0.3, 0.4],
    ]
    assert [point.velocities for point in result.trajectory.points] == [
        [0.0, 0.0],
        [0.6, 0.2],
    ]
    assert result.accelerations == ((0.0, 0.0), (1.2, 0.4))
    assert [state.position for state in request.path] == [[0.0, 0.1], [0.3, 0.4]]


def test_roboplan_parametrizer_rejects_missing_urdf_acceleration_without_fallback(
    mocker: MockerFixture,
) -> None:
    constructor = mocker.patch(
        "dimos.manipulation.planning.trajectory_generator."
        "roboplan_toppra_parametrizer.roboplan_toppra.PathParameterizerTOPPRA"
    )
    parametrizer = RoboPlanTOPPRAParametrizer(
        _World(_model(missing_acceleration=True)),
        RoboPlanTOPPRAParametrizationConfig(),
    )

    with pytest.raises(
        TrajectoryParametrizationError,
        match="no usable URDF acceleration limit for joint 'left/a'",
    ):
        parametrizer.parametrize(_request())

    constructor.assert_not_called()


def test_cached_group_limits_follow_each_request_joint_order(
    mocker: MockerFixture,
) -> None:
    generated = SimpleNamespace(
        joint_names=["native_b", "native_a"],
        times=[0.0, 0.5],
        positions=[np.asarray([0.0, 0.1]), np.asarray([0.3, 0.4])],
        velocities=[np.asarray([0.0, 0.0]), np.asarray([0.6, 0.2])],
        accelerations=[np.asarray([0.0, 0.0]), np.asarray([1.2, 0.4])],
    )
    native = mocker.MagicMock()
    native.generate.return_value = generated
    constructor = mocker.patch(
        "dimos.manipulation.planning.trajectory_generator."
        "roboplan_toppra_parametrizer.roboplan_toppra.PathParameterizerTOPPRA",
        return_value=native,
    )
    parametrizer = RoboPlanTOPPRAParametrizer(
        _World(_model()),
        RoboPlanTOPPRAParametrizationConfig(
            velocity_scale=0.5,
            acceleration_scale=0.25,
        ),
    )

    canonical = parametrizer.parametrize(_request())
    reversed_order = parametrizer.parametrize(_request(("right/b", "left/a")))

    constructor.assert_called_once()
    assert canonical.velocity_limits == (0.5, 1.0)
    assert canonical.acceleration_limits == (1.0, 1.5)
    assert reversed_order.velocity_limits == (1.0, 0.5)
    assert reversed_order.acceleration_limits == (1.5, 1.0)
