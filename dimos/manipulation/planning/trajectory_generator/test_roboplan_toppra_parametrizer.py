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
from dataclasses import replace
from itertools import pairwise
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from pytest_mock import MockerFixture

pytest.importorskip("roboplan.toppra")

from dimos.manipulation.planning.groups.models import (
    PlanningGroup,
    PlanningGroupDefinition,
    PlanningGroupSelection,
)
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.joint_space import (
    CoordinateTopology,
    JointCoordinate,
    JointSpace,
)
from dimos.manipulation.planning.spec.models import PlanningResult
from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.manipulation.planning.trajectory_generator.config import (
    RoboPlanTOPPRAParametrizationConfig,
)
from dimos.manipulation.planning.trajectory_generator.parametrizer import (
    TrajectoryParametrizationError,
)
from dimos.manipulation.planning.trajectory_generator.roboplan_toppra_parametrizer import (
    RoboPlanTOPPRAParametrizer,
)
from dimos.manipulation.planning.world.roboplan_model import RoboPlanGroup, RoboPlanModel
from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import PlanarBaseDefinition, RobotModel

pytestmark = pytest.mark.self_hosted


class _Scene:
    def __init__(self, *, unbounded_acceleration: bool = False) -> None:
        self.unbounded_acceleration = unbounded_acceleration

    def getVelocityLimitVectors(self, group_name: str) -> tuple[np.ndarray, np.ndarray]:
        assert group_name == "composite"
        return np.asarray([-2.0, -1.0]), np.asarray([2.0, 1.0])

    def getAccelerationLimitVectors(self, group_name: str) -> tuple[np.ndarray, np.ndarray]:
        assert group_name == "composite"
        maximum = np.finfo(np.float64).max if self.unbounded_acceleration else 4.0
        return np.asarray([-6.0, -maximum]), np.asarray([6.0, maximum])

    def getJointGroupInfo(self, group_name: str) -> SimpleNamespace:
        assert group_name == "composite"
        return SimpleNamespace(has_continuous_dofs=False)


class _World(RoboPlanWorld):
    def __init__(self, model: RoboPlanModel) -> None:
        self.model = model

    @contextmanager
    def parametrization_model(self):
        yield self.model

    def get_prepared_model(self):
        return SimpleNamespace(
            joint_space=JointSpace(
                (
                    JointCoordinate(
                        name="left/a",
                        mechanism_type="revolute",
                        topology=CoordinateTopology.INTERVAL,
                        lower=-1.0,
                        upper=1.0,
                        max_velocity=2.0,
                        max_acceleration=6.0,
                    ),
                    JointCoordinate(
                        name="right/b",
                        mechanism_type="revolute",
                        topology=CoordinateTopology.INTERVAL,
                        lower=-1.0,
                        upper=1.0,
                        max_velocity=1.0,
                        max_acceleration=4.0,
                    ),
                )
            )
        )


def _model(*, unbounded_acceleration: bool = False) -> RoboPlanModel:
    group = RoboPlanGroup(
        group_ids=("left_arm", "right_arm"),
        name="composite",
        native_names=("native_b", "native_a"),
        public_names=("right/b", "left/a"),
    )
    return RoboPlanModel(
        scene=_Scene(unbounded_acceleration=unbounded_acceleration),
        groups={frozenset(group.group_ids): group},
        all_group=group,
    )


def _selection_and_result(
    names: tuple[str, str] = ("left/a", "right/b"),
) -> tuple[PlanningGroupSelection, PlanningResult]:
    positions_by_name = {
        "left/a": (0.0, 0.3),
        "right/b": (0.1, 0.4),
    }
    groups_by_name = {
        "left/a": PlanningGroup(
            id="left_arm",
            joint_names=("left/a",),
            base_link="base",
        ),
        "right/b": PlanningGroup(
            id="right_arm",
            joint_names=("right/b",),
            base_link="base",
        ),
    }
    selection = PlanningGroupSelection.from_groups(tuple(groups_by_name[name] for name in names))
    result = PlanningResult(
        status=PlanningStatus.SUCCESS,
        path=[
            JointState(
                name=list(names),
                position=[positions_by_name[name][0] for name in names],
            ),
            JointState(
                name=list(names),
                position=[positions_by_name[name][1] for name in names],
            ),
        ],
    )
    return selection, result


def test_roboplan_parametrizer_maps_composite_order_and_native_output(
    mocker: MockerFixture,
) -> None:
    generated = SimpleNamespace(
        joint_names=["native_a", "native_b"],
        times=[0.0, 0.5],
        positions=[np.asarray([0.0, 0.1]), np.asarray([0.3, 0.4])],
        velocities=[np.asarray([0.0, 0.0]), np.asarray([0.2, 0.2])],
    )
    native = mocker.MagicMock()
    native.generate.return_value = generated
    constructor = mocker.patch(
        "dimos.manipulation.planning.trajectory_generator."
        "roboplan_toppra_parametrizer.roboplan_toppra.PathParameterizerTOPPRA",
        return_value=native,
    )
    parametrizer = RoboPlanTOPPRAParametrizer(
        RoboPlanTOPPRAParametrizationConfig(
            velocity_scale=0.5,
            acceleration_scale=0.25,
        ),
    )
    world = _World(_model())
    selection, planning_result = _selection_and_result()

    result = parametrizer.materialize_plan(
        world,
        selection,
        planning_result,
        speed_scale=0.5,
    )

    constructor.assert_called_once()
    native_path, options = native.generate.call_args.args
    assert native_path.joint_names == ["native_b", "native_a"]
    assert [row.tolist() for row in native_path.positions] == [
        [0.1, 0.0],
        [0.4, 0.3],
    ]
    assert options.velocity_scale == 0.25
    assert options.acceleration_scale == 0.125
    assert options.mode.name == "LinearBlend"
    assert options.max_blend_deviation == 0.0
    assert result.trajectory.joint_names == ["left/a", "right/b"]
    assert [point.positions for point in result.trajectory.points] == [
        [0.0, 0.1],
        [0.3, 0.4],
    ]
    assert [point.velocities for point in result.trajectory.points] == [
        [0.0, 0.0],
        [0.2, 0.2],
    ]
    assert [state.position for state in planning_result.path] == [
        [0.0, 0.1],
        [0.3, 0.4],
    ]


def test_roboplan_parametrizer_rejects_unbounded_scene_acceleration(
    mocker: MockerFixture,
) -> None:
    constructor = mocker.patch(
        "dimos.manipulation.planning.trajectory_generator."
        "roboplan_toppra_parametrizer.roboplan_toppra.PathParameterizerTOPPRA"
    )
    parametrizer = RoboPlanTOPPRAParametrizer(
        RoboPlanTOPPRAParametrizationConfig(),
    )
    selection, result = _selection_and_result()

    with pytest.raises(
        TrajectoryParametrizationError,
        match="no usable URDF acceleration limit for joint 'left/a'",
    ):
        parametrizer.materialize_plan(
            _World(_model(unbounded_acceleration=True)),
            selection,
            result,
        )

    constructor.assert_not_called()


def test_cached_group_preserves_each_request_joint_order(
    mocker: MockerFixture,
) -> None:
    generated = SimpleNamespace(
        joint_names=["native_b", "native_a"],
        times=[0.0, 0.5],
        positions=[np.asarray([0.1, 0.0]), np.asarray([0.4, 0.3])],
        velocities=[np.asarray([0.0, 0.0]), np.asarray([0.2, 0.4])],
    )
    native = mocker.MagicMock()
    native.generate.return_value = generated
    constructor = mocker.patch(
        "dimos.manipulation.planning.trajectory_generator."
        "roboplan_toppra_parametrizer.roboplan_toppra.PathParameterizerTOPPRA",
        return_value=native,
    )
    parametrizer = RoboPlanTOPPRAParametrizer(RoboPlanTOPPRAParametrizationConfig())
    world = _World(_model())
    canonical_selection, canonical_result = _selection_and_result()
    reversed_selection, reversed_result = _selection_and_result(("right/b", "left/a"))

    canonical = parametrizer.materialize_plan(
        world,
        canonical_selection,
        canonical_result,
    )
    reversed_order = parametrizer.materialize_plan(
        world,
        reversed_selection,
        reversed_result,
    )

    constructor.assert_called_once()
    assert canonical.trajectory.joint_names == ["left/a", "right/b"]
    assert reversed_order.trajectory.joint_names == ["right/b", "left/a"]


def test_roboplan_toppra_parametrizes_unbounded_planar_base(tmp_path: Path) -> None:
    model_path = tmp_path / "planar-base.urdf"
    model_path.write_text('<robot name="planar-base" version="1.2"><link name="body"/></robot>')
    planar_base = PlanarBaseDefinition(
        velocity_limits=(1.0, 1.0, 2.0),
        acceleration_limits=(2.0, 2.0, 4.0),
    )
    config = RobotModelConfig(
        model=RobotModel.from_file(model_path).with_planar_base(planar_base),
        joint_names=list(planar_base.joint_names),
        base_link=planar_base.root_link,
        planning_groups=[
            PlanningGroupDefinition(
                "moving_base",
                planar_base.joint_names,
                planar_base.root_link,
            )
        ],
    )
    world = RoboPlanWorld()
    world.load_model(prepare_robot_model(config))
    world.finalize()
    selection = PlanningGroupRegistry((config,)).select(("moving_base",))
    start = [0.0, 0.0, math.pi - 0.1]
    goal = [6.0, -6.0, math.pi + 0.1]
    result = PlanningResult(
        status=PlanningStatus.SUCCESS,
        path=[
            JointState(name=list(selection.joint_names), position=start),
            JointState(name=list(selection.joint_names), position=goal),
        ],
    )

    plan = RoboPlanTOPPRAParametrizer(RoboPlanTOPPRAParametrizationConfig()).materialize_plan(
        world, selection, result
    )

    positions = [point.positions for point in plan.trajectory.points]
    assert positions[0] == pytest.approx(start)
    assert positions[-1] == pytest.approx(goal)
    assert max(abs(current[2] - previous[2]) for previous, current in pairwise(positions)) < math.pi


def test_roboplan_parametrizer_rejects_incompatible_world(
    mocker: MockerFixture,
) -> None:
    selection, result = _selection_and_result()

    with pytest.raises(
        TrajectoryParametrizationError,
        match="RoboPlan TOPP-RA requires RoboPlanWorld",
    ):
        RoboPlanTOPPRAParametrizer(RoboPlanTOPPRAParametrizationConfig()).materialize_plan(
            mocker.MagicMock(), selection, result
        )


def test_roboplan_parametrizer_reports_missing_generated_group() -> None:
    selection, result = _selection_and_result()
    model = replace(_model(), groups={})

    with pytest.raises(
        TrajectoryParametrizationError,
        match=r"RoboPlan has no generated group for \['left_arm', 'right_arm'\]",
    ):
        RoboPlanTOPPRAParametrizer(RoboPlanTOPPRAParametrizationConfig()).materialize_plan(
            _World(model), selection, result
        )


def test_roboplan_parametrizer_rejects_group_with_different_joints() -> None:
    selection, result = _selection_and_result()
    model = _model()
    mismatched_group = replace(model.all_group, public_names=("left/a", "right/other"))
    model = replace(
        model,
        groups={frozenset(mismatched_group.group_ids): mismatched_group},
        all_group=mismatched_group,
    )

    with pytest.raises(
        TrajectoryParametrizationError,
        match="does not match selected joints",
    ):
        RoboPlanTOPPRAParametrizer(RoboPlanTOPPRAParametrizationConfig()).materialize_plan(
            _World(model), selection, result
        )


def test_roboplan_parametrizer_rejects_limit_vector_with_wrong_size(
    mocker: MockerFixture,
) -> None:
    selection, result = _selection_and_result()
    model = _model()
    mocker.patch.object(
        model.scene,
        "getVelocityLimitVectors",
        return_value=([-1.0], [1.0]),
    )

    with pytest.raises(
        TrajectoryParametrizationError,
        match="velocity limits do not match group 'composite'",
    ):
        RoboPlanTOPPRAParametrizer(RoboPlanTOPPRAParametrizationConfig()).materialize_plan(
            _World(model), selection, result
        )


def test_roboplan_parametrizer_wraps_native_generation_error(
    mocker: MockerFixture,
) -> None:
    native = mocker.MagicMock()
    native.generate.side_effect = RuntimeError("native failure")
    mocker.patch(
        "dimos.manipulation.planning.trajectory_generator."
        "roboplan_toppra_parametrizer.roboplan_toppra.PathParameterizerTOPPRA",
        return_value=native,
    )
    selection, result = _selection_and_result()

    with pytest.raises(
        TrajectoryParametrizationError,
        match="RoboPlan TOPP-RA parametrization failed: native failure",
    ) as error:
        RoboPlanTOPPRAParametrizer(RoboPlanTOPPRAParametrizationConfig()).materialize_plan(
            _World(_model()), selection, result
        )

    assert isinstance(error.value.__cause__, RuntimeError)


@pytest.mark.parametrize(
    ("generated", "message"),
    [
        (
            SimpleNamespace(
                joint_names=["native_a", "unexpected"],
                times=[0.0],
                positions=[np.asarray([0.0, 0.1])],
                velocities=[np.asarray([0.0, 0.0])],
            ),
            "returned unexpected joint names",
        ),
        (
            SimpleNamespace(
                joint_names=["native_a", "native_b"],
                times=[0.0],
                positions=[np.asarray([0.0, 0.1])],
                velocities=[],
            ),
            "returned inconsistent trajectory fields",
        ),
    ],
)
def test_roboplan_parametrizer_rejects_malformed_native_trajectory(
    mocker: MockerFixture,
    generated: SimpleNamespace,
    message: str,
) -> None:
    native = mocker.MagicMock()
    native.generate.return_value = generated
    mocker.patch(
        "dimos.manipulation.planning.trajectory_generator."
        "roboplan_toppra_parametrizer.roboplan_toppra.PathParameterizerTOPPRA",
        return_value=native,
    )
    selection, result = _selection_and_result()

    with pytest.raises(TrajectoryParametrizationError, match=message):
        RoboPlanTOPPRAParametrizer(RoboPlanTOPPRAParametrizationConfig()).materialize_plan(
            _World(_model()), selection, result
        )
