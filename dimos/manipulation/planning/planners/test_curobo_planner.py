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

"""CPU tests for the CuRobo V2 planner adapter, without importing CuRobo or Torch."""

from contextlib import nullcontext
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.planners import curobo_planner
from dimos.manipulation.planning.planners.config import CartesianPathConfig, CuRoboPlannerConfig
from dimos.manipulation.planning.planners.curobo_planner import CuRoboPlanner
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.joint_space import (
    CoordinateTopology,
    JointCoordinate,
    JointSpace,
)
from dimos.manipulation.planning.trajectory_generator.config import (
    SimpleTrapezoidParametrizationConfig,
)
from dimos.manipulation.planning.trajectory_generator.simple_parametrizer import (
    SimpleTrapezoidParametrizer,
)
from dimos.msgs.sensor_msgs.JointState import JointState


class MockTensor:
    def __init__(self, data):
        self.data = np.asarray(data)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.data

    def item(self):
        return self.data.item()

    def any(self):
        return MockTensor(self.data.any())


class MockWorld:
    def __init__(self, native):
        self.native_context = Mock(side_effect=lambda: nullcontext(native))
        self.check_config_collision_free = Mock(return_value=True)
        self.is_ready = Mock(return_value=True)
        self.get_prepared_model = Mock(
            return_value=SimpleNamespace(
                joint_space=JointSpace(
                    tuple(
                        JointCoordinate(
                            name=name,
                            mechanism_type="revolute",
                            topology=CoordinateTopology.INTERVAL,
                            lower=-2.0,
                            upper=2.0,
                            max_velocity=3.0,
                            max_acceleration=5.0,
                        )
                        for name in ("a", "b")
                    )
                )
            )
        )


def _selection(*joint_lists):
    return PlanningGroupSelection.from_groups(
        tuple(
            PlanningGroup(id=f"group{i}", joint_names=names, base_link="base")
            for i, names in enumerate(joint_lists)
        )
    )


@pytest.fixture
def backend(monkeypatch):
    native_state = SimpleNamespace(
        from_position=Mock(
            side_effect=lambda position, joint_names: SimpleNamespace(
                position=position, joint_names=joint_names
            )
        )
    )
    monkeypatch.setitem(sys.modules, "curobo.types", SimpleNamespace(JointState=native_state))
    monkeypatch.setitem(
        sys.modules,
        "dimos.manipulation.planning.world.curobo_world",
        SimpleNamespace(CuRoboWorld=MockWorld),
    )
    trajectory = SimpleNamespace(
        joint_names=["a", "locked", "b"],
        position=MockTensor([[[[0.1, 0.0, 0.2], [0.3, 0.0, 0.5], [0.4, 0.0, 0.6]]]]),
        velocity=MockTensor([[[[0.0, 0.0, 0.0], [0.7, 0.0, 0.8], [0.0, 0.0, 0.0]]]]),
        dt=MockTensor([0.125]),
    )
    result = SimpleNamespace(
        success=MockTensor([True]), get_interpolated_plan=Mock(return_value=trajectory)
    )
    native = SimpleNamespace(
        joint_names=["b", "a"],
        device_cfg=SimpleNamespace(to_device=Mock(side_effect=MockTensor)),
        plan_cspace=Mock(return_value=result),
    )
    world = MockWorld(native)
    return SimpleNamespace(
        world=world,
        planner=CuRoboPlanner(world, CuRoboPlannerConfig()),
        native=native,
        trajectory=trajectory,
        result=result,
        start=JointState(name=["a", "b"], position=[0.1, 0.2]),
        goal=JointState(name=["b", "a"], position=[0.6, 0.4]),
    )


@pytest.mark.parametrize("speed_scale", [1.0, 0.5])
def test_native_joint_plan_preserves_timed_samples_and_materializes(backend, speed_scale):
    b = backend
    selection = _selection(("b", "a"))
    result = b.planner.plan_selected_joint_path(
        b.world,
        selection,
        b.start,
        b.goal,
        max_iterations=2,
        speed_scale=speed_scale,
    )
    assert result.status is PlanningStatus.SUCCESS, result.message
    assert result.timestamps == pytest.approx([0.0, 0.125 / speed_scale, 0.25 / speed_scale])
    assert [list(p.name) for p in result.path] == [["b", "a"]] * 3
    np.testing.assert_allclose(
        [list(p.position) for p in result.path], [[0.2, 0.1], [0.5, 0.3], [0.6, 0.4]]
    )
    assert list(result.path[1].velocity) == pytest.approx([0.8 * speed_scale, 0.7 * speed_scale])
    assert result.path_length > 0
    b.result.get_interpolated_plan.assert_called_once_with()
    b.world.native_context.assert_called_once_with()
    goal, start = b.native.plan_cspace.call_args.args
    assert goal.joint_names == start.joint_names == ["b", "a"]
    np.testing.assert_allclose(goal.position.numpy(), [[0.6, 0.4]])
    np.testing.assert_allclose(start.position.numpy(), [[0.2, 0.1]])
    assert b.native.plan_cspace.call_args.kwargs == {"max_attempts": 2, "enable_graph_attempt": 2}

    parametrizer = SimpleTrapezoidParametrizer(SimpleTrapezoidParametrizationConfig())
    plan = parametrizer.materialize_plan(b.world, selection, result, speed_scale=speed_scale)
    assert [p.time_from_start for p in plan.trajectory.points] == result.timestamps
    assert list(plan.trajectory.points[1].velocities) == list(result.path[1].velocity)


def test_full_model_unnamed_input_uses_canonical_model_order(backend):
    b = backend
    result = b.planner.plan_joint_path(b.world, JointState(position=[0.1, 0.2]), b.goal)
    assert result.is_success(), result.message
    assert list(result.path[0].name) == ["a", "b"]
    assert list(result.path[-1].position) == pytest.approx([0.4, 0.6])


@pytest.mark.parametrize("groups", [(("a",),), (("a",), ("b",))])
def test_partial_and_multi_group_selection_rejected(backend, groups):
    b = backend
    result = b.planner.plan_selected_joint_path(b.world, _selection(*groups), b.start, b.goal)
    assert result.status is PlanningStatus.UNSUPPORTED
    b.world.native_context.assert_not_called()


def test_cartesian_and_foreign_world_rejected_without_planning(backend):
    b = backend
    assert (
        b.planner.plan_cartesian_path(
            b.world,
            _selection(("a", "b")),
            b.start,
            {},
            CartesianPathConfig(),
        ).status
        is PlanningStatus.UNSUPPORTED
    )
    assert (
        b.planner.plan_joint_path(MockWorld(b.native), b.start, b.goal).status
        is PlanningStatus.UNSUPPORTED
    )
    b.world.native_context.assert_not_called()


@pytest.mark.parametrize(
    "endpoint, positions, status",
    [
        ("start", [0.1, float("nan")], PlanningStatus.INVALID_START),
        ("goal", [0.4, 3.0], PlanningStatus.INVALID_GOAL),
        ("start", [0.1], PlanningStatus.INVALID_START),
    ],
)
def test_invalid_input_before_natively_planning(backend, endpoint, positions, status):
    b = backend
    setattr(b, endpoint, JointState(position=positions))
    assert b.planner.plan_joint_path(b.world, b.start, b.goal).status is status
    b.world.native_context.assert_not_called()


@pytest.mark.parametrize(
    "checks, status",
    [
        ([False], PlanningStatus.COLLISION_AT_START),
        ([True, False], PlanningStatus.COLLISION_AT_GOAL),
    ],
)
def test_world_collision_checks_are_on_the_supplied_endpoints(backend, checks, status):
    b = backend
    b.world.check_config_collision_free.side_effect = checks
    result = b.planner.plan_joint_path(b.world, b.start, b.goal)
    assert result.status is status
    checked = [call.args[0] for call in b.world.check_config_collision_free.call_args_list]
    assert [list(state.name) for state in checked] == [["a", "b"]] * len(checks)
    assert [list(state.position) for state in checked] == [[0.1, 0.2], [0.4, 0.6]][: len(checks)]
    b.native.plan_cspace.assert_not_called()


@pytest.mark.parametrize("no_result", [True, False])
def test_native_failure_does_not_use_debug_trajectory_or_retry_elsewhere(backend, no_result):
    b = backend
    b.result.success = MockTensor([False])
    b.result.debug_info = {"trajectory_source": "linear"}
    if no_result:
        b.native.plan_cspace.return_value = None
    result = b.planner.plan_joint_path(b.world, b.start, b.goal)
    assert result.status is PlanningStatus.NO_SOLUTION
    assert not result.path
    b.result.get_interpolated_plan.assert_not_called()
    b.native.plan_cspace.assert_called_once()


@pytest.mark.parametrize(
    "field, replacement",
    [
        ("joint_names", ["a", "locked", "locked"]),
        ("dt", None),
        ("dt", MockTensor(float("inf"))),
        ("velocity", MockTensor(np.zeros((2, 3)))),
        ("position", MockTensor(np.zeros((2, 3, 3)))),
        ("position", MockTensor(np.full((3, 3), np.nan))),
        ("position", MockTensor([[0.1, 0.0, 0.2]])),
        ("position", MockTensor([[0.1, 0.0, 0.2], [0.3, 0.0, 0.5], [0.0, 0.0, 0.0]])),
    ],
)
def test_malformed_or_wrong_endpoint_trajectory_is_rejected(backend, field, replacement):
    b = backend
    setattr(b.trajectory, field, replacement)
    result = b.planner.plan_joint_path(b.world, b.start, b.goal)
    assert result.status is PlanningStatus.NO_SOLUTION
    assert not result.path
    assert result.timestamps is None


def test_native_success_after_deadline_is_not_returned(backend, monkeypatch):
    b = backend
    clock = iter([0.0, 0.1, 1.2, 1.2])
    monkeypatch.setattr(curobo_planner, "time", SimpleNamespace(monotonic=lambda: next(clock)))
    assert (
        b.planner.plan_joint_path(b.world, b.start, b.goal, timeout=1.0).status
        is PlanningStatus.TIMEOUT
    )
    b.native.plan_cspace.assert_called_once()
    b.result.get_interpolated_plan.assert_not_called()
