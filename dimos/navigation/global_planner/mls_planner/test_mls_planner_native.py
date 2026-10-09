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

"""The navigation interface on the MLS planner wrapper, without starting the binary."""

from __future__ import annotations

from collections.abc import Callable, Generator
import math

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus
import pytest

from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import _resolve_single_ref
from dimos.core.module import ModuleBase
from dimos.core.stream import Stream, Transport
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.spec import NavigationInterfaceSpec, goal_id
from dimos.robot.unitree.unitree_skill_container import UnitreeSkillContainer
from dimos.spec.utils import spec_annotation_compliance


class _GoalTransport(Transport[PointStamped]):
    """Keeps every goal message the wrapper sends."""

    def __init__(self) -> None:
        self.sent: list[PointStamped] = []

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def broadcast(self, selfstream: Stream[PointStamped] | None, value: PointStamped) -> None:
        self.sent.append(value)

    def subscribe(
        self,
        callback: Callable[[PointStamped], object],
        selfstream: Stream[PointStamped] | None = None,
    ) -> Callable[[], None]:
        return lambda: None


@pytest.fixture()
def planner() -> Generator[tuple[MLSPlannerNative, list[PointStamped]], None, None]:
    transport = _GoalTransport()
    module = MLSPlannerNative()
    module.goal.transport = transport
    try:
        yield module, transport.sent
    finally:
        module._close_module()


def _status(goal_id: str, status: int) -> GoalStatus:
    return GoalStatus(goal_id=GoalID(id=goal_id), status=status)


def _latest(module: MLSPlannerNative) -> tuple[str, int] | None:
    status = module.get_status()
    return None if status is None else (status.goal_id.id, status.status)


def test_no_status_before_any_goal(
    planner: tuple[MLSPlannerNative, list[PointStamped]],
) -> None:
    module, _ = planner
    assert module.get_status() is None


def test_set_goal_sends_the_pose_position(
    planner: tuple[MLSPlannerNative, list[PointStamped]],
) -> None:
    module, sent = planner
    assert module.set_goal(PoseStamped(ts=5.0, frame_id="map", position=(1.0, 2.0, 3.0)))
    (goal,) = sent
    assert (goal.x, goal.y, goal.z, goal.ts, goal.frame_id) == (1.0, 2.0, 3.0, 5.0, "map")


def test_get_status_is_the_planners_latest_report(
    planner: tuple[MLSPlannerNative, list[PointStamped]],
) -> None:
    module, _ = planner
    report = _status("1", GoalStatus.ACTIVE)
    module._on_nav_status(report)
    assert module.get_status() is report


def test_a_new_goal_is_pending_under_its_id_until_the_planner_answers_it(
    planner: tuple[MLSPlannerNative, list[PointStamped]],
) -> None:
    module, _ = planner
    module._on_nav_status(_status("1", GoalStatus.SUCCEEDED))
    goal = PoseStamped(ts=5.25, position=(1.0, 0.0, 0.0))
    module.set_goal(goal)

    # The heartbeat still repeats the goal that came before.
    module._on_nav_status(_status("1", GoalStatus.SUCCEEDED))
    assert goal_id(goal) == "5.250000000"
    assert _latest(module) == ("5.250000000", GoalStatus.PENDING)

    module._on_nav_status(_status(goal_id(goal), GoalStatus.SUCCEEDED))
    assert _latest(module) == ("5.250000000", GoalStatus.SUCCEEDED)


def test_cancel_goal_sends_the_nan_point_and_reports_whether_a_goal_was_held(
    planner: tuple[MLSPlannerNative, list[PointStamped]],
) -> None:
    module, sent = planner
    assert not module.cancel_goal()

    module._on_nav_status(_status("1", GoalStatus.ACTIVE))
    assert module.cancel_goal()
    assert all(math.isnan(v) for v in (sent[-1].x, sent[-1].y, sent[-1].z))

    # The heartbeat still repeats the canceled goal.
    module._on_nav_status(_status("1", GoalStatus.ACTIVE))
    assert _latest(module) == ("1", GoalStatus.PREEMPTED)

    module._on_nav_status(_status("1", GoalStatus.PREEMPTED))
    module._on_nav_status(_status("2", GoalStatus.ACTIVE))
    assert _latest(module) == ("2", GoalStatus.ACTIVE)


def test_a_cancel_with_no_goal_held_keeps_the_last_status(
    planner: tuple[MLSPlannerNative, list[PointStamped]],
) -> None:
    module, sent = planner
    module._on_nav_status(_status("1", GoalStatus.SUCCEEDED))
    assert not module.cancel_goal()
    assert math.isnan(sent[-1].x)
    assert _latest(module) == ("1", GoalStatus.SUCCEEDED)


def test_cancel_goal_holds_no_goal_once_the_planner_aborted_it(
    planner: tuple[MLSPlannerNative, list[PointStamped]],
) -> None:
    module, _ = planner
    module._on_nav_status(_status("1", GoalStatus.ABORTED))
    assert not module.cancel_goal()


def test_the_planner_exposes_no_skills(
    planner: tuple[MLSPlannerNative, list[PointStamped]],
) -> None:
    module, _ = planner
    assert module.get_skills() == []


def test_the_planner_matches_the_navigation_spec_by_signature() -> None:
    assert spec_annotation_compliance(MLSPlannerNative, NavigationInterfaceSpec)


@pytest.mark.parametrize("skills", [NavigationSkillContainer, UnitreeSkillContainer])
def test_navigation_skills_resolve_to_the_mls_planner(skills: type[ModuleBase]) -> None:
    blueprint = autoconnect(MLSPlannerNative.blueprint(), skills.blueprint())
    atoms = {atom.module: atom for atom in blueprint.active_blueprints}
    consumer = atoms[skills]
    (ref,) = [ref for ref in consumer.module_refs if ref.name == "_navigation"]
    resolved = _resolve_single_ref(consumer, ref, ref.spec, blueprint, set())
    assert resolved == atoms[MLSPlannerNative].name
