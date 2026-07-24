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

from concurrent.futures import ThreadPoolExecutor
from threading import Event
from typing import Any, cast
from unittest.mock import MagicMock, call

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.replanning_a_star import global_planner as planner_module
from dimos.navigation.replanning_a_star.global_planner import GlobalPlanner

TEST_TIMEOUT = 5.0


@pytest.fixture()
def planner(mocker: MockerFixture) -> GlobalPlanner:
    mocker.patch.object(planner_module, "NavigationMap", autospec=True)
    mocker.patch.object(planner_module, "LocalPlanner", autospec=True)
    mocker.patch.object(planner_module, "PositionTracker", autospec=True)
    mocker.patch.object(planner_module, "ReplanLimiter", autospec=True)
    return GlobalPlanner(GlobalConfig())


def test_stop_marks_shutdown_before_cleanup(planner: GlobalPlanner, mocker: MockerFixture) -> None:
    planner._thread = None
    planner._disposables = MagicMock()
    cancel_goal = mocker.patch.object(planner, "cancel_goal")
    shutdown_was_visible: list[bool] = []

    def observe_shutdown() -> None:
        shutdown_was_visible.append(
            planner._stop_planner.is_set() and planner._replan_event.is_set()
        )

    planner._disposables.dispose.side_effect = observe_shutdown
    lifecycle = MagicMock()
    lifecycle.attach_mock(planner._disposables.dispose, "dispose")
    lifecycle.attach_mock(cast("MagicMock", planner._local_planner.stop), "local_stop")
    lifecycle.attach_mock(cancel_goal, "cancel")

    planner.stop()

    assert shutdown_was_visible == [True]
    assert lifecycle.mock_calls == [call.dispose(), call.local_stop(), call.cancel()]


def test_stop_cancels_goal_when_local_stop_fails(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._thread = None
    planner._disposables = MagicMock()
    cast("MagicMock", planner._local_planner.stop).side_effect = RuntimeError("stop failed")
    cancel_goal = mocker.patch.object(planner, "cancel_goal")

    with pytest.raises(RuntimeError, match="stop failed"):
        planner.stop()

    cancel_goal.assert_called_once_with()


def test_stop_keeps_live_monitor_thread_handle(planner: GlobalPlanner) -> None:
    planner._disposables = MagicMock()
    thread = MagicMock()
    thread.is_alive.return_value = True
    planner._thread = thread

    planner.stop()

    thread.join.assert_called_once_with(DEFAULT_THREAD_JOIN_TIMEOUT)
    assert planner._thread is thread


def test_start_rejects_live_monitor_thread(planner: GlobalPlanner) -> None:
    thread = MagicMock()
    thread.is_alive.return_value = True
    planner._thread = thread

    with pytest.raises(RuntimeError, match="monitor thread has already been started"):
        planner.start()

    cast("MagicMock", planner._local_planner.start).assert_not_called()


def test_start_waits_for_stop_and_rejects_restart(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._disposables = MagicMock()
    planner._local_planner = MagicMock()
    old_thread = MagicMock()
    old_thread.is_alive.return_value = False
    thread_type = mocker.patch.object(planner_module, "Thread")
    join_started = Event()
    release_join = Event()
    start_attempted = Event()

    def pause_join(_timeout: float) -> None:
        join_started.set()
        if not release_join.wait(timeout=TEST_TIMEOUT):
            raise TimeoutError("test did not release monitor join")

    def start_planner() -> None:
        start_attempted.set()
        planner.start()

    old_thread.join.side_effect = pause_join
    planner._thread = old_thread

    try:
        with ThreadPoolExecutor(max_workers=2) as executor:
            stop_future = executor.submit(planner.stop)
            assert join_started.wait(timeout=TEST_TIMEOUT)
            start_future = executor.submit(start_planner)
            assert start_attempted.wait(timeout=TEST_TIMEOUT)
            thread_type.assert_not_called()
            release_join.set()
            stop_future.result(timeout=TEST_TIMEOUT)
            with pytest.raises(RuntimeError, match="cannot be restarted"):
                start_future.result(timeout=TEST_TIMEOUT)
    finally:
        release_join.set()

    assert planner._thread is None
    thread_type.assert_not_called()


def test_shutdown_rejects_goal_and_completion(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._stop_planner.set()
    plan_path = mocker.patch.object(planner, "_plan_path")

    planner.handle_goal_request(MagicMock())
    planner._on_stopped_navigating("error")

    assert planner._current_goal is None
    assert planner._replan_reason is None
    assert not planner._replan_event.is_set()
    plan_path.assert_not_called()


def test_replan_without_active_goal_is_noop(planner: GlobalPlanner) -> None:
    planner._current_odom = MagicMock()

    planner._replan_path()

    cast("MagicMock", planner._replan_limiter.get_attempt).assert_not_called()


def test_computed_path_for_replaced_goal_is_discarded(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._current_odom = MagicMock()
    planner._current_goal = MagicMock()
    planner._goal_epoch = 1
    planner.path = MagicMock()
    planning_started = Event()
    release_planning = Event()
    resampled_path = MagicMock()
    mocker.patch.object(planner, "_find_safe_goal", return_value=MagicMock())

    def finish_after_replacement(*_args: Any) -> MagicMock:
        planning_started.set()
        assert release_planning.wait(timeout=TEST_TIMEOUT)
        return MagicMock()

    mocker.patch.object(planner, "_find_wide_path", side_effect=finish_after_replacement)
    mocker.patch.object(planner_module, "smooth_resample_path", return_value=resampled_path)

    with ThreadPoolExecutor(max_workers=1) as executor:
        plan_future = executor.submit(planner._plan_path, 1)
        assert planning_started.wait(timeout=TEST_TIMEOUT)
        planner.path.reset_mock()
        cast("MagicMock", planner._local_planner.start_planning).reset_mock()
        replacement_plan = mocker.patch.object(planner, "_plan_path")
        planner.handle_goal_request(MagicMock())
        release_planning.set()
        plan_future.result(timeout=TEST_TIMEOUT)

    replacement_plan.assert_called_once()
    planner.path.on_next.assert_not_called()
    cast("MagicMock", planner._local_planner.start_planning).assert_not_called()


def test_stop_during_path_publication_prevents_local_start(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._current_odom = MagicMock()
    planner._current_goal = MagicMock()
    planner._goal_epoch = 1
    planner._thread = None
    planner._disposables = MagicMock()
    planner.path = MagicMock()
    publication_started = Event()
    release_publication = Event()
    publication_released = []
    resampled_path = MagicMock()
    mocker.patch.object(planner, "_find_safe_goal", return_value=MagicMock())
    mocker.patch.object(planner, "_find_wide_path", return_value=MagicMock())
    mocker.patch.object(planner_module, "smooth_resample_path", return_value=resampled_path)

    def pause_publication(path: Any) -> None:
        if path is resampled_path:
            publication_started.set()
            publication_released.append(release_publication.wait(timeout=TEST_TIMEOUT))

    planner.path.on_next.side_effect = pause_publication

    try:
        with ThreadPoolExecutor(max_workers=2) as executor:
            plan_future = executor.submit(planner._plan_path, 1)
            assert publication_started.wait(timeout=TEST_TIMEOUT)
            start_planning = cast("MagicMock", planner._local_planner.start_planning)
            start_planning.reset_mock()
            stop_future = executor.submit(planner.stop)
            assert planner._stop_planner.wait(timeout=TEST_TIMEOUT)
            assert not stop_future.done()
            release_publication.set()
            plan_future.result(timeout=TEST_TIMEOUT)
            stop_future.result(timeout=TEST_TIMEOUT)
    finally:
        release_publication.set()

    start_planning.assert_not_called()
    assert publication_released == [True]


def test_path_observer_cancel_prevents_local_start(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._current_odom = MagicMock()
    planner._current_goal = MagicMock()
    planner._goal_epoch = 1
    planner.path = MagicMock()
    resampled_path = MagicMock()
    mocker.patch.object(planner, "_find_safe_goal", return_value=MagicMock())
    mocker.patch.object(planner, "_find_wide_path", return_value=MagicMock())
    mocker.patch.object(planner_module, "smooth_resample_path", return_value=resampled_path)

    def cancel_published_path(path: Any) -> None:
        if path is resampled_path:
            planner.cancel_goal()

    planner.path.on_next.side_effect = cancel_published_path

    planner._plan_path(1)

    cast("MagicMock", planner._local_planner.start_planning).assert_not_called()


def test_monitor_arrival_does_not_cancel_replacement_goal(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    old_goal = MagicMock()
    replacement_goal = MagicMock()
    planner._current_goal = old_goal
    planner._current_odom = MagicMock()
    planner._goal_epoch = 1
    planner.goal_reached = MagicMock()
    arrival_check_started = Event()
    release_arrival_check = Event()
    mocker.patch.object(planner, "_plan_path")
    mocker.patch.object(planner_module, "angle_diff", return_value=0.0)

    def distance_after_replacement(_position: Any) -> float:
        arrival_check_started.set()
        assert release_arrival_check.wait(timeout=TEST_TIMEOUT)
        return 0.0

    wait_count = 0

    def run_one_monitor_iteration(*, timeout: float) -> bool:
        nonlocal wait_count
        wait_count += 1
        if wait_count > 1:
            planner._stop_planner.set()
        return False

    old_goal.position.distance.side_effect = distance_after_replacement
    mocker.patch.object(planner._replan_event, "wait", side_effect=run_one_monitor_iteration)

    with ThreadPoolExecutor(max_workers=1) as executor:
        monitor_future = executor.submit(planner._thread_entrypoint)
        assert arrival_check_started.wait(timeout=TEST_TIMEOUT)
        planner.handle_goal_request(replacement_goal)
        release_arrival_check.set()
        monitor_future.result(timeout=TEST_TIMEOUT)

    assert planner._current_goal is replacement_goal
    planner.goal_reached.on_next.assert_not_called()


def test_find_wide_path_with_start_inside_inflation() -> None:
    """A wall observed at the last moment can be so close that its inflation
    covers the robot's own cell (the robot drove there before the costmap
    caught up). Planning must still find a way out instead of failing."""

    resolution = 0.05
    grid = np.zeros((60, 60), dtype=np.int8)
    grid[20:40, 30] = 100  # wall at x=1.5m spanning y=1.0..2.0m
    costmap = OccupancyGrid(grid=grid, resolution=resolution, origin=Pose(), frame_id="world")

    planner = GlobalPlanner(GlobalConfig())
    planner.handle_global_costmap(costmap)

    # 7 cm in front of the wall: within the inflation radius
    # (robot_width * 1.1 / 2 = 0.165m), so the start cell is engulfed.
    robot = Vector3(1.43, 1.5, 0)
    # On the other side of the wall; the path must round a wall end.
    goal = Vector3(2.75, 1.5, 0)

    path = planner._find_wide_path(goal, robot)

    assert path is not None
    assert len(path.poses) > 0
