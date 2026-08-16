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
from threading import Event, Thread
from typing import Any, cast
from unittest.mock import ANY, MagicMock, call

import numpy as np
import pytest
from pytest_mock import MockerFixture
from reactivex import Subject

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.replanning_a_star import global_planner as planner_module
from dimos.navigation.replanning_a_star.global_planner import GlobalPlanner
from dimos.navigation.replanning_a_star.local_planner import PlanStopEvent

TEST_TIMEOUT: float = 5.0


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
    cancel_goal = mocker.patch.object(planner, "_cancel_goal")
    shutdown_was_visible: list[bool] = []

    def observe_shutdown() -> None:
        shutdown_was_visible.append(
            planner._stop_planner.is_set() and planner._replan_event.is_set()
        )

    planner._disposables.dispose.side_effect = observe_shutdown
    lifecycle = MagicMock()
    lifecycle.attach_mock(planner._disposables.dispose, "dispose")
    lifecycle.attach_mock(cancel_goal, "cancel")

    planner.stop()

    assert shutdown_was_visible == [True]
    assert lifecycle.mock_calls == [
        call.dispose(),
        call.cancel(
            allow_during_shutdown=True,
            before_notifications=ANY,
        ),
    ]


def test_stop_joins_monitor_when_goal_cleanup_fails(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._disposables = MagicMock()
    thread = MagicMock()
    thread.is_alive.return_value = False
    planner._thread = thread
    mocker.patch.object(planner, "_cancel_goal", side_effect=RuntimeError("cleanup failed"))

    with pytest.raises(RuntimeError, match="cleanup failed"):
        planner.stop()

    thread.join.assert_called_once_with(DEFAULT_THREAD_JOIN_TIMEOUT)
    assert planner._thread is None


def test_stop_keeps_live_monitor_thread_handle(planner: GlobalPlanner) -> None:
    planner._disposables = MagicMock()
    thread = MagicMock()
    thread.is_alive.return_value = True
    planner._thread = thread

    planner.stop()

    thread.join.assert_called_once_with(DEFAULT_THREAD_JOIN_TIMEOUT)
    assert planner._thread is thread


def test_concurrent_stop_waits_for_critical_cleanup(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._thread = None
    planner._disposables = MagicMock()
    planner._current_goal = MagicMock()
    cleanup_started = Event()
    release_cleanup = Event()
    second_started = Event()
    second_returned = Event()

    def block_cleanup() -> None:
        cleanup_started.set()
        if not release_cleanup.wait(timeout=TEST_TIMEOUT):
            raise TimeoutError("test did not release shutdown cleanup")

    def stop_again() -> None:
        second_started.set()
        planner.stop()
        second_returned.set()

    cast("MagicMock", planner._local_planner.publish_stop_command).side_effect = block_cleanup

    try:
        with ThreadPoolExecutor(max_workers=2) as executor:
            first_stop = executor.submit(planner.stop)
            assert cleanup_started.wait(timeout=TEST_TIMEOUT)
            second_stop = executor.submit(stop_again)
            assert second_started.wait(timeout=TEST_TIMEOUT)
            assert not second_returned.wait(timeout=0.05)
            release_cleanup.set()
            first_stop.result(timeout=TEST_TIMEOUT)
            second_stop.result(timeout=TEST_TIMEOUT)
    finally:
        release_cleanup.set()

    assert second_returned.is_set()
    assert planner._stop_complete.is_set()


def test_reentrant_stop_does_not_wait_for_itself(planner: GlobalPlanner) -> None:
    planner._thread = None
    planner._disposables = MagicMock()
    planner._disposables.dispose.side_effect = planner.stop

    planner.stop()

    planner._disposables.dispose.assert_called_once_with()
    assert planner._stop_complete.is_set()


def test_stop_returns_for_preexisting_shutdown_flag(planner: GlobalPlanner) -> None:
    planner._disposables = MagicMock()
    planner._stop_planner.set()

    planner.stop()

    planner._disposables.dispose.assert_not_called()
    assert planner._stop_complete.is_set()


def test_failed_owner_stop_releases_concurrent_waiter(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._thread = None
    planner._disposables = MagicMock()
    planner._current_goal = MagicMock()
    cleanup_started = Event()
    release_cleanup = Event()
    second_returned = Event()

    def fail_cleanup() -> None:
        cleanup_started.set()
        if not release_cleanup.wait(timeout=TEST_TIMEOUT):
            raise TimeoutError("test did not release failing shutdown cleanup")
        raise RuntimeError("cleanup failed")

    def stop_again() -> None:
        planner.stop()
        second_returned.set()

    cast("MagicMock", planner._local_planner.publish_stop_command).side_effect = fail_cleanup

    try:
        with ThreadPoolExecutor(max_workers=2) as executor:
            first_stop = executor.submit(planner.stop)
            assert cleanup_started.wait(timeout=TEST_TIMEOUT)
            second_stop = executor.submit(stop_again)
            assert not second_returned.wait(timeout=0.05)
            release_cleanup.set()
            with pytest.raises(RuntimeError, match="cleanup failed"):
                first_stop.result(timeout=TEST_TIMEOUT)
            second_stop.result(timeout=TEST_TIMEOUT)
    finally:
        release_cleanup.set()

    assert second_returned.is_set()
    assert planner._stop_complete.is_set()


def test_concurrent_stop_wait_has_timeout(planner: GlobalPlanner, mocker: MockerFixture) -> None:
    planner._stop_planner.set()
    planner._stop_complete.clear()
    planner._stop_owner = Thread()
    completion_wait = mocker.patch.object(planner._stop_complete, "wait", return_value=False)
    log_error = mocker.patch.object(planner_module.logger, "error")

    planner.stop()

    completion_wait.assert_called_once_with(DEFAULT_THREAD_JOIN_TIMEOUT)
    log_error.assert_called_once_with("GlobalPlanner shutdown did not complete in time.")


def test_concurrent_stop_can_return_before_public_shutdown_notification(
    planner: GlobalPlanner,
) -> None:
    planner._current_goal = MagicMock()
    planner._disposables = MagicMock()
    notification_started = Event()
    release_notification = Event()
    planner.path = MagicMock()
    thread = MagicMock()
    thread.is_alive.return_value = False
    planner._thread = thread

    def block_notification(_path: Any) -> None:
        notification_started.set()
        if not release_notification.wait(timeout=TEST_TIMEOUT):
            raise TimeoutError("test did not release shutdown notification")

    planner.path.on_next.side_effect = block_notification

    try:
        with ThreadPoolExecutor(max_workers=2) as executor:
            first_stop = executor.submit(planner.stop)
            assert notification_started.wait(timeout=TEST_TIMEOUT)
            second_stop = executor.submit(planner.stop)
            second_stop.result(timeout=TEST_TIMEOUT)
            assert planner._stop_complete.is_set()
            cast("MagicMock", planner._local_planner.publish_stop_command).assert_called_once_with()
            thread.join.assert_called_once_with(DEFAULT_THREAD_JOIN_TIMEOUT)
            release_notification.set()
            first_stop.result(timeout=TEST_TIMEOUT)
    finally:
        release_notification.set()


def test_start_rejects_live_monitor_thread(planner: GlobalPlanner) -> None:
    thread = MagicMock()
    thread.is_alive.return_value = True
    planner._thread = thread

    with pytest.raises(RuntimeError, match="monitor thread has already been started"):
        planner.start()

    cast("MagicMock", planner._local_planner.start).assert_not_called()


def test_start_rejects_restart_after_stop(planner: GlobalPlanner) -> None:
    planner._stop_planner.set()

    with pytest.raises(RuntimeError, match="cannot be restarted"):
        planner.start()

    cast("MagicMock", planner._local_planner.start).assert_not_called()


def test_start_subscribes_to_tagged_completion_and_starts_monitor(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    thread_type = mocker.patch.object(planner_module, "Thread", autospec=True)

    planner.start()
    try:
        cast("MagicMock", planner._local_planner.start).assert_called_once_with()
        cast("MagicMock", planner._local_planner.plan_stopped.subscribe).assert_called_once_with(
            planner._on_stopped_navigating
        )
        thread_type.assert_called_once_with(target=planner._thread_entrypoint, daemon=True)
        thread_type.return_value.start.assert_called_once_with()
    finally:
        planner.stop()


def test_shutdown_rejects_goal_and_completion(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._stop_planner.set()
    plan_path = mocker.patch.object(planner, "_plan_path")

    planner.handle_goal_request(MagicMock())
    planner._on_stopped_navigating(PlanStopEvent("error", (1, 1)))

    assert planner._current_goal is None
    assert planner._replan_reason is None
    assert not planner._replan_event.is_set()
    plan_path.assert_not_called()


def test_stale_completion_does_not_replace_queued_current_completion(
    planner: GlobalPlanner,
) -> None:
    planner._goal_epoch = 2
    planner._plan_epoch = 4
    current_completion = PlanStopEvent("arrived", (2, 4))

    planner._on_stopped_navigating(current_completion)
    planner._on_stopped_navigating(PlanStopEvent("obstacle_found", (1, 3)))

    assert planner._replan_reason is current_completion
    assert planner._replan_event.is_set()


def test_shutdown_rejects_cancel_before_admission(planner: GlobalPlanner) -> None:
    planner._current_goal = MagicMock()
    planner.path = MagicMock()
    planner.goal_reached = MagicMock()
    planner._stop_planner.set()

    cancelled = planner._cancel_goal(arrived=True)

    assert not cancelled
    cast("MagicMock", planner._local_planner.deactivate_planning).assert_not_called()
    cast("MagicMock", planner._local_planner.publish_stop_command).assert_not_called()
    planner.path.on_next.assert_not_called()
    planner.goal_reached.on_next.assert_not_called()


def test_stop_without_active_goal_only_stops_motion(planner: GlobalPlanner) -> None:
    planner._current_goal = None
    planner._goal_reached = True
    planner._goal_epoch = 3
    planner._plan_epoch = 5
    planner._thread = None
    planner._disposables = MagicMock()
    planner.path = MagicMock()
    planner.goal_reached = MagicMock()

    planner.stop()

    cast("MagicMock", planner._local_planner.deactivate_planning).assert_called_once_with()
    cast("MagicMock", planner._local_planner.publish_stop_command).assert_called_once_with()
    planner.path.on_next.assert_not_called()
    planner.goal_reached.on_next.assert_not_called()
    assert planner._goal_reached
    assert (planner._goal_epoch, planner._plan_epoch) == (3, 5)


def test_stop_suppresses_notifications_from_admitted_arrival(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._current_goal = MagicMock()
    planner._goal_epoch = 1
    planner._thread = None
    planner._disposables = MagicMock()
    planner.path = MagicMock()
    planner.goal_reached = MagicMock()
    stop_command_started = Event()
    release_stop_command = Event()
    stop_started = Event()
    calls = 0
    goal_notifications: list[tuple[bool, bool]] = []

    def pause_first_stop_command() -> None:
        nonlocal calls
        calls += 1
        if calls == 1:
            stop_command_started.set()
            if not release_stop_command.wait(timeout=TEST_TIMEOUT):
                raise TimeoutError("test did not release stop command publication")

    def record_goal(msg: Any) -> None:
        goal_notifications.append((planner._stop_planner.is_set(), msg.data))

    cast(
        "MagicMock", planner._local_planner.publish_stop_command
    ).side_effect = pause_first_stop_command
    planner.goal_reached.on_next.side_effect = record_goal

    def stop_planner() -> None:
        stop_started.set()
        planner.stop()

    try:
        with ThreadPoolExecutor(max_workers=2) as executor:
            arrival_future = executor.submit(planner._cancel_goal, arrived=True)
            assert stop_command_started.wait(timeout=TEST_TIMEOUT)
            stop_future = executor.submit(stop_planner)
            assert stop_started.wait(timeout=TEST_TIMEOUT)
            assert not planner._stop_planner.wait(timeout=0.05)
            release_stop_command.set()
            arrival_future.result(timeout=TEST_TIMEOUT)
            stop_future.result(timeout=TEST_TIMEOUT)
    finally:
        release_stop_command.set()

    assert (True, True) not in goal_notifications
    planner.path.on_next.assert_called_once()


def test_position_tracker_failure_still_stops_motion(
    planner: GlobalPlanner,
) -> None:
    planner._current_goal = MagicMock()
    planner.path = MagicMock()
    planner.goal_reached = MagicMock()
    cast("MagicMock", planner._position_tracker.reset_data).side_effect = RuntimeError(
        "reset failed"
    )

    with pytest.raises(RuntimeError, match="reset failed"):
        planner.cancel_goal()

    cast("MagicMock", planner._local_planner.deactivate_planning).assert_called_once_with()
    cast("MagicMock", planner._local_planner.publish_stop_command).assert_called_once_with()


def test_deactivation_failure_still_publishes_stop(
    planner: GlobalPlanner,
) -> None:
    planner._current_goal = MagicMock()
    planner.path = MagicMock()
    planner.goal_reached = MagicMock()
    cast("MagicMock", planner._local_planner.deactivate_planning).side_effect = RuntimeError(
        "deactivation failed"
    )

    with pytest.raises(RuntimeError, match="deactivation failed"):
        planner.cancel_goal()

    cast("MagicMock", planner._local_planner.publish_stop_command).assert_called_once_with()


def test_replacement_waits_for_stop_command_publication(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._current_goal = MagicMock()
    planner.path = MagicMock()
    planner.goal_reached = MagicMock()
    publish_started = Event()
    release_publish = Event()
    replacement_started = Event()
    plan_path_called = Event()
    plan_path = mocker.patch.object(
        planner,
        "_plan_path",
        side_effect=lambda _goal_epoch: plan_path_called.set(),
    )

    def block_stop_command() -> None:
        publish_started.set()
        if not release_publish.wait(timeout=TEST_TIMEOUT):
            raise TimeoutError("test did not release stop-command publication")

    def request_replacement() -> None:
        replacement_started.set()
        planner.handle_goal_request(MagicMock())

    cast("MagicMock", planner._local_planner.publish_stop_command).side_effect = block_stop_command

    try:
        with ThreadPoolExecutor(max_workers=2) as executor:
            cancel_future = executor.submit(planner.cancel_goal)
            assert publish_started.wait(timeout=TEST_TIMEOUT)
            replacement_future = executor.submit(request_replacement)
            assert replacement_started.wait(timeout=TEST_TIMEOUT)
            assert not plan_path_called.wait(timeout=0.05)
            release_publish.set()
            cancel_future.result(timeout=TEST_TIMEOUT)
            replacement_future.result(timeout=TEST_TIMEOUT)
    finally:
        release_publish.set()

    plan_path.assert_called_once()


def test_completion_uses_the_observed_goal_epochs(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._goal_epoch = 3
    planner._plan_epoch = 5
    cancel_goal = mocker.patch.object(planner, "_cancel_goal")

    planner._handle_stop_message(PlanStopEvent("arrived", (3, 5)))

    cancel_goal.assert_called_once_with(
        expected_goal_epoch=3,
        expected_plan_epoch=5,
        arrived=True,
    )


def test_queued_completion_for_old_plan_does_not_cancel_replacement(
    planner: GlobalPlanner,
) -> None:
    replacement_goal = MagicMock()
    planner._current_goal = replacement_goal
    planner._goal_epoch = 2
    planner._plan_epoch = 4
    planner.path = MagicMock()
    planner.goal_reached = MagicMock()

    planner._handle_stop_message(PlanStopEvent("arrived", (1, 3)))

    assert planner._current_goal is replacement_goal
    cast("MagicMock", planner._local_planner.deactivate_planning).assert_not_called()
    cast("MagicMock", planner._local_planner.publish_stop_command).assert_not_called()
    planner.path.on_next.assert_not_called()
    planner.goal_reached.on_next.assert_not_called()


def test_arrival_completion_does_not_cancel_replacement_from_path_observer(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    old_goal = MagicMock()
    replacement_goal = MagicMock()
    planner._current_goal = old_goal
    planner._goal_epoch = 1
    planner.path = MagicMock()
    planner.goal_reached = MagicMock()
    plan_path = mocker.patch.object(planner, "_plan_path")

    def replace_goal(_path: Any) -> None:
        planner.handle_goal_request(replacement_goal)

    planner.path.on_next.side_effect = replace_goal

    planner._handle_stop_message(PlanStopEvent("arrived", (1, 0)))

    assert planner._current_goal is replacement_goal
    plan_path.assert_called_once_with(planner._goal_epoch)
    planner.goal_reached.on_next.assert_not_called()


def test_replan_without_active_goal_is_noop(planner: GlobalPlanner) -> None:
    planner._current_odom = MagicMock()

    planner._replan_path()

    cast("MagicMock", planner._replan_limiter.get_attempt).assert_not_called()


def test_deviation_replan_keeps_the_observed_plan_epochs(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._current_goal = MagicMock()
    planner._current_goal.position.distance.return_value = 10.0
    planner._current_odom = MagicMock()
    planner._goal_epoch = 1
    planner._plan_epoch = 3
    deviation_check_started = Event()
    release_deviation_check = Event()

    def report_deviation_after_replacement() -> float:
        deviation_check_started.set()
        assert release_deviation_check.wait(timeout=TEST_TIMEOUT)
        return planner._max_path_deviation + 1.0

    cast(
        "MagicMock", planner._local_planner.get_distance_to_path
    ).side_effect = report_deviation_after_replacement
    mocker.patch.object(planner._replan_event, "wait", return_value=False)
    replan = mocker.patch.object(
        planner,
        "_replan_path",
        side_effect=lambda *_epochs: planner._stop_planner.set(),
    )

    monitor_thread = Thread(target=planner._thread_entrypoint)
    monitor_thread.start()
    try:
        assert deviation_check_started.wait(timeout=TEST_TIMEOUT)
        with planner._activation_lock:
            with planner._lock:
                planner._current_goal = MagicMock()
                planner._goal_epoch = 2
                planner._plan_epoch = 4
        release_deviation_check.set()
        monitor_thread.join(timeout=TEST_TIMEOUT)
    finally:
        release_deviation_check.set()
        planner._stop_planner.set()
        monitor_thread.join(timeout=TEST_TIMEOUT)

    assert not monitor_thread.is_alive()
    replan.assert_called_once_with(1, 3)


def test_stuck_replan_keeps_the_observed_plan_epochs(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._current_goal = MagicMock()
    planner._current_goal.position.distance.return_value = 10.0
    planner._current_odom = MagicMock()
    planner._goal_epoch = 1
    planner._plan_epoch = 3
    planner._stuck_time_window = -1.0
    stuck_check_started = Event()
    release_stuck_check = Event()

    cast("MagicMock", planner._local_planner.get_distance_to_path).return_value = None
    cast("MagicMock", planner._local_planner.get_unique_state).return_value = (
        "path_following",
        7,
    )

    def report_stuck_after_replacement() -> bool:
        stuck_check_started.set()
        assert release_stuck_check.wait(timeout=TEST_TIMEOUT)
        return True

    cast(
        "MagicMock", planner._position_tracker.is_stuck
    ).side_effect = report_stuck_after_replacement
    mocker.patch.object(planner._replan_event, "wait", return_value=False)
    replan = mocker.patch.object(
        planner,
        "_replan_path",
        side_effect=lambda *_epochs: planner._stop_planner.set(),
    )

    monitor_thread = Thread(target=planner._thread_entrypoint)
    monitor_thread.start()
    try:
        assert stuck_check_started.wait(timeout=TEST_TIMEOUT)
        with planner._activation_lock:
            with planner._lock:
                planner._current_goal = MagicMock()
                planner._goal_epoch = 2
                planner._plan_epoch = 4
        release_stuck_check.set()
        monitor_thread.join(timeout=TEST_TIMEOUT)
    finally:
        release_stuck_check.set()
        planner._stop_planner.set()
        monitor_thread.join(timeout=TEST_TIMEOUT)

    assert not monitor_thread.is_alive()
    replan.assert_called_once_with(1, 3)


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
        cast("MagicMock", planner._local_planner.start_tagged_planning).reset_mock()
        replacement_plan = mocker.patch.object(planner, "_plan_path")
        planner.handle_goal_request(MagicMock())
        release_planning.set()
        plan_future.result(timeout=TEST_TIMEOUT)

    replacement_plan.assert_called_once()
    planner.path.on_next.assert_not_called()
    cast("MagicMock", planner._local_planner.start_tagged_planning).assert_not_called()


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
            start_planning = cast("MagicMock", planner._local_planner.start_tagged_planning)
            start_planning.reset_mock()
            stop_future = executor.submit(planner.stop)
            assert planner._stop_planner.wait(timeout=TEST_TIMEOUT)
            stop_future.result(timeout=TEST_TIMEOUT)
            release_publication.set()
            plan_future.result(timeout=TEST_TIMEOUT)
    finally:
        release_publication.set()

    start_planning.assert_not_called()
    assert publication_released == [True]


def test_path_observer_can_wait_for_stop_without_deadlock(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._current_odom = MagicMock()
    planner._current_goal = MagicMock()
    planner._goal_epoch = 1
    planner._thread = None
    planner._disposables = MagicMock()
    planner.path = MagicMock()
    resampled_path = MagicMock()
    observer_completed = Event()
    stop_threads: list[Thread] = []
    mocker.patch.object(planner, "_find_safe_goal", return_value=MagicMock())
    mocker.patch.object(planner, "_find_wide_path", return_value=MagicMock())
    mocker.patch.object(planner_module, "smooth_resample_path", return_value=resampled_path)

    def stop_from_another_thread(path: Any) -> None:
        if path is not resampled_path:
            return

        stop_thread = Thread(target=planner.stop)
        stop_threads.append(stop_thread)
        stop_thread.start()
        stop_thread.join(timeout=TEST_TIMEOUT)
        observer_completed.set()

    planner.path.on_next.side_effect = stop_from_another_thread

    planner._plan_path(1)

    assert observer_completed.is_set()
    assert len(stop_threads) == 1
    assert not stop_threads[0].is_alive()
    assert planner._stop_planner.is_set()
    cast("MagicMock", planner._local_planner.start_tagged_planning).assert_not_called()


def test_stop_during_path_publication_leaves_an_empty_path(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._current_odom = MagicMock()
    planner._current_goal = MagicMock()
    planner._goal_epoch = 1
    planner._thread = None
    resampled_path = MagicMock()
    observed_paths: list[Any] = []
    planner.path = Subject()
    mocker.patch.object(planner, "_find_safe_goal", return_value=MagicMock())
    mocker.patch.object(planner, "_find_wide_path", return_value=MagicMock())
    mocker.patch.object(planner_module, "smooth_resample_path", return_value=resampled_path)

    def stop_on_resampled_path(path: Any) -> None:
        if path is resampled_path:
            planner.stop()

    stop_subscription = planner.path.subscribe(stop_on_resampled_path)
    record_subscription = planner.path.subscribe(observed_paths.append)

    try:
        planner._plan_path(1)
    finally:
        stop_subscription.dispose()
        record_subscription.dispose()

    assert observed_paths[-2] is resampled_path
    assert observed_paths[-1].poses == []
    cast("MagicMock", planner._local_planner.start_tagged_planning).assert_not_called()


def test_cancel_observer_can_wait_for_stop_without_deadlock(
    planner: GlobalPlanner,
) -> None:
    planner._current_odom = MagicMock()
    planner._current_goal = MagicMock()
    planner._goal_epoch = 1
    planner._thread = None
    planner._disposables = MagicMock()
    planner.path = MagicMock()
    observer_completed = Event()

    def stop_from_another_thread(_path: Any) -> None:
        stop_thread = Thread(target=planner.stop)
        stop_thread.start()
        stop_thread.join(timeout=TEST_TIMEOUT)
        assert not stop_thread.is_alive()
        observer_completed.set()

    planner.path.on_next.side_effect = stop_from_another_thread

    planner._plan_path(1)

    assert observer_completed.is_set()
    assert planner._stop_planner.is_set()
    cast("MagicMock", planner._local_planner.start_tagged_planning).assert_not_called()


def test_stop_cannot_linearize_during_local_activation(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._current_odom = MagicMock()
    planner._current_goal = MagicMock()
    planner._goal_epoch = 1
    planner._thread = None
    planner._disposables = MagicMock()
    planner.path = MagicMock()
    resampled_path = MagicMock()
    activation_started = Event()
    release_activation = Event()
    stop_started = Event()
    start_planning_saw_shutdown: list[bool] = []
    mocker.patch.object(planner, "_find_safe_goal", return_value=MagicMock())
    mocker.patch.object(planner, "_find_wide_path", return_value=MagicMock())
    mocker.patch.object(planner_module, "smooth_resample_path", return_value=resampled_path)

    def pause_activation(_path: Any, _plan_epochs: tuple[int, int]) -> None:
        start_planning_saw_shutdown.append(planner._stop_planner.is_set())
        activation_started.set()
        if not release_activation.wait(timeout=TEST_TIMEOUT):
            raise TimeoutError("test did not release local activation")

    def stop_planner() -> None:
        stop_started.set()
        planner.stop()

    cast("MagicMock", planner._local_planner.start_tagged_planning).side_effect = pause_activation

    try:
        with ThreadPoolExecutor(max_workers=2) as executor:
            plan_future = executor.submit(planner._plan_path, 1)
            assert activation_started.wait(timeout=TEST_TIMEOUT)
            stop_future = executor.submit(stop_planner)
            assert stop_started.wait(timeout=TEST_TIMEOUT)
            assert not planner._stop_planner.is_set()
            release_activation.set()
            plan_future.result(timeout=TEST_TIMEOUT)
            stop_future.result(timeout=TEST_TIMEOUT)
    finally:
        release_activation.set()

    assert start_planning_saw_shutdown == [False]


def test_dequeued_completion_is_ignored_after_shutdown_starts(
    planner: GlobalPlanner, mocker: MockerFixture
) -> None:
    planner._disposables = MagicMock()
    planner.path = MagicMock()
    planner._replan_reason = PlanStopEvent("arrived", (1, 1))
    planner._replan_event.set()
    handler_entered = Event()
    release_handler = Event()
    shutdown_cancelled = Event()
    original_handler = planner._handle_stop_message

    def pause_handler(stop_message: Any) -> None:
        handler_entered.set()
        if not release_handler.wait(timeout=TEST_TIMEOUT):
            raise TimeoutError("test did not release dequeued completion")
        original_handler(stop_message)

    cancel_goal = mocker.patch.object(
        planner,
        "_cancel_goal",
        side_effect=lambda **_kwargs: shutdown_cancelled.set(),
    )
    mocker.patch.object(planner, "_handle_stop_message", side_effect=pause_handler)
    monitor_thread = Thread(target=planner._thread_entrypoint, daemon=True)
    planner._thread = monitor_thread
    monitor_thread.start()

    try:
        assert handler_entered.wait(timeout=TEST_TIMEOUT)
        with ThreadPoolExecutor(max_workers=1) as executor:
            stop_future = executor.submit(planner.stop)
            assert shutdown_cancelled.wait(timeout=TEST_TIMEOUT)
            release_handler.set()
            stop_future.result(timeout=TEST_TIMEOUT)
    finally:
        release_handler.set()
        monitor_thread.join(timeout=TEST_TIMEOUT)

    assert not monitor_thread.is_alive()
    assert cancel_goal.mock_calls == [
        call(
            allow_during_shutdown=True,
            before_notifications=ANY,
        )
    ]
    planner.path.on_next.assert_not_called()


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

    cast("MagicMock", planner._local_planner.start_tagged_planning).assert_not_called()


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
