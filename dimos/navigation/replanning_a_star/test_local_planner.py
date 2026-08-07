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

from threading import Event, Thread
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.navigation.replanning_a_star import local_planner as planner_module
from dimos.navigation.replanning_a_star.local_planner import LocalPlanner, PlanStopEvent


@pytest.fixture()
def planner(mocker: MockerFixture) -> LocalPlanner:
    mocker.patch.object(planner_module, "PController", autospec=True)
    return LocalPlanner(GlobalConfig(), MagicMock(), 0.2)


def test_tagged_completion_preserves_public_stop_message(planner: LocalPlanner) -> None:
    public_messages: list[str] = []
    plan_events: list[PlanStopEvent] = []
    public_subscription = planner.stopped_navigating.subscribe(public_messages.append)
    plan_subscription = planner.plan_stopped.subscribe(plan_events.append)

    try:
        planner._publish_stop_event("arrived", (3, 5))
    finally:
        public_subscription.dispose()
        plan_subscription.dispose()

    assert public_messages == ["arrived"]
    assert plan_events == [PlanStopEvent("arrived", (3, 5))]


def test_tagged_completion_precedes_blocking_public_observer(planner: LocalPlanner) -> None:
    public_observer_started = Event()
    release_public_observer = Event()
    plan_events: list[PlanStopEvent] = []

    def block_public_observer(_reason: str) -> None:
        public_observer_started.set()
        assert release_public_observer.wait(timeout=5.0)

    public_subscription = planner.stopped_navigating.subscribe(block_public_observer)
    plan_subscription = planner.plan_stopped.subscribe(plan_events.append)
    publish_thread = Thread(target=planner._publish_stop_event, args=("arrived", (3, 5)))

    try:
        publish_thread.start()
        assert public_observer_started.wait(timeout=5.0)
        assert plan_events == [PlanStopEvent("arrived", (3, 5))]
    finally:
        release_public_observer.set()
        publish_thread.join(timeout=5.0)
        public_subscription.dispose()
        plan_subscription.dispose()

    assert not publish_thread.is_alive()


def test_public_observer_failure_does_not_replace_tagged_completion(
    planner: LocalPlanner,
) -> None:
    plan_events: list[PlanStopEvent] = []

    def fail_public_observer(_reason: str) -> None:
        raise RuntimeError("observer failed")

    public_subscription = planner.stopped_navigating.subscribe(fail_public_observer)
    plan_subscription = planner.plan_stopped.subscribe(plan_events.append)

    try:
        planner._publish_stop_event("arrived", (3, 5))
    finally:
        public_subscription.dispose()
        plan_subscription.dispose()

    assert plan_events == [PlanStopEvent("arrived", (3, 5))]


def test_tagged_observer_failure_does_not_suppress_public_completion(
    planner: LocalPlanner,
) -> None:
    public_messages: list[str] = []

    def fail_tagged_observer(_event: PlanStopEvent) -> None:
        raise RuntimeError("observer failed")

    plan_subscription = planner.plan_stopped.subscribe(fail_tagged_observer)
    public_subscription = planner.stopped_navigating.subscribe(public_messages.append)

    try:
        planner._publish_stop_event("arrived", (3, 5))
    finally:
        plan_subscription.dispose()
        public_subscription.dispose()

    assert public_messages == ["arrived"]


def test_worker_captures_its_plan_epochs(planner: LocalPlanner, mocker: MockerFixture) -> None:
    mocker.patch.object(planner_module, "PathClearance", autospec=True)
    mocker.patch.object(planner_module, "PathDistancer", autospec=True)
    thread_type = mocker.patch.object(planner_module, "Thread", autospec=True)

    planner.start_tagged_planning(MagicMock(), (2, 7))

    stop_event = planner._stop_planning_event
    thread_type.assert_called_once_with(
        target=planner._thread_entrypoint,
        args=(stop_event, (2, 7)),
        daemon=True,
    )
    thread_type.return_value.start.assert_called_once_with()


def test_tagged_start_deactivates_previous_worker_without_stop_command(
    planner: LocalPlanner, mocker: MockerFixture
) -> None:
    mocker.patch.object(planner_module, "PathClearance", autospec=True)
    mocker.patch.object(planner_module, "PathDistancer", autospec=True)
    thread_type = mocker.patch.object(planner_module, "Thread", autospec=True)
    old_stop_event = Event()
    stop_commands: list[Twist] = []
    subscription = planner.cmd_vel.subscribe(stop_commands.append)
    planner._stop_planning_event = old_stop_event
    planner._state = "path_following"
    planner._thread = MagicMock()

    try:
        planner.start_tagged_planning(MagicMock(), (2, 7))
    finally:
        subscription.dispose()

    assert old_stop_event.is_set()
    assert planner.get_unique_state()[0] == "idle"
    assert stop_commands == []
    thread_type.return_value.start.assert_called_once_with()


def test_stop_planning_deactivates_before_publishing_zero(planner: LocalPlanner) -> None:
    stop_event = planner._stop_planning_event
    shutdown_visible: list[bool] = []
    subscription = planner.cmd_vel.subscribe(
        lambda _command: shutdown_visible.append(stop_event.is_set())
    )

    try:
        planner.stop_planning()
    finally:
        subscription.dispose()

    assert shutdown_visible == [True]


def test_stop_command_observer_failure_does_not_escape(
    planner: LocalPlanner, mocker: MockerFixture
) -> None:
    def fail_command_observer(_command: Twist) -> None:
        raise RuntimeError("observer failed")

    log_exception = mocker.patch.object(planner_module.logger, "exception")
    subscription = planner.cmd_vel.subscribe(fail_command_observer)

    try:
        planner.publish_stop_command()
    finally:
        subscription.dispose()

    log_exception.assert_called_once()


def test_worker_stop_command_observer_failure_does_not_escape(
    planner: LocalPlanner, mocker: MockerFixture
) -> None:
    stop_event = planner._stop_planning_event

    def fail_command_observer(_command: Twist) -> None:
        raise RuntimeError("observer failed")

    mocker.patch.object(planner, "_loop")
    log_exception = mocker.patch.object(planner_module.logger, "exception")
    subscription = planner.cmd_vel.subscribe(fail_command_observer)

    try:
        planner._thread_entrypoint(stop_event, (2, 7))
    finally:
        subscription.dispose()

    log_exception.assert_called_once()


def test_stale_worker_does_not_change_replacement_state_before_loop(
    planner: LocalPlanner,
) -> None:
    old_stop_event = Event()
    old_stop_event.set()
    planner._stop_planning_event = Event()
    planner._state = "final_rotation"
    planner._path = MagicMock()
    planner._path_clearance = MagicMock()

    planner._loop(old_stop_event, (1, 1))

    assert planner._state == "final_rotation"


def test_stale_worker_does_not_run_replacement_state_mid_cycle(
    planner: LocalPlanner, mocker: MockerFixture
) -> None:
    old_stop_event = Event()
    planner._stop_planning_event = old_stop_event
    planner._path = MagicMock()
    path_clearance = MagicMock()
    planner._path_clearance = path_clearance
    obstacle_check_started = Event()
    release_obstacle_check = Event()

    def block_obstacle_check() -> bool:
        obstacle_check_started.set()
        if not release_obstacle_check.wait(timeout=5.0):
            raise TimeoutError("test did not release obstacle check")
        return False

    path_clearance.is_obstacle_ahead.side_effect = block_obstacle_check
    compute_final_rotation = mocker.patch.object(
        planner,
        "_compute_final_rotation",
        return_value=Twist(),
    )
    worker = Thread(target=planner._loop, args=(old_stop_event, (1, 1)))

    try:
        worker.start()
        assert obstacle_check_started.wait(timeout=5.0)
        with planner._lock:
            old_stop_event.set()
            planner._stop_planning_event = Event()
            planner._state = "final_rotation"
        release_obstacle_check.set()
        worker.join(timeout=5.0)
    finally:
        release_obstacle_check.set()
        worker.join(timeout=5.0)

    assert not worker.is_alive()
    assert planner._state == "final_rotation"
    compute_final_rotation.assert_not_called()


def test_stale_worker_does_not_change_replacement_state_mid_compute(
    planner: LocalPlanner, mocker: MockerFixture
) -> None:
    old_stop_event = Event()
    replacement_stop_event = Event()
    pose = MagicMock()
    pose.orientation.euler = (0.0, 0.0, 0.0)
    current_odom = MagicMock()
    current_odom.orientation.euler = (0.0, 0.0, 0.0)
    path = MagicMock()
    path.poses = [pose]
    with planner._lock:
        planner._stop_planning_event = old_stop_event
        planner._path = path
        planner._current_odom = current_odom
        planner._state = "initial_rotation"

    computation_started = Event()
    release_computation = Event()
    results: list[Twist | None] = []

    def block_angle_diff(_target: float, _current: float) -> float:
        computation_started.set()
        if not release_computation.wait(timeout=5.0):
            raise TimeoutError("test did not release angle calculation")
        return 0.0

    mocker.patch.object(planner_module, "angle_diff", side_effect=block_angle_diff)
    path_following = mocker.patch.object(planner, "_compute_path_following")
    worker = Thread(
        target=lambda: results.append(planner._compute_initial_rotation(old_stop_event))
    )

    try:
        worker.start()
        assert computation_started.wait(timeout=5.0)
        with planner._lock:
            old_stop_event.set()
            planner._stop_planning_event = replacement_stop_event
            planner._state = "final_rotation"
        release_computation.set()
        worker.join(timeout=5.0)
    finally:
        release_computation.set()
        worker.join(timeout=5.0)

    assert not worker.is_alive()
    assert planner._state == "final_rotation"
    assert results == [None]
    path_following.assert_not_called()


def test_stale_worker_does_not_reset_replacement_state(
    planner: LocalPlanner, mocker: MockerFixture
) -> None:
    old_stop_event = Event()
    old_stop_event.set()
    planner._stop_planning_event = Event()
    reset_state = mocker.patch.object(planner, "_reset_state")
    commands: list[Twist] = []
    subscription = planner.cmd_vel.subscribe(commands.append)
    mocker.patch.object(planner, "_loop")

    try:
        planner._thread_entrypoint(old_stop_event, (1, 1))
    finally:
        subscription.dispose()

    reset_state.assert_not_called()
    assert commands == []
