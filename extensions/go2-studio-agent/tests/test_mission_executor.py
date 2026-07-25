"""Lifecycle tests for the background Go2 mission executor."""

from collections.abc import Callable
import json
from pathlib import Path
from threading import Lock
import time
from typing import Any

from dimos_go2_studio.mission_contracts import (
    MissionKind,
    TaskSpec,
    TaskState,
)
from dimos_go2_studio.mission_executor import MissionExecutor
from dimos_go2_studio.semantic_world import SemanticWorld

from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.base import NavigationState


class FakeNavigation:
    def __init__(self, *, accept_goal: bool = True) -> None:
        self.accept_goal = accept_goal
        self.state = NavigationState.IDLE
        self.reached = False
        self.set_goal_calls = 0
        self.cancel_goal_calls = 0
        self.goals: list[PoseStamped] = []
        self._lock = Lock()

    def set_goal(self, goal: PoseStamped) -> bool:
        with self._lock:
            self.set_goal_calls += 1
            self.goals.append(goal)
            if not self.accept_goal:
                return False
            self.state = NavigationState.FOLLOWING_PATH
            self.reached = False
            return True

    def get_state(self) -> NavigationState:
        with self._lock:
            return self.state

    def is_goal_reached(self) -> bool:
        with self._lock:
            return self.reached

    def cancel_goal(self) -> bool:
        with self._lock:
            self.cancel_goal_calls += 1
            self.state = NavigationState.IDLE
            self.reached = False
            return True

    def mark_reached_but_moving(self) -> None:
        with self._lock:
            self.reached = True
            self.state = NavigationState.FOLLOWING_PATH

    def finish_goal(self) -> None:
        with self._lock:
            self.reached = True
            self.state = NavigationState.IDLE


class FakeResolver:
    def __init__(
        self,
        result: PoseStamped | None = None,
        *,
        error: Exception | None = None,
    ) -> None:
        self.result = result if result is not None else _goal()
        self.error = error
        self.calls: list[TaskSpec] = []

    def resolve(self, task: TaskSpec) -> PoseStamped | None:
        self.calls.append(task)
        if self.error is not None:
            raise self.error
        return self.result


class MissingResolver(FakeResolver):
    def __init__(self) -> None:
        super().__init__()

    def resolve(self, task: TaskSpec) -> None:
        self.calls.append(task)
        return None


class ManualClock:
    def __init__(self) -> None:
        self.value = 0.0
        self._lock = Lock()

    def __call__(self) -> float:
        with self._lock:
            return self.value

    def wait(self, seconds: float) -> None:
        with self._lock:
            self.value += max(seconds, 0.01)
        time.sleep(0)


class TestMissionExecutor(MissionExecutor):
    """Disable transport-backed tool streams while retaining skill behavior."""

    __test__ = False

    def __init__(self, **kwargs: Any) -> None:
        self.started_tools: list[str] = []
        self.stopped_tools: list[str] = []
        self.updates: list[tuple[str, str]] = []
        super().__init__(**kwargs)

    def start_tool(self, name: str) -> None:
        self.started_tools.append(name)

    def tool_update(self, name: str, message: str) -> None:
        self.updates.append((name, message))

    def stop_tool(self, name: str) -> None:
        self.stopped_tools.append(name)


def _goal() -> PoseStamped:
    return PoseStamped(
        frame_id="map",
        position=[1.0, 2.0, 0.0],
        orientation=[0.0, 0.0, 0.0, 1.0],
    )


def _task(task_id: str = "task-0001") -> TaskSpec:
    return TaskSpec(
        task_id=task_id,
        kind=MissionKind.GO_TO_PLACE,
        destination="门口",
    )


def _status(executor: MissionExecutor) -> dict[str, Any]:
    return json.loads(executor.get_task_status())


def _wait_for(
    predicate: Callable[[], bool],
    *,
    timeout_s: float = 1.0,
) -> None:
    deadline = time.monotonic() + timeout_s
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError("timed out waiting for executor state")
        time.sleep(0.002)


def _executor(
    navigation: FakeNavigation | None = None,
    resolver: FakeResolver | None = None,
    **kwargs: Any,
) -> TestMissionExecutor:
    kwargs.setdefault("poll_interval_s", 0.002)
    kwargs.setdefault("mission_timeout_s", 5.0)
    kwargs.setdefault("cancel_wait_s", 0.2)
    return TestMissionExecutor(
        navigation=navigation or FakeNavigation(),
        resolver=resolver or FakeResolver(),
        **kwargs,
    )


def _close(executor: MissionExecutor) -> None:
    executor.stop()


def test_start_task_is_background_movement_skill() -> None:
    assert MissionExecutor.start_task.__skill__ is True
    assert MissionExecutor.start_task.__skill_lifecycle__ == "background"
    assert MissionExecutor.start_task.__skill_uses__ == [CAP_MOVEMENT]
    for name in (
        "pause_task",
        "resume_task",
        "cancel_task",
        "get_task_status",
    ):
        assert getattr(MissionExecutor, name).__skill__ is True


def test_fresh_executor_has_no_false_running_task() -> None:
    executor = _executor()
    try:
        assert _status(executor) == {"active": False, "state": "idle"}
    finally:
        _close(executor)


def test_invalid_task_closes_background_tool_without_starting_worker() -> None:
    executor = _executor()
    try:
        response = json.loads(executor.start_task('{"kind":"go_to_place"}'))

        assert response["accepted"] is False
        assert "invalid TaskSpec" in response["reason"]
        assert executor.started_tools == ["start_task"]
        assert executor.stopped_tools == ["start_task"]
        assert _status(executor) == {"active": False, "state": "idle"}
    finally:
        _close(executor)


def test_default_resolver_fails_before_navigation_goal() -> None:
    navigation = FakeNavigation()
    executor = TestMissionExecutor(
        navigation=navigation,
        poll_interval_s=0.002,
        mission_timeout_s=5.0,
        cancel_wait_s=0.2,
    )
    try:
        executor.start_task(_task("task-default-resolver").model_dump_json())
        _wait_for(lambda: _status(executor).get("state") == "failed")

        assert "resolver" in _status(executor)["terminal_reason"]
        assert navigation.set_goal_calls == 0
    finally:
        _close(executor)


def test_confirmed_semantic_place_resolves_to_exact_navigation_goal(
    tmp_path: Path,
) -> None:
    navigation = FakeNavigation()
    world = SemanticWorld(
        storage_path=tmp_path / "semantic-world.json",
        map_id="venue-hall",
        map_version="map-v1",
    )
    executor = TestMissionExecutor(
        navigation=navigation,
        poll_interval_s=0.002,
        mission_timeout_s=5.0,
        cancel_wait_s=0.2,
    )
    executor._destination_resolver = world
    try:
        world.confirm_place(
            name="会场正门",
            aliases=["入口"],
            pose=PoseStamped(
                ts=6789.0,
                frame_id="map",
                position=[2.5, -1.25, 0.0],
                orientation=[0.0, 0.0, 0.0, 1.0],
            ),
        )

        response = json.loads(
            executor.start_task(
                TaskSpec(
                    task_id="task-semantic-goal",
                    kind=MissionKind.GO_TO_PLACE,
                    destination="入口",
                ).model_dump_json()
            )
        )
        _wait_for(lambda: _status(executor).get("state") == "navigating")

        assert response["accepted"] is True
        assert navigation.set_goal_calls == 1
        goal = navigation.goals[0]
        assert goal.frame_id == "map"
        assert goal.ts == 6789.0
        assert goal.x == 2.5
        assert goal.y == -1.25
    finally:
        executor.cancel_task("task-semantic-goal")
        _close(executor)
        world.stop()


def test_goal_acknowledgement_does_not_complete_task() -> None:
    navigation = FakeNavigation()
    events: list[TaskState] = []
    executor = _executor(
        navigation,
        event_sink=lambda snapshot: events.append(snapshot.state),
    )
    try:
        response = json.loads(executor.start_task(_task().model_dump_json()))
        _wait_for(lambda: _status(executor).get("state") == "navigating")

        assert response["accepted"] is True
        assert navigation.set_goal_calls == 1
        assert _status(executor)["state"] == "navigating"
        assert TaskState.COMPLETED not in events
    finally:
        executor.cancel_task("task-0001")
        _close(executor)


def test_mission_transitions_publish_canonical_status_snapshots() -> None:
    navigation = FakeNavigation()
    executor = _executor(navigation)
    snapshots: list[dict[str, Any]] = []
    executor.mission_status_snapshot.subscribe(
        lambda message: snapshots.append(json.loads(message.data))
    )
    try:
        executor.start_task(_task("task-viewer-status").model_dump_json())
        _wait_for(lambda: _status(executor).get("state") == "navigating")

        assert [snapshot["state"] for snapshot in snapshots[:3]] == [
            "queued",
            "resolving",
            "navigating",
        ]
        assert snapshots[-1]["task"]["destination"] == "门口"

        navigation.finish_goal()
        _wait_for(lambda: _status(executor).get("state") == "completed")
        assert snapshots[-1]["state"] == "completed"
        assert snapshots[-1]["active"] is False
    finally:
        _close(executor)


def test_completion_requires_goal_reached_and_navigation_idle() -> None:
    navigation = FakeNavigation()
    events: list[TaskState] = []
    executor = _executor(
        navigation,
        event_sink=lambda snapshot: events.append(snapshot.state),
    )
    try:
        executor.start_task(_task().model_dump_json())
        _wait_for(lambda: _status(executor).get("state") == "navigating")

        navigation.mark_reached_but_moving()
        time.sleep(0.02)
        assert _status(executor)["state"] == "navigating"

        navigation.finish_goal()
        _wait_for(lambda: _status(executor).get("state") == "completed")
        status = _status(executor)

        assert status["active"] is False
        assert status["result"]["evidence_ids"] == ["arrival:task-0001"]
        assert events[:2] == [TaskState.QUEUED, TaskState.RESOLVING]
        assert events[-1] is TaskState.COMPLETED
        assert executor.stopped_tools[-1] == "start_task"
    finally:
        _close(executor)


def test_second_active_task_is_rejected() -> None:
    resolver = FakeResolver()
    executor = _executor(resolver=resolver)
    try:
        executor.start_task(_task("task-0001").model_dump_json())
        _wait_for(lambda: _status(executor).get("state") == "navigating")

        response = json.loads(
            executor.start_task(_task("task-0002").model_dump_json())
        )

        assert response == {
            "accepted": False,
            "active_task_id": "task-0001",
            "reason": "another task is active",
        }
        assert len(resolver.calls) == 1
    finally:
        executor.cancel_task("task-0001")
        _close(executor)


def test_resolver_failure_and_goal_rejection_fail_closed() -> None:
    missing_executor = _executor(resolver=MissingResolver())
    broken_executor = _executor(
        resolver=FakeResolver(error=RuntimeError("resolver unavailable"))
    )
    rejecting_navigation = FakeNavigation(accept_goal=False)
    rejecting_executor = _executor(navigation=rejecting_navigation)
    try:
        missing_executor.start_task(_task("task-0003").model_dump_json())
        _wait_for(lambda: _status(missing_executor).get("state") == "failed")
        assert "resolve" in _status(missing_executor)["terminal_reason"]

        broken_executor.start_task(_task("task-broken-resolver").model_dump_json())
        _wait_for(lambda: _status(broken_executor).get("state") == "failed")
        assert "resolver unavailable" in _status(broken_executor)["terminal_reason"]

        rejecting_executor.start_task(_task("task-0004").model_dump_json())
        _wait_for(lambda: _status(rejecting_executor).get("state") == "failed")
        assert "rejected" in _status(rejecting_executor)["terminal_reason"]
        assert rejecting_navigation.get_state() is NavigationState.IDLE
    finally:
        _close(missing_executor)
        _close(broken_executor)
        _close(rejecting_executor)


def test_timeout_cancels_navigation_and_fails_task() -> None:
    clock = ManualClock()
    navigation = FakeNavigation()
    executor = _executor(
        navigation,
        clock=clock,
        waiter=clock.wait,
        mission_timeout_s=0.05,
    )
    try:
        executor.start_task(_task("task-0005").model_dump_json())
        _wait_for(lambda: _status(executor).get("state") == "failed")
        status = _status(executor)

        assert "timeout" in status["terminal_reason"]
        assert navigation.cancel_goal_calls >= 1
        assert navigation.get_state() is NavigationState.IDLE
    finally:
        _close(executor)


def test_pause_reaches_idle_and_resume_reissues_goal() -> None:
    navigation = FakeNavigation()
    executor = _executor(navigation)
    try:
        executor.start_task(_task("task-0006").model_dump_json())
        _wait_for(lambda: _status(executor).get("state") == "navigating")

        paused = json.loads(executor.pause_task("task-0006"))
        assert paused["state"] == "paused"
        assert paused["resume_state"] == "navigating"
        assert navigation.get_state() is NavigationState.IDLE

        resumed = json.loads(executor.resume_task("task-0006"))
        assert resumed["state"] == "navigating"
        _wait_for(lambda: navigation.set_goal_calls == 2)

        navigation.finish_goal()
        _wait_for(lambda: _status(executor).get("state") == "completed")
    finally:
        _close(executor)


def test_cancellation_is_idempotent_and_finishes_idle() -> None:
    navigation = FakeNavigation()
    executor = _executor(navigation)
    try:
        executor.start_task(_task("task-0007").model_dump_json())
        _wait_for(lambda: _status(executor).get("state") == "navigating")

        first = json.loads(executor.cancel_task("task-0007"))
        first_cancel_count = navigation.cancel_goal_calls
        second = json.loads(executor.cancel_task("task-0007"))

        assert first["state"] == "cancelled"
        assert first["active"] is False
        assert second["state"] == "cancelled"
        assert navigation.cancel_goal_calls == first_cancel_count
        assert navigation.get_state() is NavigationState.IDLE
    finally:
        _close(executor)


def test_global_stop_can_cancel_active_task_without_task_id() -> None:
    navigation = FakeNavigation()
    executor = _executor(navigation)
    try:
        executor.start_task(_task("task-global-stop").model_dump_json())
        _wait_for(lambda: _status(executor).get("state") == "navigating")

        payload = json.loads(executor.cancel_active_task())

        assert payload["state"] == "cancelled"
        assert payload["active"] is False
        assert payload["navigation_idle"] is True
        assert navigation.get_state() is NavigationState.IDLE
        assert json.loads(executor.cancel_active_task())["state"] == "cancelled"
    finally:
        _close(executor)


def test_new_executor_after_restart_is_inactive() -> None:
    first = _executor()
    try:
        first.start_task(_task("task-0008").model_dump_json())
        _wait_for(lambda: _status(first).get("state") == "navigating")
    finally:
        first.cancel_task("task-0008")
        _close(first)

    restarted = _executor()
    try:
        assert _status(restarted) == {"active": False, "state": "idle"}
    finally:
        _close(restarted)
