"""Background lifecycle owner for one deterministic Go2 mission.

Stage 2 connects the executor to a persistent, confirmed-place resolver through
the DimOS module graph. Tests may still inject a resolver and navigator to
verify lifecycle behavior without moving hardware.
"""

from __future__ import annotations

from collections.abc import Callable
import json
import threading
import time
from typing import Any, Protocol

from dimos_lcm.std_msgs import String
from pydantic import ValidationError

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.base import NavigationState
from dimos.navigation.navigation_spec import NavigationInterfaceSpec
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

from .mission_contracts import (
    TaskResult,
    TaskSnapshot,
    TaskSpec,
    TaskState,
    validate_task_transition,
)
from .semantic_world import DestinationResolverSpec

logger = setup_logger()

_START_TOOL = "start_task"


class UnavailableDestinationResolver:
    """Fail-closed resolver used when the module graph is incomplete."""

    def resolve(self, task: TaskSpec) -> None:
        del task
        return None


class MissionExecutorControlSpec(Spec, Protocol):
    """Minimal non-Agent control surface used by the global stop module."""

    def cancel_active_task(self) -> str: ...


class MissionExecutor(Module):
    """Own one mission from accepted contract through a terminal state."""

    mission_status_snapshot: Out[String]
    _navigation: NavigationInterfaceSpec
    _destination_resolver: DestinationResolverSpec

    def __init__(
        self,
        *,
        navigation: NavigationInterfaceSpec | None = None,
        resolver: DestinationResolverSpec | None = None,
        clock: Callable[[], float] = time.monotonic,
        waiter: Callable[[float], Any] | None = None,
        event_sink: Callable[[TaskSnapshot], None] | None = None,
        mission_timeout_s: float = 300.0,
        poll_interval_s: float = 0.1,
        cancel_wait_s: float = 2.0,
        **kwargs: Any,
    ) -> None:
        if mission_timeout_s <= 0:
            raise ValueError("mission_timeout_s must be positive")
        if poll_interval_s <= 0:
            raise ValueError("poll_interval_s must be positive")
        if cancel_wait_s <= 0:
            raise ValueError("cancel_wait_s must be positive")

        super().__init__(**kwargs)
        if navigation is not None:
            self._navigation = navigation
        self._resolver_override = resolver
        self._unavailable_resolver = UnavailableDestinationResolver()
        self._clock = clock
        self._event_sink = event_sink
        self._mission_timeout_s = mission_timeout_s
        self._poll_interval_s = poll_interval_s
        self._cancel_wait_s = cancel_wait_s

        self._lock = threading.RLock()
        self._wake_event = threading.Event()
        self._cancel_event = threading.Event()
        self._waiter = waiter or self._wake_event.wait
        self._task_thread: threading.Thread | None = None
        self._snapshot: TaskSnapshot | None = None
        self._goal: PoseStamped | None = None
        self._deadline: float | None = None
        self._paused_at: float | None = None
        self._resume_goal_requested = False
        self._cancel_reason = "task cancelled"

    @rpc
    def start(self) -> None:
        """Start the executor and publish the canonical idle snapshot."""

        super().start()
        self.mission_status_snapshot.publish(
            String(self._json({"active": False, "state": "idle"}))
        )

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def start_task(self, task_json: str) -> str:
        """Start one validated mission as a background task.

        The call returns after the worker starts. Use `get_task_status` to
        inspect progress and `cancel_task` to stop it. Goal acceptance is not
        physical completion.

        Args:
            task_json: A JSON object matching the canonical TaskSpec contract.
        """

        # Background capability ownership is tied to this stream. Open it
        # before every early return, including same-tool reinvocation.
        self.start_tool(_START_TOOL)

        with self._lock:
            if self._thread_is_alive_locked():
                assert self._snapshot is not None
                return self._json(
                    {
                        "accepted": False,
                        "active_task_id": self._snapshot.task.task_id,
                        "reason": "another task is active",
                    }
                )

            try:
                task = TaskSpec.model_validate_json(task_json)
            except (ValidationError, ValueError) as exc:
                self.stop_tool(_START_TOOL)
                return self._json(
                    {
                        "accepted": False,
                        "reason": f"invalid TaskSpec: {exc}",
                    }
                )

            self._wake_event.clear()
            self._cancel_event.clear()
            self._goal = None
            self._deadline = self._clock() + self._mission_timeout_s
            self._paused_at = None
            self._resume_goal_requested = False
            self._cancel_reason = "task cancelled"
            self._snapshot = None
            self._transition(TaskState.QUEUED, task=task)

            thread = threading.Thread(
                target=self._run_task,
                args=(task,),
                name=f"go2-mission-{task.task_id}",
                daemon=True,
            )
            self._task_thread = thread
            thread.start()

        return self._json(
            {
                "accepted": True,
                "task_id": task.task_id,
                "state": TaskState.QUEUED.value,
            }
        )

    @skill
    def pause_task(self, task_id: str) -> str:
        """Pause an active mission and wait for navigation to become idle.

        Args:
            task_id: Stable ID of the active mission.
        """

        with self._lock:
            snapshot = self._snapshot
            if snapshot is None:
                return self._json(
                    {"accepted": False, "active": False, "reason": "no task"}
                )
            if snapshot.task.task_id != task_id:
                return self._task_mismatch(task_id, snapshot.task.task_id)
            if snapshot.state is TaskState.PAUSED:
                return self._status_json_locked()
            if snapshot.state.is_terminal or not self._thread_is_alive_locked():
                return self._status_json_locked()
            if not snapshot.state.is_resumable:
                return self._json(
                    {
                        "accepted": False,
                        "task_id": task_id,
                        "state": snapshot.state.value,
                        "reason": "task state is not pausable",
                    }
                )

            previous_state = snapshot.state
            self._paused_at = self._clock()
            self._transition(TaskState.PAUSED, resume_state=previous_state)
            self._wake_event.set()

        idle = self._cancel_navigation_and_wait_idle()
        with self._lock:
            payload = self._status_payload_locked()
            payload["navigation_idle"] = idle
            return self._json(payload)

    @skill
    def resume_task(self, task_id: str) -> str:
        """Resume a paused mission from its recorded active phase.

        Args:
            task_id: Stable ID of the paused mission.
        """

        with self._lock:
            snapshot = self._snapshot
            if snapshot is None:
                return self._json(
                    {"accepted": False, "active": False, "reason": "no task"}
                )
            if snapshot.task.task_id != task_id:
                return self._task_mismatch(task_id, snapshot.task.task_id)
            if snapshot.state is not TaskState.PAUSED:
                payload = self._status_payload_locked()
                payload["accepted"] = False
                payload["reason"] = "task is not paused"
                return self._json(payload)

            resume_state = snapshot.resume_state
            assert resume_state is not None
            if self._paused_at is not None and self._deadline is not None:
                self._deadline += max(0.0, self._clock() - self._paused_at)
            self._paused_at = None
            self._resume_goal_requested = (
                resume_state in {TaskState.NAVIGATING, TaskState.RECOVERING}
                and self._goal is not None
            )
            self._transition(resume_state)
            self._wake_event.set()
            return self._status_json_locked()

    @skill
    def cancel_task(self, task_id: str) -> str:
        """Cancel a mission, wait for navigation idle, and return final status.

        Repeating cancellation for the same terminal task is idempotent.

        Args:
            task_id: Stable ID of the mission to cancel.
        """

        with self._lock:
            snapshot = self._snapshot
            if snapshot is None:
                return self._json(
                    {"accepted": True, "active": False, "state": "idle"}
                )
            if snapshot.task.task_id != task_id:
                return self._task_mismatch(task_id, snapshot.task.task_id)
            if snapshot.state.is_terminal:
                return self._status_json_locked()

            self._cancel_reason = "operator cancelled task"
            self._cancel_event.set()
            self._wake_event.set()
            thread = self._task_thread

        idle = self._cancel_navigation_and_wait_idle()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=self._cancel_wait_s)

        with self._lock:
            payload = self._status_payload_locked()
            payload["navigation_idle"] = idle
            return self._json(payload)

    @skill
    def get_task_status(self) -> str:
        """Return the current mission lifecycle snapshot as JSON."""

        with self._lock:
            return self._status_json_locked()

    @rpc
    def cancel_active_task(self) -> str:
        """Cancel the current mission without requiring its ID.

        This RPC is intentionally not an Agent skill. It exists so the global
        `stop_all` path can terminate the canonical task before stopping every
        lower-level activity.
        """

        with self._lock:
            snapshot = self._snapshot
            if snapshot is None:
                return self._json(
                    {"accepted": True, "active": False, "state": "idle"}
                )
            task_id = snapshot.task.task_id
        return self.cancel_task(task_id)

    @rpc
    def stop(self) -> None:
        """Cancel the worker before the module and its tool streams close."""

        with self._lock:
            thread = self._task_thread
            if thread is not None and thread.is_alive():
                self._cancel_reason = "executor stopped"
                self._cancel_event.set()
                self._wake_event.set()

        if thread is not None and thread is not threading.current_thread():
            self._cancel_navigation_and_wait_idle()
            thread.join(timeout=self._cancel_wait_s)
        self.stop_tool(_START_TOOL)
        super().stop()

    def _run_task(self, task: TaskSpec) -> None:
        current_thread = threading.current_thread()
        try:
            if self._cancel_event.is_set():
                self._finish_cancelled()
                return

            self._transition(TaskState.RESOLVING)
            goal = self._resolver_ref().resolve(task)
            if goal is None:
                self._finish_failed("destination resolver could not resolve task")
                return

            with self._lock:
                self._goal = goal

            if not self._wait_until_runnable():
                self._finish_cancelled()
                return

            if not self._navigation_ref().set_goal(goal):
                self._finish_failed("navigation rejected resolved goal")
                return
            self._transition(TaskState.NAVIGATING)

            while True:
                if self._cancel_event.is_set():
                    self._cancel_navigation_and_wait_idle()
                    self._finish_cancelled()
                    return

                if not self._wait_until_runnable():
                    self._cancel_navigation_and_wait_idle()
                    self._finish_cancelled()
                    return

                if self._take_resume_goal_request():
                    if not self._navigation_ref().set_goal(goal):
                        self._finish_failed("navigation rejected resumed goal")
                        return

                navigation = self._navigation_ref()
                nav_state = navigation.get_state()
                if navigation.is_goal_reached() and nav_state is NavigationState.IDLE:
                    self._transition(
                        TaskState.COMPLETED,
                        result=TaskResult(
                            summary=f"arrived at {task.destination or task.kind.value}",
                            evidence_ids=(f"arrival:{task.task_id}",),
                        ),
                    )
                    return

                if self._deadline_expired():
                    self._cancel_navigation_and_wait_idle()
                    self._finish_failed(
                        f"mission timeout after {self._mission_timeout_s:g} seconds"
                    )
                    return

                desired = (
                    TaskState.RECOVERING
                    if nav_state is NavigationState.RECOVERY
                    else TaskState.NAVIGATING
                )
                with self._lock:
                    snapshot = self._snapshot
                    if (
                        snapshot is not None
                        and snapshot.state is not TaskState.PAUSED
                        and snapshot.state is not desired
                    ):
                        self._transition(desired)

                self._wait_once()
        except Exception as exc:
            logger.exception("Mission worker failed", task_id=task.task_id)
            self._cancel_navigation_and_wait_idle()
            self._finish_failed(f"mission executor error: {exc}")
        finally:
            # Keep the thread visible to start_task until after the old
            # capability stream is closed, preventing a new task/old-finally
            # race from closing the new stream.
            self.stop_tool(_START_TOOL)
            with self._lock:
                if self._task_thread is current_thread:
                    self._task_thread = None
                self._wake_event.clear()

    def _wait_until_runnable(self) -> bool:
        while True:
            if self._cancel_event.is_set():
                return False
            with self._lock:
                paused = (
                    self._snapshot is not None
                    and self._snapshot.state is TaskState.PAUSED
                )
            if not paused:
                return True
            self._wait_once()

    def _take_resume_goal_request(self) -> bool:
        with self._lock:
            requested = self._resume_goal_requested
            self._resume_goal_requested = False
            return requested

    def _deadline_expired(self) -> bool:
        with self._lock:
            deadline = self._deadline
            paused = (
                self._snapshot is not None
                and self._snapshot.state is TaskState.PAUSED
            )
        return not paused and deadline is not None and self._clock() >= deadline

    def _cancel_navigation_and_wait_idle(self) -> bool:
        try:
            navigation = self._navigation_ref()
            navigation.cancel_goal()
        except Exception:
            logger.exception("Failed to cancel mission navigation")
            return False

        deadline = self._clock() + self._cancel_wait_s
        while self._clock() < deadline:
            try:
                if navigation.get_state() is NavigationState.IDLE:
                    return True
            except Exception:
                logger.exception("Failed to read navigation state while cancelling")
                return False
            self._wait_once()
        try:
            return navigation.get_state() is NavigationState.IDLE
        except Exception:
            return False

    def _finish_cancelled(self) -> None:
        with self._lock:
            snapshot = self._snapshot
            if snapshot is None or snapshot.state.is_terminal:
                return
            reason = self._cancel_reason
        self._transition(TaskState.CANCELLED, terminal_reason=reason)

    def _finish_failed(self, reason: str) -> None:
        with self._lock:
            snapshot = self._snapshot
            if snapshot is None or snapshot.state.is_terminal:
                return
        self._transition(TaskState.FAILED, terminal_reason=reason)

    def _transition(
        self,
        state: TaskState,
        *,
        task: TaskSpec | None = None,
        result: TaskResult | None = None,
        terminal_reason: str | None = None,
        resume_state: TaskState | None = None,
    ) -> TaskSnapshot:
        with self._lock:
            previous = self._snapshot
            effective_task = task or (previous.task if previous is not None else None)
            if effective_task is None:
                raise RuntimeError("initial task snapshot requires TaskSpec")

            if previous is not None and previous.state is not state:
                prior_resume = (
                    previous.resume_state
                    if previous.state is TaskState.PAUSED
                    else None
                )
                validate_task_transition(
                    previous.state,
                    state,
                    resume_state=prior_resume,
                )

            snapshot = TaskSnapshot(
                task=effective_task,
                state=state,
                result=result,
                terminal_reason=terminal_reason,
                resume_state=resume_state,
            )
            self._snapshot = snapshot

        self._publish_snapshot(snapshot)
        return snapshot

    def _publish_snapshot(self, snapshot: TaskSnapshot) -> None:
        with self._lock:
            status_json = self._status_json_locked()
        self.mission_status_snapshot.publish(String(status_json))
        self.tool_update(
            _START_TOOL,
            self._json(
                {
                    "task_id": snapshot.task.task_id,
                    "state": snapshot.state.value,
                }
            ),
        )
        if self._event_sink is None:
            return
        try:
            self._event_sink(snapshot)
        except Exception:
            logger.exception(
                "Mission event sink failed",
                task_id=snapshot.task.task_id,
                state=snapshot.state.value,
            )

    def _navigation_ref(self) -> NavigationInterfaceSpec:
        navigation = getattr(self, "_navigation", None)
        if navigation is None:
            raise RuntimeError("navigation interface is not connected")
        return navigation

    def _resolver_ref(self) -> DestinationResolverSpec:
        if self._resolver_override is not None:
            return self._resolver_override
        resolver = getattr(self, "_destination_resolver", None)
        if resolver is None:
            return self._unavailable_resolver
        return resolver

    def _thread_is_alive_locked(self) -> bool:
        return self._task_thread is not None and self._task_thread.is_alive()

    def _status_payload_locked(self) -> dict[str, Any]:
        if self._snapshot is None:
            return {"active": False, "state": "idle"}
        payload = self._snapshot.model_dump(mode="json")
        payload["active"] = (
            self._thread_is_alive_locked() and not self._snapshot.state.is_terminal
        )
        return payload

    def _status_json_locked(self) -> str:
        return self._json(self._status_payload_locked())

    @staticmethod
    def _task_mismatch(requested: str, active: str) -> str:
        return MissionExecutor._json(
            {
                "accepted": False,
                "requested_task_id": requested,
                "active_task_id": active,
                "reason": "task_id does not match current task",
            }
        )

    @staticmethod
    def _json(payload: dict[str, Any]) -> str:
        return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))

    def _wait_once(self) -> None:
        self._waiter(self._poll_interval_s)
        self._wake_event.clear()
