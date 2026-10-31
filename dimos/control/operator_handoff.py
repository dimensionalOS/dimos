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

"""The handoff between autonomy and an operator: the agent's request, the operator's log.

The hold itself is the coordinator's (``OperatorHoldCoordinator``): request,
acknowledge and e-stop go to it directly, so they work with this module gone.
This module adds what sits around the hold:

* ``request_operator``, the agent's skill for asking a person for help;
* the operator event log: every request, acknowledge and e-stop the hold and
  e-stop streams show, plus what the operator's page reports (target, pick,
  take over), one JSON line each in the run's log directory;
* ``restart_autonomy``, the operator's explicit hand-back, which tells the
  agent (if one runs) to carry on;
* ``operator_status``, the hold, the latch, the mode and the recent events as
  JSON in a ``std_msgs.String``, for a page reading zenoh.

An event's ``kind`` is "takeover" from the operator's take over until autonomy
restarts, and "assist" otherwise.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import asdict
import json
import os
from pathlib import Path
import threading
import time
from typing import Any, Literal, Protocol

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OPERATOR_HOLD_TASK_NAME,
    HoldRoute,
    OperatorHoldStatus,
)
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.String import String
from dimos.spec.utils import Spec
from dimos.utils.logging_config import get_run_log_dir, setup_logger

logger = setup_logger()

OPERATOR_EVENTS_FILE = "operator_events.jsonl"
# What a page may report; the hold and e-stop events come from their streams.
PAGE_ACTIONS = ("target", "pick", "pick_done", "takeover")
Mode = Literal["autonomous", "assist", "takeover"]


class OperatorHoldSpec(Spec, Protocol):
    """A coordinator running the operator_hold task, as OperatorHoldCoordinator is."""

    def task_invoke(
        self, task_name: str, method: str, kwargs: dict[str, Any] | None = None
    ) -> Any: ...

    def get_operator_status(self) -> dict[str, Any]: ...


class OperatorHandoffConfig(ModuleConfig):
    # How often operator_status goes out when nothing changes, in Hz.
    status_hz: float = 2.0
    # Every this many seconds the hold and the latch are read back from the coordinator,
    # so a missed stream message or a restart of this module cannot leave them stale.
    resync_s: float = 2.0
    # How many of the latest events operator_status carries.
    status_events: int = 50


class OperatorHandoff(Module):
    """The agent's request_operator, the operator event log, and the restart of autonomy."""

    config: OperatorHandoffConfig

    # On/off: each message is a cue to read the whole status back from the coordinator.
    operator_hold_status: In[Bool]
    operator_estop_status: In[Bool]
    operator_status: Out[String]
    # The agent's input (McpClient's human_input): restart_autonomy tells it to carry on.
    human_input: Out[str]

    _coordinator: OperatorHoldSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._hold: dict[str, Any] | None = None  # the last OperatorHoldStatus, as a dict
        self._estopped = False
        self._mode: Mode = "autonomous"
        self._object_id = ""  # the object the operator last targeted or picked
        self._hold_reason = ""  # the reason of the hold being handled; "" once autonomy restarts
        self._events: list[dict[str, Any]] = []
        self._stop_event = threading.Event()
        self._changed = threading.Event()
        self._read_now = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        """Follow the hold and e-stop streams and start sending operator_status."""
        super().start()
        subscriptions: list[tuple[str, Callable[[], Callable[[], None]]]] = [
            ("operator_hold_status", lambda: self.operator_hold_status.subscribe(self._on_status)),
            (
                "operator_estop_status",
                lambda: self.operator_estop_status.subscribe(self._on_status),
            ),
        ]
        for port, subscribe in subscriptions:
            try:
                self.register_disposable(Disposable(subscribe()))
            except Exception:
                logger.warning("Could not subscribe", port=port, exc_info=True)
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="operator-handoff", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        self._changed.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        super().stop()

    @skill
    def request_operator(self, reason: str) -> SkillResult:
        """Ask a person for help: every joint (arms, grippers, lift, base) freezes where it is.

        Use it when you are stuck and should not try again on your own: a grasp missed,
        the object is gone, or the next move looks unsafe. After it, stop calling skills
        that move the robot and wait: an operator acknowledges, sorts it out and restarts
        autonomy with a message to you. Only an operator can release the hold.

        Args:
            reason: What went wrong, in one sentence, for the operator to read.
        """
        reason = reason.strip() or "the agent asked for help"
        try:
            status = self._coordinator.task_invoke(
                OPERATOR_HOLD_TASK_NAME,
                "request",
                {"route": HoldRoute.AGENT.value, "reason": reason},
            )
        except Exception as e:
            return SkillResult(f"Could not request an operator: {e}")
        if not isinstance(status, OperatorHoldStatus) or not status.on:
            return SkillResult(f"Requested an operator, but the hold reads {status!r}.")
        self._on_hold(status)
        return SkillResult(
            "Operator hold on: every joint is frozen until an operator acknowledges it. "
            f"Reason given: {reason}. Wait for the operator to restart autonomy.",
            metadata={
                "route": status.route,
                "reason": status.reason,
                "started_at": status.started_at,
            },
        )

    @rpc
    def record_operator_event(
        self, action: str, object_id: str = "", detail: str = ""
    ) -> dict[str, Any]:
        """Log one operator action the page saw: "target", "pick", "pick_done" or "takeover".

        Args:
            action: What the operator did. "takeover" also turns the mode to takeover
                until autonomy restarts.
            object_id: The object it concerns; "" keeps the last one.
            detail: Free text, e.g. a pick's result for "pick_done".

        Returns the event as logged. Request, acknowledge and e-stop are logged from the
        hold and e-stop streams, not here.
        """
        if action not in PAGE_ACTIONS:
            raise ValueError(f"action must be one of {PAGE_ACTIONS}, got {action!r}")
        with self._lock:
            if object_id:
                self._object_id = object_id
            if action == "takeover":
                self._mode = "takeover"
            event = self._event_locked(action, detail=detail)
        return event

    @rpc
    def restart_autonomy(self, note: str = "") -> dict[str, Any]:
        """Hand the robot back to autonomy: the operator's explicit end of a handoff.

        Refused while the hold is on or the e-stop is latched. Logs "restart", turns the
        mode back to autonomous, and tells the agent (if one runs) to carry on.

        Args:
            note: What the operator did, passed on to the agent.

        Returns ``{"ok": bool, "error": str, "status": get_operator_status()}``.
        """
        object_id = ""
        with self._lock:
            error = ""
            if self._hold is not None and self._hold.get("on"):
                error = "the operator hold is on; acknowledge it first"
            elif self._estopped:
                error = "the e-stop is latched; clear it first"
            else:
                self._event_locked("restart", detail=note)
                object_id, self._mode, self._hold_reason = self._object_id, "autonomous", ""
        if not error:
            message = "The operator has restarted autonomy; carry on with your task."
            if object_id:
                message += f" Object: {object_id}."
            if note:
                message += f" Operator's note: {note}"
            try:
                self.human_input.publish(message)
            except Exception:
                logger.warning("Telling the agent to carry on failed", exc_info=True)
        return {"ok": not error, "error": error, "status": self.get_operator_status()}

    @rpc
    def get_operator_status(self) -> dict[str, Any]:
        """The hold, the e-stop latch, the handoff mode and the latest events, in plain types."""
        with self._lock:
            return self._status_locked()

    @rpc
    def get_operator_events(self, since: float = 0.0) -> list[dict[str, Any]]:
        """Every event of this run logged after ``since`` (unix seconds), oldest first."""
        with self._lock:
            return [event for event in self._events if event["timestamp"] > since]

    def _on_status(self, _msg: Bool) -> None:
        # The message arrives on a stream thread: the RPC that reads the status runs in _run.
        self._read_now.set()
        self._changed.set()

    def _on_hold(self, status: OperatorHoldStatus) -> None:
        hold = {**asdict(status), "unheld": list(status.unheld)}
        with self._lock:
            self._apply_hold_locked(hold)

    def _apply_hold_locked(self, hold: dict[str, Any] | None) -> None:
        was = self._hold or {}
        self._hold = hold
        on = bool(hold and hold.get("on"))
        if on and hold is not None:
            if (
                not was.get("on")
                or was.get("route") != hold.get("route")
                or was.get("reason") != hold.get("reason")
            ):
                self._hold_reason = str(hold.get("reason", ""))
                if self._mode == "autonomous":
                    self._mode = "assist"
                self._event_locked("request", route=str(hold.get("route", "")))
        elif was.get("on"):
            self._event_locked("acknowledge")
        self._changed.set()

    def _apply_estop_locked(self, estopped: bool) -> None:
        if estopped != self._estopped:
            self._estopped = estopped
            self._event_locked("estop" if estopped else "estop_clear")
        self._changed.set()

    def _event_locked(self, action: str, route: str = "", detail: str = "") -> dict[str, Any]:
        event = {
            "timestamp": time.time(),
            "kind": "takeover" if self._mode == "takeover" else "assist",
            "action": action,
            "object_id": self._object_id,
            "reason": self._hold_reason,
            "route": route,
            "detail": detail,
        }
        self._events.append(event)
        # structlog owns "timestamp"; the event's own goes in as event_time.
        logger.info(
            "Operator event",
            event_time=event["timestamp"],
            **{key: value for key, value in event.items() if key != "timestamp"},
        )
        path = _events_path()
        if path is not None:
            try:
                with path.open("a", encoding="utf-8") as file:
                    file.write(json.dumps(event) + "\n")
            except OSError:
                logger.warning("Writing the operator event log failed", path=str(path))
        self._changed.set()
        return event

    def _status_locked(self) -> dict[str, Any]:
        return {
            "stamp": time.time(),
            "hold": self._hold,
            "estopped": self._estopped,
            "mode": self._mode,
            "object_id": self._object_id,
            "hold_reason": self._hold_reason,
            "events": self._events[-self.config.status_events :],
        }

    def _resync(self) -> None:
        """Read the hold and the latch back from the coordinator."""
        try:
            status = self._coordinator.get_operator_status()
        except Exception:
            logger.debug("Reading the operator status from the coordinator failed", exc_info=True)
            return
        with self._lock:
            self._apply_hold_locked(status.get("hold"))
            self._apply_estop_locked(bool(status.get("estopped")))

    def _run(self) -> None:
        period = 1.0 / self.config.status_hz
        next_resync = time.monotonic()
        while not self._stop_event.is_set():
            if self._read_now.is_set() or time.monotonic() >= next_resync:
                self._read_now.clear()
                next_resync = time.monotonic() + self.config.resync_s
                self._resync()
            with self._lock:
                self._changed.clear()
                payload = json.dumps(self._status_locked())
            try:
                self.operator_status.publish(String(payload))
            except Exception:
                logger.warning("Publishing operator_status failed", exc_info=True)
            self._changed.wait(period)


def _events_path() -> Path | None:
    """The run's operator event log; None outside a run (a test, a shell)."""
    run_dir = get_run_log_dir() or os.environ.get("DIMOS_RUN_LOG_DIR")
    return Path(run_dir) / OPERATOR_EVENTS_FILE if run_dir else None
