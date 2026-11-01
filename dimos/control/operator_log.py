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

"""A log of what the operator and the robot did during a teleop run.

Each event is a dict:
``{"ts", "kind", "action", "route", "reason", "object_id", "detail"}``,
with ``ts`` in unix seconds and the other fields strings ("" when unused).

========  ==================================================  ===========================
kind      action                                              comes from
========  ==================================================  ===========================
hold      request, acknowledge                                ``operator_hold``
safety    estop, estop_clear                                  ``operator_estop_status``
data      episode_start, episode_saved, episode_discarded     ``status`` (EpisodeStatus)
takeover  takeover                                            ``record_event``
assist    anything else the page reports (target, pick, ...)  ``record_event``
========  ==================================================  ===========================
"""

from __future__ import annotations

from collections.abc import Callable
import json
import os
from pathlib import Path
import threading
import time
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.control.tasks.operator_hold_task.operator_hold_task import OperatorHoldStatus
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.imitation.collection.episode import EpisodeStatus
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.String import String
from dimos.utils.logging_config import get_run_log_dir, setup_logger

logger = setup_logger()

OPERATOR_EVENTS_FILE = "operator_events.jsonl"


class OperatorLogConfig(ModuleConfig):
    # How many of the latest events each operator_events message carries.
    status_events: int = Field(default=50, ge=1)


class OperatorLog(Module):
    """Keep the operator event log: one JSON line per event in the run's log folder.

    Logs each hold request and acknowledge, e-stop and clear, episode start,
    save and discard, and every action the operator's page reports through
    ``record_event``. After each event it sends the latest ones on
    ``operator_events``. Any input may be left unconnected.
    """

    config: OperatorLogConfig

    # The hold task's OperatorHoldStatus as JSON (dataclasses.asdict), resent every second while on.
    operator_hold: In[String]
    # True while the coordinator's e-stop is latched.
    operator_estop_status: In[Bool]
    # The episode monitor's EpisodeStatus JSON.
    status: In[String]
    # {"ts": unix s, "count": events so far, "events": the latest status_events events}
    operator_events: Out[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._events: list[dict[str, Any]] = []
        self._hold: OperatorHoldStatus | None = None  # the hold being logged, while it is on
        self._estopped = False
        self._episode: EpisodeStatus | None = None

    @rpc
    def start(self) -> None:
        """Start following the hold, e-stop and episode streams that are connected."""
        super().start()
        inputs: list[tuple[In[Any], Callable[[Any], None]]] = [
            (self.operator_hold, self._on_hold),
            (self.operator_estop_status, self._on_estop),
            (self.status, self._on_episode),
        ]
        for port, handler in inputs:
            if port.connection is not None or port._transport is not None:
                self.register_disposable(Disposable(port.subscribe(handler)))

    @rpc
    def record_event(self, action: str, detail: str = "", object_id: str = "") -> dict[str, Any]:
        """Log one thing the operator did on the page.

        Args:
            action: What the operator did, e.g. "takeover", "target", "pick".
                "takeover" is logged with kind "takeover", anything else with
                kind "assist". Must not be empty.
            detail: Free text, e.g. the result of a pick.
            object_id: The object the action was about, "" if none.

        Returns:
            The event as logged.
        """
        if not action:
            raise ValueError("action must not be empty")
        kind = "takeover" if action == "takeover" else "assist"
        return self._log(kind, action, object_id=object_id, detail=detail)

    @rpc
    def get_events(self, since: float = 0.0) -> list[dict[str, Any]]:
        """Every event of this run newer than ``since``, oldest first.

        Args:
            since: Unix time in seconds; only events with a later ``ts`` are returned.
        """
        with self._lock:
            return [dict(event) for event in self._events if event["ts"] > since]

    @rpc
    def get_status(self) -> dict[str, Any]:
        """How many events there are, the file they go to ("" outside a run), and the last one."""
        path = _events_path()
        with self._lock:
            return {
                "count": len(self._events),
                "path": str(path) if path is not None else "",
                "last": dict(self._events[-1]) if self._events else None,
            }

    def _on_hold(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            status = OperatorHoldStatus(**{**payload, "unheld": tuple(payload.get("unheld", ()))})
        except (TypeError, ValueError):
            logger.warning("Ignoring invalid operator hold status", exc_info=True)
            return
        with self._lock:
            held = self._hold
            self._hold = status if status.on else None
            # The hold resends its status every second while on: log only the changes.
            if status.on and held is None:
                self._log_locked("hold", "request", route=status.route, reason=status.reason)
            elif not status.on and held is not None:
                self._log_locked("hold", "acknowledge", route=held.route, reason=held.reason)
            else:
                return
        self._publish()

    def _on_estop(self, msg: Bool) -> None:
        estopped = bool(msg.data)
        with self._lock:
            if estopped == self._estopped:
                return
            self._estopped = estopped
            self._log_locked("safety", "estop" if estopped else "estop_clear")
        self._publish()

    def _on_episode(self, msg: String) -> None:
        try:
            status = EpisodeStatus.from_json(msg.data)
        except ValueError:
            logger.warning("Ignoring invalid episode status", exc_info=True)
            return
        with self._lock:
            before = self._episode
            self._episode = status
            if before is None:
                # No counters to compare with yet; only a take already under way shows.
                actions = ["episode_start"] if status.state == "recording" else []
            else:
                saved = status.episodes_saved > before.episodes_saved
                actions = []
                if saved:
                    actions.append("episode_saved")
                if status.episodes_discarded > before.episodes_discarded:
                    actions.append("episode_discarded")
                # A start while recording saves the take in progress and begins a new one.
                if status.state == "recording" and (before.state == "idle" or saved):
                    actions.append("episode_start")
            if not actions:
                return
            for action in actions:
                self._log_locked("data", action, detail=status.task_label or "")
        self._publish()

    def _log(self, kind: str, action: str, object_id: str, detail: str) -> dict[str, Any]:
        with self._lock:
            event = dict(self._log_locked(kind, action, object_id=object_id, detail=detail))
        self._publish()
        return event

    def _log_locked(
        self,
        kind: str,
        action: str,
        route: str = "",
        reason: str = "",
        object_id: str = "",
        detail: str = "",
    ) -> dict[str, Any]:
        event: dict[str, Any] = {
            "ts": time.time(),
            "kind": kind,
            "action": action,
            "route": route,
            "reason": reason,
            "object_id": object_id,
            "detail": detail,
        }
        self._events.append(event)
        logger.info("Operator event", **event)
        path = _events_path()
        if path is not None:
            try:
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open("a", encoding="utf-8") as file:
                    file.write(json.dumps(event) + "\n")
            except OSError:
                logger.warning("Writing the operator event log failed", path=str(path))
        return event

    def _publish(self) -> None:
        """Send the event count and the latest events on operator_events."""
        with self._lock:
            payload = json.dumps(
                {
                    "ts": time.time(),
                    "count": len(self._events),
                    "events": self._events[-self.config.status_events :],
                }
            )
        try:
            self.operator_events.publish(String(payload))
        except Exception:
            logger.warning("Publishing operator_events failed", exc_info=True)


def _events_path() -> Path | None:
    """The run's operator event file; None outside a run (a test, a shell)."""
    # Worker processes only inherit the run folder through the environment.
    run_dir = get_run_log_dir() or os.environ.get("DIMOS_RUN_LOG_DIR")
    return Path(run_dir) / OPERATOR_EVENTS_FILE if run_dir else None
