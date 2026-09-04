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

"""Single point of operator-input → EpisodeStatus translation.

Watches Quest buttons and accepts RPC commands, runs the episode state machine,
publishes EpisodeStatus on every transition. RecordReplay (or whatever
records the bus) captures that stream into session.db; DataPrep reads only
the recorded EpisodeStatus events offline — never raw operator input.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Literal, TypeAlias

from pydantic import Field, field_validator
from reactivex.abc import DisposableBase
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.imitation_msgs.EpisodeStatus import (
    EpisodeEvent,
    EpisodeStatus,
    RecordingState,
)
from dimos.teleop.quest.quest_types import BUTTON_ALIASES, Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# A button/keyboard press requests one of these; `toggle` resolves to
# `start`/`save` based on the current state, so it never reaches the output.
EpisodeCommand: TypeAlias = Literal["start", "save", "discard", "toggle"]


def _default_button_map() -> dict[EpisodeCommand, str]:
    return {"toggle": "B", "discard": "Y"}


class EpisodeMonitorModuleConfig(ModuleConfig):
    button_map: dict[EpisodeCommand, str] = Field(default_factory=_default_button_map)
    task: str

    @field_validator("task")
    @classmethod
    def _validate_task(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("task must be a non-empty description")
        return value.strip()

    @field_validator("button_map")
    @classmethod
    def _validate_button_map(cls, value: dict[EpisodeCommand, str]) -> dict[EpisodeCommand, str]:
        invalid = {
            button
            for button in value.values()
            if BUTTON_ALIASES.get(button, button) not in Buttons.BITS
        }
        if invalid:
            raise ValueError(
                f"unknown Quest button mappings: {sorted(invalid)}; "
                f"valid aliases: {sorted(BUTTON_ALIASES)}"
            )
        resolved = [BUTTON_ALIASES.get(button, button) for button in value.values()]
        if len(resolved) != len(set(resolved)):
            raise ValueError("each episode command must use a distinct Quest button")
        return value


class EpisodeMonitorModule(Module):
    config: EpisodeMonitorModuleConfig

    button_pressed: In[Buttons]
    status: Out[EpisodeStatus]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._state: RecordingState = "idle"
        self._saved: int = 0
        self._discarded: int = 0
        self._last_event: EpisodeEvent = "init"
        self._lock = threading.Lock()
        self._transition_lock = threading.Lock()
        self._stopping = False
        self._input_subscriptions: list[DisposableBase] = []

    @rpc
    def start(self) -> None:
        super().start()
        # Registered so the base Module.stop() disposes them on shutdown.
        self._input_subscriptions = [
            self.register_disposable(Disposable(self.button_pressed.subscribe(self._on_buttons))),
        ]
        # Emit an initial idle status so subscribers (and recorders) have a
        # known starting point in the timeline.
        with self._lock:
            status = self._snapshot("init", time.time())
        self._emit(status)

    @rpc
    def stop(self) -> None:
        with self._transition_lock:
            with self._lock:
                self._stopping = True
            for subscription in self._input_subscriptions:
                subscription.dispose()
            self._input_subscriptions.clear()
            # Do not synthesize a save or discard. If the process is interrupted
            # mid-take, DataPrep must see the unmatched start as incomplete.
        super().stop()

    # ── port handlers ────────────────────────────────────────────────────────

    def _on_buttons(self, msg: Buttons) -> None:
        """Advance the state machine for configured button-press edges."""
        ts = time.time()
        fired: list[EpisodeCommand] = []
        with self._lock:
            if self._stopping:
                return
            for event_name, alias_or_attr in self.config.button_map.items():
                attr = BUTTON_ALIASES.get(alias_or_attr, alias_or_attr)
                if bool(getattr(msg, attr)):
                    fired.append(event_name)
        for event_name in fired:
            self._transition(event_name, ts)

    @rpc
    def command(self, event: EpisodeCommand) -> EpisodeStatus:
        """Apply an episode command from an attached operator interface."""
        return self._transition(event, time.time())

    @rpc
    def get_status(self) -> EpisodeStatus:
        """Return the latest episode state without publishing a new event."""
        with self._lock:
            return self._snapshot(self._last_event, time.time())

    def _transition(self, event: EpisodeCommand, ts: float) -> EpisodeStatus:
        """State-machine transition. Publishes EpisodeStatus on every change.

        ``toggle`` resolves to ``start`` when idle and ``save`` when recording,
        so one button can begin and end a take. The resolved event is what gets
        published (DataPrep only ever sees start/save/discard).
        """
        if event not in ("start", "save", "discard", "toggle"):
            raise ValueError(f"unknown episode command: {event!r}")
        with self._transition_lock:
            with self._lock:
                if self._stopping:
                    return self._snapshot(self._last_event, ts)
                if event == "toggle":
                    event = "save" if self._state == "recording" else "start"
                if event == "start":
                    # Auto-commit any in-progress episode (matches DataPrep extractor).
                    if self._state == "recording":
                        self._saved += 1
                    self._state = "recording"
                elif event == "save":
                    if self._state == "recording":
                        self._saved += 1
                    self._state = "idle"
                elif event == "discard":
                    if self._state == "recording":
                        self._discarded += 1
                    self._state = "idle"
                self._last_event = event
                # Snapshot under the mutation's lock so the event matches the state.
                status = self._snapshot(event, ts)
            return self._emit(status)

    def _snapshot(self, last_event: EpisodeEvent, ts: float) -> EpisodeStatus:
        """Build a status from current state. Caller must hold `self._lock`."""
        return EpisodeStatus(
            ts=ts,
            state=self._state,
            episodes_saved=self._saved,
            episodes_discarded=self._discarded,
            last_event=last_event,
            task_label=self.config.task,
        )

    def _emit(self, status: EpisodeStatus) -> EpisodeStatus:
        """Publish + log a snapshot. Must run outside the lock (does I/O)."""
        self.status.publish(status)
        self._log_status(status)
        return status

    def _log_status(self, status: EpisodeStatus) -> None:
        """One-line operator feedback to the terminal on every transition."""
        verb = {
            "start": "▶ RECORDING episode",
            "save": "✓ SAVED episode",
            "discard": "✗ DISCARDED episode",
            "init": "· ready",
        }.get(status.last_event, status.last_event)
        label = f" [{status.task_label}]" if status.task_label else ""
        logger.info(
            "[collect] %s%s  (state=%s  saved=%d  discarded=%d)",
            verb,
            label,
            status.state,
            status.episodes_saved,
            status.episodes_discarded,
        )
