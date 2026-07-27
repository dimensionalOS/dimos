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

"""Single point of teleop-input → EpisodeStatus translation.

Watches buttons / keyboard, runs the start/save/discard state machine,
publishes EpisodeStatus on every transition. RecordReplay (or whatever
records the bus) captures that stream into session.db; DataPrep reads
only the recorded EpisodeStatus events offline — never raw buttons or
keypresses.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Literal, TypeAlias

from pydantic import BaseModel
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.teleop.quest.quest_types import BUTTON_ALIASES, Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# A button/keyboard press requests one of these; `toggle` resolves to
# `start`/`save` based on the current state, so it never reaches the output.
EpisodeCommand: TypeAlias = Literal["start", "save", "discard", "toggle"]
# What gets published as `EpisodeStatus.last_event` (`init` on boot).
EpisodeEvent: TypeAlias = Literal["start", "save", "discard", "init"]
RecordingState: TypeAlias = Literal["idle", "recording"]


class EpisodeStatus(BaseModel):
    ts: float
    state: RecordingState
    episodes_saved: int
    episodes_discarded: int
    last_event: EpisodeEvent = "init"
    task_label: str | None = None


class KeyPress(BaseModel):
    """Single keypress event from a keyboard input source."""

    key: str
    ts: float


class EpisodeMonitorModuleConfig(ModuleConfig):
    button_map: dict[EpisodeCommand, str] = {
        "toggle": "B",
        "discard": "Y",
    }
    keyboard_map: dict[EpisodeCommand, str] = {}
    default_task_label: str | None = None


class EpisodeMonitorModule(Module):
    config: EpisodeMonitorModuleConfig

    teleop_buttons: In[Buttons]
    # TODO: no KeyPress producer exists yet — add a pygame keyboard module that
    # publishes KeyPress so this port is actually fed (today only buttons drive it).
    keyboard: In[KeyPress]
    status: Out[EpisodeStatus]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._state: RecordingState = "idle"
        self._saved: int = 0
        self._discarded: int = 0
        self._prev_bits: dict[str, bool] = {}  # rising-edge detection for buttons
        self._lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        # Registered so the base Module.stop() disposes them on shutdown.
        self.register_disposable(Disposable(self.teleop_buttons.subscribe(self._on_buttons)))
        self.register_disposable(Disposable(self.keyboard.subscribe(self._on_keyboard)))
        # Emit an initial idle status so subscribers (and recorders) have a
        # known starting point in the timeline.
        with self._lock:
            status = self._snapshot("init", time.time())
        self._emit(status)

    @rpc
    def reset_counters(self) -> EpisodeStatus:
        with self._lock:
            self._state = "idle"
            self._saved = 0
            self._discarded = 0
            self._prev_bits = {}
            status = self._snapshot("init", time.time())
        return self._emit(status)

    # ── port handlers ────────────────────────────────────────────────────────

    def _on_buttons(self, msg: Buttons) -> None:
        """Rising-edge detect against `config.button_map`; advance state machine."""
        ts = time.time()
        # Edge-detect under the lock (it shares `_prev_bits` with reset_counters),
        # then fire transitions outside it — `_transition` takes the same lock.
        fired: list[EpisodeCommand] = []
        with self._lock:
            for event_name, alias_or_attr in self.config.button_map.items():
                attr = BUTTON_ALIASES.get(alias_or_attr, alias_or_attr)
                try:
                    pressed = bool(getattr(msg, attr))
                except AttributeError:
                    continue
                prev = self._prev_bits.get(attr, False)
                self._prev_bits[attr] = pressed
                if pressed and not prev:  # rising edge
                    fired.append(event_name)
        for event_name in fired:
            self._transition(event_name, ts)

    def _on_keyboard(self, msg: KeyPress) -> None:
        """Match `msg.key` against `config.keyboard_map`; advance state machine."""
        for event_name, key in self.config.keyboard_map.items():
            if msg.key == key:
                self._transition(event_name, msg.ts)
                break

    def _transition(self, event: EpisodeCommand, ts: float) -> None:
        """State-machine transition. Publishes EpisodeStatus on every change.

        ``toggle`` resolves to ``start`` when idle and ``save`` when recording,
        so one button can begin and end a take. The resolved event is what gets
        published (DataPrep only ever sees start/save/discard).
        """
        with self._lock:
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
            # Snapshot under the mutation's lock so the event matches the state.
            status = self._snapshot(event, ts)
        self._emit(status)

    def _snapshot(self, last_event: EpisodeEvent, ts: float) -> EpisodeStatus:
        """Build a status from current state. Caller must hold `self._lock`."""
        return EpisodeStatus(
            ts=ts,
            state=self._state,
            episodes_saved=self._saved,
            episodes_discarded=self._discarded,
            last_event=last_event,
            task_label=self.config.default_task_label,
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
