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

"""``EpisodeBoundary`` — dispatch a Quest controller button edge to the
``RerunDataRecorder``'s ``rotate_recording`` RPC.

Keeping teleop semantics (which Buttons field, debounce) out of the generic
recording module — the recorder stays reusable across non-teleop blueprints
while the rollover trigger lives next to the rest of `dimos/teleop/quest/`.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.manipulation.data_collection.recorder import RerunDataRecorder
from dimos.teleop.quest.quest_types import Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class EpisodeBoundaryConfig(ModuleConfig):
    """Configuration for :class:`EpisodeBoundary`.

    ``button`` names a digital field on :class:`Buttons`. The default is
    ``right_secondary`` (the B button on the right controller). **Do not pick
    ``right_primary`` (A)** — that is the press-and-hold engage button used by
    `TeleopIKTask` and `PinkTeleopTask` to start tracking, so reusing it
    would rotate the recording every time the operator engages. B is on the
    same hand as the trigger (so the operator already has that hand
    committed) but a deliberate reach away from A.
    """

    button: str = "right_secondary"
    debounce_seconds: float = 0.5


class EpisodeBoundary(Module):
    config: EpisodeBoundaryConfig

    buttons: In[Buttons]
    recorder: RerunDataRecorder

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._prev_pressed: bool = False
        self._last_rotate_at: float = 0.0
        self._edge_lock: threading.Lock = threading.Lock()

    def _on_buttons(self, msg: Buttons) -> None:
        try:
            pressed = bool(getattr(msg, self.config.button))
        except AttributeError:
            logger.warning(
                "EpisodeBoundary: configured button %r not found on Buttons",
                self.config.button,
            )
            return

        with self._edge_lock:
            rising = pressed and not self._prev_pressed
            self._prev_pressed = pressed
            if not rising:
                return
            now = time.monotonic()
            if now - self._last_rotate_at < self.config.debounce_seconds:
                return
            self._last_rotate_at = now

        try:
            new_path = self.recorder.toggle_recording()
        except Exception:
            logger.exception("EpisodeBoundary: toggle_recording() raised")
            return
        if new_path is None:
            logger.info("EpisodeBoundary: recording stopped — waiting for next press to resume")
        else:
            logger.info("EpisodeBoundary: recording started → %s", new_path)

    @rpc
    def start(self) -> None:
        super().start()
        unsub = self.buttons.subscribe(self._on_buttons)
        self.register_disposable(Disposable(unsub))


__all__ = ["EpisodeBoundary", "EpisodeBoundaryConfig"]
