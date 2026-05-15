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

"""``RolloutToggle`` — dispatch a Quest controller button edge to the
``PolicyModule``'s ``start_rollout`` / ``stop_rollout`` RPCs.

Mirrors `dimos.teleop.quest.episode_boundary.EpisodeBoundary`: a tiny
module that watches one configurable digital `Buttons` field for a
debounced rising edge and toggles a sibling module's state. Co-located
under `dimos/manipulation/policy/` because it imports `PolicyModule` —
keeping the dependency one-directional.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.manipulation.policy.module import PolicyModule
from dimos.teleop.quest.quest_types import Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class RolloutToggleConfig(ModuleConfig):
    """Configuration for :class:`RolloutToggle`.

    ``button`` names a digital field on :class:`Buttons`. The default is
    ``left_secondary`` (the Y button on the left controller). **Do not pick
    ``right_primary`` (A)** — that is the press-and-hold engage button used by
    teleop tasks. ``right_secondary`` (B) is taken by `EpisodeBoundary` for
    data-collection rollover. ``*_trigger`` is taken by gripper control. Y is
    on the opposite hand from the engage / trigger so the operator can hit it
    without disturbing teleop state.
    """

    button: str = "left_secondary"
    debounce_seconds: float = 0.5


class RolloutToggle(Module):
    """Toggle a `PolicyModule`'s rollout state on a configurable button edge."""

    config: RolloutToggleConfig

    buttons: In[Buttons]
    policy_module: PolicyModule

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._prev_pressed: bool = False
        self._last_toggle_at: float = 0.0
        self._unknown_button_logged: bool = False
        self._edge_lock: threading.Lock = threading.Lock()

    def _on_buttons(self, msg: Buttons) -> None:
        try:
            pressed = bool(getattr(msg, self.config.button))
        except AttributeError:
            if not self._unknown_button_logged:
                logger.warning(
                    "RolloutToggle: configured button %r not found on Buttons",
                    self.config.button,
                )
                self._unknown_button_logged = True
            return

        with self._edge_lock:
            rising = pressed and not self._prev_pressed
            self._prev_pressed = pressed
            if not rising:
                return
            now = time.monotonic()
            if now - self._last_toggle_at < self.config.debounce_seconds:
                return
            self._last_toggle_at = now

        try:
            if self.policy_module.is_rollout_active():
                self.policy_module.stop_rollout()
                logger.info("RolloutToggle: rollout stopped")
            else:
                self.policy_module.start_rollout()
                logger.info("RolloutToggle: rollout started")
        except Exception as exc:
            # structlog drops `logger.exception`'s traceback in our default
            # format — include the exception class+message in the message
            # itself so live debugging isn't blind.
            logger.exception(f"RolloutToggle: dispatch raised: {type(exc).__name__}: {exc}")

    @rpc
    def start(self) -> None:
        super().start()
        unsub = self.buttons.subscribe(self._on_buttons)
        self.register_disposable(Disposable(unsub))


__all__ = ["RolloutToggle", "RolloutToggleConfig"]
