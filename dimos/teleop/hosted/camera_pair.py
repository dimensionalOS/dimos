#!/usr/bin/env python3
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

"""Pick which stereo pair the operator sees, from a controller button.

``CameraMuxModule`` composites exactly two inputs side by side, which is one
pair. A robot with more than one pair -- head and wrist, say -- needs
something upstream to choose, and choosing has to be possible from the
headset: an operator whose hands are busy cannot reach the console.

Each eye is fed from the matching side of the selected pair, so the operator
never gets one eye off the head and the other off a wrist.

A side whose camera has gone quiet falls back to the other pair's camera for
that same side, rather than going black. That is not cosmetic: on a real link
a starved video track takes the operator's unreliable command channel down
with it, and the button needed to switch back rides that channel. Showing a
stale-but-live view keeps the operator in control of the robot.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image
from dimos.teleop.quest.quest_types import BUTTON_ALIASES, Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_A, _B = 0, 1
_LEFT, _RIGHT = 0, 1


class CameraPairSelectConfig(ModuleConfig):
    # Face-button label (A/B/X/Y) or a raw Buttons attribute. Toggles on the
    # press edge, so a held button switches once rather than strobing.
    toggle_button: str = "B"
    # Names for the log lines, so an operator reading them knows the view.
    pair_a_name: str = "a"
    pair_b_name: str = "b"
    # A camera with no frame this recently is treated as dead, and that eye
    # falls back to the other pair.
    stale_after_s: float = 2.0


class CameraPairSelectModule(Module):
    """Forward one of two stereo pairs to the mux, toggled from the controller."""

    config: CameraPairSelectConfig

    pair_a_left: In[Image]
    pair_a_right: In[Image]
    pair_b_left: In[Image]
    pair_b_right: In[Image]
    teleop_buttons: In[Buttons]

    cam1: Out[Image]
    cam2: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._selected = _A
        self._was_pressed = False
        # [pair][side] monotonic stamp of the last frame seen.
        self._last_frame = [[0.0, 0.0], [0.0, 0.0]]
        self._fallback_logged = [False, False]
        self._lock = threading.Lock()

    @property
    def _button(self) -> str:
        return BUTTON_ALIASES.get(self.config.toggle_button, self.config.toggle_button)

    def _name(self, pair: int) -> str:
        return self.config.pair_b_name if pair == _B else self.config.pair_a_name

    @rpc
    def start(self) -> None:
        super().start()
        for stream, pair, side in (
            (self.pair_a_left, _A, _LEFT),
            (self.pair_a_right, _A, _RIGHT),
            (self.pair_b_left, _B, _LEFT),
            (self.pair_b_right, _B, _RIGHT),
        ):
            self.register_disposable(
                Disposable(
                    stream.subscribe(
                        lambda msg, pair=pair, side=side: self._forward(msg, pair, side)
                    )
                )
            )
        self.register_disposable(Disposable(self.teleop_buttons.subscribe(self._on_buttons)))
        logger.info(
            "camera pair toggle on %s; showing %s",
            self.config.toggle_button,
            self.config.pair_a_name,
        )

    def _source_for(self, side: int, now: float) -> int:
        """Which pair should feed this eye right now.

        The selected pair, unless its camera on this side has gone quiet and
        the other pair's has not.
        """
        selected = self._selected
        if now - self._last_frame[selected][side] <= self.config.stale_after_s:
            return selected
        other = _B if selected == _A else _A
        if now - self._last_frame[other][side] <= self.config.stale_after_s:
            return other
        return selected

    def _forward(self, msg: Image, pair: int, side: int) -> None:
        """Publish a frame only when its pair is the live source for that eye.

        Arrival is recorded for every pair, selected or not, so the fallback
        knows which cameras are actually alive. Nothing is scheduled: the
        fallback rides the frames that are still arriving.
        """
        with self._lock:
            now = time.monotonic()
            self._last_frame[pair][side] = now
            source = self._source_for(side, now)
            if source != pair:
                return
            fell_back = source != self._selected
            announce = fell_back and not self._fallback_logged[side]
            self._fallback_logged[side] = fell_back
            names = (self._name(self._selected), self._name(source))
        if announce:
            logger.warning(
                "%s camera on the %s pair is quiet — showing %s for that eye",
                "left" if side == _LEFT else "right",
                names[0],
                names[1],
            )
        (self.cam1 if side == _LEFT else self.cam2).publish(msg)

    def _on_buttons(self, msg: Buttons) -> None:
        try:
            pressed = bool(getattr(msg, self._button))
        except AttributeError:
            logger.warning("unknown toggle button %r", self.config.toggle_button)
            return
        with self._lock:
            edge = pressed and not self._was_pressed
            self._was_pressed = pressed
            if not edge:
                return
            self._selected = _B if self._selected == _A else _A
            self._fallback_logged = [False, False]
            showing = self._name(self._selected)
        logger.info("camera pair → %s", showing)
