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

"""DimSim-specific Go2 camera observation semantics."""

from __future__ import annotations

from threading import Condition
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.go2.connection import GO2Connection

_FRESH_FRAME_TIMEOUT_SEC = 5.0


class DimSimGO2Connection(GO2Connection):
    """Go2 connection whose observation waits for a post-request DimSim frame."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._init_fresh_frame_state()

    def _init_fresh_frame_state(self) -> None:
        self._fresh_frame_condition = Condition()
        self._latest_dimsim_frame: Image | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.color_image.subscribe(self._on_dimsim_frame)),
        )

    def _on_dimsim_frame(self, image: Image) -> None:
        with self._fresh_frame_condition:
            if self._latest_dimsim_frame is not None and image.ts <= self._latest_dimsim_frame.ts:
                return
            self._latest_dimsim_frame = image
            self._fresh_frame_condition.notify_all()

    def _wait_for_frame_after(self, timestamp: float, timeout: float) -> Image | None:
        deadline = time.monotonic() + timeout
        with self._fresh_frame_condition:
            while self._latest_dimsim_frame is None or self._latest_dimsim_frame.ts <= timestamp:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._fresh_frame_condition.wait(remaining)
            return self._latest_dimsim_frame

    @skill
    def observe(self) -> Image | None:
        """Return a camera frame captured after this observation request.

        DimSim rendering and perception can lag behind robot movement. Waiting
        for a frame with a newer sensor timestamp prevents the agent from
        steering from a camera pose that predates its latest movement.
        Returns None if no fresh frame arrives within five seconds.
        """
        return self._wait_for_frame_after(
            time.time(),
            _FRESH_FRAME_TIMEOUT_SEC,
        )
