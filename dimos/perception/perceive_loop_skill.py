# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

import json
from threading import RLock
from typing import TYPE_CHECKING

from langchain_core.messages import HumanMessage

from dimos.agents.agent import AgentSpec
from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.models.vl.create import create
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.Image import sharpness_window
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

if TYPE_CHECKING:
    from reactivex.abc import DisposableBase

    from dimos.core.global_config import GlobalConfig
    from dimos.models.vl.base import VlModel

logger = setup_logger()


class PerceiveLoopSkill(Module):
    color_image: In[Image]

    _agent_spec: AgentSpec
    _period: float = 0.5  # seconds - how often to run the perceive loop

    def __init__(
        self,
        cfg: GlobalConfig,
    ) -> None:
        super().__init__()
        self._global_config: GlobalConfig = cfg
        self._vl_model: VlModel = create(cfg.detection_model)
        self._active_lookout: tuple[str, ...] = ()
        self._lookout_subscription: DisposableBase | None = None
        self._model_started: bool = False
        self._lock = RLock()

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        self._stop_lookout()
        super().stop()

    @skill
    def look_out_for(self, description_of_things: list[str]) -> str:
        """This tool will continuously look out for things matching the
        description in the input list, and notify the agent whenever it finds a
        match. After the match is found, it will stop.

        You can ask it for `look_out_for(["small dogs", "cats"])` and you will be
        notified back whenever such a detection is made.
        """

        with self._lock:
            if self._active_lookout:
                return (
                    f"Already looking for something else ({self._active_lookout}). "
                    "Cancel the current lookout with the `stop_looking_out` tool"
                )

            sharpest = backpressure(
                sharpness_window(1.0 / self._period, self.color_image.pure_observable())
            )
            self._vl_model.start()
            self._model_started = True
            self._active_lookout = tuple(description_of_things)
            self._lookout_subscription = sharpest.subscribe(
                on_next=self._on_image,
                on_error=lambda e: logger.exception("Error in perceive loop", exc_info=e),
            )

        return (
            f"Started looking for {json.dumps(description_of_things)}. This will "
            "run continuously until you stop it by calling the `stop_looking_out` "
            "tool. Note that it can be intensive, so please cancel when you don't "
            "need to use it in order to save resources."
        )

    @skill
    def stop_looking_out(self) -> str:
        """Stop looking out. Use this to end `look_out_for` tool calls."""
        with self._lock:
            active_lookout_str = json.dumps(self._active_lookout)
            self._stop_lookout()
        return f"Stopped looking out for {active_lookout_str}"

    def _on_image(self, image: Image) -> None:
        with self._lock:
            if not self._active_lookout:
                return
            active_lookout = self._active_lookout
            active_lookout_str = json.dumps(active_lookout)

        detections = self._vl_model.query_detections(image, active_lookout_str)
        if not detections:
            return

        with self._lock:
            if not self._active_lookout:
                return
            if self._lookout_subscription is not None:
                self._lookout_subscription.dispose()
                self._lookout_subscription = None
            self._active_lookout = ()
            self._vl_model.stop()
            self._model_started = False

        self._agent_spec.add_message(
            HumanMessage(f"Found a match for {active_lookout_str}. Please announce audibly.")
        )

    def _stop_lookout(self) -> None:
        with self._lock:
            if self._lookout_subscription is not None:
                self._lookout_subscription.dispose()
                self._lookout_subscription = None
            self._active_lookout = ()
            if self._model_started:
                self._vl_model.stop()
                self._model_started = False
