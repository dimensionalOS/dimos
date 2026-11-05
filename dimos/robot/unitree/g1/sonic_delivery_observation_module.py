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

"""Passive typed-stream adapter for the supervised delivery-freshness profile."""

from collections.abc import AsyncIterator
import threading
from time import monotonic
from typing import Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.robot.unitree.g1.sonic_delivery_observation import (
    SonicCameraDelivery,
    SonicDeliveryPairer,
    SonicJointDelivery,
)
from dimos.robot.unitree.g1.sonic_observation import SonicObservation


class SonicDeliveryObservationConfig(ModuleConfig):
    left_hand_slot_names: tuple[str, ...]
    right_hand_slot_names: tuple[str, ...]
    max_delivery_age_seconds: float = Field(default=0.2, gt=0, le=0.2, allow_inf_nan=False)
    max_delivery_span_seconds: float = Field(gt=0, le=0.2, allow_inf_nan=False)


class SonicDeliveryObservationModule(Module):
    """Pair independent original receiver records; never infer acquisition age.

    Receivers, adapter and service must share a Linux host and monotonic clock.
    Input faults stop output; an already sent snapshot expires at the service's
    configured deadline. This module owns no hardware or controller transport.
    Replace receiver/adapter/service together after a source fault or restart.
    """

    config: SonicDeliveryObservationConfig
    body_delivery: In[SonicJointDelivery]
    left_hand_delivery: In[SonicJointDelivery]
    right_hand_delivery: In[SonicJointDelivery]
    camera_delivery: In[SonicCameraDelivery]
    observation: Out[SonicObservation]

    def __init__(self, **kwargs: Any) -> None:
        self._lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._running = self._receiver_session_started = False
        self._rejections = 0
        self._last_rejection: str | None = None
        super().__init__(**kwargs)
        self._pairer = SonicDeliveryPairer(
            left_hand_slot_names=self.config.left_hand_slot_names,
            right_hand_slot_names=self.config.right_hand_slot_names,
            max_age=self.config.max_delivery_age_seconds,
            max_span=self.config.max_delivery_span_seconds,
        )

    @rpc
    def start(self) -> None:
        with self._lifecycle_lock:
            if self._running:
                return
            if self._receiver_session_started:
                raise RuntimeError("replace receiver/adapter/service session after stop")
            super().start()

    @rpc
    def stop(self) -> None:
        with self._lifecycle_lock:
            super().stop()

    async def main(self) -> AsyncIterator[None]:
        with self._lock:
            self._receiver_session_started = self._running = True
        try:
            yield
        finally:
            with self._lock:
                self._running = False

    def _handle_joint_delivery(self, group: str, sample: SonicJointDelivery) -> None:
        with self._lock:
            if not self._running:
                return
            try:
                if sample.group != group:
                    raise ValueError("joint delivery connected to the wrong component port")
                obs = self._pairer.push_joints(sample, now=monotonic())
                if obs is not None:
                    self.observation.publish(obs)
            except ValueError as exc:
                self._rejections += 1
                self._last_rejection = str(exc)

    @rpc
    async def handle_body_delivery(self, sample: SonicJointDelivery) -> None:
        """Accept one body delivery with its original local receipt time."""
        self._handle_joint_delivery("body", sample)

    @rpc
    async def handle_left_hand_delivery(self, sample: SonicJointDelivery) -> None:
        """Accept one left-hand delivery with its original local receipt time."""
        self._handle_joint_delivery("left_hand", sample)

    @rpc
    async def handle_right_hand_delivery(self, sample: SonicJointDelivery) -> None:
        """Accept one right-hand delivery with its original local receipt time."""
        self._handle_joint_delivery("right_hand", sample)

    @rpc
    async def handle_camera_delivery(self, sample: SonicCameraDelivery) -> None:
        """Accept an owned RGB image with its preserved local receipt time."""
        with self._lock:
            if not self._running:
                return
            try:
                obs = self._pairer.push_camera(sample, now=monotonic())
                if obs is not None:
                    self.observation.publish(obs)
            except ValueError as exc:
                self._rejections += 1
                self._last_rejection = str(exc)

    @rpc
    def get_observation_status(self) -> dict[str, Any]:
        """Report delivery freshness/provenance; acquisition age/skew stay unknown."""
        with self._lock:
            status = self._pairer.get_status(now=monotonic())
            if not self._running:
                status["state"] = "stopped"
            return status | {
                "rejected_deliveries": self._rejections,
                "last_rejection": self._last_rejection,
            }
