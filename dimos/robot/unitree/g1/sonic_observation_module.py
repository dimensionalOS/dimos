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

"""Read-only camera/state diagnostics and acquisition-timestamped observations."""

from collections.abc import AsyncIterator
import math
import threading
from time import monotonic, time as wall_time
from typing import Any, Self

from pydantic import Field, model_validator

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.unitree.g1.sonic_feedback import FeedbackBuffer, SonicFeedback
from dimos.robot.unitree.g1.sonic_observation import (
    POLICY_JOINT_NAMES,
    SonicObservation,
    policy_joint_state,
)
from dimos.robot.unitree.g1.sonic_observation_pairing import (
    SonicMeasuredState,
    SonicObservationPairer,
)


class SonicObservationConfig(ModuleConfig):
    camera_frame_id: str = Field(min_length=1)
    # Explicit commissioning budget; this does not assert hardware synchronization.
    max_skew_seconds: float = Field(gt=0, le=0.2, allow_inf_nan=False)
    camera_timestamp_error_seconds: float = Field(ge=0, le=0.1, allow_inf_nan=False)
    left_hand_slot_names: tuple[str, ...]
    right_hand_slot_names: tuple[str, ...]
    max_age_seconds: float = Field(default=0.2, gt=0, le=0.2, allow_inf_nan=False)
    clock_change_limit_seconds: float = Field(default=0.01, gt=0, le=0.02, allow_inf_nan=False)

    @model_validator(mode="after")
    def validate_mapping(self) -> Self:
        for names, expected in (
            (self.left_hand_slot_names, POLICY_JOINT_NAMES[14:21]),
            (self.right_hand_slot_names, POLICY_JOINT_NAMES[21:28]),
        ):
            if len(names) != 7 or set(names) != set(expected):
                raise ValueError("each hand mapping must name its seven policy joints exactly once")
        if self.max_skew_seconds > self.max_age_seconds:
            raise ValueError("pairing skew cannot exceed freshness budget")
        return self


class SonicObservationModule(Module):
    """Run with camera, state producer and service on the same Linux host.

    Existing g1_debug feedback supplies diagnostics only. Its receipt or ROS
    timestamp cannot replace body/hand acquisition times. A separate producer
    must supply measured_state before observations can reach a service.
    This module has no control socket or controller lifecycle operations.
    """

    config: SonicObservationConfig
    color_image: In[Image]
    feedback: In[SonicFeedback]
    measured_state: In[SonicMeasuredState]
    observation: Out[SonicObservation]

    def __init__(self, **kwargs: Any) -> None:
        self._lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        super().__init__(**kwargs)
        self._running = False
        self._reset()

    def _reset(self) -> None:
        self._pairer = SonicObservationPairer(
            max_skew=self.config.max_skew_seconds, max_age=self.config.max_age_seconds
        )
        self._feedback = FeedbackBuffer(stale_after=self.config.max_age_seconds)
        self._clock_offset: float | None = None
        self._clock_uncertainty = 0.0
        self._fault: str | None = None
        self._last_rejection: str | None = None
        self._camera_at: float | None = None
        self._measured_at: float | None = None
        self._observation_at: float | None = None
        self._diagnostic_state: list[float] | None = None
        self._published = self._images = self._states = 0
        self._rejected_images = self._rejected_states = self._rejected_feedback = 0

    def _clock(self) -> float:
        before = monotonic()
        wall = wall_time()
        after = monotonic()
        uncertainty = (after - before) / 2
        offset = wall - (before + after) / 2
        if (
            not all(math.isfinite(t) for t in (before, wall, after))
            or after < before
            or uncertainty > self.config.clock_change_limit_seconds / 2
            or (
                self._clock_offset is not None
                and abs(offset - self._clock_offset) + uncertainty
                > self.config.clock_change_limit_seconds
            )
        ):
            self._fault = "local camera clock mapping changed or is uncertain; replace module/session required"
            self._pairer = SonicObservationPairer(
                max_skew=self.config.max_skew_seconds, max_age=self.config.max_age_seconds
            )
            self._observation_at = None
        elif self._clock_offset is None:
            self._clock_offset = offset
            self._clock_uncertainty = uncertainty
        return after

    @rpc
    def start(self) -> None:
        with self._lifecycle_lock:
            if not self._running:
                super().start()

    @rpc
    def stop(self) -> None:
        with self._lifecycle_lock:
            super().stop()

    def _camera_uncertainty(self) -> float:
        return (
            self._clock_uncertainty
            + self.config.clock_change_limit_seconds
            + self.config.camera_timestamp_error_seconds
        )

    async def main(self) -> AsyncIterator[None]:
        with self._lock:
            self._reset()
            self._clock()
            self._running = True
        try:
            yield
        finally:
            with self._lock:
                self._running = False
                self._reset()

    def _publish_observation(self, obs: SonicObservation | None) -> None:
        if obs is not None:
            self.observation.publish(obs)
            self._published += 1
            self._observation_at = obs.observed_at

    @rpc
    async def handle_color_image(self, image: Image) -> None:
        """Accept a same-host RealSense RGB image with its original UNIX timestamp."""
        with self._lock:
            if not self._running:
                return
            now = self._clock()
            if self._fault is not None:
                return
            try:
                if image.frame_id != self.config.camera_frame_id or image.format != ImageFormat.RGB:
                    raise ValueError("camera frame ID or RGB format mismatch")
                assert self._clock_offset is not None
                stamp = image.ts - self._clock_offset
                obs = self._pairer.push_image(
                    image.data, observed_at=stamp, now=now, uncertainty=self._camera_uncertainty()
                )
                self._camera_at = stamp
                self._images += 1
                self._publish_observation(obs)
            except ValueError as exc:
                self._rejected_images += 1
                self._last_rejection = str(exc)

    @rpc
    async def handle_measured_state(self, state: SonicMeasuredState) -> None:
        """Accept named joints with bounded same-host acquisition estimates."""
        with self._lock:
            if not self._running:
                return
            now = self._clock()
            if self._fault is not None:
                return
            try:
                obs = self._pairer.push_state(state, now=now)
                self._measured_at = min(lower for lower, _ in state.acquisition_bounds)
                self._states += 1
                self._publish_observation(obs)
            except ValueError as exc:
                self._rejected_states += 1
                self._last_rejection = str(exc)

    @rpc
    async def handle_feedback(self, feedback: SonicFeedback) -> None:
        """Record diagnostic joint values and receipt age; never creates observations."""
        with self._lock:
            if not self._running:
                return
            now = self._clock()
            if self._fault is not None:
                return
            try:
                if not 0 <= now - feedback.received_monotonic < self.config.max_age_seconds:
                    raise ValueError("feedback receipt is stale or on another monotonic clock")
                positions = dict(zip(POLICY_JOINT_NAMES[:14], feedback.body_q[15:29], strict=True))
                positions.update(
                    zip(self.config.left_hand_slot_names, feedback.left_hand_q, strict=True)
                )
                positions.update(
                    zip(self.config.right_hand_slot_names, feedback.right_hand_q, strict=True)
                )
                values = policy_joint_state(positions).tolist()
                if not self._feedback.accept(feedback):
                    raise ValueError("feedback index must advance")
                self._diagnostic_state = values
            except ValueError as exc:
                self._rejected_feedback += 1
                self._last_rejection = str(exc)

    @rpc
    def get_observation_status(self) -> dict[str, Any]:
        """Report source freshness and diagnostic pairing without granting control readiness."""
        with self._lock:
            now = self._clock() if self._running else monotonic()
            age = None if self._observation_at is None else now - self._observation_at
            state = (
                "stopped"
                if not self._running
                else "fault"
                if self._fault
                else "waiting"
                if age is None
                else "fresh"
                if 0 <= age < self.config.max_age_seconds
                else "stale"
            )
            feedback = self._feedback.latest(now=now)
            reason = (
                self._fault or "module stopped"
                if not self._running or self._fault
                else "measurement acquisition timestamps unavailable"
                if self._measured_at is None
                else "camera source timestamp unavailable"
                if self._camera_at is None
                else "paired observation fresh"
                if state == "fresh"
                else "waiting for fresh aligned acquisitions"
            )
            return {
                "state": state,
                "readiness_reason": reason,
                "camera_age_s": None if self._camera_at is None else now - self._camera_at,
                "measured_state_age_s": None
                if self._measured_at is None
                else now - self._measured_at,
                "observation_age_s": age,
                "feedback_receipt_age_s": self._feedback.age(now=now),
                "camera_source_to_feedback_receipt_s": None
                if feedback is None or self._camera_at is None
                else feedback.received_monotonic - self._camera_at,
                "diagnostic_state": None if feedback is None else self._diagnostic_state,
                "diagnostic_time_basis": "camera source versus telemetry receipt; acquisition alignment unverified",
                "clock_mapping": "same-host UNIX to monotonic; fixed offset",
                "clock_mapping_uncertainty_s": self._clock_uncertainty,
                "camera_time_uncertainty_s": self._camera_uncertainty(),
                "clock_offset_s": self._clock_offset,
                "max_skew_s": self.config.max_skew_seconds,
                "accepted_images": self._images,
                "accepted_measured_states": self._states,
                "observations_published": self._published,
                "rejected_images": self._rejected_images,
                "rejected_states": self._rejected_states,
                "rejected_feedback": self._rejected_feedback,
                "last_rejection": self._last_rejection,
            }
