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

from __future__ import annotations

from dataclasses import dataclass
from threading import Condition, Event, Thread
import time
from typing import Any, Self

from pydantic import Field, model_validator
from reactivex.disposable import Disposable

from dimos.control.safety.guardrail_policy import (
    GuardrailDecision,
    GuardrailHealth,
    GuardrailPolicy,
    GuardrailState,
    OpticalFlowMagnitudeGuardrailPolicy,
    OpticalFlowMagnitudePolicyConfig,
)
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

_THREAD_JOIN_TIMEOUT_S = 2.0


logger = setup_logger()


class RGBCollisionGuardrailConfig(ModuleConfig):
    # Scheduling
    guarded_output_publish_hz: float = Field(default=10.0, gt=0.0)
    risk_evaluation_hz: float = Field(default=10.0, gt=0.0)

    # Freshness and fail-closed behavior
    command_timeout_s: float = Field(default=0.25, gt=0.0)
    image_timeout_s: float = Field(default=0.25, gt=0.0)
    risk_timeout_s: float = Field(default=0.25, gt=0.0)
    fail_closed_on_missing_image: bool = True
    publish_zero_on_stop: bool = True
    frame_pair_max_gap_s: float = Field(default=0.2, gt=0.0)

    # Motion gating
    forward_motion_deadband_mps: float = Field(default=0.05, ge=0.0)
    clamp_forward_speed_mps: float = Field(default=0.1, ge=0.0)

    # Forward ROI geometry
    flow_downsample_width_px: int = Field(default=160, ge=32)
    forward_roi_top_fraction: float = Field(default=0.45, ge=0.0, le=1.0)
    forward_roi_bottom_fraction: float = Field(default=0.95, ge=0.0, le=1.0)
    forward_roi_width_fraction: float = Field(default=0.5, gt=0.0, le=1.0)

    # Image-quality checks
    low_texture_variance_threshold: float = Field(default=150.0, ge=0.0)
    occlusion_dark_pixel_threshold: int = Field(default=20, ge=0, le=255)
    occlusion_bright_pixel_threshold: int = Field(default=235, ge=0, le=255)
    occlusion_extreme_fraction_threshold: float = Field(default=0.9, ge=0.0, le=1.0)

    # Flow thresholds and hysteresis
    caution_flow_magnitude_threshold: float = Field(default=0.8, ge=0.0)
    stop_flow_magnitude_threshold: float = Field(default=1.5, ge=0.0)
    caution_frame_count: int = Field(default=2, ge=1)
    stop_frame_count: int = Field(default=2, ge=1)
    clear_frame_count: int = Field(default=3, ge=1)
    stop_release_frame_count: int = Field(default=2, ge=1)

    @model_validator(mode="after")
    def validate_thresholds(self) -> Self:
        if self.forward_roi_top_fraction >= self.forward_roi_bottom_fraction:
            raise ValueError(
                "forward_roi_top_fraction must be less than forward_roi_bottom_fraction"
            )

        if self.occlusion_dark_pixel_threshold >= self.occlusion_bright_pixel_threshold:
            raise ValueError(
                "occlusion_dark_pixel_threshold must be less than occlusion_bright_pixel_threshold"
            )

        if self.caution_flow_magnitude_threshold > self.stop_flow_magnitude_threshold:
            raise ValueError(
                "caution_flow_magnitude_threshold must be less than or equal to "
                "stop_flow_magnitude_threshold"
            )

        return self


@dataclass
class _GuardrailRuntimeState:
    latest_image: Image | None = None
    previous_image: Image | None = None
    latest_image_time: float | None = None
    previous_image_time: float | None = None
    latest_cmd_vel: Twist | None = None
    latest_cmd_time: float | None = None
    last_decision: GuardrailDecision | None = None
    last_risk_time: float | None = None
    last_publish_time: float | None = None
    next_risk_time: float | None = None
    pending_cmd_update: bool = False
    pending_decision_publish: bool = False
    state: GuardrailState = GuardrailState.INIT
    image_generation: int = 0
    last_evaluated_image_generation: int = -1


@dataclass(frozen=True)
class _RiskEvaluationInput:
    previous_image: Image
    current_image: Image
    incoming_cmd_vel: Twist
    health: GuardrailHealth


class RGBCollisionGuardrail(Module[RGBCollisionGuardrailConfig]):
    """RGB-only motion guardrail for direct Twist control."""

    default_config = RGBCollisionGuardrailConfig

    color_image: In[Image]
    incoming_cmd_vel: In[Twist]
    safe_cmd_vel: Out[Twist]

    _condition: Condition
    _runtime_state: _GuardrailRuntimeState
    _stop_event: Event
    _thread: Thread | None
    _policy: GuardrailPolicy

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._condition = Condition()
        self._runtime_state = _GuardrailRuntimeState()
        self._stop_event = Event()
        self._thread = None
        self._policy = self._build_policy()

    def _build_policy(self) -> GuardrailPolicy:
        policy_config = OpticalFlowMagnitudePolicyConfig(
            forward_motion_deadband_mps=self.config.forward_motion_deadband_mps,
            clamp_forward_speed_mps=self.config.clamp_forward_speed_mps,
            flow_downsample_width_px=self.config.flow_downsample_width_px,
            forward_roi_top_fraction=self.config.forward_roi_top_fraction,
            forward_roi_bottom_fraction=self.config.forward_roi_bottom_fraction,
            forward_roi_width_fraction=self.config.forward_roi_width_fraction,
            low_texture_variance_threshold=self.config.low_texture_variance_threshold,
            occlusion_dark_pixel_threshold=self.config.occlusion_dark_pixel_threshold,
            occlusion_bright_pixel_threshold=self.config.occlusion_bright_pixel_threshold,
            occlusion_extreme_fraction_threshold=self.config.occlusion_extreme_fraction_threshold,
            caution_flow_magnitude_threshold=self.config.caution_flow_magnitude_threshold,
            stop_flow_magnitude_threshold=self.config.stop_flow_magnitude_threshold,
            caution_frame_count=self.config.caution_frame_count,
            stop_frame_count=self.config.stop_frame_count,
            clear_frame_count=self.config.clear_frame_count,
            stop_release_frame_count=self.config.stop_release_frame_count,
        )
        return OpticalFlowMagnitudeGuardrailPolicy(policy_config)

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()

        with self._condition:
            self._runtime_state.next_risk_time = time.monotonic()

        self._disposables.add(Disposable(self.color_image.subscribe(self._on_color_image)))
        self._disposables.add(
            Disposable(self.incoming_cmd_vel.subscribe(self._on_incoming_cmd_vel))
        )

        self._thread = Thread(
            target=self._decision_loop,
            name=f"{self.__class__.__name__}-thread",
            daemon=True,
        )
        self._thread.start()

        logger.info(
            "RGB guardrail started",
            risk_evaluation_hz=self.config.risk_evaluation_hz,
            guarded_output_publish_hz=self.config.guarded_output_publish_hz,
            command_timeout_s=self.config.command_timeout_s,
            image_timeout_s=self.config.image_timeout_s,
            risk_timeout_s=self.config.risk_timeout_s,
            frame_pair_max_gap_s=self.config.frame_pair_max_gap_s,
        )

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()

        if self.config.publish_zero_on_stop:
            self.safe_cmd_vel.publish(Twist.zero())

        if self._thread is not None:
            self._thread.join(timeout=_THREAD_JOIN_TIMEOUT_S)
            if self._thread.is_alive():
                logger.warning(
                    "RGB guardrail worker thread did not stop within timeout",
                    timeout_s=_THREAD_JOIN_TIMEOUT_S,
                )
            self._thread = None

        logger.info("RGB guardrail stopped")
        super().stop()

    def _on_color_image(self, image: Image) -> None:
        now = time.monotonic()
        with self._condition:
            self._runtime_state.previous_image = self._runtime_state.latest_image
            self._runtime_state.previous_image_time = self._runtime_state.latest_image_time
            self._runtime_state.latest_image = image
            self._runtime_state.latest_image_time = now
            self._runtime_state.image_generation += 1
            self._condition.notify()

    def _on_incoming_cmd_vel(self, cmd_vel: Twist) -> None:
        now = time.monotonic()
        with self._condition:
            self._runtime_state.latest_cmd_vel = cmd_vel
            self._runtime_state.latest_cmd_time = now
            self._runtime_state.pending_cmd_update = True
            self._condition.notify()

    def _decision_loop(self) -> None:
        while not self._stop_event.is_set():
            risk_input: _RiskEvaluationInput | None = None

            with self._condition:
                timeout_s = self._next_wakeup_timeout_locked()
                self._condition.wait(timeout=timeout_s)

                if self._stop_event.is_set():
                    return

                now = time.monotonic()
                if self._should_recompute_risk_locked(now):
                    risk_input = self._take_risk_evaluation_input_locked(now)

            # Evaluate a consistent snapshot outside the condition lock so image
            # and command callbacks stay cheap. If newer inputs arrive while
            # evaluation runs, the next wakeup will process that fresher snapshot.
            policy_decision: GuardrailDecision | None = None
            if risk_input is not None:
                try:
                    policy_decision = self._policy.evaluate(
                        previous_image=risk_input.previous_image,
                        current_image=risk_input.current_image,
                        incoming_cmd_vel=risk_input.incoming_cmd_vel,
                        health=risk_input.health,
                    )
                except Exception:
                    with self._condition:
                        _logged_state = self._runtime_state.state.value

                    logger.exception(
                        "RGB guardrail policy evaluation failed",
                        state=_logged_state,
                    )
                    policy_decision = self._build_sensor_degraded_decision(
                        "policy_evaluation_failed"
                    )

            cmd_vel_to_publish: Twist | None = None
            publish_time: float | None = None

            with self._condition:
                now = time.monotonic()

                if policy_decision is not None:
                    self._store_decision_locked(
                        policy_decision,
                        now,
                        update_risk_time=True,
                    )
                else:
                    fallback_decision = self._select_fallback_decision_locked(now)
                    if fallback_decision is not None:
                        self._store_decision_locked(
                            fallback_decision,
                            now,
                            update_risk_time=False,
                        )

                cmd_vel_to_publish = self._consume_publish_cmd_locked(now)
                if cmd_vel_to_publish is not None:
                    publish_time = now

            if cmd_vel_to_publish is not None and publish_time is not None:
                self.safe_cmd_vel.publish(cmd_vel_to_publish)
                with self._condition:
                    self._runtime_state.last_publish_time = publish_time

    def _guarded_output_publish_period_s(self) -> float:
        return 1.0 / self.config.guarded_output_publish_hz

    def _risk_evaluation_period_s(self) -> float:
        return 1.0 / self.config.risk_evaluation_hz

    def _should_recompute_risk_locked(self, now: float) -> bool:
        next_risk_time = self._runtime_state.next_risk_time
        if next_risk_time is not None and now < next_risk_time:
            return False

        if self._runtime_state.previous_image is None:
            return False

        return (
            self._runtime_state.image_generation
            != self._runtime_state.last_evaluated_image_generation
        )

    def _is_cmd_fresh_locked(self, now: float) -> bool:
        latest_cmd_time = self._runtime_state.latest_cmd_time
        if latest_cmd_time is None:
            return False
        return (now - latest_cmd_time) <= self.config.command_timeout_s

    def _is_image_fresh_locked(self, now: float) -> bool:
        latest_image_time = self._runtime_state.latest_image_time
        if latest_image_time is None:
            return False
        return (now - latest_image_time) <= self.config.image_timeout_s

    def _is_frame_pair_fresh_locked(self) -> bool:
        previous_image_time = self._runtime_state.previous_image_time
        latest_image_time = self._runtime_state.latest_image_time
        if previous_image_time is None or latest_image_time is None:
            return False
        return (latest_image_time - previous_image_time) <= self.config.frame_pair_max_gap_s

    def _is_risk_fresh_locked(self, now: float) -> bool:
        last_risk_time = self._runtime_state.last_risk_time
        if last_risk_time is None:
            return False
        return (now - last_risk_time) <= self.config.risk_timeout_s

    def _build_health_locked(self, now: float) -> GuardrailHealth:
        return GuardrailHealth(
            has_previous_frame=self._runtime_state.previous_image is not None,
            image_fresh=self._is_image_fresh_locked(now),
            cmd_fresh=self._is_cmd_fresh_locked(now),
            risk_fresh=self._is_risk_fresh_locked(now),
            frame_pair_fresh=self._is_frame_pair_fresh_locked(),
        )

    def _resolved_cmd_for_latest_locked(self, now: float) -> Twist:
        latest_cmd_vel = self._runtime_state.latest_cmd_vel
        if latest_cmd_vel is None:
            return Twist.zero()

        if not self._is_cmd_fresh_locked(now):
            return Twist.zero()

        last_decision = self._runtime_state.last_decision
        if last_decision is None:
            return Twist.zero()

        if last_decision.state == GuardrailState.PASS:
            return latest_cmd_vel

        return last_decision.cmd_vel

    def _take_risk_evaluation_input_locked(self, now: float) -> _RiskEvaluationInput | None:
        previous_image = self._runtime_state.previous_image
        current_image = self._runtime_state.latest_image
        incoming_cmd_vel = self._runtime_state.latest_cmd_vel

        self._runtime_state.next_risk_time = now + self._risk_evaluation_period_s()

        if previous_image is None or current_image is None or incoming_cmd_vel is None:
            return None

        self._runtime_state.last_evaluated_image_generation = self._runtime_state.image_generation

        return _RiskEvaluationInput(
            previous_image=previous_image,
            current_image=current_image,
            incoming_cmd_vel=incoming_cmd_vel,
            health=self._build_health_locked(now),
        )

    def _store_decision_locked(
        self,
        decision: GuardrailDecision,
        now: float,
        *,
        update_risk_time: bool,
    ) -> None:
        had_previous_decision = self._runtime_state.last_decision is not None
        previous_state = self._runtime_state.state

        self._runtime_state.last_decision = decision
        if update_risk_time:
            self._runtime_state.last_risk_time = now
        self._runtime_state.state = decision.state

        # Request an immediate publish when the policy says so or when the
        # high-level state changes. Freshness checks still apply later.
        if (
            decision.publish_immediately
            or not had_previous_decision
            or previous_state != decision.state
        ):
            self._runtime_state.pending_decision_publish = True

        if previous_state != decision.state:
            logger.info(
                "RGB guardrail state changed",
                previous_state=previous_state.value,
                state=decision.state.value,
                reason=decision.reason,
            )

    def _consume_publish_cmd_locked(self, now: float) -> Twist | None:
        if self._runtime_state.pending_decision_publish:
            self._runtime_state.pending_decision_publish = False
            return self._resolved_cmd_for_latest_locked(now)

        if self._runtime_state.pending_cmd_update:
            self._runtime_state.pending_cmd_update = False
            return self._resolved_cmd_for_latest_locked(now)

        latest_cmd_time = self._runtime_state.latest_cmd_time
        if latest_cmd_time is not None and not self._is_cmd_fresh_locked(now):
            return Twist.zero()

        if self._should_republish_non_pass_output_locked(now):
            last_decision = self._runtime_state.last_decision
            if last_decision is not None:
                return last_decision.cmd_vel

        return None

    def _should_republish_non_pass_output_locked(self, now: float) -> bool:
        last_decision = self._runtime_state.last_decision
        if last_decision is None:
            return False

        if last_decision.state == GuardrailState.PASS:
            return False

        last_publish_time = self._runtime_state.last_publish_time
        if last_publish_time is None:
            return True

        return (now - last_publish_time) >= self._guarded_output_publish_period_s()

    def _next_wakeup_timeout_locked(self) -> float:
        now = time.monotonic()

        if self._runtime_state.pending_cmd_update or self._runtime_state.pending_decision_publish:
            return 0.0

        timeouts: list[float] = [self._risk_evaluation_period_s()]

        next_risk_time = self._runtime_state.next_risk_time
        if next_risk_time is not None:
            timeouts.append(max(next_risk_time - now, 0.0))

        latest_cmd_time = self._runtime_state.latest_cmd_time
        if latest_cmd_time is not None and self._is_cmd_fresh_locked(now):
            timeouts.append(max((latest_cmd_time + self.config.command_timeout_s) - now, 0.0))

        latest_image_time = self._runtime_state.latest_image_time
        if latest_image_time is not None and self._is_image_fresh_locked(now):
            timeouts.append(max((latest_image_time + self.config.image_timeout_s) - now, 0.0))

        last_risk_time = self._runtime_state.last_risk_time
        if last_risk_time is not None and self._is_risk_fresh_locked(now):
            timeouts.append(max((last_risk_time + self.config.risk_timeout_s) - now, 0.0))

        if self._should_republish_non_pass_output_locked(now):
            timeouts.append(0.0)
        else:
            last_decision = self._runtime_state.last_decision
            last_publish_time = self._runtime_state.last_publish_time
            if (
                last_decision is not None
                and last_decision.state != GuardrailState.PASS
                and last_publish_time is not None
            ):
                next_publish_time = last_publish_time + self._guarded_output_publish_period_s()
                timeouts.append(max(next_publish_time - now, 0.0))

        return min(timeouts)

    def _build_zero_decision(
        self,
        state: GuardrailState,
        reason: str,
        *,
        publish_immediately: bool = False,
        risk_score: float = 0.0,
    ) -> GuardrailDecision:
        return GuardrailDecision(
            state=state,
            cmd_vel=Twist.zero(),
            reason=reason,
            risk_score=risk_score,
            publish_immediately=publish_immediately,
        )

    def _build_init_decision(self, reason: str) -> GuardrailDecision:
        return self._build_zero_decision(
            GuardrailState.INIT,
            reason,
            publish_immediately=True,
        )

    def _build_sensor_degraded_decision(self, reason: str) -> GuardrailDecision:
        return self._build_zero_decision(
            GuardrailState.SENSOR_DEGRADED,
            reason,
            publish_immediately=True,
            risk_score=1.0,
        )

    def _select_fallback_decision_locked(self, now: float) -> GuardrailDecision | None:
        if self._runtime_state.latest_cmd_vel is None:
            return self._build_init_decision("no_command_received")

        if self._runtime_state.latest_image is None:
            if self.config.fail_closed_on_missing_image:
                return self._build_init_decision("waiting_for_first_image")
            return None

        if self._runtime_state.previous_image is None:
            if self.config.fail_closed_on_missing_image:
                return self._build_init_decision("waiting_for_frame_pair")
            return None

        if not self._is_image_fresh_locked(now):
            return self._build_sensor_degraded_decision("image_stale")

        if not self._is_frame_pair_fresh_locked():
            return self._build_sensor_degraded_decision("frame_pair_stale")

        if self._runtime_state.last_risk_time is None:
            return self._build_init_decision("waiting_for_first_risk_evaluation")

        if not self._is_risk_fresh_locked(now):
            return self._build_sensor_degraded_decision("risk_state_stale")

        return None


rgb_collision_guardrail = RGBCollisionGuardrail.blueprint
