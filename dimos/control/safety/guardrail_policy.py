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
from enum import Enum
from typing import Any, Protocol, cast

import cv2
import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image

GrayImage = NDArray[np.uint8]


class GuardrailState(str, Enum):
    INIT = "init"
    PASS = "pass"
    CLAMP = "clamp"
    STOP_LATCHED = "stop_latched"
    SENSOR_DEGRADED = "sensor_degraded"


@dataclass(frozen=True)
class GuardrailHealth:
    has_previous_frame: bool
    image_fresh: bool
    cmd_fresh: bool
    risk_fresh: bool


@dataclass
class GuardrailDecision:
    """Policy result consumed by the guardrail worker.

    publish_immediately requests an immediate worker-side publish on the next
    loop iteration. It does not bypass command freshness checks.
    """

    state: GuardrailState
    cmd_vel: Twist
    reason: str
    risk_score: float = 0.0
    publish_immediately: bool = False


@dataclass(frozen=True)
class OpticalFlowMagnitudePolicyConfig:
    forward_motion_deadband_mps: float
    clamp_forward_speed_mps: float
    flow_downsample_width_px: int
    forward_roi_top_fraction: float
    forward_roi_bottom_fraction: float
    forward_roi_width_fraction: float
    low_texture_variance_threshold: float
    occlusion_dark_pixel_threshold: int
    occlusion_bright_pixel_threshold: int
    occlusion_extreme_fraction_threshold: float
    caution_flow_magnitude_threshold: float
    stop_flow_magnitude_threshold: float
    caution_frame_count: int
    stop_frame_count: int
    clear_frame_count: int


class GuardrailPolicy(Protocol):
    def evaluate(
        self,
        previous_image: Image,
        current_image: Image,
        incoming_cmd_vel: Twist,
        health: GuardrailHealth,
    ) -> GuardrailDecision: ...


class OpticalFlowMagnitudeGuardrailPolicy(GuardrailPolicy):
    """Forward-motion RGB guardrail using flow magnitude in a central lower ROI."""

    # V1 keeps Farneback internals fixed to reduce tuning surface. Promote
    # these to config only after hardware tuning shows they need adjustment.
    _FARNEBACK_PYR_SCALE = 0.5
    _FARNEBACK_LEVELS = 3
    _FARNEBACK_WINDOW_SIZE = 15
    _FARNEBACK_ITERATIONS = 3
    _FARNEBACK_POLY_N = 5
    _FARNEBACK_POLY_SIGMA = 1.2
    _FARNEBACK_FLAGS = 0

    def __init__(self, config: OpticalFlowMagnitudePolicyConfig) -> None:
        self._config = config
        self._hysteresis_state = GuardrailState.PASS
        self._caution_hits = 0
        self._stop_hits = 0
        self._clear_hits = 0

    def evaluate(
        self,
        previous_image: Image,
        current_image: Image,
        incoming_cmd_vel: Twist,
        health: GuardrailHealth,
    ) -> GuardrailDecision:
        if not health.has_previous_frame:
            self._reset_hysteresis()
            return self._zero_decision(
                GuardrailState.INIT,
                "missing_previous_frame",
                risk_score=0.0,
            )

        if not health.image_fresh:
            self._reset_hysteresis()
            return self._zero_decision(
                GuardrailState.SENSOR_DEGRADED,
                "image_not_fresh",
                risk_score=1.0,
                publish_immediately=True,
            )

        forward_speed = float(incoming_cmd_vel.linear.x)
        if forward_speed <= self._config.forward_motion_deadband_mps:
            self._reset_hysteresis()
            return self._pass_decision(incoming_cmd_vel, "forward_guard_inactive", 0.0)

        previous_gray, current_gray = self._prepare_gray_pair(previous_image, current_image)
        previous_roi, current_roi = self._extract_forward_rois(previous_gray, current_gray)

        if previous_roi.size == 0 or current_roi.size == 0:
            self._reset_hysteresis()
            return self._zero_decision(
                GuardrailState.SENSOR_DEGRADED,
                "invalid_forward_roi",
                risk_score=1.0,
                publish_immediately=True,
            )

        if self._is_occluded(current_roi):
            self._reset_hysteresis()
            return self._zero_decision(
                GuardrailState.SENSOR_DEGRADED,
                "forward_roi_occluded",
                risk_score=1.0,
                publish_immediately=True,
            )

        if self._is_low_texture(current_roi):
            self._reset_hysteresis()
            return self._zero_decision(
                GuardrailState.SENSOR_DEGRADED,
                "forward_roi_low_texture",
                risk_score=1.0,
                publish_immediately=True,
            )

        mean_flow_magnitude = self._mean_flow_magnitude(previous_roi, current_roi)
        next_state = self._next_state(mean_flow_magnitude)
        self._active_state = next_state

        if next_state == GuardrailState.STOP_LATCHED:
            return self._stop_forward_decision(
                incoming_cmd_vel,
                "forward_flow_stop",
                mean_flow_magnitude,
            )

        if next_state == GuardrailState.CLAMP:
            reason = (
                "forward_flow_clamp"
                if mean_flow_magnitude >= self._config.caution_flow_magnitude_threshold
                else "forward_flow_recovery"
            )
            return self._clamp_forward_decision(
                incoming_cmd_vel,
                reason,
                mean_flow_magnitude,
            )

        return self._pass_decision(
            incoming_cmd_vel,
            "forward_flow_clear",
            mean_flow_magnitude,
        )

    def _prepare_gray_pair(
        self,
        previous_image: Image,
        current_image: Image,
    ) -> tuple[GrayImage, GrayImage]:
        previous_gray = self._to_resized_gray(previous_image)
        current_gray = self._to_resized_gray(current_image)

        shared_height = min(previous_gray.shape[0], current_gray.shape[0])
        shared_width = min(previous_gray.shape[1], current_gray.shape[1])

        return (
            np.ascontiguousarray(previous_gray[:shared_height, :shared_width]),
            np.ascontiguousarray(current_gray[:shared_height, :shared_width]),
        )

    def _to_resized_gray(self, image: Image) -> GrayImage:
        gray = cast("GrayImage", image.to_grayscale().data)
        if gray.dtype != np.uint8:
            gray = cv2.convertScaleAbs(gray)  # type: ignore[call-overload]

        height, width = gray.shape[:2]
        if width <= 0 or height <= 0:
            raise ValueError("Image has invalid dimensions")

        target_width = min(width, self._config.flow_downsample_width_px)
        if target_width == width:
            return cast("GrayImage", np.ascontiguousarray(gray))

        scale = target_width / float(width)
        target_height = max(round(height * scale), 2)
        resized = cv2.resize(  # type: ignore[call-overload]
            gray,
            (target_width, target_height),
            interpolation=cv2.INTER_AREA,
        )
        return cast("GrayImage", np.ascontiguousarray(resized))

    def _extract_forward_rois(
        self,
        previous_gray: GrayImage,
        current_gray: GrayImage,
    ) -> tuple[GrayImage, GrayImage]:
        height, width = current_gray.shape
        x0, x1, y0, y1 = self._forward_roi_bounds(width=width, height=height)

        return (
            np.ascontiguousarray(previous_gray[y0:y1, x0:x1]),
            np.ascontiguousarray(current_gray[y0:y1, x0:x1]),
        )

    def _forward_roi_bounds(self, *, width: int, height: int) -> tuple[int, int, int, int]:
        roi_width = max(round(width * self._config.forward_roi_width_fraction), 2)
        x0 = max((width - roi_width) // 2, 0)
        x1 = min(x0 + roi_width, width)

        y0 = min(max(round(height * self._config.forward_roi_top_fraction), 0), height - 1)
        y1 = min(max(round(height * self._config.forward_roi_bottom_fraction), y0 + 1), height)

        return x0, x1, y0, y1

    def _is_low_texture(self, roi: GrayImage) -> bool:
        return float(np.var(roi)) < self._config.low_texture_variance_threshold

    def _is_occluded(self, roi: GrayImage) -> bool:
        dark_fraction = float(np.mean(roi <= self._config.occlusion_dark_pixel_threshold))
        bright_fraction = float(np.mean(roi >= self._config.occlusion_bright_pixel_threshold))
        return (
            max(dark_fraction, bright_fraction) >= self._config.occlusion_extreme_fraction_threshold
        )

    def _mean_flow_magnitude(self, previous_roi: GrayImage, current_roi: GrayImage) -> float:
        flow = cv2.calcOpticalFlowFarneback(  # type: ignore[call-overload]
            previous_roi,
            current_roi,
            cast("Any", None),
            self._FARNEBACK_PYR_SCALE,
            self._FARNEBACK_LEVELS,
            self._FARNEBACK_WINDOW_SIZE,
            self._FARNEBACK_ITERATIONS,
            self._FARNEBACK_POLY_N,
            self._FARNEBACK_POLY_SIGMA,
            self._FARNEBACK_FLAGS,
        )

        magnitude, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])
        return float(np.mean(magnitude))

    def _next_state(self, mean_flow_magnitude: float) -> GuardrailState:
        if mean_flow_magnitude >= self._config.stop_flow_magnitude_threshold:
            self._stop_hits += 1
            # Stop-level flow is also caution-level flow. This lets us clamp
            # first when stop evidence has not yet met its own persistence rule.
            self._caution_hits += 1
            self._clear_hits = 0
        elif mean_flow_magnitude >= self._config.caution_flow_magnitude_threshold:
            self._stop_hits = 0
            self._caution_hits += 1
            self._clear_hits = 0
        else:
            self._stop_hits = 0
            self._caution_hits = 0
            self._clear_hits += 1

        if self._hysteresis_state == GuardrailState.STOP_LATCHED:
            if self._stop_hits >= self._config.stop_frame_count:
                return GuardrailState.STOP_LATCHED
            if self._clear_hits >= self._config.clear_frame_count:
                return GuardrailState.PASS
            return GuardrailState.CLAMP

        if self._hysteresis_state == GuardrailState.CLAMP:
            if self._stop_hits >= self._config.stop_frame_count:
                return GuardrailState.STOP_LATCHED
            if self._clear_hits >= self._config.clear_frame_count:
                return GuardrailState.PASS
            return GuardrailState.CLAMP

        if self._stop_hits >= self._config.stop_frame_count:
            return GuardrailState.STOP_LATCHED

        if self._caution_hits >= self._config.caution_frame_count:
            return GuardrailState.CLAMP

        return GuardrailState.PASS

    def _reset_hysteresis(self) -> None:
        """Reset internal clamp/stop evidence without changing module output state."""
        self._hysteresis_state = GuardrailState.PASS
        self._caution_hits = 0
        self._stop_hits = 0
        self._clear_hits = 0

    def _pass_decision(
        self,
        incoming_cmd_vel: Twist,
        reason: str,
        risk_score: float,
    ) -> GuardrailDecision:
        cmd_vel = Twist(
            linear=incoming_cmd_vel.linear,
            angular=incoming_cmd_vel.angular,
        )
        return GuardrailDecision(
            state=GuardrailState.PASS,
            cmd_vel=cmd_vel,
            reason=reason,
            risk_score=risk_score,
        )

    def _clamp_forward_decision(
        self,
        incoming_cmd_vel: Twist,
        reason: str,
        risk_score: float,
    ) -> GuardrailDecision:
        cmd_vel = Twist(
            linear=incoming_cmd_vel.linear,
            angular=incoming_cmd_vel.angular,
        )
        cmd_vel.linear.x = min(float(cmd_vel.linear.x), self._config.clamp_forward_speed_mps)
        return GuardrailDecision(
            state=GuardrailState.CLAMP,
            cmd_vel=cmd_vel,
            reason=reason,
            risk_score=risk_score,
        )

    def _stop_forward_decision(
        self,
        incoming_cmd_vel: Twist,
        reason: str,
        risk_score: float,
    ) -> GuardrailDecision:
        cmd_vel = Twist(
            linear=incoming_cmd_vel.linear,
            angular=incoming_cmd_vel.angular,
        )
        cmd_vel.linear.x = 0.0
        return GuardrailDecision(
            state=GuardrailState.STOP_LATCHED,
            cmd_vel=cmd_vel,
            reason=reason,
            risk_score=risk_score,
        )

    def _zero_decision(
        self,
        state: GuardrailState,
        reason: str,
        *,
        risk_score: float = 0.0,
        publish_immediately: bool = False,
    ) -> GuardrailDecision:
        return GuardrailDecision(
            state=state,
            cmd_vel=Twist.zero(),
            reason=reason,
            risk_score=risk_score,
            publish_immediately=publish_immediately,
        )
