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

"""Pair source-timestamped RGB and named body/Dex3 measurements on one host."""

from collections import deque
from dataclasses import dataclass
import math
from numbers import Real

import numpy as np
from numpy.typing import NDArray

from dimos.robot.unitree.g1.sonic_observation import SonicObservation, policy_joint_state


@dataclass(frozen=True)
class SonicMeasuredState:
    """Acquisition estimates in the observation/session host's monotonic clock.

    Each body/left-hand/right-hand estimate has an absolute error bound, not
    a standard deviation. Zero means an exact time, never an unknown bound.
    Publication, receipt and ROS clock samples alone are insufficient.
    Sequence and component estimates must advance; producers must preserve
    estimates and bounds for cached measurements instead of restamping them.
    """

    sequence: int
    positions: dict[str, float]
    body_observed_at: float
    left_hand_observed_at: float
    right_hand_observed_at: float
    acquisition_uncertainty_seconds: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def acquisition_times(self) -> tuple[float, float, float]:
        return self.body_observed_at, self.left_hand_observed_at, self.right_hand_observed_at

    @property
    def acquisition_bounds(self) -> tuple[tuple[float, float], ...]:
        """Validated inclusive intervals, ordered body, left hand, right hand."""
        errors = self.acquisition_uncertainty_seconds
        if (
            not isinstance(errors, tuple)
            or len(errors) != 3
            or not all(
                isinstance(error, Real)
                and not isinstance(error, bool)
                and math.isfinite(error)
                and error >= 0
                for error in errors
            )
        ):
            raise ValueError("acquisition uncertainty requires three finite nonnegative bounds")
        times = self.acquisition_times
        if not all(
            isinstance(t, Real) and not isinstance(t, bool) and math.isfinite(t) for t in times
        ):
            raise ValueError("acquisition time invalid")
        bounds = tuple(
            (float(t) - float(error), float(t) + float(error))
            for t, error in zip(times, errors, strict=True)
        )
        if not all(math.isfinite(endpoint) for interval in bounds for endpoint in interval):
            raise ValueError("acquisition interval invalid")
        return bounds


class SonicObservationPairer:
    """Bounded pairing of each image with one unused measured-state snapshot.

    No interpolation or receipt-time substitution. max_skew is an explicit
    deployment budget, distinct from max_age. The caller serializes access.
    """

    def __init__(self, *, max_skew: float, max_age: float = 0.2, capacity: int = 32) -> None:
        if not (
            math.isfinite(max_skew)
            and math.isfinite(max_age)
            and 0 < max_skew <= max_age <= 0.2
            and type(capacity) is int
            and 0 < capacity <= 256
        ):
            raise ValueError("finite pairing budgets and bounded capacity required")
        self._max_skew, self._max_age = max_skew, max_age
        self._states: deque[tuple[int, tuple[float, ...], NDArray[np.float32]]] = deque(
            maxlen=capacity
        )
        self._image: tuple[float, float, NDArray[np.uint8]] | None = None
        self._last_image_time = self._last_observed_at = self._last_now = -math.inf
        self._last_state_times = (-math.inf, -math.inf, -math.inf)
        self._last_state_sequence = self._used_state_sequence = -1
        self._sequence = 0

    @property
    def buffered_states(self) -> int:
        return len(self._states)

    def _expire(self, now: float) -> None:
        if not math.isfinite(now) or now < self._last_now:
            raise ValueError("finite nondecreasing local monotonic time required")
        self._last_now = now
        # Increasing estimates can have decreasing lower bounds as uncertainty changes.
        self._states = deque(
            (entry for entry in self._states if now - min(entry[1]) < self._max_age),
            maxlen=self._states.maxlen,
        )
        if self._image is not None and now - self._image[0] >= self._max_age:
            self._image = None

    def _fresh(self, times: tuple[float, ...], now: float) -> None:
        if not all(math.isfinite(t) and 0 <= now - t < self._max_age for t in times):
            raise ValueError("acquisition time stale, future or invalid")

    def push_image(
        self, rgb: NDArray[np.uint8], *, observed_at: float, now: float, uncertainty: float = 0.0
    ) -> SonicObservation | None:
        self._expire(now)
        if not math.isfinite(uncertainty) or uncertainty < 0:
            raise ValueError("invalid camera acquisition uncertainty")
        lower, upper = observed_at - uncertainty, observed_at + uncertainty
        self._fresh((lower, upper), now)
        if observed_at <= self._last_image_time:
            raise ValueError("image must be newer than previous source frame")
        if rgb.shape != (480, 640, 3) or rgb.dtype != np.uint8:
            raise ValueError("RGB must be uint8 [480, 640, 3]")
        self._image = lower, upper, rgb.copy()
        self._last_image_time = observed_at
        return self._pair()

    def push_state(self, state: SonicMeasuredState, *, now: float) -> SonicObservation | None:
        self._expire(now)
        times = state.acquisition_times
        endpoints = tuple(t for interval in state.acquisition_bounds for t in interval)
        self._fresh(endpoints, now)
        if type(state.sequence) is not int or state.sequence < 0:
            raise ValueError("measured-state sequence must be a nonnegative integer")
        if state.sequence <= self._last_state_sequence or any(
            t <= previous for t, previous in zip(times, self._last_state_times, strict=True)
        ):
            raise ValueError("state sequence and all acquisition times must be newer")
        values = policy_joint_state(state.positions)
        self._states.append((state.sequence, endpoints, values))
        self._last_state_sequence, self._last_state_times = state.sequence, times
        return self._pair()

    def _pair(self) -> SonicObservation | None:
        if self._image is None:
            return None
        lower, upper, rgb = self._image
        stamp = (lower + upper) / 2
        candidates = [
            entry
            for entry in self._states
            if entry[0] > self._used_state_sequence
            and max(upper, *entry[1]) - min(lower, *entry[1]) <= self._max_skew
            and min(lower, *entry[1]) > self._last_observed_at
        ]
        if not candidates:
            return None
        index, times, values = min(
            candidates, key=lambda item: max(abs(stamp - t) for t in item[1])
        )
        obs = SonicObservation(self._sequence, min(lower, *times), values, rgb)
        self._sequence += 1
        self._used_state_sequence, self._last_observed_at = index, obs.observed_at
        self._image = None
        return obs
