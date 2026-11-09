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

"""Separate delivery-freshness contract; no sensor acquisition-time claims."""

from dataclasses import asdict, dataclass
import math
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray

from dimos.robot.unitree.g1.sonic_observation import (
    DELIVERY_COMPONENTS,
    POLICY_JOINT_NAMES,
    SonicDeliveryStamp,
    SonicObservation,
    policy_joint_state,
)


@dataclass(frozen=True)
class SonicJointDelivery:
    """One receiver-owned component: 29 body slots or seven explicit hand slots."""

    group: Literal["body", "left_hand", "right_hand"]
    positions: tuple[float, ...]
    stamp: SonicDeliveryStamp

    def __post_init__(self) -> None:
        if self.group not in DELIVERY_COMPONENTS[1:]:
            raise ValueError("unknown joint delivery group")
        size = 29 if self.group == "body" else 7
        if (
            not isinstance(self.positions, tuple)
            or len(self.positions) != size
            or any(
                type(q) not in (int, float)
                or not math.isfinite(q)
                or abs(q) > np.finfo(np.float32).max
                for q in self.positions
            )
        ):
            raise ValueError("finite measured joint slots required")
        self.stamp.validate()


@dataclass(frozen=True)
class SonicCameraDelivery:
    """Original receiver stamp and an owned RGB image; never a cached restamp."""

    rgb: NDArray[np.uint8]
    stamp: SonicDeliveryStamp

    def __post_init__(self) -> None:
        self.stamp.validate()
        rgb = np.asarray(self.rgb)
        if rgb.shape != (480, 640, 3) or rgb.dtype != np.uint8:
            raise ValueError("RGB must be uint8 [480, 640, 3]")
        rgb = rgb.copy()
        rgb.setflags(write=False)
        object.__setattr__(self, "rgb", rgb)


class SonicDeliveryPairer:
    """Latest-only pairing of four independently received inputs on one host.

    A source change or marker rollback latches a fault. Replace the receiver,
    adapter and service session together; reconnecting cannot clear it. The
    caller serializes access. No interpolation or acquisition clock conversion.
    """

    def __init__(
        self,
        *,
        left_hand_slot_names: tuple[str, ...],
        right_hand_slot_names: tuple[str, ...],
        max_age: float,
        max_span: float,
    ) -> None:
        if not (
            math.isfinite(max_age) and math.isfinite(max_span) and 0 < max_span <= max_age <= 0.2
        ):
            raise ValueError("finite delivery budgets up to 200 ms required")
        for names, expected in (
            (left_hand_slot_names, POLICY_JOINT_NAMES[14:21]),
            (right_hand_slot_names, POLICY_JOINT_NAMES[21:28]),
        ):
            if len(names) != 7 or set(names) != set(expected):
                raise ValueError("hand mapping must name exactly its seven policy joints")
        self._names = {
            "body": POLICY_JOINT_NAMES[:14],
            "left_hand": left_hand_slot_names,
            "right_hand": right_hand_slot_names,
        }
        self._max_age, self._max_span = max_age, max_span
        self._stamps: dict[str, SonicDeliveryStamp] = {}
        self._positions: dict[str, float] = {}
        self._rgb: NDArray[np.uint8] | None = None
        self._used: dict[str, int | float] = {}
        self._sequence = 0
        self._now = -math.inf
        self._fault: str | None = None
        self._observed_at: float | None = None

    def _clock(self, now: float) -> None:
        if not math.isfinite(now) or now < self._now:
            self._fault = "local monotonic clock invalid; replace receiver/adapter/service session"
        if self._fault:
            raise ValueError(self._fault)
        self._now = now

    def _accept(self, group: str, stamp: SonicDeliveryStamp, now: float) -> None:
        self._clock(now)
        stamp.validate()
        if not 0 <= now - stamp.received_at < self._max_age:
            raise ValueError("component receipt stale or from the future")
        previous = self._stamps.get(group)
        if previous is not None:
            if (
                stamp.source_id != previous.source_id
                or stamp.source_marker < previous.source_marker
            ):
                self._fault = f"{group} source changed or rolled back; replace receiver/adapter/service session"
                raise ValueError(self._fault)
            if stamp.source_marker == previous.source_marker:
                raise ValueError(f"duplicate {group} delivery marker")
            if stamp.received_at <= previous.received_at:
                raise ValueError("component receipt must advance")
        self._stamps[group] = stamp

    def push_joints(self, sample: SonicJointDelivery, *, now: float) -> SonicObservation | None:
        # Revalidate deserialized receiver records before changing replay markers.
        sample = SonicJointDelivery(sample.group, sample.positions, sample.stamp)
        values = sample.positions[15:29] if sample.group == "body" else sample.positions
        positions = dict(zip(self._names[sample.group], values, strict=True))
        self._accept(sample.group, sample.stamp, now)
        self._positions.update(positions)
        return self._pair(now)

    def push_camera(self, sample: SonicCameraDelivery, *, now: float) -> SonicObservation | None:
        sample = SonicCameraDelivery(sample.rgb, sample.stamp)
        self._accept("camera", sample.stamp, now)
        self._rgb = sample.rgb
        return self._pair(now)

    def _pair(self, now: float) -> SonicObservation | None:
        if len(self._stamps) != 4 or self._rgb is None:
            return None
        stamps = tuple(self._stamps[group] for group in DELIVERY_COMPONENTS)
        times = tuple(stamp.received_at for stamp in stamps)
        if (
            not all(0 <= now - t < self._max_age for t in times)
            or max(times) - min(times) > self._max_span
            or any(
                s.source_marker <= self._used.get(g, -math.inf)
                for g, s in zip(DELIVERY_COMPONENTS, stamps, strict=True)
            )
        ):
            return None
        obs = SonicObservation(
            self._sequence,
            min(times),
            policy_joint_state(self._positions),
            self._rgb,
            time_basis="delivery",
            delivery_stamps=stamps,
        )
        self._sequence += 1
        self._observed_at = obs.observed_at
        self._used = {group: stamp.source_marker for group, stamp in self._stamps.items()}
        return obs

    def get_status(self, *, now: float) -> dict[str, Any]:
        try:
            self._clock(now)
        except ValueError:
            pass
        ages = {
            group: None if group not in self._stamps else now - self._stamps[group].received_at
            for group in DELIVERY_COMPONENTS
        }
        times = [stamp.received_at for stamp in self._stamps.values()]
        fresh = self._observed_at is not None and 0 <= now - self._observed_at < self._max_age
        return {
            "state": "fault"
            if self._fault
            else "fresh"
            if fresh
            else "waiting"
            if self._observed_at is None
            else "stale",
            "fault": self._fault,
            "time_basis": "delivery",
            "delivery_ages_s": ages,
            "delivery_span_s": max(times) - min(times) if len(times) == 4 else None,
            "acquisition_age_s": None,
            "acquisition_skew_s": None,
            "sources": {group: asdict(stamp) for group, stamp in self._stamps.items()},
            "observations_published": self._sequence,
            "max_delivery_age_s": self._max_age,
            "max_delivery_span_s": self._max_span,
        }
