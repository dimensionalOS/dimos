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

"""Decode the external C++ SONIC runtime's single-frame g1_debug telemetry.

This is the inspected PC2 29-body/7+7-hand schema, not a general SONIC API.
The publisher already restores body offsets and motor order. Hand indices
remain raw firmware slots until the physical finger mapping is calibrated.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import msgpack  # type: ignore[import-untyped]

from dimos.control.components import make_humanoid_joints
from dimos.msgs.sensor_msgs.JointState import JointState

TOPIC = b"g1_debug"
MAX_PACKET_BYTES = 131072


@dataclass(frozen=True)
class SonicReferenceGuard:
    """Reported state of the opt-in external SONIC receiver guard.

    Expiry and planner confirmation are independent: an expired stream can
    have completed handover. Confirmation describes the controller path, not
    physical settling or task completion. ``applied_index`` means installed
    in the endpoint, not consumed by this particular policy tick. Indices
    may belong to a previous epoch during first-frame grace. ``age_ms`` is
    receiver-reported age since arm/accepted receipt; -1 means unavailable.
    Only a fresh enclosing SonicFeedback makes this a current observation.
    """

    armed: bool
    expired: bool
    planner_selected: bool
    planner_confirmed: bool
    rearm_requires_planner: bool
    accepted_index: int
    applied_index: int
    rejected: int
    epoch: int
    age_ms: float


@dataclass(frozen=True)
class SonicFeedback:
    index: int
    source_timestamp: float | None
    received_at: float
    received_monotonic: float
    body_q: tuple[float, ...]
    body_dq: tuple[float, ...]
    left_hand_q: tuple[float, ...]
    left_hand_dq: tuple[float, ...]
    right_hand_q: tuple[float, ...]
    right_hand_dq: tuple[float, ...]
    reference_guard: SonicReferenceGuard | None = None
    # The controller's latent reference, not measured joints or a task-success signal.
    token_state: tuple[float, ...] | None = None

    def joint_states(self) -> tuple[JointState, JointState, JointState]:
        """Return body/left/right positions and velocities, stamped at local receipt.

        Source time remains separate: zero means unavailable and nonzero does
        not establish clock synchronization. No global pose or effort is inferred.
        """
        return (
            JointState(
                ts=self.received_at,
                name=make_humanoid_joints("g1"),
                position=list(self.body_q),
                velocity=list(self.body_dq),
            ),
            JointState(
                ts=self.received_at,
                name=[f"g1/left_hand_motor_{i}" for i in range(7)],
                position=list(self.left_hand_q),
                velocity=list(self.left_hand_dq),
            ),
            JointState(
                ts=self.received_at,
                name=[f"g1/right_hand_motor_{i}" for i in range(7)],
                position=list(self.right_hand_q),
                velocity=list(self.right_hand_dq),
            ),
        )


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _vector(payload: dict[str, Any], name: str, size: int) -> tuple[float, ...]:
    values = payload.get(name)
    if not isinstance(values, list) or len(values) != size:
        raise ValueError(f"{name} must contain {size} values")
    return tuple(_number(value, name) for value in values)


def _guard_bool(payload: dict[str, Any], name: str) -> bool:
    value = payload.get(name)
    if not isinstance(value, bool):
        raise ValueError(f"reference_guard.{name} must be a boolean")
    return value


def _guard_int(payload: dict[str, Any], name: str, minimum: int = 0) -> int:
    value = payload.get(name)
    if type(value) is not int or value < minimum:
        raise ValueError(f"reference_guard.{name} must be an integer >= {minimum}")
    return value


def _reference_guard(value: Any) -> SonicReferenceGuard:
    if not isinstance(value, dict):
        raise ValueError("reference_guard must be a map")
    guard = SonicReferenceGuard(
        armed=_guard_bool(value, "armed"),
        expired=_guard_bool(value, "expired"),
        planner_selected=_guard_bool(value, "planner_selected"),
        planner_confirmed=_guard_bool(value, "planner_confirmed"),
        rearm_requires_planner=_guard_bool(value, "rearm_requires_planner"),
        accepted_index=_guard_int(value, "accepted_index", -1),
        applied_index=_guard_int(value, "applied_index", -1),
        rejected=_guard_int(value, "rejected"),
        epoch=_guard_int(value, "epoch"),
        age_ms=_number(value.get("age_ms"), "reference_guard.age_ms"),
    )
    if guard.age_ms < 0 and guard.age_ms != -1:
        raise ValueError("reference_guard.age_ms must be nonnegative or -1")
    if guard.applied_index > guard.accepted_index:
        raise ValueError("reference_guard.applied_index cannot exceed accepted_index")
    if guard.armed and guard.expired:
        raise ValueError("reference_guard cannot be both armed and expired")
    if guard.planner_confirmed and (not guard.planner_selected or guard.armed):
        raise ValueError("reference_guard planner confirmation conflicts with selected mode")
    return guard


def decode_feedback(
    packet: bytes, *, received_at: float, received_monotonic: float
) -> SonicFeedback:
    """Validate one topic-prefixed MessagePack frame; raise ValueError on bad data."""
    if len(packet) > MAX_PACKET_BYTES or not packet.startswith(TOPIC):
        raise ValueError("Invalid g1_debug envelope")
    try:
        payload = msgpack.unpackb(
            packet[len(TOPIC) :], raw=False, max_array_len=4096, max_map_len=256
        )
    except (ValueError, TypeError, msgpack.UnpackException) as exc:
        raise ValueError("Invalid g1_debug MessagePack") from exc
    if not isinstance(payload, dict):
        raise ValueError("g1_debug must contain a map")
    index = payload.get("index")
    if type(index) is not int or index < 0:
        raise ValueError("index must be a nonnegative integer")
    source_timestamp = _number(payload.get("ros_timestamp"), "ros_timestamp")
    if source_timestamp < 0:
        raise ValueError("ros_timestamp must be nonnegative")
    return SonicFeedback(
        index=index,
        source_timestamp=source_timestamp or None,
        received_at=_number(received_at, "received_at"),
        received_monotonic=_number(received_monotonic, "received_monotonic"),
        body_q=_vector(payload, "body_q", 29),
        body_dq=_vector(payload, "body_dq", 29),
        left_hand_q=_vector(payload, "left_hand_q", 7),
        left_hand_dq=_vector(payload, "left_hand_dq", 7),
        right_hand_q=_vector(payload, "right_hand_q", 7),
        right_hand_dq=_vector(payload, "right_hand_dq", 7),
        reference_guard=(
            _reference_guard(payload["reference_guard"]) if "reference_guard" in payload else None
        ),
        token_state=_vector(payload, "token_state", 64) if "token_state" in payload else None,
    )


class FeedbackBuffer:
    """Arrival freshness and sequence filtering. Caller synchronizes access.

    A controller index reset requires a new buffer/session. Repeated packets
    never refresh the arrival clock. This checks stream freshness, not whether
    the controller's underlying DDS measurements are current.
    """

    def __init__(self, *, stale_after: float) -> None:
        self._stale_after = _number(stale_after, "stale_after")
        if self._stale_after <= 0:
            raise ValueError("stale_after must be positive")
        self._state: SonicFeedback | None = None
        self.accepted = 0
        self.rejected_indices = 0

    def accept(self, state: SonicFeedback) -> bool:
        if self._state is not None and state.index <= self._state.index:
            self.rejected_indices += 1
            return False
        self._state = state
        self.accepted += 1
        return True

    def age(self, *, now: float) -> float | None:
        if self._state is None:
            return None
        return now - self._state.received_monotonic

    def latest(self, *, now: float) -> SonicFeedback | None:
        age = self.age(now=now)
        if age is None or not 0 <= age < self._stale_after:
            return None
        return self._state
