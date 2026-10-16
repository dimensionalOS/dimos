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

"""Transport-free, single-caller task session for the guarded SONIC prototype.

The caller must own the sole reference publisher and call tick at 50 Hz using
one local monotonic clock for feedback receipt, observation capture and calls.
This core neither powers the controller nor verifies physical task success.
Native receiver expiry is still required if the caller/process stops running.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import asdict, dataclass, replace
from enum import StrEnum
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray

from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback
from dimos.robot.unitree.g1.sonic_observation import ObservationTimeBasis
from dimos.robot.unitree.g1.sonic_reference import encode_command, encode_token_reference

TICK_SECONDS = 0.02
_SIM_RIGHT_ORDER = (0, 1, 2, 5, 6, 3, 4)


def _finite(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return float(value)


@dataclass(frozen=True)
class SonicModelProfile:
    """Explicit sonic78 contract; handback delegates hands to the native planner.

    The latent step limit is not a joint-speed limit. This profile does not
    establish physical hand mapping, grasp retention or a safe loaded return.
    """

    name: str
    source_fps: Literal[30, 50]
    right_hand_order: Literal["native", "mujoco_swap"]
    handback_policy: Literal["native_planner"]
    seed_seconds: float = 1.0
    arm_timeout: float = 1.0
    first_chunk_timeout: float = 5.0
    handback_timeout: float = 2.0
    feedback_timeout: float = 0.2
    max_tick_gap: float = 0.1
    max_duration: float = 30.0
    max_observation_age: float = 0.5
    blend_seconds: float = 0.3
    token_bound: float = 1.25
    token_step_limit: float = 0.05
    observation_time_basis: ObservationTimeBasis = "acquisition"

    def __post_init__(self) -> None:
        if self.observation_time_basis not in ("acquisition", "delivery"):
            raise ValueError("unsupported observation_time_basis")
        if not self.name or self.source_fps not in (30, 50):
            raise ValueError("name and explicit sonic78 source_fps (30 or 50) are required")
        if self.right_hand_order not in ("native", "mujoco_swap"):
            raise ValueError("unsupported right_hand_order")
        if self.handback_policy != "native_planner":
            raise ValueError("explicit native_planner handback_policy is required")
        for name in (
            "seed_seconds",
            "arm_timeout",
            "first_chunk_timeout",
            "handback_timeout",
            "feedback_timeout",
            "max_tick_gap",
            "max_duration",
            "max_observation_age",
            "blend_seconds",
            "token_bound",
            "token_step_limit",
        ):
            if _finite(getattr(self, name), name) <= 0:
                raise ValueError(f"{name} must be positive")
        if not TICK_SECONDS <= self.max_tick_gap < 0.2:
            raise ValueError("max_tick_gap must be below the prototype's 200 ms lease")


class SessionPhase(StrEnum):
    IDLE = "idle"
    SEEDING = "seeding"
    ARMING = "arming"
    WAITING = "waiting_for_chunk"
    RUNNING = "running"
    HANDBACK = "handback"
    DONE = "done"


@dataclass(frozen=True)
class SessionStatus:
    task_id: str
    phase: SessionPhase = SessionPhase.IDLE
    outcome: Literal["finished", "cancelled", "failed"] | None = None
    reason: str = ""
    handback_confirmed: bool = False
    last_frame_index: int = -1
    last_chunk_sequence: int = -1


class JsonlSessionJournal:
    """Create a new journal; append and fsync each lifecycle/chunk event.

    Does not log each 50 Hz packet. Callers must budget storage latency before
    attaching a real transport; the session detects an ensuing tick gap.
    """

    def __init__(self, path: Path) -> None:
        self._path = path
        with path.open("x", encoding="utf-8"):
            pass

    def __call__(self, event: dict[str, Any]) -> None:
        line = json.dumps(event, allow_nan=False, separators=(",", ":"))
        with self._path.open("a", encoding="utf-8") as file:
            file.write(line + "\n")
            file.flush()
            os.fsync(file.fileno())


class SonicTaskSession:
    """One-shot session; no threads, sockets, native process control or retries.

    Use decoded feedback. submit_chunk accepts direct sonic78 actions only;
    observed_at denotes the chunk's first sample on the local monotonic clock.
    Serialized calls and an exclusive external publisher are prerequisites.
    A successful send is not a controller acknowledgement. Call tick during
    handback too, until DONE, even after an inference worker fails.
    """

    def __init__(
        self,
        task_id: str,
        profile: SonicModelProfile,
        *,
        send: Callable[[bytes], None],
        journal: Callable[[dict[str, Any]], None],
    ) -> None:
        if not task_id:
            raise ValueError("task_id is required")
        self._profile = profile
        self._send = send
        self._journal = journal
        self._status = SessionStatus(task_id)
        self._feedback: SonicFeedback | None = None
        self._feedback_fault: str | None = None
        self._now = -math.inf
        self._started = 0.0
        self._phase_started = 0.0
        self._last_tick = 0.0
        self._last_emit = -math.inf
        self._initial_epoch = 0
        self._highest_epoch = 0
        self._first_stream_index: int | None = None
        self._request_index = -1
        self._request_time = 0.0
        self._last_handback_send = -math.inf
        self._last_action = np.zeros(78, dtype=np.float64)
        self._blend_from = self._last_action.copy()
        self._chunk: NDArray[np.float64] | None = None
        self._observed_at = -math.inf
        self._chunk_received = 0.0

    @property
    def profile(self) -> SonicModelProfile:
        return self._profile

    @property
    def status(self) -> SessionStatus:
        return self._status

    def _clock(self, now: float) -> None:
        if _finite(now, "now") < self._now:
            raise ValueError("now must be monotonic")
        self._now = now

    def _record(self, event: str, **details: Any) -> None:
        try:
            self._journal(
                {
                    "event": event,
                    "monotonic": self._now,
                    "feedback_index": None if self._feedback is None else self._feedback.index,
                    "feedback": None if self._feedback is None else asdict(self._feedback),
                    "status": asdict(self._status),
                    **details,
                }
            )
        except Exception:
            self._status = replace(
                self._status,
                phase=SessionPhase.DONE,
                outcome="failed",
                reason="journal failed",
                handback_confirmed=False,
            )
            # Logging failure must never leave the policy publisher active.
            try:
                self._send(encode_command(start=False, stop=False, planner=True))
            except Exception:
                pass  # Recovery is explicitly unconfirmed; native expiry remains necessary.
            raise

    def _transition(self, phase: SessionPhase) -> None:
        self._status = replace(self._status, phase=phase)
        self._phase_started = self._now
        self._record("terminal" if phase == SessionPhase.DONE else "phase")

    def _planner_action(self, state: SonicFeedback) -> NDArray[np.float64]:
        if state.token_state is None:
            raise ValueError("planner token_state is required")
        right = np.asarray(state.right_hand_q, dtype=np.float64)
        if self.profile.right_hand_order == "mujoco_swap":
            right = right[list(_SIM_RIGHT_ORDER)]
        action: NDArray[np.float64] = self._actions(
            np.array([(*state.token_state, *state.left_hand_q, *right)])
        )[0]
        return action

    def _actions(self, values: NDArray[Any]) -> NDArray[np.float64]:
        raw = np.asarray(values)
        if raw.ndim != 2 or raw.shape[1] != 78 or not 1 <= len(raw) <= 1000:
            raise ValueError("actions must have shape [1..1000, 78]")
        if raw.dtype.kind not in "fiu":
            raise ValueError("actions must be numeric, not boolean or strings")
        actions = raw.astype(np.float64, copy=True)
        if not np.all(np.isfinite(actions)) or np.max(np.abs(actions)) > np.finfo(np.float32).max:
            raise ValueError("actions must contain finite float32 values")
        if np.max(np.abs(actions[:, :64])) > self.profile.token_bound:
            raise ValueError("token bound exceeded")
        return actions

    def start(self, state: SonicFeedback, *, now: float) -> None:
        if self.status.phase != SessionPhase.IDLE:
            raise RuntimeError("session is one-shot")
        self._clock(now)
        guard = state.reference_guard
        if not 0 <= now - state.received_monotonic < self.profile.feedback_timeout:
            raise ValueError("fresh feedback is required")
        if guard is None or not guard.planner_confirmed or guard.armed:
            raise ValueError("confirmed guarded planner is required")
        if guard.accepted_index >= 2**63 - 1:
            raise ValueError("wire index exhausted")
        action = self._planner_action(state)
        self._feedback = state
        self._initial_epoch = guard.epoch
        self._highest_epoch = guard.epoch
        self._status = replace(self.status, last_frame_index=guard.accepted_index)
        self._last_action = action
        self._started = self._last_tick = now
        self._request_index, self._request_time = state.index, now
        self._record("start", profile=asdict(self.profile), captured_action=action.tolist())
        self._transition(SessionPhase.SEEDING)
        self._emit(encode_command(start=False, stop=False, planner=True))

    def observe(self, state: SonicFeedback, *, now: float) -> bool:
        self._clock(now)
        previous = self._feedback
        if self.status.phase in (SessionPhase.IDLE, SessionPhase.DONE):
            return False
        if not 0 <= now - state.received_monotonic < self.profile.feedback_timeout:
            return False
        if previous is not None and (
            state.index <= previous.index or state.received_monotonic <= previous.received_monotonic
        ):
            return False
        self._feedback = state
        if state.reference_guard is not None:
            self._highest_epoch = max(self._highest_epoch, state.reference_guard.epoch)
        if self.status.phase != SessionPhase.HANDBACK and self._feedback_fault is None:
            self._feedback_fault = self._problem()
            if self._feedback_fault is not None:
                self._record("feedback_fault", reason=self._feedback_fault)
        return True

    def _problem(self) -> str | None:
        if self._feedback_fault is not None:
            return self._feedback_fault
        state = self._feedback
        if (
            state is None
            or not 0 <= self._now - state.received_monotonic < self.profile.feedback_timeout
        ):
            return "feedback stale"
        if self._now - self._last_tick > self.profile.max_tick_gap + 1e-9:
            return "tick deadline missed"
        if self._now - self._started >= self.profile.max_duration:
            return "task deadline exceeded"
        guard = state.reference_guard
        if guard is None:
            return "reference guard missing"
        if guard.accepted_index > self.status.last_frame_index:
            return "unexpected reference owner"
        phase = self.status.phase
        if phase == SessionPhase.SEEDING:
            if guard.epoch != self._initial_epoch or not guard.planner_confirmed:
                return "planner lost during seeding"
        elif phase == SessionPhase.ARMING:
            if self._now - self._phase_started >= self.profile.arm_timeout:
                return "stream arm timeout"
            if guard.epoch not in (self._initial_epoch, self._initial_epoch + 1):
                return "unexpected reference epoch"
            if guard.epoch == self._initial_epoch + 1 and (
                guard.expired or guard.rearm_requires_planner
            ):
                return "stream expired during arming"
        elif (
            guard.epoch != self._initial_epoch + 1
            or not guard.armed
            or guard.expired
            or guard.planner_selected
        ):
            return "stream guard lost"
        if (
            phase == SessionPhase.WAITING
            and self._now - self._phase_started >= self.profile.first_chunk_timeout
        ):
            return "first chunk timeout"
        if phase == SessionPhase.RUNNING:
            assert self._chunk is not None
            age = self._now - self._observed_at
            if age + 1e-9 >= len(self._chunk) / self.profile.source_fps:
                return "chunk horizon exhausted"
            if age >= self.profile.max_observation_age:
                return "observation stale"
        return None

    def submit_chunk(
        self,
        *,
        task_id: str,
        sequence: int,
        observed_at: float,
        actions: NDArray[Any],
        now: float,
    ) -> None:
        self._clock(now)
        if self.status.phase not in (SessionPhase.WAITING, SessionPhase.RUNNING):
            raise ValueError("session is not accepting chunks")
        problem = self._problem()
        if problem is not None:
            self._handback("failed", problem)
            raise ValueError(problem)
        try:
            if task_id != self.status.task_id:
                raise ValueError("chunk belongs to another task")
            if type(sequence) is not int or sequence <= self.status.last_chunk_sequence:
                raise ValueError("chunk sequence must increase")
            age = now - _finite(observed_at, "observed_at")
            if not 0 <= age < self.profile.max_observation_age or observed_at <= self._observed_at:
                raise ValueError("observation must be fresh and newer than the previous chunk")
            data = self._actions(actions)
            if age >= len(data) / self.profile.source_fps:
                raise ValueError("chunk horizon exhausted before arrival")
        except ValueError as exc:
            self._record("chunk_rejected", reason=str(exc))
            raise
        self._chunk = data
        self._observed_at = observed_at
        self._chunk_received = now
        self._blend_from = self._last_action.copy()
        self._status = replace(self.status, last_chunk_sequence=sequence)
        self._record(
            "chunk_accepted",
            sequence=sequence,
            observed_at=observed_at,
            rows=len(data),
            source_fps=self.profile.source_fps,
            sha256=hashlib.sha256(data.astype("<f4").tobytes()).hexdigest(),
        )
        if self.status.phase == SessionPhase.WAITING:
            self._transition(SessionPhase.RUNNING)

    def _emit(self, packet: bytes) -> bool:
        try:
            self._send(packet)
            return True
        except Exception as exc:
            self._handback("failed", f"packet send failed: {type(exc).__name__}: {exc}")
            return False

    def _pose(self, action: NDArray[np.float64]) -> None:
        index = self.status.last_frame_index + 1
        if index >= 2**63:
            self._handback("failed", "wire index exhausted")
            return
        right = action[71:]
        if self.profile.right_hand_order == "mujoco_swap":
            right = right[list(_SIM_RIGHT_ORDER)]
        packet = encode_token_reference(
            token=action[:64].tolist(),
            left_hand=action[64:71].tolist(),
            right_hand=right.tolist(),
            frame_index=index,
        )
        # Reserve even on an ambiguous send failure; never reuse a wire index.
        self._status = replace(self.status, last_frame_index=index)
        if self._emit(packet):
            self._last_action = action.copy()
            self._last_emit = self._now
            if self.status.phase == SessionPhase.ARMING and self._first_stream_index is None:
                self._first_stream_index = index

    def _handback(self, outcome: Literal["finished", "cancelled", "failed"], reason: str) -> None:
        if self.status.phase in (SessionPhase.IDLE, SessionPhase.DONE, SessionPhase.HANDBACK):
            return
        self._status = replace(self.status, outcome=outcome, reason=reason)
        self._request_index = -1 if self._feedback is None else self._feedback.index
        self._request_time = self._now
        self._transition(SessionPhase.HANDBACK)
        self._send_handback()

    def _send_handback(self) -> None:
        self._last_handback_send = self._now
        try:
            self._send(encode_command(start=False, stop=False, planner=True))
        except Exception as exc:
            self._status = replace(
                self.status,
                outcome="failed",
                reason=f"{self.status.reason}; handback send failed: {exc}",
            )
            self._record("handback_send_failed")

    def cancel(self, *, now: float) -> None:
        self._clock(now)
        self._handback("cancelled", "cancel requested")

    def finish(self, *, now: float) -> None:
        """End streaming; does not assert successful grasp/placement or settling."""
        self._clock(now)
        self._handback("finished", "finish requested")

    def fail(self, reason: str, *, now: float) -> None:
        """Report an action-source/transport failure and request planner handback."""
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("failure reason is required")
        self._clock(now)
        self._handback("failed", reason)

    def tick(self, *, now: float) -> None:
        self._clock(now)
        phase = self.status.phase
        if phase in (SessionPhase.IDLE, SessionPhase.DONE):
            return
        state = self._feedback
        assert state is not None
        guard = state.reference_guard
        if phase == SessionPhase.HANDBACK:
            if (
                guard is not None
                and guard.planner_confirmed
                and state.index > self._request_index
                and state.received_monotonic > self._request_time
                and state.received_monotonic < self._request_time + self.profile.handback_timeout
                and 0 <= now - state.received_monotonic < self.profile.feedback_timeout
                and self._initial_epoch <= guard.epoch <= self._initial_epoch + 1
                and guard.epoch == self._highest_epoch
            ):
                self._status = replace(self.status, handback_confirmed=True)
                self._transition(SessionPhase.DONE)
            elif now - self._phase_started >= self.profile.handback_timeout:
                self._status = replace(
                    self.status, outcome="failed", reason=f"{self.status.reason}; handback timeout"
                )
                self._transition(SessionPhase.DONE)
            elif now - self._last_handback_send >= 0.1:
                self._send_handback()
            return
        problem = self._problem()
        if problem is not None:
            self._handback("failed", problem)
            return
        self._last_tick = now
        assert guard is not None
        if (
            phase == SessionPhase.SEEDING
            and now - self._phase_started + 1e-9 >= self.profile.seed_seconds
        ):
            if state.index <= self._request_index or state.received_monotonic <= self._request_time:
                self._handback("failed", "planner request unconfirmed")
                return
            self._transition(SessionPhase.ARMING)
            if not self._emit(encode_command(start=False, stop=False, planner=False)):
                return
        elif phase == SessionPhase.ARMING:
            if (
                guard.epoch == self._initial_epoch + 1
                and guard.armed
                and not guard.expired
                and not guard.planner_selected
                and self._first_stream_index is not None
                and guard.applied_index >= self._first_stream_index
            ):
                self._transition(SessionPhase.WAITING)
            elif now - self._phase_started >= self.profile.arm_timeout:
                self._handback("failed", "stream arm timeout")
                return
        action = self._last_action
        if self.status.phase == SessionPhase.RUNNING:
            assert self._chunk is not None
            age = now - self._observed_at
            position = age * self.profile.source_fps
            lower = min(int(position), len(self._chunk) - 1)
            upper = min(lower + 1, len(self._chunk) - 1)
            target = self._chunk[lower] + (position - lower) * (
                self._chunk[upper] - self._chunk[lower]
            )
            alpha = min((now - self._chunk_received) / self.profile.blend_seconds, 1.0)
            action = self._blend_from + alpha * (target - self._blend_from)
            action[:64] = self._last_action[:64] + np.clip(
                action[:64] - self._last_action[:64],
                -self.profile.token_step_limit,
                self.profile.token_step_limit,
            )
        if now - self._last_emit + 1e-9 >= TICK_SECONDS:
            self._pose(action)
