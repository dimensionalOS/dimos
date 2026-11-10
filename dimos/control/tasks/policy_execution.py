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

"""Execution mechanics shared by ONNX policy control tasks.

A policy task runs a learned controller inside the coordinator tick loop: it
claims joints, decides which coordinator ticks run inference, holds its last
command between them, and makes sure a result computed for a session that has
since been cancelled never reaches the motors.  :class:`PolicyTask` is that
execution skeleton; :class:`OnnxActor` and the small state helpers below are
its building blocks.

What does *not* live here is anything the trained controller defines: the
observation layout and history, action decoding, model switching, startup
ramps, feedback-loss handling, teleoperation and safety sessions stay in each
backend, because those are part of that policy's contract rather than of the
coordinator.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from pathlib import Path
import threading
from typing import Any

import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort

from dimos.control.task import (
    BaseControlTask,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.hardware.manipulators.spec import ControlMode
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

#: Deadline comparison tolerance for rate cadences. Coordinator time is an
#: accumulated float, so a nominal 0.05 s deadline must not slip one tick.
_DEADLINE_TOLERANCE_S = 1e-9


def preferred_onnx_providers(*, prefer_cuda: bool = True) -> list[str]:
    """Return ONNX Runtime providers in preference order.

    CUDA is opt-in per consumer: deployment backends that pin CPU numerics pass
    ``prefer_cuda=False`` and get a deterministic CPU-only list.
    """
    available = ort.get_available_providers()
    providers: list[str] = []

    if prefer_cuda and "CUDAExecutionProvider" in available:
        preload_dlls = getattr(ort, "preload_dlls", None)
        if preload_dlls is not None:
            try:
                preload_dlls(cuda=True, cudnn=True, msvc=False)
            except Exception as exc:
                logger.warning(
                    "Failed to preload ONNXRuntime CUDA/cuDNN libraries",
                    error=repr(exc),
                )
        providers.append("CUDAExecutionProvider")

    providers.append("CPUExecutionProvider")
    return providers


class OnnxActor:
    """One deterministic ONNX graph: a single observation row in, one action row out.

    The actor owns only session loading and name/shape normalization. Numeric
    validation and action decoding belong to the consumer, because the accepted
    ranges and decode order differ per trained controller.
    """

    def __init__(
        self,
        model_path: str | Path,
        *,
        providers: list[str] | None = None,
        input_name: str | None = None,
        output_name: str | None = None,
    ) -> None:
        self._session: Any = ort.InferenceSession(
            str(model_path), providers=providers or ["CPUExecutionProvider"]
        )
        self._input_name = input_name or self._session.get_inputs()[0].name
        self._output_name = output_name

    @property
    def session(self) -> Any:
        """The underlying ``onnxruntime.InferenceSession``."""
        return self._session

    @property
    def providers(self) -> list[str]:
        """Providers the session actually runs on."""
        return list(self._session.get_providers())

    @property
    def input_name(self) -> str:
        return self._input_name

    @property
    def output_name(self) -> str | None:
        return self._output_name

    def run(self, observation: NDArray[np.float32] | NDArray[np.float64]) -> NDArray[np.float32]:
        """Run the actor and return its first output as a flat float32 row."""
        feed = np.ascontiguousarray(observation, dtype=np.float32)
        outputs = self._session.run(
            [self._output_name] if self._output_name is not None else None,
            {self._input_name: feed},
        )
        return np.asarray(outputs[0], dtype=np.float32).reshape(-1)


class PolicySessionGuard:
    """Generation counter for one active policy session.

    A task prepares a policy tick in one generation, may run the actor outside
    its session lock, then asks :meth:`current` before emitting. Invalidating
    the session bumps the generation, so a late result from a released,
    estopped, preempted or stopped session can never reach the motors.

    The guard owns only the generation and active flag; approval inputs (goals,
    deadman, feedback freshness) and fault detail stay with the backend.
    """

    def __init__(self) -> None:
        self._epoch = 0
        self._active = False

    @property
    def active(self) -> bool:
        return self._active

    @property
    def epoch(self) -> int:
        return self._epoch

    def begin(self) -> int:
        """Open a session and return its generation."""
        self._active = True
        self._epoch += 1
        return self._epoch

    def invalidate(self) -> None:
        """Close the session; an in-flight result for it is no longer current."""
        if self._active:
            self._epoch += 1
        self._active = False

    def current(self, epoch: int) -> bool:
        """Return True while ``epoch`` is still the active generation."""
        return self._active and self._epoch == epoch


class PolicyTickScheduler:
    """Decide which coordinator ticks run policy inference.

    Exactly one cadence is configured:

    * :meth:`every` — infer on every n-th poll, counted from the last reset.
      This is the coordinator-decimation contract of a policy trained at a fixed
      multiple of the tick rate.
    * :meth:`at_hz` — infer when coordinator time reaches the next deadline.
      The first poll after :meth:`reset` is due, so a fresh session runs the
      actor immediately instead of waiting a full period.

    :meth:`mark` reports that an inference result was actually used and advances
    the rate deadline. A held or failed tick does not advance it, so a missed
    inference is retried on the next tick rather than silently skipped.
    """

    def __init__(self, *, every_n_ticks: int | None = None, hz: float | None = None) -> None:
        if (every_n_ticks is None) == (hz is None):
            raise ValueError("configure exactly one of every_n_ticks or hz")
        if every_n_ticks is not None and every_n_ticks < 1:
            raise ValueError("every_n_ticks must be >= 1")
        if hz is not None and hz <= 0.0:
            raise ValueError("hz must be positive")
        self._every_n_ticks = every_n_ticks
        self._period_s = None if hz is None else 1.0 / hz
        self._ticks = 0
        self._next_deadline: float | None = None

    @classmethod
    def every(cls, n_ticks: int) -> PolicyTickScheduler:
        """Infer on every ``n_ticks``-th coordinator tick."""
        return cls(every_n_ticks=n_ticks)

    @classmethod
    def at_hz(cls, hz: float) -> PolicyTickScheduler:
        """Infer at ``hz`` on the coordinator's monotonic clock."""
        return cls(hz=hz)

    @property
    def period_s(self) -> float | None:
        """Inference period for a rate scheduler, ``None`` for decimation."""
        return self._period_s

    @property
    def tick_count(self) -> int:
        """Polls counted since the last reset (decimation cadence only)."""
        return self._ticks

    def reset(self, t_now: float | None = None) -> None:
        """Restart the cadence; a rate cadence is immediately due at ``t_now``."""
        self._ticks = 0
        self._next_deadline = t_now if self._period_s is not None else None

    def poll(self, t_now: float) -> bool:
        """Return True when this coordinator tick should run inference.

        A rate tick is due once ``t_now`` reaches the deadline within the same
        1 ns tolerance the Metal task has always used. Coordinator ticks are
        accumulated floats (``i * 0.01``), so comparing exactly would push a
        deadline of 0.15 to the 0.16 tick and shift every later inference.
        """
        if self._every_n_ticks is not None:
            self._ticks += 1
            return self._ticks % self._every_n_ticks == 0
        if self._next_deadline is None:
            return True
        return t_now + _DEADLINE_TOLERANCE_S >= self._next_deadline

    def mark(self, t_now: float) -> None:
        """Advance the rate deadline after a result was emitted.

        A tick that arrives late schedules the next inference one period from
        *now*, so a stall never produces a catch-up burst of stale inferences.
        """
        if self._period_s is None:
            return
        deadline = (
            self._next_deadline if self._next_deadline is not None else t_now
        ) + self._period_s
        if deadline <= t_now:
            deadline = t_now + self._period_s
        self._next_deadline = deadline


class PolicyCommandHold:
    """Last command emitted by a policy task, with dry-run output gating.

    Policy tasks compute slower than the coordinator ticks, so between policy
    ticks they re-emit the last command. Dry-run runs the actor but withholds
    commands; both behaviours belong to output plumbing, not to a backend.
    """

    def __init__(
        self,
        joint_names: Sequence[str],
        *,
        mode: ControlMode = ControlMode.SERVO_POSITION,
    ) -> None:
        self._joint_names = list(joint_names)
        self._mode = mode
        self._positions: list[float] | None = None
        self._dry_run = False

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    @property
    def positions(self) -> list[float] | None:
        return None if self._positions is None else list(self._positions)

    @property
    def dry_run(self) -> bool:
        return self._dry_run

    def set_dry_run(self, enabled: bool) -> None:
        self._dry_run = bool(enabled)

    def set(self, positions: Iterable[float]) -> None:
        """Record the latest command without deciding whether to emit it."""
        self._positions = [float(value) for value in positions]

    def clear(self) -> None:
        """Forget the held command, e.g. on stop or a session reset."""
        self._positions = None

    def output(self) -> JointCommandOutput | None:
        """Build the held command, ignoring dry-run (used by explicit holds)."""
        if self._positions is None:
            return None
        return JointCommandOutput(
            joint_names=list(self._joint_names),
            positions=list(self._positions),
            mode=self._mode,
        )

    def hold(self) -> JointCommandOutput | None:
        """Re-emit the held command unless dry-run or nothing has run yet."""
        if self._dry_run:
            return None
        return self.output()

    def emit(self, positions: Iterable[float]) -> JointCommandOutput | None:
        """Record a fresh command and emit it unless dry-run is enabled."""
        self.set(positions)
        if self._dry_run:
            return None
        return self.output()


@dataclass(frozen=True)
class PolicyTick:
    """One due policy inference, stamped with the session generation it serves.

    ``payload`` is whatever the backend's :meth:`PolicyTask.act` needs from the
    tick that scheduled it (measured state, goal, observation inputs).  The
    executor treats it as opaque.
    """

    epoch: int
    t_now: float
    reset: bool = False
    payload: Any = None


class PolicyTask(BaseControlTask):
    """Coordinator task skeleton for one ONNX policy.

    Subclasses decide what to observe and when a session may open or must end.
    This class owns the mechanics every policy task repeats:

    * the joint claim and the ``is_active`` loop flag;
    * the cadence, last-command hold and dry-run output gate;
    * the session generation, so a release, e-stop, preemption or stop during
      ``reset``/``act`` makes the in-flight result unusable;
    * the two locks that make cancellation responsive while an actor runs, and
      the single reset -> act -> commit boundary around it.

    ``prepare`` runs under the session lock and must be short: it reads state,
    applies backend policy decisions and returns either an output to emit now
    or a :class:`PolicyTick` asking for one inference.  ``act`` runs under the
    inference lock only, so cancellation never waits for the actor.
    """

    def __init__(
        self,
        name: str,
        joint_names: Sequence[str],
        *,
        scheduler: PolicyTickScheduler,
        priority: int,
        mode: ControlMode = ControlMode.SERVO_POSITION,
        dry_run: bool = False,
    ) -> None:
        self._name = name
        self._joint_names = list(joint_names)
        self._joint_set = frozenset(joint_names)
        self._priority = priority
        self._mode = mode
        self._scheduler = scheduler
        self._hold = PolicyCommandHold(self._joint_names, mode=mode)
        self._hold.set_dry_run(dry_run)
        self._guard = PolicySessionGuard()
        # Session state: held only for short, non-blocking sections.
        self._lock = threading.Lock()
        # Runtime ownership: one observe/act (or reset) at a time.  Never taken
        # while holding ``_lock``, so release/e-stop never wait for the actor.
        self._inference_lock = threading.Lock()
        self._loop_active = False
        self._session_requested = False
        self._requested_at: float | None = None
        self._policy_ticks = 0

    # ----------------------------------------------------------- public state
    @property
    def claim_priority(self) -> int:
        return self._priority

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=self._joint_set,
            priority=self._priority,
            mode=self._mode,
        )

    def is_active(self) -> bool:
        """Active while the loop runs, a session runs, or a request is pending."""
        with self._lock:
            return self._loop_active or self._session_requested or self._guard.active

    @property
    def dry_run(self) -> bool:
        return self._hold.dry_run

    @property
    def session_active(self) -> bool:
        return self._guard.active

    @property
    def session_epoch(self) -> int:
        return self._guard.epoch

    @property
    def policy_ticks(self) -> int:
        return self._policy_ticks

    @property
    def held_positions(self) -> list[float] | None:
        return self._hold.positions

    def set_dry_run(self, enabled: bool) -> None:
        with self._lock:
            self._hold.set_dry_run(enabled)

    def start(self) -> None:
        """Enter the tick loop; no policy session is opened by starting."""
        self._loop_active = True

    def stop(self) -> None:
        """Leave the tick loop and close any session."""
        with self._lock:
            self._loop_active = False
            self.invalidate_session_locked()

    # --------------------------------------------------------- session state
    def request_session(self, t_now: float) -> None:
        """Ask for a session; the next tick decides whether it may open."""
        with self._lock:
            self.request_session_locked(t_now)

    def request_session_locked(self, t_now: float) -> None:
        """Record a session request; the next tick decides whether to open it."""
        self._session_requested = True
        self._requested_at = t_now

    def clear_session_request_locked(self) -> None:
        self._session_requested = False
        self._requested_at = None

    def begin_session_locked(self, t_now: float, *, clear_command: bool = True) -> int:
        """Open a session and return its generation.

        ``clear_command`` is the backend's call: a task that re-emits its
        current pose while arming keeps that command, one that must never
        replay a previous session's target clears it.
        """
        self.clear_session_request_locked()
        self._policy_ticks = 0
        if clear_command:
            self._hold.clear()
        self._scheduler.reset(t_now)
        return self._guard.begin()

    def invalidate_session_locked(self) -> None:
        """Close the session and drop everything tied to it (never resumes)."""
        self._guard.invalidate()
        self._hold.clear()
        self._scheduler.reset()
        self._policy_ticks = 0
        self.clear_session_request_locked()
        self.on_session_ended()

    def invalidate_session(self) -> None:
        with self._lock:
            self.invalidate_session_locked()

    # ------------------------------------------------------------- execution
    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Prepare under the session lock, then run outside it and commit."""
        if not (self._loop_active or self._session_requested or self._guard.active):
            return None
        with self._lock:
            plan = self.prepare(state)
        if plan is None or isinstance(plan, JointCommandOutput):
            return plan
        return self._run_tick(plan)

    def _run_tick(self, tick: PolicyTick) -> JointCommandOutput | None:
        with self._inference_lock:
            with self._lock:
                if not self._guard.current(tick.epoch):
                    return None
            try:
                if tick.reset:
                    self.reset_policy(tick)
                    with self._lock:
                        # A release/e-stop/preemption may have landed in reset.
                        if not self._guard.current(tick.epoch):
                            return None
                command = np.asarray(self.act(tick), dtype=np.float32).reshape(-1)
            except Exception:
                logger.exception("policy inference failed", task=self._name)
                with self._lock:
                    if self._guard.current(tick.epoch):
                        self.on_inference_error(tick)
                return None

        # Output handoff boundary: a command from an invalidated or superseded
        # session is dropped, never emitted and never replayed.
        with self._lock:
            if not self._guard.current(tick.epoch):
                return None
            values = self.commit_guard(tick, command)
            if values is None:
                return None
            values = np.asarray(values, dtype=np.float32).reshape(-1)
            if values.shape != (len(self._joint_names),) or not np.isfinite(values).all():
                logger.error("policy produced an invalid command", task=self._name)
                self.on_invalid_command()
                return None
            self._policy_ticks += 1
            self._scheduler.mark(tick.t_now)
            return self._hold.emit(values)

    # ---------------------------------------------------------------- hooks
    def prepare(self, state: CoordinatorState) -> PolicyTick | JointCommandOutput | None:
        """Decide this tick; runs under the session lock.  Must be quick."""
        raise NotImplementedError

    def reset_policy(self, tick: PolicyTick) -> None:
        """Reset backend runtime state for a fresh session, before ``act``."""

    def act(self, tick: PolicyTick) -> NDArray[np.float32] | NDArray[np.float64]:
        """Compute one command; runs under the inference lock only."""
        raise NotImplementedError

    def commit_guard(
        self, tick: PolicyTick, command: NDArray[np.float32]
    ) -> NDArray[np.float32] | None:
        """Last backend check before emission; runs under the session lock.

        Return the command to emit, or invalidate the session and return None.
        The default accepts the command unchanged.
        """
        return command

    def on_inference_error(self, tick: PolicyTick) -> None:
        """Handle an actor/reset exception; runs under the session lock."""
        self.invalidate_session_locked()

    def on_invalid_command(self) -> None:
        """Handle a wrong-shaped or non-finite command; runs under the lock."""
        self.invalidate_session_locked()

    def on_session_ended(self) -> None:
        """Drop backend state tied to the session; runs under the session lock."""
