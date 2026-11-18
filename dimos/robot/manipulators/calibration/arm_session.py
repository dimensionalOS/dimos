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

"""One calibration tool's hold on an arm: a tuning session streamed at a fixed rate.

:class:`ArmSession` opens a tuning session on an :class:`~dimos.robot.
manipulators.calibration.hooks.ArmTuningHooks` connection and, while it
lasts, sends the arm's targets every tick (at least 50 Hz, well inside the
connection's 0.1 s command deadman, and every call renews the session's
lease). Each tick reads the arm back and stops it when a target would leave
the workspace, when a measured point sags out of it, or when a joint lags its
target by more than ``MAX_POSE_ERROR_DEG``. Ramps are the minimum-jerk ramps
of :mod:`~dimos.robot.manipulators.calibration.motion`. Leaving the session,
for any reason, ends the tuning session: the connection then holds floating
joints where they are, drops feed-forward torques and restores the gains,
and its deadman holds the arm where it is. Nothing here moves the arm home on
its own.

Joint vectors are in the spec's ``joint_names`` order, radians. Times come
from a :class:`Clock`, Unix seconds, the same clock the connection stamps its
readings with, so markers line up with the recorded joint states.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator, Mapping, Sequence
from contextlib import contextmanager
from dataclasses import asdict
import math
import threading
import time
from types import TracebackType
from typing import Any, Literal, Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.robot.manipulators.calibration.float_analysis import FloatEvent
from dimos.robot.manipulators.calibration.gravity_poses import ArmModel
from dimos.robot.manipulators.calibration.hooks import ArmTuningHooks, JointGainsState
from dimos.robot.manipulators.calibration.motion import (
    MAX_POSE_ERROR_DEG,
    SegmentGuard,
    check_step,
    plan_path,
    ramp,
    tracking_error,
)
from dimos.robot.manipulators.calibration.spec import ArmCalibrationSpec
from dimos.robot.manipulators.calibration.step_analysis import REST_WINDOW_S, StepEvent
from dimos.robot.manipulators.damiao.status import DamiaoConnectionStatus
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MarkerStream = Literal["gravity_events", "step_events", "float_events"]
GRAVITY_EVENTS: MarkerStream = "gravity_events"
STEP_EVENTS: MarkerStream = "step_events"
FLOAT_EVENTS: MarkerStream = "float_events"

Vector = NDArray[np.float64]


class AbortRequestedError(Exception):
    """The operator aborted the procedure."""


class StepFailedError(Exception):
    """A step cannot go on; the message says why, in words for the operator."""


class SessionLostError(StepFailedError):
    """The connection ended the tuning session or stopped taking targets."""


class JointDriftedError(StepFailedError):
    """A floating joint moved too far; it was held again."""


class TuningConnection(ArmTuningHooks, Protocol):
    """An arm connection a calibration procedure drives: the tuning hooks and its status."""

    def status(self) -> DamiaoConnectionStatus:
        """The connection's state and the calibration file each arm runs."""
        ...


class Clock(Protocol):
    """Wall time and waiting; tests substitute a simulated clock."""

    def now(self) -> float:
        """Unix seconds."""
        ...

    def sleep(self, seconds: float) -> None: ...

    def wait(self, event: threading.Event, seconds: float) -> bool:
        """Wait up to ``seconds`` for ``event``; whether it is set."""
        ...


class RealClock:
    """``time.time`` and real waits."""

    def now(self) -> float:
        return time.time()

    def sleep(self, seconds: float) -> None:
        if seconds > 0.0:
            time.sleep(seconds)

    def wait(self, event: threading.Event, seconds: float) -> bool:
        return event.wait(max(seconds, 0.0))


class SessionHost(Protocol):
    """What a session needs from the procedure that runs it."""

    def check_abort(self) -> None:
        """Raise ``AbortRequestedError`` once the operator has aborted."""
        ...

    def marker(self, stream: MarkerStream, payload: dict[str, Any]) -> None:
        """Publish a marker to be recorded next to the joint states."""
        ...


def plain_error(error: BaseException) -> str:
    """The message of ``error`` without a remote traceback."""
    return str(error.args[0]) if error.args else type(error).__name__


class TrackingLog:
    """Target minus measured position per joint, deg: while ramping and at rest.

    A hold contributes the last ``REST_WINDOW_S`` of it, when the arm has settled.
    """

    def __init__(self, joint_names: Sequence[str]) -> None:
        self._names = list(joint_names)
        self._ramp: list[Vector] = []
        self._rest: list[Vector] = []

    def add_ramp(self, error_deg: Vector) -> None:
        self._ramp.append(error_deg)

    def add_rest(self, error_deg: Vector) -> None:
        self._rest.append(error_deg)

    def rms_deg(self, *, rest: bool) -> dict[str, float]:
        rows = self._rest if rest else self._ramp
        if not rows:
            return {}
        rms = np.sqrt(np.mean(np.square(np.vstack(rows)), axis=0))
        return {name: float(value) for name, value in zip(self._names, rms, strict=True)}

    def summary(self) -> dict[str, Any]:
        """``rest_rms_deg`` and ``ramp_rms_deg`` by joint, and the sample counts."""
        return {
            "rest_rms_deg": self.rms_deg(rest=True),
            "ramp_rms_deg": self.rms_deg(rest=False),
            "rest_samples": len(self._rest),
            "ramp_samples": len(self._ramp),
        }


class ArmSession:
    """A tuning session on one arm, streamed every tick until the ``with`` block ends.

    Args:
        connection: The arm's connection.
        spec: The arm.
        arm: Its model and workspace (``ArmModel(spec)``).
        host: Abort checks and marker publishing.
        clock: Time and waiting.
        speed_rad_s: Peak joint speed of every ramp.
        stream_hz: Ticks per second.
        lease_s: The tuning session's lease; every tick renews it.
    """

    def __init__(
        self,
        connection: TuningConnection,
        spec: ArmCalibrationSpec,
        arm: ArmModel,
        host: SessionHost,
        clock: Clock,
        *,
        speed_rad_s: float,
        stream_hz: float,
        lease_s: float,
    ) -> None:
        self._connection = connection
        self._spec = spec
        self._arm = arm
        self._host = host
        self._clock = clock
        self._speed = speed_rad_s
        self._period = 1.0 / stream_hz
        self._lease_s = lease_s
        self._names = list(spec.joint_names)
        self._target: Vector = np.zeros(len(self._names))
        self._measured: Vector = np.zeros(len(self._names))
        self._reading_ts = 0.0
        self._gains: dict[str, JointGainsState] = {}
        self._floating: dict[str, float] = {}
        self._guard: SegmentGuard | None = None
        self._next_tick = 0.0
        self._open = False

    def __enter__(self) -> ArmSession:
        self._host.check_abort()
        try:
            self._connection.tuning_begin(self._lease_s)
        except Exception as error:
            raise SessionLostError(f"the connection refused a tuning session: {plain_error(error)}")
        self._open = True
        try:
            self._read()
            self._target = self._measured.copy()
            self._gains = self._call("tuning_get_gains", self._connection.tuning_get_gains)
            self._guard = SegmentGuard(self._arm, self._target, self._target)
            self._next_tick = self._clock.now()
            self._send()
        except BaseException:
            self._close()
            raise
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self._close()

    def _close(self) -> None:
        if not self._open:
            return
        self._open = False
        try:
            self._connection.tuning_end()
        except Exception as error:
            # The lease and the deadman end it anyway; say so and go on.
            logger.warning(f"tuning_end failed, the session lapses on its own: {error}")

    @property
    def joint_names(self) -> list[str]:
        return list(self._names)

    @property
    def target(self) -> Vector:
        return self._target.copy()

    @property
    def measured(self) -> Vector:
        return self._measured.copy()

    @property
    def reading_ts(self) -> float:
        """When the connection took the latest reading."""
        return self._reading_ts

    def gains(self, joint: str) -> tuple[float, float]:
        state = self._gains[joint]
        return (float(state["kp"]), float(state["kd"]))

    def _call(self, name: str, call: Callable[..., Any], *args: Any) -> Any:
        try:
            return call(*args)
        except Exception as error:
            raise SessionLostError(f"{name} failed: {plain_error(error)}") from error

    def _read(self) -> None:
        samples = self._call("tuning_read", self._connection.tuning_read)
        missing = [name for name in self._names if name not in samples]
        if missing:
            raise StepFailedError(f"the connection does not read {missing}")
        measured = np.asarray([samples[name]["position"] for name in self._names], dtype=float)
        if not np.isfinite(measured).all():
            raise StepFailedError(f"the connection reads non-finite positions: {measured.tolist()}")
        self._measured = measured
        self._reading_ts = float(samples[self._names[0]]["ts"])

    def _send(self) -> None:
        commanded = self._target.copy()
        for joint in self._floating:
            commanded[self._names.index(joint)] = self._measured[self._names.index(joint)]
        targets = {name: float(value) for name, value in zip(self._names, commanded, strict=True)}
        result = self._call("tuning_command", self._connection.tuning_command, targets)
        if not result["ok"]:
            raise SessionLostError(f"the connection stopped taking targets: {result['error']}")

    def keepalive(self) -> None:
        """One tick without the wait: read, check, and send the current target."""
        self._host.check_abort()
        self._read()
        self._check()
        self._send()

    def tick(self, until: threading.Event | None = None) -> bool:
        """One tick, then wait for the next; with ``until``, whether it was set meanwhile."""
        self.keepalive()
        self._next_tick += self._period
        delay = self._next_tick - self._clock.now()
        if delay < -self._period:
            # Fell behind (a slow call): restart the schedule instead of bursting.
            self._next_tick = self._clock.now()
            delay = 0.0
        if until is not None and self._clock.wait(until, delay):
            # Woken early: the next tick is due now, not a period after the planned one.
            self._next_tick = self._clock.now()
            return True
        if until is None:
            self._clock.sleep(delay)
        return False

    def wait_for(self, event: threading.Event) -> None:
        """Hold the arm where it is until ``event`` is set."""
        while not self.tick(event):
            pass

    def _check(self) -> None:
        held = [name for name in self._names if name not in self._floating]
        indices = [self._names.index(name) for name in held]
        lag = tracking_error(held, self._target[indices], self._measured[indices])
        if lag is not None:
            raise StepFailedError(f"stopped: {lag}; the arm holds where it is")
        assert self._guard is not None
        sag = self._guard.check_measured(self._measured)
        if sag is not None:
            raise StepFailedError(f"stopped: {sag}; the arm holds where it is")

    def ramp_to(self, target: Vector, log: TrackingLog | None = None) -> None:
        """Ramp straight to ``target``; every target on the way is checked first."""
        end = np.asarray(target, dtype=np.float64)
        guard = SegmentGuard(self._arm, self._target, end)
        self._guard = guard
        for q in ramp(self._target, end, self._speed, self._period):
            refused = guard.check_target(q)
            if refused is not None:
                raise StepFailedError(f"stopped before {refused}; the arm holds where it is")
            self._target = q
            self.tick()
            if log is not None:
                log.add_ramp(np.degrees(self._target - self._measured))
        self._guard = SegmentGuard(self._arm, end, end)

    def move_to(self, target: Vector, log: TrackingLog | None = None) -> None:
        """Go to ``target`` straight when that clears the table, else through home."""
        path = plan_path(self._arm, self._target, np.asarray(target, dtype=np.float64))
        if path is None:
            raise StepFailedError(
                "no ramp from here to the target keeps the workspace clearance, not even "
                "through home; the arm holds where it is"
            )
        for waypoint in path:
            self.ramp_to(waypoint, log)

    def retrace_home(self, visited: Sequence[Vector]) -> None:
        """Go home, backing out through ``visited`` poses (latest last) when home is not direct."""
        home = np.asarray(self._arm.home, dtype=np.float64)
        trail = [np.asarray(pose, dtype=np.float64) for pose in visited]
        while plan_path(self._arm, self._target, home) is None and trail:
            previous = trail.pop()
            if not np.allclose(previous, self._target) and self._arm.transition_ok(
                self._target, previous
            ):
                self.ramp_to(previous)
        self.move_to(home)

    def hold(self, seconds: float, log: TrackingLog | None = None) -> None:
        """Hold the current target for ``seconds``."""
        start = self._clock.now()
        while self._clock.now() - start < seconds - 1e-9:
            self.tick()
            if log is not None and self._clock.now() - start >= seconds - REST_WINDOW_S:
                log.add_rest(np.degrees(self._target - self._measured))

    def set_gains(self, joint: str, kp: float, kd: float) -> tuple[float, float]:
        """Hold ``joint`` with these gains for the session; returns them as applied."""
        state = self._call("tuning_set_gains", self._connection.tuning_set_gains, joint, kp, kd)
        self._gains[joint] = state
        return (float(state["kp"]), float(state["kd"]))

    def reset_gains(self) -> None:
        """Back to the calibration's gains, else the robot's."""
        self._call("tuning_reset_gains", self._connection.tuning_reset_gains)
        self._gains = self._call("tuning_get_gains", self._connection.tuning_get_gains)

    def step_joint(self, joint: str, delta_deg: float, hold_s: float) -> StepEvent:
        """Move ``joint``'s target by ``delta_deg`` at once, mark it, and hold ``hold_s``."""
        check_step(delta_deg)
        index = self._names.index(joint)
        before = float(self._target[index])
        stepped = self._target.copy()
        stepped[index] = before + math.radians(delta_deg)
        guard = SegmentGuard(self._arm, self._target, stepped)
        refused = guard.check_target(stepped)
        if refused is not None:
            raise StepFailedError(f"the {joint} step of {delta_deg:+g} deg was refused: {refused}")
        kp, kd = self.gains(joint)
        event = StepEvent(
            joint=joint,
            delta_deg=float(delta_deg),
            kp=kp,
            kd=kd,
            t0=self._clock.now(),
            hold_s=float(hold_s),
            target_before_deg=math.degrees(before),
            target_after_deg=math.degrees(float(stepped[index])),
        )
        self._host.marker(STEP_EVENTS, asdict(event))
        self._guard = guard
        self._target = stepped
        self.hold(hold_s)
        self._guard = SegmentGuard(self._arm, stepped, stepped)
        return event

    def mark_pose(self, label: str, data: Mapping[str, Any]) -> float:
        """Publish a ``gravity_events`` pose marker with the current targets; its time."""
        t0 = self._clock.now()
        self._host.marker(
            GRAVITY_EVENTS,
            {
                "label": label,
                "t0": t0,
                "targets_deg": {
                    name: math.degrees(float(value))
                    for name, value in zip(self._names, self._target, strict=True)
                },
                "data": dict(data),
            },
        )
        return t0

    @contextmanager
    def floating(self, joint: str, kd: float) -> Iterator[float]:
        """Let ``joint`` float with ``kd`` damping inside the block; yields the damping as applied.

        The joint's target follows its measured position meanwhile, and it is
        held again where it is when the block ends, however it ends.
        """
        state = self._call(
            "tuning_set_float", self._connection.tuning_set_float, joint, True, float(kd)
        )
        index = self._names.index(joint)
        self._floating[joint] = float(self._measured[index])
        try:
            float_kd = state["float_kd"]
            yield float(kd if float_kd is None else float_kd)
        finally:
            del self._floating[joint]
            try:
                self._connection.tuning_set_float(joint, False)
            except Exception as error:
                # Ending the session holds a floating joint too; the caller sees
                # the session loss on its next tick.
                logger.warning(f"releasing {joint} failed: {plain_error(error)}")
            self._target[index] = self._measured[index]
            self._guard = SegmentGuard(self._arm, self._target, self._target)

    def float_window(
        self, joint: str, seconds: float, *, max_drift_deg: float = MAX_POSE_ERROR_DEG
    ) -> tuple[Vector, Vector]:
        """Record ``seconds`` of readings while ``joint`` floats: times and joint rows.

        Raises ``JointDriftedError`` when the joint moves more than ``max_drift_deg`` from
        where it floated.
        """
        if joint not in self._floating:
            raise ValueError(f"{joint} is not floating")
        index = self._names.index(joint)
        times: list[float] = []
        rows: list[Vector] = []
        start = self._clock.now()
        while self._clock.now() - start < seconds - 1e-9:
            self.tick()
            times.append(self._reading_ts)
            rows.append(self._measured.copy())
            drift = math.degrees(float(self._measured[index]) - self._floating[joint])
            if abs(drift) > max_drift_deg:
                raise JointDriftedError(
                    f"{joint} drifted {drift:+.1f} deg while floating (limit "
                    f"{max_drift_deg:g} deg); it is held again"
                )
        return np.asarray(times), np.vstack(rows)

    def mark_float(self, side: str, index: int, duration_s: float, kd: float) -> FloatEvent:
        """Publish a ``float_events`` window marker starting now."""
        event = FloatEvent(
            side=side,
            index=index,
            t0=self._clock.now(),
            duration_s=float(duration_s),
            kd=float(kd),
            floating=True,
            q0_deg={
                name: math.degrees(float(value))
                for name, value in zip(self._names, self._measured, strict=True)
            },
        )
        self._host.marker(FLOAT_EVENTS, asdict(event))
        return event
