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

"""One calibration tool's hold on an arm: a config session streamed at a fixed rate.

:class:`ArmSession` opens a config session on an :class:`~dimos.robot.
manipulators.calibration.hooks.ArmConfigHooks` connection and, while it
lasts, publishes the arm's targets every tick (at least 50 Hz) to an
:class:`ArmDriver`: the control coordinator's pass-through task, which sends
each one to the connection on its next tick, unchanged. Before the session
begins it streams the arm's measured position until the connection takes it,
so the coordinator is commanding the arm when the session starts. An empty
``config_set`` carrying the session's id renews the lease every
``LEASE_RENEW_S``. Each tick reads the arm back from the latest
``joint_state`` message (a :class:`StateSource`) and stops it when the
reading is older than ``MAX_READING_AGE_S``, when a target would leave the
workspace or pass a joint limit, when a measured point sags out of the
workspace, or when a joint lags its target by more than
``MAX_POSE_ERROR_DEG`` while holding, or while ramping by
``RAMP_LAG_LIMIT_DEG`` more than it is expected to
(``motion.ramp_lag_limits_deg``). A floating joint (a gain of kp 0) is held
again when it moves faster than ``FLOAT_SPEED_LIMIT_DEG_S`` or drifts more
than ``FLOAT_DRIFT_LIMIT_DEG``. Ramps are the minimum-jerk ramps of
:mod:`~dimos.robot.manipulators.calibration.motion`. A session method that
raises first has the driver hold the arm where it is (``ArmDriver.hold``), so
the coordinator stops sending the last target. Leaving the session, for any
reason, holds the arm and then ends the config session (``config_reset``):
the connection undoes every live change (floating joints held where they are,
torque offsets dropped, gains restored), and its deadman holds the arm where
it is once the coordinator sends nothing. The only motion home without a
confirmation is :meth:`ArmSession.park`, on a
time budget, which the procedure runs when its module stops or a prompt goes
unanswered.

Joint vectors are in the spec's ``joint_names`` order, radians. Times come
from a :class:`Clock`, Unix seconds, the same clock the connection stamps its
readings with, so markers line up with the recorded joint states.
"""

from __future__ import annotations

from collections.abc import Callable, Collection, Iterator, Mapping, Sequence
from contextlib import contextmanager
from dataclasses import asdict
import functools
import itertools
import math
import threading
import time
from types import TracebackType
from typing import Any, Literal, Protocol, TypeVar, cast

import numpy as np
from numpy.typing import NDArray

from dimos.control.connection.arm_config import (
    ArmConfigState,
    ConfigApplied,
    ConfigPatch,
    JointGains,
)
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.calibration.float_analysis import FloatEvent
from dimos.robot.manipulators.calibration.gravity_poses import ArmModel
from dimos.robot.manipulators.calibration.hooks import ArmConfigHooks
from dimos.robot.manipulators.calibration.motion import (
    MAX_POSE_ERROR_DEG,
    SegmentGuard,
    check_step,
    joint_limits,
    limit_error,
    plan_path,
    ramp,
    ramp_duration_s,
    ramp_lag_limits_deg,
    tracking_error,
)
from dimos.robot.manipulators.calibration.spec import ArmCalibrationSpec
from dimos.robot.manipulators.calibration.step_analysis import REST_WINDOW_S, StepEvent
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MarkerStream = Literal["gravity_events", "step_events", "float_events"]
GRAVITY_EVENTS: MarkerStream = "gravity_events"
STEP_EVENTS: MarkerStream = "step_events"
FLOAT_EVENTS: MarkerStream = "float_events"

Vector = NDArray[np.float64]

# A reading older than this fails the tick: the connection stopped reading the arm.
MAX_READING_AGE_S = 0.25
# A floating joint is held again past either of these.
FLOAT_DRIFT_LIMIT_DEG = 5.0
FLOAT_SPEED_LIMIT_DEG_S = 20.0
# Every joint this close to home counts as at home.
HOME_TOLERANCE_DEG = 3.0
# The longest a session goes without renewing its lease.
LEASE_RENEW_S = 0.5
# Before a session begins, the measured position is streamed at least this
# long, and until the connection has taken it, at most PRIME_TIMEOUT_S.
PRIME_S = 0.1
PRIME_TIMEOUT_S = 1.0

_Method = TypeVar("_Method", bound=Callable[..., Any])


class AbortRequestedError(Exception):
    """The operator aborted the procedure."""


class StepFailedError(Exception):
    """A step cannot go on; the message says why, in words for the operator."""


class SessionLostError(StepFailedError):
    """The connection ended the config session or stopped taking targets."""


class StaleReadingError(StepFailedError):
    """The latest joint state is older than ``MAX_READING_AGE_S``: the connection is not publishing."""


class JointDriftedError(StepFailedError):
    """A floating joint moved too far or too fast; it was held again."""


class ParkRefusedError(Exception):
    """The arm cannot be parked safely; it holds where it is."""


class StateSource(Protocol):
    """Where the arm's joint readings come from: the connection's ``joint_state`` stream."""

    def latest(self) -> JointState | None:
        """The latest message, ``None`` before the first."""
        ...


class ArmDriver(Protocol):
    """Where a session's targets go: the control coordinator's pass-through task."""

    def send(self, targets: JointState) -> None:
        """Publish one sample of position targets."""
        ...

    def hold(self) -> None:
        """Hold the arm where it is now, until the next sample."""
        ...


def joint_reading(
    state: StateSource | None, source: str, names: Sequence[str], clock: Clock
) -> tuple[Vector, Vector, float]:
    """The latest positions and velocities of ``names``, and when they were read.

    The message names joints ``"{source}/{joint}"``, as a connection publishes
    them. Raises ``StepFailedError`` before the first message, or when it
    leaves out one of ``names``, and ``StaleReadingError`` when it is older
    than ``MAX_READING_AGE_S`` by ``clock``, inside a config session or not.
    """
    try:
        msg = None if state is None else state.latest()
    except Exception as error:
        raise StepFailedError(f"the arm's joint states could not be read: {plain_error(error)}")
    if msg is None:
        raise StepFailedError(
            "no joint state has arrived from the connection: it is not reading the arm"
        )
    prefix = f"{source}/"
    index = {
        name.removeprefix(prefix): i for i, name in enumerate(msg.name) if name.startswith(prefix)
    }
    missing = [name for name in names if name not in index]
    if missing:
        raise StepFailedError(f"the connection does not read {missing}")
    age = clock.now() - float(msg.ts)
    if age > MAX_READING_AGE_S:
        raise StaleReadingError(
            f"the latest reading is {age:.2f} s old (limit {MAX_READING_AGE_S:g} s), so the "
            "connection is not publishing the arm's joint states"
        )
    rows = [index[name] for name in names]
    position = np.asarray([msg.position[i] for i in rows], dtype=float)
    if len(msg.velocity) == len(msg.name):
        velocity = np.asarray([msg.velocity[i] for i in rows], dtype=float)
    else:
        velocity = np.zeros(len(rows))
    return position, velocity, float(msg.ts)


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
    Rows are filed under ``pose``, the pose being ramped to or held at, so two
    logs can be compared at the same poses.
    """

    def __init__(self, joint_names: Sequence[str]) -> None:
        self._names = list(joint_names)
        self._ramp: list[tuple[str | None, Vector]] = []
        self._rest: list[tuple[str | None, Vector]] = []
        self.pose: str | None = None

    def add_ramp(self, error_deg: Vector) -> None:
        self._ramp.append((self.pose, error_deg))

    def add_rest(self, error_deg: Vector) -> None:
        self._rest.append((self.pose, error_deg))

    @property
    def poses(self) -> list[str]:
        """The poses held at, in order."""
        return list(dict.fromkeys(pose for pose, _ in self._rest if pose is not None))

    def _rows(self, *, rest: bool, poses: Collection[str] | None) -> list[Vector]:
        rows = self._rest if rest else self._ramp
        return [row for pose, row in rows if poses is None or pose in poses]

    def rms_deg(self, *, rest: bool, poses: Collection[str] | None = None) -> dict[str, float]:
        rows = self._rows(rest=rest, poses=poses)
        if not rows:
            return {}
        rms = np.sqrt(np.mean(np.square(np.vstack(rows)), axis=0))
        return {name: float(value) for name, value in zip(self._names, rms, strict=True)}

    def summary(self, poses: Collection[str] | None = None) -> dict[str, Any]:
        """``rest_rms_deg`` and ``ramp_rms_deg`` by joint and the sample counts, at ``poses``."""
        return {
            "rest_rms_deg": self.rms_deg(rest=True, poses=poses),
            "ramp_rms_deg": self.rms_deg(rest=False, poses=poses),
            "rest_samples": len(self._rows(rest=True, poses=poses)),
            "ramp_samples": len(self._rows(rest=False, poses=poses)),
        }


def _holds_on_failure(method: _Method) -> _Method:
    """Hold the arm before an error from ``method`` goes on."""

    @functools.wraps(method)
    def wrapper(self: ArmSession, *args: Any, **kwargs: Any) -> Any:
        try:
            return method(self, *args, **kwargs)
        except Exception as error:
            self._hold_after(error)
            raise

    return cast("_Method", wrapper)


class ArmSession:
    """A config session on one arm, streamed every tick until the ``with`` block ends.

    Args:
        connection: The arm's connection.
        driver: Where its targets go.
        state: Its joint readings.
        source: Its robot name, the prefix of the readings' joint names.
        spec: The arm.
        arm: Its model and workspace (``ArmModel(spec)``).
        host: Abort checks and marker publishing.
        clock: Time and waiting.
        speed_rad_s: Peak joint speed of every ramp.
        stream_hz: Ticks per second.
        lease_s: The config session's lease, renewed every ``LEASE_RENEW_S`` or half of it.
    """

    def __init__(
        self,
        connection: ArmConfigHooks,
        driver: ArmDriver,
        state: StateSource | None,
        source: str,
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
        self._driver = driver
        self._state = state
        self._source = source
        self._spec = spec
        self._arm = arm
        self._host = host
        self._clock = clock
        self._speed = speed_rad_s
        self._period = 1.0 / stream_hz
        self._lease_s = lease_s
        self._renew_s = min(LEASE_RENEW_S, lease_s / 2.0)
        self._renewed = 0.0
        self._names = list(spec.joint_names)
        self._command_names = [f"{source}/{name}" for name in self._names]
        self._lower, self._upper = joint_limits(spec)
        self._target: Vector = np.zeros(len(self._names))
        self._measured: Vector = np.zeros(len(self._names))
        self._velocity: Vector = np.zeros(len(self._names))
        self._reading_ts = 0.0
        # Per joint while ramping; MAX_POSE_ERROR_DEG for every joint otherwise.
        self._lag_limits_deg: Vector | None = None
        self._session_id = ""
        self._gains: dict[str, JointGains] = {}
        self._floating: dict[str, float] = {}
        self._guard: SegmentGuard | None = None
        self._next_tick = 0.0
        self._open = False
        self._held_for: BaseException | None = None

    def __enter__(self) -> ArmSession:
        self._host.check_abort()
        try:
            self._prime()
        except BaseException:
            self._hold_arm()
            raise
        try:
            started = self._connection.config_set(
                ConfigPatch(arm_id=self._spec.arm_id), scope="session", lease_s=self._lease_s
            )
        except Exception as error:
            self._hold_arm()
            raise SessionLostError(f"the connection refused a config session: {plain_error(error)}")
        if not started.session_id:
            self._hold_arm()
            raise SessionLostError("the connection started no config session")
        self._session_id = started.session_id
        self._renewed = self._clock.now()
        self._open = True
        try:
            self._read()
            self._gains = dict(self._arm_state().gains)
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
        self._hold_arm()
        try:
            self._connection.config_reset()
        except Exception as error:
            # The lease and the deadman end it anyway; say so and go on.
            logger.warning(f"config_reset failed, the session lapses on its own: {error}")

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

    @property
    def session_id(self) -> str:
        """The config session, as ``config_set`` started it."""
        return self._session_id

    def gains(self, joint: str) -> tuple[float, float]:
        state = self._gains[joint]
        return (float(state.kp), float(state.kd))

    def _call(self, name: str, call: Callable[..., Any], *args: Any, **kwargs: Any) -> Any:
        try:
            return call(*args, **kwargs)
        except Exception as error:
            raise SessionLostError(f"{name} failed: {plain_error(error)}") from error

    def _hold_arm(self) -> None:
        """Have the driver hold the arm where it is now; a failure is logged."""
        try:
            self._driver.hold()
        except Exception as error:
            logger.warning(
                f"holding the arm failed; the coordinator stops sending targets once they go "
                f"stale: {plain_error(error)}"
            )

    def _hold_after(self, error: BaseException) -> None:
        """Hold the arm once for ``error``, however many session methods it passes through."""
        if error is not self._held_for:
            self._held_for = error
            self._hold_arm()

    def _prime(self) -> None:
        """Stream the measured position until the connection takes it, before the session begins.

        At least ``PRIME_S``, until the connection reports a command taken
        since the first target went out. Raises ``SessionLostError`` when none
        is within ``PRIME_TIMEOUT_S``: nothing forwards the targets.
        """
        self._read()
        self._target = self._measured.copy()
        self._guard = SegmentGuard(self._arm, self._target, self._target)
        start = self._clock.now()
        self._next_tick = start
        while True:
            self._host.check_abort()
            self._send()
            self._wait_tick(None)
            self._read()
            elapsed = self._clock.now() - start
            if elapsed >= PRIME_S:
                status = self._call("status", self._connection.status)
                taken = status.last_command_time
                if taken is not None and taken >= start:
                    return
            if elapsed >= PRIME_TIMEOUT_S:
                raise SessionLostError(
                    f"the connection took no target within {PRIME_TIMEOUT_S:g} s: the control "
                    "coordinator is not forwarding the calibration's targets"
                )

    def _arm_state(self) -> ArmConfigState:
        states = self._call("config_get", self._connection.config_get)
        state: ArmConfigState | None = states.get(self._spec.arm_id)
        if state is None:
            raise StepFailedError(f"the connection does not report {self._spec.arm_id}")
        return state

    def _set_gains(self, gains: dict[str, JointGains], what: str) -> ConfigApplied:
        """``gains`` for the session; raises ``StepFailedError`` when the connection cannot."""
        applied: ConfigApplied = self._call(
            "config_set",
            self._connection.config_set,
            ConfigPatch(arm_id=self._spec.arm_id, gains=gains),
            scope="session",
            session_id=self._session_id,
        )
        if applied.unsupported:
            reasons = "; ".join(
                f"{key}: {applied.why.get(key, 'not supported')}" for key in applied.unsupported
            )
            raise StepFailedError(f"the connection cannot {what}: {reasons}")
        return applied

    def _read(self) -> None:
        try:
            measured, velocity, reading_ts = joint_reading(
                self._state, self._source, self._names, self._clock
            )
        except StaleReadingError as stale:
            raise StepFailedError(f"stopped: {stale}; the arm holds where it is") from stale
        if not np.isfinite(measured).all():
            raise StepFailedError(f"the connection reads non-finite positions: {measured.tolist()}")
        self._measured = measured
        self._velocity = np.where(np.isfinite(velocity), velocity, 0.0)
        self._reading_ts = reading_ts

    def _send(self) -> None:
        commanded = self._target.copy()
        for joint in self._floating:
            commanded[self._names.index(joint)] = self._measured[self._names.index(joint)]
        targets = JointState(
            ts=self._clock.now(),
            name=list(self._command_names),
            position=[float(value) for value in commanded],
        )
        try:
            self._driver.send(targets)
        except Exception as error:
            raise SessionLostError(f"the targets could not be sent: {plain_error(error)}")

    def _renew(self) -> None:
        """Renew the lease with an empty ``config_set``, ``LEASE_RENEW_S`` after the last renewal."""
        now = self._clock.now()
        if now - self._renewed < self._renew_s:
            return
        self._renewed = now
        self._call(
            "config_set",
            self._connection.config_set,
            ConfigPatch(arm_id=self._spec.arm_id),
            scope="session",
            session_id=self._session_id,
        )

    @_holds_on_failure
    def keepalive(self) -> None:
        """One tick without the wait: read, check, send the current target, renew the lease."""
        self._host.check_abort()
        self._read()
        self._check()
        self._send()
        self._renew()

    @_holds_on_failure
    def tick(self, until: threading.Event | None = None) -> bool:
        """One tick, then wait for the next; with ``until``, whether it was set meanwhile."""
        self.keepalive()
        return self._wait_tick(until)

    def _wait_tick(self, until: threading.Event | None) -> bool:
        """Wait for the next tick; with ``until``, whether it was set meanwhile."""
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

    @_holds_on_failure
    def wait_for(self, event: threading.Event, deadline: float | None = None) -> bool:
        """Hold the arm where it is until ``event`` is set; False when ``deadline`` came first."""
        while not self.tick(event):
            if deadline is not None and self._clock.now() >= deadline:
                return event.is_set()
        return True

    def _check(self) -> None:
        held = [name for name in self._names if name not in self._floating]
        indices = [self._names.index(name) for name in held]
        limits = (
            MAX_POSE_ERROR_DEG if self._lag_limits_deg is None else self._lag_limits_deg[indices]
        )
        lag = tracking_error(held, self._target[indices], self._measured[indices], limits)
        if lag is not None:
            raise StepFailedError(f"stopped: {lag}; the arm holds where it is")
        assert self._guard is not None
        sag = self._guard.check_measured(self._measured)
        if sag is not None:
            raise StepFailedError(f"stopped: {sag}; the arm holds where it is")

    def _refuse_past_limits(self, q: Vector, what: str) -> None:
        past = limit_error(self._names, q, self._lower, self._upper)
        if past is not None:
            raise StepFailedError(
                f"{what} would pass a joint limit ({past}); it was not sent and the arm holds "
                "where it is"
            )

    @_holds_on_failure
    def ramp_to(
        self,
        target: Vector,
        log: TrackingLog | None = None,
        *,
        speed_rad_s: float | None = None,
        deadline: float | None = None,
    ) -> None:
        """Ramp straight to ``target``; every target on the way is checked first.

        ``speed_rad_s`` replaces the session's speed; past ``deadline`` (clock
        time) the ramp stops and the arm holds where it is.
        """
        end = np.asarray(target, dtype=np.float64)
        self._refuse_past_limits(end, "the ramp's end")
        guard = SegmentGuard(self._arm, self._target, end)
        self._guard = guard
        start_lag = np.degrees(self._measured - self._target)
        follow = self._follow_s()
        try:
            for q in ramp(self._target, end, speed_rad_s or self._speed, self._period):
                refused = guard.check_target(q)
                if refused is not None:
                    raise StepFailedError(f"stopped before {refused}; the arm holds where it is")
                if deadline is not None and self._clock.now() > deadline:
                    raise StepFailedError("the time budget ran out; the arm holds where it is")
                speed = np.degrees(q - self._target) / self._period
                self._lag_limits_deg = ramp_lag_limits_deg(start_lag, speed, follow)
                self._target = q
                self.tick()
                if log is not None:
                    log.add_ramp(np.degrees(self._target - self._measured))
        finally:
            self._lag_limits_deg = None
        self._guard = SegmentGuard(self._arm, end, end)

    def _follow_s(self) -> Vector:
        """Each joint's PD following time, kd / kp, s: how far it lags per unit of speed."""
        follow = []
        for name in self._names:
            state = self._gains.get(name)
            kp = 0.0 if state is None else float(state.kp)
            kd = 0.0 if state is None else float(state.kd)
            # A joint without stiffness cannot follow: the hold limit applies once it moves.
            follow.append(kd / kp if kp > 0.0 else 1e6)
        return np.asarray(follow, dtype=np.float64)

    @_holds_on_failure
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

    def hold_here(self) -> None:
        """Target where the arm is now, so a ramp starts from there."""
        self._target = self._measured.copy()
        self._guard = SegmentGuard(self._arm, self._target, self._target)

    @_holds_on_failure
    def park(self, speed_rad_s: float, deadline: float) -> str:
        """Back out home at ``speed_rad_s`` by ``deadline`` (clock time); what it did.

        The arm is first held where it is, then ramps home straight or through
        home's guard-checked path (``motion.plan_path``). Raises
        ``ParkRefusedError``, without moving, when no path keeps the workspace
        clearance or the ramps would not end before ``deadline``; a ramp cut
        short by ``deadline``, a stall or a guard raises ``StepFailedError``.
        Either way the arm holds where it is.
        """
        start = self._clock.now()
        self.hold_here()
        home = np.asarray(self._arm.home, dtype=np.float64)
        if math.degrees(float(np.max(np.abs(self._measured - home)))) <= HOME_TOLERANCE_DEG:
            return "already at home"
        path = plan_path(self._arm, self._target, home)
        if path is None:
            raise ParkRefusedError(
                "no ramp home keeps the workspace clearance from here, not even through home"
            )
        duration = sum(
            ramp_duration_s(a, b, speed_rad_s) for a, b in itertools.pairwise([self._target, *path])
        )
        left = deadline - start
        if duration > left:
            raise ParkRefusedError(
                f"the way home takes {duration:.1f} s at {math.degrees(speed_rad_s):g} deg/s, "
                f"more than the {max(left, 0.0):.1f} s of its time budget left"
            )
        for waypoint in path:
            self.ramp_to(waypoint, speed_rad_s=speed_rad_s, deadline=deadline)
        return f"parked at home in {self._clock.now() - start:.1f} s"

    @_holds_on_failure
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

    @_holds_on_failure
    def hold(self, seconds: float, log: TrackingLog | None = None) -> None:
        """Hold the current target for ``seconds``."""
        start = self._clock.now()
        while self._clock.now() - start < seconds - 1e-9:
            self.tick()
            if log is not None and self._clock.now() - start >= seconds - REST_WINDOW_S:
                log.add_rest(np.degrees(self._target - self._measured))

    @_holds_on_failure
    def set_gains(self, joint: str, kp: float, kd: float) -> tuple[float, float]:
        """Hold ``joint`` with these gains for the session; returns them as applied (clamped).

        Raises ``StepFailedError`` when the connection cannot change them.
        """
        applied = self._set_gains({joint: JointGains(kp=kp, kd=kd)}, f"change {joint}'s gains")
        state = (applied.applied.gains or {}).get(joint)
        if state is None:
            raise StepFailedError(f"the connection did not say which gains {joint} got")
        self._gains[joint] = state
        return (float(state.kp), float(state.kd))

    @_holds_on_failure
    def reset_gains(self) -> None:
        """Undo every live change, keeping the session: the calibration's gains, else the robot's."""
        self._call("config_reset", self._connection.config_reset, keep_session=True)
        self._gains = dict(self._arm_state().gains)

    @_holds_on_failure
    def step_joint(self, joint: str, delta_deg: float, hold_s: float) -> StepEvent:
        """Move ``joint``'s target by ``delta_deg`` at once, mark it, and hold ``hold_s``."""
        check_step(delta_deg)
        index = self._names.index(joint)
        before = float(self._target[index])
        stepped = self._target.copy()
        stepped[index] = before + math.radians(delta_deg)
        self._refuse_past_limits(stepped, f"the {joint} step of {delta_deg:+g} deg")
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

        Floating is a gain of kp 0 (``config_set``); raises ``StepFailedError``
        before the block when the connection cannot float the joint. The
        joint's target follows its measured position meanwhile, and it is held
        again where it is, with the gains it had, when the block ends, however
        it ends.
        """
        applied = self._set_gains({joint: JointGains(kp=0.0, kd=float(kd))}, f"float {joint}")
        index = self._names.index(joint)
        self._floating[joint] = float(self._measured[index])
        try:
            floated = (applied.applied.gains or {}).get(joint)
            yield float(kd if floated is None else floated.kd)
        finally:
            del self._floating[joint]
            try:
                # Its gains as this session holds it; floating never changed them.
                self._connection.config_set(
                    ConfigPatch(arm_id=self._spec.arm_id, gains={joint: self._gains[joint]}),
                    scope="session",
                    session_id=self._session_id,
                )
            except Exception as error:
                # Ending the session holds a floating joint too; the caller sees
                # the session loss on its next tick.
                logger.warning(f"releasing {joint} failed: {plain_error(error)}")
            self._target[index] = self._measured[index]
            self._guard = SegmentGuard(self._arm, self._target, self._target)

    @_holds_on_failure
    def float_window(
        self,
        joint: str,
        seconds: float,
        *,
        max_drift_deg: float = FLOAT_DRIFT_LIMIT_DEG,
        max_speed_deg_s: float = FLOAT_SPEED_LIMIT_DEG_S,
    ) -> tuple[Vector, Vector]:
        """Record ``seconds`` of readings while ``joint`` floats: times and joint rows.

        Raises ``JointDriftedError`` when the joint moves more than ``max_drift_deg``
        from where it floated, or faster than ``max_speed_deg_s``: it is falling.
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
            speed = math.degrees(float(self._velocity[index]))
            if abs(drift) > max_drift_deg:
                raise JointDriftedError(
                    f"{joint} drifted {drift:+.1f} deg while floating (limit "
                    f"{max_drift_deg:g} deg); it is held again"
                )
            if abs(speed) > max_speed_deg_s:
                raise JointDriftedError(
                    f"{joint} drifted at {speed:+.0f} deg/s while floating (limit "
                    f"{max_speed_deg_s:g} deg/s): it was falling; it is held again"
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
