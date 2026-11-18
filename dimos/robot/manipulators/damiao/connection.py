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

"""The DimOS side of a Damiao robot: ports, limits, timing and faults.

The motor layer lives in :mod:`dimos.robot.manipulators.damiao.arm` and the
robot's data in :mod:`dimos.robot.manipulators.damiao.robot`. This module only
translates streams into targets, publishes readings at a fixed rate, and
latches a fault when feedback breaks.

At start it applies each arm's saved calibration file, if there is one
(``DamiaoConnectionConfig.calibration``): the file's gains and fitted link
masses. Without a file the arm runs exactly its URDF and bus defaults.

A connection deployed with ``tuning=True`` (calibration blueprints only)
serves the ``tuning_*`` RPCs of :class:`~dimos.robot.manipulators.
calibration.hooks.ArmTuningHooks`: one calibration tool owns the arm inside a
tuning session (:meth:`DamiaoConnection.tuning_begin`) and changes gains live.
Stopping such a connection first gives the calibration tool, stopping at the
same time, up to ``TUNING_PARK_WAIT_S`` to park the arm and end its session
(:meth:`DamiaoConnection.stop`): the modules of a run stop in parallel, so
nothing else orders the two.
"""

from __future__ import annotations

from abc import abstractmethod
from collections.abc import Callable, Iterator, Mapping, Sequence
from contextlib import contextmanager
from dataclasses import dataclass, fields
import functools
import hashlib
import numbers
from pathlib import Path
import threading
import time
from typing import Any, ClassVar, Concatenate, ParamSpec, TypeVar
import uuid

from pydantic import Field, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import (
    ConnectionConfig,
    ConnectionModule,
    ConnectionStatus,
    Values,
)
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import limits as limits_of, movable_joints
from dimos.control.contract.validate import Rejected, validate_command
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.calibration._inertial import LinkInertial
from dimos.robot.manipulators.calibration.hooks import (
    AppliedCalibration,
    AppliedInertials,
    CalibrationApplied,
    CommandResult,
    JointGainsState,
    JointSample,
    LinkMass,
    TuningSession,
)
from dimos.robot.manipulators.calibration.schema import (
    ARMS_STATE_DIR,
    ArmCalibration,
    ArmCalibrationError,
    calibration_file,
    calibration_path,
    load_calibration,
    to_inertial_overrides,
)
from dimos.robot.manipulators.calibration.spec import model_sha256
from dimos.robot.manipulators.damiao.arm import ArmReading, DamiaoArm, TuningReset
from dimos.robot.manipulators.damiao.robot import DamiaoRobot
from dimos.robot.manipulators.damiao.status import DamiaoConnectionStatus
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

__all__ = [
    "CALIBRATION_ROOT",
    "MAX_GRAVITY_STEP_NM",
    "MAX_TUNING_LEASE_S",
    "TUNING_PARK_GRACE_S",
    "TUNING_PARK_WAIT_S",
    "DamiaoConnection",
    "DamiaoConnectionConfig",
    "DamiaoConnectionStatus",
    "zero_stiffness_joints",
]

#: Where "auto" looks for each arm's calibration, ``<root>/<arm_id>/calibration.json``.
#: Read at every load, so tests can point it at a scratch directory.
CALIBRATION_ROOT: Path = ARMS_STATE_DIR
#: The longest lease a tuning session may ask for, seconds.
MAX_TUNING_LEASE_S = 30.0
#: The most any joint's gravity torque at the measured pose may change when a
#: calibration is applied live without ``force``, N m.
MAX_GRAVITY_STEP_NM = 2.0
#: How long ``stop`` waits for the calibration tool to park the arm and end its
#: tuning session before it halts and disables the motors, seconds.
TUNING_PARK_WAIT_S = 10.0
#: How long ``stop`` waits for a calibration tool that is about to park (no
#: session yet, or one that just ended) to begin its park session, seconds.
TUNING_PARK_GRACE_S = 1.0

_P = ParamSpec("_P")
_T = TypeVar("_T")


def zero_stiffness_joints(robot: DamiaoRobot, *, teach: bool, source: str = "arm") -> list[str]:
    """Joints a teaching connection commands with zero stiffness, in joint order.

    Hand-guided tasks use this declaration to prove the deployment will not
    fight the operator: every arm joint plus the jaws that follow the hand.
    Names carry the connection's robot name, e.g. ``arm/yam_joint1``.
    """
    if not teach:
        return []
    names = [
        *robot.arm_joint_names,
        *(gripper.joint for gripper in robot.grippers if gripper.name in robot.passive_grippers),
    ]
    return [f"{source}/{name}" for name in names]


class DamiaoConnectionConfig(ConnectionConfig):
    """Settings of a Damiao connection.

    Attributes:
        teach: Drive every arm joint with zero stiffness so an operator can
            guide it by hand.
        state_rate_hz: How often the joints are read and published.
        deadman_timeout_s: How long without a command before the arm holds.
        calibration: Which calibration to apply at start. "auto" applies each
            arm's saved file (``calibration_path(arm_id)``) when there is one;
            "off" applies none; anything else is the path of one calibration
            file, applied to the arm it names. An arm without a file, or whose
            file does not fit this robot, runs its URDF masses and bus gains.
        tuning: Serve the ``tuning_*`` RPCs, which change how the energized
            motors are driven. Only a calibration blueprint turns this on;
            without it every tuning RPC raises.
    """

    teach: bool = False
    state_rate_hz: float = Field(default=100.0, gt=0, allow_inf_nan=False)
    deadman_timeout_s: float = Field(default=0.1, gt=0, allow_inf_nan=False)
    calibration: str = Field(default="auto", min_length=1)
    tuning: bool = False

    @model_validator(mode="after")
    def _position_only(self) -> Self:
        if any(interfaces != (Interface.POSITION,) for interfaces in self.joint_commands.values()):
            raise ValueError("Damiao connections accept position commands only")
        return self


@dataclass(frozen=True, slots=True)
class _ArmFile:
    """One arm's calibration file, read and checked against the robot."""

    path: Path
    calibration: ArmCalibration
    gains: dict[str, tuple[float, float]]
    links: dict[str, LinkInertial]
    sha256: str

    def summary(self) -> AppliedCalibration:
        return AppliedCalibration(
            path=str(self.path),
            created=self.calibration.created.isoformat(),
            gains=sorted(self.gains),
            links=sorted(self.links),
        )

    def inertials(self) -> AppliedInertials:
        return AppliedInertials(
            path=str(self.path),
            created=self.calibration.created.isoformat(),
            sha256=self.sha256,
            links={
                name: LinkMass(mass_kg=link.mass_kg, com_m=list(link.com_m))
                for name, link in sorted(self.links.items())
            },
        )


@dataclass(slots=True)
class _TuningSession:
    """The calibration tool's hold on the arm, until ``deadline`` (``time.monotonic``)."""

    session_id: str
    lease_s: float
    deadline: float
    # Position commands dropped while the session lasts.
    ignored: int = 0


def _tuning_rpc(
    method: Callable[Concatenate[DamiaoConnection, _P], _T],
) -> Callable[Concatenate[DamiaoConnection, _P], _T]:
    """A tuning RPC: refused unless ``config.tuning``; raising ends the tuning session."""

    def call(self: DamiaoConnection, /, *args: _P.args, **kwargs: _P.kwargs) -> _T:
        if not self.config.tuning:
            raise RuntimeError(
                f"{self._label}: tuning is off; only a connection deployed for "
                "calibration (tuning=True) serves the tuning_* RPCs"
            )
        try:
            return method(self, *args, **kwargs)
        except Exception as error:
            with self._hardware_lock:
                self._end_tuning(f"{method.__name__} raised {type(error).__name__}: {error}")
            raise

    functools.update_wrapper(call, method)
    return call


def _real(name: str, value: Any) -> float:
    """``value`` as a float; a bool, a string or anything else not a number raises ``TypeError``."""
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise TypeError(f"{name} must be a number, got {value!r}")
    return float(value)


def _flag(name: str, value: Any) -> bool:
    """``value`` if it is a real bool; anything else, the string "false" included, raises."""
    if not isinstance(value, bool):
        raise TypeError(f"{name} must be True or False, got {value!r}")
    return value


def _merged(
    files: Mapping[str, _ArmFile | None],
) -> tuple[dict[str, tuple[float, float]], dict[str, LinkInertial]]:
    """Every file's gains and link masses together; arms never share joints or links."""
    gains: dict[str, tuple[float, float]] = {}
    links: dict[str, LinkInertial] = {}
    for item in files.values():
        if item is not None:
            gains.update(item.gains)
            links.update(item.links)
    return gains, links


def _undone(reset: TuningReset) -> str:
    parts = []
    if reset.released:
        parts.append(f"held {reset.released} where they are")
    if reset.feedforward:
        parts.append(f"dropped the feed-forward of {reset.feedforward}")
    if reset.gains:
        parts.append(f"gains of {reset.gains} back to the calibration")
    return "; ".join(parts) or "nothing to undo"


class DamiaoConnection(ConnectionModule):
    """Own one Damiao robot, its measured states and position targets.

    The ``tuning_*`` RPCs implement
    :class:`~dimos.robot.manipulators.calibration.hooks.ArmTuningHooks`. They
    change motor gains and torques of the energized arm live, are served only
    with ``config.tuning`` and are for calibration tools only; see
    :meth:`tuning_begin` for the session they run in.
    """

    #: The robot this connection deploys: motors, gains, grippers and models.
    robot: ClassVar[DamiaoRobot]
    config: DamiaoConnectionConfig
    position_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        kwargs.setdefault(
            "joint_commands", {name: (Interface.POSITION,) for name in self.robot.joint_names}
        )
        super().__init__(**kwargs)
        self._arm: DamiaoArm | None = None
        self._fault: str | None = None
        self._state_frame = ""
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None
        # Guarded by _lock: the calibration file each arm runs, None for none.
        self._applied: dict[str, _ArmFile | None] = {}
        self._model_hash: str | None = None
        # The latest reading and when it was taken, for tuning_read().
        self._latest: tuple[float, ArmReading] | None = None
        # Guarded by _lock, and changed only under _hardware_lock as well:
        # the tuning session, and why the last one ended.
        self._tuning_session: _TuningSession | None = None
        self._tuning_ended = ""
        # When the last session ended, time.monotonic(); None before the first.
        self._tuning_ended_at: float | None = None
        # Set on the thread inside tuning_command: write() lets it through a
        # tuning session and records what became of it.
        self._tuning_writer = threading.local()

    @property
    def joint_names(self) -> list[str]:
        return list(self.robot.joint_names)

    @property
    def arm_joints(self) -> dict[str, list[str]]:
        return self.robot.arm_joints

    @property
    def gripper_joints(self) -> dict[str, str]:
        return self.robot.gripper_joints

    @abstractmethod
    def can_ports(self) -> list[str]:
        """Return one explicitly configured CAN selector per robot bus, in order."""

    def connect(self) -> None:
        """Validate the deployed model and topology, then open the buses."""
        assert self.config.model is not None
        if {joint.name for joint in movable_joints(self.config.model)} != set(self.joint_names):
            raise ValueError("connection model joints must match the vendor topology")
        self._state_frame = self.frame(self.config.model.load().root_link)
        ports = self.can_ports()
        if len(ports) != len(self.robot.buses) or any(not port.strip() for port in ports):
            raise ValueError("CAN selectors must cover exactly the robot buses")
        if len(set(ports)) != len(ports):
            raise ValueError("CAN selectors must be distinct")
        arm = DamiaoArm(self.robot, ports, teach=self.config.teach)
        arm.connect()
        self._arm = arm
        self._read_state()
        # Before enable, so the first torque the motors get is the calibrated one.
        files = self._read_calibrations(arm, self.config.calibration, strict=False)
        self._apply_calibrations(arm, files, always=False)

    @rpc
    def start(self) -> None:
        try:
            Module.start(self)
            self._session_id = uuid.uuid4().hex
            self._limits = limits_of(self.config)
            self.connect()
            with self._hardware_lock:
                arm = self._arm
                assert arm is not None
                logger.info(
                    "Enabling robot and calibrating grippers",
                    connection=type(self).__name__,
                    source=self.config.source,
                )
                arm.enable()
                self._read_state()
                self.register_disposable(
                    Disposable(self.position_command.subscribe(self._on_position))
                )
                self._reader = threading.Thread(
                    target=self._read_loop, daemon=True, name=f"{self.config.source}-read"
                )
                self._reader.start()
                self._connected = True
                logger.info(
                    "Robot connection ready",
                    connection=type(self).__name__,
                    source=self.config.source,
                )
        except Exception:
            try:
                self.stop()
            except Exception as cleanup_error:
                self._report(
                    f"startup cleanup failed: {type(cleanup_error).__name__}: {cleanup_error}"
                )
            raise

    @rpc
    def stop(self) -> None:
        if self.config.tuning:
            self._await_tuning_park()
        self._stop_reading.set()
        if (
            self._reader is not None
            and self._reader.is_alive()
            and self._reader is not threading.current_thread()
        ):
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        try:
            try:
                super().stop()
            finally:
                with self._hardware_lock:
                    self._end_tuning("the connection stopped")
                    arm, self._arm = self._arm, None
                    try:
                        if arm is not None:
                            arm.disable()
                    finally:
                        self._fault = None
        finally:
            if (
                self._reader is not None
                and self._reader.is_alive()
                and self._reader is not threading.current_thread()
            ):
                self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
                if self._reader.is_alive():
                    error = f"{self.config.source} reader did not stop before the shutdown timeout"
                    self._report(error)
                    raise RuntimeError(error)

    def _await_tuning_park(self) -> None:
        """Before stopping, let the calibration tool park the arm and end its session.

        The modules of a run stop in parallel, so the calibration module may
        still be backing the arm out home when this connection is told to
        stop, and disabling the motors then would drop the arm where it is.
        Waits while a tuning session lasts, and for ``TUNING_PARK_GRACE_S``
        after the stop began or the last session ended for one about to
        begin, at most ``TUNING_PARK_WAIT_S`` in all. Readings, the deadman
        and the session lease run as usual meanwhile.
        """
        started = time.monotonic()
        deadline = started + TUNING_PARK_WAIT_S
        logged = False
        while True:
            with self._lock:
                active = self._tuning_session is not None
                ended_at = self._tuning_ended_at
            if not self._connected or self._fault is not None or self._arm is None:
                return
            now = time.monotonic()
            if not active:
                since = max(started, ended_at if ended_at is not None else started)
                if now - since >= TUNING_PARK_GRACE_S:
                    return
            elif not logged:
                logger.warning(
                    f"{self._label}: stopping; waiting up to {TUNING_PARK_WAIT_S:g} s for the "
                    "calibration tool to park the arm and end its tuning session"
                )
                logged = True
            if now >= deadline:
                logger.error(
                    f"{self._label}: the calibration tool did not end its tuning session within "
                    f"{TUNING_PARK_WAIT_S:g} s; halting and disabling the arm where it is"
                )
                return
            time.sleep(0.02)

    @rpc
    def status(self) -> DamiaoConnectionStatus:
        """A snapshot of the connection and the calibration each arm runs."""
        base = super().status()
        with self._lock:
            applied = dict(self._applied)
        return DamiaoConnectionStatus(
            **{item.name: getattr(base, item.name) for item in fields(ConnectionStatus)},
            calibration={
                arm_id: None if item is None else item.summary() for arm_id, item in applied.items()
            },
        )

    @rpc
    @_tuning_rpc
    def tuning_begin(self, lease_s: float = 2.0) -> TuningSession:
        """Start the one tuning session: until it ends, ``position_command`` is ignored.

        The gain, float and feed-forward setters and ``tuning_command`` need
        a session; every ``tuning_*`` call renews its lease. It ends with
        :meth:`tuning_end`, when ``lease_s`` (at most ``MAX_TUNING_LEASE_S``)
        passes without a tuning call, when the connection halts (the command
        deadman included: keep streaming ``tuning_command``), faults or stops,
        and when any tuning call raises. Ending it holds floating joints where
        they are, drops feed-forward torques and gain overrides (back to the
        applied calibration, else the bus defaults) and logs one line. Raises
        while another session is active, and so, like any tuning call that
        raises, ends that session too. For calibration tools only.
        """
        lease = _real("lease_s", lease_s)
        if not 0.0 < lease <= MAX_TUNING_LEASE_S:
            raise ValueError(
                f"lease_s must be above 0 and at most {MAX_TUNING_LEASE_S:g} s, got {lease_s}"
            )
        with self._hardware_lock:
            self._tuning_arm()
            if not self._connected:
                raise RuntimeError("the Damiao connection is not running")
            with self._lock:
                if self._tuning_session is not None:
                    raise RuntimeError(
                        "a tuning session is already active; it ends with tuning_end or "
                        "when its lease lapses"
                    )
                session = _TuningSession(uuid.uuid4().hex, lease, time.monotonic() + lease)
                self._tuning_session = session
        logger.info(
            f"{self._label}: tuning session {session.session_id[:8]} started, lease {lease:g} s; "
            "position_command is ignored until it ends"
        )
        return TuningSession(session_id=session.session_id, lease_s=lease)

    @rpc
    @_tuning_rpc
    def tuning_end(self) -> None:
        """End the tuning session and undo every live change; without one, nothing happens."""
        with self._hardware_lock:
            self._end_tuning("tuning_end", expected=True)

    @rpc
    @_tuning_rpc
    def tuning_arm_ids(self) -> list[str]:
        """The calibration ids of this robot's arms, in bus order. For calibration tools only."""
        self._renew(required=False)
        return self.robot.arm_ids()

    @rpc
    @_tuning_rpc
    def tuning_joint_names(self) -> list[str]:
        """The arm joints whose gains can be tuned, in motor order. For calibration tools only."""
        self._renew(required=False)
        return self.robot.arm_joint_names

    @rpc
    @_tuning_rpc
    def tuning_get_gains(self) -> dict[str, JointGainsState]:
        """How each arm joint is driven now. For calibration tools only."""
        with self._tuning_call(session=False) as arm:
            return {joint: self._gains_state(arm, joint) for joint in self.robot.arm_joint_names}

    @rpc
    @_tuning_rpc
    def tuning_set_gains(self, joint: str, kp: float, kd: float) -> JointGainsState:
        """Hold ``joint`` with these gains for the session, clamped to kp 0..500 and kd 0..5.

        kp in N m/rad, kd in N m s/rad; returns them as applied. Needs a
        tuning session and changes the energized motor's gains at once; for
        calibration tools only.
        """
        with self._tuning_call(session=True) as arm:
            arm.set_gains(joint, _real("kp", kp), _real("kd", kd))
            return self._gains_state(arm, joint)

    @rpc
    @_tuning_rpc
    def tuning_reset_gains(self) -> None:
        """Back to the calibration's gains, else the bus defaults. For calibration tools only."""
        with self._tuning_call(session=False) as arm:
            arm.reset_gains()

    @rpc
    @_tuning_rpc
    def tuning_set_float(self, joint: str, on: bool, kd: float | None = None) -> JointGainsState:
        """Let ``joint`` float with zero stiffness, or hold it again where it is.

        Gravity stays compensated; a robot that does not compensate gravity
        refuses with ``ValueError``. Needs a tuning session and changes the
        energized motor's gains at once; for calibration tools only.
        """
        on = _flag("on", on)
        damping = None if kd is None else _real("kd", kd)
        with self._tuning_call(session=True) as arm:
            arm.set_float(joint, on, damping)
            return self._gains_state(arm, joint)

    @rpc
    @_tuning_rpc
    def tuning_set_feedforward(self, joint: str, tau_nm: float) -> JointGainsState:
        """Add ``tau_nm`` to ``joint``'s gravity torque, clamped to a fifth of its rated torque.

        For friction-sized terms, not to hold a load. Returns the joint's
        state with the torque as applied. Needs a tuning session and changes
        the energized motor's torque at once; for calibration tools only.
        """
        with self._tuning_call(session=True) as arm:
            arm.set_feedforward(joint, _real("tau_nm", tau_nm))
            return self._gains_state(arm, joint)

    @rpc
    @_tuning_rpc
    def tuning_reset_all(self) -> None:
        """Undo every live change as ending the session does; the session goes on.

        Floating joints are held where they are, feed-forward torques are
        dropped and gains go back to the calibration, else the bus defaults.
        For calibration tools only.
        """
        with self._tuning_call(session=False) as arm:
            reset = arm.reset_tuning()
        logger.info(f"{self._label}: tuning reset: {_undone(reset)}")

    @rpc
    @_tuning_rpc
    def tuning_read(self) -> dict[str, JointSample]:
        """The latest reading of every joint, as published. For calibration tools only."""
        with self._tuning_call(session=False):
            latest = self._latest
        if latest is None:
            raise RuntimeError("the Damiao robot has not been read yet")
        ts, reading = latest
        return {
            name: JointSample(position=position, velocity=velocity, effort=effort, ts=ts)
            for name, position, velocity, effort in zip(
                self.joint_names, reading.position, reading.velocity, reading.effort, strict=True
            )
        }

    @rpc
    @_tuning_rpc
    def tuning_command(self, targets: dict[str, float]) -> CommandResult:
        """Send position targets by joint name, as a ``position_command`` would.

        Needs a tuning session. The targets are checked against the joint
        limits and restart the deadman; stop sending and the arm holds, which
        ends the session. A refused command raises ``ValueError``; one the
        motors did not get returns ``ok`` False with the reason. Nothing
        limits how far one command jumps, so step the targets at the rate the
        arm may move. For calibration tools only.
        """
        if not isinstance(targets, dict):
            raise TypeError(f"targets must be a dict of joint name to position, got {targets!r}")
        unknown = sorted(set(targets) - set(self.joint_names))
        if unknown:
            raise ValueError(f"not joints of this Damiao robot: {unknown}")
        values = {
            f"{self.config.source}/{name}/{Interface.POSITION.value}": _real(name, value)
            for name, value in targets.items()
        }
        result = validate_command(values, self.limits)
        if isinstance(result, Rejected):
            raise ValueError(f"{result.reason}: {result.detail}")
        with self._tuning_call(session=True):
            if not self._connected:
                raise RuntimeError("the Damiao connection is not running")
        writer = self._tuning_writer
        writer.active, writer.written, writer.error = True, False, None
        try:
            self.command(values)
        finally:
            writer.active = False
        if writer.error is not None:
            return CommandResult(ok=False, error=writer.error)
        if not writer.written:
            return CommandResult(ok=False, error="the Damiao connection is not running")
        return CommandResult(ok=True, error=None)

    @rpc
    @_tuning_rpc
    def tuning_apply_calibration(
        self, path: str | None = None, force: bool = False
    ) -> CalibrationApplied:
        """Apply a calibration now; returns what each arm runs and the gravity torque steps.

        ``None`` re-reads each arm's saved file, as "auto" does at start;
        "off" returns every arm to its URDF masses and bus gains; any other
        value is one file, applied to the arm it names while every other arm
        keeps what it runs. A file that does not fit raises
        ``ArmCalibrationError`` with nothing changed.

        Apply near home. New masses change the gravity torques at once; when
        any joint's, at the measured pose, would change by more than
        ``MAX_GRAVITY_STEP_NM``, nothing is applied and ``ValueError`` is
        raised, unless ``force``. For calibration tools only.
        """
        if path is not None and not isinstance(path, str):
            raise TypeError(f"path must be a string or None, got {path!r}")
        force = _flag("force", force)
        source = "auto" if path is None else path
        with self._tuning_call(session=False) as arm:
            with self._lock:
                running = dict(self._applied)
            if source in ("auto", "off"):
                files = self._read_calibrations(arm, source, strict=False)
            else:
                files = self._read_calibrations(arm, source, strict=True, keep=running)
            _, links = _merged(files)
            before, after = arm.gravity(), arm.gravity(links)
            step = {joint: after[joint] - before[joint] for joint in before}
            large = {
                joint: round(value, 2)
                for joint, value in step.items()
                if abs(value) > MAX_GRAVITY_STEP_NM
            }
            if large and not force:
                raise ValueError(
                    f"applying {source} would change the gravity torque at this pose by more "
                    f"than {MAX_GRAVITY_STEP_NM:g} N m: {large} (N m); apply calibrations near "
                    "home, or pass force=True"
                )
            arms = self._apply_calibrations(arm, files, always=True)
        return CalibrationApplied(arms=arms, gravity_step_nm=step)

    @rpc
    @_tuning_rpc
    def tuning_applied_calibration(self) -> dict[str, AppliedInertials | None]:
        """What each arm runs, by arm id: its file, the file's sha256 and the link masses.

        ``None`` for an arm on its URDF masses and bus gains. The masses are
        those read when the file was applied; the file may have changed on
        disk since. For calibration tools only.
        """
        self._renew(required=False)
        with self._lock:
            applied = dict(self._applied)
        return {
            arm_id: None if item is None else item.inertials() for arm_id, item in applied.items()
        }

    def _tuning_arm(self) -> DamiaoArm:
        """The connected arm, for a tuning call made under the hardware lock."""
        if self._fault is not None:
            raise RuntimeError(self._fault)
        if self._arm is None:
            raise RuntimeError("Damiao robot is disconnected")
        return self._arm

    @contextmanager
    def _tuning_call(self, *, session: bool) -> Iterator[DamiaoArm]:
        """The connected arm under the hardware lock, the session renewed (or ``required``)."""
        with self._hardware_lock:
            arm = self._tuning_arm()
            self._renew(required=session)
            yield arm

    def _renew(self, *, required: bool) -> None:
        """Renew the tuning session's lease; without a session raise if one is ``required``."""
        with self._lock:
            session = self._tuning_session
            if session is not None:
                session.deadline = time.monotonic() + session.lease_s
                return
            ended = self._tuning_ended
        if required:
            since = f" (the last one ended: {ended})" if ended else ""
            raise RuntimeError(f"no tuning session is active{since}; call tuning_begin first")

    def _end_tuning(self, reason: str, *, expected: bool = False, lapsed: bool = False) -> None:
        """End the tuning session, if any (with ``lapsed``, only one past its lease).

        Holds floating joints where they are, drops feed-forward torques and
        gain overrides, and logs one line. Call with the hardware lock held.
        """
        with self._lock:
            session = self._tuning_session
            if session is None or (lapsed and time.monotonic() <= session.deadline):
                return
            if lapsed:
                reason = f"{reason} of {session.lease_s:g} s"
            self._tuning_session = None
            self._tuning_ended = reason
            self._tuning_ended_at = time.monotonic()
        undone = "nothing undone"
        arm = self._arm
        if arm is not None:
            try:
                undone = _undone(arm.reset_tuning())
            except Exception as error:
                undone = f"undoing it failed: {type(error).__name__}: {error}"
                self._report(f"tuning session end failed: {type(error).__name__}: {error}")
        message = (
            f"{self._label}: tuning session {session.session_id[:8]} ended ({reason}): "
            f"{undone}; {session.ignored} position commands ignored"
        )
        if expected:
            logger.info(message)
        else:
            logger.warning(message)

    def _ignored_for_tuning(self) -> bool:
        """Whether a position command is dropped for the tuning session; warns once per session."""
        with self._lock:
            session = self._tuning_session
            if session is None:
                return False
            session.ignored += 1
            first = session.ignored == 1
        if first:
            logger.warning(
                f"{self._label}: ignoring position_command while a tuning session owns the arm"
            )
        return True

    def _gains_state(self, arm: DamiaoArm, joint: str) -> JointGainsState:
        kp, kd = arm.get_gains()[joint]
        floating = arm.floating
        return JointGainsState(
            kp=kp,
            kd=kd,
            floating=joint in floating,
            float_kd=floating.get(joint),
            feedforward_nm=arm.feedforward.get(joint, 0.0),
        )

    def _arm_joints_by_id(self) -> dict[str, list[str]]:
        ids = self.robot.arm_ids()
        if not ids:
            return {}
        return {
            arm_id: [motor.name for motor in bus.arm]
            for arm_id, bus in zip(ids, self.robot.buses, strict=True)
        }

    def _robot_model_sha256(self) -> str:
        if self._model_hash is None:
            self._model_hash = model_sha256(self.robot.gravity_model)
        return self._model_hash

    def _read_calibrations(
        self,
        arm: DamiaoArm,
        source: str,
        *,
        strict: bool,
        keep: Mapping[str, _ArmFile | None] | None = None,
    ) -> dict[str, _ArmFile | None]:
        """Each arm's calibration from ``source`` ("auto", "off" or a file), checked.

        A file that does not fit is logged as a warning and that arm gets its
        defaults, or, when ``strict``, raised. With one file, the arms it does
        not name get what ``keep`` gives them, else their defaults.
        """
        arms = self._arm_joints_by_id()
        files: dict[str, _ArmFile | None] = dict.fromkeys(arms)
        if source == "off":
            logger.info(f"{self._label}: calibration is off, using URDF defaults for {list(arms)}")
            return files
        if source == "auto":
            if not arms:
                logger.debug(f"{self._label}: the robot has no name, so no calibration is applied")
            for arm_id in arms:
                path = calibration_path(arm_id, CALIBRATION_ROOT)
                if not path.exists():
                    logger.info(f"No calibration for {arm_id}, using URDF defaults")
                    continue
                try:
                    files[arm_id] = self._read_file(arm, path, arm_id)
                except (ArmCalibrationError, ValueError) as error:
                    logger.warning(
                        f"Ignoring the calibration for {arm_id}, using URDF defaults: {error}"
                    )
            return files
        files.update({arm_id: item for arm_id, item in (keep or {}).items() if arm_id in files})
        path = calibration_file(Path(source).expanduser())
        try:
            item = self._read_file(arm, path)
            files[item.calibration.arm_id] = item
        except (ArmCalibrationError, ValueError) as error:
            if strict:
                raise
            logger.warning(f"Ignoring the calibration {path}, using URDF defaults: {error}")
        for arm_id, found in files.items():
            if found is None:
                logger.info(f"No calibration for {arm_id}, using URDF defaults")
        return files

    def _read_file(self, arm: DamiaoArm, path: Path, arm_id: str | None = None) -> _ArmFile:
        """One calibration file, refused unless made for this model and one of its arms."""
        calibration = load_calibration(
            path, expected_model_sha256=self._robot_model_sha256(), expected_arm_id=arm_id
        )
        sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        arms = self._arm_joints_by_id()
        if calibration.arm_id not in arms:
            raise ArmCalibrationError(
                f"The arm calibration at {path} is for arm {calibration.arm_id!r}; "
                f"this robot's arms are {list(arms)}."
            )
        return self._check_calibration(arm, path, calibration, arms[calibration.arm_id], sha256)

    def _check_calibration(
        self,
        arm: DamiaoArm,
        path: Path,
        calibration: ArmCalibration,
        joints: list[str],
        sha256: str,
    ) -> _ArmFile:
        """The file's gains and masses, once they are safe for the arm that ``joints`` move.

        Every gain must be for one of ``joints`` and every mass for a link
        they move; the masses must give finite torques, and none above a
        motor's rated torque at the measured pose or at home.
        """
        gains = {name: (item.kp, item.kd) for name, item in (calibration.gains or {}).items()}
        foreign = sorted(set(gains) - set(joints))
        if foreign:
            raise ArmCalibrationError(
                f"The arm calibration at {path} gives gains to joints of no arm "
                f"{calibration.arm_id!r} drives: {foreign}."
            )
        links = to_inertial_overrides(calibration)
        moved = self._links_moved_by(joints)
        strangers = sorted(set(links) - moved)
        if strangers:
            raise ArmCalibrationError(
                f"The arm calibration at {path} gives masses to links arm "
                f"{calibration.arm_id!r} does not move: {strangers}; it moves {sorted(moved)}."
            )
        arm.check_calibration(gains, links)
        self._check_rated_torques(arm, path, links, joints)
        # TODO(calibration): calibration.friction_nm is read but not applied;
        # apply it through DamiaoArm.set_feedforward once friction compensation
        # (direction from velocity, a dead band near zero) is designed.
        return _ArmFile(path=path, calibration=calibration, gains=gains, links=links, sha256=sha256)

    def _links_moved_by(self, joints: Sequence[str]) -> set[str]:
        """The links ``joints`` move directly: each one's child link in the gravity model."""
        names = set(joints)
        return {
            joint.child_link
            for joint in self.robot.gravity_model.load().joints
            if joint.name in names
        }

    def _check_rated_torques(
        self, arm: DamiaoArm, path: Path, links: Mapping[str, LinkInertial], joints: list[str]
    ) -> None:
        """Refuse masses that need more than a motor's rated torque, measured or at home."""
        rated = arm.rated_torques
        poses: list[tuple[str, Sequence[float] | None]] = [("the measured pose", None)]
        if self.robot.home_joints is not None:
            poses.append(("home", self.robot.home_joints))
        for label, pose in poses:
            torques = arm.gravity(links, pose)
            for joint in joints:
                if abs(torques[joint]) > rated[joint]:
                    raise ArmCalibrationError(
                        f"The arm calibration at {path} needs {torques[joint]:+.2f} N m of "
                        f"gravity torque on {joint} at {label}, more than its motor's rated "
                        f"{rated[joint]:g} N m; its masses cannot be right."
                    )

    def _apply_calibrations(
        self, arm: DamiaoArm, files: dict[str, _ArmFile | None], *, always: bool
    ) -> dict[str, AppliedCalibration | None]:
        """Put the files' gains and masses in place, log one line per arm, and record them.

        Without ``always`` and without a file, the arm is left untouched.
        """
        found = {arm_id: item for arm_id, item in files.items() if item is not None}
        if found or always:
            arm.apply_calibration(*_merged(files))
        for arm_id, item in found.items():
            created = item.calibration.created
            notes = "" if self.robot.gravity else " (gravity compensation is off, masses unused)"
            if item.calibration.friction_nm:
                notes += " (its friction is not applied yet)"
            logger.info(
                f"Calibration for {arm_id} from {item.path}, created "
                f"{created:%Y-%m-%d %H:%M} UTC: gains for {sorted(item.gains) or 'no joints'}, "
                f"masses for {sorted(item.links) or 'no links'}{notes}"
            )
        with self._lock:
            self._applied = dict(files)
        return {arm_id: None if item is None else item.summary() for arm_id, item in files.items()}

    def _on_position(self, msg: JointState) -> None:
        if self._ignored_for_tuning():
            return
        try:
            values = joint_state_to_values(msg, (Interface.POSITION,))
        except ValueError as error:
            self._report(str(error), rejection=True)
            return
        self.command(values)

    def write(self, values: Values) -> None:
        if self._fault is not None:
            raise RuntimeError(self._fault)
        arm = self._arm
        if arm is None:
            raise RuntimeError("Damiao robot is inactive")
        writer = self._tuning_writer
        tuning = getattr(writer, "active", False)
        if tuning:
            with self._lock:
                ended = self._tuning_session is None
            if ended:
                writer.error = f"the tuning session ended ({self._tuning_ended})"
                return
        elif self._ignored_for_tuning():
            # Got past _on_position just as a tuning session began.
            return
        prefix = f"{self.config.source}/"
        try:
            arm.command(
                {
                    key.removeprefix(prefix).removesuffix("/position"): value
                    for key, value in values.items()
                }
            )
        except Exception as error:
            if tuning:
                writer.error = (
                    f"write failed: {type(error).__name__}: {error}; the arm holds and the "
                    "tuning session ended"
                )
            raise
        if tuning:
            writer.written = True

    def halt(self) -> None:
        self._end_tuning("the connection halted")
        if self._fault is not None or self._arm is None:
            return
        self._arm.hold()

    def _read_state(self) -> ArmReading:
        try:
            arm = self._arm
            if arm is None:
                raise RuntimeError("Damiao robot is disconnected")
            reading = arm.read()
            self._latest = (time.time(), reading)
            return reading
        except Exception as error:
            if self._fault is None:
                self._feedback_failed(error)
            raise

    def _feedback_failed(self, error: Exception) -> None:
        self._fault = f"feedback failed: {type(error).__name__}: {error}"
        self._report(self._fault)
        arm = self._arm
        if arm is not None:
            try:
                arm.disable()
            except Exception as disable_error:
                self._report(f"{self._fault}; disable failed: {disable_error}")
        # Disabled first, so ending it sends nothing to the failed buses.
        self._end_tuning(self._fault)

    def _read_loop(self) -> None:
        period = 1.0 / self.config.state_rate_hz
        while not self._stop_reading.is_set():
            started = time.monotonic()
            try:
                with self._hardware_lock:
                    if self._arm is None or not self._connected:
                        return
                    self._end_tuning("no tuning call within its lease", lapsed=True)
                    reading = self._read_state()
                if self._stop_reading.is_set():
                    return
                self.joint_state.publish(
                    JointState(
                        ts=time.time(),
                        frame_id=self._state_frame,
                        name=[f"{self.config.source}/{name}" for name in self.joint_names],
                        position=reading.position,
                        velocity=reading.velocity,
                        effort=reading.effort,
                    )
                )
            except Exception as error:
                with self._hardware_lock:
                    if self._fault is None:
                        self._feedback_failed(error)
                return
            self._stop_reading.wait(max(0.0, period - (time.monotonic() - started)))
