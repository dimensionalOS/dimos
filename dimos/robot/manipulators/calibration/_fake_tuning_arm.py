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

"""Test helpers: a simulated arm behind the arm config contract, its clock and its recording.

:class:`FakeTuningArm` is an ``ArmConfigHooks`` connection for one arm of a
spec, and its ``StateSource``: ``latest`` is its reading now, and with
``joint_state`` set it publishes that reading there as time advances. Each
joint follows its PD target with first-order dynamics, ``(b + kd) dq/dt =
kp (target - q) + g_comp(q) - g_true(q) + ff - friction``: the connection
compensates gravity with the model it runs (the URDF, or the URDF with the
applied calibration's masses), while the arm obeys ``truth``. Coulomb
friction holds a joint that has stopped until the drive exceeds its stiction.
Gravity is evaluated once every 10 ms, the period it also records joint
states at, like the connection's 100 Hz ``joint_state``. It keeps the config
session's rules: one session, started by ``config_set`` and named by the id
it returns, which ``config_set`` and ``config_command`` must then carry; a
lease only calls carrying that id renew, a 0.1 s deadman on targets, the
session ending on ``config_reset``, lapse, deadman or any raising call
(floating joints then held where they are, torque offsets dropped, gains
restored), and position commands dropped while a session lasts. Its :meth:`FakeTuningArm.stop`
waits for a calibration tool to park, as a connection's stop does, then
"disables" the arm and records where.

What it does not simulate: inertia (so no overshoot or ringing), velocity
limits, encoder noise and quantization, torque saturation, the other arm of
a robot, grippers, CAN timing, and the connection's own threads (everything
happens in the caller's thread as simulated time advances). With
``realtime``, simulated time follows the wall clock instead: every call first
advances the arm to ``time.time()``.

:class:`SimClock` is that simulated time: ``sleep`` advances the arm.
:class:`MemoryRecording` keeps the samples and the procedure's markers in
memory and reads them back as the sqlite recording would.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray
import pinocchio as pin

from dimos.control.connection.arm_config import (
    DEFAULT_CONFIG_LEASE_S,
    MAX_CONFIG_LEASE_S,
    AppliedFile,
    ArmConfigState,
    CommandResult,
    ConfigApplied,
    ConfigKeys,
    ConfigPatch,
    ConfigScope,
    JointGains,
    LinkGravity,
)
from dimos.control.connection.connection_module import ConnectionStatus
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.calibration._inertial import apply_inertial_overrides
from dimos.robot.manipulators.calibration.arm_session import MarkerStream
from dimos.robot.manipulators.calibration.gravity_fit import GravityRecording, PoseMarker
from dimos.robot.manipulators.calibration.gravity_poses import ArmModel
from dimos.robot.manipulators.calibration.procedure import Procedure, ProcedureStatus
from dimos.robot.manipulators.calibration.recording import RecordingUnavailableError
from dimos.robot.manipulators.calibration.schema import (
    ArmCalibration,
    calibration_path,
    load_for_spec,
    merge_patch,
    save_calibration,
    to_inertial_overrides,
)
from dimos.robot.manipulators.calibration.spec import ArmCalibrationSpec, urdf_limits
from dimos.robot.manipulators.calibration.step_analysis import StepEvent
from dimos.robot.manipulators.calibration.step_report import JointTrace, Recording

# The robot name the fake's joint states carry, "arm/left_joint2".
SOURCE = "arm"
DEADMAN_S = 0.1
SAMPLE_PERIOD_S = 0.01
MAX_GRAVITY_STEP_NM = 2.0
MAX_KP = 500.0
MAX_KD = 5.0
VISCOUS_NM_S = 0.05
DEFAULT_FRICTION_NM = 0.05
STICTION_RATIO = 1.2
# Below this speed a joint counts as stopped, and friction may hold it.
STOPPED_RAD_S = 1e-3
# How long a simulated wait lets other threads answer before time moves on.
REAL_WAIT_S = 0.002
START_TIME = 1_800_000_000.0
# What the fake's stop waits for a config session, as a connection's does.
PARK_WAIT_S = 10.0
PARK_GRACE_S = 1.0

Vector = NDArray[np.float64]


@dataclass
class _Session:
    session_id: str
    lease_s: float
    last_call: float


class MemoryRecording:
    """Joint samples and markers in memory: a ``RecordingSource`` and an ``EventSink``.

    Samples are recorded under ``f"{source}/{joint}"``, as a connection
    publishes them. Every status change is kept and counted, so a driver can
    wait for the next one.
    """

    def __init__(self, source: str = SOURCE, directory: Path | None = None) -> None:
        self._source = source
        self._directory = directory
        self._lock = threading.Condition()
        self._times: list[float] = []
        self._names: list[str] = []
        self._positions: list[Vector] = []
        self._velocities: list[Vector] = []
        self.markers: dict[str, list[dict[str, Any]]] = {}
        self.statuses: list[ProcedureStatus] = []
        self.probes: list[str] = []
        # A recorder that stopped writing: probes never land.
        self.drop_probes = False

    @property
    def directory(self) -> Path | None:
        return self._directory

    @property
    def version(self) -> int:
        with self._lock:
            return len(self.statuses)

    def add_sample(self, t: float, names: list[str], q: Vector, dq: Vector) -> None:
        with self._lock:
            self._names = names
            self._times.append(t)
            self._positions.append(q.copy())
            self._velocities.append(dq.copy())

    def marker(self, stream: MarkerStream, payload: dict[str, Any]) -> None:
        # Through JSON, as the markers reach a recording over the wire.
        with self._lock:
            self.markers.setdefault(stream, []).append(json.loads(json.dumps(payload)))

    def status(self, status: ProcedureStatus) -> None:
        with self._lock:
            self.statuses.append(json.loads(json.dumps(status)))
            self._lock.notify_all()

    def probe(self, token: str) -> None:
        with self._lock:
            if not self.drop_probes:
                self.probes.append(token)

    def wait_for_probe(self, token: str, since: float, timeout_s: float = 2.0) -> None:
        with self._lock:
            if token not in self.probes:
                raise RecordingUnavailableError(f"the probe {token} was not recorded")

    def wait_change(self, seen: int, timeout_s: float) -> bool:
        """Wait until more than ``seen`` statuses arrived; whether they did."""
        with self._lock:
            return self._lock.wait_for(lambda: len(self.statuses) > seen, timeout_s)

    def wait_for(self, until: float, timeout_s: float = 0.0) -> None:
        # Samples land as simulated time passes, so all up to the last are here.
        with self._lock:
            if not self._times or self._times[-1] < until - SAMPLE_PERIOD_S:
                raise RecordingUnavailableError(f"nothing was recorded up to {until}")

    def steps(self, start: float, end: float) -> Recording:
        with self._lock:
            t = np.asarray(self._times)
            mask = (t >= start) & (t <= end)
            q = np.degrees(np.vstack(self._positions)[mask])
            dq = np.degrees(np.vstack(self._velocities)[mask])
            events = [
                StepEvent.from_json(json.dumps(m))
                for m in self.markers.get("step_events", [])
                if start <= m["t0"] <= end
            ]
            names = list(self._names)
        times = t[mask]
        if not times.size:
            raise RecordingUnavailableError(f"nothing was recorded between {start} and {end}")
        traces = {
            f"{self._source}/{name}": JointTrace(t=times, q_deg=q[:, i], dq_deg_s=dq[:, i])
            for i, name in enumerate(names)
        }
        return Recording(traces=traces, events=events, t_end=float(times[-1]))

    def gravity(self, start: float, end: float) -> GravityRecording:
        with self._lock:
            payloads = [
                m for m in self.markers.get("gravity_events", []) if start <= m["t0"] <= end
            ]
        markers = sorted(
            (PoseMarker.from_json(json.dumps(m)) for m in payloads), key=lambda m: m.t0
        )
        return GravityRecording(recording=self.steps(start, end), markers=markers)


class FakeTuningArm:
    """One simulated arm behind the arm config contract; see the module doc for what it simulates.

    Args:
        spec: The arm.
        arm: ``ArmModel(spec)``, when already built.
        truth: The arm's real masses; the spec's model when ``None``.
        friction_nm: Coulomb friction, one value or by joint.
        gravity_compensation: Whether the connection compensates gravity.
        recording: Where joint states are recorded.
        calibration_root: Where it saves the arm's calibration file, as a
            connection's ``persist`` does: ``calibration_path(arm_id, root)``.
        applied: A calibration file applied at start, as a connection's "auto" would.
        sessions: What ``config_keys`` says of config sessions; the fake serves
            them either way.
        persist: What ``config_keys`` says can be saved; a save of anything
            else is refused as unsupported.
        start: Initial joint positions; home when ``None``.
        realtime: Simulated time follows the wall clock: each call first
            advances the arm to ``time.time()``.
    """

    def __init__(
        self,
        spec: ArmCalibrationSpec,
        *,
        arm: ArmModel | None = None,
        truth: pin.Model | None = None,
        friction_nm: float | Mapping[str, float] = DEFAULT_FRICTION_NM,
        gravity_compensation: bool = True,
        recording: MemoryRecording | None = None,
        calibration_root: Path,
        applied: Path | None = None,
        sessions: bool = True,
        persist: Sequence[str] = ("gains", "links", "fit", "friction_nm", "checks", "notes"),
        start: Vector | None = None,
        realtime: bool = False,
    ) -> None:
        self.spec = spec
        self.arm = arm if arm is not None else ArmModel(spec)
        self._names = list(spec.joint_names)
        self._lock = threading.RLock()
        self._truth = truth if truth is not None else self.arm.model.copy()
        self._truth_data = self._truth.createData()
        self._comp = self.arm.model.copy()
        self._comp_data = self._comp.createData()
        self._gravity_comp = gravity_compensation
        self._recording = recording
        self._root = calibration_root
        self._serves_sessions = sessions
        self._persist = list(persist)
        if isinstance(friction_nm, Mapping):
            self._friction = np.asarray([friction_nm[name] for name in self._names])
        else:
            self._friction = np.full(len(self._names), float(friction_nm))
        self._limits = {
            joint.name: urdf_limits(joint)
            for joint in spec.model.load().joints
            if joint.name in self._names
        }
        self.q: Vector = np.array(self.arm.home if start is None else start, dtype=np.float64)
        self.v: Vector = np.zeros(len(self._names))
        self._stopped = np.ones(len(self._names), dtype=bool)
        self.target: Vector = self.q.copy()
        self._defaults = {name: spec.gains[name] for name in self._names}
        self._calibrated: dict[str, tuple[float, float]] = {}
        self._overrides: dict[str, tuple[float, float]] = {}
        self._floating: dict[str, float] = {}
        self._feedforward: dict[str, float] = {}
        # The calibration file it runs, and the link masses that file gave.
        self._file: AppliedFile | None = None
        self._links: dict[str, LinkGravity] = {}
        # Where it publishes its reading as time advances, e.g. a module's joint_state.
        self.joint_state: Any = None
        self._realtime = realtime
        self.now = time.time() if realtime else START_TIME
        self._next_sample = self.now
        self._session: _Session | None = None
        self._sessions = 0
        self.ended: list[str] = []
        # What stop() saw: when it began, waited and disabled, and the arm then.
        self.stop_log: list[str] = []
        self.disabled_at: Vector | None = None
        # When the last session ended, time.monotonic().
        self._ended_at: float | None = None
        self._last_command: float | None = None
        self.deadman_fired = False
        self.commands = 0
        self.dropped_position_commands = 0
        # Fault injection: config_command raises once this many were taken; joints
        # in ``blocked`` cannot move.
        self.fail_after_commands: int | None = None
        self.blocked: set[str] = set()
        self.on_command: Callable[[FakeTuningArm], None] | None = None
        if applied is not None:
            self._apply(applied)

    # Simulated time.

    def advance(self, seconds: float) -> None:
        with self._lock:
            end = self.now + seconds
            while self.now < end - 1e-12:
                self._timeouts()
                h = min(end, self._next_sample) - self.now
                if h > 1e-12:
                    self._integrate(h)
                    self.now += h
                if self.now >= self._next_sample - 1e-12:
                    if self._recording is not None:
                        self._recording.add_sample(self.now, self._names, self.q, self.v)
                    self._next_sample += SAMPLE_PERIOD_S
            if self.joint_state is not None:
                self.joint_state.publish(self._reading())

    def _timeouts(self) -> None:
        if self._last_command is not None and self.now - self._last_command > DEADMAN_S:
            self._last_command = None
            self.deadman_fired = True
            self._end("the command deadman fired")
            self.target = self.q.copy()
        session = self._session
        if session is not None and self.now - session.last_call > session.lease_s:
            self._end(f"no config call within its lease of {session.lease_s:g} s")

    def _gravity_error(self) -> Vector:
        true = np.asarray(pin.computeGeneralizedGravity(self._truth, self._truth_data, self.q))
        if not self._gravity_comp:
            return -true
        comp = np.asarray(pin.computeGeneralizedGravity(self._comp, self._comp_data, self.q))
        error: Vector = comp - true
        return error

    def _integrate(self, h: float) -> None:
        error = self._gravity_error()
        for i, name in enumerate(self._names):
            if name in self.blocked:
                self.v[i] = 0.0
                continue
            floating = name in self._floating
            kp, kd = (0.0, self._floating[name]) if floating else self._gains(name)
            forcing = float(error[i]) + self._feedforward.get(name, 0.0)
            drive = kp * (float(self.target[i]) - float(self.q[i])) + forcing
            friction = float(self._friction[i])
            # Friction never pushes: a drive it can hold stops the joint.
            hold = STICTION_RATIO * friction if self._stopped[i] else friction
            if abs(drive) <= hold:
                self.v[i] = 0.0
                self._stopped[i] = True
                continue
            push = forcing - math.copysign(friction, drive)
            damping = VISCOUS_NM_S + kd
            if kp > 0.0:
                tau = damping / kp
                rest = float(self.target[i]) + push / kp
                q = rest + (float(self.q[i]) - rest) * math.exp(-h / tau)
                v = (rest - q) / tau
            else:
                v = push / damping
                q = float(self.q[i]) + v * h
            self.q[i], self.v[i] = q, v
            self._stopped[i] = abs(v) < STOPPED_RAD_S

    def _gains(self, name: str) -> tuple[float, float]:
        return self._overrides.get(name) or self._calibrated.get(name) or self._defaults[name]

    # The session rules.

    def _undo(self) -> None:
        """Hold floating joints where they are, drop torque offsets and gain overrides."""
        for name in self._floating:
            self.target[self._names.index(name)] = self.q[self._names.index(name)]
        self._floating.clear()
        self._feedforward.clear()
        self._overrides.clear()

    def _end(self, reason: str) -> None:
        if self._session is None:
            return
        self._session = None
        self._ended_at = time.monotonic()
        self.ended.append(reason)
        self._undo()

    def _call(self, name: str, call: Callable[[], Any]) -> Any:
        with self._lock:
            if self._realtime:
                self.advance(max(0.0, time.time() - self.now))
            if self.disabled_at is not None:
                raise RuntimeError("the connection stopped")
            try:
                return call()
            except Exception as error:
                self._end(f"{name} raised {type(error).__name__}: {error}")
                raise

    def _check(self, session_id: str | None) -> _Session | None:
        """The live session, renewed, once ``session_id`` names it; ``None`` without either.

        Only a call that carries the live session's id renews its lease.
        """
        session = self._session
        if session is not None and session_id == session.session_id:
            session.last_call = self.now
            return session
        if session is not None:
            if session_id is None:
                raise RuntimeError(
                    "a config session is live; pass the session_id the config_set that "
                    "started it returned"
                )
            raise RuntimeError(f"{session_id!r} is not the live config session")
        if session_id is None:
            return None
        raise RuntimeError(
            f"the config session ended ({self.ended[-1]})" if self.ended else "no config session"
        )

    def _reading(self) -> JointState:
        return JointState(
            ts=self.now,
            name=[f"{SOURCE}/{name}" for name in self._names],
            position=[float(v) for v in self.q],
            velocity=[float(v) for v in self.v],
            effort=[0.0] * len(self._names),
        )

    def _arm_state(self) -> ArmConfigState:
        return ArmConfigState(
            arm_id=self.spec.arm_id,
            gains={
                name: JointGains(kp=self._gains(name)[0], kd=self._gains(name)[1])
                for name in self._names
            },
            floating=dict(self._floating),
            torque_offset_nm={name: tau for name, tau in self._feedforward.items() if tau},
            links=dict(self._links),
            file=self._file,
            session_id=None if self._session is None else self._session.session_id,
        )

    def _joint(self, joint: str) -> int:
        if joint not in self._names:
            raise ValueError(f"{joint!r} is not an arm joint")
        return self._names.index(joint)

    def session_active(self) -> bool:
        with self._lock:
            return self._session is not None

    @property
    def session_id(self) -> str | None:
        with self._lock:
            return None if self._session is None else self._session.session_id

    def position_command(self, targets: Mapping[str, float]) -> bool:
        """A normal position command: dropped while a config session lasts."""
        with self._lock:
            if self._session is not None:
                self.dropped_position_commands += 1
                return False
            for joint, value in targets.items():
                self.target[self._joint(joint)] = value
            self._last_command = self.now
            return True

    # ArmConfigHooks and StateSource.

    def status(self) -> ConnectionStatus:
        with self._lock:
            return ConnectionStatus(
                session_id="fake",
                connected=True,
                last_command_time=self._last_command,
                deadman_fired=self.deadman_fired,
                last_rejection=None,
                last_error=None,
                config_session_id=None if self._session is None else self._session.session_id,
            )

    def latest(self) -> JointState:
        """The arm's reading now; not a connection call, so it renews nothing."""
        with self._lock:
            if self._realtime:
                self.advance(max(0.0, time.time() - self.now))
            return self._reading()

    def config_keys(self) -> ConfigKeys:
        def keys() -> ConfigKeys:
            arm_id = self.spec.arm_id
            return ConfigKeys(
                source=SOURCE,
                sessions=self._serves_sessions,
                arms={arm_id: list(self._names)},
                readable=["gains", "links", "torque_offset_nm"],
                session=["gains", *(["float"] if self._gravity_comp else []), "torque_offset_nm"],
                persist=list(self._persist),
                persist_paths={arm_id: str(calibration_path(arm_id, self._root))},
                gravity_compensation=self._gravity_comp,
            )

        result: ConfigKeys = self._call("config_keys", keys)
        return result

    def config_get(self) -> dict[str, ArmConfigState]:
        result: dict[str, ArmConfigState] = self._call(
            "config_get", lambda: {self.spec.arm_id: self._arm_state()}
        )
        return result

    def config_set(
        self,
        patch: ConfigPatch,
        scope: ConfigScope = "session",
        force: bool = False,
        apply_now: bool = True,
        session_id: str | None = None,
        lease_s: float = DEFAULT_CONFIG_LEASE_S,
    ) -> ConfigApplied:
        def apply() -> ConfigApplied:
            if not 0.0 < lease_s <= MAX_CONFIG_LEASE_S:
                raise ValueError(f"lease_s must be in (0, {MAX_CONFIG_LEASE_S:g}] s, got {lease_s}")
            if patch.arm_id != self.spec.arm_id:
                raise ValueError(f"{patch.arm_id!r} is not this fake's arm")
            session = self._check(session_id)
            if session is None and scope == "session":
                self._sessions += 1
                session = _Session(f"fake-{self._sessions}", float(lease_s), self.now)
                self._session = session
            if scope == "persist":
                applied = self._save(patch, force, apply_now)
            else:
                applied = self._apply_live(patch)
            if session is None:
                return applied
            return applied.model_copy(
                update={"session_id": session.session_id, "lease_s": session.lease_s}
            )

        result: ConfigApplied = self._call("config_set", apply)
        return result

    def _apply_live(self, patch: ConfigPatch) -> ConfigApplied:
        """A session's change: gains (kp 0 floats the joint) and torque offsets, clamped."""
        file_only = patch.model_copy(update={"gains": None, "torque_offset_nm": None})
        why = dict.fromkeys(file_only.keys(), "saved with scope 'persist' only")
        gains: dict[str, JointGains] = {}
        clamped: list[str] = []
        for joint, asked in (patch.gains or {}).items():
            index = self._joint(joint)
            keys = [f"gains.{joint}.kp", f"gains.{joint}.kd"]
            if asked.kp == 0.0:
                if not self._gravity_comp:
                    why.update(dict.fromkeys(keys, "the robot does not compensate gravity"))
                    continue
                self._floating[joint] = min(asked.kd, MAX_KD)
                gains[joint] = JointGains(kp=0.0, kd=self._floating[joint])
            else:
                if joint in self._floating:
                    # Held again where it is, then with these gains.
                    del self._floating[joint]
                    self.target[index] = self.q[index]
                self._overrides[joint] = (min(asked.kp, MAX_KP), min(asked.kd, MAX_KD))
                gains[joint] = JointGains(
                    kp=self._overrides[joint][0], kd=self._overrides[joint][1]
                )
            clamped += [
                key
                for key, want, got in zip(
                    keys, (asked.kp, asked.kd), (gains[joint].kp, gains[joint].kd), strict=True
                )
                if want != got
            ]
        offsets: dict[str, float] = {}
        for joint, tau in (patch.torque_offset_nm or {}).items():
            self._joint(joint)
            if tau:
                self._feedforward[joint] = float(tau)
            else:
                self._feedforward.pop(joint, None)
            offsets[joint] = float(tau)
        return ConfigApplied(
            arm_id=patch.arm_id,
            scope="session",
            applied=ConfigPatch(
                arm_id=patch.arm_id, gains=gains or None, torque_offset_nm=offsets or None
            ),
            clamped=clamped,
            unsupported=list(why),
            why=why,
        )

    def config_reset(self, keep_session: bool = False) -> None:
        if not keep_session:
            with self._lock:
                self._end("config_reset")
            return
        self._call("config_reset", self._undo)

    def config_command(self, targets: dict[str, float], session_id: str) -> CommandResult:
        def command() -> CommandResult:
            if not session_id:
                raise RuntimeError("config_command needs the session_id config_set returned")
            if self._check(session_id) is None:
                raise RuntimeError("no config session is live")
            for joint, value in targets.items():
                low, high = self._limits[self._names[self._joint(joint)]]
                if not low <= value <= high:
                    raise ValueError(f"{joint} target {value} is outside {low}..{high}")
            if self.fail_after_commands is not None and self.commands >= self.fail_after_commands:
                raise RuntimeError("feedback failed: the bus went silent")
            for joint, value in targets.items():
                if joint not in self._floating:
                    self.target[self._names.index(joint)] = value
            self.commands += 1
            self._last_command = self.now
            if self.on_command is not None:
                self.on_command(self)
            return CommandResult(ok=True)

        result: CommandResult = self._call("config_command", command)
        return result

    def _save(self, patch: ConfigPatch, force: bool, apply_now: bool) -> ConfigApplied:
        """A save, as a connection's: merged into the arm's file, guarded, written, applied.

        Torque offsets and kp 0 are live changes only and are left out. When
        applying it would step a joint's gravity torque at this pose by more
        than ``MAX_GRAVITY_STEP_NM`` and not ``force``, nothing is written. A
        save without gains or links changes nothing the arm runs, so it is not
        applied; neither is one with ``apply_now`` False. One with nothing a
        file holds writes nothing.
        """
        arm_id = self.spec.arm_id
        why = dict.fromkeys(
            [key for key in patch.keys() if key.startswith("torque_offset_nm")],
            "a torque offset is a live change only",
        )
        floats = [joint for joint, gains in (patch.gains or {}).items() if gains.kp == 0.0]
        why.update(
            dict.fromkeys(
                [f"gains.{joint}.{gain}" for joint in floats for gain in ("kp", "kd")],
                "kp 0 floats a joint, which only a session does",
            )
        )
        kept = {joint: gains for joint, gains in (patch.gains or {}).items() if gains.kp != 0.0}
        saved = patch.model_copy(update={"gains": kept or None, "torque_offset_nm": None})
        path = calibration_path(arm_id, self._root)
        on_disk = hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None
        running = None if self._file is None else self._file.sha256
        if patch.checks is not None and running != on_disk:
            # As the connection: a check belongs only in the file the arm runs.
            why.update(
                dict.fromkeys(
                    [key for key in patch.keys() if key.split(".")[0] == "checks"],
                    f"the arm does not run {path} as it is on disk",
                )
            )
            saved = saved.model_copy(update={"checks": None})
        refused = [key for key in saved.keys() if key.split(".")[0] not in self._persist]
        if refused or not saved.keys():
            # Nothing it may save, or nothing a file holds: nothing is written.
            why.update(dict.fromkeys(refused, "this fake saves none of it"))
            return ConfigApplied(
                arm_id=arm_id,
                scope="persist",
                applied=ConfigPatch(arm_id=arm_id),
                unsupported=list(why),
                why=why,
                applied_now=False,
            )
        now = datetime.now(timezone.utc)
        stored = (
            load_for_spec(self.spec, path)
            if path.exists()
            else ArmCalibration.for_spec(self.spec, created=now)
        )
        merged = merge_patch(stored, saved, created=now)
        live = apply_now and (saved.gains is not None or saved.links is not None)
        step: dict[str, float] = {}
        if live:
            comp = self._model(merged)
            before = pin.computeGeneralizedGravity(self._comp, self._comp_data, self.q)
            after = pin.computeGeneralizedGravity(comp, comp.createData(), self.q)
            step = {name: float(after[i] - before[i]) for i, name in enumerate(self._names)}
            large = {
                joint: round(value, 2)
                for joint, value in step.items()
                if abs(value) > MAX_GRAVITY_STEP_NM
            }
            if large and not force:
                return ConfigApplied(
                    arm_id=arm_id,
                    scope="persist",
                    applied=ConfigPatch(arm_id=arm_id),
                    unsupported=list(why),
                    why=why,
                    applied_now=False,
                    refused=(
                        f"applying it would change the gravity torque at this pose by more "
                        f"than {MAX_GRAVITY_STEP_NM:g} N m: {large} (N m), so nothing was "
                        "written; save calibrations near home, or pass force=True"
                    ),
                    gravity_step_nm=step,
                )
        target = save_calibration(path, merged)
        running = self._file is not None and Path(self._file.path) == target
        if live:
            # As the connection's apply: live gain overrides go.
            self._overrides.clear()
            self._apply(target)
        elif running and self._links == (merged.gravity.links if merged.gravity else {}):
            same_gains = {name: (g.kp, g.kd) for name, g in (merged.gains or {}).items()}
            if same_gains == self._calibrated:
                # The arm runs what the file now holds; only its record is new.
                self._apply(target)
        return ConfigApplied(
            arm_id=arm_id,
            scope="persist",
            applied=saved,
            unsupported=list(why),
            why=why,
            path=str(target),
            applied_now=live,
            gravity_step_nm=step,
        )

    def _model(self, calibration: ArmCalibration | None) -> pin.Model:
        model = self.arm.model.copy()
        if calibration is not None:
            apply_inertial_overrides(model, to_inertial_overrides(calibration))
        return model

    def _apply(self, file: Path | None) -> None:
        calibration = None if file is None else load_for_spec(self.spec, file)
        self._comp = self._model(calibration)
        self._comp_data = self._comp.createData()
        if file is None or calibration is None:
            self._calibrated = {}
            self._file = None
            self._links = {}
            return
        self._calibrated = {
            name: (gains.kp, gains.kd) for name, gains in (calibration.gains or {}).items()
        }
        self._file = AppliedFile(
            path=str(file),
            created=calibration.created.isoformat(),
            sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
        )
        self._links = {} if calibration.gravity is None else dict(calibration.gravity.links)

    def stop(self) -> None:
        """Stop as a connection does: wait for a calibration tool to park, then disable.

        Waits while a config session lasts, and ``PARK_GRACE_S`` after the stop
        began or the last session ended for one about to begin, at most
        ``PARK_WAIT_S``; then records where the arm was "disabled".
        """
        started = time.monotonic()
        self.stop_log.append("stop")
        while time.monotonic() - started < PARK_WAIT_S:
            with self._lock:
                active = self._session is not None
                ended = self._ended_at
            since = max(started, ended if ended is not None else started)
            if not active and time.monotonic() - since >= PARK_GRACE_S:
                break
            if active and "waiting" not in self.stop_log:
                self.stop_log.append("waiting")
            time.sleep(0.01)
        with self._lock:
            if self._realtime:
                self.advance(max(0.0, time.time() - self.now))
            self._end("the connection stopped")
            self.disabled_at = self.q.copy()
        self.stop_log.append("disabled")


class SimClock:
    """Simulated Unix time on a fake arm: ``sleep`` advances it."""

    def __init__(self, arm: FakeTuningArm) -> None:
        self._arm = arm

    def now(self) -> float:
        return self._arm.now

    def sleep(self, seconds: float) -> None:
        if seconds > 0.0:
            self._arm.advance(seconds)

    def wait(self, event: threading.Event, seconds: float) -> bool:
        if event.wait(REAL_WAIT_S):
            return True
        self.sleep(seconds)
        return event.is_set()


def drive(
    procedure: Procedure,
    sink: MemoryRecording,
    answer: Callable[[str], str] | None = None,
    *,
    timeout_s: float = 120.0,
) -> ProcedureStatus:
    """Answer every prompt until the run finishes; returns its final status.

    ``answer`` maps a prompt to "confirm", "skip", "abort" or "ignore" (left
    unanswered); by default every prompt is confirmed.
    """
    answered: set[int] = set()
    while True:
        seen = sink.version
        status = procedure.status()
        if status["state"] in ("done", "failed", "aborted", "idle"):
            return status
        prompt, prompt_id = status["prompt"], status["prompt_id"]
        if prompt is not None and prompt_id is not None and prompt_id not in answered:
            answered.add(prompt_id)
            decision = "confirm" if answer is None else answer(prompt)
            if decision == "abort":
                procedure.abort()
            elif decision == "skip":
                procedure.skip(prompt_id)
            elif decision != "ignore":
                procedure.confirm(prompt_id)
        if not sink.wait_change(seen, timeout_s):
            raise TimeoutError(f"the procedure stopped changing: {procedure.status()}")
