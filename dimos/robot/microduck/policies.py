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

"""Microduck policy catalogue, bank and scheduler.

Three layers, each usable without the one above it:

* ``PolicyName`` / ``POLICY_SPECS`` - the nine published Microduck ONNX
  policies: kind (base / posture / oneshot), file name, the robot variant(s)
  they physically work on, and their timing constants.
* ``PolicyBank`` - one ONNX session per loadable policy, all sharing a single
  ``MicroduckObserver`` and ``last_action`` slot (valid across switches: every
  published net declares the same joint order and home pose), stepped at the
  policies' 50 Hz.
* ``PolicyScheduler`` - the pure-python state machine that decides which
  policy runs with which 13-float command ``[vx, vy, wz, head(4), body(6)]``:
  base selection (with roller braking), sit/stand posture, timed one-shots
  (kicks, roulade, ground pick), fall handling and the ``policy_state`` JSON
  snapshot for the cockpit. Requests arrive from any thread, are resolved
  against the state they were made in and parked in a single pending slot;
  ``tick()`` runs on the sim thread only.

Command layouts, windows and the variant matrix follow what was measured in
MuJoCo with the published policies (kicks: zero command for 0.5 s with the
ball at a yaw-frame offset; roulade: 2.2 s window with the fall detector
suspended; ground pick: phase-encoded command until phase 0.7; roller: zero
throttle for 2 s before handing over to a legged policy) and upstream's
``scripts/infer_policy.py``.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.typing import NDArray

from dimos.robot.microduck.gait import (
    COMMAND_LEN,
    VX_RANGE,
    VY_RANGE,
    WZ_RANGE,
    MicroduckObserver,
    PolicySession,
    load_policy_session,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

try:
    from enum import StrEnum
except ImportError:  # pragma: no cover - Python 3.10 floor
    from enum import Enum

    class StrEnum(str, Enum):  # type: ignore[no-redef]
        """Minimal ``enum.StrEnum`` stand-in for Python 3.10."""

        def __str__(self) -> str:
            return str(self.value)


if TYPE_CHECKING:
    import mujoco


class PolicyName(StrEnum):
    """The published Microduck policies, in cockpit display order."""

    WALK = "walk"
    STAND = "stand"
    ROLLER = "roller"
    ROLLER_CROUCH = "roller_crouch"
    SITSTAND = "sitstand"
    KICK_LEFT = "kick_left"
    KICK_RIGHT = "kick_right"
    ROULADE = "roulade"
    GROUND_PICK = "ground_pick"


class PolicyKind(StrEnum):
    BASE = "base"  # runs until another base is selected; walk/roller follow the twist
    POSTURE = "posture"  # sitstand: seated until asked to stand back up
    ONESHOT = "oneshot"  # timed trick, then back to the base


# Robot MJCF variants. Roller policies need the wheeled model; sitstand falls
# over on it (the duck rolls away while seated) and so does roulade (the roll
# completes but the duck topples within a second of the walk hand-over, at
# any window length from 2.2 to 4 s - measured in the headless matrix).
DEFAULT_VARIANT = "default"
ROLLERS_VARIANT = "rollers"
VARIANTS: tuple[str, ...] = (DEFAULT_VARIANT, ROLLERS_VARIANT)

_ALL_VARIANTS = frozenset(VARIANTS)
_ROLLERS_ONLY = frozenset({ROLLERS_VARIANT})
_DEFAULT_ONLY = frozenset({DEFAULT_VARIANT})

# Timing (sim seconds; advanced by tick(dt), never by wall clock).
KICK_DURATION_S = 0.5
ROULADE_DURATION_S = 2.2  # 2.0-2.5 s valid; shorter leaves the duck on its back
ROULADE_GRACE_S = 1.0  # fall detector stays suspended this long after the roll
GROUND_PICK_PERIOD_S = 4.0  # phase += dt / period, command = [cos 2pi*phi, sin 2pi*phi]
GROUND_PICK_END_PHASE = 0.7  # upstream runtime cut-off (~2.8 s)
ROLLER_CROUCH_PERIOD_S = 3.0  # same phase encoding, looping while the base is selected
STAND_UP_DURATION_S = 2.0  # sitstand with cmd[0]=0 before handing back to the base
# Leaving the wheels for a legged base: keep the roller-family base that is
# being left for this long first - ``roller`` at ROLLER_BRAKE_THROTTLE,
# ``roller_crouch`` on its phase loop - so the duck is not walking at speed.
# This is open-loop and the legged policies cannot hold free wheels, so a
# residual glide survives the hand-over. Measured headless on the rollers
# model (brake window + the 3 s after walk takes over): from a 0.8 m/s push,
# zero throttle for 2 s hands over at <=0.3 m/s and the duck still travels
# 0.7-1.5 m forward; a roller that merely crept from rest (0.2 m/s) travels
# ~0.65 m; leaving roller_crouch ~0.8 m. It never reverses and never falls.
# The design minimum of 1 s hands over at up to 0.57 m/s and lets 1.1-2.1 m
# through. A negative throttle (-0.3) sheds speed faster but is reverse
# thrust when there is no speed to shed: 0.7-0.95 m *backwards* from rest,
# which is why it is not used.
BRAKE_DURATION_S = 2.0
ROLLER_BRAKE_THROTTLE = 0.0

# Roller command ranges (microduck_rl velocity_rollers task): throttle
# 0 = coast, >0 = push, <0 = brake; slot 2 is a heading error in rad.
ROLLER_THROTTLE_RANGE = (-0.5, 0.6)
ROLLER_HEADING_RANGE = (-1.0, 1.0)

# Ball spawn offsets in the trunk yaw frame (x forward, y left), matching the
# kick tasks' reset_ball_in_front_of_foot.
KICK_BALL_OFFSETS: dict[str, tuple[float, float]] = {
    str(PolicyName.KICK_LEFT): (0.09, 0.042),
    str(PolicyName.KICK_RIGHT): (0.09, -0.042),
}

# Pseudo-states reported in ``active`` while the scheduler is between policies.
ACTIVE_BRAKING = "braking"
ACTIVE_STANDING_UP = "standing_up"

# Fall detector threshold on the trunk's projected gravity z (-1 = upright,
# +1 = upside down): past this the duck is lying down. The sim module owns
# the detector (with its own debounce); the scheduler only consumes its
# verdict through ``notify_fall``.
FALL_GRAVITY_Z = -0.55

# A rejected request stays visible in snapshot()["last_error"] this long.
LAST_ERROR_TTL_S = 5.0

ACTIONS: tuple[str, ...] = ("start", "stop", "toggle")


@dataclass(frozen=True)
class PolicySpec:
    name: PolicyName
    kind: PolicyKind
    onnx: str
    variants: frozenset[str]
    duration: float | None = None  # oneshots: sim seconds before handing back


POLICY_SPECS: dict[PolicyName, PolicySpec] = {
    spec.name: spec
    for spec in (
        PolicySpec(PolicyName.WALK, PolicyKind.BASE, "alpha_walking.onnx", _ALL_VARIANTS),
        PolicySpec(PolicyName.STAND, PolicyKind.BASE, "alpha_stand.onnx", _ALL_VARIANTS),
        PolicySpec(PolicyName.ROLLER, PolicyKind.BASE, "roller.onnx", _ROLLERS_ONLY),
        PolicySpec(PolicyName.ROLLER_CROUCH, PolicyKind.BASE, "roller_crouch.onnx", _ROLLERS_ONLY),
        PolicySpec(PolicyName.SITSTAND, PolicyKind.POSTURE, "alpha_sitstand.onnx", _DEFAULT_ONLY),
        PolicySpec(
            PolicyName.KICK_LEFT,
            PolicyKind.ONESHOT,
            "ball_kick_left.onnx",
            _ALL_VARIANTS,
            KICK_DURATION_S,
        ),
        PolicySpec(
            PolicyName.KICK_RIGHT,
            PolicyKind.ONESHOT,
            "ball_kick_right.onnx",
            _ALL_VARIANTS,
            KICK_DURATION_S,
        ),
        PolicySpec(
            PolicyName.ROULADE,
            PolicyKind.ONESHOT,
            "roulade.onnx",
            _DEFAULT_ONLY,
            ROULADE_DURATION_S,
        ),
        PolicySpec(
            PolicyName.GROUND_PICK,
            PolicyKind.ONESHOT,
            "alpha_ground_pick.onnx",
            _ALL_VARIANTS,
            GROUND_PICK_PERIOD_S * GROUND_PICK_END_PHASE,
        ),
    )
}

POLICY_NAMES: tuple[str, ...] = tuple(str(name) for name in POLICY_SPECS)
# Plain-str spellings for the names the scheduler stores and emits (keeps
# snapshot()/tick() free of enum members).
_WALK = str(PolicyName.WALK)
_ROLLER = str(PolicyName.ROLLER)
_ROLLER_CROUCH = str(PolicyName.ROLLER_CROUCH)
_SITSTAND = str(PolicyName.SITSTAND)
_ROULADE = str(PolicyName.ROULADE)
_GROUND_PICK = str(PolicyName.GROUND_PICK)
BASE_POLICIES: frozenset[str] = frozenset(
    str(n) for n, s in POLICY_SPECS.items() if s.kind is PolicyKind.BASE
)
ONESHOT_POLICIES: frozenset[str] = frozenset(
    str(n) for n, s in POLICY_SPECS.items() if s.kind is PolicyKind.ONESHOT
)
_ROLLER_FAMILY: frozenset[str] = frozenset({_ROLLER, _ROLLER_CROUCH})

ASSET_MISSING_REASON = "asset missing"

# What a request resolves to (the pending slot holds one of these, never the
# raw (policy, action) pair - a "toggle" that meant "abort" when it was made
# must not turn into "start" because the oneshot finished before the tick).
_OP_SELECT_BASE = "select_base"  # target: base name
_OP_SIT = "sit"
_OP_STAND_UP = "stand_up"
_OP_START_ONESHOT = "start_oneshot"  # target: oneshot name
_OP_ABORT_ONESHOT = "abort_oneshot"  # target: oneshot name
_OP_ABORT_ANY = "abort_any"  # policy-less stop


@dataclass(frozen=True)
class _Op:
    kind: str
    target: str = ""  # policy name for select_base / start_oneshot / abort_oneshot


# Resolution result meaning "empty the pending slot": the request undoes the
# operation still parked there (a stop right behind an unconsumed start, a
# start right behind an unconsumed abort). Never stored in the slot itself.
_CANCEL = _Op("cancel")


def policy_availability(variant: str, missing: Iterable[str] = ()) -> dict[str, str | None]:
    """name -> ``None`` (runnable) or the reason it is not, for every policy.

    ``missing`` lists policies whose ONNX file could not be obtained; they are
    reported as ``"asset missing"``.
    """
    if variant not in _ALL_VARIANTS:
        raise ValueError(f"unknown Microduck variant {variant!r}; expected one of {VARIANTS}")
    missing_names = {str(m) for m in missing}
    out: dict[str, str | None] = {}
    for name, spec in POLICY_SPECS.items():
        key = str(name)
        if key in missing_names:
            out[key] = ASSET_MISSING_REASON
        elif variant not in spec.variants:
            if spec.variants == _ROLLERS_ONLY:
                out[key] = "requires the rollers variant (start with MICRODUCK_VARIANT=rollers)"
            else:
                out[key] = f"not supported on the {variant} variant"
        else:
            out[key] = None
    return out


class PolicyBank:
    """Every runnable Microduck policy for one robot variant, sharing one observation.

    Loads the ONNX session of each policy that ``policy_availability`` allows
    on ``variant`` and whose file exists in ``policy_dir`` (eager; ~800 KB
    each). ``step()`` is the 50 Hz control step: the caller decides which
    policy runs (``PolicyScheduler.tick``), the bank builds the shared
    observation, runs that net and updates the shared ``last_action``.

    Positional order is (policy_dir, model), like ``MicroduckGaitPolicy``::

        assets = ensure_assets(variant)
        bank = PolicyBank(assets.policy_dir, model, variant=variant, missing=assets.missing)
        sched = PolicyScheduler(bank.availability, variant, spawn_ball=...)
        # sim thread, 50 Hz:
        name, command = sched.tick(dt)
        targets = bank.step(name, command, data)
    """

    def __init__(
        self,
        policy_dir: str | Path,
        model: mujoco.MjModel,
        *,
        variant: str = DEFAULT_VARIANT,
        missing: Iterable[str] = (),
    ) -> None:
        self.variant = variant
        self.policy_dir = Path(policy_dir)
        wanted = policy_availability(variant, missing)

        self._sessions: dict[str, PolicySession] = {}
        not_found: list[str] = []
        for name, reason in wanted.items():
            if reason is not None:
                continue
            path = self.policy_dir / POLICY_SPECS[PolicyName(name)].onnx
            if not path.exists():
                not_found.append(name)
                continue
            self._sessions[name] = load_policy_session(path)
        if not self._sessions:
            raise RuntimeError(f"no Microduck policy ONNX files found in {self.policy_dir}")

        reference = self._sessions.get(_WALK) or next(iter(self._sessions.values()))
        for name, session in self._sessions.items():
            if session.joint_names != reference.joint_names or not np.allclose(
                session.default_pose, reference.default_pose, atol=1e-6
            ):
                raise RuntimeError(
                    f"Microduck policy {name} declares a different joint order or home pose "
                    f"than {reference.path.name}; the policies cannot share one observation"
                )

        self._observer = MicroduckObserver(model, reference.joint_names, reference.default_pose)
        self.joint_names: list[str] = list(reference.joint_names)
        self.default_pose: NDArray[np.float32] = reference.default_pose
        self.root_qpos_adr = self._observer.root_qpos_adr
        self.last_action = np.zeros(len(self.joint_names), dtype=np.float32)

        seen = {str(m) for m in missing} | set(not_found)
        self.missing: tuple[PolicyName, ...] = tuple(
            PolicyName(n) for n in POLICY_NAMES if n in seen
        )
        self._availability = policy_availability(variant, self.missing)

    @property
    def names(self) -> tuple[str, ...]:
        """Loaded policies, display order."""
        return tuple(n for n in POLICY_NAMES if n in self._sessions)

    @property
    def availability(self) -> dict[str, str | None]:
        """What ``PolicyScheduler`` needs: name -> None | reason (incl. missing assets)."""
        return dict(self._availability)

    @property
    def num_joints(self) -> int:
        return len(self.joint_names)

    def has(self, name: str) -> bool:
        return str(name) in self._sessions

    def reset(self) -> None:
        """Forget the previous action (after a teleport / fall recovery)."""
        self.last_action[:] = 0.0

    def initial_qpos(self, data: mujoco.MjData) -> None:
        self._observer.initial_qpos(data)

    def projected_gravity(self, data: mujoco.MjData) -> NDArray[np.float32]:
        return self._observer.projected_gravity(data)

    def root_yaw(self, data: mujoco.MjData) -> float:
        return self._observer.root_yaw(data)

    def step(self, name: str, command: Any, data: mujoco.MjData) -> NDArray[np.float32]:
        """One 50 Hz step of policy ``name``: position targets in policy joint order."""
        session = self._sessions.get(str(name))
        if session is None:
            raise KeyError(f"Microduck policy {name!r} is not loaded (loaded: {self.names})")
        cmd = np.asarray(command, dtype=np.float32).reshape(-1)
        if cmd.shape[0] != COMMAND_LEN:
            raise ValueError(f"command must have {COMMAND_LEN} entries, got {cmd.shape[0]}")
        obs = self._observer.build(data, self.last_action, cmd)
        action = session.run(obs)
        self.last_action = action.copy()
        return self.default_pose + action * session.action_scale


class PolicyScheduler:
    """Decides which Microduck policy runs, with which command, and reports it.

    Thread model: ``set_twist`` / ``request`` may be called from any thread
    (a request is resolved against the current state into one concrete
    operation that fills the single pending slot, the latest wins); ``tick``
    and ``notify_fall`` run on the sim thread and are the only methods that
    change the running policy; ``snapshot`` and the properties are safe
    anywhere. Time is sim time fed through ``tick(dt)``; the injected
    ``clock`` only ages ``last_error``.

    States (``snapshot()["active"]``): a base name (``walk``, ``stand``,
    ``roller``, ``roller_crouch``), ``sitstand`` while seated,
    ``standing_up`` (sitstand with the posture flag cleared for 2 s),
    ``braking`` (the roller-family base being left keeps running for
    ``BRAKE_DURATION_S`` - ``roller`` at ``ROLLER_BRAKE_THROTTLE``,
    ``roller_crouch`` on its phase loop - before a legged base takes over)
    or a running oneshot's name. ``locked`` covers everything but an idle,
    upright base.
    """

    def __init__(
        self,
        available: Mapping[str, str | None],
        variant: str,
        *,
        spawn_ball: Callable[[float, float], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if variant not in _ALL_VARIANTS:
            raise ValueError(f"unknown Microduck variant {variant!r}; expected one of {VARIANTS}")
        self._variant = variant
        self._available: dict[str, str | None] = {
            name: available.get(name, "not loaded") for name in POLICY_NAMES
        }
        self._spawn_ball = spawn_ball
        self._clock = clock
        # RLock: spawn_ball runs inside tick() and may read the scheduler back.
        self._lock = threading.RLock()

        bases = [n for n in POLICY_NAMES if n in BASE_POLICIES and self._available[n] is None]
        if not bases:
            raise ValueError("PolicyScheduler needs at least one available base policy")
        self._base: str = _WALK if self._available[_WALK] is None else bases[0]
        self._active: str = self._base
        self._seated = False
        self._fallen = False
        self._twist: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._pending: _Op | None = None
        self._elapsed = 0.0  # sim seconds in the current timed state
        self._phase = 0.0  # ground_pick / roller_crouch phase in [0, 1)
        self._brake_policy: str | None = None  # roller-family base kept while braking
        self._brake_target: str | None = None  # legged base to switch to afterwards
        self._roulade_grace_left = 0.0
        self._last_error: str | None = None
        self._last_error_at = 0.0

    # ------------------------------------------------------------------ inputs

    def set_twist(self, vx: float, vy: float, wz: float) -> None:
        """Latest velocity request (already gain-shaped by the caller); any thread."""
        with self._lock:
            self._twist = (float(vx), float(vy), float(wz))

    def request(self, policy: str | None, action: str) -> tuple[bool, str]:
        """Queue ``action`` (start | stop | toggle) on ``policy``; any thread.

        ``policy=None`` with ``stop`` aborts whatever is running. The request
        is resolved right here against the current state (so a ``toggle``
        means abort or start depending on what is running *now*) and the
        resulting operation waits for the next ``tick``. Returns
        ``(accepted, reason)``; a rejection also lands in
        ``snapshot()["last_error"]``. Only one operation is pending at a
        time and the newest intent wins: a newer operation replaces an older
        unconsumed one, and a request that reverses the parked operation (a
        stop behind a start, a start behind a stop) empties the slot instead.
        Accepted no-ops (``"already seated"`` etc.) leave the slot alone.
        """
        name = None if policy is None else str(policy)
        with self._lock:
            ok, reason, op = self._resolve(name, action)
            if not ok:
                self._set_error(reason)
                return ok, reason
            self._last_error = None
            if op is _CANCEL:
                self._pending = None
            elif op is not None:
                self._pending = op
            return ok, reason

    def notify_fall(self, fallen: bool) -> None:
        """Sim thread: the fall detector's verdict; ignored while suspended.

        Edge-triggered: the first ``True`` after an upright period aborts a
        running oneshot / seat / stand-up / brake and locks the scheduler
        (base policy with a zero command) until ``False``; repeats are
        no-ops. Pass a debounced verdict (the sim module's tilt timer), not
        the raw per-tick tilt, or a one-tick spike aborts tricks and
        cancels navigation through ``locked``.
        """
        with self._lock:
            if not fallen:
                self._fallen = False
                return
            if self._fallen or self._suspend_fall_detector():
                return
            self._fallen = True
            active = self._active
            if active in ONESHOT_POLICIES:
                self._end_oneshot(active)
            elif self._seated or active == ACTIVE_STANDING_UP:
                self._seated = False
                self._active = self._base
            elif active == ACTIVE_BRAKING:
                self._finish_braking()
            self._elapsed = 0.0
            self._phase = 0.0

    # -------------------------------------------------------------- sim thread

    def tick(self, dt: float) -> tuple[str, NDArray[np.float32]]:
        """Sim thread only: consume the pending request, return (policy, command13).

        The returned policy is always a real, available policy name (during
        ``braking`` the roller-family base being left, during ``standing_up``
        ``sitstand``), so ``PolicyBank.step`` can always run it.
        """
        with self._lock:
            self._expire()
            op, self._pending = self._pending, None
            if op is not None:
                self._apply(op)
            name, command = self._command()
            self._accumulate(dt)
            return name, command

    # -------------------------------------------------------------- read side

    @property
    def variant(self) -> str:
        return self._variant

    @property
    def active(self) -> str:
        with self._lock:
            return self._active

    @property
    def base(self) -> str:
        with self._lock:
            return self._base

    @property
    def seated(self) -> bool:
        with self._lock:
            return self._seated

    @property
    def fallen(self) -> bool:
        with self._lock:
            return self._fallen

    @property
    def availability(self) -> dict[str, str | None]:
        return dict(self._available)

    @property
    def suspend_fall_detector(self) -> bool:
        """True while the roulade runs and for ``ROULADE_GRACE_S`` after it."""
        with self._lock:
            return self._suspend_fall_detector()

    @property
    def locked(self) -> bool:
        """Oneshot running, seated / standing up, braking, or fallen."""
        with self._lock:
            return self._lock_reason() is not None

    def snapshot(self) -> dict[str, Any]:
        """The ``policy_state`` JSON payload (see the module docstring)."""
        with self._lock:
            active = self._active
            oneshot = None
            if active in ONESHOT_POLICIES:
                oneshot = {"name": active, "progress": round(self._progress(active), 2)}
            return {
                "variant": self._variant,
                "active": active,
                "base": self._base,
                "seated": self._seated,
                "fallen": self._fallen,
                "locked": self._lock_reason() is not None,
                "oneshot": oneshot,
                "policies": [
                    {
                        "name": name,
                        "kind": str(POLICY_SPECS[PolicyName(name)].kind),
                        "available": self._available[name] is None,
                        "reason": self._available[name],
                    }
                    for name in POLICY_NAMES
                ],
                "last_error": self._current_error(),
                "t": time.time(),
            }

    # --------------------------------------------------------------- internals

    def _suspend_fall_detector(self) -> bool:
        return self._active == _ROULADE or self._roulade_grace_left > 0.0

    def _lock_reason(self) -> str | None:
        if self._fallen:
            return "fallen"
        if self._active in ONESHOT_POLICIES:
            return f"{self._active} running"
        if self._seated:
            return "seated"
        if self._active == ACTIVE_STANDING_UP:
            return "standing up"
        if self._active == ACTIVE_BRAKING:
            return "braking"
        return None

    def _set_error(self, reason: str) -> None:
        self._last_error = reason
        self._last_error_at = self._clock()

    def _current_error(self) -> str | None:
        if self._last_error is not None and self._clock() - self._last_error_at > LAST_ERROR_TTL_S:
            self._last_error = None
        return self._last_error

    def _progress(self, name: str) -> float:
        if name == _GROUND_PICK:
            value = self._phase / GROUND_PICK_END_PHASE
        else:
            duration = POLICY_SPECS[PolicyName(name)].duration or 1.0
            value = self._elapsed / duration
        return float(min(max(value, 0.0), 1.0))

    def _resolve(self, policy: str | None, action: str) -> tuple[bool, str, _Op | None]:
        """Turn a request into (accepted, reason, operation) against the current state.

        Pure apart from reading the state. ``operation`` is ``None`` for an
        accepted no-op and ``_CANCEL`` when the request reverses what is
        still parked in the pending slot - symmetric in both directions: a
        stop / toggle behind an unconsumed start (or sit) cancels it, and a
        start / toggle behind an unconsumed abort (or stand-up) of the same
        thing cancels that abort, so the running oneshot / seat is kept.
        """
        if action not in ACTIONS:
            return False, f"unknown action {action!r}", None
        pending = self._pending
        if policy is None:
            if action != "stop":
                return False, f"{action} needs a policy name", None
            if pending is not None and pending.kind in (_OP_START_ONESHOT, _OP_SIT):
                return True, "", _CANCEL
            if self._active in ONESHOT_POLICIES or self._seated:
                return True, "", _Op(_OP_ABORT_ANY)
            return True, "", None  # nothing to abort; a parked base select survives
        if policy not in self._available:
            return False, f"unknown policy {policy!r}", None
        reason = self._available[policy]
        if reason is not None:
            return False, f"{policy} unavailable: {reason}", None

        kind = POLICY_SPECS[PolicyName(policy)].kind
        lock_reason = self._lock_reason()

        if kind is PolicyKind.BASE:
            target = _WALK if action == "stop" else policy
            if self._available[target] is not None:
                return False, f"{target} unavailable: {self._available[target]}", None
            if lock_reason is not None:
                return False, f"locked: {lock_reason}", None
            return True, "", _Op(_OP_SELECT_BASE, target)

        if kind is PolicyKind.POSTURE:
            seated = self._seated
            pending_sit = pending is not None and pending.kind == _OP_SIT
            pending_stand_up = (
                seated and pending is not None and pending.kind in (_OP_STAND_UP, _OP_ABORT_ANY)
            )
            if action == "stop":
                if pending_sit:
                    return True, "", _CANCEL
                if seated:
                    return True, "", _Op(_OP_STAND_UP)
                if self._active == ACTIVE_STANDING_UP:
                    return True, "already standing up", None
                return False, "not seated", None
            if action == "toggle":
                if pending_sit or pending_stand_up:
                    return True, "", _CANCEL
                if seated:
                    return True, "", _Op(_OP_STAND_UP)
            else:  # start
                if pending_stand_up:
                    return True, "already seated", _CANCEL
                if seated or pending_sit:
                    return True, "already seated", None
            if lock_reason is not None:
                return False, f"locked: {lock_reason}", None
            return True, "", _Op(_OP_SIT)

        # oneshot
        running = self._active == policy
        pending_start = pending == _Op(_OP_START_ONESHOT, policy)
        pending_abort = (
            running
            and pending is not None
            and (pending == _Op(_OP_ABORT_ONESHOT, policy) or pending.kind == _OP_ABORT_ANY)
        )
        if action == "stop":
            if pending_start:
                return True, "", _CANCEL
            if running:
                return True, "", _Op(_OP_ABORT_ONESHOT, policy)
            return False, f"{policy} is not running", None
        if action == "toggle":
            if pending_start or pending_abort:
                return True, "", _CANCEL
            if running:
                return True, "", _Op(_OP_ABORT_ONESHOT, policy)
        else:  # start
            if pending_abort:
                return True, "already running", _CANCEL
            if running or pending_start:
                return True, "already running", None
        if lock_reason is not None:
            return False, f"locked: {lock_reason}", None
        return True, "", _Op(_OP_START_ONESHOT, policy)

    def _apply(self, op: _Op) -> None:
        """Run a resolved operation (sim thread, start of a tick).

        Stops whose subject already ended on its own (the oneshot expired,
        the duck stood up) are silently dropped. Starts are re-checked
        against the lock because a fall may have beaten them to the tick;
        that late rejection is reported through ``last_error``.
        """
        if op.kind == _OP_ABORT_ANY:
            if self._active in ONESHOT_POLICIES:
                self._end_oneshot(self._active)
            elif self._seated:
                self._begin_standing_up()
            return
        if op.kind == _OP_ABORT_ONESHOT:
            if self._active == op.target:
                self._end_oneshot(op.target)
            return
        if op.kind == _OP_STAND_UP:
            if self._seated:
                self._begin_standing_up()
            return

        if op.kind == _OP_SELECT_BASE and op.target == self._base and self._active == self._base:
            return
        if op.kind == _OP_SIT and self._seated:
            return
        if op.kind == _OP_START_ONESHOT and self._active == op.target:
            return
        lock_reason = self._lock_reason()
        if lock_reason is not None:
            self._set_error(f"locked: {lock_reason}")
            return
        if op.kind == _OP_SELECT_BASE:
            self._select_base(op.target)
        elif op.kind == _OP_SIT:
            self._sit()
        elif op.kind == _OP_START_ONESHOT:
            self._start_oneshot(op.target)

    def _select_base(self, target: str) -> None:
        if target == self._base:
            return
        if self._base in _ROLLER_FAMILY and target not in _ROLLER_FAMILY:
            # Leaving the wheels at speed drops the duck: keep the base being
            # left (always a loaded policy) for BRAKE_DURATION_S first.
            self._active = ACTIVE_BRAKING
            self._brake_policy = self._base
            self._brake_target = target
            self._elapsed = 0.0
            return
        self._base = target
        self._active = target
        self._phase = 0.0

    def _finish_braking(self) -> None:
        self._base = self._brake_target or _WALK
        self._brake_policy = None
        self._brake_target = None
        self._active = self._base
        self._elapsed = 0.0
        self._phase = 0.0

    def _sit(self) -> None:
        self._seated = True
        self._active = _SITSTAND
        self._elapsed = 0.0

    def _begin_standing_up(self) -> None:
        self._seated = False
        self._active = ACTIVE_STANDING_UP
        self._elapsed = 0.0

    def _start_oneshot(self, name: str) -> None:
        self._active = name
        self._elapsed = 0.0
        self._phase = 0.0
        offset = KICK_BALL_OFFSETS.get(name)
        if offset is not None and self._spawn_ball is not None:
            # The callback pokes MuJoCo from inside the engine's step hook; a
            # bug there must not take the sim loop down - the kick still runs.
            try:
                self._spawn_ball(*offset)
            except Exception:
                logger.exception("Microduck ball spawn failed; kicking without a ball", policy=name)

    def _end_oneshot(self, name: str) -> None:
        self._active = self._base
        self._elapsed = 0.0
        self._phase = 0.0
        if name == _ROULADE:
            self._roulade_grace_left = ROULADE_GRACE_S

    def _phase_command(self, cmd: NDArray[np.float32]) -> None:
        """ground_pick / roller_crouch encode their progress as (cos, sin) of the phase."""
        cmd[0] = math.cos(2.0 * math.pi * self._phase)
        cmd[1] = math.sin(2.0 * math.pi * self._phase)

    def _brake_with(self) -> str:
        """The loaded roller-family policy kept running while ``braking``."""
        return self._brake_policy or self._base

    def _command(self) -> tuple[str, NDArray[np.float32]]:
        """(policy to run, command13) for the current state."""
        cmd = np.zeros(COMMAND_LEN, dtype=np.float32)
        active = self._active
        if active == ACTIVE_BRAKING:
            policy = self._brake_with()
            if policy == _ROLLER_CROUCH:
                self._phase_command(cmd)
            else:
                cmd[0] = ROLLER_BRAKE_THROTTLE
            return policy, cmd
        if active == ACTIVE_STANDING_UP:
            return _SITSTAND, cmd  # posture flag 0 = stand
        if active == _SITSTAND:
            cmd[0] = 1.0  # posture flag 1 = sit
            return active, cmd
        if active in ONESHOT_POLICIES:
            if active == _GROUND_PICK:
                self._phase_command(cmd)
            return active, cmd
        # A base: twist-driven unless locked (fallen).
        if active == _ROLLER_CROUCH:
            self._phase_command(cmd)
            return active, cmd
        if self._fallen:
            return active, cmd
        vx, vy, wz = self._twist
        if active == _WALK:
            cmd[0] = float(np.clip(vx, *VX_RANGE))
            cmd[1] = float(np.clip(vy, *VY_RANGE))
            cmd[2] = float(np.clip(wz, *WZ_RANGE))
        elif active == _ROLLER:
            cmd[0] = float(np.clip(vx, *ROLLER_THROTTLE_RANGE))
            cmd[2] = float(np.clip(wz, *ROLLER_HEADING_RANGE))
        # stand ignores the twist
        return active, cmd

    def _oneshot_done(self, name: str) -> bool:
        if name == _GROUND_PICK:
            return self._phase >= GROUND_PICK_END_PHASE - 1e-9
        duration = POLICY_SPECS[PolicyName(name)].duration or 0.0
        return self._elapsed >= duration - 1e-9

    def _expire(self) -> None:
        """Leave timed states whose window elapsed (start of a tick)."""
        active = self._active
        if active == ACTIVE_BRAKING:
            if self._elapsed >= BRAKE_DURATION_S - 1e-9:
                self._finish_braking()
        elif active == ACTIVE_STANDING_UP:
            if self._elapsed >= STAND_UP_DURATION_S - 1e-9:
                self._active = self._base
                self._elapsed = 0.0
        elif active in ONESHOT_POLICIES and self._oneshot_done(active):
            self._end_oneshot(active)

    def _accumulate(self, dt: float) -> None:
        """Age the timed states by ``dt`` sim seconds (after the command was built)."""
        if self._roulade_grace_left > 0.0:
            self._roulade_grace_left = max(0.0, self._roulade_grace_left - dt)
        active = self._active
        if active in (ACTIVE_BRAKING, ACTIVE_STANDING_UP) or active in ONESHOT_POLICIES:
            self._elapsed += dt
        if active == _GROUND_PICK:
            self._phase += dt / GROUND_PICK_PERIOD_S
        elif active == _ROLLER_CROUCH or (
            active == ACTIVE_BRAKING and self._brake_with() == _ROLLER_CROUCH
        ):
            self._phase = (self._phase + dt / ROLLER_CROUCH_PERIOD_S) % 1.0


__all__ = [
    "ACTIONS",
    "ACTIVE_BRAKING",
    "ACTIVE_STANDING_UP",
    "ASSET_MISSING_REASON",
    "BASE_POLICIES",
    "BRAKE_DURATION_S",
    "DEFAULT_VARIANT",
    "FALL_GRAVITY_Z",
    "GROUND_PICK_END_PHASE",
    "GROUND_PICK_PERIOD_S",
    "KICK_BALL_OFFSETS",
    "KICK_DURATION_S",
    "LAST_ERROR_TTL_S",
    "ONESHOT_POLICIES",
    "POLICY_NAMES",
    "POLICY_SPECS",
    "ROLLERS_VARIANT",
    "ROLLER_BRAKE_THROTTLE",
    "ROLLER_CROUCH_PERIOD_S",
    "ROLLER_HEADING_RANGE",
    "ROLLER_THROTTLE_RANGE",
    "ROULADE_DURATION_S",
    "ROULADE_GRACE_S",
    "STAND_UP_DURATION_S",
    "VARIANTS",
    "PolicyBank",
    "PolicyKind",
    "PolicyName",
    "PolicyScheduler",
    "PolicySpec",
    "policy_availability",
]
