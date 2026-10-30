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

"""PX4 Offboard flight supervisor state machine. Pure logic: no socket, no pymavlink.

States::

  IDLE -> PREFLIGHT -> STREAMING -> OFFBOARD_REQ -> ARMING -> TAKEOFF -> HOVER
  HOVER <-> YAW_TRACK / FOLLOW / TELEOP       (operator selects the guidance mode)
  any guidance state -> GOTO -> HOVER         (operator go-to; ends hovering at the goal)
  any armed state -> LANDING -> IDLE          (operator land)
  any armed state -> ABORT -> IDLE            (safety rule: PX4 put in Hold, setpoints stop)
  pilot leaves Offboard -> IDLE               (PILOT_OVERRIDE: we never touch the mode)
  PX4 disarms -> IDLE                         (the setpoint must not outlive the flight)

Every tick the owner takes a :class:`VehicleSnapshot`, calls :meth:`SupervisorCore.step`,
then :meth:`SupervisorCore.stream`. All vehicle side effects go through the
:class:`Px4Actuator` protocol, so the machine runs against a fake in tests.

Two clocks: ``now`` times the state timers and the operator inputs; vehicle staleness
comes from the snapshot's receive-time ages.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass
import enum
import math
import time
from typing import Any, Literal, Protocol

import numpy as np

from dimos.robot.px4.config import GotoConfig, GuidanceConfig, SupervisorLimits
from dimos.robot.px4.follow import follow_velocity, yaw_track_rate
from dimos.robot.px4.mavlink import (
    LANDED_ON_GROUND,
    MAIN_AUTO,
    MAIN_OFFBOARD,
    SUB_AUTO_LAND,
    SUB_AUTO_LOITER,
    VehicleSnapshot,
    body_flu_velocity_to_ned,
)
from dimos.utils.transform_utils import normalize_angle

# GOTO is a guidance state but not a selectable mode: it needs a goal, so only goto_cmd
# enters it.
GUIDANCE_STATES = frozenset({"HOVER", "YAW_TRACK", "FOLLOW", "TELEOP", "GOTO"})
ARMED_STATES = GUIDANCE_STATES | {"ARMING", "TAKEOFF", "LANDING"}
# Where the safety rules apply: the armed states but LANDING (PX4 owns the descent), and the
# pre-stream before the arm command.
_ABORTABLE = (ARMED_STATES - {"LANDING"}) | {"STREAMING", "OFFBOARD_REQ"}
# From an accepted takeoff to the hover; any other state after it means it failed.
TAKEOFF_STATES = frozenset({"PREFLIGHT", "STREAMING", "OFFBOARD_REQ", "ARMING", "TAKEOFF"})
# The HOVER reason that tells a finished go-to from an interrupted or timed-out one.
GOTO_ARRIVED = "arrived at the go-to goal"
GuidanceMode = Literal["HOVER", "YAW_TRACK", "FOLLOW", "TELEOP"]
GUIDANCE_MODES: tuple[GuidanceMode, ...] = ("HOVER", "YAW_TRACK", "FOLLOW", "TELEOP")

_ABORT_DWELL_S = 2.0
_PREFLIGHT_TIMEOUT_S = 10.0
_MAX_STEP_DT_S = 0.2


class Rejection(enum.Enum):
    """Why a command or input was refused. A closed set, so callers can classify on it."""

    NOT_TELEOP = "not_teleop"
    ESTOP_LATCHED = "estop_latched"
    ENABLE_SWITCH_OFF = "enable_switch_off"
    STALE_INPUT = "stale_input"
    PREFLIGHT_FAILED = "preflight_failed"
    NOT_ARMED = "not_armed"
    MODE_NOT_OFFBOARD = "mode_not_offboard"
    FENCE = "fence"
    CEILING = "ceiling"
    BATTERY = "battery"
    # A command that makes no sense now: takeoff while flying, estop_clear outside IDLE.
    WRONG_STATE = "wrong_state"
    # Unusable numbers: an altitude under min_alt_m, a NaN.
    INVALID_ARGUMENT = "invalid_argument"


@dataclass(frozen=True)
class TakeoffPoint:
    """Captured before arming; never changed after (it is the geofence origin)."""

    n: float
    e: float
    d0: float
    yaw: float


@dataclass(frozen=True)
class HoverPoint:
    n: float
    e: float
    d: float


@dataclass(frozen=True)
class GotoGoal:
    """Where GOTO flies to, local NED. ``yaw`` None keeps the heading the vehicle has."""

    n: float
    e: float
    d: float
    yaw: float | None = None  # degrees, NED heading


@dataclass(frozen=True)
class Setpoint:
    """What :meth:`SupervisorCore.stream` sends. ``pos`` uses n/e/d, ``vel`` uses vn/ve/vd."""

    kind: Literal["pos", "vel"]
    yaw: float  # degrees, NED heading
    n: float = 0.0
    e: float = 0.0
    d: float = 0.0
    vn: float = 0.0
    ve: float = 0.0
    vd: float = 0.0
    yaw_rate: float | None = None  # deg/s; when set, a vel setpoint commands rate not heading

    @staticmethod
    def hold(p: HoverPoint, yaw: float) -> Setpoint:
        return Setpoint(kind="pos", n=p.n, e=p.e, d=p.d, yaw=yaw)


@dataclass(frozen=True)
class TeleopCommand:
    """A body-frame FLU velocity request (dimOS ``Twist``) with its receive time."""

    forward: float
    left: float
    up: float
    yaw_rate_ccw: float  # rad/s
    t: float

    @property
    def is_zero(self) -> bool:
        return (
            self.forward == 0.0 and self.left == 0.0 and self.up == 0.0 and self.yaw_rate_ccw == 0.0
        )


def _wrap180(deg: float) -> float:
    return math.degrees(normalize_angle(math.radians(deg)))


def rate_limit_yaw(current_deg: float, desired_deg: float, max_rate_dps: float, dt: float) -> float:
    limit = max_rate_dps * dt
    return _wrap180(
        current_deg + float(np.clip(_wrap180(desired_deg - current_deg), -limit, limit))
    )


def goto_velocity(
    veh_n: float, veh_e: float, veh_d: float, goal: GotoGoal, cfg: GotoConfig
) -> tuple[float, float, float]:
    """NED velocity toward the goal: proportional to the distance left, capped per axis.

    The cap is ours, not PX4's: a far position setpoint would be flown at whatever
    MPC_XY_VEL_MAX happens to be set on the vehicle.
    """
    dn, de = goal.n - veh_n, goal.e - veh_e
    rng = math.hypot(dn, de)
    speed = min(cfg.k_pos * rng, cfg.v_max_mps)
    vn, ve = (speed * dn / rng, speed * de / rng) if rng > 1e-3 else (0.0, 0.0)
    vd = float(np.clip(cfg.k_alt * (goal.d - veh_d), -cfg.vz_max_mps, cfg.vz_max_mps))
    return vn, ve, vd


class Px4Actuator(Protocol):
    """The only way the core touches the aircraft."""

    def set_mode(self, main: int, sub: int = 0) -> None: ...

    def arm(self, value: bool) -> None: ...

    def send_position_setpoint(self, n: float, e: float, d: float, yaw_rad: float) -> None: ...

    def send_velocity_setpoint(
        self,
        vn: float,
        ve: float,
        vd: float,
        yaw_rad: float | None = None,
        yaw_rate_rad: float | None = None,
    ) -> None: ...


class SupervisorCore:
    def __init__(
        self,
        limits: SupervisorLimits,
        guidance: GuidanceConfig,
        sitl: bool = False,
    ) -> None:
        self.cfg = limits
        self.gcfg = guidance
        self.sitl = sitl
        self.state = "IDLE"
        self.state_since = time.time()
        self.reason = "startup"
        self.guidance_mode: GuidanceMode = "HOVER"
        self.fake_enable = False
        # The followed target, written by the owner as its streams arrive: the position and
        # velocity (n, e, vn, ve) and the gimbal yaw (body-relative, clockwise) to it.
        self.target_valid = False
        self.target: tuple[float, float, float, float] | None = None
        self.target_t = 0.0
        self.los_yaw_deg: float | None = None
        self.los_t = 0.0
        self.target_fresh_t = 0.0  # the last tick FOLLOW had a target to fly on
        self.sp: Setpoint | None = None
        self.yaw_cmd: float = 0.0
        self.takeoff: TakeoffPoint | None = None
        self.takeoff_alt_m = limits.takeoff_alt_m
        self.hover: HoverPoint | None = None
        self.goal: GotoGoal | None = None
        self.entered_offboard = False
        self.last_step: float | None = None
        self.estop_latched = False
        self.last_rejection: Rejection | None = None
        self.teleop: TeleopCommand | None = None
        self.teleop_d: float | None = None
        self.setpoint_count = 0
        self.transitions: list[tuple[str, str, float]] = []

    def goto(self, state: str, reason: str = "", now: float | None = None) -> None:
        t = time.time() if now is None else now
        if state != self.state:
            self.transitions.append((state, reason, t))
            self.teleop_d = None  # the next stay in TELEOP locks its own altitude
        if state in ("IDLE", "ABORT", "LANDING"):
            self.sp = None  # the setpoint stream stops with the flight
            self.teleop = None
        if state == "IDLE":
            # A selected mode never outlives its flight: the next takeoff ends in a plain
            # HOVER, whatever ended this one (land, hold, E-STOP, abort, pilot override).
            self.guidance_mode = "HOVER"
            self.entered_offboard = False
        self.state, self.state_since, self.reason = state, t, reason

    def enable_switch(self, st: VehicleSnapshot) -> bool:
        if self.sitl:
            return self.fake_enable
        if st.rc is None or st.rc_age > self.cfg.rc_stale_s:
            return False
        ch = self.cfg.enable_channel
        if ch < 1 or ch > len(st.rc.chan):
            return False
        return st.rc.chan[ch - 1] >= self.cfg.enable_threshold_us

    def preflight_failures(self, st: VehicleSnapshot) -> list[str]:
        c, f = self.cfg, []
        if st.px4_msg_age > c.px4_stale_s:
            f.append("PX4 heartbeat stale")
        if st.local is None or st.local_age > 0.5:
            f.append("local position stale")
        if st.gps is None or st.gps.fix < c.min_fix_type:
            f.append("GPS fix")
        elif not math.isnan(st.gps.eph) and st.gps.eph > c.max_eph_m:
            f.append(f"eph {st.gps.eph:.1f}")
        batt = st.batt_pct
        if batt < 0:
            if c.require_battery:
                f.append("battery unknown")
        elif batt < c.min_batt_pct:
            f.append(f"battery {batt}%")
        if not self.sitl and (st.rc is None or st.rc_age > c.rc_stale_s):
            f.append("RC stale")
        if not self.enable_switch(st):
            f.append("enable switch off")
        if st.landed_state is None or st.landed_state != LANDED_ON_GROUND:
            f.append("not on ground")
        if st.armed:
            f.append("already armed")
        # PX4 refuses to arm for Offboard without an absolute position estimate; saying so
        # here beats "arming refused" three seconds later.
        if st.estimator is not None and not st.estimator.position_valid:
            f.append("position estimate not valid")
        return f

    def _px4_said(self, st: VehicleSnapshot, hint: str) -> str:
        """PX4's own warning since the current state began, else the operator's hint."""
        text = st.statustext
        if text is not None and text.t >= self.state_since:
            return f": {text.text}"
        return f" ({hint})"

    def abort_reason(self, st: VehicleSnapshot) -> tuple[str, Rejection] | None:
        c = self.cfg
        if st.px4_msg_age > c.px4_stale_s:
            return "PX4 heartbeat lost", Rejection.STALE_INPUT
        if not self.enable_switch(st):
            return "enable switch off / RC lost", Rejection.ENABLE_SWITCH_OFF
        # The guidance laws and the fence below fly on this sample; a frozen one blinds both.
        if st.local is None or st.local_age > c.px4_stale_s:
            return "local position lost", Rejection.STALE_INPUT
        # PX4 still publishes the local position after it drops the absolute one, but it
        # drifts: the fence and the guidance laws would fly on a position PX4 disowned.
        if st.estimator is not None and not st.estimator.position_valid:
            return "position estimate not valid", Rejection.STALE_INPUT
        batt = st.batt_pct
        if 0 <= batt < c.min_batt_pct - 10:
            return f"battery {batt}%", Rejection.BATTERY
        if st.local and self.takeoff:
            dist = math.hypot(st.local.n - self.takeoff.n, st.local.e - self.takeoff.e)
            alt = self.takeoff.d0 - st.local.d
            if dist > c.geofence_radius_m:
                return f"geofence {dist:.0f} m", Rejection.FENCE
            if alt > c.max_alt_m:
                return f"altitude {alt:.1f} m", Rejection.CEILING
        return None

    # Operator commands. Each returns None when accepted or the rejection reason.

    def sitl_enable(self, value: bool) -> Rejection | None:
        if not self.sitl:
            return Rejection.WRONG_STATE
        self.fake_enable = bool(value)
        return None

    def _altitude_rejection(self, alt_m: float) -> Rejection | None:
        """Why an operator altitude (above the takeoff point) is unusable, or None."""
        c = self.cfg
        if not math.isfinite(alt_m) or alt_m < c.min_alt_m:
            return Rejection.INVALID_ARGUMENT
        if alt_m > c.max_alt_m - c.goal_margin_m:
            return Rejection.CEILING
        return None

    def takeoff_cmd(
        self, st: VehicleSnapshot, now: float | None = None, alt_m: float | None = None
    ) -> Rejection | None:
        """Take off to ``alt_m`` above the ground, or to the configured altitude when None."""
        if self.estop_latched:
            return self._reject(Rejection.ESTOP_LATCHED)
        if self.state != "IDLE":
            return self._reject(Rejection.WRONG_STATE)
        alt = self.cfg.takeoff_alt_m if alt_m is None else alt_m
        why = self._altitude_rejection(alt)
        if why is not None:
            return self._reject(why)
        self.takeoff_alt_m = alt
        self.goto("PREFLIGHT", f"operator takeoff to {alt:.1f} m", now)
        return None

    def goto_cmd(
        self,
        st: VehicleSnapshot,
        north_m: float,
        east_m: float,
        alt_m: float | None = None,
        heading_deg: float | None = None,
        relative: bool = True,
        now: float | None = None,
    ) -> Rejection | None:
        """Fly to a point and hover there. Accepted only while flying in a guidance state.

        ``north_m``/``east_m`` count from where the vehicle is (``relative``) or from the
        takeoff point; ``alt_m`` is above the takeoff point and ``heading_deg`` is a compass
        heading, both kept as they are when None. The goal must sit ``goal_margin_m``
        inside the fence and the ceiling.
        """
        if self.estop_latched:
            return self._reject(Rejection.ESTOP_LATCHED)
        if self.state not in GUIDANCE_STATES:
            flying = self.state in ARMED_STATES
            return self._reject(Rejection.WRONG_STATE if flying else Rejection.NOT_ARMED)
        if st.local is None or self.takeoff is None:
            return self._reject(Rejection.STALE_INPUT)
        heading = 0.0 if heading_deg is None else heading_deg
        if not all(math.isfinite(v) for v in (north_m, east_m, heading)):
            return self._reject(Rejection.INVALID_ARGUMENT)
        if alt_m is not None:
            why = self._altitude_rejection(alt_m)
            if why is not None:
                return self._reject(why)
        origin = st.local if relative else self.takeoff
        n, e = origin.n + north_m, origin.e + east_m
        north, east = n - self.takeoff.n, e - self.takeoff.e
        if math.hypot(north, east) > self.cfg.geofence_radius_m - self.cfg.goal_margin_m:
            return self._reject(Rejection.FENCE)
        self.goal = GotoGoal(
            n=n,
            e=e,
            d=st.local.d if alt_m is None else self.takeoff.d0 - alt_m,
            yaw=None if heading_deg is None else _wrap180(heading_deg),
        )
        # GOTO ends in HOVER at the goal, whatever mode it interrupted.
        self.guidance_mode = "HOVER"
        self.hover = None
        self.teleop = None
        self.goto("GOTO", f"go to {north:+.1f} m north, {east:+.1f} m east of takeoff", now)
        return None

    def _holds_px4(self, st: VehicleSnapshot) -> bool:
        """PX4 is in Offboard because of us, so a Hold or Land command is ours to send."""
        ours = self.state in ARMED_STATES or self.state == "OFFBOARD_REQ" or st.armed
        return ours and st.in_offboard

    def land_cmd(
        self, st: VehicleSnapshot, m: Px4Actuator, now: float | None = None
    ) -> Rejection | None:
        if not (self.state in ARMED_STATES or st.armed):
            return self._reject(Rejection.NOT_ARMED)
        if self.state == "LANDING":
            return self._reject(Rejection.WRONG_STATE)
        if not st.in_offboard:
            # The pilot owns the aircraft; we never send a mode command.
            return self._reject(Rejection.MODE_NOT_OFFBOARD)
        m.set_mode(MAIN_AUTO, SUB_AUTO_LAND)
        self.goto("LANDING", "operator land", now)
        return None

    def hold_cmd(
        self, st: VehicleSnapshot, m: Px4Actuator, now: float | None = None
    ) -> Rejection | None:
        if self._holds_px4(st):
            m.set_mode(MAIN_AUTO, SUB_AUTO_LOITER)
        self.goto("IDLE", "operator hold", now)
        return None

    def set_guidance_mode(self, mode: GuidanceMode, now: float | None = None) -> Rejection | None:
        if self.estop_latched:
            return self._reject(Rejection.ESTOP_LATCHED)
        self.guidance_mode = mode
        if self.state in GUIDANCE_STATES:
            self.goto(mode, "operator mode", now)
        return None

    def estop(
        self, st: VehicleSnapshot, m: Px4Actuator, now: float | None = None, land: bool = False
    ) -> None:
        """Hold (or Land) plus latch. Synchronous; nothing can move the aircraft until
        estop_clear."""
        self.estop_latched = True
        if not self._holds_px4(st):
            self.goto("IDLE", "E-STOP (pilot has the aircraft)" if land else "E-STOP", now)
        elif land:
            m.set_mode(MAIN_AUTO, SUB_AUTO_LAND)
            self.goto("LANDING", "E-STOP land", now)
        else:
            m.set_mode(MAIN_AUTO, SUB_AUTO_LOITER)
            self.goto("IDLE", "E-STOP", now)

    def estop_clear(self) -> Rejection | None:
        if self.state != "IDLE":
            return self._reject(Rejection.WRONG_STATE)
        self.estop_latched = False
        return None

    def on_cmd_vel(self, cmd: TeleopCommand, now: float) -> Rejection | None:
        """Accept a teleop velocity request. Honoured only in TELEOP; clamped to the limits."""
        if self.estop_latched:
            return self._reject(Rejection.ESTOP_LATCHED)
        if self.state != "TELEOP":
            return self._reject(Rejection.NOT_TELEOP)
        values = (cmd.forward, cmd.left, cmd.up, cmd.yaw_rate_ccw, cmd.t)
        if not all(math.isfinite(v) for v in values) or now - cmd.t > self.cfg.teleop_stale_s:
            return self._reject(Rejection.STALE_INPUT)
        c = self.cfg
        xy, z, yaw = c.teleop_v_xy_mps, c.teleop_v_z_mps, c.teleop_yaw_rate_rps
        k = min(1.0, xy / max(math.hypot(cmd.forward, cmd.left), 1e-9))
        self.teleop = TeleopCommand(
            forward=cmd.forward * k,
            left=cmd.left * k,
            up=float(np.clip(cmd.up, -z, z)),
            yaw_rate_ccw=float(np.clip(cmd.yaw_rate_ccw, -yaw, yaw)),
            t=cmd.t,
        )
        return None

    def _reject(self, why: Rejection) -> Rejection:
        self.last_rejection = why
        return why

    # Main step

    def step(self, st: VehicleSnapshot, m: Px4Actuator, now: float) -> None:
        s = self.state
        dt = 0.0 if self.last_step is None else max(0.0, min(_MAX_STEP_DT_S, now - self.last_step))
        self.last_step = now
        armed = st.armed
        in_offboard = st.in_offboard

        # Pilot override: we asked for Offboard, PX4 is no longer in it -> stop, never touch modes.
        if s in ARMED_STATES and s != "LANDING" and self.entered_offboard and not in_offboard:
            self.last_rejection = Rejection.MODE_NOT_OFFBOARD
            self.goto("IDLE", f"PILOT_OVERRIDE (mode now {st.mode})", now)
            return
        # A disarm (PX4's auto-disarm, QGC, the RC) can leave PX4 in Offboard: streaming on
        # would fly the old setpoint at the next arm, with no preflight.
        if s in ARMED_STATES - {"ARMING", "LANDING"} and not armed:
            self.last_rejection = Rejection.NOT_ARMED
            self.goto("IDLE", "PX4 disarmed", now)
            return
        if s in _ABORTABLE and (r := self.abort_reason(st)):
            text, self.last_rejection = r
            if s in ARMED_STATES or in_offboard:
                m.set_mode(MAIN_AUTO, SUB_AUTO_LOITER)
            if s in ARMED_STATES:
                self.goto("ABORT", text, now)
            else:  # before the arm command: cancel the takeoff, never arm and then abort
                self.goto("IDLE", f"takeoff cancelled: {text}", now)
            return

        if s == "ABORT":
            if now - self.state_since > _ABORT_DWELL_S:
                self.goto("IDLE", "after abort", now)
        elif s == "PREFLIGHT":
            fails = self.preflight_failures(st)
            if fails:
                if now - self.state_since > _PREFLIGHT_TIMEOUT_S:
                    self.last_rejection = (
                        Rejection.ENABLE_SWITCH_OFF
                        if "enable switch off" in fails
                        else Rejection.PREFLIGHT_FAILED
                    )
                    self.goto("IDLE", "preflight failed: " + ", ".join(fails), now)
                else:
                    self.reason = "waiting: " + ", ".join(fails)
            else:
                assert st.local is not None
                self.takeoff = TakeoffPoint(
                    n=st.local.n,
                    e=st.local.e,
                    d0=st.local.d,
                    yaw=st.yaw_deg if st.yaw_deg is not None else 0.0,
                )
                self.yaw_cmd = self.takeoff.yaw
                self.sp = Setpoint.hold(
                    HoverPoint(self.takeoff.n, self.takeoff.e, self.takeoff.d0), self.yaw_cmd
                )
                self.goto(
                    "STREAMING",
                    f"takeoff point N{self.takeoff.n:.1f} E{self.takeoff.e:.1f} D{self.takeoff.d0:.1f}",
                    now,
                )
        elif s == "STREAMING":
            if now - self.state_since >= self.cfg.prestream_s:
                m.set_mode(MAIN_OFFBOARD)
                self.goto("OFFBOARD_REQ", "requested OFFBOARD", now)
        elif s == "OFFBOARD_REQ":
            if in_offboard:
                self.entered_offboard = True
                m.arm(True)
                self.goto("ARMING", "arm requested", now)
            elif now - self.state_since > self.cfg.ack_timeout_s:
                self.last_rejection = Rejection.MODE_NOT_OFFBOARD
                hint = "check COM_RC_OVERRIDE / preflight in QGC"
                self.goto("IDLE", "PX4 did not enter OFFBOARD" + self._px4_said(st, hint), now)
        elif s == "ARMING":
            if armed:
                self.goto("TAKEOFF", "armed, climbing", now)
            elif now - self.state_since > self.cfg.ack_timeout_s:
                self.last_rejection = Rejection.NOT_ARMED
                m.set_mode(MAIN_AUTO, SUB_AUTO_LOITER)
                self.goto("IDLE", "arming refused" + self._px4_said(st, "see QGC messages"), now)
        elif s == "TAKEOFF":
            assert self.takeoff is not None
            d_goal = self.takeoff.d0 - self.takeoff_alt_m
            d_ramp = self.takeoff.d0 - self.cfg.climb_rate_mps * (now - self.state_since)
            self.sp = Setpoint.hold(
                HoverPoint(self.takeoff.n, self.takeoff.e, max(d_goal, d_ramp)), self.yaw_cmd
            )
            if (
                st.local
                and abs(st.local.d - d_goal) < self.cfg.hover_tolerance_m
                and d_ramp <= d_goal
            ):
                self.hover = HoverPoint(n=self.takeoff.n, e=self.takeoff.e, d=d_goal)
                self.goto("HOVER", f"at {self.takeoff_alt_m} m", now)
        elif s == "HOVER":
            self.hover = self.hover or self.here(st)
            self.sp = Setpoint.hold(self.hover, self.yaw_cmd)
            if self.guidance_mode != "HOVER" and now - self.state_since > self.cfg.hover_settle_s:
                self.goto(self.guidance_mode, "settled", now)
        elif s == "YAW_TRACK":
            self._step_yaw_track(st, now, dt)
        elif s == "FOLLOW":
            self._step_follow(st, now, dt)
        elif s == "TELEOP":
            self._step_teleop(st, now)
        elif s == "GOTO":
            self._step_goto(st, now, dt)
        elif s == "LANDING":
            if st.landed_state == LANDED_ON_GROUND and not armed:
                self.goto("IDLE", "landed and disarmed", now)

    def target_fresh(self, now: float) -> bool:
        stale = now - self.target_t > self.cfg.target_stale_s
        return self.target_valid and self.target is not None and not stale

    def _step_yaw_track(self, st: VehicleSnapshot, now: float, dt: float) -> None:
        self.hover = self.hover or self.here(st)
        fresh = now - self.los_t <= self.cfg.target_stale_s
        rate = yaw_track_rate(self.los_yaw_deg if fresh else None, self.gcfg.yaw_track)
        self.yaw_cmd = _wrap180(self.yaw_cmd + rate * dt)
        self.sp = Setpoint.hold(self.hover, self.yaw_cmd)

    def _step_follow(self, st: VehicleSnapshot, now: float, dt: float) -> None:
        if self.target_fresh(now):
            assert st.local is not None and self.takeoff is not None and self.target is not None
            self.target_fresh_t = now
            here, d0 = st.local, self.takeoff.d0
            vn, ve, vd, bearing = follow_velocity(
                here.n, here.e, here.d, d0, self.target, self.gcfg.follow
            )
            rate = self.gcfg.yaw_track.max_yaw_rate_dps
            self.yaw_cmd = rate_limit_yaw(self.yaw_cmd, bearing, rate, dt)
            self.sp = Setpoint(kind="vel", vn=vn, ve=ve, vd=vd, yaw=self.yaw_cmd)
            self.hover = None
            self.reason = "following"
            return
        # The loss clock starts at the later of the last target and entering FOLLOW.
        age = now - max(self.target_fresh_t, self.state_since)
        self.hover = self.hover or self.here(st)
        if age <= self.gcfg.follow.loss_hover_s:
            self.sp = Setpoint.hold(self.hover, self.yaw_cmd)
            self.reason = f"target lost {age:.1f}s: holding position"
        else:
            self.guidance_mode = "HOVER"
            self.goto("HOVER", f"target lost {age:.1f}s, holding here", now)

    def _step_teleop(self, st: VehicleSnapshot, now: float) -> None:
        c, here = self.cfg, self.here(st)
        if self.teleop_d is None:
            self.teleop_d = self.hover.d if self.hover is not None else here.d
        cmd = self.teleop
        moving = cmd is not None and now - cmd.t <= c.teleop_stale_s and not cmd.is_zero
        if moving:
            assert cmd is not None
            yaw_rad = math.radians(st.yaw_deg if st.yaw_deg is not None else self.yaw_cmd)
            vn, ve, vd = body_flu_velocity_to_ned(cmd.forward, cmd.left, cmd.up, yaw_rad)
            if c.teleop_lock_altitude:
                # Hold the altitude TELEOP started at. Zero vertical speed is not a hold:
                # every key sags a little, and the keyboard has no way back up.
                vd = c.teleop_k_alt * (self.teleop_d - here.d)
                vd = float(np.clip(vd, -c.teleop_v_z_mps, c.teleop_v_z_mps))
            # dimOS yaw rate is counter-clockwise positive; NED heading rate is clockwise.
            rate_dps = -math.degrees(cmd.yaw_rate_ccw)
            self.yaw_cmd = st.yaw_deg if st.yaw_deg is not None else self.yaw_cmd
            self.sp = Setpoint(kind="vel", vn=vn, ve=ve, vd=vd, yaw=self.yaw_cmd, yaw_rate=rate_dps)
            self.hover = None
            self.reason = "teleop"
        else:
            if self.hover is None:
                d = self.teleop_d if c.teleop_lock_altitude else here.d
                self.hover = HoverPoint(n=here.n, e=here.e, d=d)
                self.yaw_cmd = st.yaw_deg if st.yaw_deg is not None else self.yaw_cmd
            self.sp = Setpoint.hold(self.hover, self.yaw_cmd)
            self.reason = "teleop idle: holding position"

    def _step_goto(self, st: VehicleSnapshot, now: float, dt: float) -> None:
        assert self.goal is not None
        goal, gc, here = self.goal, self.gcfg.goto, self.here(st)
        if goal.yaw is not None:
            self.yaw_cmd = rate_limit_yaw(self.yaw_cmd, goal.yaw, gc.max_yaw_rate_dps, dt)
        rng = math.hypot(goal.n - here.n, goal.e - here.e)
        tol = self.cfg.hover_tolerance_m
        yaw_now = st.yaw_deg if st.yaw_deg is not None else self.yaw_cmd
        facing = goal.yaw is None or abs(_wrap180(goal.yaw - yaw_now)) <= gc.yaw_tolerance_deg
        if rng < tol and abs(goal.d - here.d) < tol and facing:
            # The position setpoint of HOVER closes what is left of the tolerance.
            self.hover = HoverPoint(n=goal.n, e=goal.e, d=goal.d)
            if goal.yaw is not None:
                self.yaw_cmd = goal.yaw
            self.goto("HOVER", GOTO_ARRIVED, now)
        elif now - self.state_since > gc.timeout_s:
            self.hover = here
            self.goto("HOVER", f"go-to timed out {rng:.1f} m short, holding here", now)
        else:
            vn, ve, vd = goto_velocity(here.n, here.e, here.d, goal, gc)
            self.sp = Setpoint(kind="vel", vn=vn, ve=ve, vd=vd, yaw=self.yaw_cmd)
            self.reason = f"go-to: {rng:.1f} m to go"

    @staticmethod
    def here(st: VehicleSnapshot) -> HoverPoint:
        assert st.local is not None, "no local position"
        return HoverPoint(n=st.local.n, e=st.local.e, d=st.local.d)

    def stream(self, m: Px4Actuator) -> bool:
        """Send the current setpoint, once per tick (the tick runs at ``setpoint_hz``).
        Returns True when one was sent."""
        sp = self.sp
        if sp is None:
            return False
        if sp.kind == "pos":
            m.send_position_setpoint(sp.n, sp.e, sp.d, math.radians(sp.yaw))
        elif sp.yaw_rate is not None:
            m.send_velocity_setpoint(sp.vn, sp.ve, sp.vd, yaw_rate_rad=math.radians(sp.yaw_rate))
        else:
            m.send_velocity_setpoint(sp.vn, sp.ve, sp.vd, yaw_rad=math.radians(sp.yaw))
        self.setpoint_count += 1
        return True

    def status(self, st: VehicleSnapshot, now: float | None = None) -> dict[str, Any]:
        """What the supervisor knows, for the status RPC."""
        t = time.time() if now is None else now
        sp = self.sp
        out: dict[str, Any] = dict(
            t=t,
            state=self.state,
            reason=self.reason,
            guidance_mode=self.guidance_mode,
            armed=st.armed,
            px4_mode=st.mode,
            enable=self.enable_switch(st),
            sitl=self.sitl,
            setpoint=None if sp is None else dataclasses.asdict(sp),
            goal=dataclasses.asdict(self.goal) if self.state == "GOTO" and self.goal else None,
            local=None if st.local is None else dict(n=st.local.n, e=st.local.e, d=st.local.d),
            batt_pct=st.sys_status.batt_pct if st.sys_status else None,
            gps=None if st.gps is None else dict(fix=st.gps.fix, sats=st.gps.sats, eph=st.gps.eph),
            target_fresh=self.target_fresh(t),
            target=None
            if self.target is None
            else dict(n=self.target[0], e=self.target[1], gimbal_yaw_body_deg=self.los_yaw_deg),
            position_valid=None if st.estimator is None else st.estimator.position_valid,
            estop_latched=self.estop_latched,
            last_rejection=None if self.last_rejection is None else self.last_rejection.value,
            setpoint_count=self.setpoint_count,
            teleop_age_s=None if self.teleop is None else t - self.teleop.t,
        )
        if st.local and self.takeoff:
            north, east = st.local.n - self.takeoff.n, st.local.e - self.takeoff.e
            out["alt_m"] = self.takeoff.d0 - st.local.d
            out["dist_m"] = math.hypot(north, east)
            out["north_m"], out["east_m"] = north, east
        return out
