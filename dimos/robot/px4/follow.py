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

"""Target following for the PX4 supervisor: the YAW_TRACK and FOLLOW guidance states.

YAW_TRACK unwinds the gimbal toward the airframe centre-line by yawing the vehicle; FOLLOW
holds a standoff distance and altitude to the estimated target with velocity feed-forward.
Neither ever commands from raw pixel error.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.utils.transform_utils import normalize_angle

if TYPE_CHECKING:
    from dimos.robot.px4.mavlink import VehicleSnapshot
    from dimos.robot.px4.supervisor_core import SupervisorCore


@dataclass(frozen=True)
class YawTrackConfig:
    """Gains of the YAW_TRACK law."""

    deadband_deg: float = 15.0
    k_yaw: float = 0.6
    max_yaw_rate_dps: float = 30.0


@dataclass(frozen=True)
class FollowConfig:
    """Gains and limits of the FOLLOW law."""

    standoff_m: float = 12.0
    altitude_m: float = 10.0
    k_range: float = 0.4
    k_alt: float = 0.6
    v_max_mps: float = 2.0
    vz_max_mps: float = 0.7
    ff_gain: float = 0.8
    range_deadband_m: float = 1.5
    loss_hover_s: float = 5.0


@dataclass
class TargetEstimate:
    """The target as the supervisor sees it (NED, metres, m/s).

    ``valid`` with ``n``/``e`` is what FOLLOW flies on; a line of sight alone
    (``los_valid``, no position) is enough for YAW_TRACK.
    """

    valid: bool = False
    n: float | None = None
    e: float | None = None
    vn: float = 0.0
    ve: float = 0.0
    los_valid: bool = False
    gimbal_yaw_body_deg: float | None = None


@dataclass(frozen=True)
class FollowCommand:
    vn: float
    ve: float
    vd: float
    yaw_deg: float  # the bearing to the target
    range_m: float


def yaw_track_rate(
    gimbal_yaw_body_deg: float | None, target_visible: bool, cfg: YawTrackConfig
) -> float:
    """deg/s of vehicle yaw to command. Positive = clockwise (right), toward the gimbal."""
    if not target_visible or gimbal_yaw_body_deg is None:
        return 0.0
    g = gimbal_yaw_body_deg
    db = cfg.deadband_deg
    if abs(g) <= db:
        return 0.0
    err = g - math.copysign(db, g)
    return float(np.clip(cfg.k_yaw * err, -cfg.max_yaw_rate_dps, cfg.max_yaw_rate_dps))


def follow_velocity(
    veh_n: float,
    veh_e: float,
    veh_d: float,
    d_takeoff: float,
    target: TargetEstimate,
    cfg: FollowConfig,
) -> FollowCommand:
    """Velocity + yaw command to keep standoff geometry to the target."""
    assert target.n is not None and target.e is not None, "follow_velocity needs a position"
    dn, de = target.n - veh_n, target.e - veh_e
    rng = math.hypot(dn, de)
    bearing = (math.degrees(math.atan2(de, dn)) + 360.0) % 360.0
    if rng < 1e-3:
        un, ue = 1.0, 0.0
    else:
        un, ue = dn / rng, de / rng
    err = rng - cfg.standoff_m
    if abs(err) <= cfg.range_deadband_m:
        v_rad = 0.0
    else:
        v_rad = cfg.k_range * (err - math.copysign(cfg.range_deadband_m, err))
    vn = v_rad * un + cfg.ff_gain * target.vn
    ve = v_rad * ue + cfg.ff_gain * target.ve
    speed = math.hypot(vn, ve)
    if speed > cfg.v_max_mps:
        vn, ve = vn * cfg.v_max_mps / speed, ve * cfg.v_max_mps / speed
    d_des = d_takeoff - cfg.altitude_m
    vd = float(np.clip(cfg.k_alt * (d_des - veh_d), -cfg.vz_max_mps, cfg.vz_max_mps))
    return FollowCommand(vn=vn, ve=ve, vd=vd, yaw_deg=bearing, range_m=rng)


class FollowGuidance:
    """The target the supervisor was last given, and the two states that fly on it."""

    def __init__(self, yaw_track: YawTrackConfig, follow: FollowConfig, stale_s: float) -> None:
        self.yaw_track = yaw_track
        self.cfg = follow
        self.stale_s = stale_s
        self.target: TargetEstimate | None = None
        self.target_rx_t = 0.0  # any target estimate
        self.position_rx_t = 0.0  # the position in it, which a line of sight can re-send
        self.target_valid_t = 0.0  # last estimate with a usable (valid, n/e) target

    def on_target(
        self, target: TargetEstimate, now: float, position_t: float | None = None
    ) -> None:
        self.target, self.target_rx_t = target, now
        self.position_rx_t = now if position_t is None else position_t

    def fresh(self, now: float) -> bool:
        return (
            self.target is not None
            and self.target.valid
            and self.target.n is not None
            and self.target.e is not None
            and now - self.position_rx_t <= self.stale_s
        )

    def status(self, now: float) -> dict[str, Any]:
        t = self.target
        return dict(
            target_fresh=self.fresh(now),
            target=None
            if t is None
            else dict(n=t.n, e=t.e, gimbal_yaw_body_deg=t.gimbal_yaw_body_deg),
        )

    def step(self, core: SupervisorCore, st: VehicleSnapshot, now: float, dt: float) -> None:
        """One tick of YAW_TRACK or FOLLOW, whichever ``core.state`` is."""
        if core.state == "YAW_TRACK":
            self._step_yaw_track(core, st, now, dt)
        else:
            self._step_follow(core, st, now, dt)

    def _step_yaw_track(
        self, core: SupervisorCore, st: VehicleSnapshot, now: float, dt: float
    ) -> None:
        # config.py takes its follow configs from here and supervisor_core imports config.py,
        # so the core's names cannot be imported at module level.
        from dimos.robot.px4.supervisor_core import Setpoint

        core.hover = core.hover or core.here(st)
        visible = self.fresh(now) or (
            self.target is not None
            and self.target.los_valid
            and now - self.target_rx_t <= self.stale_s
        )
        g = self.target.gimbal_yaw_body_deg if self.target else None
        rate = yaw_track_rate(g, visible, self.yaw_track)
        core.yaw_cmd = math.degrees(normalize_angle(math.radians(core.yaw_cmd + rate * dt)))
        core.sp = Setpoint(
            kind="pos",
            n=core.hover.n,
            e=core.hover.e,
            d=core.hover.d,
            yaw=core.yaw_cmd,
            yaw_rate=rate,
        )
        if core.guidance_mode != "YAW_TRACK":
            core.goto("HOVER", "mode change", now)

    def _step_follow(
        self, core: SupervisorCore, st: VehicleSnapshot, now: float, dt: float
    ) -> None:
        from dimos.robot.px4.supervisor_core import Setpoint, rate_limit_yaw

        if self.fresh(now):
            self.target_valid_t = now
        # Loss clock starts at the later of: last valid target, entering FOLLOW.
        age = now - max(self.target_valid_t, core.state_since)
        if self.fresh(now):
            assert st.local is not None and core.takeoff is not None and self.target is not None
            core.reason = "following"
            cmd = follow_velocity(
                st.local.n, st.local.e, st.local.d, core.takeoff.d0, self.target, self.cfg
            )
            core.yaw_cmd = rate_limit_yaw(
                core.yaw_cmd, cmd.yaw_deg, self.yaw_track.max_yaw_rate_dps, dt
            )
            core.sp = Setpoint(
                kind="vel",
                vn=cmd.vn,
                ve=cmd.ve,
                vd=cmd.vd,
                yaw=core.yaw_cmd,
            )
            core.hover = None
        elif age <= self.cfg.loss_hover_s:
            core.hover = core.hover or core.here(st)
            core.sp = Setpoint(
                kind="pos", n=core.hover.n, e=core.hover.e, d=core.hover.d, yaw=core.yaw_cmd
            )
            core.reason = f"target lost {age:.1f}s: holding position"
        else:
            core.hover = core.hover or core.here(st)
            core.guidance_mode = "HOVER"
            core.goto("HOVER", f"target lost {age:.1f}s, holding here", now)
            return
        if core.guidance_mode != "FOLLOW":
            core.hover = core.here(st)
            core.goto("HOVER", "mode change", now)
