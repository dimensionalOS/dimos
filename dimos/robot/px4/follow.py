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

"""The YAW_TRACK and FOLLOW guidance laws. Pure functions; the supervisor flies them.

YAW_TRACK unwinds the gimbal toward the airframe centre-line by yawing the vehicle; FOLLOW
holds a standoff distance and altitude to the estimated target with velocity feed-forward.
Neither ever commands from raw pixel error.
"""

from __future__ import annotations

import math

import numpy as np

from dimos.robot.px4.config import FollowConfig, YawTrackConfig


def yaw_track_rate(gimbal_yaw_body_deg: float | None, cfg: YawTrackConfig) -> float:
    """deg/s of vehicle yaw to command. Positive = clockwise (right), toward the gimbal."""
    if gimbal_yaw_body_deg is None:
        return 0.0
    g, db = gimbal_yaw_body_deg, cfg.deadband_deg
    if abs(g) <= db:
        return 0.0
    err = g - math.copysign(db, g)
    return float(np.clip(cfg.k_yaw * err, -cfg.max_yaw_rate_dps, cfg.max_yaw_rate_dps))


def follow_velocity(
    veh_n: float,
    veh_e: float,
    veh_d: float,
    d_takeoff: float,
    target: tuple[float, float, float, float],
    cfg: FollowConfig,
) -> tuple[float, float, float, float]:
    """NED velocity and heading (degrees) that keep the standoff to ``target`` (n, e, vn, ve)."""
    tn, te, tvn, tve = target
    dn, de = tn - veh_n, te - veh_e
    rng = math.hypot(dn, de)
    un, ue = (dn / rng, de / rng) if rng > 1e-3 else (1.0, 0.0)
    err = rng - cfg.standoff_m
    v_rad = 0.0
    if abs(err) > cfg.range_deadband_m:
        v_rad = cfg.k_range * (err - math.copysign(cfg.range_deadband_m, err))
    vn = v_rad * un + cfg.ff_gain * tvn
    ve = v_rad * ue + cfg.ff_gain * tve
    speed = math.hypot(vn, ve)
    if speed > cfg.v_max_mps:
        vn, ve = vn * cfg.v_max_mps / speed, ve * cfg.v_max_mps / speed
    d_des = d_takeoff - cfg.altitude_m
    vd = float(np.clip(cfg.k_alt * (d_des - veh_d), -cfg.vz_max_mps, cfg.vz_max_mps))
    return vn, ve, vd, math.degrees(math.atan2(de, dn))
