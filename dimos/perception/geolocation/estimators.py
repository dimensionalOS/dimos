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

"""Line-of-sight estimator and target ground-position estimator. Pure, no I/O.

The line of sight is valid only for a selected track observed in this frame, with fresh
vehicle and gimbal attitude and no gimbal failure. The target estimator intersects
the ray with a flat ground plane at the takeoff height and runs a constant-velocity Kalman
filter on [N, E, VN, VE]. NED, metres, degrees.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from typing import Literal

import numpy as np
from numpy.typing import NDArray

from dimos.perception.geolocation.geometry import LOSSolver, intersect_ground


@dataclass(frozen=True)
class EstimatorConfig:
    agl_source: Literal["relative_alt", "fixed"] = "relative_alt"
    fixed_agl_m: float = 1.0
    # The camera's drop below the point ``rel_alt`` is measured at.
    camera_below_ref_m: float = 0.0
    # Height above ground of the point the box centre is taken to be, per class.
    aim_height_m: dict[str, float] = field(
        default_factory=lambda: {
            "person": 1.0,
            "bicycle": 0.9,
            "motorcycle": 0.8,
            "car": 0.8,
            "bus": 1.5,
            "truck": 1.5,
            "dog": 0.3,
        }
    )
    default_aim_height_m: float = 0.8
    min_depression_deg: float = 5.0
    max_range_m: float = 150.0
    kf_q_pos: float = 0.05
    kf_q_vel: float = 0.8
    kf_r_base_m: float = 0.5
    kf_r_per_m: float = 0.06
    kf_init_pos_sigma_m: float = 5.0
    kf_init_vel_sigma_mps: float = 2.0
    max_meas_age_s: float = 1.0
    drop_after_s: float = 5.0
    max_state_age_s: float = 0.5
    max_gimbal_age_s: float = 1.0


@dataclass(frozen=True)
class Observation:
    """One tracked detection of a frame, in pixels of the camera model's resolution."""

    track_id: int
    class_name: str
    bbox: tuple[float, float, float, float]  # x, y, w, h


@dataclass(frozen=True)
class VehicleGeo:
    """The vehicle as the estimators need it: heading, position, height."""

    yaw_deg: float | None = None
    attitude_age_s: float = math.inf
    n: float | None = None
    e: float | None = None
    d: float | None = None
    rel_alt: float | None = None


@dataclass(frozen=True)
class GimbalGeo:
    pitch_deg: float
    yaw_deg: float
    age_s: float
    failure_flags: int = 0


@dataclass(frozen=True)
class LosResult:
    capture_time: float
    track_id: int | None = None  # set once the selected track is in the frame
    class_name: str | None = None
    valid: bool = False
    reason: str = ""
    los_ned: tuple[float, float, float] | None = None
    azimuth_deg: float | None = None
    elevation_deg: float | None = None
    gimbal_yaw_body_deg: float | None = None


class LosEstimator:
    def __init__(self, solver: LOSSolver, cfg: EstimatorConfig) -> None:
        self.solver = solver
        self.cfg = cfg

    def process(
        self,
        observations: list[Observation],
        selected: int | None,
        capture_time: float,
        vehicle: VehicleGeo,
        gimbal: GimbalGeo | None,
    ) -> LosResult:
        base = LosResult(capture_time=capture_time)
        if selected is None:
            return replace(base, reason="no selection")
        track = next((o for o in observations if o.track_id == selected), None)
        if track is None:
            return replace(base, reason="selected target not visible")
        base = replace(base, track_id=track.track_id, class_name=track.class_name)
        if vehicle.yaw_deg is None or vehicle.attitude_age_s > self.cfg.max_state_age_s:
            return replace(base, reason="vehicle attitude stale")
        if gimbal is None or gimbal.age_s > self.cfg.max_gimbal_age_s:
            return replace(base, reason="gimbal attitude stale")
        if gimbal.failure_flags:
            return replace(base, reason=f"gimbal failure_flags={gimbal.failure_flags}")
        x, y, w, h = track.bbox
        r = self.solver.solve(
            x + w / 2.0, y + h / 2.0, gimbal.pitch_deg, gimbal.yaw_deg, vehicle.yaw_deg
        )
        if not all(map(math.isfinite, (*r.los_ned, r.gimbal_yaw_body_deg))):
            return replace(base, reason="non-finite line of sight")
        return replace(
            base,
            valid=True,
            reason="ok",
            los_ned=r.los_ned,
            azimuth_deg=r.azimuth_deg,
            elevation_deg=r.elevation_deg,
            gimbal_yaw_body_deg=r.gimbal_yaw_body_deg,
        )


class TargetKF:
    """[n, e, vn, ve] constant-velocity filter."""

    def __init__(self, cfg: EstimatorConfig) -> None:
        self.cfg = cfg
        self.x: NDArray[np.float64] | None = None
        self.P: NDArray[np.float64] = np.eye(4)
        self.t = 0.0
        self.t_meas: float | None = None
        self.track_id: int | None = None

    def reset(self, track_id: int | None, n: float, e: float, t: float) -> None:
        c = self.cfg
        self.x = np.array([n, e, 0.0, 0.0])
        self.P = np.diag([c.kf_init_pos_sigma_m**2] * 2 + [c.kf_init_vel_sigma_mps**2] * 2)
        self.t = t
        self.t_meas = t
        self.track_id = track_id

    def predict(self, t: float) -> None:
        if self.x is None:
            return
        dt = max(0.0, min(1.0, t - self.t))
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        q_p, q_v = self.cfg.kf_q_pos, self.cfg.kf_q_vel
        q = np.diag([q_p * dt, q_p * dt, q_v * dt, q_v * dt])
        self.x = f @ self.x
        self.P = f @ self.P @ f.T + q
        self.t = t

    def update(self, n: float, e: float, range_m: float, t: float) -> None:
        self.predict(t)
        assert self.x is not None
        sigma = self.cfg.kf_r_base_m + self.cfg.kf_r_per_m * range_m
        r = np.eye(2) * sigma**2
        h = np.zeros((2, 4))
        h[0, 0] = h[1, 1] = 1.0
        z = np.array([n, e])
        y = z - h @ self.x
        s = h @ self.P @ h.T + r
        k = self.P @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ y
        self.P = (np.eye(4) - k @ h) @ self.P
        self.t_meas = t


@dataclass(frozen=True)
class TargetState:
    """The filtered target on the flat ground plane at takeoff height (NED, metres)."""

    t: float
    valid: bool
    reason: str
    n: float | None = None
    e: float | None = None
    vn: float | None = None
    ve: float | None = None
    range_m: float | None = None
    bearing_deg: float | None = None


def agl_metres(cfg: EstimatorConfig, veh: VehicleGeo) -> tuple[float | None, str]:
    if cfg.agl_source == "fixed":
        return cfg.fixed_agl_m, "fixed"
    if veh.rel_alt is None:
        return None, "relative_alt unavailable"
    return veh.rel_alt - cfg.camera_below_ref_m, "relative_alt"


class TargetEstimator:
    def __init__(self, cfg: EstimatorConfig) -> None:
        self.cfg = cfg
        self.kf = TargetKF(cfg)

    def process(self, los: LosResult | None, veh: VehicleGeo, now: float) -> TargetState:
        cfg = self.cfg
        reason: str | None = None
        meas: dict[str, float] | None = None
        if los is None:
            reason = "no LOS packets"
        elif not los.valid or los.los_ned is None:
            reason = f"LOS: {los.reason}"
        else:
            agl_val, agl_src = agl_metres(cfg, veh)
            if agl_val is None:
                reason = f"AGL: {agl_src}"
            elif agl_val < 0.3:
                reason = f"AGL {agl_val:.2f} m too small ({agl_src})"
            else:
                aim = cfg.aim_height_m.get(los.class_name or "", cfg.default_aim_height_m)
                d_cam = veh.d if veh.d is not None else 0.0
                n_cam = veh.n if veh.n is not None else 0.0
                e_cam = veh.e if veh.e is not None else 0.0
                # Camera at d_cam, ground plane at d_cam + agl, aim plane aim m above ground.
                plane_d = d_cam + agl_val - aim
                pt, rng = intersect_ground(
                    (n_cam, e_cam, d_cam), los.los_ned, plane_d, cfg.min_depression_deg
                )
                if pt is None or not isinstance(rng, float):
                    reason = f"intersect: {rng}"
                elif not all(map(math.isfinite, (*pt, rng))):
                    # NaN passes every comparison and would never leave the filter.
                    reason = "non-finite measurement"
                elif rng > cfg.max_range_m:
                    reason = f"range {rng:.0f} m > max"
                else:
                    meas = {"n": pt[0], "e": pt[1], "range_m": rng, "t": los.capture_time or now}
        if meas is not None and los is not None:
            if self.kf.track_id != los.track_id or self.kf.x is None:
                self.kf.reset(los.track_id, meas["n"], meas["e"], meas["t"])
            else:
                self.kf.update(meas["n"], meas["e"], meas["range_m"], meas["t"])
        else:
            self.kf.predict(now)
            if self.kf.t_meas is not None and now - self.kf.t_meas > cfg.drop_after_s:
                self.kf.x = None
        return self._output(veh, reason, now)

    def _output(self, veh: VehicleGeo, reason: str | None, now: float) -> TargetState:
        kf = self.kf
        if kf.x is None or kf.t_meas is None:
            return TargetState(t=now, valid=False, reason=reason or "ok")
        age = now - kf.t_meas
        n, e, vn, ve = (float(v) for v in kf.x)
        dn = n - (veh.n or 0.0)
        de = e - (veh.e or 0.0)
        valid = age <= self.cfg.max_meas_age_s
        if not valid and reason is None:
            reason = f"no measurement for {age:.1f}s"
        return TargetState(
            t=now,
            valid=valid,
            reason=reason or "ok",
            n=n,
            e=e,
            vn=vn,
            ve=ve,
            range_m=math.hypot(dn, de),
            bearing_deg=(math.degrees(math.atan2(de, dn)) + 360) % 360,
        )
