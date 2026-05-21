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

"""Time-indexed SE(2) reference trajectories for plant-bottleneck diagnosis.

A ``Trajectory`` is a callable ``ref_fn(t) -> TrajRefState`` plus a
duration, a recommended controller mode, and a serializable spec for
replay from ``run.json``.

The reference is the **kinematic ideal** under the unicycle model
starting at the origin facing +x. A perfectly-behaved plant would still
show ``along_track_lag ~ L + tau`` against this reference — that is the
expected baseline, not a failure mode. See ``score_run_with_trajectory``.

Each helper picks a recommended controller mode. Open-loop FF is reserved
for short trials and trials with no sustained yaw rate (no compounding
integration drift in yaw). Sustained or oscillating ``wz`` needs the
low-gain P fallback to keep the robot on the reference long enough for
the e(t) decomposition to be interpretable.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import math
from typing import Any, Literal

import numpy as np

ControllerMode = Literal["openloop_ff", "lowgain_p"]


@dataclass(frozen=True)
class TrajRefState:
    """Reference pose + velocity at a given time."""

    t: float
    x: float
    y: float
    yaw: float
    vx: float
    wz: float


RefFn = Callable[[float], TrajRefState]


@dataclass(frozen=True)
class Trajectory:
    """A time-indexed reference plus its replay descriptor.

    ``spec`` is JSON-serializable and round-trips through
    :func:`trajectory_from_spec`. It is what ``run.json`` carries so
    ``diagnose.py`` can rebuild ``ref_fn`` long after the run.
    """

    ref_fn: RefFn
    duration_s: float
    recommended_mode: ControllerMode
    spec: dict[str, Any] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Primitives — closed-form integrals of the unicycle model
# ---------------------------------------------------------------------------


def straight(v: float, duration: float) -> Trajectory:
    """Constant ``vx=v``, ``wz=0`` from the origin along +x for ``duration`` seconds."""

    def ref_fn(t: float) -> TrajRefState:
        t = _clip(t, 0.0, duration)
        return TrajRefState(t=t, x=v * t, y=0.0, yaw=0.0, vx=v, wz=0.0)

    return Trajectory(
        ref_fn=ref_fn,
        duration_s=duration,
        recommended_mode="openloop_ff",
        spec={"kind": "straight", "v": v, "duration": duration},
    )


def circle(v: float, w: float, duration: float) -> Trajectory:
    """Constant ``vx=v, wz=w`` from the origin. Radius = ``v/w``."""
    if abs(w) < 1e-9:
        return straight(v, duration)

    inv_w = 1.0 / w

    def ref_fn(t: float) -> TrajRefState:
        t = _clip(t, 0.0, duration)
        yaw = w * t
        x = v * inv_w * math.sin(yaw)
        y = v * inv_w * (1.0 - math.cos(yaw))
        return TrajRefState(t=t, x=x, y=y, yaw=yaw, vx=v, wz=w)

    return Trajectory(
        ref_fn=ref_fn,
        duration_s=duration,
        recommended_mode="lowgain_p",
        spec={"kind": "circle", "v": v, "w": w, "duration": duration},
    )


def step_vx(v_target: float, duration: float, *, t_step: float = 0.5) -> Trajectory:
    """Zero until ``t_step``, then constant ``vx=v_target``, ``wz=0``."""

    def ref_fn(t: float) -> TrajRefState:
        t = _clip(t, 0.0, duration)
        if t < t_step:
            return TrajRefState(t=t, x=0.0, y=0.0, yaw=0.0, vx=0.0, wz=0.0)
        return TrajRefState(t=t, x=v_target * (t - t_step), y=0.0, yaw=0.0, vx=v_target, wz=0.0)

    return Trajectory(
        ref_fn=ref_fn,
        duration_s=duration,
        recommended_mode="openloop_ff",
        spec={
            "kind": "step_vx",
            "v_target": v_target,
            "duration": duration,
            "t_step": t_step,
        },
    )


def step_wz(
    vx: float,
    w_target: float,
    duration: float,
    *,
    t_step: float = 0.5,
) -> Trajectory:
    """Straight at ``vx`` until ``t_step``, then constant ``wz=w_target`` while still moving at ``vx``."""

    if abs(w_target) < 1e-9:
        return straight(vx, duration)

    inv_w = 1.0 / w_target

    def ref_fn(t: float) -> TrajRefState:
        t = _clip(t, 0.0, duration)
        if t < t_step:
            return TrajRefState(t=t, x=vx * t, y=0.0, yaw=0.0, vx=vx, wz=0.0)
        x0 = vx * t_step
        tau_active = t - t_step
        yaw = w_target * tau_active
        x = x0 + vx * inv_w * math.sin(yaw)
        y = vx * inv_w * (1.0 - math.cos(yaw))
        return TrajRefState(t=t, x=x, y=y, yaw=yaw, vx=vx, wz=w_target)

    return Trajectory(
        ref_fn=ref_fn,
        duration_s=duration,
        recommended_mode="openloop_ff",
        spec={
            "kind": "step_wz",
            "vx": vx,
            "w_target": w_target,
            "duration": duration,
            "t_step": t_step,
        },
    )


def trapezoidal_vx(v_max: float, accel: float, duration: float) -> Trajectory:
    """Trapezoidal ``vx`` profile: ramp 0 → v_max → 0 with magnitude ``accel`` m/s^2, ``wz=0``.

    The hold portion is whatever's left after accel + decel time. If
    ``duration`` is too short to reach ``v_max``, hold time is zero and
    the profile is triangular.
    """
    if accel <= 0.0 or v_max <= 0.0:
        raise ValueError(f"accel and v_max must be positive (got {accel=}, {v_max=})")

    t_accel = v_max / accel
    if 2.0 * t_accel > duration:
        # Triangular: v_peak is whatever we reach in duration/2 at accel
        t_accel = duration / 2.0
        v_peak = accel * t_accel
        t_hold = 0.0
    else:
        v_peak = v_max
        t_hold = duration - 2.0 * t_accel
    t_decel_start = t_accel + t_hold

    # Accumulated x at each phase boundary
    x_at_accel_end = 0.5 * accel * t_accel * t_accel
    x_at_hold_end = x_at_accel_end + v_peak * t_hold

    def ref_fn(t: float) -> TrajRefState:
        t = _clip(t, 0.0, duration)
        if t < t_accel:
            v = accel * t
            x = 0.5 * accel * t * t
        elif t < t_decel_start:
            v = v_peak
            x = x_at_accel_end + v_peak * (t - t_accel)
        else:
            dt_decel = t - t_decel_start
            v = max(0.0, v_peak - accel * dt_decel)
            x = x_at_hold_end + v_peak * dt_decel - 0.5 * accel * dt_decel * dt_decel
        return TrajRefState(t=t, x=x, y=0.0, yaw=0.0, vx=v, wz=0.0)

    return Trajectory(
        ref_fn=ref_fn,
        duration_s=duration,
        recommended_mode="openloop_ff",
        spec={
            "kind": "trapezoidal_vx",
            "v_max": v_max,
            "accel": accel,
            "duration": duration,
        },
    )


def sinusoidal_wz(
    vx: float,
    w_amp: float,
    freq_hz: float,
    duration: float,
    *,
    integration_dt: float = 0.001,
) -> Trajectory:
    """Constant ``vx``, ``wz = w_amp * sin(2*pi*freq_hz*t)``.

    No closed form for the position integral (cos of an oscillating
    argument). Precomputes a fine numerical integration once at
    construction; ``ref_fn`` linearly interpolates that grid.
    """
    if integration_dt <= 0.0:
        raise ValueError(f"integration_dt must be positive (got {integration_dt=})")

    n = math.ceil(duration / integration_dt) + 1
    ts = np.linspace(0.0, duration, n)
    omega = 2.0 * math.pi * freq_hz

    # yaw(t) = integral of wz(s) ds = (w_amp/omega) * (1 - cos(omega*t))
    if omega < 1e-9:
        yaws = np.zeros(n)
    else:
        yaws = (w_amp / omega) * (1.0 - np.cos(omega * ts))

    cos_yaw = np.cos(yaws)
    sin_yaw = np.sin(yaws)
    xs = np.zeros(n)
    ys = np.zeros(n)
    # Trapezoidal integration of (vx*cos(yaw), vx*sin(yaw))
    for i in range(1, n):
        dt = ts[i] - ts[i - 1]
        xs[i] = xs[i - 1] + 0.5 * vx * (cos_yaw[i] + cos_yaw[i - 1]) * dt
        ys[i] = ys[i - 1] + 0.5 * vx * (sin_yaw[i] + sin_yaw[i - 1]) * dt

    def ref_fn(t: float) -> TrajRefState:
        t = _clip(t, 0.0, duration)
        # Linear interpolation on the precomputed grid
        x = float(np.interp(t, ts, xs))
        y = float(np.interp(t, ts, ys))
        yaw = float(np.interp(t, ts, yaws))
        wz = w_amp * math.sin(omega * t)
        return TrajRefState(t=t, x=x, y=y, yaw=yaw, vx=vx, wz=wz)

    return Trajectory(
        ref_fn=ref_fn,
        duration_s=duration,
        recommended_mode="lowgain_p",
        spec={
            "kind": "sinusoidal_wz",
            "vx": vx,
            "w_amp": w_amp,
            "freq_hz": freq_hz,
            "duration": duration,
        },
    )


# ---------------------------------------------------------------------------
# Start-pose anchoring
# ---------------------------------------------------------------------------


def anchor_trajectory(
    ref_fn: RefFn,
    start_x: float,
    start_y: float,
    start_yaw: float,
) -> RefFn:
    """Wrap ``ref_fn`` so its output is expressed in the world frame.

    Trajectory primitives are defined in a local frame: the robot starts
    at the origin facing +x. On a real robot the start pose is wherever
    odom puts it. Without this transform, ``e(t) = pose - ref`` measures
    the fixed start-pose offset, not plant behavior.

    Applies the SE(2) transform ``(start_x, start_y, start_yaw)`` to the
    position and heading. Body-frame ``vx``/``wz`` are invariant under a
    rigid transform of the path and pass through unchanged.
    """
    cos_y = math.cos(start_yaw)
    sin_y = math.sin(start_yaw)

    def wrapped(t: float) -> TrajRefState:
        loc = ref_fn(t)
        return TrajRefState(
            t=loc.t,
            x=start_x + cos_y * loc.x - sin_y * loc.y,
            y=start_y + sin_y * loc.x + cos_y * loc.y,
            yaw=start_yaw + loc.yaw,
            vx=loc.vx,
            wz=loc.wz,
        )

    return wrapped


# ---------------------------------------------------------------------------
# Replay
# ---------------------------------------------------------------------------


_FACTORIES: dict[str, Callable[..., Trajectory]] = {
    "straight": straight,
    "circle": circle,
    "step_vx": step_vx,
    "step_wz": step_wz,
    "trapezoidal_vx": trapezoidal_vx,
    "sinusoidal_wz": sinusoidal_wz,
}


def trajectory_from_spec(spec: dict[str, Any]) -> Trajectory:
    """Reconstruct a :class:`Trajectory` from its replay ``spec`` dict.

    Used by ``diagnose.py`` to rebuild ``ref_fn`` from ``run.json``.
    """
    kind = spec.get("kind")
    factory = _FACTORIES.get(kind) if isinstance(kind, str) else None
    if factory is None:
        raise ValueError(f"unknown trajectory kind: {kind!r}")
    kwargs = {k: v for k, v in spec.items() if k != "kind"}
    return factory(**kwargs)


# ---------------------------------------------------------------------------


def _clip(t: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, t))


__all__ = [
    "ControllerMode",
    "RefFn",
    "TrajRefState",
    "Trajectory",
    "circle",
    "sinusoidal_wz",
    "step_vx",
    "step_wz",
    "straight",
    "trajectory_from_spec",
    "trapezoidal_vx",
]
