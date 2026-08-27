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

"""Stride decomposition: speed = stride length x stride frequency, per leg.

The ``speed`` statistic hides WHICH factor a simulator gets wrong, and
``gait_hz`` (autocorrelation of the body-height bob) does not measure leg
cadence: on the freewalk grounding it reads 1.33 (sim) and 1.72 (real) while
the legs of BOTH cycle at 1.93-1.96 Hz. This module measures the legs
directly — foot positions from rigid FK on the joint angles, touchdown events
from the gravity-aligned foot height, stride length as body planar travel
between a leg's consecutive touchdowns.

Engine-free on purpose (README 6): the same instrument runs on a recording
(``st.lq`` + tracker travel) and on a closed-loop rollout (``run.q`` +
``run.pos``), so a sim-real stride comparison can never be an instrument
difference. FK is validated against MuJoCo to machine precision in the tests.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from dimos.simulation.sysid.rotations import quat_to_mat

if TYPE_CHECKING:
    from dimos.simulation.sysid.recording import Streams

# Go2 leg geometry, metres, from the menagerie go2.xml body tree (base ->
# hip -> thigh -> calf -> foot geom). Joint axes: hip abduction about +x,
# thigh and calf about +y. Leg order matches MUJOCO_ACTUATOR_NAMES.
LEGS = ("FL", "FR", "RL", "RR")
_HIP_XY = {
    "FL": (0.1934, 0.0465),
    "FR": (0.1934, -0.0465),
    "RL": (-0.1934, 0.0465),
    "RR": (-0.1934, -0.0465),
}
_THIGH_Y = {"FL": 0.0955, "FR": -0.0955, "RL": 0.0955, "RR": -0.0955}
_L_THIGH = 0.213
_FOOT = np.array([-0.002, 0.0, -0.213])  # foot geom centre in the calf frame

# A leg whose foot-height range is under this never left the ground: no strides.
_MIN_SWING_M = 0.02
# Stride-interval gate: outside a plausible walking cadence (0.6-4 Hz) the
# interval spans a pause, not a stride.
_INTERVAL_S = (0.25, 1.6)


def foot_base(q: np.ndarray) -> np.ndarray:
    """(n, 12) joint angles -> (n, 4, 3) foot centres in the base frame."""
    n = len(q)
    out = np.empty((n, 4, 3))
    for i, leg in enumerate(LEGS):
        qh, qt, qc = q[:, 3 * i], q[:, 3 * i + 1], q[:, 3 * i + 2]
        c, s = np.cos(qc), np.sin(qc)  # calf joint, Ry
        fx = c * _FOOT[0] + s * _FOOT[2]
        fz = -s * _FOOT[0] + c * _FOOT[2]
        px, pz = fx, fz - _L_THIGH
        c, s = np.cos(qt), np.sin(qt)  # thigh joint, Ry
        tx = c * px + s * pz
        tz = -s * px + c * pz
        hy0 = _THIGH_Y[leg]
        c, s = np.cos(qh), np.sin(qh)  # hip abduction, Rx
        out[:, i, 0] = tx + _HIP_XY[leg][0]
        out[:, i, 1] = c * hy0 - s * tz + _HIP_XY[leg][1]
        out[:, i, 2] = s * hy0 + c * tz
    return out


def touchdowns(t: np.ndarray, h: np.ndarray) -> np.ndarray:
    """Touchdown times from one leg's gravity-aligned foot height.

    Hysteresis between 35% and 65% of the height range: a single threshold
    chatters on the bob, and the exact stance boundary is not needed — only
    the once-per-cycle event.
    """
    lo, hi = np.percentile(h, 15), np.percentile(h, 85)
    if hi - lo < _MIN_SWING_M:
        return np.zeros(0)
    a, b = lo + 0.35 * (hi - lo), lo + 0.65 * (hi - lo)
    stance = h[0] < a
    events = []
    for i in range(1, len(h)):
        if stance and h[i] > b:
            stance = False
        elif not stance and h[i] < a:
            stance = True
            events.append(t[i])
    return np.array(events)


@dataclass(frozen=True)
class Strides:
    """Per-leg medians and their means; NaN when a leg never strode."""

    stride_hz: float  # mean over legs of 1 / median touchdown interval
    stride_len: float  # mean over legs of median body travel per stride, m
    stride_hz_leg: tuple[float, ...]
    stride_len_leg: tuple[float, ...]
    n_strides: int

    def as_dict(self) -> dict[str, float]:
        return {"stride_hz": self.stride_hz, "stride_len": self.stride_len}


def strides(
    t: np.ndarray,
    q: np.ndarray,
    quat: np.ndarray,
    planar: np.ndarray | None,
    moving: np.ndarray,
) -> Strides:
    """Stride frequency and length from joints + body attitude + planar travel.

    ``planar`` is the (n, 2) body position at ``t`` (tracker on the real
    side, ``run.pos`` in sim); ``None`` — a tracker-less recording — still
    measures the cadence and leaves every length NaN. ``moving`` masks
    samples where the operator commanded motion, so pauses cannot read as
    long strides (the interval gate drops any stride spanning a mask edge).
    """
    fb = foot_base(q)
    R = quat_to_mat(quat)
    h = np.einsum("nij,nkj->nki", R, fb)[:, :, 2]
    hz, ln, n = [], [], 0
    for i in range(4):
        ev = touchdowns(t, h[:, i])
        if len(ev) < 4:
            hz.append(float("nan"))
            ln.append(float("nan"))
            continue
        mv = np.interp(ev, t, moving.astype(float)) > 0.5
        iv = np.diff(ev)
        good = mv[1:] & mv[:-1] & (iv > _INTERVAL_S[0]) & (iv < _INTERVAL_S[1])
        if good.sum() < 3:
            hz.append(float("nan"))
            ln.append(float("nan"))
            continue
        hz.append(1.0 / float(np.median(iv[good])))
        if planar is None:
            ln.append(float("nan"))
        else:
            p = np.stack([np.interp(ev, t, planar[:, k]) for k in range(2)], 1)
            d = np.linalg.norm(np.diff(p, axis=0), axis=1)
            ln.append(float(np.median(d[good])))
        n += int(good.sum())
    return Strides(
        stride_hz=float(np.nanmean(hz)) if np.isfinite(hz).any() else float("nan"),
        stride_len=float(np.nanmean(ln)) if np.isfinite(ln).any() else float("nan"),
        stride_hz_leg=tuple(hz),
        stride_len_leg=tuple(ln),
        n_strides=n,
    )


def real_strides(st: Streams, *, start: float = 6.0, seconds: float | None = None) -> Strides:
    """A recording's strides: joints + IMU attitude at 500 Hz, tracker travel.

    Tracker-less recordings measure the cadence only (lengths NaN).
    """
    from dimos.robot.unitree.go2.sim.sysid.ingest import TRACKER_Z, mount_matrix
    from dimos.robot.unitree.go2.sim.sysid.real import cmd_at

    seconds = float(st.wt[-1]) - start if seconds is None else seconds
    sel = (st.lt >= start) & (st.lt < start + seconds)
    t, q, quat = st.lt[sel], st.lq[sel], st.lquat[sel]
    moving = np.linalg.norm(cmd_at(st, t)[:, :2], axis=1) > 0.25
    planar = None
    if st.has_markers:
        base_p, _ = st.base_pose_room(mount_matrix(), TRACKER_Z)
        planar = np.stack([np.interp(t, st.vt, base_p[:, k]) for k in range(2)], 1)
    return strides(t, q, quat, planar, moving)
