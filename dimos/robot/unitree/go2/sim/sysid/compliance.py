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

"""Series compliance between the motor-side sensors and the world (README 9).

    python -m dimos.robot.unitree.go2.sim.sysid.compliance REC.mcap

The joint channels' blind spot (README 9): ``q``/``dq``/``tau_est`` all live motor-side, so a
compliance after the gear — leg-link flex, belt wind-up, pad squash — is
structurally invisible to every joint-channel instrument. This one closes
the loop through the tracker's POSITION (its attitude is retracted, README 6;
the IMU supplies all attitude here) with zero free parameters:

* **Vertical stiffness** — during a leg's ground dwell the foot is fixed,
  so rigid FK predicts the base height above it; the tracker measures the
  actual. The residual regressed on the leg's vertical foot force
  (Jacobian-transpose from measured ``tau_est``) is a deflection per
  newton. Within-stance demeaning first: every slow bias — room-frame
  tilt, the ``TRACKER_Z`` guess, mount yaw — is constant over one 0.3 s
  stance and drops out, which is what keeps README 6's trap (mount error
  masquerading as compliance) shut.

* **Kinematic excess** — with the foot fixed, rigid kinematics predicts
  the base planar speed from the leg sweep alone; the tracker measures the
  actual. Their difference regressed on the horizontal foot force
  (magnitudes only — frame-free) is the propulsion-axis signature: a
  series element stores deflection under load and releases it as body
  motion the encoders never see.

BOTH numbers are meaningful only against the SAME instrument run on a
rigid-plant rollout: MuJoCo's soft contact gives even a rigid simulation a
finite vertical compliance, and foot rolling plus stance slip bias the
kinematic gain on both sides. The claim is always real-minus-control.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass

import numpy as np

from dimos.robot.unitree.go2.sim.sysid.gait import LEGS, foot_base
from dimos.simulation.sysid.rotations import quat_to_mat

DWELL_MM = 15.0  # a foot within this of its height floor is treated as grounded
SMOOTH_S = 0.05  # velocity smoothing, seconds — same on 500 Hz and 50 Hz data
STANCE_S = (0.1, 0.6)  # dwell intervals outside a plausible stance are pauses
BOOTSTRAP = 200


def leg_jacobian(q3: np.ndarray, leg: int) -> np.ndarray:
    """3x3 ``d(foot_base)/d(q_leg)`` by finite differences on the FK."""
    eps = 1e-6
    q12 = np.zeros(12)
    q12[3 * leg : 3 * leg + 3] = q3
    base = foot_base(q12[None])[0, leg]
    out = np.zeros((3, 3))
    for j in range(3):
        qp = q12.copy()
        qp[3 * leg + j] += eps
        out[:, j] = (foot_base(qp[None])[0, leg] - base) / eps
    return out


def foot_forces(
    q: np.ndarray, tau: np.ndarray, rot: np.ndarray, leg: int, idx: np.ndarray
) -> np.ndarray:
    """Gravity-frame foot force at samples ``idx``: ``f = -(J^T)^-1 tau``."""
    out = np.full((len(idx), 3), np.nan)
    for k, i in enumerate(idx):
        jac = leg_jacobian(q[i, 3 * leg : 3 * leg + 3], leg)
        try:
            f_base = -np.linalg.solve(jac.T, tau[i, 3 * leg : 3 * leg + 3])
        except np.linalg.LinAlgError:
            continue
        out[k] = rot[i] @ f_base
    return out


def _dwell_intervals(m: np.ndarray, dt: float) -> list[np.ndarray]:
    d = np.diff(m.astype(int))
    starts = list(np.where(d == 1)[0] + 1)
    ends = list(np.where(d == -1)[0] + 1)
    if m[0]:
        starts = [0, *starts]
    if m[-1]:
        ends = [*ends, len(m)]
    return [
        np.arange(a, b)
        for a, b in zip(starts, ends, strict=True)
        if STANCE_S[0] <= (b - a) * dt <= STANCE_S[1]
    ]


def _smooth(x: np.ndarray, n: int) -> np.ndarray:
    return np.convolve(x, np.ones(n) / n, mode="same") if n > 1 else x


def _pooled_slope(ys: list[np.ndarray], xs: list[np.ndarray]) -> tuple[float, float, float, int]:
    """Within-stance-demeaned regression slope with a stance-level bootstrap CI.

    An unexcited regressor (a leg whose force never varies) yields NaN, not a
    warning: no excitation means no measurement, which is different from zero.
    """
    y = np.concatenate(ys)
    x = np.concatenate(xs)
    if float(x @ x) < 1e-12:
        return float("nan"), float("nan"), float("nan"), len(y)
    slope = float((x @ y) / (x @ x))
    rng = np.random.default_rng(0)
    draws = []
    for _ in range(BOOTSTRAP):
        pick = rng.integers(0, len(ys), len(ys))
        y2 = np.concatenate([ys[i] for i in pick])
        x2 = np.concatenate([xs[i] for i in pick])
        d2 = float(x2 @ x2)
        if d2 >= 1e-12:
            draws.append(float((x2 @ y2) / d2))
    if not draws:
        return slope, float("nan"), float("nan"), len(y)
    lo, hi = np.percentile(draws, [5, 95])
    return slope, float(lo), float(hi), len(y)


@dataclass(frozen=True)
class LegResult:
    """One leg's two regressions, each with a 90% stance-bootstrap CI."""

    dz_dfz: tuple[float, float, float]  # m/N: vertical deflection per newton
    dv_dfh: tuple[float, float, float]  # (m/s)/N: kinematic excess per newton
    v_bias: float  # m/s: mean within-stance (v_body - v_kin)
    n: int


def measure(
    t: np.ndarray,
    q: np.ndarray,
    quat: np.ndarray,
    planar: np.ndarray,
    base_z: np.ndarray,
    tau: np.ndarray,
    moving: np.ndarray,
    *,
    stride: int = 2,
) -> list[LegResult]:
    """Both instruments, per leg. ``stride`` subsamples the (slow) force solve."""
    dt = float(np.median(np.diff(t)))
    n = max(1, int(SMOOTH_S / dt))
    rot = quat_to_mat(quat)
    fw = np.einsum("nij,nkj->nki", rot, foot_base(q))
    v_body = _smooth(np.linalg.norm(np.diff(planar, axis=0), axis=1) / dt, n)
    out = []
    for leg in range(4):
        z = fw[:, leg, 2]
        dwell = (z < np.percentile(z[moving], 10) + DWELL_MM * 1e-3) & moving
        v_kin = _smooth(np.linalg.norm(np.diff(fw[:, leg, :2], axis=0), axis=1) / dt, n)
        dh_p, fz_p, dv_p, fh_p, biases = [], [], [], [], []
        for iv in _dwell_intervals(dwell, dt):
            iv = iv[:: max(1, stride)]
            iv = iv[iv < len(v_body)]
            if len(iv) < 5:
                continue
            f = foot_forces(q, tau, rot, leg, iv)
            ok = np.isfinite(f[:, 2])
            if ok.sum() < 5:
                continue
            delta_h = (base_z[iv] + z[iv])[ok]  # h_trk - h_kin, h_kin = -z
            f_z = f[ok, 2]
            dh_p.append(delta_h - delta_h.mean())
            fz_p.append(f_z - f_z.mean())
            dv = (v_body[iv] - v_kin[iv])[ok]
            f_h = np.linalg.norm(f[ok, :2], axis=1)
            dv_p.append(dv - dv.mean())
            fh_p.append(f_h - f_h.mean())
            biases.append(float(dv.mean()))
        if not dh_p:
            nan3 = (float("nan"),) * 3
            out.append(LegResult(nan3, nan3, float("nan"), 0))
            continue
        s1, lo1, hi1, npts = _pooled_slope(dh_p, fz_p)
        s2, lo2, hi2, _ = _pooled_slope(dv_p, fh_p)
        out.append(LegResult((s1, lo1, hi1), (s2, lo2, hi2), float(np.mean(biases)), npts))
    return out


def report(results: list[LegResult]) -> str:
    lines = []
    for leg, r in zip(LEGS, results, strict=True):
        z, v = r.dz_dfz, r.dv_dfh
        lines.append(
            f"{leg}: dz/dFz {z[0] * 1e6:+7.1f} um/N [{z[1] * 1e6:+.1f}, {z[2] * 1e6:+.1f}]"
            f"  dv/dFh {v[0] * 1e3:+7.3f} (mm/s)/N [{v[1] * 1e3:+.3f}, {v[2] * 1e3:+.3f}]"
            f"  v_bias {r.v_bias * 1e3:+6.1f} mm/s  n={r.n}"
        )
    return "\n".join(lines)


def main() -> None:
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams
    from dimos.robot.unitree.go2.sim.sysid.real import cmd_at

    ap = argparse.ArgumentParser(prog="go2.sim.sysid.compliance")
    ap.add_argument("recording")
    ap.add_argument("--start", type=float, default=6.0)
    args = ap.parse_args()
    st = read_streams(args.recording)
    span = float(st.wt[-1]) - args.start
    sel = (st.lt >= args.start) & (st.lt < args.start + span)
    t = st.lt[sel]
    moving = np.linalg.norm(cmd_at(st, t)[:, :2], axis=1) > 0.25
    base_p, _ = st.base_pose_room()
    pos = np.stack([np.interp(t, st.vt, base_p[:, k]) for k in range(3)], 1)
    res = measure(t, st.lq[sel], st.lquat[sel], pos[:, :2], pos[:, 2], st.ltau[sel], moving)
    print(report(res))


if __name__ == "__main__":
    main()
