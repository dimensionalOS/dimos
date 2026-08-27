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

"""Measure the DRIVE, with no simulator in the loop.

    python -m dimos.robot.unitree.go2.sim.sysid.drive REC.mcap

The recordings carry both sides of the drive: the commands (``cq``, ``ckp``,
``ckd``, ``cdq``, ``ctau``) and the board's own state samples (``lq``,
``ldq``) fix the torque the PD law DEMANDED at every 500 Hz sample, and
``ltau`` (``tau_est``) is the torque the drive says it DELIVERED. The
transfer from one to the other IS the drive, measured on the real robot with
ZERO free parameters — the instrument README 9's latency-proxy verdict called for, aimed
at the three candidate mechanisms an ``action_latency`` could be standing in
for: drive dynamics beyond first order, backlash/deadband, and (by regime
contrast against the no-contact suspended recording) contact compliance.

Measured on the 2026-08-16 sessions (freewalk-hard walking + suspended):

* **No resonance.** Free second-order fits land at 37-57 Hz with damping
  ~0.8-1.1 — they mimic a lag, never a peak; |H| exceeds 1 nowhere in band.
  A drive that could overshoot and inject energy at gait frequency is ruled
  out.
* **No backlash/deadband.** At |dq| < 3 rad/s the delivered/demanded gain is
  0.95-1.03 in EVERY demand-amplitude bin down to 0-0.5 N·m (walking). The
  low-gain cells are all |dq| >= 3 — that is the measured torque envelope
  (:data:`~dimos.robot.unitree.go2.sim.plant.TORQUE_ENVELOPES`), already a
  named mechanism, not a new one.
* **The drive is first-order to within measurement error**: equivalent lag
  ~2-9 ms (walking: hips 4.9, thighs 7.4, calf 1.8 ms mean), delay and
  first-order fits indistinguishable, same order as the plant's
  ``actuator_tau`` (5.25 ms fitted). The walking time-domain residual
  optimum sits at a +8 ms command shift and decomposes as ~1.3 ms transport
  + ZOH + this lag — no hidden 12 ms anywhere in the drive.
* **The sensor is not late**: joint acceleration (which never touches
  ``tau_est``) correlates best with the demand at ~0 ms and with ``tau_est``
  at 0 ms on the suspended recording — torque acts on time, so the lag read
  off ``ltau`` is the drive's, not a reporting filter's.
* **In-band under-delivery is real**: thigh |H| dips to ~0.83 over 3-16 Hz
  where swing speeds pass 3 rad/s — the envelope acting at gait harmonics.
  A plant fitted WITHOUT the envelope absorbs that average deficit into its
  viscous/inertial knobs instead (see ``fit --envelope``).

CAVEAT the reader should keep: the demand is computed from the same measured
``q``/``dq`` the response feeds back into, so H(f) is a closed-loop estimate;
with the coherences seen here (0.85-1.0 in band while walking) the shape
survives, but treat third-digit magnitudes as soft. ``ltau`` is the SDK's
motor-side estimate; its scale rides on the gear-ratio constant, which
cancels out of every SHAPE statement above.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import itertools
from pathlib import Path

import numpy as np

from dimos.robot.unitree.go2.sim.plant import (
    MUJOCO_ACTUATOR_NAMES,
    TORQUE_LIMITS,
    pd_torque,
)
from dimos.robot.unitree.go2.sim.sysid.recording import Streams

# hip / thigh / calf per leg, MuJoCo actuator order.
JOINT_TYPES: tuple[str, ...] = tuple(n.split("_")[1] for n in MUJOCO_ACTUATOR_NAMES)

FS = 500.0  # the analysis grid; the lowstate clock is ~500 Hz by construction

# The dq threshold above which the measured torque envelope derates: gain
# read below it is the drive's own, gain above it belongs to the envelope.
ENVELOPE_DQ = 3.0


def demanded_torque(st: Streams, shift_s: float = 0.0, *, clip: bool = True) -> np.ndarray:
    """The torque the board's PD law asked for, at every lowstate sample.

    Commands are zero-order held (the board applies the latest one), the law
    is :func:`~dimos.robot.unitree.go2.sim.plant.pd_torque` on the SAME
    ``q``/``dq`` samples the board used, and the result is clipped to
    :data:`~dimos.robot.unitree.go2.sim.plant.TORQUE_LIMITS`. ``shift_s``
    delays the command timeline — the open-loop knob
    :func:`residual_shift_sweep` turns.
    """
    idx = np.clip(np.searchsorted(st.ct + shift_s, st.lt, "right") - 1, 0, len(st.ct) - 1)
    tau = pd_torque(st.cq[idx], st.cdq[idx], st.ckp[idx], st.ckd[idx], st.ctau[idx], st.lq, st.ldq)
    return np.clip(tau, -TORQUE_LIMITS, TORQUE_LIMITS) if clip else tau


def _window(st: Streams, t0: float, t1: float) -> np.ndarray:
    sel = (st.lt >= t0) & (st.lt < t1)
    if sel.sum() < 1000:
        raise ValueError(f"drive: only {sel.sum()} lowstate samples in [{t0}, {t1})")
    return sel


def _uniform(st: Streams, sel: np.ndarray, x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Columns of ``x`` resampled onto a uniform ``FS`` grid inside ``sel``."""
    t = st.lt[sel]
    grid = np.arange(t[0], t[-1], 1.0 / FS)
    return grid, np.stack([np.interp(grid, t, x[sel][:, j]) for j in range(x.shape[1])], 1)


def residual_shift_sweep(
    st: Streams, *, t0: float, t1: float, shifts_ms: tuple[float, ...]
) -> list[tuple[float, float]]:
    """RMS of ``ltau - demand`` per command shift: where the delivered torque
    sits in time. A first-order lag reads as its own time constant here, so
    the optimum BOUNDS delay + lag together; the transfer function separates
    them (as far as this band can — in it they are near-indistinguishable)."""
    sel = _window(st, t0, t1)
    out = []
    for ms in shifts_ms:
        d = demanded_torque(st, ms / 1e3)
        out.append((float(ms), float(np.sqrt(np.mean((st.ltau[sel] - d[sel]) ** 2)))))
    return out


@dataclass(frozen=True)
class ModelFit:
    """One joint's best delay / first-order / second-order explanation of H(f).

    Errors are coherence-weighted RMS of the complex misfit; comparable only
    within one joint. ``fn_hz``/``zeta`` describe the free second-order fit —
    a resonance claim needs ``fn_hz`` INSIDE the fitted band with ``zeta``
    well under 1 AND a clearly lower error, not merely the lowest of three.
    """

    joint: int
    err_delay: float
    gain_delay: float
    delay_s: float
    err_lag: float
    gain_lag: float
    lag_s: float
    err_2nd: float
    gain_2nd: float
    fn_hz: float
    zeta: float


@dataclass(frozen=True)
class DriveTF:
    """Per-joint drive transfer function ``demand -> tau_est`` with coherence."""

    f: np.ndarray  # (nf,)
    H: np.ndarray  # (nf, 12) complex
    coherence: np.ndarray  # (nf, 12)

    @classmethod
    def measure(
        cls, st: Streams, *, t0: float, t1: float, shift_s: float = 0.0, nperseg: int = 4096
    ) -> DriveTF:
        from scipy import signal

        sel = _window(st, t0, t1)
        _, X = _uniform(st, sel, demanded_torque(st, shift_s))
        _, Y = _uniform(st, sel, st.ltau)
        Hs, cohs = [], []
        f = np.zeros(0)
        for j in range(12):
            f, pxy = signal.csd(X[:, j], Y[:, j], fs=FS, nperseg=nperseg)
            _, pxx = signal.welch(X[:, j], fs=FS, nperseg=nperseg)
            _, pyy = signal.welch(Y[:, j], fs=FS, nperseg=nperseg)
            Hs.append(pxy / pxx)
            cohs.append(np.abs(pxy) ** 2 / (pxx * pyy))
        return cls(f=f, H=np.stack(Hs, 1), coherence=np.stack(cohs, 1))

    def model_fits(self, *, fmin: float = 1.0, fmax: float = 20.0) -> list[ModelFit]:
        """Grid-fit the three candidate drive models to each joint's H(f)."""
        m = (self.f >= fmin) & (self.f <= fmax)
        fm = self.f[m]
        s = 2j * np.pi * fm
        gains = np.arange(0.70, 1.16, 0.02)
        delays = np.arange(0.0, 0.0205, 0.0005)
        lags = np.arange(0.0005, 0.0305, 0.0005)
        fns = np.arange(5.0, 62.5, 2.5)
        zetas = np.arange(0.2, 2.05, 0.1)
        out = []
        for j in range(12):
            Hm, w = self.H[m, j], self.coherence[m, j]

            def err(model: np.ndarray, Hm: np.ndarray = Hm, w: np.ndarray = w) -> float:
                return float(np.sqrt(np.sum(w * np.abs(Hm - model) ** 2) / np.sum(w)))

            e_d, g_d, td = min(
                (err(g * np.exp(-2j * np.pi * fm * t)), g, t) for t in delays for g in gains
            )
            e_1, g_1, t1_ = min((err(g / (1 + s * t)), g, t) for t in lags for g in gains)
            e_2, g_2, fn, z = min(
                (err(g * wn**2 / (s**2 + 2 * z * wn * s + wn**2)), g, fn, z)
                for fn in fns
                for z in zetas
                for wn in (2 * np.pi * fn,)
                for g in gains
            )
            out.append(
                ModelFit(
                    joint=j,
                    err_delay=e_d,
                    gain_delay=float(g_d),
                    delay_s=float(td),
                    err_lag=e_1,
                    gain_lag=float(g_1),
                    lag_s=float(t1_),
                    err_2nd=e_2,
                    gain_2nd=float(g_2),
                    fn_hz=float(fn),
                    zeta=float(z),
                )
            )
        return out


@dataclass(frozen=True)
class GainCell:
    """Delivered/demanded gain in one (|demand|, |dq|) cell — the deadband
    detector: backlash shows as gain FALLING toward small demands at LOW
    speed; the envelope owns whatever happens above ``ENVELOPE_DQ``."""

    demand_lo: float
    demand_hi: float
    fast: bool  # |dq| >= ENVELOPE_DQ
    gain: float
    n: int


def amplitude_gain(
    st: Streams,
    *,
    t0: float,
    t1: float,
    shift_s: float = 0.0,
    bins: tuple[float, ...] = (0.0, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 35.0),
    min_n: int = 500,
) -> list[GainCell]:
    """Least-squares gain ``ltau ~ g * demand`` per amplitude and speed cell."""
    sel = _window(st, t0, t1)
    d = demanded_torque(st, shift_s)[sel]
    y = st.ltau[sel]
    speed = np.abs(st.ldq[sel])
    out = []
    for lo, hi in itertools.pairwise(bins):
        for fast in (False, True):
            m = (np.abs(d) >= lo) & (np.abs(d) < hi) & ((speed >= ENVELOPE_DQ) == fast)
            if m.sum() < min_n:
                continue
            g = float(np.sum(d[m] * y[m]) / np.sum(d[m] ** 2))
            out.append(GainCell(lo, hi, fast, g, int(m.sum())))
    return out


def torque_timing(
    st: Streams,
    *,
    t0: float,
    t1: float,
    joints: tuple[int, ...] = (1, 4, 7, 10),
    shifts_ms: tuple[float, ...] = (-10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0),
) -> dict[str, list[tuple[float, float]]]:
    """WHEN torque acts, via a signal that never touches ``tau_est``.

    Joint acceleration responds to the torque the drive actually produced,
    so its correlation against the shifted demand places the true actuation
    time, and against shifted ``tau_est`` says whether the ESTIMATE is
    reported late. Cleanest on a suspended recording (thigh joints swing
    freely); in stance the contact forces dominate ddq and flatten it.
    """
    from scipy import signal

    sel = _window(st, t0, t1)
    grid, dq = _uniform(st, sel, st.ldq)
    dt = float(grid[1] - grid[0])
    ddq = signal.savgol_filter(dq, 25, 3, deriv=1, delta=dt, axis=0)
    _, tau = _uniform(st, sel, st.ltau)
    out: dict[str, list[tuple[float, float]]] = {"demand": [], "tau_est": []}
    for ms in shifts_ms:
        d = demanded_torque(st, ms / 1e3)
        _, D = _uniform(st, sel, d)
        rs = [float(np.corrcoef(D[:, j], ddq[:, j])[0, 1]) for j in joints]
        out["demand"].append((float(ms), float(np.mean(rs))))
        k = round(ms / 1e3 / dt)
        rs = [
            float(np.corrcoef(np.roll(tau[:, j], k)[50:-50], ddq[50:-50, j])[0, 1]) for j in joints
        ]
        out["tau_est"].append((float(ms), float(np.mean(rs))))
    return out


def _type_mean(vals: list[float]) -> dict[str, float]:
    return {
        t: float(np.mean([v for v, jt in zip(vals, JOINT_TYPES, strict=True) if jt == t]))
        for t in ("hip", "thigh", "calf")
    }


def main() -> None:
    ap = argparse.ArgumentParser(prog="go2.sim.sysid.drive", description=__doc__)
    ap.add_argument("recording")
    ap.add_argument("--t0", type=float, default=6.0)
    ap.add_argument("--t1", type=float, default=None)
    args = ap.parse_args()

    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams

    st = read_streams(args.recording)
    t1 = float(st.lt[-1]) if args.t1 is None else args.t1
    print(f"DRIVE  {Path(args.recording).name}  t={args.t0:g}..{t1:g}s")

    pts = residual_shift_sweep(
        st, t0=args.t0, t1=t1, shifts_ms=(-10, -5, -2, 0, 2, 5, 8, 10, 12, 15, 20)
    )
    best = min(pts, key=lambda p: p[1])
    print(
        "shift sweep (delay + lag together): "
        + "  ".join(f"{ms:+g}:{r:.2f}" for ms, r in pts)
        + f"  -> optimum {best[0]:+g} ms"
    )

    tf = DriveTF.measure(st, t0=args.t0, t1=t1)
    fits = tf.model_fits()
    print("model fits, 1-20 Hz (err = coherence-weighted complex misfit):")
    for mf in fits:
        res = " RESONANT?" if mf.fn_hz < 20.0 and mf.zeta < 0.7 else ""
        print(
            f"  j{mf.joint:02d} {JOINT_TYPES[mf.joint]:>5}: "
            f"delay e={mf.err_delay:.3f} (g={mf.gain_delay:.2f}, {mf.delay_s * 1e3:4.1f} ms) | "
            f"lag e={mf.err_lag:.3f} (g={mf.gain_lag:.2f}, {mf.lag_s * 1e3:4.1f} ms) | "
            f"2nd e={mf.err_2nd:.3f} (fn={mf.fn_hz:g} Hz, z={mf.zeta:.1f}){res}"
        )
    lag = _type_mean([mf.lag_s * 1e3 for mf in fits])
    print(
        f"equivalent first-order lag: hip {lag['hip']:.1f} / thigh {lag['thigh']:.1f} / "
        f"calf {lag['calf']:.1f} ms"
    )

    print("amplitude-binned gain (deadband detector; fast cells belong to the envelope):")
    for c in amplitude_gain(st, t0=args.t0, t1=t1, shift_s=best[0] / 1e3):
        tag = "dq>=3 (envelope)" if c.fast else "dq<3"
        print(
            f"  |demand| {c.demand_lo:4g}-{c.demand_hi:<4g} {tag:<17} gain {c.gain:5.3f}  n {c.n}"
        )

    print("torque timing via ddq (never touches tau_est):")
    timing = torque_timing(st, t0=args.t0, t1=t1)
    for name, series in timing.items():
        peak = max(series, key=lambda p: p[1])
        row = "  ".join(f"{ms:+g}:{r:+.3f}" for ms, r in series)
        print(f"  ddq vs {name:<8} {row}  -> peak {peak[0]:+g} ms")


if __name__ == "__main__":
    main()
