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

"""What can this recording actually measure? Sensitivity before fitting.

    python -m dimos.robot.unitree.go2.sim.sysid.identify REC.mcap --channel accel

A fit reports an argmin whether or not the data constrains one. This asks the
prior question: nudge each knob, replay, and see how far the prediction moves.
A knob the prediction ignores is not measured by this recording, and fitting
it produces a number with no evidence behind it — which is how
``trunk_inertia_scale`` once came to sit at its search ceiling.

Per knob, in increasing order of usefulness:

* **sensitivity** — ``||dx/dtheta_i||``, the central-difference Jacobian
  column norm. Near zero means blind.
* **degeneracy** — two knobs whose columns point the same way cannot be told
  apart by this data; only their combination can.
* **spectrum** — eigenvalues of ``J^T J``. The count above the noise is how
  many independent things the recording pins, and the smallest eigenvector
  names the combination it cannot.
* **resolution** — the fraction of the knob's PHYSICAL range needed to move
  the prediction by one residual RMS. Under 1 means the data resolves it.

Perturbations are a fraction of each knob's physical range
(:data:`~dimos.robot.unitree.go2.sim.ranges.KNOBS`), not of its value, so the
columns are comparable: a 5% step means 5% of what that knob could plausibly
be, for every knob. Log-scaled knobs step in log space for the same reason.

Every rollout in a Jacobian shares one clip schedule per segment — the
schedule is a pure function of ``(segment, window, seed, protect)`` — or the
clip boundaries would move between columns and the differences would measure
the schedule instead of the physics.

The recording is SAMPLED, not sliced: N seeded segments spread across it
(:func:`~dimos.robot.unitree.go2.sim.sysid.regimes.sample_segments`), because
the information is concentrated — one contiguous window swung a resolution
number 10x depending on where it ended — and the report says per segment
where the resolving power came from, so a recording that resolves 7 of 14
knobs is legible as "the robot never did anything hard here", not as a
failure of the method.
"""

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import numpy as np

from dimos.robot.unitree.go2.sim.backend import Backend
from dimos.robot.unitree.go2.sim.ranges import CONTACT_DEFAULTS, Knob, load_preset
from dimos.robot.unitree.go2.sim.sysid.ingest import Streams, read_declarations, read_streams
from dimos.robot.unitree.go2.sim.sysid.regimes import (
    Segment,
    protected,
    regimes,
    sample_segments,
)
from dimos.robot.unitree.go2.sim.sysid.replay import ReplayResult, replay

CHANNELS: tuple[str, ...] = ("joint", "rot", "pos", "accel", "gyro", "tau")


def nudge(
    values: Mapping[str, float], name: str, knob: Knob, frac: float, sign: int
) -> dict[str, float]:
    """A copy of ``values`` with one knob moved by ``frac`` of its range.

    The centre is the applied value if present, else the engine default we
    know (:data:`CONTACT_DEFAULTS` — no early preset carries a contact value,
    which is what keeps old rollouts identical). A knob with neither has no
    centre to differentiate around, and that is an error, not a guess.
    """
    value = float(values.get(name, CONTACT_DEFAULTS.get(name, float("nan"))))
    if not math.isfinite(value):
        raise KeyError(f"{name} has no value on this plant and no known default")
    if knob.log:
        step = frac * (math.log(knob.hi) - math.log(knob.lo))
        moved = math.exp(math.log(max(value, knob.lo)) + sign * step)
    else:
        moved = value + sign * frac * (knob.hi - knob.lo)
    return {**values, name: float(np.clip(moved, knob.lo, knob.hi))}


def _channel(r: ReplayResult, channel: str) -> np.ndarray:
    """The prediction whose movement we measure, flattened.

    WHICH CHANNEL MATTERS MORE THAN THE KNOBS. Trunk inertia hardly moves a
    joint angle — it moves the BODY, by counter-rotating it against the legs;
    scoring only ``joint`` reports it unidentifiable no matter how good the
    data is. ``accel`` is the trunk's specific force at the full physics rate,
    the quantity an impact actually lives in, and it resolves 12 of 14 knobs
    against ``joint``'s 4. ``gyro`` and ``tau`` have never been scored before
    this port — that is why they are here.
    """
    p = r.prediction
    if channel == "joint":
        return p.q.ravel()
    if channel == "rot":
        return p.body_rot.reshape(len(p.body_rot), 9).ravel()  # entries move ~1:1 with radians
    if channel == "pos":
        return p.body_pos.ravel()
    if channel == "accel":
        return p.imu_accel.ravel()
    if channel == "gyro":
        return p.imu_gyro.ravel()
    if channel == "tau":
        return p.tau.ravel()
    raise ValueError(f"unknown channel {channel!r}: expected one of {CHANNELS}")


def _residual(r: ReplayResult, channel: str) -> np.ndarray | None:
    """Base-plant residual (sim - real) on the channel; None without the real side."""
    p = r.prediction
    out: np.ndarray
    if channel == "joint":
        out = (p.q - r.q_real).ravel()
    elif channel == "rot":
        if r.r_real is None:
            return None
        out = (p.body_rot - r.r_real).reshape(len(p.body_rot), 9).ravel()
    elif channel == "pos":
        if r.p_real is None:
            return None
        out = (p.body_pos - r.p_real).ravel()
    elif channel == "accel":
        out = (p.imu_accel - r.a_real).ravel()
    elif channel == "gyro":
        out = (p.imu_gyro - r.w_real).ravel()
    elif channel == "tau":
        out = (p.tau - r.tau_real).ravel()
    else:
        raise ValueError(f"unknown channel {channel!r}: expected one of {CHANNELS}")
    return out


@dataclass(frozen=True)
class SegmentRows:
    segment: Segment
    rows: slice  # this segment's rows of J and residual


@dataclass(frozen=True)
class Sensitivity:
    """One Jacobian and everything needed to say what the data can see."""

    J: np.ndarray  # (rows, knobs) — d(prediction)/d(fraction of range)
    names: list[str]
    channel: str
    residual: np.ndarray  # (rows,) base-plant residual; NaN where unmeasured
    segments: list[SegmentRows]


def jacobian(
    st: Streams,
    segments: Sequence[Segment],
    backend: Backend,
    values: Mapping[str, float],
    *,
    frac: float = 0.05,
    window: float | tuple[float, float] | None = 0.4,
    seed: int = 0,
    params: Sequence[str] | None = None,
    channel: str = "joint",
    protect: np.ndarray | None = None,
    suspended: bool = False,
) -> Sensitivity:
    """Central-difference ``dx/dtheta`` over the sampled segments.

    Within a segment every rollout — both signs of every knob, and the base
    run the residual comes from — shares one clip schedule (pure function of
    the segment, ``window``, ``seed + segment index`` and ``protect``), or
    the differences would measure the schedule instead of the physics.
    """
    knobs = backend.knobs()
    names = list(params) if params is not None else list(knobs)
    missing = [n for n in names if n not in knobs]
    if missing:
        raise KeyError(f"backend {backend.name!r} exposes no knob(s) {missing}")

    def run(seg_i: int, seg: Segment, vals: Mapping[str, float]) -> ReplayResult:
        backend.apply(vals)
        return replay(
            st,
            seg.t0,
            seg.duration,
            backend,
            window=window,
            seed=seed + seg_i,
            protect=protect,
            suspended=suspended,
        )

    blocks: list[np.ndarray] = []
    residuals: list[np.ndarray] = []
    rows: list[SegmentRows] = []
    row0 = 0
    for seg_i, seg in enumerate(segments):
        base = run(seg_i, seg, values)
        x0 = _channel(base, channel)
        res = _residual(base, channel)
        cols = []
        for name in names:
            out = []
            for sign in (+1, -1):
                r = run(seg_i, seg, nudge(values, name, knobs[name], frac, sign))
                out.append(_channel(r, channel))
            n = min(len(out[0]), len(out[1]))
            cols.append((out[0][:n] - out[1][:n]) / (2.0 * frac))
        n = min(min(len(c) for c in cols), len(x0))
        blocks.append(np.stack([c[:n] for c in cols], axis=1))
        residuals.append(res[:n] if res is not None else np.full(n, np.nan))
        rows.append(SegmentRows(seg, slice(row0, row0 + n)))
        row0 += n
    backend.apply(dict(values))  # leave the backend as we found it
    return Sensitivity(
        J=np.concatenate(blocks, axis=0),
        names=names,
        channel=channel,
        residual=np.concatenate(residuals),
        segments=rows,
    )


def analyse(J: np.ndarray, names: list[str]) -> dict[str, Any]:
    """Sensitivity, pairwise degeneracy and the spectrum of ``J^T J``."""
    sens = np.linalg.norm(J, axis=0) / max(np.sqrt(J.shape[0]), 1.0)
    unit = J / np.where(np.linalg.norm(J, axis=0) > 0, np.linalg.norm(J, axis=0), 1.0)
    corr = unit.T @ unit
    evals, evecs = np.linalg.eigh(J.T @ J)
    order = evals[::-1]
    return {
        "sensitivity": sens,
        "corr": corr,
        "evals": order,
        "evecs": evecs[:, ::-1],
        "names": names,
        "condition": float(order[0] / order[-1]) if order[-1] > 0 else float("inf"),
    }


def resolution(sens: Sensitivity) -> np.ndarray:
    """Fraction of each knob's range that moves the prediction by one residual RMS.

    Under 1 means this recording resolves the knob on this channel. NaN means
    the channel has no measured side here (``pos``/``rot`` without a tracker),
    so resolvability is undefined however sensitive the prediction is.
    """
    res = sens.residual[np.isfinite(sens.residual)]
    if len(res) == 0:
        return np.full(len(sens.names), np.nan)
    rms = float(np.sqrt(np.mean(res**2)))
    s = np.linalg.norm(sens.J, axis=0) / max(np.sqrt(sens.J.shape[0]), 1.0)
    return np.where(s > 0, rms / np.where(s > 0, s, 1.0), np.inf)


def format_report(a: dict[str, Any]) -> str:
    names, sens, corr = a["names"], a["sensitivity"], a["corr"]
    evals, evecs = a["evals"], a["evecs"]
    top = sens.max() if sens.max() > 0 else 1.0
    out = [
        "SENSITIVITY  prediction movement per 100% of the knob's range",
        f"  {'knob':<26s} {'movement':>10s}  {'vs best':>8s}",
    ]
    for i in np.argsort(-sens):
        bar = "#" * round(30 * sens[i] / top)
        out.append(f"  {names[i]:<26s} {sens[i]:10.5f}  {100 * sens[i] / top:7.1f}%  {bar}")
    out.append("")
    out.append("DEGENERATE PAIRS  |cos| > 0.9 between columns: this data sees only the combination")
    pairs = [(abs(corr[i, j]), i, j) for i in range(len(names)) for j in range(i + 1, len(names))]
    strong = [p for p in sorted(pairs, reverse=True) if p[0] > 0.9]
    for c, i, j in strong:
        sign = "+" if corr[i, j] > 0 else "-"
        out.append(f"  {names[i]:<26s} {sign} {names[j]:<26s} cos {c:.3f}")
    out.extend(["  (none)"] if not strong else [])
    out.append("")
    tot = float(evals.sum()) or 1.0
    out.append(f"SPECTRUM of J^T J   condition number {a['condition']:.3g}")
    out.append(f"  {'#':>2s} {'eigenvalue':>12s} {'share':>7s}  dominant directions")
    for k in range(len(evals)):
        v = evecs[:, k]
        big = np.argsort(-np.abs(v))[:3]
        desc = "  ".join(f"{v[i]:+.2f}*{names[i]}" for i in big if abs(v[i]) > 0.15)
        out.append(f"  {k:2d} {evals[k]:12.4g} {100 * evals[k] / tot:6.2f}%  {desc}")
    return "\n".join(out)


def format_resolution(sens: Sensitivity) -> str:
    """Which knobs this recording pins, on this channel, as a countable claim."""
    r = resolution(sens)
    out = [
        "RESOLUTION  fraction of the knob's range that moves the prediction by",
        "one residual RMS — under 1 means this recording resolves the knob",
        f"  {'knob':<26s} {'resolution':>10s}",
    ]
    for i in np.argsort(r):
        tag = "" if not np.isfinite(r[i]) else ("  resolved" if r[i] < 1.0 else "")
        shown = f"{r[i]:10.3f}" if np.isfinite(r[i]) else f"{'n/a':>10s}"
        out.append(f"  {sens.names[i]:<26s} {shown}{tag}")
    n_ok = int(np.sum(r < 1.0)) if np.any(np.isfinite(r)) else 0
    if np.all(np.isnan(r)):
        out.append("  (no measured side on this channel — no tracker in this recording)")
    else:
        out.append(f"  resolves {n_ok} of {len(sens.names)}")
    return "\n".join(out)


def format_segments(sens: Sensitivity, st: Streams) -> str:
    """WHERE the information came from — the answer to "why only 7 of 14?".

    A segment's share of the Jacobian's energy against how hard the robot was
    actually working there. A recording whose resolving power all sits in one
    dynamic stretch resolved what it did because of that stretch; one that
    resolves little did so because the robot never did anything hard — which
    is capture guidance, not a failure of the method. Standing still
    contributes exactly nothing.
    """
    total = float(np.sum(sens.J**2)) or 1.0
    g0 = float(np.median(np.linalg.norm(st.lacc, axis=1))) if len(st.lacc) else 0.0
    out = [
        "INFORMATION BY SEGMENT  share of Jacobian energy vs how dynamic the robot was",
        f"  {'segment':<18s} {'J-share':>8s} {'accel dev p90':>14s} {'joint speed p90':>16s}",
    ]
    for sr in sens.segments:
        share = float(np.sum(sens.J[sr.rows] ** 2)) / total
        m = (st.lt >= sr.segment.t0) & (st.lt <= sr.segment.t1)
        if m.any():
            adev = float(np.percentile(np.abs(np.linalg.norm(st.lacc[m], axis=1) - g0), 90))
            jspd = float(np.percentile(np.abs(st.ldq[m]), 90))
        else:
            adev = jspd = float("nan")
        span = f"t={sr.segment.t0:6.1f}..{sr.segment.t1:6.1f}"
        bar = "#" * round(30 * share)
        out.append(f"  {span:<18s} {100 * share:7.1f}% {adev:14.2f} {jspd:16.2f}  {bar}")
    out.append(
        "  A near-zero share means the robot did nothing informative there; a"
        " recording that resolves few knobs needs harder maneuvers, not a better fit."
    )
    return "\n".join(out)


def main() -> None:
    ap = argparse.ArgumentParser(prog="go2.sim.sysid.identify")
    ap.add_argument("recording")
    ap.add_argument("--preset", default="measured")
    ap.add_argument("--channel", default="joint", help=" | ".join(CHANNELS))
    ap.add_argument(
        "--frac", type=float, default=0.05, help="perturbation, as a fraction of the knob's range"
    )
    ap.add_argument(
        "--segments", type=int, default=8, help="scored segments sampled across the recording"
    )
    ap.add_argument(
        "--segment-length",
        type=float,
        nargs=2,
        default=(4.0, 8.0),
        metavar=("LO", "HI"),
        help="segment length range, s",
    )
    ap.add_argument(
        "--window",
        type=float,
        nargs="+",
        default=[0.4],
        metavar="S",
        help="clip length, s: one value = fixed, two = U(lo, hi), 0 = never re-init",
    )
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--start", type=float, default=None, help="restrict to [start, start+seconds]")
    ap.add_argument("--seconds", type=float, default=None)
    args = ap.parse_args()

    from dimos.robot.unitree.go2.sim.model import MujocoBackend

    st = read_streams(args.recording)
    declared = read_declarations(args.recording)
    spans = regimes(st, declared)
    suspended = bool(declared.suspended)
    if suspended and args.channel == "accel":
        print(
            "WARNING: accel is dead on a suspended recording — pinning the trunk zeroes"
            " the signal being scored. Fit hanging files on the joint channel."
        )

    t_lo = max(float(st.lt[0]), float(st.ct[0]))
    t_hi = min(float(st.lt[-1]), float(st.ct[-1]))
    if args.start is not None:
        t_lo = max(t_lo, args.start)
    if args.seconds is not None:
        t_hi = min(t_hi, t_lo + args.seconds)
    seg_lo, seg_hi = float(args.segment_length[0]), float(args.segment_length[1])
    segs = sample_segments(t_lo, t_hi, n=args.segments, length=(seg_lo, seg_hi), seed=args.seed)

    window: float | tuple[float, float] | None
    if len(args.window) == 1:
        window = None if args.window[0] <= 0 else float(args.window[0])
    else:
        window = (float(args.window[0]), float(args.window[1]))

    preset = load_preset(args.preset)
    values = {**preset.physics, "actuator_tau": preset.actuator_tau}
    backend = MujocoBackend()
    print(
        f"{Path(args.recording).name}  {len(segs)} segments over t={t_lo:.1f}..{t_hi:.1f}s  "
        f"base {args.preset}  frac {args.frac}  channel {args.channel}"
    )
    sens = jacobian(
        st,
        segs,
        backend,
        values,
        frac=args.frac,
        window=window,
        seed=args.seed,
        channel=args.channel,
        protect=protected(spans),
        suspended=suspended,
    )
    print(f"{sens.J.shape[0]} residual samples x {sens.J.shape[1]} knobs\n")
    print(format_report(analyse(sens.J, sens.names)))
    print()
    print(format_resolution(sens))
    print()
    print(format_segments(sens, st))


if __name__ == "__main__":
    main()
