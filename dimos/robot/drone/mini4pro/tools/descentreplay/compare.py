#!/usr/bin/env python3
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

"""Compare the descent law's would-be commands against the human's recorded sticks.

    tools/descentreplay/run --compare <out.jsonl> [--deadband 0.05] [--max-lag 3.0]

Reads the jsonl `tools/descentreplay/run` produced — `cmd` lines (the compiled law) and
`human` lines (the recorded RC sticks through the compiled `StickMapping`, rotated to
north/east) — resamples both onto a common 0.1 s grid over their overlap, and reports, per
axis:

  - **sign agreement**: of the grid points where both were commanding (above the deadband),
    the fraction where they pushed the same direction;
  - **Pearson correlation** over the overlap;
  - **lag**: the shift of the controller series against the human's that maximises the
    correlation — positive means the controller's command trails the human's.

Two honesty notes, printed with the numbers. The magnitudes are not comparable one-to-one:
the human's sticks are scaled by the Q1 envelope (full stick = 3 m/s) while DJI's real
full-stick response is faster, and the human flies accelerations where the law flies a
clamped proportional velocity — the *directions* and the *timing* are the evidence, the
amplitude ratio is not. And the human's vertical carries the whole descent while the law
descends only inside its cone, so `vd` agreement is expected to be the weakest of the three
whenever the pilot descends while off-centre — which is exactly the behaviour the cone
exists to forbid.
"""

import argparse
import json
import math
import sys


def load(path):
    cmds, humans = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            o = json.loads(line)
            k = o.get("k")
            if k == "cmd":
                cmds.append((o["t"], o["vn"], o["ve"], o["vd"]))
            elif k == "human":
                humans.append((o["t"], o["vn"], o["ve"], o["vd"]))
    return cmds, humans


def interp(series, t):
    """Linear interpolation of a (t, vn, ve, vd) series; None outside its span."""
    if not series or t < series[0][0] or t > series[-1][0]:
        return None
    lo, hi = 0, len(series) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    t0, *v0 = series[lo]
    t1, *v1 = series[hi]
    if t1 == t0:
        return v0
    a = (t - t0) / (t1 - t0)
    return [v0[i] + a * (v1[i] - v0[i]) for i in range(3)]


def pearson(xs, ys):
    n = len(xs)
    if n < 3:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys, strict=False))
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    if sxx <= 0 or syy <= 0:
        return None
    return sxy / math.sqrt(sxx * syy)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("jsonl")
    ap.add_argument(
        "--deadband",
        type=float,
        default=0.05,
        help="m/s below which an axis counts as 'not commanding' (default 0.05)",
    )
    ap.add_argument(
        "--max-lag", type=float, default=3.0, help="lag search half-window, seconds (default 3.0)"
    )
    ap.add_argument(
        "--grid", type=float, default=0.1, help="resampling step, seconds (default 0.1)"
    )
    args = ap.parse_args()

    cmds, humans = load(args.jsonl)
    if not cmds or not humans:
        print(f"nothing to compare: {len(cmds)} cmd lines, {len(humans)} human lines")
        return 1

    t0 = max(cmds[0][0], humans[0][0])
    t1 = min(cmds[-1][0], humans[-1][0])
    if t1 <= t0:
        print("the controller and human series do not overlap in time")
        return 1

    grid = []
    t = t0
    while t <= t1:
        c = interp(cmds, t)
        h = interp(humans, t)
        if c is not None and h is not None:
            grid.append((t, c, h))
        t += args.grid

    axes = ["vn", "ve", "vd"]
    print(
        f"overlap {t0:.1f}..{t1:.1f} s, {len(grid)} grid points at {args.grid} s "
        f"({len(cmds)} cmd, {len(humans)} human lines)"
    )
    print()
    print(
        f"{'axis':<5} {'both-active':>11} {'sign-agree':>10} {'pearson':>8} "
        f"{'best-lag':>9} {'r@lag':>7}"
    )

    for i, axis in enumerate(axes):
        both = [
            (c[i], h[i])
            for (_, c, h) in grid
            if abs(c[i]) > args.deadband and abs(h[i]) > args.deadband
        ]
        agree = sum(1 for c, h in both if (c > 0) == (h > 0)) / len(both) if both else None
        r0 = pearson([c[i] for (_, c, _h) in grid], [h[i] for (_, _c, h) in grid])

        # Lag: shift the controller against the human and take the best correlation.
        best_lag, best_r = None, None
        steps = int(args.max_lag / args.grid)
        for s in range(-steps, steps + 1):
            lag = s * args.grid
            xs, ys = [], []
            for t, _c, h in grid:
                c = interp(cmds, t - lag)
                if c is None:
                    continue
                xs.append(c[i])
                ys.append(h[i])
            r = pearson(xs, ys)
            if r is not None and (best_r is None or r > best_r):
                best_r, best_lag = r, lag
        print(
            f"{axis:<5} {len(both):>11} "
            f"{('%.0f%%' % (agree * 100)) if agree is not None else '—':>10} "
            f"{(f'{r0:.3f}') if r0 is not None else '—':>8} "
            f"{(f'{best_lag:+.1f}s') if best_lag is not None else '—':>9} "
            f"{(f'{best_r:.3f}') if best_r is not None else '—':>7}"
        )

    print()
    print(
        "notes: magnitudes are not one-to-one (envelope-scaled sticks vs clamped proportional law);"
    )
    print(
        "       vd disagrees by design wherever the pilot descends while the lateral error "
        "is outside the cone."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
