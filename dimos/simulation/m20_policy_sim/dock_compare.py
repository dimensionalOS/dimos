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

"""The dock detector against charge_manager on a vendor dock recording.

    cargo run --release --example dock_pcap -- CAPTURE.pcap > sweeps.jsonl   (in the controller's rust/)
    python -m dimos.simulation.m20_policy_sim.dock_compare sweeps.jsonl charge_manager.log

charge_manager logs its reflector detector's pose of the robot relative to the docked point every ~100 ms ("get Target
position" while undocking, "mInitYaw" while docking; x out m, y m, yaw rad; China Standard Time). Pairs each with the
sweep nearest it at the best-fitting log lag, fits the one unknown, base_link's distance from the posts when docked
(our x minus theirs), and prints the residuals.
"""

from __future__ import annotations

import datetime as dt
import json
import re
import statistics as st
import sys

CST = dt.timezone(dt.timedelta(hours=8))
# reported per this much distance out from the docked point (m); a charge_manager pose this far (m) across from ours is
# one of its glitches
BAND_M = 0.25
GLITCH_M = 0.1
POSE = re.compile(
    r"\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d)\.(\d{3})\.(\d{3})\].*(?:get Target position:|mInitYaw) "
    r"x:(-?[\d.]+), y:(-?[\d.]+), yaw:(-?[\d.]+)"
)


def vendor_poses(path: str) -> list[tuple[float, float, float, float]]:
    out = []
    for line in open(path, encoding="utf-8", errors="replace"):
        m = POSE.search(line)
        if m:
            t = dt.datetime.strptime(m[1], "%Y-%m-%d %H:%M:%S").replace(tzinfo=CST).timestamp()
            out.append(
                (t + int(m[2]) / 1e3 + int(m[3]) / 1e6, float(m[4]), float(m[5]), float(m[6]))
            )
    return out


def main() -> None:
    ours = [
        (r["t"], r["out"], r["y"], r["yaw"])
        for r in map(json.loads, open(sys.argv[1]))
        if r["found"]
    ]
    vendor = [v for v in vendor_poses(sys.argv[2]) if ours[0][0] <= v[0] <= ours[-1][0] + 0.5]
    print(f"{len(ours)} sweeps with the dock, {len(vendor)} charge_manager poses in their span")

    def nearest(t: float) -> tuple[float, float, float, float] | None:
        best = min(ours, key=lambda o: abs(o[0] - t))
        return best if abs(best[0] - t) < 0.06 else None

    best = None
    for lag_ms in range(-300, 301, 10):
        pairs = [(v, o) for v in vendor if (o := nearest(v[0] - lag_ms / 1e3))]
        if len(pairs) < 10:
            continue
        offset = st.median(o[1] - v[1] for v, o in pairs)
        err = st.mean(abs(o[1] - offset - v[1]) for v, o in pairs)
        if best is None or err < best[0]:
            best = (err, lag_ms, offset, pairs)
    if best is None:
        sys.exit("fewer than 10 poses pair with a sweep")
    _, lag, offset, pairs = best
    print(f"log lag {lag} ms after the sweep; docked: base_link {offset:.4f} m out from the posts")
    # charge_manager's own detector glitches now and then (a lone pose 0.5 m off): counted, kept out of the stats
    glitch = [abs(o[2] - v[2]) > GLITCH_M for v, o in pairs]
    print(
        f"{sum(glitch)} of {len(pairs)} charge_manager poses > {GLITCH_M} m off ours across (its glitches) left out"
    )
    bands: dict[float, list[tuple[float, float, float]]] = {}
    for (v, o), bad in zip(pairs, glitch, strict=True):
        if not bad:
            bands.setdefault(round(v[1] / BAND_M) * BAND_M, []).append(
                (o[1] - offset - v[1], o[2] - v[2], o[3] - v[3])
            )
    print(
        "out from docked (m): pairs; ours - charge_manager: x, y (mm), yaw (mrad) as median / MAD"
    )
    for band in sorted(bands):
        e = bands[band]
        cells = []
        for k, scale in ((0, 1e3), (1, 1e3), (2, 1e3)):
            values = [a[k] * scale for a in e]
            med = st.median(values)
            cells.append(f"{med:+6.1f} / {st.median(abs(x - med) for x in values):4.1f}")
        print(f"  {band:4.2f}: {len(e):3d}   x {cells[0]}   y {cells[1]}   yaw {cells[2]}")


if __name__ == "__main__":
    main()
