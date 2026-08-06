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

"""One page comparing every odometry method across every recording.

    python -m dimos.mapping.combined_html <recording_dir> [<recording_dir> ...]

Metrics are not all computed the same way and the page says so per row rather
than presenting one tidy column. cuVSLAM and NeRF-VO are scored here, by one
aligner, from timestamped trajectories. RTAB-Map arrives as a plot export with no
timestamps, so its trajectory can be drawn but not re-scored; its own reported
RMSE is shown and attributed. Quietly recomputing it from resampled positions
would produce a number that looks comparable and is not.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np

from dimos.mapping.topdown_html import (
    COLOURS,
    LABELS,
    SCALE_FREE_METHODS,
    align,
    pointlio_trajectory,
    segment_error,
)

# Methods whose exported trajectory carries no timestamp, so it can be plotted
# against the reference but not scored by the shared aligner.
# RTAB-Map trajectories arrive without timestamps, so this page cannot align them.
UNSCORABLE = {"rtabmap", "rtabmap_online", "rtabmap_stereo_imu", "rtabmap_stereo_imu_online"}

EXTRA_COLOURS = {
    "rtabmap": "#ffb454",
    "rtabmap_online": "#a8742a",
    "orbslam3_stereo": "#5ee0c0",
    "orbslam3_stereo_inertial": "#2f9c86",
    "cuvslam_inertial": "#3d6ea8",
    "rtabmap_stereo_imu": "#ff7ab6",
    "rtabmap_stereo_imu_online": "#a8446f",
    "pointlio_corrected": "#4fd39a",
    "cuvslam_slam": "#8ac6ff",
}
# Icon vocabulary, per Jeff: 👀 stereo (always stereo IR), 👁 mono IR, 🌈 colour mono,
# ⏫ IMU, 🔁 loop closure, 🟢 pose graph, 📺 live, 📚 offline.
# Icons carry the configuration so the labels stay short; the page prints a key.
ICON_KEY = [
    ("\U0001f440", "stereo IR"),
    ("\U0001f441", "mono IR"),
    ("\U0001f308", "colour mono"),
    ("\u23eb", "IMU"),
    ("\U0001f501", "loop closure"),
    ("\U0001f7e2", "pose graph"),
    ("\U0001f4fa", "live"),
    ("\U0001f4da", "offline"),
]
EXTRA_LABELS = {
    "cuvslam": "\U0001f440 \U0001f4fa cuVSLAM",
    "cuvslam_inertial": "\U0001f440 \u23eb \U0001f4fa cuVSLAM",
    "cuvslam_slam": "\U0001f440 \U0001f501 \U0001f7e2 \U0001f4fa cuVSLAM",
    "orbslam3_stereo": "\U0001f440 \U0001f501 \U0001f7e2 \U0001f4fa ORB-SLAM3",
    "orbslam3_stereo_inertial": "\U0001f440 \u23eb \U0001f501 \U0001f7e2 \U0001f4fa ORB-SLAM3",
    "nerf_vo": "\U0001f308 \U0001f4fa NeRF-VO",
    "rtabmap": "\U0001f308 \U0001f501 \U0001f7e2 \U0001f4da RTAB-Map",
    "rtabmap_online": "\U0001f308 \U0001f501 \U0001f4fa RTAB-Map",
    "rtabmap_stereo_imu": "\U0001f440 \u23eb \U0001f501 \U0001f7e2 \U0001f4da RTAB-Map",
    "rtabmap_stereo_imu_online": "\U0001f440 \u23eb \U0001f501 \U0001f4fa RTAB-Map",
    "pointlio_corrected": "\U0001f7e2 Point-LIO",
}
# Reported by the RTAB-Map run rather than measured here.
# RTAB-Map drift %, supplied by the RTAB-Map runs (same metric: rmse / Point-LIO path
# length, Umeyama rigid, no scale). Their pipeline reproduces this page's existing
# colour+depth online column exactly, which is what makes the rest comparable.
# Cells whose number is real but does not mean what the column implies. Shown, not
# hidden -- but flagged, because on a plain heat map they would read as wins.
CAVEATED = {
    ("hotel", "rtabmap"),
    ("hotel", "rtabmap_online"),
    ("hotel", "rtabmap_stereo_imu"),
    ("hotel", "rtabmap_stereo_imu_online"),
    ("small_loop_right_night", "rtabmap"),
}
REPORTED_DRIFT_PCT: dict[str, dict[str, Any]] = {
    "sf_office_stairs": {
        "rtabmap": 0.43,
        "rtabmap_online": 0.97,
        "rtabmap_stereo_imu": 0.90,
        "rtabmap_stereo_imu_online": 3.43,
    },
    "sf_office1": {
        "rtabmap": 0.34,
        "rtabmap_online": 1.22,
        "rtabmap_stereo_imu": 0.20,
        "rtabmap_stereo_imu_online": 1.74,
    },
    "sf_office1_2": {
        "rtabmap": 0.56,
        "rtabmap_online": 2.37,
        "rtabmap_stereo_imu": 0.93,
        "rtabmap_stereo_imu_online": 4.82,
    },
    "outdoor_small_right_loop": {
        "rtabmap": 1.51,
        "rtabmap_online": 1.62,
        "rtabmap_stereo_imu": 1.30,
        "rtabmap_stereo_imu_online": 17.89,
        "note": (
            "colour+depth fragmented into 3115 of 3901 nodes (91% of the route), so its "
            "1.51/1.62% are mildly flattered. The stereo run is a single session at 105%."
        ),
    },
    # Both RTAB-Map hotel numbers are withheld rather than shown: Point-LIO is the
    # denominator and its hotel trajectory misses loop closure by 7.97 m over 161.7 m,
    # so these would be two disagreeing systems reported as one system's error.
    "hotel": {
        "rtabmap": 6.58,
        "rtabmap_online": 19.43,
        "rtabmap_stereo_imu": 9.92,
        "rtabmap_stereo_imu_online": 15.92,
        "note": (
            "offline numbers withheld. The Point-LIO reference drifts here -- it ends "
            "7.97 m from its start on a 161.7 m route, where stairs and sf_office1 close "
            "to 0.02-0.06 m -- so drift against it is two systems disagreeing rather "
            "than RTAB-Map error."
        ),
    },
    "small_loop_right_night": {
        "rtabmap": 0.54,
        "rtabmap_online": 20.49,
        "rtabmap_stereo_imu": 2.00,
        "rtabmap_stereo_imu_online": 8.63,
        "note": (
            "colour+depth offline is omitted: it reads 0.54% but fragmented into 28 "
            "sessions and only 46 m of the 470 m route survives export, so it scores "
            "10% of the recording. The stereo run at 2.00% covers the whole route."
        ),
    },
}

# Enough samples to pin the yaw without being swayed by any one stretch of the path.
YAW_ALIGN_SAMPLES = 400


def _resample(points: np.ndarray, count: int) -> np.ndarray:
    """Sample a path at `count` evenly spaced positions along its own ordering."""
    source = np.linspace(0.0, 1.0, len(points))
    target = np.linspace(0.0, 1.0, count)
    return np.column_stack([np.interp(target, source, points[:, axis]) for axis in (0, 1)])


def yaw_align(trajectory: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Rotate a trajectory about its own start point to best match the reference.

    RTAB-Map trajectories arrive without timestamps and in their own frame, usually at a
    different yaw, which makes them hard to compare against everything else by eye. There
    is no correspondence to fit a full transform against, but both cover the same route in
    the same order, so matching by normalised position along the path and solving for the
    single remaining degree of freedom -- yaw about the start -- is well posed. This moves
    the drawing only; nothing here is scored.
    """
    if len(trajectory) < 2 or len(reference) < 2:
        return trajectory
    moving = _resample(trajectory, YAW_ALIGN_SAMPLES) - trajectory[0]
    fixed = _resample(reference, YAW_ALIGN_SAMPLES) - reference[0]
    # Closed-form 2-D Procrustes restricted to rotation.
    cross = float(np.sum(moving[:, 0] * fixed[:, 1] - moving[:, 1] * fixed[:, 0]))
    dot = float(np.sum(moving[:, 0] * fixed[:, 0] + moving[:, 1] * fixed[:, 1]))
    angle = np.arctan2(cross, dot)
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    aligned: np.ndarray = (trajectory - trajectory[0]) @ rotation.T + reference[0]
    return aligned


def collect(recording_dir: Path) -> dict[str, Any]:
    name = recording_dir.name
    reference = pointlio_trajectory(recording_dir / f"{name}.db")
    reference_path = float(np.linalg.norm(np.diff(reference[:, 1:4], axis=0), axis=1).sum())
    series: dict[str, Any] = {
        "pointlio": {
            "xy": reference[:, 1:3].tolist(),
            "label": LABELS["pointlio"],
            "colour": COLOURS["pointlio"],
            "path_m": round(reference_path, 1),
        }
    }
    for path in sorted(recording_dir.glob("*_traj.npy")):
        method = path.stem.removesuffix("_traj")
        trajectory = np.load(path)
        entry: dict[str, Any] = {
            "label": EXTRA_LABELS.get(method, LABELS.get(method, method)),
            "colour": EXTRA_COLOURS.get(method, COLOURS.get(method, "#cccccc")),
            "scale_fitted": method in SCALE_FREE_METHODS,
        }
        if method in UNSCORABLE:
            entry["xy"] = yaw_align(trajectory[:, 1:3], reference[:, 1:3]).tolist()
            entry["scored_here"] = False
            reported = REPORTED_DRIFT_PCT.get(name, {}).get(method)
            if isinstance(reported, (int, float)):
                entry["drift_pct"] = reported
                entry["ate_rmse_m"] = round(reported / 100 * reference_path, 3)
                if (name, method) in CAVEATED:
                    entry["caveat"] = True
        else:
            scale_free = method in SCALE_FREE_METHODS
            fitted = align(trajectory, reference, scale_free)
            if not fitted:
                continue
            fitted.update(segment_error(trajectory, reference, scale_free))
            entry.update(fitted)
            entry["scored_here"] = True
        series[method] = entry

    return {
        "recording": name,
        "reference_path_m": round(reference_path, 1),
        "note": REPORTED_DRIFT_PCT.get(name, {}).get("note"),
        "series": series,
        "stats": json.loads((recording_dir / "stats.json").read_text())
        if (recording_dir / "stats.json").exists()
        else {},
    }


_TEMPLATE = """<!doctype html>
<meta charset="utf-8">
<title>D455 odometry comparison</title>
<style>
  :root {
    color-scheme: dark;
    --bg:#0f1116; --panel:#151922; --ink:#e8ecf4; --dim:#8b93a3;
    --line:#232833; --accent:#57a6ff; --warn:#ffb454; --bad:#ff6b6b;
  }
  * { box-sizing:border-box }
  body { margin:0; background:var(--bg); color:var(--ink);
         font:13px/1.65 ui-monospace, SFMono-Regular, Menlo, monospace }
  header { padding:34px 34px 18px; max-width:1500px; margin:0 auto }
  h1 { font-size:25px; margin:0 0 8px; letter-spacing:-0.4px }
  h2 { font-size:18px; margin:0 0 4px; letter-spacing:-0.2px }
  .wrap { max-width:1500px; margin:0 auto }
  .sub, .muted { color:var(--dim) }
  .note { color:var(--warn) }
  .fail { color:var(--bad) }

  .card { background:var(--panel); border:1px solid var(--line);
          border-radius:14px; margin:0 34px 22px; padding:20px 24px }
  .card.scroll { overflow-x:auto }
  .card > .cap { color:var(--dim); font-size:11px; letter-spacing:.11em;
                 text-transform:uppercase; margin-bottom:14px }
  .ckey { text-transform:none; letter-spacing:0; margin-left:16px; font-size:11px }
  .ckey b { font-weight:400 }
  .ckey b.best { color:#8de36f } .ckey b.note { color:var(--warn) }
  .chip { display:inline-block; padding:1px 7px; border-radius:4px; color:var(--ink);
          font-size:10px; margin-right:2px }
  #summary td.heat { color:#eef3fb; font-variant-numeric:tabular-nums;
                     box-shadow:inset 0 0 0 1px rgba(255,255,255,.06) }
  #summary td.heat sup { opacity:.75; margin-left:1px }
  /* a caveated cell must not read as a clean win */
  #summary td.heat.caveat { background-image:repeating-linear-gradient(45deg,
      rgba(0,0,0,0) 0 5px, rgba(255,255,255,.13) 5px 10px); opacity:.82 }
  #foot { margin:0 34px 22px; color:var(--dim); font-size:12px }
  #foot li { margin-bottom:5px }
  #summary tbody tr:hover td.heat { filter:brightness(1.25) }

  #summary { border-collapse:separate; border-spacing:4px 4px; font-size:12px;
             width:100%; margin:-4px }
  #summary th, #summary td { padding:9px 13px; text-align:right; white-space:nowrap }
  #summary thead th { color:var(--dim); font-weight:400; padding-bottom:12px;
                      border-bottom:1px solid var(--line); vertical-align:bottom }
  #summary tbody tr:hover td:first-child { color:#fff }
  #summary tbody td { padding:9px 13px; font-size:13px; border-radius:6px }
  #summary tbody td:first-child { font-size:13px; padding-right:22px }
  #summary td:first-child, #summary th:first-child { text-align:left }
  #summary .best { color:#8de36f }

  /* controls stay reachable while scrolling the maps */
  #controls { position:sticky; top:0; z-index:5; background:var(--panel);
              box-shadow:0 8px 18px -10px rgba(0,0,0,.85) }
  #key { display:flex; gap:16px; flex-wrap:wrap; color:var(--dim); font-size:12px;
         margin-bottom:12px; padding-bottom:11px; border-bottom:1px solid var(--line) }
  #key b { color:var(--ink); font-weight:400; font-size:13px }
  #global .ttl { color:var(--dim); margin-bottom:10px }
  #global .cols { display:flex; gap:14px; flex-wrap:wrap; align-items:stretch }
  #global .col { display:flex; flex-direction:column; gap:5px;
                 border:1px solid var(--line); border-radius:11px; padding:12px 11px;
                 background:#11151d; flex:1 1 150px; max-width:215px }
  #global .colhead { color:var(--dim); font-size:11px; letter-spacing:.05em;
                     text-align:center; padding-bottom:6px; margin-bottom:2px;
                     border-bottom:1px solid var(--line) }
  #global label { white-space:nowrap; border:1px solid transparent; border-radius:6px;
                  padding:4px 6px; transition:.12s; display:flex; align-items:center;
                  gap:7px }
  #global label input { margin:0; flex:0 0 auto }
  /* icons sit in a fixed box so every row lines up regardless of how many it has */
  #global label .icons { flex:1 1 auto; text-align:left; letter-spacing:2px;
                         font-size:14px; line-height:1 }
  #global label:hover { border-color:#3a4354 }
  #global label.on { border-color:currentColor; background:#1a2130 }
  #global label:hover { background:#1b2029 }
  #global .btn { cursor:pointer; color:var(--dim); border:1px dashed var(--line);
                 border-radius:999px; padding:3px 11px; background:none }
  #global .btn:hover { color:var(--ink) }

  #plots.grid { display:grid; gap:22px; padding:0 34px 34px;
                grid-template-columns:repeat(auto-fit, minmax(430px, 1fr)) }
  #plots.grid > .card { margin:0 }
  .rec { display:flex; flex-direction:column; align-items:stretch; gap:0 }
  .rec > h2 { text-align:center; margin:4px 0 5px }
  .rec > .sub { text-align:center; margin-bottom:20px }
  .rec > svg { margin-bottom:10px }
  .rec > h2, .rec > .sub { align-self:center }
  svg { background:#0a0c10; border:1px solid var(--line); border-radius:10px;
        display:block; width:100%; height:auto; aspect-ratio:1 / 1 }
  .hint { fill:#39414f; font:13px ui-monospace, Menlo, monospace }

  /* the per-map numbers repeat for every recording, so they stay folded away */
  details.nums { width:100%; max-width:900px; margin-top:8px }
  details.nums > summary { cursor:pointer; color:var(--dim); font-size:12px;
                           list-style:none; padding:11px 0 9px; text-align:center;
                           border-top:1px dashed var(--line) }
  details.nums > summary::-webkit-details-marker { display:none }
  details.nums > summary:hover { color:var(--ink) }
  details.nums > summary::before { content:"▸ "; }
  details.nums[open] > summary::before { content:"▾ "; }

  table.legend { border-collapse:collapse; font-size:12px; width:100% }
  table.legend td, table.legend th { padding:4px 9px; text-align:right; white-space:nowrap }
  table.legend thead th { color:var(--dim); font-weight:400;
                          border-bottom:1px solid var(--line) }
  table.legend tbody tr:hover { background:#1b2029 }
  table.legend td:first-child, table.legend th:first-child { text-align:left; width:99% }
  .swatch { display:inline-block; width:10px; height:10px; border-radius:3px;
            margin-right:7px; vertical-align:-1px; flex:0 0 auto }
  #global label .swatch { margin-right:0 }
  label { cursor:pointer; user-select:none }
  .icons { letter-spacing:1px; margin-right:5px }
</style>
<header>
  <h1>D455 odometry comparison</h1>
  <div class="sub" id="sub"></div>
</header>
<div class="wrap"><div class="card" id="controls">
  <div class="cap">legend &amp; visibility</div>
  <div id="key"></div>
  <div id="global"></div>
</div>
<div class="card scroll">
  <div class="cap">drift % against Point-LIO
    <span class="ckey">whole table, log scale:
      <span class="chip" style="background:rgb(33,86,140)">0.2%</span>
      <span class="chip" style="background:rgb(54,79,116)"></span>
      <span class="chip" style="background:rgb(74,72,92)"></span>
      <span class="chip" style="background:rgb(113,58,69)"></span>
      <span class="chip" style="background:rgb(152,44,46)">20%</span>
    </span>
    <span class="ckey"><b>*</b> reported by that method, not scored here</span>
    <span class="ckey"><b>&dagger;</b> real number, not comparable &mdash; see footnotes</span>
    <span class="ckey"><b class="muted">&mdash;</b> not available</span>
  </div>
  <table id="summary"></table>
</div>
<ul id="foot">
  <li><b>&dagger; hotel, every RTAB-Map cell.</b> Point-LIO is the denominator and its
      hotel trajectory ends <b>7.97 m from its own start</b> on a 161.7 m route, where
      stairs and sf_office1 close to 0.02&ndash;0.06 m. These are two systems disagreeing,
      not RTAB-Map error. The whole hotel row should be read with that in mind.</li>
  <li><b>&dagger; night, RTAB-Map colour+depth offline (0.54%).</b> That run fragmented
      into 28 sessions and only <b>46 m of the 470 m route</b> survives export, so it is
      0.54% of the 10% it actually mapped. It would otherwise win the row on a stub. The
      comparable cell is the stereo run at 2.00%, which covers the full route.</li>
  <li><b>outdoor, RTAB-Map colour+depth.</b> Fragmented at 3115 of 3901 nodes (91% of
      route), so 1.51/1.62% are mildly flattered. Kept unflagged as the shortfall is small.</li>
</ul>
<p class="muted" style="margin:0 34px 18px">
  Drift is ATE / reference path length. <b>Scored here</b> means measured by this page's
  aligner from timestamped poses; <b>reported</b> means taken from that method's own run,
  because its exported trajectory carries no timestamps and cannot be re-scored the same way.
  Segment error re-fits every 10 m independently &mdash; it survives scale jumps and is the
  fairer comparison between a metric and a scale-free method. RTAB-Map paths are yaw-aligned
  to the reference about their start point so the shapes can be compared by eye.
</p>
<div id="plots" class="grid"></div></div>
<script>
const data = __PAYLOAD__
const ICON_KEY = __ICON_KEY__
const SIZE = 560, PAD = 30

function project(xy, box) {
    return xy.map(([x, y]) => [
        PAD + ((x - box.cx) / box.span + 0.5) * (SIZE - 2 * PAD),
        SIZE - PAD - ((y - box.cy) / box.span + 0.5) * (SIZE - 2 * PAD),
    ])
}
function bounds(series) {
    const all = Object.values(series).flatMap((s) => s.xy)
    const xs = all.map((p) => p[0]), ys = all.map((p) => p[1])
    const minX = Math.min(...xs), maxX = Math.max(...xs)
    const minY = Math.min(...ys), maxY = Math.max(...ys)
    return { cx: (minX + maxX) / 2, cy: (minY + maxY) / 2,
             span: (Math.max(maxX - minX, maxY - minY) || 1) * 1.08 }
}

const fmt = (v, unit) => (v === undefined || v === null) ? "&mdash;" : v + (unit || "")

// summary table across every recording. Derived from the data rather than hard
// coded, otherwise a newly added method plots but silently misses this table.
const methods = (() => {
    const seen = []
    for (const rec of data) {
        for (const name of Object.keys(rec.series)) {
            // Point-LIO and its corrected pose graph are the reference, not competitors.
            // Scoring the corrected one against the raw one measures the reference's own
            // drift, which does not belong in a column of tracker results.
            if (!name.startsWith("pointlio") && !seen.includes(name)) { seen.push(name) }
        }
    }
    return seen
})()
const labelOf = (m) => {
    const any = data.find((rec) => rec.series[m])
    return any ? any.series[m].label : m
}
const colourOf = (m) => {
    const any = data.find((rec) => rec.series[m])
    return any ? any.series[m].colour : "#ccc"
}
// icons alone in the header; the full label is the tooltip
let head = `<thead><tr><th>recording</th><th>ref path</th>`
for (const m of methods) {
    const lab = labelOf(m)
    const words = lab.split(" ")
    const isName = (w) => /[A-Za-z]/.test(w)
    const icons = words.filter((w) => !isName(w)).join(" ")
    const nm = words.filter(isName).join(" ")
    head += `<th title="${lab}"><span class="icons">${icons}</span><br>` +
            `<span class="swatch" style="background:${colourOf(m)}"></span>${nm}</th>`
}
head += "</tr></thead>"
// Normalised across the WHOLE table, not per row, so a colour means the same thing
// everywhere and recordings can be compared against each other as well as trackers.
// The scale is logarithmic because the values span two orders of magnitude
// (0.20% to 20.49%); a linear ramp would put almost everything at the blue end.
// Interpolated through RGB rather than around the hue wheel, so the ramp never
// passes through green or yellow. Neutral slate in the middle keeps it from going muddy.
const HEAT_STOPS = [[33, 86, 140], [74, 72, 92], [152, 44, 46]]
const heat = (value, low, high) => {
    const t = high > low ? Math.min(1, Math.max(0, (value - low) / (high - low))) : 0
    const span = 1 / (HEAT_STOPS.length - 1)
    const i = Math.min(HEAT_STOPS.length - 2, Math.floor(t / span))
    const f = (t - i * span) / span
    const c = HEAT_STOPS[i].map((v, k) => Math.round(v + (HEAT_STOPS[i + 1][k] - v) * f))
    return `rgb(${c[0]}, ${c[1]}, ${c[2]})`
}
const allValues = data.flatMap((rec) =>
    methods.map((m) => rec.series[m] && rec.series[m].drift_pct)
           .filter((v) => typeof v === "number"))
const lo = Math.log(Math.min(...allValues)), hi = Math.log(Math.max(...allValues))

let body = "<tbody>"
for (const rec of data) {
    body += `<tr><td>${rec.recording}</td><td class="muted">${rec.reference_path_m} m</td>`
    for (const m of methods) {
        const s = rec.series[m]
        if (!s || typeof s.drift_pct !== "number") {
            body += `<td class="muted">&mdash;</td>`
            continue
        }
        const marks = (s.scored_here ? "" : "*") + (s.caveat ? "\u2020" : "")
        const cls = "heat" + (s.caveat ? " caveat" : "")
        body += `<td class="${cls}" style="background:${heat(Math.log(s.drift_pct), lo, hi)}">` +
                `${fmt(s.drift_pct, "%")}${marks ? `<sup>${marks}</sup>` : ""}</td>`
    }
    body += "</tr>"
}
document.getElementById("summary").innerHTML = head + body + "</tbody>"
document.getElementById("sub").textContent =
    `${data.length} recordings, drift % against Point-LIO. Orange = reported by that method, not scored here.`

// one panel per recording
const plots = document.getElementById("plots")
for (const rec of data) {
    const box = bounds(rec.series)
    let paths = `<rect width="${SIZE}" height="${SIZE}" fill="#0c0e12"/>`
    for (const [name, s] of Object.entries(rec.series)) {
        const pts = project(s.xy, box)
        const d = pts.map((p, i) => (i ? "L" : "M") + p.map((v) => v.toFixed(1)).join(" ")).join("")
        paths += `<path d="${d}" fill="none" stroke="${s.colour}"
                   stroke-width="${name === "pointlio" ? 2.2 : 1.4}" opacity="0.9"
                   style="display:none" data-name="${name}"/>`
        paths += `<circle cx="${pts[0][0].toFixed(1)}" cy="${pts[0][1].toFixed(1)}" r="3"
                   fill="${s.colour}" style="display:none" data-name="${name}"/>`
    }
    let rows = `<tr><th>trajectory</th><th>ATE</th><th>drift</th><th>segment</th>
                <th>scale</th><th>source</th></tr>`
    for (const [name, s] of Object.entries(rec.series)) {
        if (name === "pointlio") {
            rows += `<tr><td><label><input type="checkbox" data-rec="${rec.recording}"
                     data-name="${name}"><span class="swatch"
                     style="background:${s.colour}"></span>${s.label}</label></td>
                     <td>&mdash;</td><td>&mdash;</td><td>&mdash;</td><td>&mdash;</td>
                     <td class="muted">reference</td></tr>`
            continue
        }
        rows += `<tr><td><label><input type="checkbox" data-rec="${rec.recording}"
                 data-name="${name}"><span class="swatch"
                 style="background:${s.colour}"></span>${s.label}</label></td>
                 <td>${fmt(s.ate_rmse_m, " m")}</td>
                 <td>${fmt(s.drift_pct, "%")}</td>
                 <td>${fmt(s.segment_ate_median_m, " m")}</td>
                 <td>${s.scale_fitted ? "x" + s.scale : "1.0"}</td>
                 <td class="${s.scored_here ? "muted" : "note"}">${
                     s.scored_here ? "scored here" : "reported"}</td></tr>`
    }
    const failed = rec.note
        ? `<p class="fail">RTAB-Map: ${rec.note}</p>` : ""
    plots.insertAdjacentHTML("beforeend", `
      <div class="card rec">
        <h2>${rec.recording}</h2>
        <div class="sub">reference path ${rec.reference_path_m} m</div>
        <svg viewBox="0 0 ${SIZE} ${SIZE}" preserveAspectRatio="xMidYMid meet"
             data-rec="${rec.recording}">${paths}
          <text class="hint" x="${SIZE / 2}" y="${SIZE / 2}" text-anchor="middle"
                data-hint="1">pick a trajectory above</text>
        </svg>
        ${failed}
        <details class="nums">
          <summary>per-trajectory numbers</summary>
          <table class="legend">${rows}</table>
        </details>
      </div>`)
}

document.getElementById("key").innerHTML =
    ICON_KEY.map(([icon, meaning]) => `<span><b>${icon}</b> ${meaning}</span>`).join("")

// one toggle per approach, applying across every map
const globalBar = document.getElementById("global")
const familyOf = (m) =>
    m.startsWith("pointlio") ? "Point-LIO"
    : m.startsWith("cuvslam") ? "cuVSLAM"
    : m.startsWith("orbslam3") ? "ORB-SLAM3"
    : m.startsWith("rtabmap") ? "RTAB-Map"
    : m.startsWith("nerf") ? "NeRF-VO" : "other"

const families = []
for (const m of ["pointlio", ...methods]) {
    const any = data.find((rec) => rec.series[m])
    if (!any) { continue }
    const fam = familyOf(m)
    let group = families.find((g) => g.name === fam)
    if (!group) { group = { name: fam, items: [] }; families.push(group) }
    group.items.push([m, any.series[m]])
}

let bar = `<div class="ttl">nothing is drawn until you pick something
           <button class="btn" id="pick-all">all</button>
           <button class="btn" id="pick-none">none</button></div><div class="cols">`
for (const group of families) {
    bar += `<div class="col"><div class="colhead">${group.name}</div>`
    for (const [m, s] of group.items) {
        // the tracker name is the column head, so the row only needs its icons
        const icons = s.label.split(" ").filter((w) => !/[A-Za-z]/.test(w)).join(" ")
        bar += `<label style="color:${s.colour}"><input type="checkbox" data-all="${m}">` +
               `<span class="swatch" style="background:${s.colour}"></span>` +
               `<span class="icons" style="color:var(--ink)">${icons}</span></label>`
    }
    bar += `</div>`
}
globalBar.innerHTML = bar + `</div>`

const setVisible = (name, on) => {
    for (const el of document.querySelectorAll(`svg [data-name="${name}"]`)) {
        el.style.display = on ? "" : "none"
    }
    refreshHints()
}
// the placeholder only shows while a map is genuinely empty
const refreshHints = () => {
    for (const svg of document.querySelectorAll("svg[data-rec]")) {
        const any = [...svg.querySelectorAll("path[data-name]")]
            .some((el) => el.style.display !== "none")
        const hint = svg.querySelector("[data-hint]")
        if (hint) { hint.style.display = any ? "none" : "" }
    }
}

const applyAll = (on) => {
    for (const cb of document.querySelectorAll("input[data-all]")) {
        cb.checked = on
        cb.closest("label").classList.toggle("on", on)
        setVisible(cb.dataset.all, on)
    }
    for (const box of document.querySelectorAll("input[data-rec]")) { box.checked = on }
}
document.getElementById("pick-all").onclick = () => applyAll(true)
document.getElementById("pick-none").onclick = () => applyAll(false)
refreshHints()

document.addEventListener("change", (e) => {
    const cb = e.target
    if (!cb.dataset) { return }
    if (cb.dataset.all) {
        cb.closest("label").classList.toggle("on", cb.checked)
        setVisible(cb.dataset.all, cb.checked)
        // keep the per-map boxes in step so the two controls never disagree
        for (const box of document.querySelectorAll(`input[data-name="${cb.dataset.all}"][data-rec]`)) {
            box.checked = cb.checked
        }
        return
    }
    if (!cb.dataset.rec) { return }
    const svg = document.querySelector(`svg[data-rec="${cb.dataset.rec}"]`)
    for (const el of svg.querySelectorAll(`[data-name="${cb.dataset.name}"]`)) {
        el.style.display = cb.checked ? "" : "none"
    }
})
</script>
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording_dir", type=Path, nargs="+")
    parser.add_argument("--out", type=Path, default=Path("/tmp/d455_comparison.html"))
    args = parser.parse_args()

    payload = []
    for directory in args.recording_dir:
        if not (directory / f"{directory.name}.db").exists():
            print(f"combined_html: no recording in {directory}", file=sys.stderr)
            continue
        payload.append(collect(directory))
    args.out.write_text(
        _TEMPLATE.replace("__PAYLOAD__", json.dumps(payload)).replace(
            "__ICON_KEY__", json.dumps(ICON_KEY)
        )
    )
    print(args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
