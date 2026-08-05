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
UNSCORABLE = {"rtabmap", "rtabmap_online"}

EXTRA_COLOURS = {
    "rtabmap": "#ffb454",
    "rtabmap_online": "#a8742a",
    "orbslam3_stereo": "#5ee0c0",
    "orbslam3_stereo_inertial": "#2f9c86",
    "cuvslam_inertial": "#3d6ea8",
}
EXTRA_LABELS = {
    "rtabmap": "RTAB-Map (offline, pose graph optimised)",
    "rtabmap_online": "RTAB-Map (online, live pose)",
    "orbslam3_stereo": "ORB-SLAM3 (stereo)",
    "orbslam3_stereo_inertial": "ORB-SLAM3 (stereo-inertial)",
    "cuvslam": "cuVSLAM (stereo, live odometry)",
    "cuvslam_inertial": "cuVSLAM (stereo-inertial)",
}
# Reported by the RTAB-Map run rather than measured here.
REPORTED_RMSE = {
    "sf_office_stairs": {"rtabmap_online": 0.46, "rtabmap": 0.20},
    "sf_office1": {"rtabmap_online": 1.51, "rtabmap": 0.43},
    "sf_office1_2": {"rtabmap_online": 3.56, "rtabmap": 0.84},
    "outdoor_small_right_loop": {
        "rtabmap_online": 7.81,
        "rtabmap": 7.26,
        "note": "offline pose graph covers 3115 of 3901 poses (79.9%) -- fragmented",
    },
    # Not "failed" -- both produced output. Described by what the trajectory does,
    # measured here, because the source report carries no failure annotation and
    # inheriting the word would be repeating a claim rather than making one.
    "hotel": {
        "note": (
            "diverges: 158% of the reference path length. RTAB-Map also flags the "
            "Point-LIO reference here as unreliable -- but removing every reference "
            "pose implying over 5 m/s moves cuVSLAM 11.480 -> 11.481 m, so the poor "
            "scores are not explained by reference glitches."
        )
    },
    "small_loop_right_night": {
        "note": (
            "offline pose graph covers only 1719 of 5558 poses (46 m of a 467 m loop). "
            "Its Point-LIO reference has 895 poses implying over 5 m/s; excluding them "
            "moves cuVSLAM 44.089 -> 44.141 m, so again not a reference artefact."
        )
    },
}


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
            entry["xy"] = trajectory[:, 1:3].tolist()
            entry["scored_here"] = False
            reported = REPORTED_RMSE.get(name, {}).get(method)
            if reported is not None:
                entry["ate_rmse_m"] = reported
                entry["drift_pct"] = round(reported / max(reference_path, 1e-6) * 100, 2)
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
        "note": REPORTED_RMSE.get(name, {}).get("note"),
        "series": series,
        "stats": json.loads((recording_dir / "stats.json").read_text())
        if (recording_dir / "stats.json").exists()
        else {},
    }


_TEMPLATE = """<!doctype html>
<meta charset="utf-8">
<title>D455 odometry comparison</title>
<style>
  :root { color-scheme: dark }
  body { margin:0; background:#111318; color:#e8e8ee;
         font:13px/1.55 ui-monospace, SFMono-Regular, Menlo, monospace }
  header { padding:18px 22px 4px }
  h1 { font-size:17px; margin:0 0 4px }
  h2 { font-size:14px; margin:0 0 2px }
  .sub, .muted { color:#8b93a3 }
  .note { color:#ffb454 }
  .fail { color:#ff6b6b }
  #summary { margin:10px 22px 4px; border-collapse:collapse; font-size:12px }
  #summary th, #summary td { padding:4px 12px 4px 0; text-align:right; white-space:nowrap }
  #summary th { color:#8b93a3 }
  #summary td:first-child, #summary th:first-child { text-align:left }
  .rec { display:flex; gap:20px; padding:14px 22px; align-items:flex-start;
         border-top:1px solid #232833; flex-wrap:wrap }
  svg { background:#0c0e12; border:1px solid #232833; border-radius:6px }
  table.legend { border-collapse:collapse; font-size:12px }
  table.legend td, table.legend th { padding:3px 10px 3px 0; text-align:right; white-space:nowrap }
  table.legend th { color:#8b93a3 }
  table.legend td:first-child, table.legend th:first-child { text-align:left }
  .swatch { display:inline-block; width:9px; height:9px; border-radius:2px; margin-right:6px }
  label { cursor:pointer; user-select:none }
</style>
<header>
  <h1>D455 odometry comparison</h1>
  <div class="sub" id="sub"></div>
</header>
<table id="summary"></table>
<p class="muted" style="margin:0 22px 8px">
  Drift is ATE / reference path length. <b>Scored here</b> means measured by this page's
  aligner from timestamped poses; <b>reported</b> means taken from that method's own run,
  because its exported trajectory carries no timestamps and cannot be re-scored the same way.
  Segment error re-fits every 10 m independently &mdash; it survives scale jumps and is the
  fairer comparison between a metric and a scale-free method.
</p>
<div id="plots"></div>
<script>
const data = __PAYLOAD__
const SIZE = 460, PAD = 26

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

// summary table across every recording
const methods = ["cuvslam", "nerf_vo", "rtabmap", "rtabmap_online"]
let head = "<tr><th>recording</th><th>ref path</th>"
for (const m of methods) { head += `<th>${m}</th>` }
head += "</tr>"
let body = ""
for (const rec of data) {
    body += `<tr><td>${rec.recording}</td><td>${rec.reference_path_m} m</td>`
    for (const m of methods) {
        const s = rec.series[m]
        if (!s) { body += `<td class="muted">&mdash;</td>`; continue }
        const cls = s.scored_here ? "" : "note"
        body += `<td class="${cls}">${fmt(s.drift_pct, "%")}</td>`
    }
    body += "</tr>"
}
document.getElementById("summary").innerHTML = head + body
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
                   data-name="${name}"/>`
        paths += `<circle cx="${pts[0][0].toFixed(1)}" cy="${pts[0][1].toFixed(1)}" r="3"
                   fill="${s.colour}" data-name="${name}"/>`
    }
    let rows = `<tr><th>trajectory</th><th>ATE</th><th>drift</th><th>segment</th>
                <th>scale</th><th>source</th></tr>`
    for (const [name, s] of Object.entries(rec.series)) {
        if (name === "pointlio") {
            rows += `<tr><td><label><input type="checkbox" data-rec="${rec.recording}"
                     data-name="${name}" checked><span class="swatch"
                     style="background:${s.colour}"></span>${s.label}</label></td>
                     <td>&mdash;</td><td>&mdash;</td><td>&mdash;</td><td>&mdash;</td>
                     <td class="muted">reference</td></tr>`
            continue
        }
        rows += `<tr><td><label><input type="checkbox" data-rec="${rec.recording}"
                 data-name="${name}" checked><span class="swatch"
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
      <div class="rec">
        <svg width="${SIZE}" height="${SIZE}" data-rec="${rec.recording}">${paths}</svg>
        <div>
          <h2>${rec.recording}</h2>
          <div class="sub">reference path ${rec.reference_path_m} m</div>
          ${failed}
          <table class="legend">${rows}</table>
        </div>
      </div>`)
}

document.addEventListener("change", (e) => {
    const cb = e.target
    if (!cb.dataset || !cb.dataset.rec) { return }
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
    args.out.write_text(_TEMPLATE.replace("__PAYLOAD__", json.dumps(payload)))
    print(args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
