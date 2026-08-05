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

"""Render one recording's odometry paths as a self-contained top-down HTML page.

    python -m dimos.mapping.topdown_html <recording_dir>

Reads Point-LIO from the recording and every ``<method>_traj.npy`` sitting beside
it, so it produces a useful page before all the methods have been run rather than
needing the full set.

Each estimate is aligned to Point-LIO before plotting, because every method starts
at its own origin. Metric methods get a *rigid* fit; a monocular method has no
observable scale, so it gets a *similarity* fit and is labelled as such in the
legend. Quietly scale-fitting a metric method would hide exactly the drift we are
looking for, and rigidly fitting a monocular one would look like catastrophic
failure that is really just unit choice.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np

from dimos.memory2.replay import Replay
from dimos.memory2.store.sqlite import SqliteStore

# Monocular methods recover shape but not size, so they are fitted with scale.
SCALE_FREE_METHODS = {"nerf_vo"}
# How far an estimate may be from the nearest reference pose and still be scored.
MAX_REFERENCE_GAP_S = 0.1
# Any stamp past this is wall-clock rather than seconds-since-start.
UNIX_EPOCH_THRESHOLD_S = 1e8
COLOURS = {
    "pointlio": "#8de36f",
    "cuvslam": "#57a6ff",
    "cuvslam_corrected": "#3d6ea8",
    "rtabmap": "#ffb454",
    "nerf_vo": "#e06be0",
}
LABELS = {
    "pointlio": "Point-LIO (reference)",
    "cuvslam": "cuVSLAM (live odometry)",
    "cuvslam_corrected": "cuVSLAM (pose-graph corrected, retroactive)",
    "rtabmap": "RTAB-Map",
    "nerf_vo": "NeRF-VO",
}


def pointlio_trajectory(db_path: Path) -> np.ndarray:
    """Reference trajectory as ``ts x y z``, in the gravity-aligned world frame."""
    replay = Replay(store=SqliteStore(path=str(db_path)))
    rows = []
    for _ts, odometry in replay.stream("pointlio_odometry").iterate_ts():
        pose = odometry.pose.pose if hasattr(odometry.pose, "pose") else odometry.pose
        rows.append([odometry.ts, pose.position.x, pose.position.y, pose.position.z])
    return np.array(rows, dtype=float)


def _umeyama(
    source: np.ndarray, target: np.ndarray, with_scale: bool
) -> tuple[np.ndarray, float] | None:
    """Fit source onto target; returns the transformed points and the scale used."""
    if len(source) < 3:
        return None
    source_mean, target_mean = source.mean(0), target.mean(0)
    source_centred, target_centred = source - source_mean, target - target_mean
    u, singular, vt = np.linalg.svd(source_centred.T @ target_centred / len(source))
    correction = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        correction[2, 2] = -1
    rotation = vt.T @ correction @ u.T
    variance = (source_centred**2).sum() / len(source)
    scale = float((singular @ np.diag(correction)).sum() / variance) if with_scale else 1.0
    translation = target_mean - scale * rotation @ source_mean
    return (scale * rotation @ source.T).T + translation, scale


def align(estimate: np.ndarray, reference: np.ndarray, with_scale: bool) -> dict[str, Any]:
    """Umeyama fit of an estimate onto the reference, sampled at shared times."""
    if len(estimate) < 3 or len(reference) < 3:
        return {}
    # Methods disagree on time convention: some emit seconds from their first
    # pose, others absolute unix time. Guessing wrong shifts a trajectory by the
    # whole recording and looks like that method failing, so detect it.
    start = reference[0, 0]
    absolute = estimate[0, 0] > UNIX_EPOCH_THRESHOLD_S
    wanted = estimate[:, 0] if absolute else estimate[:, 0] + start
    indices = np.clip(np.searchsorted(reference[:, 0], wanted), 0, len(reference) - 1)
    # Point-LIO drops out for seconds at a time on some recordings, and nearest
    # neighbour matching will happily pair a pose against a reference 2 s away and
    # call the difference error. Pairs without a nearby reference are excluded from
    # the fit and the score, and how many were dropped is reported.
    offset = np.abs(reference[indices, 0] - wanted)
    close = offset <= MAX_REFERENCE_GAP_S
    if close.sum() < 3:
        return {}
    paired = reference[indices[close], 1:4]
    source = estimate[close, 1:4]

    result = _umeyama(source, paired, with_scale)
    if result is None:
        return {}
    fitted, scale = result
    error = np.linalg.norm(fitted - paired, axis=1)
    # A method that tracks half the recording and stops scores a *better* ATE than
    # one that tracks all of it, so coverage is reported next to the error rather
    # than left for the reader to infer from the path length.
    span = wanted[-1] - wanted[0]
    reference_span = reference[-1, 0] - reference[0, 0]
    reference_path = float(np.linalg.norm(np.diff(reference[:, 1:4], axis=0), axis=1).sum())
    return {
        "xy": fitted[:, :2].tolist(),
        "scale": round(scale, 4),
        "ate_rmse_m": round(float(np.sqrt((error**2).mean())), 3),
        "ate_max_m": round(float(error.max()), 3),
        "path_m": round(float(np.linalg.norm(np.diff(fitted, axis=0), axis=1).sum()), 1),
        # Absolute error scales with how far the rig travelled, so comparing raw ATE
        # across a 47 m corridor and a 481 m loop makes the long run look broken when
        # it is mid-pack. Error per metre travelled is the comparable number.
        #
        # Denominator is the reference's FULL path, not the path across scored points:
        # error at a scored point reflects drift accumulated over everything before
        # it, including stretches with no trustworthy reference. Measuring only the
        # scored span also cuts corners between samples, shortening the denominator
        # and inflating drift exactly where coverage is worst.
        "drift_pct": round(
            float(np.sqrt((error**2).mean()) / max(reference_path, 1e-6) * 100),
            2,
        ),
        "coverage": round(span / reference_span, 3) if reference_span > 0 else None,
        "scored_fraction": round(float(close.mean()), 3),
    }


def segment_error(
    estimate: np.ndarray, reference: np.ndarray, with_scale: bool, metres: float = 10.0
) -> dict[str, Any]:
    """Error over short segments, each aligned on its own.

    A single global fit cannot describe a monocular trajectory whose scale changes
    mid-sequence: the residual it reports is the cost of forcing one similarity
    transform onto two regimes, not the tracker's accuracy. Re-fitting per segment
    asks a different and fairer question -- how well is the local motion recovered --
    and it is the only way to compare a scale-free method against a metric one
    without the comparison being dominated by scale bookkeeping.
    """
    start = reference[0, 0]
    absolute = estimate[0, 0] > UNIX_EPOCH_THRESHOLD_S
    wanted = estimate[:, 0] if absolute else estimate[:, 0] + start
    indices = np.clip(np.searchsorted(reference[:, 0], wanted), 0, len(reference) - 1)
    close = np.abs(reference[indices, 0] - wanted) <= MAX_REFERENCE_GAP_S
    if close.sum() < 8:
        return {}
    paired = reference[indices[close], 1:4]
    source = estimate[close, 1:4]

    travelled = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(paired, axis=0), axis=1))])
    errors, scales = [], []
    edge = 0.0
    while edge < travelled[-1] - metres / 2:
        window = (travelled >= edge) & (travelled < edge + metres)
        edge += metres
        if window.sum() < 8:
            continue
        fitted = _umeyama(source[window], paired[window], with_scale)
        if fitted is None:
            continue
        aligned, scale = fitted
        errors.append(
            float(np.sqrt((np.linalg.norm(aligned - paired[window], axis=1) ** 2).mean()))
        )
        scales.append(scale)
    if not errors:
        return {}
    return {
        "segment_m": metres,
        "segments": len(errors),
        "segment_ate_median_m": round(float(np.median(errors)), 3),
        "segment_ate_max_m": round(float(np.max(errors)), 3),
        # A scale that moves between segments is the signature of a monocular
        # tracker re-estimating size mid-sequence, which a global fit hides.
        "segment_scale_spread": round(float(np.max(scales) / max(np.min(scales), 1e-9)), 2),
    }


def collect(recording_dir: Path) -> dict[str, Any]:
    db = recording_dir / f"{recording_dir.name}.db"
    reference = pointlio_trajectory(db)
    series: dict[str, Any] = {
        "pointlio": {
            "xy": reference[:, 1:3].tolist(),
            "scale": 1.0,
            "ate_rmse_m": 0.0,
            "drift_pct": 0.0,
            "coverage": 1.0,
            "scored_fraction": 1.0,
            "path_m": round(
                float(np.linalg.norm(np.diff(reference[:, 1:], axis=0), axis=1).sum()), 1
            ),
        }
    }
    for path in sorted(recording_dir.glob("*_traj.npy")):
        method = path.stem.removesuffix("_traj")
        trajectory = np.load(path)
        scale_free = method in SCALE_FREE_METHODS
        fitted = align(trajectory, reference, scale_free)
        if fitted:
            fitted.update(segment_error(trajectory, reference, scale_free))
            series[method] = fitted
    return series


def render(recording_dir: Path) -> Path:
    series = collect(recording_dir)
    stats_path = recording_dir / "stats.json"
    stats = json.loads(stats_path.read_text()) if stats_path.exists() else {}

    payload = {
        "recording": recording_dir.name,
        "series": {
            name: {
                **values,
                "colour": COLOURS.get(name, "#cccccc"),
                "label": LABELS.get(name, name),
                "scale_fitted": name in SCALE_FREE_METHODS,
            }
            for name, values in series.items()
        },
        "stats": stats,
    }
    out = recording_dir / "topdown.html"
    out.write_text(_TEMPLATE.replace("__PAYLOAD__", json.dumps(payload)))
    return out


_TEMPLATE = """<!doctype html>
<meta charset="utf-8">
<title>top-down odometry</title>
<style>
  :root { color-scheme: dark }
  body { margin: 0; background: #111318; color: #e8e8ee;
         font: 13px/1.5 ui-monospace, SFMono-Regular, Menlo, monospace }
  header { padding: 14px 18px 6px; }
  h1 { font-size: 15px; margin: 0 0 2px; font-weight: 600 }
  .sub { color: #8b93a3 }
  #wrap { display: flex; gap: 18px; padding: 8px 18px 18px; align-items: flex-start;
          flex-wrap: wrap }
  svg { background: #0c0e12; border: 1px solid #232833; border-radius: 6px }
  table { border-collapse: collapse; font-size: 12px }
  th, td { padding: 3px 10px 3px 0; text-align: right; white-space: nowrap }
  th { color: #8b93a3; font-weight: 600 }
  td:first-child, th:first-child { text-align: left }
  .swatch { display: inline-block; width: 9px; height: 9px; border-radius: 2px;
            margin-right: 6px }
  label { cursor: pointer; user-select: none }
  .note { color: #ffb454 }
  .muted { color: #8b93a3 }
</style>
<header>
  <h1 id="title"></h1>
  <div class="sub" id="subtitle"></div>
</header>
<div id="wrap">
  <svg id="plot" width="720" height="720"></svg>
  <div>
    <table id="legend"></table>
    <p class="note" id="coveragenote"></p>
    <p class="note" id="scalenote"></p>
    <table id="perf"></table>
    <p class="muted" id="machine"></p>
  </div>
</div>
<script>
const data = __PAYLOAD__

const svg = document.getElementById("plot")
const size = 720, pad = 34
const names = Object.keys(data.series)
const visible = new Set(names)

/** Bounds over every series, so toggling does not rescale the plot under you. */
function bounds() {
    const all = names.flatMap((n) => data.series[n].xy)
    const xs = all.map((p) => p[0]), ys = all.map((p) => p[1])
    const minX = Math.min(...xs), maxX = Math.max(...xs)
    const minY = Math.min(...ys), maxY = Math.max(...ys)
    const span = Math.max(maxX - minX, maxY - minY) || 1
    return { cx: (minX + maxX) / 2, cy: (minY + maxY) / 2, span: span * 1.08 }
}
const box = bounds()
const project = ([x, y]) => [
    pad + ((x - box.cx) / box.span + 0.5) * (size - 2 * pad),
    // +y is left in the robot frame, so flip to put it left on screen too
    size - pad - ((y - box.cy) / box.span + 0.5) * (size - 2 * pad),
]

function draw() {
    const metres = box.span
    const step = Math.pow(10, Math.floor(Math.log10(metres / 4)))
    let parts = [`<rect width="${size}" height="${size}" fill="#0c0e12"/>`]
    for (let g = Math.ceil((box.cx - metres / 2) / step) * step; g < box.cx + metres / 2; g += step) {
        const [gx] = project([g, 0])
        parts.push(`<line x1="${gx}" y1="${pad}" x2="${gx}" y2="${size - pad}" stroke="#1b1f27"/>`)
    }
    for (let g = Math.ceil((box.cy - metres / 2) / step) * step; g < box.cy + metres / 2; g += step) {
        const [, gy] = project([0, g])
        parts.push(`<line x1="${pad}" y1="${gy}" x2="${size - pad}" y2="${gy}" stroke="#1b1f27"/>`)
    }
    for (const name of names) {
        if (!visible.has(name)) { continue }
        const s = data.series[name]
        const d = s.xy.map((p, i) => (i ? "L" : "M") + project(p).map((v) => v.toFixed(1)).join(" "))
        const width = name === "pointlio" ? 2.4 : 1.6
        parts.push(`<path d="${d.join("")}" fill="none" stroke="${s.colour}"
                    stroke-width="${width}" stroke-linejoin="round" opacity="0.92"/>`)
        const [sx, sy] = project(s.xy[0])
        parts.push(`<circle cx="${sx}" cy="${sy}" r="3.5" fill="${s.colour}"/>`)
    }
    parts.push(`<text x="${pad}" y="${size - 12}" fill="#8b93a3" font-size="11">
                grid ${step} m — start marked with a dot</text>`)
    svg.innerHTML = parts.join("")
}

document.getElementById("title").textContent = data.recording + " — top-down odometry"
document.getElementById("subtitle").textContent =
    (data.stats.duration_s ? data.stats.duration_s + " s recording · " : "") +
    names.length + " trajectories, all aligned to Point-LIO"

const legend = names.map((name) => {
    const s = data.series[name]
    return `<tr>
      <td><label><input type="checkbox" data-name="${name}" checked>
        <span class="swatch" style="background:${s.colour}"></span>${s.label}</label></td>
      <td>${s.ate_rmse_m === 0 ? "—" : s.ate_rmse_m + " m"}</td>
      <td>${s.drift_pct != null ? s.drift_pct + "%" : "—"}</td>
      <td class="${s.coverage != null && s.coverage < 0.95 ? "note" : ""}">${
          s.coverage != null ? (s.coverage * 100).toFixed(0) + "%" : "—"}</td>
      <td class="${s.scored_fraction != null && s.scored_fraction < 0.9 ? "note" : ""}">${
          s.scored_fraction != null ? (s.scored_fraction * 100).toFixed(0) + "%" : "—"}</td>
      <td>${s.path_m} m</td>
      <td>${s.scale_fitted ? "x" + s.scale : ""}</td>
    </tr>`
}).join("")
document.getElementById("legend").innerHTML =
    `<tr><th>trajectory</th><th>ATE rmse</th><th>drift</th><th>covered</th><th>scored</th><th>path</th><th>scale</th></tr>` + legend

if (names.some((n) => data.series[n].coverage != null && data.series[n].coverage < 0.95)) {
    document.getElementById("coveragenote").textContent =
        "Highlighted trajectories do not span the whole recording. ATE is computed " +
        "only over what they covered, so a method that stops early scores better " +
        "than one that tracks throughout - compare coverage before comparing error."
}
if (names.some((n) => data.series[n].scale_fitted)) {
    document.getElementById("scalenote").textContent =
        "Scale-fitted trajectories are monocular: size is not observable from the data, " +
        "so a similarity fit is used and the fitted factor is shown. Their shape is " +
        "comparable, their absolute size is not."
}

const methods = data.stats.methods || {}
if (Object.keys(methods).length) {
    const rows = Object.entries(methods).map(([name, m]) => `<tr>
        <td>${name}</td>
        <td>${m.real_time_factor ?? "-"}x</td>
        <td>${m.feed_fps ?? "—"}</td>
        <td>${m.feed_pose_yield != null ? (m.feed_pose_yield * 100).toFixed(0) + "%" : "—"}</td>
        <td>${m.cpu_mean_pct ?? "—"}%</td>
        <td>${m.rss_peak_gb ?? "—"}</td>
        <td>${m.gpu_mem_peak_gb ?? "—"}</td>
      </tr>`).join("")
    document.getElementById("perf").innerHTML =
        `<tr><th>method</th><th>total rtf</th><th>feed fps</th><th>pose yield</th>
             <th>cpu</th><th>rss gb</th><th>gpu gb</th></tr>` + rows
}
const machine = data.stats.machine
if (machine) {
    document.getElementById("machine").textContent =
        `${machine.cpu_count} threads · governor ${(machine.cpu_governor || []).join(",")} · ` +
        `${(machine.gpu || []).join(" · ")} — clocks are not the boot defaults on this box, ` +
        `so timings are only comparable against runs in the same state.`
}

svg.addEventListener("click", () => {})
document.getElementById("legend").addEventListener("change", (event) => {
    const name = event.target.dataset.name
    if (!name) { return }
    if (event.target.checked) { visible.add(name) } else { visible.delete(name) }
    draw()
})
draw()
</script>
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording_dir", type=Path, nargs="+")
    args = parser.parse_args()
    for directory in args.recording_dir:
        if not (directory / f"{directory.name}.db").exists():
            print(f"topdown_html: no recording in {directory}", file=sys.stderr)
            continue
        print(render(directory))
    return 0


if __name__ == "__main__":
    sys.exit(main())
