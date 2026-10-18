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

"""Dark top-down of a gsc run: raw odometry vs gsc-corrected, with the loop closures GNC kept.

usage: render_loops.py <label> <dataset> [<dataset> ...]   reads <dataset>.<label>.json -> <dataset>.<label>.loops.png
"""

import json
import sys

import bench
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from paths import OUT

BACKGROUND = "#0d1117"


def density(points, extent):
    x0, x1, y0, y1 = extent
    # ~1500 cells across, never finer than 10 cm, so a 500 m drive isn't a few faint pixels
    cell = max(0.1, max(x1 - x0, y1 - y0) / 1500)
    bins = (max(1, int((x1 - x0) / cell)), max(1, int((y1 - y0) / cell)))
    z = points[:, 2]
    ground = np.median(z)
    points = points[(z > ground + 0.2) & (z < ground + 2.0)]
    hist, _, _ = np.histogram2d(points[:, 0], points[:, 1], bins=bins, range=[[x0, x1], [y0, y1]])
    return np.log1p(hist.T)


def render(label: str, name: str) -> None:
    summary = json.loads((OUT / f"{name}.summary.json").read_text())
    times, poses, clouds = bench.load(bench.source_for(name))
    result = json.loads((OUT / f"{name}.{label}.json").read_text())
    odom = np.asarray(result["odom_rows"], float)
    graph = np.asarray(result["graph"], float)
    raw_keyframes = odom[np.clip(np.searchsorted(odom[:, 0], graph[:, 0]), 0, len(odom) - 1)]
    positions, corrected = bench.apply_correction(
        bench.correction_table(raw_keyframes, graph), times, poses, clouds
    )
    kept = [e for e in result["committed_loop_edges"] if e["gnc_weight"] > 0]
    raw_points, corrected_points = np.vstack(clouds), np.vstack(corrected)
    both = np.vstack([raw_points[:, :2], corrected_points[:, :2]])
    x0, y0 = np.percentile(both, 0.5, axis=0) - 5
    x1, y1 = np.percentile(both, 99.5, axis=0) + 5
    extent = (x0, x1, y0, y1)
    raw_map, corrected_map = density(raw_points, extent), density(corrected_points, extent)
    vmax = np.percentile(
        np.concatenate([raw_map[raw_map > 0], corrected_map[corrected_map > 0]]), 99
    )
    aspect = (y1 - y0) / (x1 - x0)
    fig, axes = plt.subplots(1, 2, figsize=(22, 11 * aspect + 1.4), facecolor=BACKGROUND)
    voxels_raw = summary["voxels_raw"]
    stats = summary.get(label, {})
    panels = [
        ("raw odometry", raw_map, poses[:, :3], raw_keyframes[:, 1:3], False),
        ("gsc_pgo", corrected_map, positions, graph[:, 1:3], True),
    ]
    for ax, (title, image, path, nodes, is_gsc) in zip(axes, panels, strict=True):
        ax.set_facecolor(BACKGROUND)
        ax.imshow(image, origin="lower", extent=extent, cmap="magma", vmin=0, vmax=vmax)
        ax.plot(path[:, 0], path[:, 1], color="#58a6ff", lw=0.6, alpha=0.8)
        for edge in kept:
            a, b = nodes[edge["source_id"]], nodes[edge["target_id"]]
            ax.plot([a[0], b[0]], [a[1], b[1]], color="#3fb950", lw=1.2, alpha=0.85)
        ax.plot(*path[0, :2], "o", color="#3fb950", ms=8, mec="white")
        ax.plot(*path[-1, :2], "s", color="#f85149", ms=8, mec="white")
        text = title
        if is_gsc:
            delta = (stats["voxels"] / voxels_raw - 1) * 100 if stats else float("nan")
            text += f"   loops kept {len(kept)}   voxels {delta:+.1f}%"
            if stats.get("tag_spread_m") is not None:
                text += f"   AprilTag spread {stats['tag_spread_m']:.2f} m"
        elif summary.get("tag_spread_raw_m") is not None:
            text += f"   AprilTag spread {summary['tag_spread_raw_m']:.2f} m"
        ax.set_title(text, color="white", fontsize=13)
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_aspect("equal")
        ax.tick_params(colors="#8b949e", labelsize=8)
        for spine in ax.spines.values():
            spine.set_color("#30363d")
    fig.suptitle(
        f"{name}   {summary['duration_s'] / 60:.0f} min, {summary['scans']} scans   start ● end ■   green = loop closures (left: where their ends were before correction)",
        color="white",
        fontsize=14,
    )
    fig.tight_layout()
    out = OUT / f"{name}.{label}.loops.png"
    fig.savefig(out, dpi=120, facecolor=BACKGROUND)
    plt.close(fig)
    print(out, flush=True)


if __name__ == "__main__":
    for dataset in sys.argv[2:]:
        render(sys.argv[1], dataset)
