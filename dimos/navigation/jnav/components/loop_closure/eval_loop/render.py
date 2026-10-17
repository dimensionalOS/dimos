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

"""Top-down raw / Ivan PGO / gsc_pgo maps for each benchmarked dataset -> <name>.topdown.png"""

import json
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from paths import OUT as HERE

CELL_M = 0.05


def density(points, extent):
    x0, x1, y0, y1 = extent
    bins = (max(1, int((x1 - x0) / CELL_M)), max(1, int((y1 - y0) / CELL_M)))
    # keep only above-ground returns (most go2 points are ground) so walls dominate
    z = points[:, 2]
    ground = np.median(z)
    points = points[(z > ground + 0.2) & (z < ground + 2.0)]
    hist, _, _ = np.histogram2d(points[:, 0], points[:, 1], bins=bins, range=[[x0, x1], [y0, y1]])
    return np.log1p(hist.T)


def main():
    for name in sys.argv[1:]:
        maps = np.load(HERE / f"{name}.maps.npz")
        summary = json.loads((HERE / f"{name}.summary.json").read_text())
        methods = [
            ("ivan", "Ivan PGO, PGO() defaults"),
            ("ivan_live", "Ivan PGO, LIVE_PGO"),
            ("gsc", "gsc_pgo"),
        ]
        allpts = np.vstack([maps["raw"]] + [maps[key] for key, _ in methods])
        x0, y0 = np.percentile(allpts[:, :2], 0.5, axis=0) - 2
        x1, y1 = np.percentile(allpts[:, :2], 99.5, axis=0) + 2
        extent = (x0, x1, y0, y1)
        panels = [("raw odometry", maps["raw"], maps["raw_path"], summary["voxels_raw"], None)]
        panels += [
            (title, maps[key], maps[f"{key}_path"], summary[key]["voxels"], summary[key])
            for key, title in methods
        ]
        aspect = (y1 - y0) / (x1 - x0)
        fig, axes = plt.subplots(1, 4, figsize=(28, 7 * aspect + 1.2), facecolor="white")
        vmax = max(density(p, extent).max() for _, p, _, _, _ in panels)
        for ax, (title, points, path, voxels, stats) in zip(axes, panels, strict=True):
            ax.imshow(
                density(points, extent),
                origin="lower",
                extent=extent,
                cmap="Greys",
                vmin=0,
                vmax=vmax * 0.8,
            )
            colors = plt.cm.cool(np.linspace(0, 1, len(path)))
            ax.scatter(path[:, 0], path[:, 1], c=colors, s=1.5, linewidths=0)
            ax.plot(*path[0, :2], "o", color="#00c853", ms=8, mec="k")
            ax.plot(*path[-1, :2], "s", color="#d50000", ms=8, mec="k")
            delta = (voxels - summary["voxels_raw"]) / summary["voxels_raw"] * 100
            text = f"{title}\nvoxels {voxels} ({delta:+.1f}%)"
            if stats:
                text += f"  loops {stats['loops']}  max shift {stats['max_shift_m']:.2f} m  {stats['runtime_s']:.1f}s"
                if stats.get("crashed_unguarded"):
                    text += "\n(crashed unguarded; shown with the ICP guard)"
            ax.set_title(text, fontsize=11)
            ax.set_xlim(x0, x1)
            ax.set_ylim(y0, y1)
            ax.set_aspect("equal")
            ax.tick_params(labelsize=7)
        fig.suptitle(
            f"{name}  ({summary['duration_s']:.0f}s, {summary['scans']} scans)  start ● end ■",
            fontsize=13,
        )
        fig.tight_layout()
        out = HERE / f"{name}.topdown.png"
        fig.savefig(out, dpi=110)
        plt.close(fig)
        print(out)


if __name__ == "__main__":
    main()
