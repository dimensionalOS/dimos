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

"""Top-down maps, raw odometry beside each module's correction, from the live runs: gsc_maps.png and ivan_maps.png
in $EVAL_LOOP_OUT, one row per recording.

usage: plot_maps.py [<dataset> ...]   (default: huge_loop_go2_fastlio small_loop_go2)
"""

import json
import sys

import bench
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from paths import OUT
from PIL import Image

BACKGROUND, TEXT, GRID = "#0d1117", "#e6edf3", "#30363d"
MODULES = [("gsc_live", "gsc_pgo"), ("ivan_live", "Ivan's module")]


def keyframes(name: str, key: str) -> tuple[np.ndarray, np.ndarray]:
    """(raw, optimized) keyframe rows [ts, x, y, z, qx, qy, qz, qw] of a live run."""
    if key == "ivan_live":
        raw, optimized = np.load(OUT / f"{name}.ivan_live_keyframes.npy")
        return raw, optimized
    result = json.loads((OUT / f"{name}.gsc_live.json").read_text())
    odom, graph = np.asarray(result["odom_rows"], float), np.asarray(result["graph"], float)
    index = np.clip(np.searchsorted(odom[:, 0], graph[:, 0]), 0, len(odom) - 1)
    return odom[index], graph


def wall_points(clouds, positions) -> np.ndarray:
    """Each scan from 1 m below to 1.5 m above its own sensor, so height drift doesn't hide the walls."""
    return np.vstack(
        [
            cloud[(cloud[:, 2] > z - 1.0) & (cloud[:, 2] < z + 1.5), :2]
            for cloud, z in zip(clouds, positions[:, 2], strict=True)
        ]
    )


def density(points: np.ndarray, low: np.ndarray, high: np.ndarray) -> np.ndarray:
    cell = max(0.1, float(np.max(high - low)) / 900)
    shape = np.ceil((high - low) / cell).astype(int) + 1
    ij = np.floor((points - low) / cell).astype(int)
    counts = np.zeros(shape[::-1])
    np.add.at(counts, (ij[:, 1], ij[:, 0]), 1)
    return np.log1p(counts)


def main() -> None:
    names = sys.argv[1:] or ["huge_loop_go2_fastlio", "small_loop_go2"]
    loaded = {}
    for name in names:
        times, poses, clouds = bench.load(bench.source_for(name))
        raw = wall_points(clouds, poses[:, :3])
        corrected = {}
        for key, _ in MODULES:
            table = bench.correction_table(*keyframes(name, key))
            positions, fixed = bench.apply_correction(table, times, poses, clouds)
            corrected[key] = wall_points(fixed, positions)
        loaded[name] = (raw, corrected)
    for key, label in MODULES:
        fig, axes = plt.subplots(
            len(names), 2, figsize=(11, 5.2 * len(names)), facecolor=BACKGROUND, squeeze=False
        )
        for row, name in enumerate(names):
            raw, corrected = loaded[name]
            both = np.vstack([raw, corrected[key]])
            low, high = both.min(0), both.max(0)
            for ax, points, title in [
                (axes[row, 0], raw, "raw odometry"),
                (axes[row, 1], corrected[key], label),
            ]:
                image = density(points, low, high)
                ax.imshow(
                    image,
                    origin="lower",
                    cmap="bone",
                    vmax=np.percentile(image[image > 0], 99.5),
                    extent=[low[0], high[0], low[1], high[1]],
                )
                ax.set_facecolor("black")
                ax.set_xticks([])
                ax.set_yticks([])
                for spine in ax.spines.values():
                    spine.set_color(GRID)
                ax.set_title(f"{name}: {title}", color=TEXT, fontsize=10)
        fig.tight_layout()
        out = OUT / f"{key.split('_')[0]}_maps.png"
        fig.savefig(out, dpi=80, facecolor=BACKGROUND)
        plt.close(fig)
        Image.open(out).convert("RGB").quantize(64, method=Image.Quantize.FASTOCTREE).save(
            out, optimize=True
        )
        print(out)


if __name__ == "__main__":
    main()
