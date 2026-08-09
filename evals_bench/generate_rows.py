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

"""Generate pointcloud VQA rows for go2_short.

Privileged modality: full-resolution clouds + odom, quizzing whatever lossy
encoding the agent receives. Run once; rows.json is committed as static data
so ground truth is identical across evo experiments.

Families
  single-cloud : extent (A), vertical span (B), nearest obstacle (C)
  cloud stream : extent change (F), area trend MCQ (J), growth compass MCQ (H)
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from dimos.memory2.cli.dataset import open_dataset

DATASET = "go2_short"
SINGLE_TS = [5.0, 20.0, 40.0, 58.0]
WINDOWS = [(0.0, 20.0), (15.0, 45.0), (30.0, 59.0), (0.0, 59.0)]
OBSTACLE_Z = (0.15, 1.0)  # world-frame band: above floor returns, below cap
VOXEL = 0.2
COMPASS = {
    (1, 0): "east",
    (1, 1): "northeast",
    (0, 1): "north",
    (-1, 1): "northwest",
    (-1, 0): "west",
    (-1, -1): "southwest",
    (0, -1): "south",
    (0, -1 + 0): "south",
    (1, -1): "southeast",
}


def horiz_extent(pts: np.ndarray) -> float:
    return float(max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])))


def voxels2d(pts: np.ndarray) -> set[tuple[int, int]]:
    q = np.floor(pts[:, :2] / VOXEL).astype(int)
    return set(map(tuple, q))


def compass_of(v: np.ndarray) -> str:
    angle = np.arctan2(v[1], v[0])
    names = ["east", "northeast", "north", "northwest", "west", "southwest", "south", "southeast"]
    return names[int(np.round(angle / (np.pi / 4))) % 8]


def main() -> None:
    store = open_dataset(DATASET)
    rows: list[dict[str, object]] = []
    try:
        lidar = store.streams.lidar
        t0 = lidar.first().ts

        for t in SINGLE_TS:
            obs = lidar.range_time(0, t).to_list()[-1]
            ts_rel = obs.ts - t0
            pts = obs.data.points_f32()
            ctx_window = [round(ts_rel - 0.05, 2), round(ts_rel + 0.05, 2)]

            a = round(horiz_extent(pts), 2)
            rows.append(
                {
                    "id": f"pc_extent_t{t:g}",
                    "family": "extent",
                    "type": "numeric",
                    "q": "Consider the mapped point cloud shown. What is its largest "
                    "horizontal extent (the longer side of its axis-aligned "
                    "bounding box in the x-y plane), in meters?",
                    "a": a,
                    "band": round(max(0.5, 0.10 * a), 2),
                    "ctx": "lidar",
                    "window": ctx_window,
                }
            )

            b = round(float(np.ptp(pts[:, 2])), 2)
            rows.append(
                {
                    "id": f"pc_zspan_t{t:g}",
                    "family": "zspan",
                    "type": "numeric",
                    "q": "Consider the mapped point cloud shown. What is its vertical "
                    "span (max z minus min z), in meters?",
                    "a": b,
                    "band": 0.3,
                    "ctx": "lidar",
                    "window": ctx_window,
                }
            )

            odom = store.streams.odom.range_time(0, t).to_list()[-1]
            pos = np.array([odom.data.position.x, odom.data.position.y, odom.data.position.z])
            band_pts = pts[(pts[:, 2] >= OBSTACLE_Z[0]) & (pts[:, 2] <= OBSTACLE_Z[1])]
            d = np.hypot(band_pts[:, 0] - pos[0], band_pts[:, 1] - pos[1])
            d = d[d > 0.15]
            c = round(float(d.min()), 2)
            rows.append(
                {
                    "id": f"pc_nearest_t{t:g}",
                    "family": "nearest",
                    "type": "numeric",
                    "q": "You are the robot; your current pose is the odom observation "
                    "shown. Using the mapped point cloud, how far away is the "
                    "nearest obstacle point at body height (z between 0.15 and "
                    "1.0 m), horizontal distance in meters?",
                    "a": c,
                    "band": round(max(0.25, 0.30 * c), 2),
                    "ctx": "lidar+odom",
                    "window": ctx_window,
                    "odom_window": [round(max(0.0, t - 0.5), 2), round(t + 0.1, 2)],
                }
            )

        for w0, w1 in WINDOWS:
            frames = lidar.range_time(w0, w1).to_list()
            first, last = frames[0], frames[-1]
            fp, lp = first.data.points_f32(), last.data.points_f32()
            wid = f"{w0:g}_{w1:g}"

            # ponytail: extent-change family dropped — this recording maps the
            # whole room in frame 1, so change truths degenerate to ~0 and a
            # blind "0" guess wins. Revisit on a dataset with real exploration.
            va, vb = voxels2d(fp), voxels2d(lp)
            ratio = len(vb) / max(1, len(va))
            j = "grow" if ratio > 1.05 else ("shrink" if ratio < 0.95 else "same")
            rows.append(
                {
                    "id": f"pc_areatrend_{wid}",
                    "family": "areatrend",
                    "type": "mcq",
                    "q": "You are shown a sequence of mapped point clouds over time. "
                    "From the first shown cloud to the last, does the mapped "
                    "floor area grow, shrink, or stay about the same? Answer "
                    "with exactly one word: grow, shrink, or same.",
                    "a": j,
                    "choices": ["grow", "shrink", "same"],
                    "ctx": "lidar",
                    "window": [w0, w1],
                }
            )

            new = np.array(sorted(vb - va))
            if len(new) >= 50:
                centroid_new = new.mean(axis=0) * VOXEL
                centroid_first = np.array(sorted(va)).mean(axis=0) * VOXEL
                h = compass_of(centroid_new - centroid_first)
                rows.append(
                    {
                        "id": f"pc_direction_{wid}",
                        "family": "direction",
                        "type": "mcq",
                        "q": "You are shown a sequence of mapped point clouds over "
                        "time (world frame: +x is east, +y is north). In which "
                        "compass direction, relative to the center of the first "
                        "shown cloud, did the map gain most of its new coverage? "
                        "Answer with exactly one of: east, northeast, north, "
                        "northwest, west, southwest, south, southeast.",
                        "a": h,
                        "choices": [
                            "east",
                            "northeast",
                            "north",
                            "northwest",
                            "west",
                            "southwest",
                            "south",
                            "southeast",
                        ],
                        "ctx": "lidar",
                        "window": [w0, w1],
                    }
                )
    finally:
        store.stop()

    for row in rows:
        row["dataset"] = DATASET
    out = Path(__file__).parent / "rows.json"
    out.write_text(json.dumps(rows, indent=2) + "\n")
    print(f"wrote {len(rows)} rows -> {out}")
    for row in rows:
        print(f"  {row['id']:<28} a={row['a']}")


if __name__ == "__main__":
    main()
