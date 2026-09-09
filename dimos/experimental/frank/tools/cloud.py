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

"""The lidar map as text FRANK can read cell by cell: a height raster of everything seen so far.

    uv run python dimos/experimental/frank/tools/cloud.py sketch            # whole map, robot marked @@
    uv run python dimos/experimental/frank/tools/cloud.py sketch --around 6  # only 6 m around the robot
    uv run python dimos/experimental/frank/tools/cloud.py sketch --json      # the raw encoding

Same idea as DimOS's `PointCloud2.agent_encode` (PR #3896): one character pair per cell, lowest
and highest lidar return, world frame, north up. Floor reads as `33`/`44`, a chair as `4B`, a wall
as `4G` or taller, nothing seen as `..`. Use it to tell where the room continues past what the costmap
knows, and what is standing between you and a spot. Read-only.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from typing import Any

import numpy as np

from dimos.porcelain.dimos import Dimos

ALPHABET = "0123456789ABCDEFGHIJKLMNOPQRSTU"
Z_MIN = -0.5
Z_STEP = 0.1
MAX_CELLS = 48
BOX_Z = (0.15, 1.0)

LEGEND = (
    "World-frame metres, +x east, +y north, rows north to south, each prefixed with its y; "
    "two characters per cell west to east from origin_xy_m, lowest then highest return as "
    "round((z - z_min_m) / z_step_m) in 0-9A-U. `..` = nothing seen there yet. Heights are from the "
    "lidar, which sits 0.3 m up, so flat floor reads 33-44; a chair or table lifts the second "
    "character to about 8-C, a wall to D or higher. A row of `..` next to floor is space nobody has "
    "scanned, not a wall. boxes are x-y extents of returns between 0.15 and 1.0 m "
    "high (things you would walk into), xmin:xmax@ymin:ymax. @@ is the robot."
)


def encode(pts: np.ndarray, frame_id: str = "world", ts: float | None = None) -> dict[str, Any]:
    """PR #3896's agent_encode over an (N, 3) array, verbatim in spirit: scalars, window,
    centroid, floor footprint, min/max height raster, body-height boxes."""
    n = int(pts.shape[0])
    out: dict[str, Any] = {
        "frame_id": frame_id,
        "ts": None if ts is None else round(float(ts), 2),
        "num_points": n,
        "window_m": {"x": [], "y": [], "z": []},
        "centroid_xy_m": [],
        "floor_footprint_m2": 0.0,
        "raster": {
            "cell_m": 0.0,
            "origin_xy_m": [],
            "z_step_m": Z_STEP,
            "z_min_m": Z_MIN,
            "rows": [],
        },
        "boxes": {"z_m": list(BOX_Z), "xmin:xmax@ymin:ymax": ""},
    }
    if n == 0:
        return out
    xy, z = pts[:, :2], pts[:, 2]
    mins, maxs = pts.min(axis=0), pts.max(axis=0)
    out["window_m"] = {
        k: [round(float(mins[i]), 2), round(float(maxs[i]), 2)] for i, k in enumerate("xyz")
    }
    cx, cy = xy.mean(axis=0)
    out["centroid_xy_m"] = [round(float(cx), 2), round(float(cy), 2)]
    out["floor_footprint_m2"] = round(
        float(np.unique(np.floor(xy / 0.2).astype(np.int64), axis=0).shape[0]) * 0.04, 1
    )
    out["raster"] = height_raster(pts)
    band = xy[(z >= BOX_Z[0]) & (z <= BOX_Z[1])]
    out["boxes"] = {"z_m": list(BOX_Z), "xmin:xmax@ymin:ymax": body_height_boxes(band)}
    return out


def height_raster(pts: np.ndarray) -> dict[str, Any]:
    xy = pts[:, :2]
    lo, hi = xy.min(axis=0), xy.max(axis=0)
    cell = 0.25
    while True:
        origin = np.floor(lo / cell) * cell
        shape = np.floor((hi - origin) / cell).astype(np.int64) + 1
        if int(shape.max()) <= MAX_CELLS:
            break
        cell *= 2.0
    nx, ny = int(shape[0]), int(shape[1])
    ij = np.floor((xy - origin) / cell).astype(np.int64)
    lin = ij[:, 1] * nx + ij[:, 0]
    levels = len(ALPHABET)
    q = np.clip(np.rint((pts[:, 2] - Z_MIN) / Z_STEP), 0, levels - 1).astype(np.int64)
    qmin = np.full(nx * ny, levels, dtype=np.int64)
    qmax = np.full(nx * ny, -1, dtype=np.int64)
    np.minimum.at(qmin, lin, q)
    np.maximum.at(qmax, lin, q)
    glyph = np.array([*ALPHABET, "."])
    qmin[qmax < 0] = levels
    qmax[qmax < 0] = levels
    pairs = np.char.add(glyph[qmin], glyph[qmax]).reshape(ny, nx)
    labels = [f"{origin[1] + j * cell:.2f}" for j in range(ny)]
    width = max(len(s) for s in labels)
    rows = [f"{labels[j]:>{width}} " + "".join(pairs[j].tolist()) for j in range(ny - 1, -1, -1)]
    return {
        "cell_m": cell,
        "origin_xy_m": [round(float(origin[0]), 2), round(float(origin[1]), 2)],
        "z_step_m": Z_STEP,
        "z_min_m": Z_MIN,
        "rows": rows,
    }


def body_height_boxes(xy: np.ndarray, max_cells: int = 28) -> str:
    if xy.shape[0] == 0:
        return ""
    lo, hi = xy.min(axis=0), xy.max(axis=0)
    span = float(max(hi[0] - lo[0], hi[1] - lo[1]))
    cell = next((c for c in (0.25, 0.4, 0.8, 1.6, 3.2) if span / c < max_cells), 6.4)
    iy = np.floor((xy[:, 1] - lo[1]) / cell).astype(int)
    parts = []
    for r in range(int(iy.max()), -1, -1):
        sel = xy[iy == r]
        if sel.shape[0] == 0:
            continue
        sel = sel[np.argsort(sel[:, 0])]
        rx = sel[:, 0]
        breaks = np.flatnonzero(np.diff(rx) > cell)
        starts = np.concatenate(([0], breaks + 1))
        ends = np.concatenate((breaks, [rx.size - 1]))
        for s, e in zip(starts, ends, strict=False):
            a, b = f"{rx[s]:.2f}", f"{rx[e]:.2f}"
            run = a if a == b else f"{a}:{b}"
            ry = sel[s : e + 1, 1]
            ya, yb = f"{ry.min():.2f}", f"{ry.max():.2f}"
            run += f"@{ya}" if ya == yb else f"@{ya}:{yb}"
            parts.append(run)
    return ",".join(parts)


def mark_robot(enc: dict[str, Any], x: float, y: float) -> None:
    """Overwrite the robot's cell with @@ so the raster is anchored."""
    r = enc["raster"]
    if not r["rows"]:
        return
    cell, (ox, oy) = r["cell_m"], r["origin_xy_m"]
    col = math.floor((x - ox) / cell)
    j = math.floor((y - oy) / cell)
    ny = len(r["rows"])
    row_i = ny - 1 - j
    if not (0 <= row_i < ny):
        return
    label, _, body = r["rows"][row_i].partition(" ")
    if 0 <= col < len(body) // 2:
        body = body[: 2 * col] + "@@" + body[2 * col + 2 :]
        r["rows"][row_i] = f"{label} {body}"


def sketch(around: float | None, as_json: bool, gather_s: float = 3.0) -> str:
    app = Dimos.connect(timeout=10.0)
    try:
        od = app.peek_stream("odom", timeout=5.0)
        if od is None:
            raise SystemExit("no odom yet")
        # The Go2 publishes its lidar as world-frame chunks; a few seconds of them is the map
        # around the robot. (The stack's accumulated global_map does not answer a peek.)
        chunks, t0 = [], time.time()
        while time.time() - t0 < gather_s:
            pc = app.peek_stream("lidar", timeout=3.0)
            if pc is not None:
                chunks.append(pc)
        if not chunks:
            raise SystemExit("nothing on the lidar stream")
        pc = chunks[0]
    finally:
        app.stop()
    pts = np.vstack([c.points_f32() for c in chunks])
    x, y = float(od.position.x), float(od.position.y)
    if around is not None:
        keep = (np.abs(pts[:, 0] - x) <= around) & (np.abs(pts[:, 1] - y) <= around)
        pts = pts[keep]
    enc = encode(pts, pc.frame_id, getattr(pc, "ts", None))
    mark_robot(enc, x, y)
    if as_json:
        return json.dumps(enc)
    r = enc["raster"]
    yaw = float(od.yaw)
    yaw = math.degrees(yaw) if abs(yaw) < 7 else yaw
    head = [
        f"robot @@ at x={x:.1f} y={y:.1f} yaw={yaw:.0f} deg; {enc['num_points']} points from the last {gather_s:.0f} s, "
        f"footprint {enc['floor_footprint_m2']} m2, x {enc['window_m']['x']} y {enc['window_m']['y']} world",
        f"cell {r['cell_m']} m, west edge x={r['origin_xy_m'][0]}; north up",
        LEGEND,
        "",
    ]
    return "\n".join(head + r["rows"] + ["", "boxes " + enc["boxes"]["xmin:xmax@ymin:ymax"]])


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = p.add_subparsers(dest="cmd", required=True)
    sk = sub.add_parser("sketch")
    sk.add_argument("--around", type=float, help="metres around the robot; default whole map")
    sk.add_argument("--json", action="store_true")
    sk.add_argument("--seconds", type=float, default=3.0, help="how much lidar to stack")
    a = p.parse_args(argv)
    print(sketch(a.around, a.json, a.seconds))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
