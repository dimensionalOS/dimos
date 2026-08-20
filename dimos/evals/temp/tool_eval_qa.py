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

"""Local QA viewer for the pointcloud eval rows.

    python -m dimos.evals.temp.tool_eval_qa            # http://localhost:8765
    python -m dimos.evals.temp.tool_eval_qa --prerender  # draw every row up front

One page per row: the lidar frame the model is shown, top-down, with the
question's geometry and the expected answer drawn over it, the encoder's
raster for the same frame, and a veto button. Vetoes go to
``suites/go2_pointcloud_vetoes.json``; :func:`generate.cases` drops a vetoed
row from every slice, and the ``static`` gate hashes the file like any other
suite file. Keys on a row page: ``j`` next, ``k`` previous, ``v`` veto.

The geometry is recomputed from the row's own question text with the suite's
own functions, so what is drawn is what the truth was built from.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
import html
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib
from io import BytesIO
import json
from pathlib import Path
import re
import threading
from typing import Any
from urllib.parse import parse_qs, urlparse

import numpy as np

from dimos.evals import generate
from dimos.evals.temp import split

SUITES_DIR = Path(__file__).parents[1] / "suites"
CACHE = Path(".evo_bench/qa")

SUITE_FILES: tuple[tuple[str, str], ...] = (
    # (json stem, how the slice is known: "split" = split.assign, "committed" =
    # the row's own split field, else the fixed slice)
    ("free_range", "committed"),
    ("floor_height", "committed"),
    ("free_disk", "committed"),
    ("gap_width", "committed"),
    ("frontier", "committed"),
    ("free_range_holdout", "holdout"),
    ("clearance", "split"),
    ("route", "split"),
    ("glass", "split"),
    ("doorway", "holdout"),
    ("rooms", "holdout"),
    ("floor_level", "holdout"),
    ("stairs", "holdout"),
    ("probes", "holdout"),
    ("", "frozen"),  # go2_pointcloud_vqa.json, the geometry suite
)

COORD = r"\(\s*(-?\d+(?:\.\d+)?),\s*(-?\d+(?:\.\d+)?)\s*\)"


# -- rows ------------------------------------------------------------------------


def load_rows() -> dict[str, dict[str, Any]]:
    """Every row of every pointcloud suite, with its slice, keyed by id."""
    out: dict[str, dict[str, Any]] = {}
    for stem, how in SUITE_FILES:
        path = SUITES_DIR / (
            f"go2_pointcloud_{stem}_vqa.json" if stem else "go2_pointcloud_vqa.json"
        )
        if not path.exists():
            continue
        rows = json.loads(path.read_text())
        if how == "split":
            rows = split.assign(rows)
        for row in rows:
            row = dict(row)
            if how == "committed":
                row.setdefault("split", "train")
            elif how != "split":
                row.setdefault("split", how)
            row["suite"] = stem or "geometry"
            out[str(row["id"])] = row
    return out


def by_family(rows: dict[str, dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    table: dict[str, list[dict[str, Any]]] = {}
    for row in rows.values():
        table.setdefault(str(row["family"]), []).append(row)
    return table


# -- frames ----------------------------------------------------------------------

_stores: dict[str, Any] = {}
_lock = threading.Lock()


def _store(name: str) -> Any:
    if name not in _stores:
        from dimos.memory.cli.dataset import open_dataset

        _stores[name] = open_dataset(name)
    return _stores[name]


def frame_of(row: dict[str, Any]) -> tuple[Any, np.ndarray | None]:
    """The cloud and robot position the model is shown for this row.

    The lidar context is resolved the way the runner resolves it — the same
    selector, the same sub-sampling to ``context_budget`` — and the last shown
    cloud is drawn. ``probe`` rows carry their edits in the row itself and are
    resolved by the probes suite.
    """
    if row.get("probe"):
        probes = importlib.import_module("dimos.evals.suites.go2_pointcloud_probes")
        cloud = probes.cloud_for(row)
    else:
        cloud = None
    store = _store(str(row["dataset"]))
    robot = None
    for entry in row["context"]:
        name, window = str(entry[0]), tuple(float(v) for v in entry[1])
        fuse = generate._fuse_of(entry)
        observations = list(generate._select(name, window, fuse)(store))
        if not observations:
            continue
        if len(observations) > 8:
            step = (len(observations) - 1) / 7
            observations = [observations[round(i * step)] for i in range(8)]
        if name == "lidar" and cloud is None:
            cloud = observations[-1].data
        elif name == "odom":
            position = observations[-1].data.position
            robot = np.array([float(position.x), float(position.y)])
    return cloud, robot


# -- drawing ---------------------------------------------------------------------


def _floats(text: str, pattern: str) -> list[float]:
    match = re.search(pattern, text)
    if not match:
        raise ValueError(f"{pattern!r} not in question")
    return [float(g) for g in match.groups()]


def _square(ax: Any, x0: float, y0: float, w: float, h: float, **style: Any) -> None:
    from matplotlib.patches import Rectangle

    ax.add_patch(Rectangle((x0, y0), w, h, fill=False, **style))


def _cells(
    ax: Any, mask: np.ndarray, origin: np.ndarray, cell: float, color: str, alpha: float
) -> None:
    """Shade every True cell of a ``_cell_grid`` mask."""
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return
    from matplotlib.collections import PatchCollection
    from matplotlib.patches import Rectangle

    patches = [
        Rectangle((origin[0] + i * cell, origin[1] + j * cell), cell, cell)
        for j, i in zip(ys, xs, strict=True)
    ]
    ax.add_collection(PatchCollection(patches, facecolor=color, edgecolor="none", alpha=alpha))


def draw_free_range(ax: Any, row: dict[str, Any], pts: np.ndarray, robot: np.ndarray | None) -> str:
    from matplotlib.patches import Circle

    suite = importlib.import_module("dimos.evals.suites.go2_pointcloud_free_range")
    point = np.array(_floats(str(row["q"]), r"world point " + COORD))
    lengths, cap = suite.runs(pts, point)
    ax.add_patch(Circle(point, cap, fill=False, ls="--", color="k", lw=1))
    win = str(row["a"])
    for i, name in enumerate(generate.COMPASS):
        theta = np.radians(i * 45.0)
        is_win = name == win
        u = np.array([np.cos(theta), np.sin(theta)])
        n = np.array([-u[1], u[0]]) * suite.HALF_WIDTH
        a, b = point + u * suite.START, point + u * lengths[i]
        poly = np.array([a + n, b + n, b - n, a - n])
        ax.fill(poly[:, 0], poly[:, 1], color="lime" if is_win else "royalblue", alpha=0.45 if is_win else 0.2, lw=0)
        ax.plot([a[0], b[0]], [a[1], b[1]], color="green" if is_win else "navy", lw=2 if is_win else 1)
        short = "".join(w[0] for w in re.findall("north|south|east|west", name)).upper()
        ax.annotate(f"{short} {lengths[i]:.1f}", b, fontsize=8, color="green" if is_win else "navy")
    ax.plot(*point, marker="*", ms=14, color="magenta", mec="k")
    _zoom(ax, point, cap + 0.6)
    verdict = "none (no heading clears the threshold)" if win == "none" else f"most open: {win}"
    return f"{verdict}; cap {cap:.1f} m"


def draw_floor_height(ax: Any, row: dict[str, Any], pts: np.ndarray, robot: np.ndarray | None) -> str:
    from matplotlib.patches import Circle

    suite = importlib.import_module("dimos.evals.suites.go2_pointcloud_floor_height")
    if robot is not None:
        ax.add_patch(Circle(robot, suite.FAR, fill=False, ls="--", color="k", lw=1))
    note = draw_points(ax, row, pts, robot)
    if robot is not None:
        _zoom(ax, robot, suite.FAR + 1.0)
    return "dashed = the few meters looked at; " + note


def draw_free_disk(ax: Any, row: dict[str, Any], pts: np.ndarray, robot: np.ndarray | None) -> str:
    from matplotlib.patches import Circle

    suite = importlib.import_module("dimos.evals.suites.go2_pointcloud_free_disk")
    point = np.array(_floats(str(row["q"]), r"world point " + COORD))
    origin, count, _, zmax = generate._cell_grid(pts, suite.CELL)
    wx, wy = generate._cell_centers(origin, suite.CELL, count.shape)
    measured = count > 0
    body = robot is not None and (np.hypot(wx - robot[0], wy - robot[1]) <= suite.BODY)
    swept = measured & ((zmax <= suite.LOW_Z) | body)
    _cells(ax, swept, origin, suite.CELL, "lime", 0.25)
    _cells(ax, measured & ~swept, origin, suite.CELL, "red", 0.25)
    ax.add_patch(Circle(point, suite.NEAR, fill=False, ls="--", color="k", lw=1))
    ax.plot(*point, marker="*", ms=14, color="magenta", mec="k")
    if row["a"]:
        x, y = row["a"][0][0], row["a"][0][1]
        ax.plot(x, y, marker="+", ms=16, color="magenta", mew=3)
        ax.annotate("clear spot centre", (x, y), color="magenta", fontsize=10, fontweight="bold", xytext=(8, 8), textcoords="offset points")
        verdict = f"spot at ({x}, {y})"
    else:
        verdict = "none (no clear mapped patch big enough)"
    _zoom(ax, point, suite.NEAR + 1.0)
    return f"green = swept & nothing above {suite.LOW_Z}; red = holds a return above; white = unmapped. {verdict}"


def draw_gap_width(ax: Any, row: dict[str, Any], pts: np.ndarray, robot: np.ndarray | None) -> str:
    from matplotlib.patches import Circle
    from scipy.sparse.csgraph import connected_components
    from scipy.spatial import cKDTree

    suite = importlib.import_module("dimos.evals.suites.go2_pointcloud_gap_width")
    point = np.array(_floats(str(row["q"]), r"world point " + COORD))
    near = np.hypot(pts[:, 0] - point[0], pts[:, 1] - point[1]) <= suite.REACH
    body = robot is not None and (
        np.hypot(pts[:, 0] - robot[0], pts[:, 1] - robot[1]) <= suite.BODY
    )
    high = pts[near & ~body & (pts[:, 2] > suite.LOW_Z)][:, :2].astype(np.float64)
    ax.add_patch(Circle(point, suite.REACH, fill=False, ls="--", color="k", lw=1))
    ax.plot(*point, marker="*", ms=14, color="magenta", mec="k")
    if high.shape[0] < 2:
        return "fewer than two returns above the edge"
    tree = cKDTree(high)
    _, label = connected_components(tree.sparse_distance_matrix(tree, suite.LINK), directed=False)
    keep = np.flatnonzero(np.bincount(label) >= suite.MIN_PTS)
    colors = ["tab:purple", "tab:green", "tab:cyan", "black", "tab:brown", "tab:pink", "tab:olive"]
    groups = []
    for k, g in enumerate(keep):
        sel = high[label == g]
        groups.append(sel)
        ax.scatter(
            sel[:, 0],
            sel[:, 1],
            s=14,
            color=colors[k % len(colors)],
            label=f"group {k} ({len(sel)})",
        )
    best: tuple[float, np.ndarray, np.ndarray] | None = None
    for a in range(len(groups)):
        ta = cKDTree(groups[a])
        for b in range(a + 1, len(groups)):
            d, idx = ta.query(groups[b])
            m = int(d.argmin())
            if best is None or d[m] < best[0]:
                best = (float(d[m]), groups[a][idx[m]], groups[b][m])
    if best is not None:
        ax.plot([best[1][0], best[2][0]], [best[1][1], best[2][1]], color="magenta", lw=3)
        ax.annotate(
            f"{best[0]:.2f} m",
            (best[1] + best[2]) / 2,
            color="magenta",
            fontsize=11,
            fontweight="bold",
        )
    ax.legend(loc="lower right", fontsize=8)
    _zoom(ax, point, suite.REACH + 0.5)
    return (
        f"{len(groups)} groups of >= {suite.MIN_PTS}; narrowest {best[0]:.3f} m"
        if best
        else f"{len(groups)} group"
    )


def draw_route(ax: Any, row: dict[str, Any], pts: np.ndarray, robot: np.ndarray | None) -> str:
    from scipy import ndimage

    suite = importlib.import_module("dimos.evals.suites.go2_pointcloud_route")
    goal = np.array(_floats(str(row["q"]), r"reach the point at " + COORD))
    if robot is None:
        return "no odom"
    bounds = (np.minimum(robot, goal) - 0.5, np.maximum(robot, goal) + 0.5)
    origin, count, _, zmax = generate._cell_grid(pts, suite.CELL, bounds=bounds)
    wx, wy = generate._cell_centers(origin, suite.CELL, count.shape)
    from_robot = np.hypot(wx - robot[0], wy - robot[1])
    obstacle = (zmax > suite.LOW_Z) & (from_robot > suite.BODY)
    passable = ndimage.distance_transform_edt(~obstacle) * suite.CELL > suite.MARGIN
    c_origin, c_count, _, _ = generate._cell_grid(pts, suite.SWEPT_CELL, bounds=bounds)
    cj = np.floor((wy - c_origin[1]) / suite.SWEPT_CELL).astype(np.int64)
    ci = np.floor((wx - c_origin[0]) / suite.SWEPT_CELL).astype(np.int64)
    ok = (cj >= 0) & (cj < c_count.shape[0]) & (ci >= 0) & (ci < c_count.shape[1])
    held = np.zeros_like(passable)
    held[ok] = c_count[cj[ok], ci[ok]] > 0
    swept = passable & held
    labels, _ = ndimage.label(swept, structure=np.ones((3, 3)))
    start = from_robot <= suite.BODY
    reach_ids = set(labels[swept & start].tolist()) - {0}
    reached = np.isin(labels, list(reach_ids)) if reach_ids else np.zeros_like(swept)
    _cells(ax, swept & ~reached, origin, suite.CELL, "lime", 0.18)
    _cells(ax, reached, origin, suite.CELL, "green", 0.35)
    _cells(ax, passable & ~swept, origin, suite.CELL, "grey", 0.3)
    _cells(ax, ~passable, origin, suite.CELL, "red", 0.12)
    ax.plot(*goal, marker="X", ms=14, color="magenta", mec="k")
    ax.plot([robot[0], goal[0]], [robot[1], goal[1]], color="magenta", ls=":", lw=1)
    return (
        "dark green = swept cells connected to the robot; light green = swept, not connected; "
        "grey = passable but unmeasured; red tint = within the margin of a return above the edge"
    )


def draw_clearance(ax: Any, row: dict[str, Any], pts: np.ndarray, robot: np.ndarray | None) -> str:
    suite = importlib.import_module("dimos.evals.suites.go2_pointcloud_clearance")
    if robot is None:
        return "no odom"
    name = str(row["id"]).rsplit("_", 1)[-1]
    theta = np.radians(generate.COMPASS.index(name) * 45.0)
    u = np.array([np.cos(theta), np.sin(theta)])
    n = np.array([-u[1], u[0]]) * suite.HALF_WIDTH
    a, b = robot + u * suite.SELF_RETURN, robot + u * suite.REACH
    poly = np.array([a + n, b + n, b - n, a - n])
    color = "red" if row["a"] == "blocked" else "lime"
    ax.fill(poly[:, 0], poly[:, 1], color=color, alpha=0.35, lw=0)
    band = pts[(pts[:, 2] >= generate.BODY_Z[0]) & (pts[:, 2] <= generate.BODY_Z[1])]
    ax.scatter(band[:, 0], band[:, 1], s=3, color="k", alpha=0.5, label="body-height returns")
    _zoom(ax, robot, 3.0)
    return f"corridor {2 * suite.HALF_WIDTH:g} x {suite.REACH:g} m due {name}"


def draw_points(ax: Any, row: dict[str, Any], pts: np.ndarray, robot: np.ndarray | None) -> str:
    """Hand-authored coords families: the expected points, or none."""
    answer = row["a"]
    if not isinstance(answer, list) or not answer:
        return "expected: none"
    for p in answer:
        ax.plot(p[0], p[1], marker="o", ms=16, mfc="none", mec="magenta", mew=3)
        label = f"({p[0]}, {p[1]})" + (f" {p[2]:+.2f}" if len(p) > 2 else "")
        ax.annotate(
            label,
            (p[0], p[1]),
            color="magenta",
            fontsize=10,
            fontweight="bold",
            xytext=(8, 8),
            textcoords="offset points",
        )
    return f"expected {answer}"


def draw_probe(ax: Any, row: dict[str, Any], pts: np.ndarray, robot: np.ndarray | None) -> str:
    probes = importlib.import_module("dimos.evals.suites.go2_pointcloud_probes")
    note = probes.draw(ax, row, pts, robot)
    return (
        str(note) + " | " + DRAWERS.get(str(row.get("asks", "")), draw_points)(ax, row, pts, robot)
    )


def draw_frontier(ax: Any, row: dict[str, Any], pts: np.ndarray, robot: np.ndarray | None) -> str:
    from scipy import ndimage

    suite = importlib.import_module("dimos.evals.suites.go2_pointcloud_frontier")
    point = np.array(_floats(str(row["q"]), r"world point " + COORD))
    if robot is None:
        return "no odom"
    origin = np.floor(pts[:, :2].min(axis=0) / suite.CELL) * suite.CELL
    shape = np.floor((pts[:, :2].max(axis=0) - origin) / suite.CELL).astype(np.int64) + 1
    nx, ny = int(shape[0]), int(shape[1])
    ij = np.floor((pts[:, :2] - origin) / suite.CELL).astype(np.int64)
    lin = ij[:, 1] * nx + ij[:, 0]
    count = np.bincount(lin, minlength=nx * ny).reshape(ny, nx)
    zmax = np.full(nx * ny, -np.inf)
    np.maximum.at(zmax, lin, pts[:, 2])
    zmax = zmax.reshape(ny, nx)
    wx, wy = generate._cell_centers(origin, suite.CELL, (ny, nx))
    r = np.hypot(wx - robot[0], wy - robot[1])
    measured = count > 0
    obstacle = (zmax > suite.LOW_Z) & (r > suite.BODY)
    passable = ndimage.distance_transform_edt(~obstacle) * suite.CELL > suite.MARGIN
    free = measured & passable
    labels, _ = ndimage.label(free, structure=np.ones((3, 3)))
    start = {int(v) for v in labels[r <= suite.BODY] if v}
    reach = np.isin(labels, list(start)) if start else np.zeros_like(free)
    _cells(ax, reach, origin, suite.CELL, "green", 0.3)
    _cells(ax, obstacle, origin, suite.CELL, "red", 0.3)
    _cells(ax, ~measured, origin, suite.CELL, "grey", 0.18)
    color = "lime" if row["a"] == "reachable" else "red"
    ax.plot(*point, marker="*", ms=18, color=color, mec="k", mew=1.5)
    ax.annotate(str(row["a"]), point, color=color, fontsize=12, fontweight="bold", xytext=(8, 8), textcoords="offset points")
    _zoom(ax, robot, 4.0)
    return "green = floor reachable from robot; red = obstacle; grey = unmapped; star = the asked spot"


def draw_default(ax: Any, row: dict[str, Any], pts: np.ndarray, robot: np.ndarray | None) -> str:
    return f"expected {row['a']!r}"


def _zoom(ax: Any, centre: np.ndarray, half: float) -> None:
    ax.set_xlim(centre[0] - half, centre[0] + half)
    ax.set_ylim(centre[1] - half, centre[1] + half)


DRAWERS: dict[str, Callable[[Any, dict[str, Any], np.ndarray, np.ndarray | None], str]] = {
    "free_range": draw_free_range,
    "floor_height": draw_floor_height,
    "free_disk": draw_free_disk,
    "gap_width": draw_gap_width,
    "frontier": draw_frontier,
    "route": draw_route,
    "clearance": draw_clearance,
    "doorway": draw_points,
    "floorlevel": draw_points,
    "stairs": draw_points,
}


def render(row: dict[str, Any]) -> tuple[bytes, str, str]:
    """``(png, note, raster)`` for one row."""
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    cloud, robot = frame_of(row)
    pts = cloud.points_f32()
    fig = Figure(figsize=(10, 10), dpi=96)
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111)
    z = pts[:, 2]
    low = z <= 0.15
    ax.scatter(
        pts[low, 0], pts[low, 1], c=z[low], cmap="Blues", vmin=-0.6, vmax=0.3, s=1.2, linewidths=0
    )
    sc = ax.scatter(
        pts[~low, 0],
        pts[~low, 1],
        c=z[~low],
        cmap="autumn_r",
        vmin=0.15,
        vmax=1.5,
        s=2.5,
        linewidths=0,
    )
    fig.colorbar(
        sc, ax=ax, fraction=0.03, pad=0.01, label="z of returns above 0.15 m (blues: at or below)"
    )
    lo, hi = pts[:, :2].min(axis=0), pts[:, :2].max(axis=0)
    _square(ax, lo[0], lo[1], hi[0] - lo[0], hi[1] - lo[1], ec="k", lw=0.8, ls="-.")
    if robot is not None:
        ax.plot(*robot, marker="D", ms=11, color="cyan", mec="k", label="robot (odom)")
    ax.set_xlim(lo[0] - 0.3, hi[0] + 0.3)
    ax.set_ylim(lo[1] - 0.3, hi[1] + 0.3)
    drawer = draw_probe if row.get("probe") else DRAWERS.get(str(row["family"]), draw_default)
    try:
        note = drawer(ax, row, pts, robot)
    except Exception as e:  # a drawer failing must not hide the frame
        note = f"overlay failed: {e!r}"
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3, alpha=0.5)
    ax.set_xlabel("x east (m)")
    ax.set_ylabel("y north (m)")
    ax.set_title(f"{row['id']}   expected: {row['a']}", fontsize=11)
    buf = BytesIO()
    fig.savefig(buf, format="png", bbox_inches="tight")
    encoded = cloud.agent_encode()
    raster = "\n".join(encoded["raster"]["rows"]) if isinstance(encoded.get("raster"), dict) else ""
    header = json.dumps({k: v for k, v in encoded.items() if k not in ("raster", "boxes")})
    return buf.getvalue(), note, header + "\n" + raster


# -- server ----------------------------------------------------------------------


class App:
    def __init__(self) -> None:
        self.rows = load_rows()
        self.families = by_family(self.rows)
        self.cache: dict[str, tuple[bytes, str, str]] = {}
        CACHE.mkdir(parents=True, exist_ok=True)

    def reload(self) -> None:
        for module in list(importlib.sys.modules):
            if module.startswith("dimos.evals.suites."):
                importlib.reload(importlib.sys.modules[module])
        self.rows = load_rows()
        self.families = by_family(self.rows)
        self.cache.clear()

    def rendered(self, row_id: str) -> tuple[bytes, str, str]:
        if row_id in self.cache:
            return self.cache[row_id]
        png, note, meta = (
            CACHE / f"{row_id}.png",
            CACHE / f"{row_id}.note",
            CACHE / f"{row_id}.raster",
        )
        if png.exists() and note.exists() and meta.exists():
            out = (png.read_bytes(), note.read_text(), meta.read_text())
        else:
            with _lock:
                out = render(self.rows[row_id])
            png.write_bytes(out[0])
            note.write_text(out[1])
            meta.write_text(out[2])
        self.cache[row_id] = out
        return out

    # vetoes

    def veto(self, row_id: str, reason: str) -> None:
        struck = generate.vetoes()
        if row_id in struck:
            del struck[row_id]
        else:
            struck[row_id] = reason or "vetoed in tool_eval_qa"
        generate.VETOES.write_text(json.dumps(dict(sorted(struck.items())), indent=2) + "\n")

    # pages

    def page_index(self) -> str:
        struck = generate.vetoes()
        items = []
        for family, rows in sorted(self.families.items()):
            counts: dict[str, int] = {}
            for r in rows:
                key = "vetoed" if r["id"] in struck else str(r["split"])
                counts[key] = counts.get(key, 0) + 1
            summary = "  ".join(f"{k} {v}" for k, v in sorted(counts.items()))
            items.append(
                f'<li><a href="/family/{family}">{family}</a> <span class="dim">{len(rows)} rows: {summary}</span></li>'
            )
        return _page(
            "pointcloud eval QA",
            f"<h1>pointcloud eval QA</h1><ul>{''.join(items)}</ul><p class='dim'>{len(struck)} vetoed. <a href='/reload'>reload rows</a></p>",
        )

    def page_family(self, family: str) -> str:
        struck = generate.vetoes()
        rows = sorted(self.families[family], key=lambda r: (str(r["split"]), str(r["id"])))
        cards = []
        for r in rows:
            cls = "card vetoed" if r["id"] in struck else "card"
            cards.append(
                f'<div class="{cls}"><a href="/row/{r["id"]}"><img loading="lazy" src="/img/{r["id"]}.png"></a>'
                f'<div class="cap"><b>{r["split"]}</b> {html.escape(str(r["a"]))}<br><span class="dim">{r["id"]}</span></div></div>'
            )
        return _page(
            family,
            f'<p><a href="/">all families</a></p><h1>{family} <span class="dim">{len(rows)} rows</span></h1><div class="grid">{"".join(cards)}</div>',
        )

    def page_row(self, row_id: str) -> str:
        row = self.rows[row_id]
        family = str(row["family"])
        siblings = sorted(self.families[family], key=lambda r: (str(r["split"]), str(r["id"])))
        ids = [str(r["id"]) for r in siblings]
        i = ids.index(row_id)
        prev_id, next_id = ids[i - 1], ids[(i + 1) % len(ids)]
        struck = generate.vetoes()
        veto_state = f"VETOED: {html.escape(struck[row_id])}" if row_id in struck else "in play"
        _, note, raster = self.rendered(row_id)
        context = html.escape(json.dumps(row["context"]))
        body = f"""
<p><a href="/">all</a> · <a href="/family/{family}">{family}</a> · {i + 1}/{len(ids)} ·
<a id="prev" href="/row/{prev_id}">prev (k)</a> · <a id="next" href="/row/{next_id}">next (j)</a></p>
<h1>{row_id}</h1>
<p><b>{row["split"]}</b> · {row["dataset"]} · {row["type"]} · <span id="veto-state">{veto_state}</span>
<button onclick="veto()">toggle veto (v)</button> <input id="reason" placeholder="reason" size="40"></p>
<div class="two">
<div><img src="/img/{row_id}.png"><p class="note">{html.escape(note)}</p></div>
<div>
<h3>question</h3><p class="q">{html.escape(str(row["q"]))}</p>
<h3>expected</h3><pre>{html.escape(json.dumps(row["a"]))}</pre>
<h3>context</h3><pre>{context}</pre>
<h3>encoder output for the drawn frame</h3><pre class="raster">{html.escape(raster)}</pre>
</div></div>
<script>
function veto() {{
  fetch('/veto', {{method: 'POST', headers: {{'Content-Type': 'application/x-www-form-urlencoded'}},
    body: 'id={row_id}&reason=' + encodeURIComponent(document.getElementById('reason').value)}})
    .then(() => location.reload());
}}
document.addEventListener('keydown', e => {{
  if (e.target.tagName === 'INPUT') return;
  if (e.key === 'j') location.href = document.getElementById('next').href;
  if (e.key === 'k') location.href = document.getElementById('prev').href;
  if (e.key === 'v') veto();
}});
</script>"""
        return _page(row_id, body)


def _page(title: str, body: str) -> str:
    return f"""<!doctype html><html><head><meta charset="utf-8"><title>{html.escape(title)}</title>
<style>
body{{font-family:system-ui,sans-serif;margin:16px;color:#222}} a{{color:#06c}} .dim{{color:#777;font-size:90%}}
.grid{{display:grid;grid-template-columns:repeat(auto-fill,minmax(300px,1fr));gap:10px}}
.card img{{width:100%;border:1px solid #ccc}} .card.vetoed img{{opacity:.35}} .card.vetoed .cap{{text-decoration:line-through}}
.cap{{font-size:85%}} .two{{display:grid;grid-template-columns:minmax(500px,1fr) minmax(400px,1fr);gap:16px}}
.two img{{width:100%;border:1px solid #ccc}} pre{{white-space:pre-wrap;background:#f4f4f4;padding:8px;font-size:12px}}
pre.raster{{white-space:pre;overflow-x:auto;font-size:11px;line-height:1.15}} .q{{background:#fffbe6;padding:8px}}
.note{{font-size:90%;color:#444}} button{{font-size:100%}}
</style></head><body>{body}</body></html>"""


class Handler(BaseHTTPRequestHandler):
    app: App

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _send(self, body: bytes, kind: str = "text/html; charset=utf-8", code: int = 200) -> None:
        self.send_response(code)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        url = urlparse(self.path)
        parts = [p for p in url.path.split("/") if p]
        try:
            if not parts:
                return self._send(self.app.page_index().encode())
            if parts[0] == "reload":
                self.app.reload()
                self.send_response(303)
                self.send_header("Location", "/")
                self.end_headers()
                return None
            if parts[0] == "family":
                return self._send(self.app.page_family(parts[1]).encode())
            if parts[0] == "row":
                return self._send(self.app.page_row(parts[1]).encode())
            if parts[0] == "img":
                png, _, _ = self.app.rendered(parts[1].removesuffix(".png"))
                return self._send(png, "image/png")
        except KeyError:
            return self._send(b"no such row", code=404)
        except Exception as e:
            return self._send(f"<pre>{html.escape(repr(e))}</pre>".encode(), code=500)
        return self._send(b"not found", code=404)

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length", "0"))
        form = parse_qs(self.rfile.read(length).decode())
        if urlparse(self.path).path == "/veto":
            self.app.veto(form.get("id", [""])[0], form.get("reason", [""])[0])
            return self._send(b"ok", "text/plain")
        return self._send(b"not found", code=404)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--prerender", action="store_true", help="draw every row in the background at start"
    )
    args = parser.parse_args(argv)
    app = App()
    Handler.app = app
    if args.prerender:

        def work() -> None:
            for row_id in list(app.rows):
                try:
                    app.rendered(row_id)
                except Exception as e:
                    print(f"{row_id}: {e!r}")

        threading.Thread(target=work, daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(
        f"QA viewer on http://localhost:{args.port}  ({len(app.rows)} rows, {len(app.families)} families)"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
