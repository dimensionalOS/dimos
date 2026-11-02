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

"""Interactive 3D pages of the planner's surfaces.

`scenes` draws the voxel_problem_scenes example: each synthetic world as whole
voxels and as planes, with the path each one found. `surfaces` draws a
replay_bench dump with every walkable region, connected in the planner's graph,
in its own color.

Usage:
    uv run python -m dimos.navigation.global_planner.mls_planner.utils.map_pages \\
        scenes scenes.json voxel_problem.html
    uv run python -m dimos.navigation.global_planner.mls_planner.utils.map_pages \\
        surfaces dump_dir surfaces.html --title "China Office Surfaces" \\
        --focus 3.0 7.3 1.3
"""

from __future__ import annotations

import base64
import colorsys
import json
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
import typer

from dimos.navigation.global_planner.mls_planner.utils.surface_error import (
    SURFACE_STRIDE,
    dump_voxel_size,
    plane_heights,
    voxel_keys,
)

app = typer.Typer()
# Floats per dumped cell: center x and y, surface height, wall clearance, walkable region.
CELL_STRIDE = 5

THREE = "https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"
ORBIT = "https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"

STYLE = """
:root {
  --ground: #eef1f3; --panel: #ffffff; --ink: #1b2329; --muted: #5d6b76;
  --rule: #d5dde3; --voxel: #8fa3b5; --plane: #d9912b; --path: #1f8a7c; --blocked: #c4453a;
  --scene: #f7f9fa;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    --ground: #12171b; --panel: #1a2127; --ink: #e3e9ed; --muted: #95a3ad;
    --rule: #2c363e; --voxel: #5f7385; --plane: #e8a54a; --path: #3cc0ad; --blocked: #e2685c;
    --scene: #161c21; color-scheme: dark;
  }
}
:root[data-theme="dark"] {
  --ground: #12171b; --panel: #1a2127; --ink: #e3e9ed; --muted: #95a3ad;
  --rule: #2c363e; --voxel: #5f7385; --plane: #e8a54a; --path: #3cc0ad; --blocked: #e2685c;
  --scene: #161c21; color-scheme: dark;
}
body { background: var(--ground); color: var(--ink); font: 15px/1.55 "IBM Plex Sans", system-ui, sans-serif;
  padding-inline: 16px; padding-block: 24px 48px; }
main { max-width: 1180px; margin: 0 auto; display: grid; gap: 28px; }
h1 { font: 600 28px/1.2 "IBM Plex Sans Condensed", "IBM Plex Sans", sans-serif; margin: 0; text-wrap: balance; }
h2 { font: 600 19px/1.3 "IBM Plex Sans Condensed", "IBM Plex Sans", sans-serif; margin: 0; text-wrap: balance; }
p { margin: 0; max-width: 68ch; color: var(--muted); }
.lede { color: var(--ink); }
.scene { display: grid; gap: 12px; padding-block: 20px 4px; border-top: 1px solid var(--rule); }
.pair { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 14px; }
figure { margin: 0; background: var(--panel); border: 1px solid var(--rule); border-radius: 6px; overflow: hidden; }
figcaption { display: flex; justify-content: space-between; align-items: center; gap: 8px; padding: 8px 12px;
  font: 500 13px/1.3 "IBM Plex Mono", ui-monospace, monospace; letter-spacing: .02em; }
.view { aspect-ratio: 16 / 10; max-width: 100%; background: var(--scene); display: block; width: 100%; cursor: grab; }
.chip { padding: 2px 8px; border-radius: 999px; font-size: 12px; color: var(--panel); }
.chip.ok { background: var(--path); } .chip.no { background: var(--blocked); }
.legend { display: flex; flex-wrap: wrap; gap: 6px 18px; font: 13px/1.4 "IBM Plex Mono", ui-monospace, monospace; color: var(--muted); }
.legend span::before { content: ""; display: inline-block; width: 10px; height: 10px; margin-right: 6px;
  border-radius: 2px; background: var(--c); vertical-align: -1px; }
.stats { font: 13px/1.5 "IBM Plex Mono", ui-monospace, monospace; color: var(--muted); font-variant-numeric: tabular-nums; }
.big { aspect-ratio: auto; height: min(78vh, 760px); }
"""

FONTS = (
    "https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500"
    "&family=IBM+Plex+Sans+Condensed:wght@600&family=IBM+Plex+Sans:wght@400;500&display=swap"
)

# Shared three.js setup: a scene in a canvas that renders on demand and
# follows the theme's tokens.
VIEWER_JS = """
function token(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }
function makeView(canvas, center, radius) {
  const renderer = new THREE.WebGLRenderer({canvas, antialias: true});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(40, 1, 0.01, radius * 40);
  camera.up.set(0, 0, 1);
  camera.position.set(center.x - radius * 1.1, center.y - radius * 1.6, center.z + radius * 0.9);
  const controls = new THREE.OrbitControls(camera, canvas);
  controls.target.copy(center);
  controls.update();
  scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 0.9));
  const sun = new THREE.DirectionalLight(0xffffff, 0.6); sun.position.set(-1, -2, 3); scene.add(sun);
  const render = () => {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (canvas.width !== Math.floor(w * renderer.getPixelRatio())) {
      renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix();
    }
    scene.background = new THREE.Color(token('--scene'));
    renderer.render(scene, camera);
  };
  controls.addEventListener('change', render);
  new ResizeObserver(render).observe(canvas);
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change', render);
  return {scene, render};
}
function boxes(view, centers, size, color, opacity, scaleZ, normals) {
  const geo = new THREE.BoxGeometry(size, size, size * (scaleZ || 1));
  const mat = new THREE.MeshLambertMaterial({color, transparent: opacity < 1, opacity, depthWrite: opacity >= 1});
  const mesh = new THREE.InstancedMesh(geo, mat, centers.length / 3);
  const m = new THREE.Matrix4(), at = new THREE.Vector3(), turn = new THREE.Quaternion();
  const up = new THREE.Vector3(0, 0, 1), normal = new THREE.Vector3(), one = new THREE.Vector3(1, 1, 1);
  for (let i = 0; i < centers.length / 3; i++) {
    at.fromArray(centers, 3 * i);
    // With normals, each box is turned from level to its normal.
    if (normals) turn.setFromUnitVectors(up, normal.fromArray(normals, 3 * i));
    mesh.setMatrixAt(i, m.compose(at, turn, one));
  }
  view.scene.add(mesh);
  return mesh;
}
function line(view, points, color, width) {
  const geo = new THREE.BufferGeometry().setFromPoints(points.map(p => new THREE.Vector3(p[0], p[1], p[2])));
  view.scene.add(new THREE.Line(geo, new THREE.LineBasicMaterial({color, linewidth: width || 1})));
}
"""

SCENES_JS = """
for (const [i, s] of SCENES.entries()) {
  const v = s.voxel_size, n = s.surface.length;
  const flat = s.voxels.flat();
  const lo = s.voxels.reduce((a, p) => [Math.min(a[0], p[0]), Math.min(a[1], p[1]), Math.min(a[2], p[2])], [1e9, 1e9, 1e9]);
  const hi = s.voxels.reduce((a, p) => [Math.max(a[0], p[0]), Math.max(a[1], p[1]), Math.max(a[2], p[2])], [-1e9, -1e9, -1e9]);
  const center = new THREE.Vector3((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2);
  const radius = Math.max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]) / 2 + v;
  const y = (lo[1] + hi[1]) / 2;
  const truth = s.surface.map((z, ix) => [(ix + 0.5) * v, y, z + 0.002]);
  for (const mode of ['whole_voxels', 'planes']) {
    const view = makeView(document.getElementById(`v${i}-${mode}`), center, radius);
    boxes(view, flat, v * 0.96, token('--voxel'), 0.28);
    // Each surface cell as a thin tile at the height the planner read.
    const tiles = s[mode].cells.flat();
    boxes(view, tiles, v * 0.92, token('--plane'), 1, 0.08);
    line(view, truth, token('--ink'));
    if (s[mode].path) {
      line(view, s[mode].path.map(p => [p[0], p[1], p[2] + 0.03]), token('--path'), 3);
    }
    view.render();
  }
}
"""

SURFACES_JS = """
const bytes = Uint8Array.from(atob(DATA.cells), c => c.charCodeAt(0));
const view_ = new DataView(bytes.buffer);
const stride = 14, count = bytes.length / stride;
const v = DATA.voxel_size;
const palette = DATA.palette.map(h => new THREE.Color(h));
const c = DATA.center;
const view = makeView(document.getElementById('map'), new THREE.Vector3(c[0], c[1], c[2]), DATA.radius);
// Walkable tiles and the rest, each as centers, normals and (walkable only) colors.
const walk = {centers: [], normals: [], colors: []}, rest = {centers: [], normals: []};
for (let i = 0; i < count; i++) {
  const o = stride * i;
  const region = view_.getInt32(o + 8, true);
  const into = region < 0 ? rest : walk;
  into.centers.push((view_.getInt16(o, true) + 0.5) * v, (view_.getInt16(o + 2, true) + 0.5) * v, view_.getFloat32(o + 4, true));
  const nx = view_.getInt8(o + 12) / 127, ny = view_.getInt8(o + 13) / 127;
  into.normals.push(nx, ny, Math.sqrt(Math.max(0, 1 - nx * nx - ny * ny)));
  if (region >= 0) {
    const color = palette[region % palette.length];
    walk.colors.push(color.r, color.g, color.b);
  }
}
// Each voxel's plane as a thin tile at its height and tilt; what no robot walks on is faint.
const mesh = boxes(view, walk.centers, v * 0.95, 0xffffff, 1, 0.125, walk.normals);
mesh.instanceColor = new THREE.InstancedBufferAttribute(new Float32Array(walk.colors), 3);
boxes(view, rest.centers, v * 0.95, token('--voxel'), 0.18, 0.125, rest.normals);
view.render();
"""


def page(title: str, body: str, data: object, script: str) -> str:
    """A self-contained page: the shared shell, the data, and its script."""
    return f"""<title>{title}</title>
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="{FONTS}">
<style>{STYLE}</style>
<main>{body}</main>
<script src="{THREE}"></script>
<script src="{ORBIT}"></script>
<script>const {"SCENES" if isinstance(data, list) else "DATA"} = {json.dumps(data, separators=(",", ":"))};
{VIEWER_JS}
{script}</script>
"""


@app.command()
def scenes(scenes_json: Path, out: Path) -> None:
    """The voxel problem scenes, whole voxels beside planes."""
    data = json.loads(scenes_json.read_text())
    sections = []
    for i, s in enumerate(data):
        figures = []
        for mode, label in (("whole_voxels", "Whole voxels"), ("planes", "Planes")):
            crosses = s[mode]["path"] is not None
            chip = (
                '<span class="chip ok">path found</span>'
                if crosses
                else '<span class="chip no">no path</span>'
            )
            figures.append(
                f'<figure><canvas class="view" id="v{i}-{mode}"></canvas>'
                f"<figcaption><span>{label}</span>{chip}</figcaption></figure>"
            )
        sections.append(
            f'<section class="scene"><h2>{s["name"]}</h2><p>{s["caption"]}</p>'
            f'<p class="stats">voxel {s["voxel_size"] * 100:.0f} cm · step limit '
            f"{s['step_threshold_m'] * 100:.0f} cm</p>"
            f'<div class="pair">{"".join(figures)}</div></section>'
        )
    body = (
        "<header><h1>The voxel problem, with and without planes</h1></header>"
        '<p class="lede">The MLS planner used to read every surface at the top face of its voxel. '
        "Each world below is the same voxels read two ways: whole voxels, and the plane the ray "
        "tracer now fits each voxel. Drag to orbit, scroll to zoom.</p>"
        '<div class="legend"><span style="--c: var(--voxel)">occupied voxel</span>'
        '<span style="--c: var(--plane)">surface the planner read</span>'
        '<span style="--c: var(--ink)">true surface</span>'
        '<span style="--c: var(--path)">planned path</span></div>' + "".join(sections)
    )
    out.write_text(page("Voxel Problem Scenes", body, data, SCENES_JS))
    print(f"{len(data)} scenes -> {out}")


def walkable_regions(
    dump: Path, voxels: NDArray[np.float32], voxel_size: float
) -> NDArray[np.int32]:
    """Each voxel's walkable region from the dump's planner cells: the cell in its column nearest its plane, within a voxel, else -1."""
    cells = np.fromfile(dump / "cells.f32", dtype="<f4").reshape(-1, CELL_STRIDE)
    cell_keys = voxel_keys(np.c_[cells[:, :2], np.zeros(len(cells))], voxel_size)
    order = np.argsort(cell_keys, kind="stable")
    cell_keys, cells = cell_keys[order], cells[order]
    keys = voxel_keys(np.c_[voxels[:, :2], np.zeros(len(voxels))], voxel_size)
    heights = plane_heights(voxels, voxel_size)
    lo, hi = np.searchsorted(cell_keys, keys, "left"), np.searchsorted(cell_keys, keys, "right")
    best = np.full(len(voxels), voxel_size, dtype=np.float32)
    region = np.full(len(voxels), -1, dtype=np.int32)
    # Columns hold few cells, so step through each column's k-th cell at once.
    for k in range(int((hi - lo).max(initial=0))):
        has = lo + k < hi
        at = np.minimum(lo + k, len(cells) - 1)
        dz = np.where(has, np.abs(cells[at, 2] - heights), np.inf)
        closer = dz <= best
        best[closer] = dz[closer]
        region[closer] = cells[at[closer], 4].astype(np.int32)
    return region


@app.command()
def surfaces(
    dump: Path,
    out: Path,
    min_area_m2: float = typer.Option(0.25, help="Smallest walkable region given its own color"),
    title: str = typer.Option(..., help="The page's name, e.g. the dataset's"),
    focus: tuple[float, float, float] = typer.Option(
        ..., help="Point the view opens on, in the map frame"
    ),
    view_radius_m: float = typer.Option(3.5, help="How far the view opens from the focus"),
) -> None:
    """A replay_bench dump with each walkable region, connected in the planner's graph, in its own color."""
    voxel_size = dump_voxel_size(dump)
    voxels = np.fromfile(dump / "voxels.f32", dtype="<f4").reshape(-1, SURFACE_STRIDE)
    voxels = voxels[np.isfinite(voxels[:, 6])]
    walkable = walkable_regions(dump, voxels, voxel_size)
    sizes = np.bincount(walkable[walkable >= 0])
    large = sizes * voxel_size**2 >= min_area_m2
    # Big regions first, so the largest get the most distinct palette slots.
    rank = np.full(len(sizes) + 1, -1, dtype=np.int32)
    by_size = np.flatnonzero(large)[np.argsort(-sizes[large], kind="stable")]
    rank[by_size] = np.arange(len(by_size), dtype=np.int32)

    ij = np.floor(voxels[:, :2] / voxel_size).astype(np.int16)
    record = np.zeros(
        len(voxels),
        dtype=[("i", "<i2"), ("j", "<i2"), ("z", "<f4"), ("w", "<i4"), ("nx", "i1"), ("ny", "i1")],
    )
    record["i"], record["j"] = ij[:, 0], ij[:, 1]
    record["z"] = plane_heights(voxels, voxel_size)
    # Index -1 picks the last slot, which stays -1 for tiles on no walkable cell.
    record["w"] = rank[walkable]
    # The up-facing normal's x and y in 1/127ths; the page recovers z.
    record["nx"], record["ny"] = (np.round(voxels[:, 3:5] * 127).astype(np.int8)).T
    golden = 0.61803398875
    palette = [
        "#{:02x}{:02x}{:02x}".format(
            *tuple(int(255 * c) for c in colorsys.hsv_to_rgb(i * golden % 1.0, 0.62, 0.92))
        )
        for i in range(48)
    ]
    data = {
        "voxel_size": voxel_size,
        "cells": base64.b64encode(record.tobytes()).decode(),
        "palette": palette,
        "center": list(focus),
        "radius": view_radius_m,
    }
    body = (
        f"<header><h1>{title}</h1></header>"
        f'<p class="lede">Every voxel the ray tracer fit a plane, as a tile at that plane\'s height and tilt, '
        "colored by the planner's own graph: surfaces a robot can walk between, stairs included, share "
        f"one color, each region of {min_area_m2:g} m² or more its own. Faint grey is under wall "
        "clearance, not ground, or a smaller region. Drag to orbit, scroll to zoom.</p>"
        f'<p class="stats">{len(voxels):,} voxels with planes · {len(by_size):,} walkable regions · '
        f"voxel {voxel_size * 100:.0f} cm</p>"
        '<figure><canvas class="view big" id="map"></canvas></figure>'
    )
    out.write_text(page(title, body, data, SURFACES_JS))
    print(f"{len(voxels)} voxels, {len(by_size)} walkable regions -> {out}")


if __name__ == "__main__":
    app()
