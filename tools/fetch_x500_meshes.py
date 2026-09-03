#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Fetch the Holybro X500 meshes and convert them for MuJoCo.

    .venv/bin/python tools/fetch_x500_meshes.py

Source is PX4/PX4-gazebo-models (BSD-3-Clause), the same assets Gazebo renders,
so the simulated drone looks like the one people are used to seeing there.

Two conversions are needed and neither is optional:

* **MuJoCo cannot load COLLADA.** It reads STL, OBJ and MSH. The X500 frame and
  motors ship as ``.dae``, so they are converted to OBJ here (needs pycollada).
* **The frame must NOT be decimated.** It is ~123,000 disconnected pieces of
  triangle soup (standoffs, screws, plates), not a welded surface. Decimating it
  to 16k destroyed 98% of its surface area and left a scatter of fragments that
  rendered as a few specks in mid-air. Welding first does not save it either:
  60k still loses 55% of the area. MuJoCo stores each mesh ONCE and instances it
  per drone, so the full 179k costs the same whether you fly one drone or six.
  Binary STL keeps it to ~9 MB. The props are solid single parts and do decimate
  cleanly (89% of area at 4k faces).

Every mesh is also **centred on its bounding box**. Left uncentred, MuJoCo
re-centres a mesh geom and shifts ``geom_pos`` by the centroid, which threw every
prop (0.026, 0.173) off the arm it was supposed to sit on.

The meshes are purely decorative: ``scene.py`` attaches them with contype 0 and
mass 0, keeping the primitives as the collision shapes. If this script is never
run, the simulator falls back to the box-and-cylinder airframe and flies exactly
the same.
"""

from __future__ import annotations

from pathlib import Path
import sys
import urllib.request

BASE = "https://raw.githubusercontent.com/PX4/PX4-gazebo-models/main/models/x500_base/meshes"
OUT = Path(__file__).resolve().parents[1] / "dimos/simulation/px4_hil/assets/x500"

# source file -> (output name, target triangle count)
# source file -> (output name, target triangle count; None = do not decimate)
MESHES: dict[str, tuple[str, int | None]] = {
    "NXP-HGD-CF.dae": ("frame.stl", None),
    "5010Base.dae": ("motor_base.stl", None),
    "5010Bell.dae": ("motor_bell.stl", None),
    "1345_prop_ccw.stl": ("prop_ccw.stl", 4000),
    "1345_prop_cw.stl": ("prop_cw.stl", 4000),
}


def main() -> int:
    try:
        import trimesh
    except ImportError:
        print("needs trimesh: uv pip install trimesh pycollada fast_simplification", file=sys.stderr)
        return 1

    OUT.mkdir(parents=True, exist_ok=True)
    tmp = OUT / ".download"
    tmp.mkdir(exist_ok=True)

    total = 0.0
    for src, (dst, target) in MESHES.items():
        raw = tmp / src
        if not raw.exists():
            print(f"  fetching {src} ...", flush=True)
            try:
                urllib.request.urlopen(f"{BASE}/{src}", timeout=120)
            except Exception as exc:
                print(f"    FAILED: {exc}", file=sys.stderr)
                return 1
            urllib.request.urlretrieve(f"{BASE}/{src}", raw)

        mesh = trimesh.load(raw, force="mesh")
        before, area_before = len(mesh.faces), mesh.area
        if target and before > target:
            mesh.merge_vertices()
            mesh = mesh.simplify_quadric_decimation(face_count=target)
        mesh.apply_translation(-mesh.bounding_box.centroid)
        # Strip materials and UVs; MuJoCo colours these from the MJCF.
        mesh = trimesh.Trimesh(vertices=mesh.vertices, faces=mesh.faces)
        mesh.export(OUT / dst)
        mb = (OUT / dst).stat().st_size / 1e6
        total += mb
        kept = mesh.area / area_before * 100 if area_before else 100.0
        print(
            f"  {src:22s} {before:>8,} -> {len(mesh.faces):>7,} tris  "
            f"area {kept:5.1f}% kept   {dst:15s} {mb:5.2f} MB"
        )
        if kept < 80.0:
            print("    WARNING: lost most of the surface -- do not decimate this one", file=sys.stderr)

    for leftover in tmp.iterdir():
        leftover.unlink()
    tmp.rmdir()
    print(f"\nwrote {len(MESHES)} meshes to {OUT} ({total:.2f} MB total)")
    print("Source: PX4/PX4-gazebo-models, BSD-3-Clause.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
