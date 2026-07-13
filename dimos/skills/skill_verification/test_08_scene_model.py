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

"""Wishlist item 8 — full 3D scene model with cross-sections (SceneModel).

Headless: builds a synthetic two-story house point cloud, runs the real
SceneModel, and produces a plan cut, an elevation, a mesh and a sliceable rrd
for review. See manifest.yaml (08-scene-3d-model).
"""

from __future__ import annotations

import numpy as np

from dimos.mapping.reconstruction.scene_model import SceneModel

SKILL_ID = "08-scene-3d-model"


def _synthetic_house() -> np.ndarray:
    """Two-story house: floor slabs, four walls/story, roof, table, stair volume.

    Same scene as misc/skills_dashboard/scripts/gen_scene_model_figures.py so the
    review artifact matches the committed dashboard figure.
    """
    rng = np.random.default_rng(11)
    pts: list[np.ndarray] = []

    def plane(xs: tuple[float, float], ys: tuple[float, float], zs: tuple[float, float], n: int) -> None:
        pts.append(
            np.column_stack([rng.uniform(*xs, n), rng.uniform(*ys, n), rng.uniform(*zs, n)])
        )

    for floor_z in (0.0, 3.0):  # two stories
        plane((0, 8), (0, 5), (floor_z, floor_z + 0.03), 6000)  # slab
        plane((0, 8), (0, 0.06), (floor_z, floor_z + 2.9), 3500)  # south wall
        plane((0, 8), (4.94, 5), (floor_z, floor_z + 2.9), 3500)  # north wall
        plane((0, 0.06), (0, 5), (floor_z, floor_z + 2.9), 2500)  # west wall
        plane((7.94, 8), (0, 5), (floor_z, floor_z + 2.9), 2500)  # east wall
    plane((0, 8), (0, 5), (5.95, 6.0), 5000)  # roof
    plane((2.2, 3.2), (1.6, 2.6), (0.72, 0.78), 700)  # table, story 1
    plane((5.2, 6.4), (2.0, 2.4), (0.0, 3.0), 900)  # stair volume
    return np.vstack(pts).astype(np.float32)


def test_scene_model_sections_and_exports(skill_output) -> None:
    """SceneModel takes plan/elevation cuts and exports mesh + rrd; render them."""
    import matplotlib.pyplot as plt

    points = _synthetic_house()
    model = SceneModel(points)

    plan = model.horizontal_section(z=1.5, thickness=0.2, resolution=0.05)
    elev = model.vertical_section((0, 2.1), (8, 2.1), thickness=0.5, resolution=0.05)

    # --- machine-checkable invariants -------------------------------------
    assert plan.density.ndim == 2 and plan.density.size > 0
    assert elev.density.ndim == 2 and elev.density.size > 0
    # A plan cut through the walls must actually hit wall points.
    assert int(plan.density.sum()) > 0, "plan cut hit no geometry"
    assert int(elev.density.sum()) > 0, "elevation cut hit no geometry"

    # --- review artifact: 3D scatter + plan + elevation -------------------
    fig = plt.figure(figsize=(13.2, 4.4))
    ax3 = fig.add_subplot(1, 3, 1, projection="3d")
    ax_p = fig.add_subplot(1, 3, 2)
    ax_e = fig.add_subplot(1, 3, 3)

    rng = np.random.default_rng(0)
    sub = model.points[rng.choice(len(model.points), 9000, replace=False)]
    ax3.scatter(sub[:, 0], sub[:, 1], sub[:, 2], s=0.4, c=sub[:, 2], cmap="viridis", linewidths=0)
    ax3.set_title(f"SceneModel ({len(model.points):,} pts)")
    ax3.view_init(elev=18, azim=-60)

    ax_p.imshow(
        np.log1p(plan.density),
        origin="lower",
        cmap="Greys",
        extent=(plan.extent[0], plan.extent[2], plan.extent[1], plan.extent[3]),
    )
    ax_p.set_title("horizontal_section(z=1.5)")
    ax_p.set_xlabel("x [m]")
    ax_p.set_ylabel("y [m]")

    ax_e.imshow(
        np.log1p(elev.density),
        origin="lower",
        cmap="Greys",
        extent=(elev.extent[0], elev.extent[2], elev.extent[1], elev.extent[3]),
    )
    ax_e.set_title("vertical_section((0,2.1)->(8,2.1))")
    ax_e.set_xlabel("along cut [m]")
    ax_e.set_ylabel("z [m]")

    fig.tight_layout()
    fig.savefig(skill_output.produced("scene_sections.png"), dpi=150)
    plt.close(fig)

    # --- review artifact: mesh (.glb) -------------------------------------
    # Coarse Poisson (voxel=0.2, shallow depth) keeps the test fast.
    mesh = model.to_mesh(voxel=0.2, poisson_depth=7)
    assert len(mesh.vertices) > 0 and len(mesh.triangles) > 0
    mesh_path = skill_output.path("scene_model.glb")
    model.save_mesh(mesh_path, mesh=mesh)
    assert mesh_path.is_file() and mesh_path.stat().st_size > 0
    skill_output.produced("scene_model.glb")

    # --- review artifact: sliceable rrd -----------------------------------
    vox = model.voxel_occupancy(voxel=0.2)
    rrd_path = skill_output.path("scene_model.rrd")
    model.save_rerun_sliceable(rrd_path, vox, z_step=0.5)
    assert rrd_path.is_file() and rrd_path.stat().st_size > 0
    skill_output.produced("scene_model.rrd")
