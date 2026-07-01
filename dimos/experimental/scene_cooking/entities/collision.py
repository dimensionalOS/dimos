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

"""Cook-time convex decomposition for scene-package entities.

Mesh entities need convex collision geometry for MuJoCo's narrowphase.
The cooker decomposes each entity's ``visual.glb`` with CoACD (chair
legs / seat / back each get their own hull, so contacts are
chair-shaped) and writes the hulls into the package next to the visual:

```text
entities/<safe_id>/
├── visual.glb
└── mujoco_collision/
    ├── hull_000.obj
    └── ...
```

The hull paths are recorded per entity as ``collision_paths`` in
``scene.meta.json`` and loaded verbatim by the runtime composer
(``dimos/simulation/mujoco/scene_package_entity_composer.py``) — there is no runtime
decomposition and no per-machine cache; the package is self-contained.
"""

from __future__ import annotations

from pathlib import Path

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

COLLISION_DIR_NAME = "mujoco_collision"

_COACD_MAX_HULLS = 32
_COACD_THRESHOLD = 0.05
_COACD_RESOLUTION = 500
_COACD_MCTS_ITERATIONS = 30
_COACD_MCTS_NODES = 10


def cook_entity_collision_hulls(
    visual_mesh_path: str | Path,
    out_dir: str | Path,
    *,
    rebake: bool = False,
) -> list[Path]:
    """Decompose one entity mesh into convex hulls under ``out_dir``.

    Idempotent: existing ``hull_*.obj`` files are reused unless ``rebake``.
    Falls back to a single convex hull when CoACD is unavailable or fails
    on the mesh. Returns ``[]`` (with a warning) when the mesh can't be
    read at all — the runtime composer then uses an AABB box.
    """
    mesh_path = Path(visual_mesh_path)
    out_dir = Path(out_dir)

    if not rebake:
        existing = sorted(out_dir.glob("hull_*.obj"))
        if existing:
            return existing

    try:
        import open3d as o3d  # type: ignore[import-untyped]
    except ImportError as exc:
        logger.warning("entity hulls: open3d unavailable (%s); skipping %s", exc, mesh_path)
        return []

    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    if not mesh.has_vertices():
        logger.warning("entity hulls: no vertices in %s; skipping", mesh_path)
        return []

    parts = _run_coacd(mesh, mesh_path)

    out_dir.mkdir(parents=True, exist_ok=True)
    for stale in out_dir.glob("hull_*.obj"):
        stale.unlink()

    import numpy as np

    out_paths: list[Path] = []
    if parts:
        for i, (vertices, triangles) in enumerate(parts):
            hull_mesh = o3d.geometry.TriangleMesh()
            hull_mesh.vertices = o3d.utility.Vector3dVector(np.asarray(vertices, dtype=np.float64))
            hull_mesh.triangles = o3d.utility.Vector3iVector(np.asarray(triangles, dtype=np.int32))
            path = out_dir / f"hull_{i:03d}.obj"
            o3d.io.write_triangle_mesh(str(path), hull_mesh, write_vertex_normals=False)
            out_paths.append(path)
    else:
        hull, _ = mesh.compute_convex_hull()
        path = out_dir / "hull_000.obj"
        o3d.io.write_triangle_mesh(str(path), hull, write_vertex_normals=False)
        out_paths.append(path)

    return out_paths


def _run_coacd(mesh: object, mesh_path: Path) -> list[tuple[object, object]]:
    """CoACD parts for an open3d mesh; ``[]`` means fall back to one hull."""
    try:
        import coacd  # type: ignore[import-not-found, import-untyped]
        import numpy as np

        if not getattr(_run_coacd, "_coacd_silenced", False):
            coacd.set_log_level("error")
            _run_coacd._coacd_silenced = True  # type: ignore[attr-defined]

        cm = coacd.Mesh(
            np.asarray(mesh.vertices, dtype=np.float64),  # type: ignore[attr-defined]
            np.asarray(mesh.triangles, dtype=np.int32),  # type: ignore[attr-defined]
        )
        parts = coacd.run_coacd(
            cm,
            threshold=_COACD_THRESHOLD,
            max_convex_hull=_COACD_MAX_HULLS,
            resolution=_COACD_RESOLUTION,
            mcts_iterations=_COACD_MCTS_ITERATIONS,
            mcts_nodes=_COACD_MCTS_NODES,
        )
        return list(parts)
    except Exception as exc:
        logger.warning(
            "entity hulls: CoACD failed for %s (%s); using single convex hull", mesh_path, exc
        )
        return []
