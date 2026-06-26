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

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from dimos.simulation.mujoco.entity_scene import add_entities_to_spec, entity_body_name


def _write_hull_obj(path: Path) -> None:
    o3d = pytest.importorskip("open3d")
    path.parent.mkdir(parents=True, exist_ok=True)
    mesh = o3d.geometry.TriangleMesh.create_box(0.1, 0.1, 0.1)
    o3d.io.write_triangle_mesh(str(path), mesh)


def _mesh_entity(entity_id: str, collision_paths: list[str] | None) -> dict[str, Any]:
    entity: dict[str, Any] = {
        "id": entity_id,
        "initial_pose": {"x": 0.0, "y": 0.0, "z": 0.5, "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0},
        "aabb": {"min": [-0.05, -0.05, 0.45], "max": [0.05, 0.05, 0.55]},
        "descriptor": {
            "entity_id": entity_id,
            "kind": "dynamic",
            "shape_hint": "mesh",
            "extents": [],
            "mass": 1.0,
        },
        "physics": {"shape": "mesh"},
    }
    if collision_paths is not None:
        entity["collision_paths"] = collision_paths
    return entity


def test_mesh_entity_loads_cooked_hulls(tmp_path: Path) -> None:
    mujoco = pytest.importorskip("mujoco")
    hulls = [tmp_path / "mujoco_collision" / f"hull_{i:03d}.obj" for i in range(2)]
    for hull in hulls:
        _write_hull_obj(hull)

    spec = mujoco.MjSpec()
    add_entities_to_spec(spec, [_mesh_entity("prop", [str(h) for h in hulls])])
    model = spec.compile()

    assert model.nmesh == 2
    geom_types = {
        model.geom_type[i]
        for i in range(model.ngeom)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "").startswith(
            entity_body_name("prop")
        )
    }
    assert geom_types == {mujoco.mjtGeom.mjGEOM_MESH}


def test_mesh_entity_without_hulls_falls_back_to_box(tmp_path: Path) -> None:
    mujoco = pytest.importorskip("mujoco")

    spec = mujoco.MjSpec()
    add_entities_to_spec(spec, [_mesh_entity("prop", None)])
    model = spec.compile()

    assert model.nmesh == 0
    assert model.ngeom == 1
    assert model.geom_type[0] == mujoco.mjtGeom.mjGEOM_BOX


def test_mesh_entity_with_missing_hull_files_falls_back_to_box(tmp_path: Path) -> None:
    mujoco = pytest.importorskip("mujoco")
    present = tmp_path / "mujoco_collision" / "hull_000.obj"
    _write_hull_obj(present)
    missing = tmp_path / "mujoco_collision" / "hull_001.obj"

    spec = mujoco.MjSpec()
    add_entities_to_spec(spec, [_mesh_entity("prop", [str(present), str(missing)])])
    model = spec.compile()

    assert model.nmesh == 0
    assert model.geom_type[0] == mujoco.mjtGeom.mjGEOM_BOX
