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

# coacd must load after open3d: importing coacd first segfaults open3d's
# extension module (clashing vendored native libs). Plain import (not
# importorskip) so collection fails loudly if coacd is missing.
# isort: off
import open3d as o3d
import coacd  # noqa: F401

# isort: on

from dimos.experimental.scene_cooking.entities.collision import cook_entity_collision_hulls


def _write_box_mesh(path: Path) -> None:
    mesh = o3d.geometry.TriangleMesh.create_box(0.1, 0.1, 0.1)
    o3d.io.write_triangle_mesh(str(path), mesh)


def test_cook_entity_collision_hulls(tmp_path: Path) -> None:
    source = tmp_path / "visual.obj"
    _write_box_mesh(source)
    out_dir = tmp_path / "mujoco_collision"

    hulls = cook_entity_collision_hulls(source, out_dir)

    assert hulls, "expected at least one hull"
    assert all(path.exists() for path in hulls)
    assert all(path.parent == out_dir for path in hulls)
    assert all(path.name.startswith("hull_") for path in hulls)


def test_cook_entity_collision_hulls_is_idempotent(tmp_path: Path) -> None:
    source = tmp_path / "visual.obj"
    _write_box_mesh(source)
    out_dir = tmp_path / "mujoco_collision"

    first = cook_entity_collision_hulls(source, out_dir)
    mtimes = [path.stat().st_mtime_ns for path in first]

    second = cook_entity_collision_hulls(source, out_dir)
    assert second == first
    assert [path.stat().st_mtime_ns for path in second] == mtimes

    rebaked = cook_entity_collision_hulls(source, out_dir, rebake=True)
    assert rebaked
    assert all(path.exists() for path in rebaked)


def test_cook_entity_collision_hulls_unreadable_mesh(tmp_path: Path) -> None:
    source = tmp_path / "visual.obj"
    source.write_text("not a mesh\n")

    assert cook_entity_collision_hulls(source, tmp_path / "mujoco_collision") == []
