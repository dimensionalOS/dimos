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

import trimesh

from dimos.experimental.scene_cooking.browser.collision import cook_browser_collision
from dimos.experimental.scene_cooking.mujoco.collision_policy import CollisionSpec


def _write_box_glb(path: Path) -> None:
    trimesh.creation.box(extents=(1.0, 1.0, 1.0)).export(path)


def _artifact_mtimes(result_path: Path, objects_path: Path) -> tuple[int, int]:
    return result_path.stat().st_mtime_ns, objects_path.stat().st_mtime_ns


def test_browser_collision_reuses_cache_for_same_inputs(tmp_path: Path) -> None:
    source = tmp_path / "scene.glb"
    _write_box_glb(source)
    out_dir = tmp_path / "browser"

    first = cook_browser_collision(source, out_dir, collision_spec=CollisionSpec())
    assert first is not None and first.objects_path is not None
    baseline = _artifact_mtimes(first.path, first.objects_path)

    again = cook_browser_collision(source, out_dir, collision_spec=CollisionSpec())
    assert again is not None and again.objects_path is not None
    assert _artifact_mtimes(again.path, again.objects_path) == baseline


def test_browser_collision_rebuilds_when_policy_changes(tmp_path: Path) -> None:
    source = tmp_path / "scene.glb"
    _write_box_glb(source)
    out_dir = tmp_path / "browser"

    first = cook_browser_collision(source, out_dir, collision_spec=CollisionSpec())
    assert first is not None and first.objects_path is not None
    baseline = _artifact_mtimes(first.path, first.objects_path)

    # A sidecar-level policy change must invalidate the cached artifacts
    # even though the files already exist and rebake was not requested.
    changed = cook_browser_collision(
        source, out_dir, collision_spec=CollisionSpec(tiny_prim_extent_m=0.5)
    )
    assert changed is not None and changed.objects_path is not None
    assert _artifact_mtimes(changed.path, changed.objects_path) != baseline


def test_browser_collision_rebuilds_when_manifest_missing(tmp_path: Path) -> None:
    """Packages cooked before the manifest existed rebuild once, then cache."""
    source = tmp_path / "scene.glb"
    _write_box_glb(source)
    out_dir = tmp_path / "browser"

    first = cook_browser_collision(source, out_dir, collision_spec=CollisionSpec())
    assert first is not None and first.objects_path is not None
    manifest_path = first.path.with_suffix(".manifest.json")
    assert manifest_path.exists()

    manifest_path.unlink()
    rebuilt = cook_browser_collision(source, out_dir, collision_spec=CollisionSpec())
    assert rebuilt is not None
    assert manifest_path.exists()
