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

from pathlib import Path
import re

import trimesh

from dimos.manipulation.planning.utils.mesh_utils import _convert_meshes


def _write_box_mesh(path: Path, extents: tuple[float, float, float]) -> None:
    trimesh.creation.box(extents=extents).export(str(path))


def test_convert_meshes_same_stem_different_dirs_stay_distinct(tmp_path: Path) -> None:
    """Visual and collision meshes often share a file stem; converted OBJs
    must not overwrite each other."""
    visual_dir = tmp_path / "visual"
    collision_dir = tmp_path / "collision"
    visual_dir.mkdir()
    collision_dir.mkdir()
    _write_box_mesh(visual_dir / "link3.stl", (1.0, 1.0, 1.0))
    _write_box_mesh(collision_dir / "link3.stl", (2.0, 2.0, 2.0))

    urdf = (
        f'<mesh filename="{visual_dir / "link3.stl"}"/>'
        f'<mesh filename="{collision_dir / "link3.stl"}"/>'
    )
    converted = _convert_meshes(urdf, tmp_path)

    obj_paths = [Path(p) for p in re.findall(r'filename="([^"]+\.obj)"', converted)]
    assert len(obj_paths) == 2
    assert obj_paths[0] != obj_paths[1]
    sizes = sorted(trimesh.load(str(p), force="mesh").extents[0] for p in obj_paths)
    assert sizes[0] == 1.0
    assert sizes[1] == 2.0


def test_convert_meshes_same_file_referenced_twice_converts_once(tmp_path: Path) -> None:
    mesh = tmp_path / "part.stl"
    _write_box_mesh(mesh, (1.0, 1.0, 1.0))

    urdf = f'<mesh filename="{mesh}"/><mesh filename="{mesh}"/>'
    converted = _convert_meshes(urdf, tmp_path)

    obj_paths = set(re.findall(r'filename="([^"]+\.obj)"', converted))
    assert len(obj_paths) == 1
