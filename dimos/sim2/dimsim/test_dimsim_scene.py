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
from typing import Any

import pytest

from dimos.sim2.dimsim import dimsim_scene

pytest.importorskip("pxr")
trimesh = pytest.importorskip("trimesh")


@pytest.fixture
def pxr() -> Any:
    from dimos.robot.assets.usd_utils.pxr import load_pxr

    return load_pxr()


@pytest.fixture
def meshes(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    cache = tmp_path / "meshes"
    monkeypatch.setattr(dimsim_scene, "_MESHES", cache)
    return cache


def _glb(path: Path, visual: Any = None) -> Path:
    box = trimesh.creation.box()
    if visual is not None:
        box.visual = visual
    path.write_bytes(trimesh.Scene(box).export(file_type="glb"))
    return path


def test_the_mesh_cache_is_made_for_this_user_alone(pxr: Any, meshes: Path, tmp_path: Path) -> None:
    dimsim_scene._mesh_file(pxr, _glb(tmp_path / "box.glb"), "test")
    assert meshes.stat().st_mode & 0o777 == 0o700


def test_a_mesh_cache_others_can_write_to_is_refused(
    pxr: Any, meshes: Path, tmp_path: Path
) -> None:
    meshes.mkdir()
    meshes.chmod(0o777)
    with pytest.raises(PermissionError, match="writable by you alone"):
        dimsim_scene._mesh_file(pxr, _glb(tmp_path / "box.glb"), "test")


def test_the_mesh_cache_is_not_in_the_shared_temporary_directory() -> None:
    import tempfile

    from dimos.constants import CACHE_DIR

    assert dimsim_scene._MESHES.is_relative_to(CACHE_DIR)
    assert not dimsim_scene._MESHES.is_relative_to(tempfile.gettempdir())
