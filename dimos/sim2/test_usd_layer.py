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

from __future__ import annotations

from pathlib import Path

import pytest

from dimos.robot.assets.usd_utils.assets import asset_hash
from dimos.sim2 import usd_layer
from dimos.sim2.usd_layer import scene_as_files

pytest.importorskip("pxr")

_SCENE = """#usda 1.0
(
    subLayers = [@/dimos/assets/lights.usda@]
)

def Xform "room" (
    append references = @/dimos/assets/chair.usda@
    payload = @/dimos/assets/table.usda@
)
{
    def Mesh "rug" (
        assetInfo = {
            asset identifier = @/dimos/assets/rug.obj@
            string version = "%s"
        }
    )
    {
    }
    asset[] textures = [@/dimos/assets/wood.png@]
}
"""


@pytest.fixture(autouse=True)
def _fresh_cache(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(usd_layer, "_CACHE", tmp_path / "layers")
    monkeypatch.setattr(usd_layer, "_ASSETS", tmp_path / "assets")


def test_every_asset_a_scene_names_is_fetched_to_disk() -> None:
    files = {
        name: f"{name} bytes".encode()
        for name in ("lights.usda", "chair.usda", "table.usda", "rug.obj", "wood.png")
    }
    fetched: list[str] = []

    def fetch(name: str) -> bytes:
        fetched.append(name)
        return files[Path(name).name]

    scene = scene_as_files(_SCENE % asset_hash(files["rug.obj"]), fetch).read_text()

    assert "/dimos/assets/" not in scene
    assert sorted(Path(name).name for name in fetched) == sorted(files)
    for data in files.values():
        assert asset_hash(data) in scene


def test_a_hashed_asset_already_on_disk_is_not_fetched_again() -> None:
    rug = b"a rug fetched before"
    names: list[str] = []

    def fetch(name: str) -> bytes:
        names.append(name)
        return rug if name.endswith(".obj") else b"other"

    scene = _SCENE % asset_hash(rug)
    scene_as_files(scene, fetch)
    names.clear()
    scene_as_files(scene, fetch)

    assert "/dimos/assets/rug.obj" not in names
