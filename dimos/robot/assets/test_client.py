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


import pytest

from dimos.robot.assets.client import materialize_scene
from dimos.robot.assets.usd_assets import asset_hash


def layer_for(data):
    return f'''#usda 1.0
(
    defaultPrim = "mesh"
)
def Mesh "mesh" (
    assetInfo = {{
        asset identifier = @/remote/mesh.obj@
        string version = "{asset_hash(data)}"
    }}
) {{
}}
'''


def test_scene_download_is_content_checked_and_cache_location_independent(tmp_path):
    mesh = b"v 0 0 0\\nv 1 0 0\\nv 0 1 0\\nf 1 2 3\\n"

    def fetch(name):
        assert name == "/remote/mesh.obj"
        return mesh

    first = materialize_scene(layer_for(mesh), fetch, cache=tmp_path / "a")
    second = materialize_scene(layer_for(mesh), fetch, cache=tmp_path / "b")
    assert first.revision == second.revision
    assert first.usd == second.usd
    assert "/remote/" not in first.usd
    assert next((tmp_path / "b").glob("*.obj")).read_bytes() == mesh


def test_changed_asset_bytes_are_rejected(tmp_path):
    with pytest.raises(ValueError, match="asset hash mismatch"):
        materialize_scene(layer_for(b"expected"), lambda _: b"wrong", cache=tmp_path)


def test_mesh_reference_is_not_checked_against_its_source_files_hash(tmp_path):
    original = b"solid source mesh"
    referenced = b'#usda 1.0\n(defaultPrim = "mesh")\ndef Mesh "mesh" {}\n'
    files = {"/remote/mesh.obj": original, "/remote/mesh.usda": referenced}
    usd = layer_for(original).replace(
        ") {\n}", "    prepend references = @/remote/mesh.usda@\n) {\n}"
    )
    scene = materialize_scene(usd, files.__getitem__, cache=tmp_path)
    assert "/remote/" not in scene.usd
    assert (
        tmp_path / (asset_hash(referenced).replace(":", "-") + ".usda")
    ).read_bytes() == referenced
