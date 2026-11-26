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

from dimos.robot.assets.model import UsdModel
from dimos.sim2.description import SceneConfig, compose_scene, scene_revision


@pytest.fixture
def scene():
    return UsdModel.from_layer('#usda 1.0\n(defaultPrim = "world")\ndef Xform "world" {}\n')


def test_scene_selection_survives_config_serialization(scene):
    restored = SceneConfig.model_validate(SceneConfig(scene=scene).model_dump())
    assert restored.scene.usd == scene.usd
    assert scene_revision(restored.scene) == scene_revision(scene)


def test_scene_defaults_to_hardware_composition():
    assert SceneConfig().scene is None


def test_composition_requires_a_root_hardware_module():
    with pytest.raises(ValueError, match="exactly one unmounted"):
        compose_scene([])


def test_scene_identity_does_not_depend_on_a_consumers_cache_directory(tmp_path):
    scenes = []
    for host in ("physics", "renderer"):
        cache = tmp_path / host
        cache.mkdir()
        mesh = cache / "mesh.usda"
        mesh.write_text('#usda 1.0\n(defaultPrim = "shape")\ndef Cube "shape" {}\n')
        scenes.append(
            UsdModel.from_layer(f"""#usda 1.0
(defaultPrim = "world")
def Xform "world" {{
    def Xform "object" (prepend references = @{mesh}@) {{}}
}}
""")
        )
    assert scenes[0].usd != scenes[1].usd
    assert scene_revision(scenes[0]) == scene_revision(scenes[1])
    changed = UsdModel.from_layer(scenes[1].usd.replace('"object"', '"another_object"'))
    assert scene_revision(changed) != scene_revision(scenes[0])
