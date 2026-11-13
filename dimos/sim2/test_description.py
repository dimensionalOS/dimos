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

from pydantic import ValidationError
import pytest

from dimos.robot.assets.model import UsdModel
from dimos.robot.usd_manager import Mount, UsdManager
from dimos.sim2.description import SceneConfig


@pytest.fixture
def scene():
    return UsdModel.from_layer('#usda 1.0\n(defaultPrim = "world")\ndef Xform "world" {}\n')


def test_scene_selection_survives_config_serialization(scene):
    config = SceneConfig(scene=scene)
    restored = SceneConfig.model_validate(config.model_dump())
    assert restored.scene.usd == scene.usd
    assert restored.scene.revision == scene.revision


def test_scene_manager_reference_accepts_the_same_blueprint_as_mounts():
    manager = UsdManager.blueprint(instance_name="scene")
    config = SceneConfig(scene_manager=manager)
    restored = SceneConfig.model_validate(config.model_dump())
    assert restored.scene_manager == Mount.of(manager)


def test_scene_selection_requires_one_authority(scene):
    with pytest.raises(ValidationError, match="exactly one"):
        SceneConfig()
    with pytest.raises(ValidationError, match="exactly one"):
        SceneConfig(scene=scene, scene_manager="scene")
