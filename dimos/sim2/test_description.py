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

from dataclasses import replace

from pydantic import TypeAdapter
import pytest

from dimos.robot.assets.model import RobotModel
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.description import Instance, SceneConfig


def test_cli_configuration_roundtrip_keeps_physical_model_variants(world):
    adapter = TypeAdapter(SceneConfig)
    restored = adapter.validate_python(adapter.dump_python(world))
    assert restored.revision == world.revision
    assert restored.instances[0].model.load().root_link == "left/base_link"


def test_model_owner_resolves_mounts_once_for_later_consumers(tmp_path):
    source = tmp_path / "robot.urdf"
    source.write_text('<robot name="r"><link name="base"/></robot>')
    configured = (
        RobotModel.from_file(source)
        .with_fixed_frame("camera", "base", xyz=(0.1, 0, 0.3))
        .with_prefix("robot/")
    )
    manager = UrdfManager(models={"robot": configured})
    try:
        models = manager.resolved_models()
        world = SceneConfig((Instance("robot"),)).resolve(models)
        # A reader receives the resolved text, not a path it must reload.
        source.write_text('<robot name="changed"><link name="different"/></robot>')
        resolved = world.instances[0].description
        assert resolved.xml == models["robot"].xml
        assert resolved.root_link == "robot/base"
        assert resolved.joints[0].child_link == "robot/camera"
        assert world.revision == SceneConfig((Instance("robot"),)).resolve(models).revision
        with pytest.raises(ValueError, match="conflicting model"):
            conflicting = RobotModel.from_file(source).with_prefix("different/")
            replace(world, instances=(Instance("robot", conflicting),)).resolve(models)
    finally:
        manager.dispose()
