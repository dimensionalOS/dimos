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
from dimos.sim2.geometry.mujoco import compile_scene


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
        contribution = manager.robot_models()
        senders = (("urdf_manager", contribution), ("unrelated", {"other_config": True}))
        world = SceneConfig((Instance("robot"),)).resolve(senders)
        # A reader receives the resolved text, not a path it must reload.
        source.write_text('<robot name="changed"><link name="different"/></robot>')
        resolved = world.instances[0].description
        assert resolved.xml == contribution["robot_models"]["robot"].xml
        assert resolved.root_link == "robot/base"
        assert resolved.joints[0].child_link == "robot/camera"
        assert world.revision == SceneConfig((Instance("robot"),)).resolve(senders).revision
        with pytest.raises(ValueError, match="conflicting model"):
            conflicting = RobotModel.from_file(source).with_prefix("different/")
            replace(world, instances=(Instance("robot", conflicting),)).resolve(senders)
    finally:
        manager.dispose()


def test_shared_models_combine_multiple_providers_and_reject_duplicate_ids(world):
    first, second = world.instances[:2]
    senders = (
        ("first_manager", {"robot_models": {first.id: first.description}}),
        ("second_manager", {"robot_models": {second.id: second.description}}),
    )
    requested = SceneConfig((Instance(first.id), Instance(second.id)))
    resolved = requested.resolve(senders)
    assert resolved.instances[0].description.xml == first.description.xml
    assert resolved.instances[1].description.xml == second.description.xml
    with pytest.raises(ValueError, match=f"duplicate shared model {first.id}"):
        requested.resolve((*senders, ("duplicate_manager", senders[0][1])))


def test_missing_shared_model_fails_during_configuration():
    with pytest.raises(ValueError, match="model missing has not been resolved"):
        SceneConfig((Instance("missing"),)).resolve(())


def test_collision_exclusions_change_scene_identity_and_bind_bodies(world):
    first = replace(world.instances[0], collision_exclusions=(("left/base_link", "left/tool"),))
    configured = replace(world, instances=(first, *world.instances[1:]))
    assert configured.revision != world.revision
    model = compile_scene(configured)
    expected = (model.body("left/base_link").id << 16) + model.body("left/tool").id
    assert expected in model.exclude_signature


def test_collision_exclusion_rejects_unknown_body(world):
    first = replace(world.instances[0], collision_exclusions=(("left/base_link", "missing"),))
    with pytest.raises(ValueError, match="invalid collision exclusion"):
        compile_scene(replace(world, instances=(first,)))
