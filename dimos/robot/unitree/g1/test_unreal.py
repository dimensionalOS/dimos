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

"""The G1 scene can be exported to Unreal after resetting its upstream environment."""

from contextlib import closing
from functools import partial

import pytest

from dimos.robot.unitree.g1.simulation import g1_model
from dimos.sim2.environment import MotorScene
from dimos.sim2.sensors.camera.unreal.assets import export_scene
from dimos.sim2.world.scene import scene_path


@pytest.mark.self_hosted
def test_full_g1_logistics_world_exports_after_reset_without_injected_ui_targets():
    path = scene_path(None, "logistics.xml")
    with closing(MotorScene(path, {"g1": partial(g1_model, path)})) as environment:
        environment.reset()
        model, data = environment.sim.model._model, environment.sim.data._data
        manifest = export_scene(model, data)

        assert model.nu == 29
        assert {geom["id"] for geom in manifest["geoms"]} == set(range(model.ngeom))
        assert any(model.geom(geom["id"]).name.startswith("robotg1_") for geom in manifest["geoms"])
        assert {model.geom(i).name for i in range(model.ngeom)}.isdisjoint(
            {
                "left_eef_target_box",
                "left_eef_target_sphere",
                "right_eef_target_box",
                "right_eef_target_sphere",
            }
        )
