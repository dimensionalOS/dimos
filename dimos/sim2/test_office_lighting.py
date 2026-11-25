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

import numpy as np
import pytest

pytest.importorskip("pxr")


@pytest.mark.self_hosted
def test_the_office_camera_sees_the_room_as_its_lighting_notes_describe_it(renders):
    import mujoco

    from dimos.robot.unitree.g1.blueprints.basic.three_g1_office_555 import Office555Config
    from dimos.sim2.mujoco.view import MujocoView
    from dimos.sim2.mujoco.world import SensorMount
    from dimos.sim2.usd_layer import usd_layer

    view = MujocoView(
        usd_layer(Office555Config().usd), (), SensorMount("camera", "overview", "cam")
    )
    image = view.render("cam", 640, 480)
    brightness = image.astype(float).mean(axis=2) / 255
    model, data = view.model, view.data
    renderer = mujoco.Renderer(model, 480, 640)
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera="cam")
    geom = renderer.render()[..., 0]
    renderer.close()
    view.close()

    floor = geom == model.geom("/office_555/Geometry/slab_ground/floor").id
    walls = np.isin(
        geom,
        [
            g
            for g in range(model.ngeom)
            if model.geom(g).name.endswith(("_g0",)) and "/box_" in model.geom(g).name
        ],
    )
    desks = np.isin(
        geom,
        [
            g
            for g in range(model.ngeom)
            if "/desk_" in model.geom(g).name and model.geom(g).name.endswith("_g0")
        ],
    )
    # scene.usda's lights and albedos were sized so AgX shows the Controller's room: its walls at 0.83 and its floor at 0.41 (Jeff's screenshot).
    assert np.median(brightness[walls]) == pytest.approx(0.83, abs=0.06)
    assert np.median(brightness[floor]) == pytest.approx(0.41, abs=0.06)
    # The sun through the skylights lights patches of floor as bright as the walls (0.84 in the screenshot).
    assert np.percentile(brightness[floor], 97) > 0.75
    # The desks keep their orange through AgX: red over green over blue, far apart.
    desk = np.median(image[desks].astype(float), axis=0) / 255
    assert desk[0] > desk[1] > desk[2] and (desk[0] - desk[2]) / desk[0] > 0.4
    black = (model.geom_rgba[:, :3].max(axis=1) == 0) & (model.geom_matid < 0)
    painted = (geom >= 0) & ~black[geom]
    assert brightness[painted].min() > 0.02, "only a surface authored black renders black"
    # AgX rolls even the sunlit floor off below white.
    assert (image[geom >= 0] >= 254).all(axis=-1).mean() == 0


@pytest.mark.self_hosted
def test_the_offices_skylights_show_its_dome_as_the_sky(renders):
    import mujoco

    from dimos.robot.unitree.g1.blueprints.basic.three_g1_office_555 import Office555Config
    from dimos.sim2.mujoco.look import display
    from dimos.sim2.mujoco.view import MujocoView
    from dimos.sim2.usd_layer import usd_layer

    view = MujocoView(usd_layer(Office555Config().usd))
    model, data = view.model, view.data
    camera = mujoco.MjvCamera()
    # From the floor below skylight_1, looking up through it.
    camera.lookat = data.xpos[model.body("skylight_1").id]
    camera.azimuth, camera.elevation, camera.distance = 90, 70, 5
    renderer = mujoco.Renderer(model, 120, 160)
    renderer.update_scene(data, camera=camera)
    image = display(renderer.render(), view._headroom, "agx")
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera=camera)
    sky = renderer.render()[..., 0] < 0
    renderer.close()
    view.close()

    assert sky.mean() > 0.05, "the skylight is in frame"
    # scene.usda's dome: white at 2.5 (times the stage's 1.05 exposure), which AgX shows at 0.88.
    assert image[sky].min() > 200
