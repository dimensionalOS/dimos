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
    # scene.usda: "measured in frame, the floor sits around 0.44 and no pixel falls into black at all"
    assert np.median(brightness[floor]) == pytest.approx(0.44, abs=0.1)
    black = (model.geom_rgba[:, :3].max(axis=1) == 0) & (model.geom_matid < 0)
    painted = (geom >= 0) & ~black[geom]
    assert brightness[painted].min() > 0.02, "only a surface authored black renders black"
    # The stage's neutral tone curve rolls bright walls off below white rather than clipping them.
    assert (image[geom >= 0] >= 254).any(axis=-1).mean() == 0
