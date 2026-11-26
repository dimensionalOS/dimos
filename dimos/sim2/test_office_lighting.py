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
    from dimos.sim2.mujoco.sensor_mount import SensorMount
    from dimos.sim2.mujoco.view import MujocoView
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

    floor = geom == model.geom("/office_555/Geometry/slab_ground/floor_visual").id
    walls = np.isin(
        geom,
        [
            g
            for g in range(model.ngeom)
            if model.geom(g).name.endswith("_g0_visual") and "/box_" in model.geom(g).name
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
    # scene.usda's sky, sun and paint were fit so AgX shows the Controller's walls and floor (0.83, 0.41 in Jeff's screenshot).
    assert np.median(brightness[walls]) == pytest.approx(0.83, abs=0.06)
    # MuJoCo draws no fittings, whose light reaches the floor most: 0.07 under Blender's 0.44 here.
    assert np.median(brightness[floor]) == pytest.approx(0.44 - 0.07, abs=0.03)
    # The sun through the skylights lights patches of floor brighter than the rest of it, as the Controller's 0.84.
    patches = np.percentile(brightness[floor], 97)
    assert patches == pytest.approx(0.84, abs=0.06) and patches > np.median(brightness[floor])
    # The desks keep their orange through AgX: red over green over blue, far apart.
    desk = np.median(image[desks].astype(float), axis=0) / 255
    assert desk[0] > desk[1] > desk[2] and (desk[0] - desk[2]) / desk[0] > 0.4
    # A surface authored black, by its displayColor or by the material that now carries it.
    material_black = np.append(model.mat_rgba[:, :3].max(axis=1) == 0, False)[model.geom_matid]
    black = (model.geom_rgba[:, :3].max(axis=1) == 0) | material_black
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
    # The sky's texels are scaled by the shade's headroom, as the view renders it.
    image = display(renderer.render(), view._headroom, "agx")
    renderer.close()
    # A segment id is drawn as a color, and multisampling blends those at every edge into ids no geom has.
    model.vis.quality.offsamples = 0
    segmenter = mujoco.Renderer(model, 120, 160)
    segmenter.enable_segmentation_rendering()
    segmenter.update_scene(data, camera=camera)
    sky = segmenter.render()[..., 0] < 0
    segmenter.close()
    view.close()

    assert sky.mean() > 0.05, "the skylight is in frame"
    # scene.usda's dome, white at 3.56 times the stage's 1.05 exposure, as AgX shows it.
    from dimos.sim2.mujoco.look import agx

    expected = agx(np.full((1, 3), 3.559 * 2**0.070389, np.float32))[0] * 255
    assert np.median(image[sky], axis=0) == pytest.approx(expected, abs=2)


def test_every_office_visual_is_a_mesh_with_a_material_that_blender_imports():
    from pxr import Usd, UsdGeom, UsdShade

    from dimos.robot.assets.usd_utils.lfs import pull_lfs_files
    from dimos.simulation.scenes.office_555.room import SCENE

    pull_lfs_files([SCENE])
    stage = Usd.Stage.Open(str(SCENE))
    visuals = [
        prim
        for prim in stage.Traverse()
        if prim.IsA(UsdGeom.Gprim)
        and UsdGeom.Imageable(prim).ComputePurpose() == UsdGeom.Tokens.default_
        and not prim.HasAPI("PhysicsCollisionAPI")
    ]
    # Blender 5.1 imports a Cube or Cylinder with no material, and reads no displayColor at all.
    assert {prim.GetTypeName() for prim in visuals} == {"Mesh"}
    unbound = [p for p in visuals if not UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]]
    assert unbound == []
    # The ground plane under the floor slab is the one collider still drawn: nothing sees it.
    drawn_colliders = [
        prim.GetName()
        for prim in stage.Traverse()
        if prim.HasAPI("PhysicsCollisionAPI")
        and UsdGeom.Imageable(prim).ComputePurpose() == UsdGeom.Tokens.default_
    ]
    assert drawn_colliders == ["ground"]


def test_every_file_the_office_names_hashes_to_the_sha256_it_declares():
    from dimos.core.hardware_files import declared_hashes
    from dimos.robot.assets.usd_utils.assets import asset_hash
    from dimos.robot.assets.usd_utils.lfs import pull_lfs_files, pull_usd_assets
    from dimos.simulation.scenes.office_555.room import SCENE

    pull_lfs_files([SCENE])
    pull_usd_assets([SCENE.read_text()], root=SCENE.parent)
    declared = declared_hashes(SCENE.read_text())
    assert declared
    stale = [
        name
        for name, sha256 in declared.items()
        if sha256 is not None and asset_hash((SCENE.parent / name).read_bytes()) != sha256
    ]
    assert not stale, f"rehash these in scene.usda, or a renderer refuses them: {stale}"
