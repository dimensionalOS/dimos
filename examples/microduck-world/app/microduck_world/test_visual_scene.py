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

import base64
import gzip
import json
from io import BytesIO

import mujoco
import numpy as np
import pytest
from microduck_world.comparison import COMPARE_OFFSET, comparison_camera, comparison_requested
from microduck_world.visual_scene import body_snapshot, export_scene, write_scene
from PIL import Image


@pytest.fixture
def model():
    return mujoco.MjModel.from_xml_string("""
    <mujoco><asset><mesh name="shape" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/></asset>
    <worldbody>
      <geom name="floor" type="plane" size="2 2 .1"/>
      <body name="trunk_base" pos="1 2 3" euler="0 0 90">
        <freejoint/>
        <geom name="visual" type="mesh" mesh="shape" group="2" pos=".2 0 .1"/>
        <geom name="collision" type="box" size=".1 .1 .1" group="3"/>
        <body name="limb" pos=".3 0 0">
          <camera name="test_pov" pos=".1 .2 .3" euler="10 20 30" fovy="70"/>
          <joint name="hinge" type="hinge" axis="0 0 1"/>
          <geom name="limb_visual" type="capsule" size=".02 .1" pos=".1 0 0"/>
        </body>
      </body>
    </worldbody></mujoco>""")


def test_visual_groups_and_compiled_mesh_buffers(model):
    scene = export_scene(model, {})
    assert [geom["name"] for geom in scene["geoms"]] == ["floor", "visual", "limb_visual"]
    mesh = scene["meshes"]["0"]
    np.testing.assert_array_equal(
        np.frombuffer(base64.b64decode(mesh["positions"]), dtype="<f4").reshape(-1, 3),
        model.mesh_vert,
    )
    np.testing.assert_array_equal(
        np.frombuffer(base64.b64decode(mesh["indices"]), dtype="<u4").reshape(-1, 3),
        model.mesh_face,
    )


def test_exported_body_and_local_geom_transforms_match_mujoco(model):
    data = mujoco.MjData(model)
    data.qpos[0:3] = [2, -1, 4]
    data.qpos[-1] = 0.7
    mujoco.mj_forward(model, data)
    scene = export_scene(model, {})
    poses = dict(zip(scene["bodyIds"], body_snapshot(data, scene["bodyIds"]), strict=True))
    for geom in scene["geoms"]:
        pose = poses[geom["body"]]
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, np.array(pose[3:]))
        position = np.array(pose[:3]) + rotation.reshape(3, 3) @ geom["position"]
        np.testing.assert_allclose(position, data.geom_xpos[geom["id"]], atol=2e-6)
        local_rotation = np.zeros(9)
        mujoco.mju_quat2Mat(local_rotation, np.array(geom["quaternion"]))
        orientation = rotation.reshape(3, 3) @ local_rotation.reshape(3, 3)
        np.testing.assert_allclose(orientation.ravel(), data.geom_xmat[geom["id"]], atol=2e-6)
    assert poses[scene["focusBody"]][:3] == [2.0, -1.0, 4.0]


def test_model_assets_are_immutable_and_gzip_matches(model, tmp_path):
    scene = export_scene(model, {"background": "#ddeeff"})
    first = write_scene(scene, tmp_path)
    assert write_scene(scene, tmp_path) == first
    encoded = (tmp_path / first).read_bytes()
    assert gzip.decompress((tmp_path / (first + ".gz")).read_bytes()) == encoded
    assert json.loads(encoded)["appearance"] == {"background": "#ddeeff"}
    scene["appearance"]["background"] = "#ffffff"
    second = write_scene(scene, tmp_path)
    assert second != first
    assert (tmp_path / first).read_bytes() == encoded


def test_exported_camera_matches_mujoco_after_joint_motion(model):
    data = mujoco.MjData(model)
    data.qpos[-1] = 0.7
    mujoco.mj_forward(model, data)
    scene = export_scene(model, {}, "test_pov")
    camera = scene["camera"]
    poses = dict(zip(scene["bodyIds"], body_snapshot(data, scene["bodyIds"]), strict=True))
    pose = poses[camera["body"]]
    rotation, local = np.zeros(9), np.zeros(9)
    mujoco.mju_quat2Mat(rotation, np.array(pose[3:]))
    mujoco.mju_quat2Mat(local, np.array(camera["quaternion"]))
    np.testing.assert_allclose(
        np.array(pose[:3]) + rotation.reshape(3, 3) @ camera["position"],
        data.cam_xpos[0],
        atol=2e-6,
    )
    np.testing.assert_allclose(
        (rotation.reshape(3, 3) @ local.reshape(3, 3)).ravel(), data.cam_xmat[0], atol=2e-6
    )
    assert camera["fovy"] == 70
    assert camera["near"] == pytest.approx(model.vis.map.znear * model.stat.extent)
    assert camera["far"] == pytest.approx(model.vis.map.zfar * model.stat.extent)


def test_comparison_camera_matches_three_follow_position(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    scene = mujoco.MjvScene(model, maxgeom=100)
    camera = comparison_camera(data, 1)
    mujoco.mjv_updateScene(
        model, data, mujoco.MjvOption(), None, camera, mujoco.mjtCatBit.mjCAT_ALL, scene
    )
    midpoint = (scene.camera[0].pos + scene.camera[1].pos) / 2
    np.testing.assert_allclose(midpoint, data.xpos[1] + COMPARE_OFFSET, atol=1e-6)


@pytest.mark.parametrize(
    "stats,expected",
    [
        ({"perRobot": {"duck": {"subs": ["world_compare_image"]}}}, True),
        ({"perRobot": {"duck": {"subs": ["world_state", "color_image"]}}}, False),
        ({"perRobot": {}}, False),
        ({"perRobot": []}, False),
        ({"perRobot": {"duck": {"subs": "world_compare_image"}}}, False),
        (None, False),
    ],
)
def test_comparison_only_renders_for_actual_jpeg_subscribers(stats, expected):
    assert comparison_requested(stats) is expected


def test_turf_texture_keeps_compiled_pixels_and_object_repeats():
    model = mujoco.MjModel.from_xml_string("""
    <mujoco><asset>
      <texture name="turf" type="2d" builtin="checker" width="8" height="4"
               rgb1=".1 .4 .2" rgb2=".2 .5 .3"/>
      <material name="grass" texture="turf" texuniform="false" texrepeat="1 2"/>
    </asset><worldbody>
      <geom name="field" type="box" size="3 2 .05" material="grass"/>
      <geom name="other" type="box" size="1 1 .05" pos="0 0 -1" material="grass"/>
    </worldbody></mujoco>""")
    scene = export_scene(model, {})
    assert len(scene["textures"]) == 1
    assert all(g["texture"] == {"id": "0", "repeat": [1.0, 2.0]} for g in scene["geoms"])
    with Image.open(BytesIO(base64.b64decode(scene["textures"]["0"]))) as image:
        np.testing.assert_array_equal(np.asarray(image), model.tex_data.reshape(4, 8, 3))
