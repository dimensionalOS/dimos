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

"""Geometry and camera contracts of the MuJoCo → Unreal boundary."""

import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from numpy.testing import assert_allclose
import pytest
from scipy.spatial.distance import cdist
from scipy.spatial.transform import Rotation

from dimos.sim2.unreal.assets import camera_parameters, export_scene, geom_poses


def _compiled(xml):
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def _compiled_with_assets(metadata):
    root = ET.fromstring("""
        <mujoco><worldbody>
          <geom name="collider" type="box" size="1 1 1" pos="3 0 0"/>
          <geom name="shelf_visual" type="sphere" size=".001" mass="0"
            contype="0" conaffinity="0" rgba="0 0 0 0" pos="1 -2 3" euler="0 0 90"/>
          <body pos="0 1 2">
            <freejoint/>
            <inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/>
            <geom name="moving_visual" type="sphere" size=".001" mass="0"
              contype="0" conaffinity="0"/>
          </body>
        </worldbody></mujoco>
    """)
    ET.SubElement(
        ET.SubElement(root, "custom"),
        "text",
        name="dimos_unreal_assets",
        data=json.dumps(metadata),
    )
    return _compiled(ET.tostring(root, encoding="unicode"))


def test_native_assets_replace_markers_and_follow_static_and_dynamic_poses():
    model, data = _compiled_with_assets(
        {
            "version": 1,
            "geoms": {
                "shelf_visual": {"asset": "/Game/Props/Shelf.Shelf", "scale": [1, 2, 3]},
                "moving_visual": {"asset": "/Game/Props/Box.Box", "scale": [0.5, 1, 2]},
            },
        }
    )
    scene = export_scene(model, data)
    collider, shelf, moving = scene["geoms"]

    assert collider["mesh_id"] == scene["meshes"][0]["id"]
    assert len(scene["meshes"]) == 1
    assert set(shelf) == {"id", "position", "quaternion", "static_mesh", "scale"}
    assert shelf["static_mesh"] == "/Game/Props/Shelf.Shelf"
    assert shelf["scale"] == [1, 2, 3]
    assert_allclose(shelf["position"], [100, 200, 300])
    assert_allclose(shelf["quaternion"], [0, 0, -np.sqrt(0.5), np.sqrt(0.5)])
    assert moving == {
        "id": 2,
        "position": [0, -100, 200],
        "quaternion": [0, 0, 0, 1],
        "static_mesh": "/Game/Props/Box.Box",
        "scale": [0.5, 1, 2],
    }

    data.qpos[:3] += [1, 2, 3]
    mujoco.mj_forward(model, data)
    static_pose, dynamic_pose = geom_poses(data, [shelf["id"], moving["id"]])
    assert_allclose(static_pose["position"], shelf["position"])
    assert_allclose(dynamic_pose["position"], [100, -300, 500])


def test_hidden_collider_still_returns_physics_rays_but_is_not_rendered():
    model, data = _compiled_with_assets(
        {
            "version": 1,
            "geoms": {
                "shelf_visual": {"asset": "/Game/Props/Shelf.Shelf", "scale": [1, 1, 1]},
            },
            "hidden_geoms": ["collider"],
        }
    )
    scene = export_scene(model, data)
    hit = np.full(1, -1, dtype=np.int32)
    distance = mujoco.mj_ray(model, data, np.zeros(3), np.array([1.0, 0, 0]), None, True, -1, hit)

    assert [geom["id"] for geom in scene["geoms"]] == [1, 2]
    assert scene["geoms"][0]["static_mesh"] == "/Game/Props/Shelf.Shelf"
    assert distance == 2
    assert hit.tolist() == [0]
    assert model.geom_rgba[0, 3] == 1
    assert model.geom_contype[0] == 1


def test_native_binding_rejects_missing_geom():
    model, data = _compiled_with_assets(
        {
            "version": 1,
            "geoms": {"missing": {"asset": "/Game/Props/Shelf.Shelf", "scale": [1, 1, 1]}},
        }
    )
    with pytest.raises(ValueError, match="unknown geom 'missing'"):
        export_scene(model, data)


@pytest.mark.parametrize(
    "scale",
    [[0, 1, 1], [-1, 1, 1], [np.inf, 1, 1], [np.nan, 1, 1], [True, 1, 1], ["1", 1, 1], [1, 1]],
)
def test_native_binding_rejects_invalid_scale(scale):
    model, data = _compiled_with_assets(
        {
            "version": 1,
            "geoms": {"shelf_visual": {"asset": "/Game/Props/Shelf.Shelf", "scale": scale}},
        }
    )
    with pytest.raises(ValueError, match="invalid dimos_unreal_assets"):
        export_scene(model, data)


@pytest.mark.parametrize(
    "metadata",
    [
        {"version": True, "geoms": {}},
        {"version": 2, "geoms": {}},
        {"version": 1, "geoms": {}, "unexpected": 3},
        {
            "version": 1,
            "geoms": {"shelf_visual": {"asset": "Shelf", "scale": [1, 1, 1]}},
        },
        {
            "version": 1,
            "geoms": {"shelf_visual": {"asset": "/Plugin/Shelf.Shelf", "scale": [1, 1, 1]}},
        },
        {
            "version": 1,
            "geoms": {
                "shelf_visual": {
                    "asset": "/Game/Shelf.Shelf",
                    "scale": [1, 1, 1],
                    "rgba": [1, 0, 0, 1],
                }
            },
        },
    ],
)
def test_native_asset_metadata_rejects_unknown_schema_and_unsupported_asset_paths(metadata):
    model, data = _compiled_with_assets(metadata)
    with pytest.raises(ValueError, match="invalid dimos_unreal_assets"):
        export_scene(model, data)


@pytest.mark.parametrize(
    ("hidden", "error"),
    [
        (["missing"], "unknown geom 'missing'"),
        (["collider", "collider"], "duplicate names"),
        (["shelf_visual"], "cannot hide native asset markers"),
    ],
)
def test_hidden_geom_bindings_reject_missing_duplicate_or_native_markers(hidden, error):
    model, data = _compiled_with_assets(
        {
            "version": 1,
            "geoms": {"shelf_visual": {"asset": "/Game/Shelf.Shelf", "scale": [1, 1, 1]}},
            "hidden_geoms": hidden,
        }
    )
    with pytest.raises(ValueError, match=error):
        export_scene(model, data)


def test_mesh_export_preserves_source_scale_and_authored_pose_once():
    model, data = _compiled("""
        <mujoco>
          <asset>
            <mesh name="tetra" scale=".2 .3 .4"
              vertex="1 2 3  3 2 3  1 5 3  1 2 7"
              face="0 2 1  0 1 3  0 3 2  1 2 3"/>
          </asset>
          <worldbody>
            <body pos="2 -3 4" axisangle="0 0 1 90">
              <geom type="mesh" mesh="tetra" pos=".4 -.5 .6" axisangle="1 0 0 90"/>
            </body>
          </worldbody>
        </mujoco>
    """)
    scene = export_scene(model, data)
    mesh, geom = scene["meshes"][0], scene["geoms"][0]
    exported = Rotation.from_quat(geom["quaternion"]).apply(mesh["vertices"]) + geom["position"]

    source = np.array([[1, 2, 3], [3, 2, 3], [1, 5, 3], [1, 2, 7]]) * [0.2, 0.3, 0.4]
    local = Rotation.from_euler("x", 90, degrees=True).apply(source) + np.array([0.4, -0.5, 0.6])
    world = Rotation.from_euler("z", 90, degrees=True).apply(local) + np.array([2, -3, 4])
    expected = world * [100, -100, 100]
    # Compiler centering and normal seams can change vertex order/count.
    assert_allclose(cdist(expected, exported).min(axis=1), 0, atol=2e-5)
    assert_allclose(cdist(exported, expected).min(axis=1), 0, atol=2e-5)


def test_shared_mesh_has_independent_poses_and_export_never_steps_physics():
    model, data = _compiled("""
        <mujoco>
          <asset><mesh name="tetra" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/></asset>
          <worldbody>
            <geom type="mesh" mesh="tetra" pos="3 0 0"/>
            <body pos="0 1 2"><freejoint/><geom type="mesh" mesh="tetra"/></body>
          </worldbody>
        </mujoco>
    """)
    before = data.qpos.copy()
    scene = export_scene(model, data)
    assert len(scene["meshes"]) == 1
    assert scene["geoms"][0]["mesh_id"] == scene["geoms"][1]["mesh_id"]
    assert_allclose(data.qpos, before)
    assert data.time == 0

    data.qpos[:3] += [1, 2, 3]
    mujoco.mj_forward(model, data)
    poses = geom_poses(data, [0, 1])
    assert_allclose(poses[0]["position"], scene["geoms"][0]["position"])
    assert_allclose(
        np.array(poses[1]["position"]) - scene["geoms"][1]["position"], [100, -200, 300]
    )


@pytest.mark.parametrize(
    ("kind", "size", "half_extents"),
    [
        ("box", "1 2 3", [100, 200, 300]),
        ("sphere", ".3", [30, 30, 30]),
        ("capsule", ".2 .5", [20, 20, 70]),
        ("cylinder", ".2 .5", [20, 20, 50]),
        ("plane", "2 3 .1", [200, 300, 0]),
    ],
)
def test_primitives_preserve_dimensions_normals_and_surface_orientation(kind, size, half_extents):
    model, data = _compiled(
        f'<mujoco><worldbody><geom type="{kind}" size="{size}"/></worldbody></mujoco>'
    )
    mesh = export_scene(model, data)["meshes"][0]
    vertices = np.array(mesh["vertices"])
    normals = np.array(mesh["normals"])
    triangles = np.array(mesh["triangles"]).reshape(-1, 3)
    assert_allclose(vertices.max(axis=0), half_extents, atol=1e-10)
    assert_allclose(vertices.min(axis=0), -np.array(half_extents), atol=1e-10)
    assert_allclose(np.linalg.norm(normals, axis=1), 1)
    edges = vertices[triangles[:, 1:]] - vertices[triangles[:, :1]]
    geometric_normals = np.cross(edges[:, 0], edges[:, 1])
    # Unreal's clockwise front faces have the opposite RH cross-product sign.
    assert np.all(np.sum(geometric_normals * normals[triangles].mean(axis=1), axis=1) < 0)


def test_visibility_and_material_colors_follow_authored_scene():
    model, data = _compiled("""
        <mujoco>
          <asset><material name="red" rgba="1 0 0 1" roughness=".25"/></asset>
          <worldbody>
            <geom type="sphere" size=".1" material="red"/>
            <geom type="sphere" size=".1" material="red" rgba="0 1 0 1"/>
            <geom type="sphere" size=".1" rgba="0 0 1 0"/>
            <geom type="sphere" size=".1" group="5"/>
          </worldbody>
        </mujoco>
    """)
    scene = export_scene(model, data)
    assert [geom["id"] for geom in scene["geoms"]] == [0, 1, 3]
    assert [geom["rgba"] for geom in scene["geoms"]] == [
        [1, 0, 0, 1],
        [0, 1, 0, 1],
        [0.5, 0.5, 0.5, 1],
    ]
    assert [geom["roughness"] for geom in scene["geoms"]] == [0.25, 0.25, 0.6]
    assert [geom["id"] for geom in export_scene(model, data, geom_groups=(5,))["geoms"]] == [3]


def test_partial_transparency_fails_instead_of_becoming_an_opaque_occluder():
    model, data = _compiled(
        '<mujoco><worldbody><geom type="sphere" size="1" rgba="1 0 0 .5"/></worldbody></mujoco>'
    )
    with pytest.raises(ValueError, match="partially transparent"):
        export_scene(model, data)


def test_camera_conversion_keeps_forward_depth_right_and_up():
    model, data = _compiled("""
        <mujoco><worldbody>
          <camera pos="1 -2 3" euler="20 30 40" fovy="60"/>
        </worldbody></mujoco>
    """)
    camera = camera_parameters(model, data, 0, 640, 480)
    point_mj_camera = np.array([0.3, 0.2, -2.4])
    point_world = data.cam_xmat[0].reshape(3, 3) @ point_mj_camera + data.cam_xpos[0]
    point_unreal_world = point_world * [100, -100, 100]
    point_unreal_camera = (
        Rotation.from_quat(camera["quaternion"])
        .inv()
        .apply(point_unreal_world - camera["position"])
    )
    assert_allclose(point_unreal_camera, [240, 30, 20])
    forward, right, up = point_unreal_camera
    pixel = [
        camera["fx"] * right / forward + camera["cx"],
        camera["cy"] - camera["fy"] * up / forward,
    ]
    assert_allclose(pixel, [371.461524227, 204.858983849])


def test_camera_calibration_uses_authored_focal_length_and_principal_offset():
    model, data = _compiled("""
        <mujoco><worldbody>
          <camera sensorsize=".036 .024" resolution="600 400"
            focalpixel="500 520" principalpixel="12 8"/>
        </worldbody></mujoco>
    """)
    camera = camera_parameters(model, data, 0, 1200, 800)
    assert_allclose(
        [camera[key] for key in ("fx", "fy", "cx", "cy")], [1000, 1040, 575.5, 383.5], rtol=1e-6
    )
    # Independently compare against MuJoCo's actual GL camera frustum, without a GPU.
    view = mujoco.MjvCamera()
    view.type = mujoco.mjtCamera.mjCAMERA_FIXED
    view.fixedcamid = 0
    scene = mujoco.MjvScene(model, maxgeom=1)
    mujoco.mjv_updateScene(
        model, data, mujoco.MjvOption(), None, view, mujoco.mjtCatBit.mjCAT_ALL, scene
    )
    frustum = scene.camera[0]
    left = frustum.frustum_center - frustum.frustum_width
    right = frustum.frustum_center + frustum.frustum_width
    top, bottom = frustum.frustum_top, frustum.frustum_bottom
    assert_allclose(camera["cx"], -left / (right - left) * 1200 - 0.5, rtol=1e-6)
    assert_allclose(camera["cy"], top / (top - bottom) * 800 - 0.5, rtol=1e-6)


def test_orthographic_camera_fails_explicitly():
    model, data = _compiled(
        '<mujoco><worldbody><camera projection="orthographic"/></worldbody></mujoco>'
    )
    with pytest.raises(ValueError, match="perspective"):
        camera_parameters(model, data, 0, 640, 480)


def test_unsupported_geometry_fails_instead_of_silently_disappearing():
    model, data = _compiled(
        '<mujoco><worldbody><geom type="ellipsoid" size="1 2 3"/></worldbody></mujoco>'
    )
    with pytest.raises(ValueError, match="unsupported Unreal visual type"):
        export_scene(model, data)
