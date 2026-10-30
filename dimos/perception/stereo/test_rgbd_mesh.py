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

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np
import pytest

from dimos.mapping.experimental.mesh import MESH_ENTITY, MeshColours
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.shape_msgs.TriangleMesh import TriangleMesh
from dimos.perception.stereo.rgbd_mesh import RgbdMesh
from dimos.visualization.rerun.bridge import RerunEntry, is_keyed_by_seq

if TYPE_CHECKING:
    from collections.abc import Generator


@pytest.fixture
def rgbd_mesh_module() -> Generator[RgbdMesh, None, None]:
    module = RgbdMesh(
        voxel_size=0.05,
        region_m=4.0,
        emit_every=1,
        min_emit_interval_s=0.0,
        min_cell_pixels=50,
    )
    try:
        yield module
    finally:
        module.stop()


def test_triangle_mesh_lcm_roundtrip() -> None:
    vertices = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0]], dtype=np.float32)
    faces = np.array([[0, 1, 2]], dtype=np.uint32)
    colors = np.array([[255, 0, 0], [0, 255, 0], [0, 0, 255]], dtype=np.uint8)
    normals = np.array([[0.0, 0.0, -1.0], [0.0, 0.0, -1.0], [0.0, 0.0, -1.0]], dtype=np.float32)
    key = (-2, -1, 0)

    mesh = TriangleMesh(
        vertices,
        faces,
        normals,
        key=key,
        ts=1234.5,
        colors=colors,
    )
    decoded = TriangleMesh.lcm_decode(mesh.lcm_encode())

    assert decoded.ts == 1234.5
    assert decoded.key == key
    np.testing.assert_array_equal(decoded.vertices, vertices)
    np.testing.assert_array_equal(decoded.faces, faces)
    np.testing.assert_array_equal(decoded.normals, normals)
    assert decoded.colors is not None
    np.testing.assert_array_equal(decoded.colors, colors)

    uncolored = TriangleMesh(vertices, faces, normals, key=key, ts=1234.5)
    decoded_uncolored = TriangleMesh.lcm_decode(uncolored.lcm_encode())
    assert decoded_uncolored.colors is None


def test_mesh_colours_preserves_rgb_mesh_and_routes_to_chunk_entity() -> None:
    mesh = TriangleMesh(
        [[0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0]],
        [[0, 1, 2]],
        key=(-2, 3, 0),
        colors=[[10, 20, 30], [40, 50, 60], [70, 80, 90]],
    )

    colours = MeshColours(alpha=1.0)
    entries = colours(mesh)

    assert is_keyed_by_seq(colours) is True
    assert len(entries) == 1
    entry = entries[0]
    assert isinstance(entry, RerunEntry)
    assert entry.path == f"{MESH_ENTITY}/-2_3_0"
    assert entry.static is True


def test_rgbd_mesh_integrates_depth_wall_and_emits_colored_mesh(
    rgbd_mesh_module: RgbdMesh, mocker: Any
) -> None:
    published: list[TriangleMesh] = []
    mocker.patch.object(rgbd_mesh_module.mesh, "publish", side_effect=published.append)

    tf_pose = Transform(
        translation=Vector3(1.0, 1.0, 1.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
        child_frame_id="camera_optical",
        ts=100.0,
    )
    fake_tf = mocker.MagicMock()
    fake_tf.get.return_value = tf_pose
    mocker.patch.object(
        RgbdMesh, "tfbuffer", new_callable=mocker.PropertyMock, return_value=fake_tf
    )

    width, height = 64, 48
    info = CameraInfo(
        frame_id="camera_optical",
        width=width,
        height=height,
        K=[50.0, 0.0, 32.0, 0.0, 50.0, 24.0, 0.0, 0.0, 1.0],
    )
    rgbd_mesh_module._on_camera_info(info)

    rgb_data = np.full((height, width, 3), [200, 100, 50], dtype=np.uint8)
    depth_mm = np.full((height, width), 1500, dtype=np.uint16)
    color = Image(data=rgb_data, format=ImageFormat.RGB, frame_id="camera_optical", ts=100.0)
    depth = Image(data=depth_mm, format=ImageFormat.DEPTH16, frame_id="camera_optical", ts=100.0)

    rgbd_mesh_module._integrate_pair(color, depth)

    assert len(published) == 1
    out = published[0]
    assert out.key == (0, 0, 0)
    assert len(out.vertices) > 0
    assert len(out.faces) > 0
    assert out.normals.shape == out.vertices.shape
    assert out.colors is not None
    assert out.colors.shape == out.vertices.shape
