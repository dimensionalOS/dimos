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

from dimos_cdrgen.registry import decode, encode
import numpy as np
import pytest

from dimos.mapping.experimental.mesh import MESH_ENTITY, MeshColours
from dimos.message_utils import camera_info, image, mesh as mesh_utils, time as message_time
from dimos.messages.dimos_msgs.msg import TriangleMesh
from dimos.messages.geometry_msgs.msg import (
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos.messages.std_msgs.msg import Header
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


def test_triangle_mesh_cdr_roundtrip() -> None:
    vertices = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0]], dtype=np.float32)
    faces = np.array([[0, 1, 2]], dtype=np.uint32)
    normals = np.array([[0.0, 0.0, -1.0], [0.0, 0.0, -1.0], [0.0, 0.0, -1.0]], dtype=np.float32)
    key = (-2, -1, 0)

    mesh = mesh_utils.from_arrays(
        vertices,
        faces,
        normals,
        key=key,
        header=Header(stamp=message_time.time_from_seconds(1234.5), frame_id=""),
    )
    decoded = decode(encode(mesh), TriangleMesh)

    assert message_time.to_seconds(decoded.header.stamp) == 1234.5
    np.testing.assert_array_equal(decoded.key, key)
    np.testing.assert_array_equal(decoded.vertices.reshape(-1, 3), vertices)
    np.testing.assert_array_equal(decoded.faces.reshape(-1, 3), faces)
    np.testing.assert_array_equal(decoded.normals.reshape(-1, 3), normals)


def test_mesh_colours_preserves_mesh_and_routes_to_chunk_entity() -> None:
    mesh = mesh_utils.from_arrays(
        [[0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0]],
        [[0, 1, 2]],
        key=(-2, 3, 0),
        header=Header(stamp=message_time.time_from_seconds(0.0), frame_id=""),
    )

    colours = MeshColours(alpha=1.0)
    entries = colours(mesh)

    assert is_keyed_by_seq(colours) is True
    assert len(entries) == 1
    entry = entries[0]
    assert isinstance(entry, RerunEntry)
    assert entry.path == f"{MESH_ENTITY}/-2_3_0"
    assert entry.static is True


def test_rgbd_mesh_integrates_depth_wall_and_emits_a_mesh(
    rgbd_mesh_module: RgbdMesh, mocker: Any
) -> None:
    published: list[TriangleMesh] = []
    mocker.patch.object(rgbd_mesh_module.mesh, "publish", side_effect=published.append)

    tf_pose = TransformStamped(
        header=Header(stamp=message_time.time_from_seconds(100.0), frame_id="world"),
        child_frame_id="camera_optical",
        transform=Transform(
            translation=Vector3(1.0, 1.0, 1.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
    )
    fake_tf = mocker.MagicMock()
    fake_tf.get.return_value = tf_pose
    mocker.patch.object(
        RgbdMesh, "tfbuffer", new_callable=mocker.PropertyMock, return_value=fake_tf
    )

    width, height = 64, 48
    header = Header(stamp=message_time.time_from_seconds(100.0), frame_id="camera_optical")
    info = camera_info.from_intrinsics(50.0, 50.0, 32.0, 24.0, width, height, header=header)
    rgbd_mesh_module._on_camera_info(info)

    rgb_data = np.full((height, width, 3), [200, 100, 50], dtype=np.uint8)
    depth_mm = np.full((height, width), 1500, dtype=np.uint16)
    color = image.from_array(rgb_data, encoding="rgb8", header=header)
    depth = image.from_array(depth_mm, encoding="16UC1", header=header)

    rgbd_mesh_module._integrate_pair(color, depth)

    assert len(published) == 1
    out = published[0]
    assert tuple(int(value) for value in out.key) == (0, 0, 0)
    assert len(out.vertices) > 0
    assert len(out.faces) > 0
    assert out.normals.shape == out.vertices.shape
