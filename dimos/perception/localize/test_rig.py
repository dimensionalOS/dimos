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

"""Camera frames, depth joins, and lidar registration used by localization."""

from dimos_cdrgen.registry import decode, encode
from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.sensor_msgs.msg import CameraInfo, Image, PointCloud2
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
import numpy as np
import pytest

from dimos.memory.store.memory import MemoryStore
from dimos.memory.type.observation import Observation
from dimos.msgs.image import image_from_array
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.msgs.time import time_from_seconds, to_seconds
from dimos.perception.localize.rig import Rig
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.robot.unitree.go2.camera_calibration import front_camera_calibration


def _transform(parent: str, child: str, x: float, ts: float = 2.0) -> TransformStamped:
    return TransformStamped(
        header=Header(stamp=time_from_seconds(ts), frame_id=parent),
        child_frame_id=child,
        transform=Transform(
            translation=Vector3(x=x, y=0.0, z=0.0),
            rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


@pytest.mark.parametrize("frame, expected_x", [("front", 2.5), ("rear", 1.5)])
def test_camera_pose_composes_each_cameras_mount(frame: str, expected_x: float) -> None:
    tf = MultiTBuffer()
    tf.receive_transform(
        _transform("world", "base_link", 2.0),
        _transform("base_link", "front", 0.5),
        _transform("base_link", "rear", -0.5),
    )
    rig = Rig(cameras={}, tf=tf)

    pose = rig.camera_pose(2.0, frame)
    camera = rig.world_to_optical(2.0, frame)

    assert pose is not None and camera is not None
    assert pose.header.frame_id == "world"
    assert pose.pose.position.x == expected_x
    assert to_seconds(pose.header.stamp) == 2.0
    assert camera.header.frame_id == frame
    assert camera.child_frame_id == "world"
    assert camera.transform.translation.x == -expected_x
    assert rig.camera_pose(3.0, frame) is None


@pytest.fixture
def memory():
    with MemoryStore() as store:
        yield store


def test_from_store_finds_each_cameras_color_and_aligned_depth(memory) -> None:
    info = front_camera_calibration()
    for frame in ("front", "rear"):
        info = decode(encode(info), CameraInfo)
        info.header.frame_id = frame
        memory.stream(f"{frame}_info", CameraInfo).append(info, ts=2.0)
        # Wire data is flat; classification must inspect decoded pixels.
        color = image_from_array(
            np.full((2, 2, 3), 100, dtype=np.uint8),
            encoding="rgb8",
            header=Header(stamp=time_from_seconds(2.0), frame_id=frame),
        )
        depth = image_from_array(
            np.full((2, 2), 1000, dtype=np.uint16),
            encoding="16UC1",
            header=Header(stamp=time_from_seconds(2.0), frame_id=frame),
        )
        memory.stream(f"{frame}_color", Image).append(decode(encode(color), Image), ts=2.0)
        memory.stream(f"{frame}_depth", Image).append(decode(encode(depth), Image), ts=2.0)
    memory.stream("tf", TFMessage).append(
        TFMessage(transforms=[_transform("world", "front", 2.5), _transform("world", "rear", 1.5)]),
        ts=2.0,
    )

    rig = Rig.from_store(memory)

    assert set(rig.cameras) == {"front", "rear"}
    assert rig.world_frame == "world"
    assert rig.time_range() == (2.0, 2.0)
    assert rig.depth_at(2.0, "rear").encoding == "16UC1"
    assert rig.frame_at(Observation(ts=2.0, _data=color)).header.frame_id == "rear"
    assert rig.camera_pose(2.0, "front").pose.position.x == 2.5


def test_lidar_map_registers_and_deduplicates_scans_in_world_frame() -> None:
    tf = MultiTBuffer()
    tf.receive_transform(_transform("world", "lidar", 10.0))
    cloud = pointcloud_from_xyz(
        np.array([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0], [0.10, 0.0, 0.0]]),
        header=Header(stamp=time_from_seconds(2.0), frame_id="lidar"),
    )
    rig = Rig(
        cameras={}, tf=tf, lidar=[Observation(ts=2.0, _data=decode(encode(cloud), PointCloud2))]
    )

    merged = rig.lidar_map()

    assert merged is not None
    assert merged.header.frame_id == "world"
    assert to_seconds(merged.header.stamp) == 2.0
    np.testing.assert_allclose(
        pointcloud_xyz(merged), [[10.0, 0.0, 0.0], [10.1, 0.0, 0.0]], atol=1e-6
    )
    assert rig.lidar_map() is merged
