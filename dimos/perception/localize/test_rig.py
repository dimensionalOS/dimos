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

import numpy as np
import pytest

from dimos.memory.store.memory import MemoryStore
from dimos.memory.type.observation import Observation
from dimos.message_utils import camera_info, image as image_utils, pointcloud, time as message_time
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos.messages.sensor_msgs.msg import CameraInfo, Image
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.perception.detection.project import _world_to_optical, sees
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.perception.localize.association import Groups
from dimos.perception.localize.localize import index_frames
from dimos.perception.localize.module import as_detection_array
from dimos.perception.localize.rig import Rig, RigCamera, _camera_span
from dimos.perception.localize.types import Localization
from dimos.protocol.tf.tf import MultiTBuffer


def _transform(parent: str, child: str, x: float, ts: float = 2.0) -> TransformStamped:
    return TransformStamped(
        header=Header(stamp=message_time.time_from_seconds(ts), frame_id=parent),
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
    assert message_time.to_seconds(pose.header.stamp) == 2.0
    assert camera.header.frame_id == frame
    assert camera.child_frame_id == "world"
    assert camera.transform.translation.x == -expected_x
    assert rig.camera_pose(3.0, frame) is None


@pytest.fixture
def memory():
    with MemoryStore() as store:
        yield store


@pytest.mark.parametrize("depth_dtype", ["<u2", ">u2"])
def test_from_store_finds_each_cameras_color_and_aligned_depth(memory, depth_dtype) -> None:
    for frame in ("front", "rear"):
        info = camera_info.from_intrinsics(
            100,
            100,
            1,
            1,
            2,
            2,
            header=Header(frame_id=frame, stamp=message_time.time_from_seconds(2)),
        )
        memory.stream(f"{frame}_info", CameraInfo).append(info, ts=2.0)
        # Wire data is flat; classification must inspect decoded pixels.
        color = image_utils.from_array(
            np.full((2, 2, 3), 100, dtype=np.uint8),
            encoding="rgb8",
            header=Header(stamp=message_time.time_from_seconds(2.0), frame_id=frame),
        )
        depth = image_utils.from_array(
            np.full((2, 2), 1000, dtype=depth_dtype),
            encoding="16UC1",
            header=Header(stamp=message_time.time_from_seconds(2.0), frame_id=frame),
        )
        memory.stream(f"{frame}_color", Image).append(color, ts=2.0)
        memory.stream(f"{frame}_depth", Image).append(depth, ts=2.0)
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
    cloud = pointcloud.from_xyz(
        np.array([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0], [0.10, 0.0, 0.0]]),
        header=Header(stamp=message_time.time_from_seconds(2.0), frame_id="lidar"),
    )
    cloud.header.stamp = Time(sec=2, nanosec=123456789)
    rig = Rig(cameras={}, tf=tf, lidar=[Observation(ts=2.0, _data=cloud)])

    merged = rig.lidar_map()

    assert merged is not None
    assert merged.header.frame_id == "world"
    assert merged.header.stamp == message_time.time_from_seconds(2.0)
    np.testing.assert_allclose(
        pointcloud.xyz(merged), [[10.0, 0.0, 0.0], [10.1, 0.0, 0.0]], atol=1e-6
    )
    assert rig.lidar_map() is merged


def test_frame_index_accepts_generated_camera_pose_without_changing_observation_api(memory):
    tf = MultiTBuffer()
    tf.receive_transform(_transform("world", "front", 2.5))
    rig = Rig(cameras={}, tf=tf)
    image = image_utils.from_array(
        np.full((4, 4, 3), 100, dtype=np.uint8),
        encoding="rgb8",
        header=Header(frame_id="front", stamp=message_time.time_from_seconds(2)),
    )
    feed = memory.stream("color", Image)
    feed.append(image, ts=2)
    frames = index_frames(rig, feed).to_list()
    assert len(frames) == 1
    assert frames[0].data is image
    assert frames[0].tags["camera"] == "front"
    assert frames[0].pose_tuple == (2.5, 0, 0, 0, 0, 0, 1)


@pytest.mark.parametrize("depth_dtype", ["<u2", ">u2"])
def test_visibility_uses_mount_and_depth(depth_dtype):
    mount = _transform("base_link", "camera_optical", 0.5)
    observation = Observation(ts=2, _data=None, pose=(1, 2, 3))
    camera = camera_info.from_intrinsics(
        100,
        100,
        10,
        10,
        20,
        20,
        header=mount.header,
    )
    transform = _world_to_optical(observation, "world", None, mount, "camera_optical", 1)
    assert transform is not None
    assert (
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
    ) == (-1.5, -2, -3)
    assert sees((1.5, 2, 5), camera, base_to_optical=mount)(observation)
    depth = image_utils.from_array(
        np.full((20, 20), 1000, dtype=depth_dtype),
        encoding="16UC1",
        header=mount.header,
    )
    assert not sees((1.5, 2, 5), camera, base_to_optical=mount, depth=lambda _: depth)(observation)


@pytest.mark.parametrize("voxel", [0.0, 0.1])
def test_group_union_keeps_exact_newest_cloud_header(voxel):
    header = Header(frame_id="world", stamp=Time(sec=1700000000, nanosec=123456788))
    image = image_utils.from_array(
        np.zeros((2, 2, 3), dtype=np.uint8), encoding="rgb8", header=header
    )
    groups = Groups()
    for ns, points in [
        (123456789, [[0, 0, 2], [0.01, 0, 2]]),
        (123456788, [[0.02, 0, 2], [0.03, 0, 2]]),
    ]:
        cloud = pointcloud.from_xyz(
            np.array(points),
            header=Header(frame_id="world", stamp=Time(sec=1700000000, nanosec=ns)),
        )
        detection = Detection3DPC(
            image=image,
            ts=1700000000.1234567,
            bbox=(0, 0, 2, 2),
            track_id=-1,
            class_id=0,
            confidence=0.9,
            name="target",
            pointcloud=cloud,
        )
        assert groups.add(detection, radius=0.1, voxel=voxel) == 0
    merged = groups.clouds[0]
    assert merged.header.stamp == Time(sec=1700000000, nanosec=123456789)
    assert pointcloud.xyz(merged).mean(axis=0) == pytest.approx([0.015, 0, 2])
    assert len(pointcloud.xyz(merged)) == (1 if voxel else 4)


def test_localization_boxes_contain_object_bounds():
    cloud = pointcloud.from_xyz(
        np.array([[1, 2, 3], [3, 4, 5]]),
        header=Header(frame_id="world", stamp=message_time.time_from_seconds(2)),
    )
    hit = Localization(
        instance_id="object-1",
        semantic_score=0.9,
        ambiguity_margin=0.2,
        position_world_xyz=(2, 3, 4),
        frame_id="world",
        last_seen_timestamp=2,
        point_cloud=cloud,
        n_views=2,
    )
    message = as_detection_array(["cup"], [[hit]], "world")
    assert message.header.frame_id == "world"
    assert message.detections[0].id == "object-1"
    assert message.detections[0].bbox.center.position.x == 2
    assert message.detections[0].bbox.size.z == 2
    assert message.detections[0].results[0].hypothesis.class_id == "cup"


def test_camera_travel_uses_generated_tf_positions(memory):
    color = memory.stream("color", Image)
    for ts in (0.0, 11.0):
        color.append(
            image_utils.from_array(
                np.ones((2, 2, 3), dtype=np.uint8),
                encoding="rgb8",
                header=Header(stamp=message_time.time_from_seconds(ts), frame_id="camera"),
            ),
            ts=ts,
        )
    tf = MultiTBuffer(buffer_size=100)
    for ts in range(12):
        tf.receive_transform(_transform("world", "camera", ts * 0.5, ts=float(ts)))
    info = camera_info.from_intrinsics(
        1, 1, 1, 1, 2, 2, header=Header(stamp=message_time.time_from_seconds(0), frame_id="camera")
    )
    cameras = {"camera": RigCamera(info=info, color=color, depth=None)}
    assert _camera_span(cameras, tf, "world") == 5.5
