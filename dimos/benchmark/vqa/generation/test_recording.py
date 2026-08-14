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

# Copyright 2026 Dimensional Inc.

from pathlib import Path
from typing import Any

import numpy as np

from dimos.benchmark.vqa.generation.recording import (
    _align_one,
    _rectify_go2_image,
    _resolve_pointcloud_to_camera,
)
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.type.observation import Observation
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.connection import BASE_TO_OPTICAL


def test_align_one_selects_nearest_observation(tmp_path: Path) -> None:
    store = SqliteStore(path=str(tmp_path / "recording.db"))
    store.start()
    try:
        images = store.stream("color_image", Image)
        clouds = store.stream("lidar", PointCloud2)
        images.append(Image.from_numpy(np.zeros((1, 1, 3), dtype=np.uint8)), ts=10.0)
        for ts in (9.8, 9.97, 10.08):
            cloud = PointCloud2.from_numpy(np.zeros((1, 3), dtype=np.float32))
            cloud.ts = ts
            clouds.append(cloud, ts=ts)

        aligned = _align_one(images.order_by("ts"), clouds.order_by("ts"), 0.25)

        assert aligned.ts == 9.97
    finally:
        store.stop()


def test_pointcloud_transform_prefers_recorded_tf() -> None:
    expected = Transform(
        translation=Vector3(1.0, 2.0, 3.0),
        frame_id="camera_optical",
        child_frame_id="lidar",
    )

    class _Tf:
        def get(self, *args: Any, **kwargs: Any) -> Transform:
            return expected

    image = Image.from_numpy(np.zeros((1, 1, 3), dtype=np.uint8))
    cloud = PointCloud2.from_numpy(np.zeros((1, 3), dtype=np.float32))
    cloud.frame_id = "lidar"

    resolved = _resolve_pointcloud_to_camera(
        Observation(ts=10.0, pose=Pose(), _data=image),
        Observation(ts=10.0, pose=Pose(), _data=cloud),
        "camera_optical",
        _Tf(),
        0.25,
    )

    assert resolved is expected


def test_world_pointcloud_uses_captured_robot_pose_and_static_mount_without_tf() -> None:
    image = Image.from_numpy(np.zeros((1, 1, 3), dtype=np.uint8))
    cloud = PointCloud2.from_numpy(np.zeros((1, 3), dtype=np.float32))
    cloud.frame_id = "world"
    camera_pose = Pose(Vector3(1.0, 2.0, 3.0), Quaternion())

    resolved = _resolve_pointcloud_to_camera(
        Observation(ts=10.0, pose=camera_pose, _data=image),
        Observation(ts=10.0, pose=Pose(), _data=cloud),
        "camera_optical",
        None,
        0.25,
    )

    expected = -(Transform.from_pose("base_link", camera_pose) + BASE_TO_OPTICAL)
    np.testing.assert_allclose(resolved.to_matrix(), expected.to_matrix())


def test_rectification_supports_standard_and_fisheye_calibration() -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    for model, distortion in (("plumb_bob", [0.0] * 5), ("equidistant", [0.0] * 4)):
        source = CameraInfo(
            width=4,
            height=4,
            K=[2.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 0.0, 1.0],
            D=distortion,
            distortion_model=model,
        )

        rectified, camera_info = _rectify_go2_image(image, source)

        assert rectified.shape == image.shape
        assert camera_info.width == image.width
