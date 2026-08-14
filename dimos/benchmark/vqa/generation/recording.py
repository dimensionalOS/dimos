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
"""Build self-contained VQA frames from Go2 Memory2 recordings."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, cast

import numpy as np

from dimos.benchmark.vqa.contracts import CalibratedFrame
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.tf import StreamTF
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.connection import BASE_TO_OPTICAL, GO2Connection

if TYPE_CHECKING:
    from dimos.memory2.type.observation import Observation
    from dimos.protocol.tf.tf import TFLookup


def load_go2_frame(recording: str, frame_index: int, tolerance_s: float = 0.25) -> CalibratedFrame:
    """Load one image with its nearest point cloud and captured calibration."""
    if frame_index < 0 or tolerance_s <= 0:
        raise ValueError("frame_index must be non-negative and tolerance_s must be positive")
    store = SqliteStore(path=recording, must_exist=True)
    store.start()
    try:
        streams = set(store.list_streams())
        lidar_name = "pointlio_lidar" if "pointlio_lidar" in streams else "lidar"
        images = store.stream("color_image", Image).order_by("ts")
        image_obs = images.offset(frame_index).first()
        lidar_obs = _align_one(
            images.offset(frame_index).limit(1),
            store.stream(lidar_name, PointCloud2).order_by("ts"),
            tolerance_s,
        )
        image_data = image_obs.data
        lidar_data = lidar_obs.data
        source_camera_info = (
            _align_one(
                images.offset(frame_index).limit(1),
                store.stream("camera_info", CameraInfo).order_by("ts"),
                tolerance_s,
            ).data
            if "camera_info" in streams
            else GO2Connection.camera_info_static
        )
        image, camera_info = _rectify_go2_image(image_data, source_camera_info)
        pointcloud_to_camera = _resolve_pointcloud_to_camera(
            image_obs,
            lidar_obs,
            camera_info.frame_id,
            StreamTF.from_store(store),
            tolerance_s,
        )
    finally:
        store.stop()
    return CalibratedFrame(
        id=f"go2-{frame_index}",
        image=image,
        pointcloud=lidar_data,
        camera_info=camera_info,
        pointcloud_to_camera=pointcloud_to_camera,
        image_is_rectified=True,
    )


def _align_one(primary: Any, secondary: Any, tolerance_s: float) -> Observation[Any]:
    """Return the nearest secondary observation aligned to one primary observation."""
    try:
        return cast(
            "Observation[Any]", primary.align(secondary, tolerance=tolerance_s).first().data[1]
        )
    except LookupError as exc:
        raise ValueError("recording has no synchronized observation within tolerance") from exc


def _resolve_pointcloud_to_camera(
    image_obs: Observation[Image],
    lidar_obs: Observation[PointCloud2],
    camera_frame: str,
    tf: TFLookup | None,
    tolerance_s: float,
) -> Transform:
    """Resolve camera <- point-cloud from recorded TF or captured observation poses."""
    cloud_frame = lidar_obs.data.frame_id
    if not cloud_frame:
        raise ValueError("recorded point cloud requires a frame_id")
    if tf is not None:
        transform = tf.get(
            camera_frame,
            cloud_frame,
            time_point=image_obs.ts,
            time_tolerance=tolerance_s,
        )
        if transform is not None:
            return transform

    robot_pose = image_obs.pose
    if robot_pose is None:
        raise ValueError("recording requires robot pose or a complete recorded TF chain")
    world_from_camera = Transform.from_pose("base_link", robot_pose) + BASE_TO_OPTICAL
    camera_from_world = -world_from_camera
    if cloud_frame == "world":
        return camera_from_world
    cloud_pose = lidar_obs.pose
    if cloud_pose is None:
        raise ValueError("recording requires point-cloud pose or a complete recorded TF chain")
    world_from_cloud = Transform(
        translation=cloud_pose.position,
        rotation=cloud_pose.orientation,
        frame_id="world",
        child_frame_id=cloud_frame,
        ts=lidar_obs.ts,
    )
    return camera_from_world + world_from_cloud


def _rectify_go2_image(image: Image, source: CameraInfo) -> tuple[Image, CameraInfo]:
    import cv2

    matrix = np.asarray(source.K, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(source.D, dtype=np.float64)
    size = (image.width, image.height)
    if source.distortion_model in ("equidistant", "fisheye"):
        map_x, map_y = cv2.fisheye.initUndistortRectifyMap(
            matrix, distortion, np.eye(3), matrix, size, cv2.CV_32FC1
        )
    elif source.distortion_model in ("", "plumb_bob", "rational_polynomial"):
        map_x, map_y = cv2.initUndistortRectifyMap(
            matrix, distortion, None, matrix, size, cv2.CV_32FC1
        )
    else:
        raise ValueError(f"unsupported camera distortion model: {source.distortion_model}")
    data = cv2.remap(image.data, map_x, map_y, interpolation=cv2.INTER_LINEAR)
    camera_info = CameraInfo.from_intrinsics(
        matrix[0, 0],
        matrix[1, 1],
        matrix[0, 2],
        matrix[1, 2],
        image.width,
        image.height,
        frame_id="camera_optical",
    )
    return Image(data=data, format=image.format, frame_id=image.frame_id, ts=image.ts), camera_info
