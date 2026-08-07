# Copyright 2026 Dimensional Inc.
"""Build self-contained VQA frames from Go2 Memory2 recordings."""

from __future__ import annotations

import numpy as np

from dimos.benchmark.vqa.models import CalibratedFrame
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.go2.connection import BASE_TO_OPTICAL, GO2Connection


def load_go2_frame(recording: str, frame_index: int, tolerance_s: float = 0.25) -> CalibratedFrame:
    """Load one image with its nearest LiDAR and odometry observations."""
    if frame_index < 0 or tolerance_s <= 0:
        raise ValueError("frame_index must be non-negative and tolerance_s must be positive")
    store = SqliteStore(path=recording, must_exist=True)
    store.start()
    try:
        image_obs = store.streams.color_image.offset(frame_index).first()
        lidar_obs = store.streams.lidar.at(image_obs.ts, tolerance_s).first()
        odom_obs = store.streams.odom.at(image_obs.ts, tolerance_s).first()
        image_data = image_obs.data
        lidar_data = lidar_obs.data
        odom_data = odom_obs.data
    finally:
        store.stop()
    image, camera_info = _rectify_go2_image(image_data)
    world_to_camera = -(Transform.from_pose("base_link", odom_data) + BASE_TO_OPTICAL)
    return CalibratedFrame(
        id=f"go2-{frame_index}",
        image=image,
        pointcloud=lidar_data,
        camera_info=camera_info,
        pointcloud_to_camera=world_to_camera,
        image_is_rectified=True,
        original_image=image_data,
    )


def _rectify_go2_image(image: Image) -> tuple[Image, CameraInfo]:
    import cv2

    source = GO2Connection.camera_info_static
    matrix = np.asarray(source.K, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(source.D, dtype=np.float64)
    size = (image.width, image.height)
    map_x, map_y = cv2.fisheye.initUndistortRectifyMap(
        matrix, distortion, np.eye(3), matrix, size, cv2.CV_32FC1
    )
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
