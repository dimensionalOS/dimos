# Copyright 2025 Dimensional Inc.
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

import time
from typing import List, Optional, Tuple

import numpy as np
from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,
)
from dimos_lcm.sensor_msgs import CameraInfo
from reactivex import operators as ops

from dimos.core import In, Out, rpc
from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection2d.module2D import Detection2DModule

# from dimos.perception.detection2d.detic import Detic2DDetector
from dimos.perception.detection2d.type import (
    Detection2D,
    ImageDetections2D,
    ImageDetections3D,
)

# Type aliases for clarity
ImageDetections = Tuple[Image, List[Detection2D]]
ImageDetection = Tuple[Image, Detection2D]


class Detection3DModule(Detection2DModule):
    camera_info: CameraInfo

    image: In[Image] = None  # type: ignore
    pointcloud: In[PointCloud2] = None  # type: ignore
    # type: ignore
    detections: Out[Detection2DArray] = None  # type: ignore
    annotations: Out[ImageAnnotations] = None  # type: ignore

    detected_pointcloud_0: Out[PointCloud2] = None  # type: ignore
    detected_pointcloud_1: Out[PointCloud2] = None  # type: ignore
    detected_pointcloud_2: Out[PointCloud2] = None  # type: ignore
    detected_image_0: Out[Image] = None  # type: ignore
    detected_image_1: Out[Image] = None  # type: ignore
    detected_image_2: Out[Image] = None  # type: ignore

    def __init__(self, camera_info: CameraInfo, *args, **kwargs):
        self.camera_info = camera_info
        super().__init__(*args, **kwargs)

    def detect(self, image: Image) -> ImageDetections:
        detections = Detection2D.from_detector(
            self.detector.process_image(image.to_opencv()), ts=image.ts
        )
        return (image, detections)

    def project_points_to_camera(
        self,
        points_3d: np.ndarray,
        camera_matrix: np.ndarray,
        extrinsics: Transform,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Project 3D points to 2D camera coordinates."""
        # Transform points from world to camera_optical frame
        points_homogeneous = np.hstack([points_3d, np.ones((points_3d.shape[0], 1))])
        extrinsics_matrix = extrinsics.to_matrix()
        points_camera = (extrinsics_matrix @ points_homogeneous.T).T

        # Filter out points behind the camera
        valid_mask = points_camera[:, 2] > 0
        points_camera = points_camera[valid_mask]

        # Project to 2D
        points_2d_homogeneous = (camera_matrix @ points_camera[:, :3].T).T
        points_2d = points_2d_homogeneous[:, :2] / points_2d_homogeneous[:, 2:3]

        return points_2d, valid_mask

    def filter_points_in_detections(
        self,
        detections: ImageDetections2D,
        pointcloud: PointCloud2,
        world_to_camera_transform: Transform,
    ) -> List[Optional[PointCloud2]]:
        """Filter lidar points that fall within detection bounding boxes."""
        # Extract camera parameters
        camera_info = self.camera_info
        fx, fy, cx = camera_info.K[0], camera_info.K[4], camera_info.K[2]
        cy = camera_info.K[5]
        image_width = camera_info.width
        image_height = camera_info.height

        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

        # Convert pointcloud to numpy array
        lidar_points = pointcloud.as_numpy()

        # Project all points to camera frame
        points_2d_all, valid_mask = self.project_points_to_camera(
            lidar_points, camera_matrix, world_to_camera_transform
        )
        valid_3d_points = lidar_points[valid_mask]
        points_2d = points_2d_all.copy()

        # Filter points within image bounds
        in_image_mask = (
            (points_2d[:, 0] >= 0)
            & (points_2d[:, 0] < image_width)
            & (points_2d[:, 1] >= 0)
            & (points_2d[:, 1] < image_height)
        )
        points_2d = points_2d[in_image_mask]
        valid_3d_points = valid_3d_points[in_image_mask]

        filtered_pointclouds: List[Optional[PointCloud2]] = []

        for detection in detections:
            # Extract bbox from Detection2D object
            bbox = detection.bbox
            x_min, y_min, x_max, y_max = bbox

            # Find points within this detection box (with small margin)
            margin = 5  # pixels
            in_box_mask = (
                (points_2d[:, 0] >= x_min - margin)
                & (points_2d[:, 0] <= x_max + margin)
                & (points_2d[:, 1] >= y_min - margin)
                & (points_2d[:, 1] <= y_max + margin)
            )

            detection_points = valid_3d_points[in_box_mask]

            # Create PointCloud2 message for this detection
            if detection_points.shape[0] > 0:
                detection_pointcloud = PointCloud2.from_numpy(
                    detection_points,
                    frame_id=pointcloud.frame_id,
                    timestamp=pointcloud.ts,
                )
                filtered_pointclouds.append(detection_pointcloud)
            else:
                filtered_pointclouds.append(None)

        return filtered_pointclouds

    def combine_pointclouds(self, pointcloud_list: List[PointCloud2]) -> PointCloud2:
        """Combine multiple pointclouds into a single one."""
        # Filter out None values
        valid_pointclouds = [pc for pc in pointcloud_list if pc is not None]

        if not valid_pointclouds:
            # Return empty pointcloud if no valid pointclouds
            return PointCloud2.from_numpy(
                np.array([]).reshape(0, 3), frame_id="world", timestamp=time.time()
            )

        # Combine all point arrays
        all_points = np.vstack([pc.as_numpy() for pc in valid_pointclouds])

        # Use frame_id and timestamp from first pointcloud
        combined_pointcloud = PointCloud2.from_numpy(
            all_points,
            frame_id=valid_pointclouds[0].frame_id,
            timestamp=valid_pointclouds[0].ts,
        )

        return combined_pointcloud

    def hidden_point_removal(
        self, camera_transform: Transform, pc: PointCloud2, radius: float = 100.0
    ):
        camera_position = camera_transform.inverse().translation
        camera_pos_np = camera_position.to_numpy().reshape(3, 1)

        pcd = pc.pointcloud
        try:
            _, visible_indices = pcd.hidden_point_removal(camera_pos_np, radius)
            visible_pcd = pcd.select_by_index(visible_indices)

            return PointCloud2(visible_pcd, frame_id=pc.frame_id, ts=pc.ts)
        except Exception as e:
            return pc

    def cleanup_pointcloud(self, pc: PointCloud2) -> PointCloud2:
        height = pc.filter_by_height(-0.05)
        statistical, _ = height.pointcloud.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0
        )
        return PointCloud2(statistical, pc.frame_id, pc.ts)

    def process_frame(
        self,
        # these have to be timestamp aligned
        detections: ImageDetections2D,
        pointcloud: PointCloud2,
        transform: Transform,
    ) -> ImageDetections3D:
        if not transform:
            return ImageDetections3D(detections.image, [])

        pointcloud_list = self.filter_points_in_detections(detections, pointcloud, transform)

        detection3d_list = []
        for detection, pc in zip(detections, pointcloud_list):
            if pc is None:
                continue
            pc = self.hidden_point_removal(transform, self.cleanup_pointcloud(pc))
            if pc is None:
                continue

            detection3d_list.append(detection.to_3d(pointcloud=pc, transform=transform))

        return ImageDetections3D(detections.image, detection3d_list)

    @rpc
    def start(self):
        time_tolerance = 5.0  # seconds

        def detection2d_to_3d(args):
            detections, pc = args
            transform = self.tf.get("camera_optical", "world", detections.image.ts, time_tolerance)
            return self.process_frame(detections, pc, transform)

        combined_stream = self.detection_stream().pipe(
            ops.with_latest_from(self.pointcloud.observable()), ops.map(detection2d_to_3d)
        )

        self.detection_stream().subscribe(
            lambda det: self.detections.publish(det.to_ros_detection2d_array())
        )

        self.detection_stream().subscribe(
            lambda det: self.annotations.publish(det.to_image_annotations())
        )

        combined_stream.subscribe(self._handle_combined_detections)

    def _handle_combined_detections(self, detections: ImageDetections3D):
        if not detections:
            return

        print(detections)

        if len(detections) > 0:
            self.detected_pointcloud_0.publish(detections[0].pointcloud)
            self.detected_image_0.publish(detections[0].cropped_image())

        if len(detections) > 1:
            self.detected_pointcloud_1.publish(detections[1].pointcloud)
            self.detected_image_1.publish(detections[1].cropped_image())

        if len(detections) > 3:
            self.detected_pointcloud_2.publish(detections[2].pointcloud)
            self.detected_image_2.publish(detections[2].cropped_image())
