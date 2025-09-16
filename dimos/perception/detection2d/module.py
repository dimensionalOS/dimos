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
import functools
import pickle
import time
from typing import Any, Callable, List, Optional, Tuple

import numpy as np
from dimos_lcm.foxglove_msgs.Color import Color
from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,
    PointsAnnotation,
    TextAnnotation,
)
from dimos_lcm.foxglove_msgs.Point2 import Point2
from dimos_lcm.sensor_msgs import CameraInfo
from dimos_lcm.vision_msgs import (
    BoundingBox2D,
    Detection2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.std_msgs import Header
from dimos.msgs.vision_msgs import Detection2DArray

# from dimos.perception.detection2d.detic import Detic2DDetector
from dimos.perception.detection2d.type import (
    Detection2D,
    Detection3D,
    build_imageannotations,
)
from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector
from dimos.protocol.tf.tf import TF
from dimos.types.timestamped import to_ros_stamp


class Detection2DArrayFix(Detection2DArray):
    msg_name = "vision_msgs.Detection2DArray"


# Type aliases for clarity
ImageDetections = Tuple[Image, List[Detection2D]]
ImageDetection = Tuple[Image, Detection2D]


def build_detection2d_array_fix(imageDetections: ImageDetections) -> Detection2DArrayFix:
    """Build Detection2DArrayFix from image and list of Detection2D objects."""
    [image, detections] = imageDetections
    return Detection2DArray(
        detections_length=len(detections),
        header=Header(image.ts, "camera_link"),
        detections=[det.to_ros_detection2d() for det in detections],
    )


class Detection2DModule(Module):
    image: In[Image] = None  # type: ignore
    detections: Out[Detection2D] = None  # type: ignore
    annotations: Out[ImageAnnotations] = None  # type: ignore

    # _initDetector = Detic2DDetector
    _initDetector = Yolo2DDetector

    def __init__(self, *args, detector=Optional[Callable[[Any], Any]], **kwargs):
        super().__init__(*args, **kwargs)
        if detector:
            self._detectorClass = detector
        self.detector = self._initDetector()

    def process_frame(self, image: Image) -> List[Detection2D]:
        detections = Detection2D.from_detector(
            self.detector.process_image(image.to_opencv()), image=image
        )
        return detections

    @functools.cache
    def detection_stream(self):
        # Returns stream of individual Detection2D objects
        detection_stream = self.image.observable().pipe(
            ops.map(self.process_frame),
            ops.flat_map(
                lambda detections: ops.from_iterable(detections)
            ),  # Flatten list to individual items
        )

        # Publish each detection individually
        detection_stream.subscribe(self.detections.publish)

        # Convert each Detection2D to ImageAnnotations
        detection_stream.pipe(ops.map(lambda detection: detection.to_imageannotations())).subscribe(
            self.annotations.publish
        )

        return detection_stream

    @rpc
    def start(self):
        self.detection_stream()

    @rpc
    def stop(self): ...


class Detection3DModule(Detection2DModule):
    camera_info: In[CameraInfo] = None  # type: ignore
    pointcloud: In[PointCloud2] = None  # type: ignore
    filtered_pointcloud: Out[PointCloud2] = None  # type: ignore
    image: In[Image] = None  # type: ignore
    detections: Out[Detection2DArrayFix] = None  # type: ignore
    annotations: Out[ImageAnnotations] = None  # type: ignore

    def detect(self, image: Image) -> ImageDetections:
        detections = Detection2D.from_detector(
            self.detector.process_image(image.to_opencv()), image=image
        )
        return (image, detections)

    @functools.cache
    def detection_stream(self):
        detection_stream = self.image.observable().pipe(ops.map(self.detect))

        detection_stream.pipe(ops.map(build_imageannotations)).subscribe(self.annotations.publish)
        detection_stream.pipe(
            ops.filter(lambda x: len(x[1]) != 0), ops.map(build_detection2d_array_fix)
        ).subscribe(self.detections.publish)

        return detection_stream

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
        pointcloud: PointCloud2,
        image: Image,
        camera_info: CameraInfo,
        detection_list: List[Detection2D],
        world_to_camera_transform: Transform,
    ) -> List[Optional[PointCloud2]]:
        """Filter lidar points that fall within detection bounding boxes."""
        # Extract camera parameters
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

        for detection in detection_list:
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

    def process_frame(  # type: ignore[override]
        self,
        detections: List[Detection2D],
        pointcloud: PointCloud2,
        camera_info: CameraInfo,
        transform: Transform,
    ) -> List[Detection3D]:
        if not transform:
            return []

        # Get image from first detection (all should have same image)
        if not detections:
            return []

        print("DETECTIONS", detections)
        image = detections[0].image
        if image is None:
            return []

        pointcloud_list = self.filter_points_in_detections(
            pointcloud, image, camera_info, detections, transform
        )

        detection3d_list = []
        for detection, pc in zip(detections, pointcloud_list):
            if pc is None:
                continue
            pc = self.hidden_point_removal(transform, self.cleanup_pointcloud(pc))
            if pc is None:
                continue
            detection3d_list.append(detection.to_3d(pointcloud=pc, transform=transform))

        return detection3d_list

    @functools.cache
    def pointcloud_stream(self):
        # Returns stream of List[Detection3D]
        # Buffer Detection2D objects by image timestamp to process them together
        return self.detection_stream().pipe(
            ops.buffer_with_time(0.1),  # Buffer detections within 100ms window
            ops.filter(lambda detections: len(detections) > 0),
            ops.with_latest_from(self.pointcloud.observable(), self.camera_info.observable()),
            ops.map(
                lambda args: self.process_frame(
                    *args,  # [List[Detection2D], PointCloud2, CameraInfo]
                    self.tf.get("camera_optical", "world"),
                )
            ),
            ops.filter(lambda detection3d_list: len(detection3d_list) > 0),
        )

    @rpc
    def start(self):
        # Publish combined pointcloud from all Detection3D objects
        self.pointcloud_stream().pipe(
            ops.map(
                lambda detection3d_list: self.combine_pointclouds(
                    [det.pointcloud for det in detection3d_list]
                )
            )
        ).subscribe(self.filtered_pointcloud.publish)


class DetectionDBModule(Detection3DModule):
    @rpc
    def start(self):
        super().start()
        self.pointcloud_stream().subscribe(self.add_detections)

    def add_detections(self, detection3d_list: List[Detection3D]):
        for det3d in detection3d_list:
            if det3d.pointcloud is None:
                continue
            self.add_detection(det3d, det3d.pointcloud)

    # TODO collect all detections from a recording, store the stream
    # replay the stream into add_detection, validate the output
    def add_detection(self, detection: Detection3D, pc: PointCloud2):
        print("DETECTION", detection, pc)
