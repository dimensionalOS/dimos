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
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)
from reactivex import operators as ops

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.std_msgs import Header

# from dimos.perception.detection2d.detic import Detic2DDetector
from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector
from dimos.protocol.tf.tf import TF
from dimos.types.timestamped import to_ros_stamp


class Detection2DArrayFix(Detection2DArray):
    msg_name = "vision_msgs.Detection2DArray"


Bbox = Tuple[float, float, float, float]
CenteredBbox = Tuple[float, float, float, float]
# yolo and detic have bad output formats
InconvinientDetectionFormat = Tuple[List[Bbox], List[int], List[int], List[float], List[List[str]]]


Detection = Tuple[Bbox, int, int, float, str]
Detections = List[Detection]
ImageDetections = Tuple[Image, Detections]
ImageDetection = Tuple[Image, Detection]


def get_bbox_center(bbox: Bbox) -> CenteredBbox:
    x1, y1, x2, y2 = bbox
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    width = float(x2 - x1)
    height = float(y2 - y1)
    return [center_x, center_y, width, height]


def build_bbox(bbox: Bbox) -> BoundingBox2D:
    center_x, center_y, width, height = get_bbox_center(bbox)

    return BoundingBox2D(
        center=Pose2D(
            position=Point2D(x=center_x, y=center_y),
            theta=0.0,
        ),
        size_x=width,
        size_y=height,
    )


def build_detection2d(image, detection) -> Detection2D:
    [bbox, track_id, class_id, confidence, name] = detection

    return Detection2D(
        header=Header(image.ts, "camera_link"),
        bbox=build_bbox(bbox),
        results=[
            ObjectHypothesisWithPose(
                ObjectHypothesis(
                    class_id=class_id,
                    score=1.0,
                )
            )
        ],
    )


def build_detection2d_array(imageDetections: ImageDetections) -> Detection2DArrayFix:
    [image, detections] = imageDetections
    return Detection2DArrayFix(
        detections_length=len(detections),
        header=Header(image.ts, "camera_link"),
        detections=list(
            map(
                functools.partial(build_detection2d, image),
                detections,
            )
        ),
    )


# yolo and detic have bad formats this translates into list of detections
def better_detection_format(inconvinient_detections: InconvinientDetectionFormat) -> Detections:
    bboxes, track_ids, class_ids, confidences, names = inconvinient_detections
    return [
        [bbox, track_id, class_id, confidence, name]
        for bbox, track_id, class_id, confidence, name in zip(
            bboxes, track_ids, class_ids, confidences, names
        )
    ]


def build_imageannotation_text(image: Image, detection: Detection) -> ImageAnnotations:
    [bbox, track_id, class_id, confidence, name] = detection

    x1, y1, x2, y2 = bbox

    font_size = int(image.height / 35)
    return [
        TextAnnotation(
            timestamp=to_ros_stamp(image.ts),
            position=Point2(x=x1, y=y2 + font_size),
            text=f"confidence: {confidence:.3f}",
            font_size=font_size,
            text_color=Color(r=1.0, g=1.0, b=1.0, a=1),
            background_color=Color(r=0, g=0, b=0, a=1),
        ),
        TextAnnotation(
            timestamp=to_ros_stamp(image.ts),
            position=Point2(x=x1, y=y1),
            text=f"{name}_{class_id}_{track_id}",
            font_size=font_size,
            text_color=Color(r=1.0, g=1.0, b=1.0, a=1),
            background_color=Color(r=0, g=0, b=0, a=1),
        ),
    ]


def build_imageannotation_box(image: Image, detection: Detection) -> ImageAnnotations:
    [bbox, track_id, class_id, confidence, name] = detection

    x1, y1, x2, y2 = bbox

    thickness = image.height / 720

    return PointsAnnotation(
        timestamp=to_ros_stamp(image.ts),
        outline_color=Color(r=0.0, g=0.0, b=0.0, a=1.0),
        fill_color=Color(r=1.0, g=0.0, b=0.0, a=0.15),
        thickness=thickness,
        points_length=4,
        points=[
            Point2(x1, y1),
            Point2(x1, y2),
            Point2(x2, y2),
            Point2(x2, y1),
        ],
        type=PointsAnnotation.LINE_LOOP,
    )


def build_imageannotations(image_detections: [Image, Detections]) -> ImageAnnotations:
    [image, detections] = image_detections

    def flatten(xss):
        return [x for xs in xss for x in xs]

    points = list(map(functools.partial(build_imageannotation_box, image), detections))
    texts = list(flatten(map(functools.partial(build_imageannotation_text, image), detections)))

    return ImageAnnotations(
        texts=texts,
        texts_length=len(texts),
        points=points,
        points_length=len(points),
    )


# needs to emit a detection class not this stupid array
# detection class can render itself into image annotations, or 2ddetections etc
# combining detection class with pointcloud gives you a detection with pointcloud
class Detection2DModule(Module):
    image: In[Image] = None
    detections: Out[Detection2DArrayFix] = None
    annotations: Out[ImageAnnotations] = None

    # _initDetector = Detic2DDetector
    _initDetector = Yolo2DDetector

    def __init__(self, *args, detector=Optional[Callable[[Any], Any]], **kwargs):
        super().__init__(*args, **kwargs)
        if detector:
            self._detectorClass = detector
        self.detector = self._initDetector()

    def process_frame(self, image: Image) -> Detections:
        return [image, better_detection_format(self.detector.process_image(image.to_opencv()))]

    @functools.cache
    def detection_stream(self):
        # from dimos.activate_cuda import _init_cuda
        detection_stream = self.image.observable().pipe(ops.map(self.process_frame))

        detection_stream.pipe(ops.map(build_imageannotations)).subscribe(self.annotations.publish)
        detection_stream.pipe(
            ops.filter(lambda x: len(x) != 0), ops.map(build_detection2d_array)
        ).subscribe(self.detections.publish)

        return detection_stream

    @rpc
    def start(self):
        self.detection_stream()

    @rpc
    def stop(self): ...


class DetectionPointcloud(Detection2DModule):
    camera_info: In[CameraInfo] = None
    pointcloud: In[PointCloud2] = None
    filtered_pointcloud: Out[PointCloud2] = None
    image: In[Image] = None
    detections: Out[Detection2DArrayFix] = None
    annotations: Out[ImageAnnotations] = None

    def detect(self, image: Image) -> Detections:
        return [image, better_detection_format(self.detector.process_image(image.to_opencv()))]

    @functools.cache
    def detection_stream(self):
        # from dimos.activate_cuda import _init_cuda
        detection_stream = self.image.observable().pipe(ops.map(self.detect))

        detection_stream.pipe(ops.map(build_imageannotations)).subscribe(self.annotations.publish)
        detection_stream.pipe(
            ops.filter(lambda x: len(x) != 0), ops.map(build_detection2d_array)
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
        detection_list: List[Detection],
        world_to_camera_transform: Transform,
    ) -> List[PointCloud2]:
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

        filtered_pointclouds = []

        for detection in detection_list:
            # Detection format: [bbox, track_id, class_id, confidence, names]
            bbox, track_id, class_id, confidence, names = detection
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
        imagedetections: ImageDetections,
        pointcloud: PointCloud2,
        camera_info: CameraInfo,
        transform: Transform,
    ):
        if not transform:
            return None
        image, detection_list = imagedetections
        # print("Processing frame for detection with pointcloud", image)
        # Filter pointcloud based on detections

        separate_pcs = []
        for pc in self.filter_points_in_detections(
            pointcloud, image, camera_info, detection_list, transform
        ):
            if pc is None:
                continue
            pc = self.hidden_point_removal(transform, self.cleanup_pointcloud(pc))
            if pc is None:
                continue
            separate_pcs.append(pc)

        # Combine all filtered pointclouds into one
        combined_pc = self.combine_pointclouds(separate_pcs)

        return [image, detection_list, separate_pcs, combined_pc]

    @functools.cache
    def pointcloud_stream(self):
        return self.detection_stream().pipe(
            ops.with_latest_from(self.pointcloud.observable(), self.camera_info.observable()),
            ops.map(lambda data: self.process_frame(*data, self.tf.get("camera_optical", "world"))),
            ops.filter(lambda x: x is not None),
        )

    @rpc
    def start(self):
        self.pointcloud_stream().pipe(ops.map(lambda x: x[3])).subscribe(
            self.filtered_pointcloud.publish
        )


class DetectionDB(DetectionPointcloud):
    @rpc
    def start(self):
        super().start()
        self.pointcloud_stream().pipe(
            ops.map(lambda x: [x[1], x[2]]),
        ).subscribe(self.add_detections)

    def add_detections(self, data: List[[Detections, PointCloud2]]):
        for detection, pc in zip(*data):
            if pc is None:
                continue
            self.add_detection(detection, pc)

    # TODO collect all detections from a recording, store the stream
    # replay the stream into add_detection, validate the output
    def add_detection(self, detection: Detection, pc: PointCloud2):
        print("DETECTION", detection, pc)
