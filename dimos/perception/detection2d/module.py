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
from typing import Any, Callable, List, Optional, Tuple

from dimos_lcm.foxglove_msgs.Color import Color
from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,
    PointsAnnotation,
    TextAnnotation,
)
from dimos_lcm.foxglove_msgs.Point2 import Point2
from dimos_lcm.geometry_msgs import Point
from dimos_lcm.std_msgs import Time as ROSTime
from dimos_lcm.vision_msgs import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)
from dimos_lcm.visualization_msgs import ImageMarker
from reactivex import operators as ops

from dimos.core import In, Module, Out, rpc
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.std_msgs import Header
from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector
from dimos.types.timestamped import to_ros_stamp


class Detection2DArrayFix(Detection2DArray):
    msg_name = "vision_msgs.Detection2DArray"


Bbox = Tuple[float, float, float, float]
CenteredBbox = Tuple[float, float, float, float]
# yolo and detic have bad output formats
InconvinientDetectionFormat = Tuple[List[Bbox], List[int], List[int], List[float], List[List[str]]]


Detection = Tuple[Bbox, int, int, float, List[str]]
Detections = List[Detection]


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


def build_detection2d(detection: Detection) -> Detection2D:
    [bbox, track_id, class_id, confidence, name] = detection

    return Detection2D(
        header=Header("camera_link"),
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


def build_detection2d_array(detections: Detections) -> Detection2DArrayFix:
    return Detection2DArrayFix(
        detections_length=len(detections),
        header=Header("camera_link"),
        detections=list(
            map(
                build_detection2d,
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


previous_markers = set()


def build_imageannotation_text(detection: Detection) -> ImageAnnotations:
    [bbox, track_id, class_id, confidence, name] = detection

    x, y, width, height = get_bbox_center(bbox)

    return TextAnnotation(
        text_color=Color(r=1.0, g=1.0, b=1.0),
        background_color=Color(r=0, g=0, b=0),
        text=name,
        position=Point2(x=0, y=0),
        font_size=20,
    )


def build_imageannotation(detection: Detection) -> ImageAnnotations:
    [bbox, track_id, class_id, confidence, name] = detection

    x1, y1, x2, y2 = bbox
    x, y, width, height = get_bbox_center(bbox)

    return PointsAnnotation(
        outline_color=Color(r=1.0, g=1.0, b=1.0),
        points_length=4,
        points=[
            Point2(x1, y1),
            Point2(x1, y2),
            Point2(x2, y2),
            Point2(x2, y1),
        ],
        type=PointsAnnotation.LINE_LOOP,
    )


def build_imageannotations(detections: Detections) -> List[ImageMarker]:
    points = list(map(build_imageannotation, detections))
    return ImageAnnotations(
        #        texts=texts,
        #        texts_length=len(texts),
        points=points,
        points_length=len(points),
    )


class Detect2DModule(Module):
    image: In[Image] = None
    detections: Out[Detection2DArrayFix] = None

    _initDetector = Yolo2DDetector

    def __init__(self, *args, detector=Optional[Callable[[Any], Any]], **kwargs):
        if detector:
            self._detectorClass = detector
        super().__init__(*args, **kwargs)

    def detect(self, image: Image) -> Detections:
        return better_detection_format(self.detector.process_image(image.to_opencv()))

    @rpc
    def start(self):
        self.detector = self._initDetector()
        self.image.observable().pipe(
            ops.map(self.detect),
            ops.filter(lambda x: len(x) != 0),
            #            ops.map(build_detection2d_array),
            ops.map(build_imageannotations),
        ).subscribe(self.detections.publish)

    @rpc
    def stop(self): ...
