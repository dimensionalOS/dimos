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

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from dimos_lcm.foxglove_msgs.Color import Color
from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,
    PointsAnnotation,
    TextAnnotation,
)
from dimos_lcm.foxglove_msgs.Point2 import Point2
from dimos_lcm.vision_msgs import (
    BoundingBox2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)
from dimos_lcm.vision_msgs import (
    Detection2D as ROSDetection2D,
)

from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.std_msgs import Header
from dimos.types.timestamped import Timestamped, to_ros_stamp

Bbox = Tuple[float, float, float, float]
CenteredBbox = Tuple[float, float, float, float]
# yolo and detic have bad output formats
InconvinientDetectionFormat = Tuple[List[Bbox], List[int], List[int], List[float], List[List[str]]]

Detection = Tuple[Bbox, int, int, float, str]
Detections = List[Detection]


# yolo and detic have bad formats this translates into list of detections
def better_detection_format(inconvinient_detections: InconvinientDetectionFormat) -> Detections:
    bboxes, track_ids, class_ids, confidences, names = inconvinient_detections
    return [
        (bbox, track_id, class_id, confidence, name[0] if name else "")
        for bbox, track_id, class_id, confidence, name in zip(
            bboxes, track_ids, class_ids, confidences, names
        )
    ]


@dataclass
class Detection2D(Timestamped):
    bbox: Bbox
    track_id: int
    class_id: int
    confidence: float
    name: str
    ts: float = 0.0
    image: Optional[Image] = None

    @classmethod
    def from_detector(
        cls, raw_detections: InconvinientDetectionFormat, **kwargs
    ) -> List["Detection2D"]:
        return [
            cls.from_detection(raw, **kwargs) for raw in better_detection_format(raw_detections)
        ]

    @classmethod
    def from_detection(cls, raw_detection: Detection, **kwargs) -> "Detection2D":
        bbox, track_id, class_id, confidence, name = raw_detection

        image = kwargs.get("image", None)
        if image is not None:
            kwargs["ts"] = image.ts

        return cls(
            bbox=bbox,
            track_id=track_id,
            class_id=class_id,
            confidence=confidence,
            name=name,
            **kwargs,
        )

    def get_bbox_center(self) -> CenteredBbox:
        x1, y1, x2, y2 = self.bbox
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        width = float(x2 - x1)
        height = float(y2 - y1)
        return (center_x, center_y, width, height)

    def build_bbox(self) -> BoundingBox2D:
        center_x, center_y, width, height = self.get_bbox_center()
        return BoundingBox2D(
            center=Pose2D(
                position=Point2D(x=center_x, y=center_y),
                theta=0.0,
            ),
            size_x=width,
            size_y=height,
        )

    def lcm_encode(self):
        return self.to_imageannotations().lcm_encode()

    def to_imageannotations(self) -> ImageAnnotations:
        bbox = self.bbox
        points = [
            Point2(x=float(bbox[0]), y=float(bbox[1])),
            Point2(x=float(bbox[2]), y=float(bbox[1])),
            Point2(x=float(bbox[2]), y=float(bbox[3])),
            Point2(x=float(bbox[0]), y=float(bbox[3])),
            Point2(x=float(bbox[0]), y=float(bbox[1])),
        ]
        return ImageAnnotations(
            circles=[],
            points=[
                PointsAnnotation(
                    timestamp=to_ros_stamp(self.ts),
                    type=PointsAnnotation.LINE_LOOP,
                    points=points,
                    outline_color=Color(r=0.0, g=1.0, b=0.0, a=1.0),
                    outline_colors=[],
                    fill_color=Color(r=0.0, g=0.0, b=0.0, a=0.0),
                    thickness=2.0,
                )
            ],
            texts=[
                TextAnnotation(
                    timestamp=to_ros_stamp(self.ts),
                    position=Point2(x=float(bbox[0]), y=float(bbox[3]) + 10),
                    text=f"{self.name} (id:{self.track_id})",
                    font_size=16.0,
                    text_color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
                    background_color=Color(r=0.0, g=0.0, b=0.0, a=0.8),
                )
            ],
        )

    def to_detection2d(self) -> ROSDetection2D:
        return ROSDetection2D(
            header=Header(self.ts, "camera_link"),
            bbox=self.build_bbox(),
            results=[
                ObjectHypothesisWithPose(
                    ObjectHypothesis(
                        class_id=self.class_id,
                        score=self.confidence,
                    )
                )
            ],
            source_id="",
            id=self.track_id,
        )

    def to_3d(self, **kwargs) -> "Detection3D":
        return Detection3D(
            bbox=self.bbox,
            track_id=self.track_id,
            class_id=self.class_id,
            confidence=self.confidence,
            name=self.name,
            ts=self.ts,
            image=self.image,
            **kwargs,
        )


@dataclass
class Detection3D(Detection2D):
    pointcloud: Optional[PointCloud2] = None
    transform: Optional[Transform] = None

    def localize(self, pointcloud: PointCloud2) -> Detection3D:
        self.pointcloud = pointcloud
        return self


def build_imageannotations(image_detections: Tuple[Image, List[Detection2D]]) -> ImageAnnotations:
    """Build ImageAnnotations from image and list of Detection2D objects."""
    image, detections = image_detections
    if not detections:
        return ImageAnnotations(circles=[], points=[], texts=[])

    all_points = []
    all_texts = []

    for detection in detections:
        annotation = detection.to_imageannotations()
        all_points.extend(annotation.points)
        all_texts.extend(annotation.texts)

    return ImageAnnotations(circles=[], points=all_points, texts=all_texts)
