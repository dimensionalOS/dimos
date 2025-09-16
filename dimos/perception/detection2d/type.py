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

from dimos_lcm.foxglove_msgs.Color import Color
from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    PointsAnnotation,
    TextAnnotation,
)
from dimos_lcm.foxglove_msgs.Point2 import Point2
from dimos_lcm.vision_msgs import (
    BoundingBox2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)
from dimos_lcm.vision_msgs import (
    Detection2D as ROSDetection2D,
)

from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.std_msgs import Header
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.types.timestamped import Timestamped, to_ros_stamp, to_timestamp

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
        (bbox, track_id, class_id, confidence, name if name else "")
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

    def to_ros_bbox(self) -> BoundingBox2D:
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

    def to_text_annotation(self) -> List[TextAnnotation]:
        x1, y1, x2, y2 = self.bbox

        font_size = int(self.image.height / 35)
        return [
            TextAnnotation(
                timestamp=to_ros_stamp(self.ts),
                position=Point2(x=x1, y=y2 + font_size),
                text=f"confidence: {self.confidence:.3f}",
                font_size=font_size,
                text_color=Color(r=1.0, g=1.0, b=1.0, a=1),
                background_color=Color(r=0, g=0, b=0, a=1),
            ),
            TextAnnotation(
                timestamp=to_ros_stamp(self.ts),
                position=Point2(x=x1, y=y1),
                text=f"{self.name}_{self.class_id}_{self.track_id}",
                font_size=font_size,
                text_color=Color(r=1.0, g=1.0, b=1.0, a=1),
                background_color=Color(r=0, g=0, b=0, a=1),
            ),
        ]

    def to_points_annotation(self) -> List[PointsAnnotation]:
        x1, y1, x2, y2 = self.bbox

        thickness = self.image.height / 720

        return [
            PointsAnnotation(
                timestamp=to_ros_stamp(self.ts),
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
        ]

    def to_annotations(self) -> ImageAnnotations:
        if self.image is None:
            raise ValueError("Image is required to create ImageAnnotations")

        points = self.to_points_annotation()
        texts = self.to_text_annotation()

        return ImageAnnotations(
            texts=texts,
            texts_length=len(texts),
            points=points,
            points_length=len(points),
        )

    @classmethod
    def from_ros_detection2d(cls, ros_det: ROSDetection2D, **kwargs) -> "Detection2D":
        """Convert from ROS Detection2D message to Detection2D object."""
        # Extract bbox from ROS format
        center_x = ros_det.bbox.center.position.x
        center_y = ros_det.bbox.center.position.y
        width = ros_det.bbox.size_x
        height = ros_det.bbox.size_y

        # Convert centered bbox to corner format
        x1 = center_x - width / 2.0
        y1 = center_y - height / 2.0
        x2 = center_x + width / 2.0
        y2 = center_y + height / 2.0
        bbox = (x1, y1, x2, y2)

        # Extract hypothesis info
        class_id = 0
        confidence = 0.0
        if ros_det.results:
            hypothesis = ros_det.results[0].hypothesis
            class_id = hypothesis.class_id
            confidence = hypothesis.score

        # Extract track_id
        track_id = int(ros_det.id) if ros_det.id.isdigit() else 0

        # Extract timestamp
        ts = to_timestamp(ros_det.header.stamp)

        # Name is not stored in ROS Detection2D, so we'll use a placeholder
        # Remove 'name' from kwargs if present to avoid duplicate
        name = kwargs.pop("name", f"class_{class_id}")

        return cls(
            bbox=bbox,
            track_id=track_id,
            class_id=class_id,
            confidence=confidence,
            name=name,
            ts=ts,
            **kwargs,
        )

    def to_ros_detection2d(self) -> ROSDetection2D:
        return ROSDetection2D(
            header=Header(self.ts, "camera_link"),
            bbox=self.to_ros_bbox(),
            results=[
                ObjectHypothesisWithPose(
                    ObjectHypothesis(
                        class_id=self.class_id,
                        score=self.confidence,
                    )
                )
            ],
            id=str(self.track_id),
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


class ImageDetections2D:
    image: Image
    detections: List[Detection2D]

    @classmethod
    def from_detector(
        cls, image: Image, raw_detections: InconvinientDetectionFormat, **kwargs
    ) -> "ImageDetections2D":
        return cls(image=image, detections=Detection2D.from_detector(raw_detections, ts=image.ts))

    def to_ros_detection2d_array(self) -> Detection2DArray:
        return Detection2DArray(
            detections_length=len(self.detections),
            header=Header(self.image.ts, "camera_optical"),
            detections=[det.to_ros_detection2d() for det in self.detections],
        )

    def to_image_annotations(self) -> ImageAnnotations:
        def flatten(xss):
            return [x for xs in xss for x in xs]

        texts = flatten(det.to_text_annotation() for det in self.detections)
        points = flatten(det.to_points_annotation() for det in self.detections)

        return ImageAnnotations(
            texts=texts,
            texts_length=len(texts),
            points=points,
            points_length=len(points),
        )


@dataclass
class Detection3D(Detection2D):
    pointcloud: Optional[PointCloud2] = None
    transform: Optional[Transform] = None

    def localize(self, pointcloud: PointCloud2) -> Detection3D:
        self.pointcloud = pointcloud
        return self


def to_imageannotation_text(detection: Detection2D) -> List[TextAnnotation]:
    x1, y1, x2, y2 = detection.bbox

    font_size = int(detection.image.height / 35)
    return [
        TextAnnotation(
            timestamp=to_ros_stamp(detection.ts),
            position=Point2(x=x1, y=y2 + font_size),
            text=f"confidence: {detection.confidence:.3f}",
            font_size=font_size,
            text_color=Color(r=1.0, g=1.0, b=1.0, a=1),
            background_color=Color(r=0, g=0, b=0, a=1),
        ),
        TextAnnotation(
            timestamp=to_ros_stamp(detection.ts),
            position=Point2(x=x1, y=y1),
            text=f"{detection.name}_{detection.class_id}_{detection.track_id}",
            font_size=font_size,
            text_color=Color(r=1.0, g=1.0, b=1.0, a=1),
            background_color=Color(r=0, g=0, b=0, a=1),
        ),
    ]


def to_imageannotation_box(detection: Detection2D) -> PointsAnnotation:
    x1, y1, x2, y2 = detection.bbox

    thickness = detection.image.height / 720

    return PointsAnnotation(
        timestamp=to_ros_stamp(detection.ts),
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


def to_imageannotations(detections: List[Detection2D]) -> ImageAnnotations:
    def flatten(xss):
        return [x for xs in xss for x in xs]

    points = list(map(to_imageannotation_box, detections))
    texts = list(flatten(map(to_imageannotation_text, detections)))

    return ImageAnnotations(
        texts=texts,
        texts_length=len(texts),
        points=points,
        points_length=len(points),
    )
