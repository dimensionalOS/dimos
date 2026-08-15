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

"""Frame-scoped private perception primitives shared by VQA generation modes."""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from dimos.benchmark.vqa.contracts import (
    CalibratedFrame,
    GroundedObject,
    ObjectDetector,
    ObjectPointLocalizer,
    ObjectSegmenter,
    PointObjectSegmenter,
    PrimitiveGroundingConfig,
    ProjectedPoints,
    VisualObject,
)
from dimos.benchmark.vqa.generation.primitives.grounding import (
    ground_segmented_objects,
    points_in_mask,
)
from dimos.benchmark.vqa.generation.primitives.projection import project_visible_points
from dimos.perception.detection.type.detection2d.base import Detection2D
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.point import Detection2DPoint
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg


class FramePerceptionPrimitives:
    """Cached object detection, segmentation, and point-cloud grounding over one frame."""

    def __init__(
        self,
        frame: CalibratedFrame,
        detector: ObjectDetector,
        segmenter: ObjectSegmenter,
        localizer: ObjectPointLocalizer | None = None,
        point_segmenter: PointObjectSegmenter | None = None,
        config: PrimitiveGroundingConfig = PrimitiveGroundingConfig(),
    ) -> None:
        if config.min_mask_area_px < 1 or config.min_foreground_points < 1:
            raise ValueError("grounding thresholds must be positive")
        if not 0 < config.duplicate_iou_threshold <= 1:
            raise ValueError("duplicate mask IoU threshold must be in (0, 1]")
        if config.duplicate_range_tolerance_m < 0:
            raise ValueError("duplicate range tolerance must be non-negative")
        self.frame = frame
        self._detector = detector
        self._segmenter = segmenter
        self._localizer = localizer
        self._point_segmenter = point_segmenter
        self._config = config
        self._detected: dict[str, ImageDetections2D] = {}
        self._segmented_queries: set[str] = set()
        self._masks: dict[str, list[Detection2DSeg]] = {}
        self._points: dict[str, list[Detection2DPoint]] = {}
        self._groundings: dict[str, list[GroundedObject]] = {}
        self._visuals: dict[str, list[VisualObject]] = {}
        self._visual_objects: dict[str, VisualObject] = {}
        self._objects: dict[str, GroundedObject] = {}
        self._object_masks: dict[str, Detection2DSeg] = {}
        self._projected: ProjectedPoints | None = None
        self._next_object_id = 0
        self._next_visual_id = 0

    def detect_objects(self, query: str) -> ImageDetections2D:
        """Run private object detection once for a semantic query."""
        cached = self._detected.get(query)
        if cached is not None:
            return cached
        detections = self._detector.detect(self.frame.image, query)
        self._detected[query] = detections
        return detections

    def segment_objects(self, query: str) -> list[Detection2DSeg]:
        """Segment all detected objects, falling back to point localization when available."""
        if query in self._segmented_queries:
            return self._masks.get(query, [])
        detections = self.detect_objects(query)
        if len(detections):
            masks = [
                mask
                for index in range(len(detections))
                for mask in self.segment_object(query, index)
            ]
        else:
            points = self._localized_points(query)
            masks = (
                self._accepted_masks(self._point_segmenter.segment_points(points))
                if self._point_segmenter is not None
                else []
            )
        self._masks[query] = masks
        self._segmented_queries.add(query)
        return masks

    def segment_object(self, query: str, index: int) -> list[Detection2DSeg]:
        """Segment one frozen detection selected by its query-local index."""
        detections = self.detect_objects(query)
        if index < 0 or index >= len(detections):
            raise ValueError("unknown_detection_index")
        selected = ImageDetections2D(self.frame.image, [detections[index]])
        masks = self._accepted_masks(self._segmenter.segment(selected))
        if masks or self._localizer is None or self._point_segmenter is None:
            return masks
        detection = detections[index]
        if not isinstance(detection, Detection2DBBox):
            return []
        x1, y1, x2, y2 = detection.bbox
        matching = [
            point
            for point in self._localized_points(query)
            if x1 <= point.x <= x2 and y1 <= point.y <= y2
        ]
        if not matching:
            return []
        center_x, center_y = detection.center_bbox
        point = min(matching, key=lambda item: (item.x - center_x) ** 2 + (item.y - center_y) ** 2)
        return self._accepted_masks(
            self._point_segmenter.segment_points(ImageDetections2D(self.frame.image, [point]))
        )

    def visual_objects(self, query: str) -> list[VisualObject]:
        """Return cached frame-scoped object evidence directly from valid detector boxes."""
        cached = self._visuals.get(query)
        if cached is not None:
            return cached
        objects: list[VisualObject] = []
        seen_ids: set[str] = set()
        for detection in self.detect_objects(query):
            if not isinstance(detection, Detection2DBBox) or not detection.is_valid():
                continue
            x1, y1, x2, y2 = detection.bbox
            candidate = VisualObject(
                "",
                detection.name,
                float(detection.confidence),
                (float(x1), float(y1), float(x2), float(y2)),
                _horizontal_direction(detection.center_bbox[0], self.frame.image.width),
            )
            object = self._canonicalize_visual(candidate)
            if object.id not in seen_ids:
                objects.append(object)
                seen_ids.add(object.id)
        self._visuals[query] = objects
        return objects

    def ground_object(self, mask: Detection2DSeg) -> GroundedObject | None:
        """Ground one mask and resolve it to a frame-scoped canonical object."""
        objects = ground_segmented_objects(
            self.frame,
            self._projected_points(),
            [mask],
            min_foreground_points=self._config.min_foreground_points,
        )
        if len(objects) != 1:
            return None
        return self._canonicalize_object(objects[0], mask)

    def ground_objects(self, query: str) -> list[GroundedObject]:
        """Project visible point-cloud support through accepted masks."""
        cached = self._groundings.get(query)
        if cached is not None:
            return cached
        masks = self.segment_objects(query)
        objects = ground_segmented_objects(
            self.frame,
            self._projected_points(),
            masks,
            min_foreground_points=self._config.min_foreground_points,
        )
        canonical: list[GroundedObject] = []
        seen_ids: set[str] = set()
        for item in objects:
            index = _object_mask_index(item)
            if index < len(masks):
                object = self._canonicalize_object(item, masks[index])
                if object.id not in seen_ids:
                    canonical.append(object)
                    seen_ids.add(object.id)
        self._groundings[query] = canonical
        return canonical

    def used_point_localization(self, query: str) -> bool:
        return query in self._points

    def has_grounding(self, query: str) -> bool:
        return query in self._groundings

    @property
    def can_localize_points(self) -> bool:
        return self._localizer is not None and self._point_segmenter is not None

    def horizontal_offset_m(self, first: GroundedObject, second: GroundedObject) -> float | None:
        """Return the first object's camera-frame X offset from the second object."""
        first_points = self._object_points(first)
        second_points = self._object_points(second)
        if first_points is None or second_points is None:
            return None
        return float(np.median(first_points[:, 0]) - np.median(second_points[:, 0]))

    def _object_points(self, object: GroundedObject) -> np.ndarray | None:
        mask = self._object_masks.get(object.id)
        if mask is None:
            raise ValueError(f"unknown grounded object: {object.id}")
        points = points_in_mask(
            self._projected_points(),
            mask.mask,
            (self.frame.image.height, self.frame.image.width),
        )
        return points if len(points) >= 6 else None

    def _projected_points(self) -> ProjectedPoints:
        if self._projected is None:
            self._projected = project_visible_points(self.frame)
        return self._projected

    def _canonicalize_object(
        self, candidate: GroundedObject, mask: Detection2DSeg
    ) -> GroundedObject:
        for object_id, existing in self._objects.items():
            existing_mask = self._object_masks[object_id]
            if self._is_duplicate_object(candidate, mask, existing, existing_mask):
                return existing
        self._next_object_id += 1
        object = replace(candidate, id=f"object:v1:{self.frame.id}:{self._next_object_id:04d}")
        self._objects[object.id] = object
        self._object_masks[object.id] = mask
        return object

    def _canonicalize_visual(self, candidate: VisualObject) -> VisualObject:
        for existing in self._visual_objects.values():
            if (
                candidate.label.casefold() == existing.label.casefold()
                and _bbox_iou(candidate.bbox, existing.bbox) >= self._config.duplicate_iou_threshold
            ):
                return existing
        self._next_visual_id += 1
        object = replace(
            candidate, id=f"visual-object:v1:{self.frame.id}:{self._next_visual_id:04d}"
        )
        self._visual_objects[object.id] = object
        return object

    def _localized_points(self, query: str) -> ImageDetections2D[Detection2DPoint]:
        cached = self._points.get(query)
        if cached is None:
            if self._localizer is None:
                return ImageDetections2D(self.frame.image, [])
            points = self._localizer.locate(self.frame.image, query)
            cached = [item for item in points if isinstance(item, Detection2DPoint)]
            self._points[query] = cached
        return ImageDetections2D(self.frame.image, cached)

    def _accepted_masks(
        self, detections: ImageDetections2D | list[Detection2D]
    ) -> list[Detection2DSeg]:
        return [
            item
            for item in detections
            if isinstance(item, Detection2DSeg)
            and int((item.mask > 0).sum()) >= self._config.min_mask_area_px
        ]

    def _is_duplicate_object(
        self,
        candidate: GroundedObject,
        candidate_mask: Detection2DSeg,
        existing: GroundedObject,
        existing_mask: Detection2DSeg,
    ) -> bool:
        if abs(candidate.range_m - existing.range_m) > self._config.duplicate_range_tolerance_m:
            return False
        return (
            _mask_iou(candidate_mask.mask, existing_mask.mask)
            >= self._config.duplicate_iou_threshold
        )


def _object_mask_index(item: GroundedObject) -> int:
    try:
        return int(item.id.rsplit("-", 1)[1])
    except (IndexError, ValueError) as exc:
        raise ValueError(f"grounded object ID lacks mask index: {item.id}") from exc


def _mask_iou(first: np.ndarray, second: np.ndarray) -> float:
    if first.shape != second.shape:
        return 0.0
    first_foreground = first > 0
    second_foreground = second > 0
    union = int(np.logical_or(first_foreground, second_foreground).sum())
    if union == 0:
        return 0.0
    intersection = int(np.logical_and(first_foreground, second_foreground).sum())
    return intersection / union


def _bbox_iou(
    first: tuple[float, float, float, float], second: tuple[float, float, float, float]
) -> float:
    left = max(first[0], second[0])
    top = max(first[1], second[1])
    right = min(first[2], second[2])
    bottom = min(first[3], second[3])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
    union = first_area + second_area - intersection
    return intersection / union if union else 0.0


def _horizontal_direction(x: float, width: int) -> str:
    if x < width / 3:
        return "left"
    if x >= 2 * width / 3:
        return "right"
    return "center"
