"""Frame-scoped private perception primitives shared by VQA generation modes."""

from __future__ import annotations

import numpy as np

from dimos.benchmark.vqa.generation.grounding import ground_segmented_objects
from dimos.benchmark.vqa.generation.primitives.contracts import HeightMeasurementResult
from dimos.benchmark.vqa.generation.primitives.geometry import (
    PlaneFitResult,
    estimate_ground_plane,
    points_in_mask,
)
from dimos.benchmark.vqa.models import (
    CalibratedFrame,
    GroundedObject,
    GroundingConfig,
    GroundPlaneEstimate,
    ObjectDetector,
    ObjectPointLocalizer,
    ObjectSegmenter,
    OracleMeasurement,
    PointObjectSegmenter,
)
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.point import Detection2DPoint
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg


class FramePerceptionPrimitives:
    """Cached private perception and geometry operations over one frozen frame."""

    def __init__(
        self,
        frame: CalibratedFrame,
        detector: ObjectDetector,
        segmenter: ObjectSegmenter,
        localizer: ObjectPointLocalizer | None = None,
        point_segmenter: PointObjectSegmenter | None = None,
        config: GroundingConfig = GroundingConfig(),
    ) -> None:
        if config.min_mask_area_px < 1 or config.min_foreground_points < 1:
            raise ValueError("grounding thresholds must be positive")
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
        self._object_masks: dict[str, Detection2DSeg] = {}
        self._plane_fit: PlaneFitResult | None = None

    def detect_objects(self, query: str) -> ImageDetections2D:
        """Run private object detection once for a semantic query."""
        cached = self._detected.get(query)
        if cached is not None:
            return cached
        detections = self._detector.detect(self.frame.image, query)
        self._detected[query] = detections
        return detections

    def segment_detections(self, query: str) -> list[Detection2DSeg]:
        """Segment accepted detections, falling back to point localization when available."""
        if query in self._segmented_queries:
            return self._masks.get(query, [])
        detections = self.detect_objects(query)
        if len(detections):
            segmented = self._segmenter.segment(detections)
        elif self._localizer is not None and self._point_segmenter is not None:
            points = self._localizer.locate(self.frame.image, query)
            self._points[query] = [item for item in points if isinstance(item, Detection2DPoint)]
            segmented = self._point_segmenter.segment_points(points)
        else:
            segmented = detections
        masks = [
            item
            for item in segmented
            if isinstance(item, Detection2DSeg)
            and int((item.mask > 0).sum()) >= self._config.min_mask_area_px
        ]
        self._masks[query] = masks
        self._segmented_queries.add(query)
        return masks

    def ground_masks(self, query: str) -> list[GroundedObject]:
        """Project visible point-cloud support through accepted masks."""
        cached = self._groundings.get(query)
        if cached is not None:
            return cached
        masks = self.segment_detections(query)
        objects = ground_segmented_objects(
            self.frame, masks, min_foreground_points=self._config.min_foreground_points
        )
        for item in objects:
            index = _object_mask_index(item)
            if index < len(masks):
                self._object_masks[item.id] = masks[index]
        self._groundings[query] = objects
        return objects

    def used_point_localization(self, query: str) -> bool:
        """Return whether segmentation for a query used positive-point localization."""
        return query in self._points

    def has_grounding(self, query: str) -> bool:
        """Return whether a query has already been grounded for this frame."""
        return query in self._groundings

    @property
    def can_localize_points(self) -> bool:
        """Return whether point localization can supplement empty detections."""
        return self._localizer is not None and self._point_segmenter is not None

    def fit_ground_plane(self) -> PlaneFitResult:
        """Fit and cache one quality-gated ground plane for the frozen frame."""
        if self._plane_fit is None:
            self._plane_fit = estimate_ground_plane(self.frame)
        return self._plane_fit

    def measure_height(
        self, object: GroundedObject, plane: GroundPlaneEstimate
    ) -> HeightMeasurementResult:
        """Measure one grounded object's visible height above an accepted plane."""
        mask = self._object_masks.get(object.id)
        if mask is None:
            raise ValueError(f"unknown grounded object: {object.id}")
        selected = points_in_mask(self.frame, mask.mask)
        flags = ["visible_point_cloud_height"]
        if len(selected) < 6:
            flags.append("sparse_object_point_support")
            return HeightMeasurementResult(
                object, plane, None, tuple(flags), "insufficient_object_support"
            )
        normal = np.asarray(plane.normal)
        distances = selected @ normal + plane.offset_m
        positive = distances[distances > 0.02]
        if len(positive) < 4 or len(positive) / len(selected) < 0.6:
            flags.append("partial_or_non_elevated_object_support")
            return HeightMeasurementResult(
                object, plane, None, tuple(flags), "ambiguous_object_extent"
            )
        flags.append("conservative_upper_percentile")
        measurement = OracleMeasurement(
            float(np.percentile(positive, 85)),
            "m",
            float(max(0.05, plane.residual_m + np.std(positive) * 0.25)),
            tuple(flags),
            (
                f"frame:{self.frame.id}",
                f"ground-plane:v1:{self.frame.id}",
                f"grounding:v1:{object.id}",
            ),
        )
        return HeightMeasurementResult(object, plane, measurement, tuple(flags))


def _object_mask_index(item: GroundedObject) -> int:
    try:
        return int(item.id.rsplit("-", 1)[1])
    except (IndexError, ValueError) as exc:
        raise ValueError(f"grounded object ID lacks mask index: {item.id}") from exc
