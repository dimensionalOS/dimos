"""Frame-scoped private perception primitives shared by VQA generation modes."""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from dimos.benchmark.vqa.contracts import (
    CalibratedFrame,
    GroundedObject,
    GroundPlaneEstimate,
    ObjectDetector,
    ObjectPointLocalizer,
    ObjectSegmenter,
    OracleMeasurement,
    PointObjectSegmenter,
    PrimitiveGroundingConfig,
)
from dimos.benchmark.vqa.generation.primitives.geometry import (
    ForwardCorridorMeasurement,
    PlaneFitResult,
    classify_forward_corridor,
    estimate_ground_plane,
    fit_surface_plane,
    measure_forward_corridor,
    measure_opening_width,
    measure_relative_plane_angle,
    points_around_mask,
    points_in_mask,
)
from dimos.benchmark.vqa.generation.primitives.grounding import ground_segmented_objects
from dimos.benchmark.vqa.generation.primitives.projection import project_visible_points
from dimos.benchmark.vqa.generation.primitives.results import (
    ClosestObjectResult,
    ForwardPathResult,
    HeightMeasurementResult,
    HorizontalRelationResult,
    ObjectPlaneRelationResult,
    OpeningWidthResult,
)
from dimos.perception.detection.type.detection2d.base import Detection2D
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
        config: PrimitiveGroundingConfig = PrimitiveGroundingConfig(),
    ) -> None:
        if config.min_mask_area_px < 1 or config.min_foreground_points < 1:
            raise ValueError("grounding thresholds must be positive")
        if not 0 < config.duplicate_mask_iou_threshold <= 1:
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
        self._objects: dict[str, GroundedObject] = {}
        self._object_masks: dict[str, Detection2DSeg] = {}
        self._next_object_id = 0
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
        segmented: list[Detection2D] = []
        if len(detections):
            segmented.extend(self._segmenter.segment(detections))
        elif self._localizer is not None and self._point_segmenter is not None:
            points = self._localizer.locate(self.frame.image, query)
            self._points[query] = [item for item in points if isinstance(item, Detection2DPoint)]
            segmented.extend(self._point_segmenter.segment_points(points))
        else:
            segmented.extend(detections)
        masks = [
            item
            for item in segmented
            if isinstance(item, Detection2DSeg)
            and int((item.mask > 0).sum()) >= self._config.min_mask_area_px
        ]
        self._masks[query] = masks
        self._segmented_queries.add(query)
        return masks

    def segment_detection(self, query: str, index: int) -> list[Detection2DSeg]:
        """Segment one frozen detection selected by its query-local index."""
        detections = self.detect_objects(query)
        if index < 0 or index >= len(detections):
            raise ValueError("unknown_detection_index")
        selected = ImageDetections2D(self.frame.image, [detections[index]])
        return [
            item
            for item in self._segmenter.segment(selected)
            if isinstance(item, Detection2DSeg)
            and int((item.mask > 0).sum()) >= self._config.min_mask_area_px
        ]

    def ground_mask(self, mask: Detection2DSeg) -> GroundedObject | None:
        """Ground one mask and resolve it to a frame-scoped canonical object."""
        objects = ground_segmented_objects(
            self.frame, [mask], min_foreground_points=self._config.min_foreground_points
        )
        if len(objects) != 1:
            return None
        return self._canonicalize_object(objects[0], mask)

    def ground_masks(self, query: str) -> list[GroundedObject]:
        """Project visible point-cloud support through accepted masks."""
        cached = self._groundings.get(query)
        if cached is not None:
            return cached
        masks = self.segment_detections(query)
        objects = ground_segmented_objects(
            self.frame, masks, min_foreground_points=self._config.min_foreground_points
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

    def fit_object_surface_plane(self, object: GroundedObject) -> PlaneFitResult:
        """Fit one object-supported surface plane without assigning it a semantic role."""
        points = self._object_points(object)
        if points is None:
            return PlaneFitResult(None, ("insufficient_object_support",), "insufficient_support")
        return fit_surface_plane(points)

    def fit_mask_surrounding_plane(self, mask: Detection2DSeg) -> PlaneFitResult:
        """Fit a surface plane from visible support around one selected mask."""
        return fit_surface_plane(points_around_mask(self.frame, mask.mask))

    def fit_object_surrounding_plane(self, object: GroundedObject) -> PlaneFitResult:
        """Fit a surface plane around one grounded object's selected mask."""
        mask = self._object_masks.get(object.id)
        if mask is None:
            return PlaneFitResult(None, ("unknown_grounded_object",), "unknown_object_id")
        return self.fit_mask_surrounding_plane(mask)

    def measure_object_pair_distance(
        self, first: GroundedObject, second: GroundedObject
    ) -> OracleMeasurement | None:
        """Measure support-centroid distance between two grounded objects."""
        first_points = self._object_points(first)
        second_points = self._object_points(second)
        if first_points is None or second_points is None:
            return None
        value = float(
            np.linalg.norm(np.median(first_points, axis=0) - np.median(second_points, axis=0))
        )
        return OracleMeasurement(
            value,
            "m",
            0.05,
            ("visible_support_centroids",),
            (f"grounding:v1:{first.id}", f"grounding:v1:{second.id}"),
        )

    def measure_object_plane_relation(
        self,
        object: GroundedObject,
        support: GroundedObject,
        plane: GroundPlaneEstimate,
        reference_normal: tuple[float, float, float],
    ) -> ObjectPlaneRelationResult:
        """Measure clearance and projected support proximity without assigning a relation label."""
        object_points = self._object_points(object)
        support_points = self._object_points(support)
        if object_points is None or support_points is None:
            return ObjectPlaneRelationResult(
                None, None, None, 0, None, 0, (), "insufficient_object_support"
            )
        normal = np.asarray(plane.normal)
        offset = plane.offset_m
        if float(normal @ np.asarray(reference_normal)) < 0:
            normal, offset = -normal, -offset
        elevation = object_points @ normal + offset
        projected_support = support_points - np.outer(support_points @ normal + offset, normal)
        projected_object = object_points - np.outer(elevation, normal)
        separation = np.linalg.norm(
            projected_object[:, np.newaxis, :] - projected_support[np.newaxis, :, :], axis=2
        ).min(axis=1)
        contact_points = object_points[np.abs(elevation) <= 0.08]
        overlap_count = 0
        if len(contact_points):
            projected_contact = contact_points - np.outer(contact_points @ normal + offset, normal)
            contact_separation = np.linalg.norm(
                projected_contact[:, np.newaxis, :] - projected_support[np.newaxis, :, :], axis=2
            ).min(axis=1)
            overlap_count = int((contact_separation <= 0.1).sum())
        return ObjectPlaneRelationResult(
            float(np.percentile(elevation, 15)),
            float(np.percentile(elevation, 85)),
            float((elevation >= 0.02).mean()),
            len(contact_points),
            float(separation.min()),
            overlap_count,
            ("visible_object_plane_relation",),
        )

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

    def measure_relative_plane_angle(
        self, first: GroundPlaneEstimate, second: GroundPlaneEstimate
    ) -> OracleMeasurement:
        """Measure the unsigned angle between two accepted planes."""
        return measure_relative_plane_angle(first, second)

    def select_closest_object(
        self, target: GroundedObject, candidates: list[GroundedObject]
    ) -> ClosestObjectResult:
        """Select the unambiguously closest candidate by private support-point centroids."""
        if not candidates:
            return ClosestObjectResult(None, None, (), "no_candidate_objects")
        if any(item.id == target.id for item in candidates):
            return ClosestObjectResult(None, None, (), "target_cannot_be_candidate")
        target_points = self._object_points(target)
        if target_points is None:
            return ClosestObjectResult(None, None, (), "insufficient_target_support")
        target_center = np.median(target_points, axis=0)
        distances: list[tuple[float, GroundedObject]] = []
        for candidate in candidates:
            candidate_points = self._object_points(candidate)
            if candidate_points is None:
                return ClosestObjectResult(None, None, (), "insufficient_candidate_support")
            distance = float(np.linalg.norm(np.median(candidate_points, axis=0) - target_center))
            distances.append((distance, candidate))
        distances.sort(key=lambda item: item[0])
        if len(distances) > 1 and distances[1][0] - distances[0][0] < 0.15:
            return ClosestObjectResult(None, None, (), "ambiguous_object_proximity")
        return ClosestObjectResult(distances[0][1], distances[0][0], ("object_centroid_proximity",))

    def classify_horizontal_relation(
        self, first: GroundedObject, second: GroundedObject
    ) -> HorizontalRelationResult:
        """Classify whether the first object's support centroid is left or right of the second's."""
        if first.id == second.id:
            return HorizontalRelationResult(None, (), "duplicate_object_id")
        first_points = self._object_points(first)
        second_points = self._object_points(second)
        if first_points is None or second_points is None:
            return HorizontalRelationResult(None, (), "insufficient_object_support")
        horizontal_offset_m = float(np.median(first_points[:, 0]) - np.median(second_points[:, 0]))
        if abs(horizontal_offset_m) < 0.1:
            return HorizontalRelationResult(None, (), "ambiguous_horizontal_relation")
        return HorizontalRelationResult(
            "left" if horizontal_offset_m < 0 else "right", ("camera_frame_support_centroids",)
        )

    def measure_opening_width(self, query: str) -> OpeningWidthResult:
        """Measure one segmented doorway aperture using its adjacent structural plane."""
        masks = self.segment_detections(query)
        if len(masks) != 1:
            return OpeningWidthResult(None, (), "ambiguous_opening_instances")
        ground_fit = self.fit_ground_plane()
        if ground_fit.estimate is None:
            return OpeningWidthResult(
                None,
                ground_fit.quality_flags,
                ground_fit.rejection_reason or "ground_plane_rejected",
            )
        return self.measure_opening_width_from_mask(masks[0], ground_fit.estimate)

    def measure_opening_width_from_mask(
        self, mask: Detection2DSeg, ground: GroundPlaneEstimate
    ) -> OpeningWidthResult:
        """Measure one selected aperture mask against an already accepted ground plane."""
        result = measure_opening_width(self.frame, mask.mask, ground)
        if result.width_m is None or result.tolerance_m is None:
            return OpeningWidthResult(
                None,
                result.quality_flags,
                result.rejection_reason or "opening_width_rejected",
            )
        measurement = OracleMeasurement(
            result.width_m,
            "m",
            result.tolerance_m,
            result.quality_flags,
            (f"frame:{self.frame.id}", f"opening-mask:v1:{self.frame.id}:{mask.name}"),
        )
        return OpeningWidthResult(
            measurement,
            result.quality_flags,
        )

    def classify_forward_path(self) -> ForwardPathResult:
        """Classify the observed camera-forward corridor as clear or blocked."""
        ground_fit = self.fit_ground_plane()
        if ground_fit.estimate is None:
            return ForwardPathResult(
                None,
                0,
                ("ground_plane_rejected", *ground_fit.quality_flags),
                ground_fit.rejection_reason,
            )
        projected = project_visible_points(self.frame)
        points = np.asarray(projected.camera_points, dtype=np.float64)
        state, flags, reason = classify_forward_corridor(points, ground_fit.estimate)
        return ForwardPathResult(state, len(points), flags, reason)

    def measure_forward_corridor(self, plane: GroundPlaneEstimate) -> ForwardCorridorMeasurement:
        """Measure visible forward corridor support against an accepted ground plane."""
        projected = project_visible_points(self.frame)
        points = np.asarray(projected.camera_points, dtype=np.float64)
        return measure_forward_corridor(points, plane)

    def _object_points(self, object: GroundedObject) -> np.ndarray | None:
        mask = self._object_masks.get(object.id)
        if mask is None:
            raise ValueError(f"unknown grounded object: {object.id}")
        points = points_in_mask(self.frame, mask.mask)
        return points if len(points) >= 6 else None

    def _canonicalize_object(
        self, candidate: GroundedObject, mask: Detection2DSeg
    ) -> GroundedObject:
        for object_id, existing in self._objects.items():
            existing_mask = self._object_masks[object_id]
            if self._is_duplicate_object(candidate, mask, existing, existing_mask):
                return existing

        self._next_object_id += 1
        object = replace(
            candidate,
            id=f"object:v1:{self.frame.id}:{self._next_object_id:04d}",
        )
        self._objects[object.id] = object
        self._object_masks[object.id] = mask
        return object

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
            >= self._config.duplicate_mask_iou_threshold
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
