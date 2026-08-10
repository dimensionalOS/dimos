# Copyright 2026 Dimensional Inc.
"""Tool-driven private answer generation for single-frame VQA."""

from __future__ import annotations

from dimos.benchmark.vqa.generation.grounding import ground_segmented_objects
from dimos.benchmark.vqa.generation.questions import generate_questions
from dimos.benchmark.vqa.generation.selection import select_nearest_object
from dimos.benchmark.vqa.models import (
    CalibratedFrame,
    GroundedObject,
    GroundingConfig,
    GroundTruthResult,
    ObjectDetector,
    ObjectPointLocalizer,
    ObjectSegmenter,
    PointObjectSegmenter,
    QuestionIntent,
    ToolTrace,
    VqaExample,
)
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.point import Detection2DPoint
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg


class VqaGroundTruthGenerator:
    """Answer constrained questions by calling detection, segmentation, and geometry tools."""

    def __init__(
        self,
        detector: ObjectDetector,
        segmenter: ObjectSegmenter,
        localizer: ObjectPointLocalizer | None = None,
        point_segmenter: PointObjectSegmenter | None = None,
        config: GroundingConfig = GroundingConfig(),
    ) -> None:
        self._detector = detector
        self._segmenter = segmenter
        self._localizer = localizer
        self._point_segmenter = point_segmenter
        if config.min_mask_area_px < 1 or config.min_foreground_points < 1:
            raise ValueError("grounding thresholds must be positive")
        self._config = config
        self._groundings: dict[str, list[GroundedObject]] = {}
        self._detected: dict[str, ImageDetections2D] = {}
        self._segmented_queries: set[str] = set()
        self._masks: dict[str, list[Detection2DSeg]] = {}
        self._detections: dict[str, list[Detection2DBBox]] = {}
        self._points: dict[str, list[Detection2DPoint]] = {}

    def answer(self, frame: CalibratedFrame, intent: QuestionIntent) -> GroundTruthResult:
        objects, trace = self.ground(frame, intent.object_query)
        return self._answer_from_objects(frame, intent, objects, trace)

    def ground(
        self, frame: CalibratedFrame, object_query: str
    ) -> tuple[list[GroundedObject], tuple[ToolTrace, ...]]:
        """Run the fixed constrained grounding recipe over shared primitives."""
        if object_query in self._groundings:
            return self._groundings[object_query], (ToolTrace("reuse_grounding", object_query),)
        trace: list[ToolTrace] = [ToolTrace("detect_objects", object_query)]
        detections = self.detect_objects(frame, object_query)
        if len(detections):
            trace.append(ToolTrace("segment_objects", f"count={len(detections)}"))
        elif self._localizer is not None and self._point_segmenter is not None:
            trace.append(ToolTrace("locate_object_point", object_query))
        masks = self.segment_detections(frame, object_query)
        if not len(detections) and object_query in self._points:
            trace.append(
                ToolTrace("segment_object_point", f"count={len(self._points[object_query])}")
            )
        trace.append(ToolTrace("get_foreground_geometry", f"masks={len(masks)}"))
        objects = self.ground_masks(frame, object_query)
        return objects, tuple(trace)

    def detect_objects(self, frame: CalibratedFrame, object_query: str) -> ImageDetections2D:
        """Run private MoonDream detection once for an opaque object query."""
        cached = self._detected.get(object_query)
        if cached is not None:
            return cached
        detections = self._detector.detect(frame.image, object_query)
        self._detected[object_query] = detections
        self._detections[object_query] = [
            item for item in detections if isinstance(item, Detection2DBBox)
        ]
        return detections

    def segment_detections(self, frame: CalibratedFrame, object_query: str) -> list[Detection2DSeg]:
        """Run private EdgeTAM segmentation once for a detected object query."""
        if object_query in self._segmented_queries:
            return self._masks.get(object_query, [])
        detections = self.detect_objects(frame, object_query)
        if len(detections):
            segmented = self._segmenter.segment(detections)
        elif self._localizer is not None and self._point_segmenter is not None:
            points = self._localizer.locate(frame.image, object_query)
            self._points[object_query] = [
                item for item in points if isinstance(item, Detection2DPoint)
            ]
            segmented = self._point_segmenter.segment_points(points)
        else:
            segmented = detections
        masks = [
            item
            for item in segmented
            if isinstance(item, Detection2DSeg)
            and int((item.mask > 0).sum()) >= self._config.min_mask_area_px
        ]
        self._masks[object_query] = masks
        self._segmented_queries.add(object_query)
        return masks

    def ground_masks(self, frame: CalibratedFrame, object_query: str) -> list[GroundedObject]:
        """Project visible point-cloud support through an accepted mask set."""
        cached = self._groundings.get(object_query)
        if cached is not None:
            return cached
        masks = self.segment_detections(frame, object_query)
        objects = ground_segmented_objects(
            frame, masks, min_foreground_points=self._config.min_foreground_points
        )
        self._groundings[object_query] = objects
        return objects

    def masks_for_query(self, object_query: str) -> tuple[Detection2DSeg, ...]:
        """Return masks produced by the most recent local grounding for a query."""
        return tuple(self._masks.get(object_query, []))

    def _answer_from_objects(
        self,
        frame: CalibratedFrame,
        intent: QuestionIntent,
        objects: list[GroundedObject],
        trace: tuple[ToolTrace, ...],
    ) -> GroundTruthResult:
        if not objects:
            rejected = VqaExample(
                f"{frame.id}-{intent.object_query}-{intent.kind}",
                _render_question(intent),
                "",
                "",
                (),
            )
            return GroundTruthResult(
                intent, rejected, "rejected", None, "no_grounded_object", (), trace
            )
        if intent.kind == "compare_nearest_by_side":
            return _compare_nearest_by_side(frame, intent, objects, trace)
        examples = generate_questions(
            frame.id, objects, [intent.object_query], distance_m=intent.threshold_m or 3.0
        )
        suffix = {
            "presence": "presence",
            "horizontal_direction": "direction",
            "within_distance": "range",
        }[intent.kind]
        example = next((item for item in examples if item.id.endswith(f"-{suffix}")), None)
        if example is not None:
            return GroundTruthResult(
                intent,
                example,
                "answered",
                example.expected_answer,
                None,
                tuple(objects),
                trace,
            )
        rejected = VqaExample(
            f"{frame.id}-{intent.object_query}-{intent.kind}", _render_question(intent), "", "", ()
        )
        return GroundTruthResult(
            intent, rejected, "rejected", None, "no_grounded_object", tuple(objects), trace
        )

def _render_question(intent: QuestionIntent) -> str:
    if intent.kind == "presence":
        return f"Is there a {intent.object_query} in the image? Answer yes or no."
    if intent.kind == "horizontal_direction":
        return f"Where is the nearest {intent.object_query}: left, center, or right?"
    if intent.kind == "compare_nearest_by_side":
        return f"Which {intent.object_query} is closer: the left one or the right one?"
    return f"Is the nearest {intent.object_query} within {intent.threshold_m or 3:g} meters? Answer yes or no."


def _compare_nearest_by_side(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
) -> GroundTruthResult:
    left = select_nearest_object(objects, "left")
    right = select_nearest_object(objects, "right")
    if left is None or right is None:
        return _rejected_result(frame, intent, objects, trace, "missing_grounded_side")
    if left.range_m == right.range_m:
        return _rejected_result(frame, intent, objects, trace, "ambiguous_nearest_by_side")
    answer = "left" if left.range_m < right.range_m else "right"
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-nearest-by-side",
        _render_question(intent),
        answer,
        "choice",
        (left.id, right.id),
        ("left", "right"),
    )
    return GroundTruthResult(intent, example, "answered", answer, None, (left, right), trace)


def _rejected_result(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
    reason: str,
) -> GroundTruthResult:
    rejected = VqaExample(
        f"{frame.id}-{intent.object_query}-{intent.kind}", _render_question(intent), "", "", ()
    )
    return GroundTruthResult(intent, rejected, "rejected", None, reason, tuple(objects), trace)
