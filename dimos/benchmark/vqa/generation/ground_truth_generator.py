# Copyright 2026 Dimensional Inc.
"""Tool-driven private answer generation for single-frame VQA."""

from __future__ import annotations

from dimos.benchmark.vqa.generation import families
from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
from dimos.benchmark.vqa.generation.questions import generate_questions
from dimos.benchmark.vqa.models import (
    CalibratedFrame,
    GroundedObject,
    GroundTruthResult,
    QuestionIntent,
    ToolTrace,
)


class VqaGroundTruthGenerator:
    """Answer constrained questions by calling detection, segmentation, and geometry tools."""

    def __init__(self, primitives: FramePerceptionPrimitives) -> None:
        self.primitives = primitives

    def answer(self, frame: CalibratedFrame, intent: QuestionIntent) -> GroundTruthResult:
        if intent.kind == "forward_path":
            return families.classify_forward_path(frame, intent, self.primitives)
        if intent.kind == "opening_width":
            return families.measure_opening_width(frame, intent, self.primitives)
        objects, trace = self.ground(frame, intent.object_query)
        return self._answer_from_objects(frame, intent, objects, trace)

    def ground(
        self, frame: CalibratedFrame, object_query: str
    ) -> tuple[list[GroundedObject], tuple[ToolTrace, ...]]:
        """Run the fixed constrained grounding recipe over shared primitives."""
        if self.primitives.has_grounding(object_query):
            return self.primitives.ground_masks(object_query), (
                ToolTrace("reuse_grounding", object_query),
            )
        trace: list[ToolTrace] = [ToolTrace("detect_objects", object_query)]
        detections = self.primitives.detect_objects(object_query)
        if len(detections):
            trace.append(ToolTrace("segment_objects", f"count={len(detections)}"))
        elif self.primitives.can_localize_points:
            trace.append(ToolTrace("locate_object_point", object_query))
        masks = self.primitives.segment_detections(object_query)
        if not len(detections) and self.primitives.used_point_localization(object_query):
            trace.append(ToolTrace("segment_object_point", object_query))
        trace.append(ToolTrace("get_foreground_geometry", f"masks={len(masks)}"))
        return self.primitives.ground_masks(object_query), tuple(trace)

    def _answer_from_objects(
        self,
        frame: CalibratedFrame,
        intent: QuestionIntent,
        objects: list[GroundedObject],
        trace: tuple[ToolTrace, ...],
    ) -> GroundTruthResult:
        if not objects:
            return families.rejected_result(frame, intent, objects, trace, "no_grounded_object")
        if intent.kind == "compare_nearest_by_side":
            return families.compare_nearest_by_side(frame, intent, objects, trace)
        if intent.kind == "visible_count":
            return families.count_visible_objects(frame, intent, objects, trace)
        if intent.kind == "camera_range":
            return families.bucket_camera_range(frame, intent, objects, trace)
        if intent.kind == "compare_left_right":
            return families.compare_left_right(
                frame, intent, objects, trace, self.primitives, self.ground
            )
        if intent.kind == "compare_height":
            return families.compare_heights(
                frame, intent, objects, trace, self.primitives, self.ground
            )
        if intent.kind == "object_on_support":
            return families.classify_object_on_support(
                frame, intent, objects, trace, self.primitives, self.ground
            )
        if intent.kind == "door_state":
            return families.classify_door_state(frame, intent, objects, trace, self.primitives)
        if intent.kind == "closest_object":
            return families.select_closest_object(
                frame, intent, objects, trace, self.primitives, self.ground
            )
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
                intent, example, "answered", example.expected_answer, None, tuple(objects), trace
            )
        return families.rejected_result(frame, intent, objects, trace, "no_grounded_object")
