# Copyright 2026 Dimensional Inc.
"""Dispatch constrained VQA intents to deterministic end-to-end families."""

from __future__ import annotations

from collections.abc import Callable

from dimos.benchmark.vqa.generation import families
from dimos.benchmark.vqa.generation.family_context import FamilyContext
from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
from dimos.benchmark.vqa.models import GroundTruthResult, QuestionIntent, QuestionKind

Family = Callable[[QuestionIntent, FamilyContext], GroundTruthResult]

FAMILIES: dict[QuestionKind, Family] = {
    "presence": families.answer_basic_object_question,
    "horizontal_direction": families.answer_basic_object_question,
    "within_distance": families.answer_basic_object_question,
    "visible_count": families.count_visible_objects,
    "camera_range": families.bucket_camera_range,
    "compare_nearest_by_side": families.compare_nearest_by_side,
    "compare_left_right": families.compare_left_right,
    "compare_height": families.compare_heights,
    "object_on_support": families.classify_object_on_support,
    "opening_width": families.measure_opening_width,
    "door_state": families.classify_door_state,
    "closest_object": families.select_closest_object,
    "forward_path": families.classify_forward_path,
}


class DeterministicQuestionAnswerer:
    """Dispatch intents while sharing one frame-scoped family context."""

    def __init__(self, primitives: FramePerceptionPrimitives) -> None:
        self.context = FamilyContext(primitives.frame, primitives)

    @property
    def primitives(self) -> FramePerceptionPrimitives:
        """Expose shared primitives to the agentic tool adapter."""
        return self.context.primitives

    def answer(self, intent: QuestionIntent) -> GroundTruthResult:
        """Run the complete deterministic family recipe for one intent."""
        return FAMILIES[intent.kind](intent, self.context)
