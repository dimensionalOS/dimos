# Copyright 2026 Dimensional Inc.
"""Dispatch constrained VQA intents to deterministic end-to-end families."""

from __future__ import annotations

from collections.abc import Callable

from dimos.benchmark.vqa.contracts import GroundTruthResult, QuestionIntent, QuestionKind
from dimos.benchmark.vqa.generation import families
from dimos.benchmark.vqa.generation.families import FamilyContext
from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives

Family = Callable[[QuestionIntent, FamilyContext], GroundTruthResult]

FAMILIES: dict[QuestionKind, Family] = {
    "presence": families.answer_basic_object_question,
    "horizontal_direction": families.answer_basic_object_question,
    "within_distance": families.answer_basic_object_question,
    "visible_count": families.count_visible_objects,
    "camera_range": families.bucket_camera_range,
    "compare_nearest_by_side": families.compare_nearest_by_side,
    "compare_left_right": families.compare_left_right,
}


class DeterministicAnswerer:
    """Dispatch intents while sharing one frame-scoped family context."""

    def __init__(self, primitives: FramePerceptionPrimitives) -> None:
        self.context = FamilyContext(primitives.frame, primitives)

    def answer(self, intent: QuestionIntent) -> GroundTruthResult:
        """Run the complete deterministic family recipe for one intent."""
        return FAMILIES[intent.kind](intent, self.context)
