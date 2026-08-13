# Copyright 2026 Dimensional Inc.
"""Result construction shared by deterministic VQA families."""

from __future__ import annotations

from dimos.benchmark.vqa.generation.family_context import FamilyContext
from dimos.benchmark.vqa.models import (
    GroundedObject,
    GroundTruthResult,
    QuestionIntent,
    ToolTrace,
    VqaExample,
)


def render_question(intent: QuestionIntent) -> str:
    """Render the public question for one constrained intent."""
    if intent.kind == "presence":
        return f"Is there a {intent.object_query} in the image? Answer yes or no."
    if intent.kind == "horizontal_direction":
        return f"Where is the nearest {intent.object_query}: left, center, or right?"
    if intent.kind == "visible_count":
        return f"How many {intent.object_query}s are visible?"
    if intent.kind == "camera_range":
        return f"How far is the nearest {intent.object_query} from the camera?"
    if intent.kind == "compare_nearest_by_side":
        return f"Which {intent.object_query} is closer: the left one or the right one?"
    if intent.kind == "compare_left_right":
        return (
            f"Is the {intent.object_query} to the left or right of the {intent.comparison_query}?"
        )
    if intent.kind == "compare_height":
        return f"Which is taller: the {intent.object_query} or the {intent.comparison_query}?"
    if intent.kind == "object_on_support":
        return f"Is the {intent.object_query} on the {intent.comparison_query}? Answer yes or no."
    if intent.kind == "opening_width":
        return f"How wide is the {intent.object_query}?"
    if intent.kind == "door_state":
        return f"Is the {intent.object_query} open or closed?"
    if intent.kind == "closest_object":
        return f"Which object is closest to the {intent.object_query}: {', '.join(intent.candidate_queries)}?"
    if intent.kind == "forward_path":
        return "Is the path directly ahead clear or blocked?"
    return f"Is the nearest {intent.object_query} within {intent.threshold_m or 3:g} meters? Answer yes or no."


def rejected_result(
    context: FamilyContext,
    intent: QuestionIntent,
    objects: tuple[GroundedObject, ...] | list[GroundedObject],
    trace: tuple[ToolTrace, ...],
    reason: str,
) -> GroundTruthResult:
    """Build a rejected deterministic result with private evidence and trace."""
    rejected = VqaExample(
        f"{context.frame.id}-{intent.object_query}-{intent.kind}",
        render_question(intent),
        "",
        "",
        (),
    )
    return GroundTruthResult(intent, rejected, "rejected", None, reason, tuple(objects), trace)
