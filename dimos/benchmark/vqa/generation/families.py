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

# Copyright 2026 Dimensional Inc.
"""Recipes and result construction for constrained single-frame VQA families."""

from __future__ import annotations

from dataclasses import dataclass

from dimos.benchmark.vqa.contracts import (
    CalibratedFrame,
    GroundedObject,
    GroundTruthResult,
    QuestionIntent,
    ToolTrace,
    VisualObject,
    VqaExample,
)
from dimos.benchmark.vqa.generation.answer_choices import (
    CAMERA_RANGE_CHOICES,
    COUNT_CHOICES,
    camera_range_choice,
    count_choice,
)
from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
from dimos.benchmark.vqa.generation.primitives.selection import select_nearest_object


@dataclass(frozen=True)
class GroundingResult:
    """Grounded objects and the operations used to establish them."""

    objects: tuple[GroundedObject, ...]
    trace: tuple[ToolTrace, ...]


@dataclass(frozen=True)
class VisualResult:
    """Visible objects and the detector operation used to establish them."""

    objects: tuple[VisualObject, ...]
    trace: tuple[ToolTrace, ...]


@dataclass
class FamilyContext:
    """Shared calibrated frame and cached primitives for deterministic families."""

    frame: CalibratedFrame
    primitives: FramePerceptionPrimitives

    def ground(self, object_query: str) -> GroundingResult:
        """Detect, segment, and ground one semantic query with traceable cache reuse."""
        if self.primitives.has_grounding(object_query):
            return GroundingResult(
                tuple(self.primitives.ground_masks(object_query)),
                (ToolTrace("reuse_grounding", object_query),),
            )
        trace: list[ToolTrace] = [ToolTrace("detect_objects", object_query)]
        detections = self.primitives.detect_objects(object_query)
        if len(detections):
            trace.append(ToolTrace("segment_objects", f"count={len(detections)}"))
        elif self.primitives.can_localize_points:
            trace.append(ToolTrace("locate_object_point", object_query))
        masks = self.primitives.segment_detections(object_query)
        if len(detections) and self.primitives.used_point_localization(object_query):
            trace.append(ToolTrace("fallback_locate_object_point", object_query))
            trace.append(ToolTrace("fallback_segment_object_point", object_query))
        elif self.primitives.used_point_localization(object_query):
            trace.append(ToolTrace("segment_object_point", object_query))
        trace.append(ToolTrace("get_foreground_geometry", f"masks={len(masks)}"))
        return GroundingResult(tuple(self.primitives.ground_masks(object_query)), tuple(trace))

    def observe(self, object_query: str) -> VisualResult:
        """Detect visible object instances without requiring point-cloud support."""
        return VisualResult(
            tuple(self.primitives.visual_objects(object_query)),
            (ToolTrace("detect_objects", object_query),),
        )


def render_question(intent: QuestionIntent) -> str:
    """Render the public question for one constrained intent."""
    if intent.kind == "presence":
        return f"Is {intent.object_query} visible in the image? Answer yes or no."
    if intent.kind == "horizontal_direction":
        return (
            f"Where is the detected instance of {intent.object_query} in the image: "
            "left, center, or right?"
        )
    if intent.kind == "visible_count":
        return f"How many visible instances of {intent.object_query} are there?"
    if intent.kind == "camera_range":
        return f"How far is the nearest detected instance of {intent.object_query} from the camera?"
    if intent.kind == "compare_nearest_by_side":
        return (
            f"Which detected instance of {intent.object_query} is closer: "
            "the left one or the right one?"
        )
    if intent.kind == "compare_left_right":
        return (
            f"Is the detected instance of {intent.object_query} to the left or right of the "
            f"detected instance of {intent.comparison_query}?"
        )
    if intent.kind == "within_distance":
        threshold = intent.threshold_m or 3
        unit = "meter" if threshold == 1 else "meters"
        return (
            f"Is any detected instance of {intent.object_query} within {threshold:g} {unit}? "
            "Answer yes or no."
        )
    raise ValueError(f"unsupported deterministic question kind: {intent.kind}")


def rejected_result(
    context: FamilyContext,
    intent: QuestionIntent,
    objects: tuple[GroundedObject | VisualObject, ...] | list[GroundedObject | VisualObject],
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


def answer_basic_object_question(
    intent: QuestionIntent, context: FamilyContext
) -> GroundTruthResult:
    """Ground one query and answer presence, direction, or distance deterministically."""
    if intent.kind in ("presence", "horizontal_direction"):
        observed = context.observe(intent.object_query)
        if not observed.objects:
            return rejected_result(
                context, intent, observed.objects, observed.trace, "no_visual_detection"
            )
        if intent.kind == "horizontal_direction" and len(observed.objects) != 1:
            return rejected_result(
                context, intent, observed.objects, observed.trace, "ambiguous_visual_object"
            )
        selected = observed.objects[0]
        choices: tuple[str, ...]
        if intent.kind == "presence":
            suffix, answer, answer_type, choices = "presence", "yes", "boolean", ("yes", "no")
            evidence = observed.objects
        else:
            suffix, answer = "direction", selected.horizontal_direction
            answer_type, choices = "choice", ("left", "center", "right")
            evidence = (selected,)
        example = VqaExample(
            f"{context.frame.id}-{intent.object_query}-{suffix}",
            render_question(intent),
            answer,
            answer_type,
            tuple(item.id for item in evidence),
            choices,
        )
        return GroundTruthResult(
            intent, example, "answered", answer, None, tuple(evidence), observed.trace
        )

    grounded = context.ground(intent.object_query)
    if not grounded.objects:
        return rejected_result(
            context, intent, grounded.objects, grounded.trace, "no_grounded_object"
        )
    nearest = select_nearest_object(list(grounded.objects))
    if nearest is None:
        return rejected_result(
            context, intent, grounded.objects, grounded.trace, "no_grounded_object"
        )
    threshold_m = intent.threshold_m or 3.0
    if threshold_m <= 0:
        raise ValueError("distance threshold must be positive")
    suffix = f"within-{threshold_m:g}m"
    answer = "yes" if nearest.range_m <= threshold_m else "no"
    answer_type, choices = "boolean", ("yes", "no")
    example = VqaExample(
        f"{context.frame.id}-{intent.object_query}-{suffix}",
        render_question(intent),
        answer,
        answer_type,
        (nearest.id,),
        choices,
    )
    return GroundTruthResult(
        intent, example, "answered", answer, None, grounded.objects, grounded.trace
    )


def count_visible_objects(intent: QuestionIntent, context: FamilyContext) -> GroundTruthResult:
    observed = context.observe(intent.object_query)
    objects, trace = observed.objects, observed.trace
    if not objects:
        return rejected_result(context, intent, objects, trace, "no_visual_detection")
    answer = count_choice(len(objects))
    example = VqaExample(
        f"{context.frame.id}-{intent.object_query}-visible-count",
        render_question(intent),
        answer,
        "choice",
        tuple(item.id for item in objects),
        COUNT_CHOICES,
    )
    return GroundTruthResult(intent, example, "answered", answer, None, objects, trace)


def bucket_camera_range(intent: QuestionIntent, context: FamilyContext) -> GroundTruthResult:
    grounded = context.ground(intent.object_query)
    objects, trace = grounded.objects, grounded.trace
    selected = select_nearest_object(list(objects))
    if selected is None:
        return rejected_result(context, intent, objects, trace, "no_grounded_object")
    answer = camera_range_choice(selected.range_m)
    example = VqaExample(
        f"{context.frame.id}-{intent.object_query}-camera-range",
        render_question(intent),
        answer,
        "choice",
        (selected.id,),
        CAMERA_RANGE_CHOICES,
    )
    return GroundTruthResult(intent, example, "answered", answer, None, (selected,), trace)


def compare_left_right(intent: QuestionIntent, context: FamilyContext) -> GroundTruthResult:
    grounded = context.ground(intent.object_query)
    objects, trace = grounded.objects, grounded.trace
    if len(objects) != 1 or intent.comparison_query is None:
        return rejected_result(context, intent, objects, trace, "ambiguous_first_relation_object")
    other = context.ground(intent.comparison_query)
    other_objects = other.objects
    trace = (*trace, *other.trace)
    if len(other_objects) != 1:
        return rejected_result(
            context, intent, [*objects, *other_objects], trace, "ambiguous_second_relation_object"
        )
    relation = context.primitives.classify_horizontal_relation(objects[0], other_objects[0])
    trace = (
        *trace,
        ToolTrace(
            "classify_horizontal_relation",
            relation.relation or relation.rejection_reason or "rejected",
        ),
    )
    if relation.relation is None:
        return rejected_result(
            context,
            intent,
            [*objects, *other_objects],
            trace,
            relation.rejection_reason or "horizontal_relation_rejected",
        )
    example = VqaExample(
        f"{context.frame.id}-{intent.object_query}-{intent.comparison_query}-left-right",
        render_question(intent),
        relation.relation,
        "choice",
        (objects[0].id, other_objects[0].id),
        ("left", "right"),
    )
    return GroundTruthResult(
        intent, example, "answered", relation.relation, None, (objects[0], other_objects[0]), trace
    )


def compare_nearest_by_side(intent: QuestionIntent, context: FamilyContext) -> GroundTruthResult:
    grounded = context.ground(intent.object_query)
    objects, trace = grounded.objects, grounded.trace
    if not objects:
        return rejected_result(context, intent, objects, trace, "no_grounded_object")
    left = select_nearest_object(list(objects), "left")
    right = select_nearest_object(list(objects), "right")
    if left is None or right is None:
        return rejected_result(context, intent, objects, trace, "missing_grounded_side")
    if abs(left.range_m - right.range_m) <= 0.1:
        return rejected_result(context, intent, objects, trace, "ambiguous_nearest_by_side")
    answer = "left" if left.range_m < right.range_m else "right"
    example = VqaExample(
        f"{context.frame.id}-{intent.object_query}-nearest-by-side",
        render_question(intent),
        answer,
        "choice",
        (left.id, right.id),
        ("left", "right"),
    )
    return GroundTruthResult(intent, example, "answered", answer, None, (left, right), trace)
