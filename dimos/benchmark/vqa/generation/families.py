# Copyright 2026 Dimensional Inc.
"""Recipes and result construction for constrained single-frame VQA families."""

from __future__ import annotations

import numpy as np

from dimos.benchmark.vqa.generation.family_common import rejected_result, render_question
from dimos.benchmark.vqa.generation.family_context import FamilyContext
from dimos.benchmark.vqa.generation.primitives.choices import (
    CAMERA_RANGE_CHOICES,
    COUNT_CHOICES,
    OPENING_WIDTH_CHOICES,
    camera_range_choice,
    count_choice,
    opening_width_choice,
)
from dimos.benchmark.vqa.generation.primitives.selection import select_nearest_object
from dimos.benchmark.vqa.models import (
    GroundedObject,
    GroundTruthResult,
    QuestionIntent,
    ToolTrace,
    VqaExample,
)


def answer_basic_object_question(
    intent: QuestionIntent, context: FamilyContext
) -> GroundTruthResult:
    """Ground one query and answer presence, direction, or distance deterministically."""
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
    choices: tuple[str, ...]
    if intent.kind == "presence":
        suffix, answer, answer_type, choices = "presence", "yes", "boolean", ("yes", "no")
    elif intent.kind == "horizontal_direction":
        suffix = "direction"
        answer = nearest.horizontal_direction
        answer_type, choices = "choice", ("left", "center", "right")
    else:
        threshold_m = intent.threshold_m or 3.0
        if threshold_m <= 0:
            raise ValueError("distance threshold must be positive")
        suffix = "range"
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
        intent,
        example,
        "answered",
        answer,
        None,
        grounded.objects,
        grounded.trace,
    )


def count_visible_objects(intent: QuestionIntent, context: FamilyContext) -> GroundTruthResult:
    grounded = context.ground(intent.object_query)
    objects, trace = grounded.objects, grounded.trace
    if not objects:
        return rejected_result(context, intent, objects, trace, "no_grounded_object")
    answer = count_choice(len(objects))
    example = VqaExample(
        f"{context.frame.id}-{intent.object_query}-visible-count",
        render_question(intent),
        answer,
        "choice",
        tuple(item.id for item in objects),
        COUNT_CHOICES,
    )
    return GroundTruthResult(intent, example, "answered", answer, None, tuple(objects), trace)


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


def compare_heights(intent: QuestionIntent, context: FamilyContext) -> GroundTruthResult:
    grounded = context.ground(intent.object_query)
    objects, trace = grounded.objects, grounded.trace
    if len(objects) != 1 or intent.comparison_query is None:
        return rejected_result(context, intent, objects, trace, "ambiguous_first_height_object")
    other = context.ground(intent.comparison_query)
    other_objects = other.objects
    trace = (*trace, *other.trace)
    if len(other_objects) != 1:
        return rejected_result(
            context, intent, [*objects, *other_objects], trace, "ambiguous_second_height_object"
        )
    plane_fit = context.primitives.fit_ground_plane()
    trace = (*trace, ToolTrace("fit_ground_plane", plane_fit.rejection_reason or "accepted"))
    if plane_fit.estimate is None:
        return rejected_result(
            context,
            intent,
            [*objects, *other_objects],
            trace,
            plane_fit.rejection_reason or "ground_plane_rejected",
        )
    first = context.primitives.measure_height(objects[0], plane_fit.estimate)
    second = context.primitives.measure_height(other_objects[0], plane_fit.estimate)
    trace = (
        *trace,
        ToolTrace("measure_height", first.rejection_reason or objects[0].id),
        ToolTrace("measure_height", second.rejection_reason or other_objects[0].id),
    )
    if first.measurement is None or second.measurement is None:
        return rejected_result(
            context,
            intent,
            [*objects, *other_objects],
            trace,
            first.rejection_reason or second.rejection_reason or "height_measurement_rejected",
        )
    first_lower = first.measurement.value - first.measurement.tolerance
    second_lower = second.measurement.value - second.measurement.tolerance
    first_upper = first.measurement.value + first.measurement.tolerance
    second_upper = second.measurement.value + second.measurement.tolerance
    if first_lower <= second_upper and second_lower <= first_upper:
        return rejected_result(
            context, intent, [*objects, *other_objects], trace, "ambiguous_height_comparison"
        )
    answer = intent.object_query if first_lower > second_upper else intent.comparison_query
    example = VqaExample(
        f"{context.frame.id}-{intent.object_query}-{intent.comparison_query}-height-comparison",
        render_question(intent),
        answer,
        "choice",
        (objects[0].id, other_objects[0].id),
        (intent.object_query, intent.comparison_query),
    )
    return GroundTruthResult(
        intent, example, "answered", answer, None, (objects[0], other_objects[0]), trace
    )


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


def classify_object_on_support(intent: QuestionIntent, context: FamilyContext) -> GroundTruthResult:
    grounded = context.ground(intent.object_query)
    objects, trace = grounded.objects, grounded.trace
    if len(objects) != 1 or intent.comparison_query is None:
        return rejected_result(context, intent, objects, trace, "ambiguous_supported_object")
    support = context.ground(intent.comparison_query)
    supports = support.objects
    trace = (*trace, *support.trace)
    if len(supports) != 1:
        return rejected_result(
            context, intent, [*objects, *supports], trace, "ambiguous_support_object"
        )
    ground_fit = context.primitives.fit_ground_plane()
    support_fit = context.primitives.fit_object_surface_plane(supports[0])
    trace = (
        *trace,
        ToolTrace("fit_ground_plane", ground_fit.rejection_reason or "accepted"),
        ToolTrace("fit_object_surface_plane", support_fit.rejection_reason or "accepted"),
    )
    if ground_fit.estimate is None or support_fit.estimate is None:
        return rejected_result(
            context,
            intent,
            [*objects, *supports],
            trace,
            ground_fit.rejection_reason
            or support_fit.rejection_reason
            or "support_relation_rejected",
        )
    if abs(float(np.dot(ground_fit.estimate.normal, support_fit.estimate.normal))) < np.cos(
        np.radians(12.0)
    ):
        return rejected_result(
            context, intent, [*objects, *supports], trace, "support_not_horizontal"
        )
    relation = context.primitives.measure_object_plane_relation(
        objects[0], supports[0], support_fit.estimate, ground_fit.estimate.normal
    )
    trace = (
        *trace,
        ToolTrace("measure_object_plane_relation", relation.rejection_reason or "accepted"),
    )
    if relation.rejection_reason is not None:
        return rejected_result(
            context, intent, [*objects, *supports], trace, relation.rejection_reason
        )
    if relation.planar_separation_m is not None and relation.planar_separation_m > 0.2:
        answer = "no"
    elif relation.lower_clearance_m is not None and relation.lower_clearance_m > 0.2:
        answer = "no"
    elif (
        relation.lower_clearance_m is not None
        and relation.upper_clearance_m is not None
        and relation.elevated_fraction is not None
        and relation.contact_point_count >= 4
        and relation.lower_clearance_m <= 0.08
        and relation.upper_clearance_m >= 0.15
        and relation.elevated_fraction >= 0.7
        and relation.contact_overlap_count >= 3
    ):
        answer = "yes"
    else:
        return rejected_result(
            context, intent, [*objects, *supports], trace, "insufficient_contact_evidence"
        )
    example = VqaExample(
        f"{context.frame.id}-{intent.object_query}-{intent.comparison_query}-on-support",
        render_question(intent),
        answer,
        "boolean",
        (objects[0].id, supports[0].id),
        ("yes", "no"),
    )
    return GroundTruthResult(
        intent, example, "answered", answer, None, (objects[0], supports[0]), trace
    )


def measure_opening_width(intent: QuestionIntent, context: FamilyContext) -> GroundTruthResult:
    context.primitives.detect_objects(intent.object_query)
    masks = context.primitives.segment_detections(intent.object_query)
    trace: tuple[ToolTrace, ...] = (
        ToolTrace("detect_objects", intent.object_query),
        ToolTrace("segment_detections", f"count={len(masks)}"),
    )
    if len(masks) != 1:
        return rejected_result(context, intent, [], trace, "ambiguous_opening_instances")
    ground_fit = context.primitives.fit_ground_plane()
    trace = (*trace, ToolTrace("fit_ground_plane", ground_fit.rejection_reason or "accepted"))
    if ground_fit.estimate is None:
        return rejected_result(
            context, intent, [], trace, ground_fit.rejection_reason or "ground_plane_rejected"
        )
    result = context.primitives.measure_opening_width_from_mask(masks[0], ground_fit.estimate)
    trace = (*trace, ToolTrace("measure_opening_width", result.rejection_reason or "accepted"))
    if result.measurement is None:
        return rejected_result(
            context, intent, [], trace, result.rejection_reason or "opening_width_rejected"
        )
    answer = opening_width_choice(result.measurement.value)
    example = VqaExample(
        f"{context.frame.id}-{intent.object_query}-opening-width",
        render_question(intent),
        answer,
        "choice",
        (),
        OPENING_WIDTH_CHOICES,
    )
    return GroundTruthResult(intent, example, "answered", answer, None, (), trace)


def compare_nearest_by_side(intent: QuestionIntent, context: FamilyContext) -> GroundTruthResult:
    grounded = context.ground(intent.object_query)
    objects, trace = grounded.objects, grounded.trace
    if not objects:
        return rejected_result(context, intent, objects, trace, "no_grounded_object")
    left = select_nearest_object(list(objects), "left")
    right = select_nearest_object(list(objects), "right")
    if left is None or right is None:
        return rejected_result(context, intent, objects, trace, "missing_grounded_side")
    if left.range_m == right.range_m:
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


def classify_door_state(intent: QuestionIntent, context: FamilyContext) -> GroundTruthResult:
    grounded = context.ground(intent.object_query)
    objects, trace = grounded.objects, grounded.trace
    if "door" not in intent.object_query.lower():
        return rejected_result(context, intent, objects, trace, "door_state_requires_door_query")
    if len(objects) != 1:
        return rejected_result(context, intent, objects, trace, "ambiguous_door_instances")
    door_fit = context.primitives.fit_object_surface_plane(objects[0])
    surrounding_fit = context.primitives.fit_object_surrounding_plane(objects[0])
    trace = (
        *trace,
        ToolTrace("fit_object_surface_plane", door_fit.rejection_reason or "accepted"),
        ToolTrace("fit_mask_surrounding_plane", surrounding_fit.rejection_reason or "accepted"),
    )
    if door_fit.estimate is None or surrounding_fit.estimate is None:
        return rejected_result(
            context,
            intent,
            objects,
            trace,
            door_fit.rejection_reason or surrounding_fit.rejection_reason or "door_state_rejected",
        )
    angle = context.primitives.measure_relative_plane_angle(
        door_fit.estimate, surrounding_fit.estimate
    )
    trace = (*trace, ToolTrace("measure_relative_plane_angle", f"{angle.value:.1f} deg"))
    if angle.value <= 12.0:
        state = "closed"
    elif angle.value >= 25.0:
        state = "open"
    else:
        return rejected_result(context, intent, objects, trace, "ambiguous_door_angle")
    example = VqaExample(
        f"{context.frame.id}-{intent.object_query}-state",
        render_question(intent),
        state,
        "choice",
        (objects[0].id,),
        ("open", "closed"),
    )
    return GroundTruthResult(intent, example, "answered", state, None, tuple(objects), trace)


def select_closest_object(intent: QuestionIntent, context: FamilyContext) -> GroundTruthResult:
    grounded = context.ground(intent.object_query)
    objects, trace = grounded.objects, grounded.trace
    if len(objects) != 1:
        return rejected_result(context, intent, objects, trace, "ambiguous_target_object")
    candidates: list[GroundedObject] = []
    for query in intent.candidate_queries:
        grounded_candidate = context.ground(query)
        matches = grounded_candidate.objects
        trace = (*trace, *grounded_candidate.trace)
        if len(matches) != 1:
            return rejected_result(
                context,
                intent,
                [*objects, *candidates, *matches],
                trace,
                "ambiguous_candidate_object",
            )
        candidates.append(matches[0])
    selected = context.primitives.select_closest_object(objects[0], candidates)
    trace = (
        *trace,
        ToolTrace(
            "select_closest_object",
            selected.object.id if selected.object else selected.rejection_reason or "rejected",
        ),
    )
    if selected.object is None:
        return rejected_result(
            context,
            intent,
            [*objects, *candidates],
            trace,
            selected.rejection_reason or "closest_object_rejected",
        )
    example = VqaExample(
        f"{context.frame.id}-{intent.object_query}-closest-object",
        render_question(intent),
        selected.object.label,
        "choice",
        (objects[0].id, *(item.id for item in candidates)),
        intent.candidate_queries,
    )
    return GroundTruthResult(
        intent,
        example,
        "answered",
        selected.object.label,
        None,
        tuple([*objects, *candidates]),
        trace,
    )


def classify_forward_path(intent: QuestionIntent, context: FamilyContext) -> GroundTruthResult:
    result = context.primitives.classify_forward_path()
    trace = (
        ToolTrace("classify_forward_path", result.state or result.rejection_reason or "rejected"),
    )
    if result.state is None:
        return rejected_result(
            context, intent, [], trace, result.rejection_reason or "forward_path_rejected"
        )
    example = VqaExample(
        f"{context.frame.id}-forward-path",
        render_question(intent),
        result.state,
        "choice",
        (),
        ("clear", "blocked"),
    )
    return GroundTruthResult(intent, example, "answered", result.state, None, (), trace)
