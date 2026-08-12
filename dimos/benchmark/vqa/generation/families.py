# Copyright 2026 Dimensional Inc.
"""Recipes and result construction for constrained single-frame VQA families."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from dimos.benchmark.vqa.generation.primitives.choices import (
    CAMERA_RANGE_CHOICES,
    COUNT_CHOICES,
    OPENING_WIDTH_CHOICES,
    camera_range_choice,
    count_choice,
    opening_width_choice,
)
from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
from dimos.benchmark.vqa.generation.primitives.selection import select_nearest_object
from dimos.benchmark.vqa.models import (
    CalibratedFrame,
    GroundedObject,
    GroundTruthResult,
    QuestionIntent,
    ToolTrace,
    VqaExample,
)

Ground = Callable[[CalibratedFrame, str], tuple[list[GroundedObject], tuple[ToolTrace, ...]]]


def render_question(intent: QuestionIntent) -> str:
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
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
    reason: str,
) -> GroundTruthResult:
    rejected = VqaExample(
        f"{frame.id}-{intent.object_query}-{intent.kind}", render_question(intent), "", "", ()
    )
    return GroundTruthResult(intent, rejected, "rejected", None, reason, tuple(objects), trace)


def count_visible_objects(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
) -> GroundTruthResult:
    answer = count_choice(len(objects))
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-visible-count",
        render_question(intent),
        answer,
        "choice",
        tuple(item.id for item in objects),
        COUNT_CHOICES,
    )
    return GroundTruthResult(intent, example, "answered", answer, None, tuple(objects), trace)


def bucket_camera_range(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
) -> GroundTruthResult:
    selected = select_nearest_object(objects)
    if selected is None:
        return rejected_result(frame, intent, objects, trace, "no_grounded_object")
    answer = camera_range_choice(selected.range_m)
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-camera-range",
        render_question(intent),
        answer,
        "choice",
        (selected.id,),
        CAMERA_RANGE_CHOICES,
    )
    return GroundTruthResult(intent, example, "answered", answer, None, (selected,), trace)


def compare_heights(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
    primitives: FramePerceptionPrimitives,
    ground: Ground,
) -> GroundTruthResult:
    if len(objects) != 1 or intent.comparison_query is None:
        return rejected_result(frame, intent, objects, trace, "ambiguous_first_height_object")
    other_objects, other_trace = ground(frame, intent.comparison_query)
    trace = (*trace, *other_trace)
    if len(other_objects) != 1:
        return rejected_result(
            frame, intent, [*objects, *other_objects], trace, "ambiguous_second_height_object"
        )
    plane_fit = primitives.fit_ground_plane()
    trace = (*trace, ToolTrace("fit_ground_plane", plane_fit.rejection_reason or "accepted"))
    if plane_fit.estimate is None:
        return rejected_result(
            frame,
            intent,
            [*objects, *other_objects],
            trace,
            plane_fit.rejection_reason or "ground_plane_rejected",
        )
    first = primitives.measure_height(objects[0], plane_fit.estimate)
    second = primitives.measure_height(other_objects[0], plane_fit.estimate)
    trace = (
        *trace,
        ToolTrace("measure_height", first.rejection_reason or objects[0].id),
        ToolTrace("measure_height", second.rejection_reason or other_objects[0].id),
    )
    if first.measurement is None or second.measurement is None:
        return rejected_result(
            frame,
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
            frame, intent, [*objects, *other_objects], trace, "ambiguous_height_comparison"
        )
    answer = intent.object_query if first_lower > second_upper else intent.comparison_query
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-{intent.comparison_query}-height-comparison",
        render_question(intent),
        answer,
        "choice",
        (objects[0].id, other_objects[0].id),
        (intent.object_query, intent.comparison_query),
    )
    return GroundTruthResult(
        intent, example, "answered", answer, None, (objects[0], other_objects[0]), trace
    )


def compare_left_right(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
    primitives: FramePerceptionPrimitives,
    ground: Ground,
) -> GroundTruthResult:
    if len(objects) != 1 or intent.comparison_query is None:
        return rejected_result(frame, intent, objects, trace, "ambiguous_first_relation_object")
    other_objects, other_trace = ground(frame, intent.comparison_query)
    trace = (*trace, *other_trace)
    if len(other_objects) != 1:
        return rejected_result(
            frame, intent, [*objects, *other_objects], trace, "ambiguous_second_relation_object"
        )
    relation = primitives.classify_horizontal_relation(objects[0], other_objects[0])
    trace = (
        *trace,
        ToolTrace(
            "classify_horizontal_relation",
            relation.relation or relation.rejection_reason or "rejected",
        ),
    )
    if relation.relation is None:
        return rejected_result(
            frame,
            intent,
            [*objects, *other_objects],
            trace,
            relation.rejection_reason or "horizontal_relation_rejected",
        )
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-{intent.comparison_query}-left-right",
        render_question(intent),
        relation.relation,
        "choice",
        (objects[0].id, other_objects[0].id),
        ("left", "right"),
    )
    return GroundTruthResult(
        intent, example, "answered", relation.relation, None, (objects[0], other_objects[0]), trace
    )


def classify_object_on_support(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
    primitives: FramePerceptionPrimitives,
    ground: Ground,
) -> GroundTruthResult:
    if len(objects) != 1 or intent.comparison_query is None:
        return rejected_result(frame, intent, objects, trace, "ambiguous_supported_object")
    supports, support_trace = ground(frame, intent.comparison_query)
    trace = (*trace, *support_trace)
    if len(supports) != 1:
        return rejected_result(
            frame, intent, [*objects, *supports], trace, "ambiguous_support_object"
        )
    ground_fit = primitives.fit_ground_plane()
    support_fit = primitives.fit_object_surface_plane(supports[0])
    trace = (
        *trace,
        ToolTrace("fit_ground_plane", ground_fit.rejection_reason or "accepted"),
        ToolTrace("fit_object_surface_plane", support_fit.rejection_reason or "accepted"),
    )
    if ground_fit.estimate is None or support_fit.estimate is None:
        return rejected_result(
            frame,
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
            frame, intent, [*objects, *supports], trace, "support_not_horizontal"
        )
    relation = primitives.measure_object_plane_relation(
        objects[0], supports[0], support_fit.estimate, ground_fit.estimate.normal
    )
    trace = (
        *trace,
        ToolTrace("measure_object_plane_relation", relation.rejection_reason or "accepted"),
    )
    if relation.rejection_reason is not None:
        return rejected_result(
            frame, intent, [*objects, *supports], trace, relation.rejection_reason
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
            frame, intent, [*objects, *supports], trace, "insufficient_contact_evidence"
        )
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-{intent.comparison_query}-on-support",
        render_question(intent),
        answer,
        "boolean",
        (objects[0].id, supports[0].id),
        ("yes", "no"),
    )
    return GroundTruthResult(
        intent, example, "answered", answer, None, (objects[0], supports[0]), trace
    )


def measure_opening_width(
    frame: CalibratedFrame, intent: QuestionIntent, primitives: FramePerceptionPrimitives
) -> GroundTruthResult:
    primitives.detect_objects(intent.object_query)
    masks = primitives.segment_detections(intent.object_query)
    trace: tuple[ToolTrace, ...] = (
        ToolTrace("detect_objects", intent.object_query),
        ToolTrace("segment_detections", f"count={len(masks)}"),
    )
    if len(masks) != 1:
        return rejected_result(frame, intent, [], trace, "ambiguous_opening_instances")
    ground_fit = primitives.fit_ground_plane()
    trace = (*trace, ToolTrace("fit_ground_plane", ground_fit.rejection_reason or "accepted"))
    if ground_fit.estimate is None:
        return rejected_result(
            frame, intent, [], trace, ground_fit.rejection_reason or "ground_plane_rejected"
        )
    result = primitives.measure_opening_width_from_mask(masks[0], ground_fit.estimate)
    trace = (*trace, ToolTrace("measure_opening_width", result.rejection_reason or "accepted"))
    if result.measurement is None:
        return rejected_result(
            frame, intent, [], trace, result.rejection_reason or "opening_width_rejected"
        )
    answer = opening_width_choice(result.measurement.value)
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-opening-width",
        render_question(intent),
        answer,
        "choice",
        (),
        OPENING_WIDTH_CHOICES,
    )
    return GroundTruthResult(intent, example, "answered", answer, None, (), trace)


def compare_nearest_by_side(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
) -> GroundTruthResult:
    left = select_nearest_object(objects, "left")
    right = select_nearest_object(objects, "right")
    if left is None or right is None:
        return rejected_result(frame, intent, objects, trace, "missing_grounded_side")
    if left.range_m == right.range_m:
        return rejected_result(frame, intent, objects, trace, "ambiguous_nearest_by_side")
    answer = "left" if left.range_m < right.range_m else "right"
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-nearest-by-side",
        render_question(intent),
        answer,
        "choice",
        (left.id, right.id),
        ("left", "right"),
    )
    return GroundTruthResult(intent, example, "answered", answer, None, (left, right), trace)


def classify_door_state(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
    primitives: FramePerceptionPrimitives,
) -> GroundTruthResult:
    if "door" not in intent.object_query.lower():
        return rejected_result(frame, intent, objects, trace, "door_state_requires_door_query")
    if len(objects) != 1:
        return rejected_result(frame, intent, objects, trace, "ambiguous_door_instances")
    door_fit = primitives.fit_object_surface_plane(objects[0])
    surrounding_fit = primitives.fit_object_surrounding_plane(objects[0])
    trace = (
        *trace,
        ToolTrace("fit_object_surface_plane", door_fit.rejection_reason or "accepted"),
        ToolTrace("fit_mask_surrounding_plane", surrounding_fit.rejection_reason or "accepted"),
    )
    if door_fit.estimate is None or surrounding_fit.estimate is None:
        return rejected_result(
            frame,
            intent,
            objects,
            trace,
            door_fit.rejection_reason or surrounding_fit.rejection_reason or "door_state_rejected",
        )
    angle = primitives.measure_relative_plane_angle(door_fit.estimate, surrounding_fit.estimate)
    trace = (*trace, ToolTrace("measure_relative_plane_angle", f"{angle.value:.1f} deg"))
    if angle.value <= 12.0:
        state = "closed"
    elif angle.value >= 25.0:
        state = "open"
    else:
        return rejected_result(frame, intent, objects, trace, "ambiguous_door_angle")
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-state",
        render_question(intent),
        state,
        "choice",
        (objects[0].id,),
        ("open", "closed"),
    )
    return GroundTruthResult(intent, example, "answered", state, None, tuple(objects), trace)


def select_closest_object(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
    primitives: FramePerceptionPrimitives,
    ground: Ground,
) -> GroundTruthResult:
    if len(objects) != 1:
        return rejected_result(frame, intent, objects, trace, "ambiguous_target_object")
    candidates: list[GroundedObject] = []
    for query in intent.candidate_queries:
        matches, candidate_trace = ground(frame, query)
        trace = (*trace, *candidate_trace)
        if len(matches) != 1:
            return rejected_result(
                frame,
                intent,
                [*objects, *candidates, *matches],
                trace,
                "ambiguous_candidate_object",
            )
        candidates.append(matches[0])
    selected = primitives.select_closest_object(objects[0], candidates)
    trace = (
        *trace,
        ToolTrace(
            "select_closest_object",
            selected.object.id if selected.object else selected.rejection_reason or "rejected",
        ),
    )
    if selected.object is None:
        return rejected_result(
            frame,
            intent,
            [*objects, *candidates],
            trace,
            selected.rejection_reason or "closest_object_rejected",
        )
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-closest-object",
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


def classify_forward_path(
    frame: CalibratedFrame, intent: QuestionIntent, primitives: FramePerceptionPrimitives
) -> GroundTruthResult:
    result = primitives.classify_forward_path()
    trace = (
        ToolTrace("classify_forward_path", result.state or result.rejection_reason or "rejected"),
    )
    if result.state is None:
        return rejected_result(
            frame, intent, [], trace, result.rejection_reason or "forward_path_rejected"
        )
    example = VqaExample(
        f"{frame.id}-forward-path",
        render_question(intent),
        result.state,
        "choice",
        (),
        ("clear", "blocked"),
    )
    return GroundTruthResult(intent, example, "answered", result.state, None, (), trace)
