# Copyright 2026 Dimensional Inc.
"""Tool-driven private answer generation for single-frame VQA."""

from __future__ import annotations

from dimos.benchmark.vqa.generation.primitives.choices import (
    CAMERA_RANGE_CHOICES,
    COUNT_CHOICES,
    camera_range_choice,
    count_choice,
)
from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
from dimos.benchmark.vqa.generation.primitives.selection import select_nearest_object
from dimos.benchmark.vqa.generation.questions import generate_questions
from dimos.benchmark.vqa.models import (
    CalibratedFrame,
    GroundedObject,
    GroundTruthResult,
    QuestionIntent,
    ToolTrace,
    VqaExample,
)


class VqaGroundTruthGenerator:
    """Answer constrained questions by calling detection, segmentation, and geometry tools."""

    def __init__(self, primitives: FramePerceptionPrimitives) -> None:
        self.primitives = primitives

    def answer(self, frame: CalibratedFrame, intent: QuestionIntent) -> GroundTruthResult:
        if intent.kind == "forward_path":
            return _classify_forward_path(frame, intent, self.primitives)
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
        objects = self.primitives.ground_masks(object_query)
        return objects, tuple(trace)

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
        if intent.kind == "visible_count":
            return _count_visible_objects(frame, intent, objects, trace)
        if intent.kind == "camera_range":
            return _bucket_camera_range(frame, intent, objects, trace)
        if intent.kind == "compare_left_right":
            return _compare_left_right(frame, intent, objects, trace, self)
        if intent.kind == "compare_height":
            return _compare_heights(frame, intent, objects, trace, self)
        if intent.kind == "door_state":
            return _classify_door_state(frame, intent, objects, trace, self.primitives)
        if intent.kind == "closest_object":
            return _select_closest_object(frame, intent, objects, trace, self)
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
    if intent.kind == "door_state":
        return f"Is the {intent.object_query} open or closed?"
    if intent.kind == "closest_object":
        return f"Which object is closest to the {intent.object_query}: {', '.join(intent.candidate_queries)}?"
    if intent.kind == "forward_path":
        return "Is the path directly ahead clear or blocked?"
    return f"Is the nearest {intent.object_query} within {intent.threshold_m or 3:g} meters? Answer yes or no."


def _count_visible_objects(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
) -> GroundTruthResult:
    answer = count_choice(len(objects))
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-visible-count",
        _render_question(intent),
        answer,
        "choice",
        tuple(item.id for item in objects),
        COUNT_CHOICES,
    )
    return GroundTruthResult(intent, example, "answered", answer, None, tuple(objects), trace)


def _bucket_camera_range(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
) -> GroundTruthResult:
    selected = select_nearest_object(objects)
    if selected is None:
        return _rejected_result(frame, intent, objects, trace, "no_grounded_object")
    answer = camera_range_choice(selected.range_m)
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-camera-range",
        _render_question(intent),
        answer,
        "choice",
        (selected.id,),
        CAMERA_RANGE_CHOICES,
    )
    return GroundTruthResult(intent, example, "answered", answer, None, (selected,), trace)


def _compare_heights(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
    generator: VqaGroundTruthGenerator,
) -> GroundTruthResult:
    if len(objects) != 1 or intent.comparison_query is None:
        return _rejected_result(frame, intent, objects, trace, "ambiguous_first_height_object")
    other_objects, other_trace = generator.ground(frame, intent.comparison_query)
    trace = (*trace, *other_trace)
    if len(other_objects) != 1:
        return _rejected_result(
            frame, intent, [*objects, *other_objects], trace, "ambiguous_second_height_object"
        )
    plane_fit = generator.primitives.fit_ground_plane()
    trace = (*trace, ToolTrace("fit_ground_plane", plane_fit.rejection_reason or "accepted"))
    if plane_fit.estimate is None:
        return _rejected_result(
            frame,
            intent,
            [*objects, *other_objects],
            trace,
            plane_fit.rejection_reason or "ground_plane_rejected",
        )
    first = generator.primitives.measure_height(objects[0], plane_fit.estimate)
    second = generator.primitives.measure_height(other_objects[0], plane_fit.estimate)
    trace = (
        *trace,
        ToolTrace("measure_height", first.rejection_reason or objects[0].id),
        ToolTrace("measure_height", second.rejection_reason or other_objects[0].id),
    )
    if first.measurement is None or second.measurement is None:
        return _rejected_result(
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
        return _rejected_result(
            frame, intent, [*objects, *other_objects], trace, "ambiguous_height_comparison"
        )
    answer = intent.object_query if first_lower > second_upper else intent.comparison_query
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-{intent.comparison_query}-height-comparison",
        _render_question(intent),
        answer,
        "choice",
        (objects[0].id, other_objects[0].id),
        (intent.object_query, intent.comparison_query),
    )
    return GroundTruthResult(
        intent, example, "answered", answer, None, (objects[0], other_objects[0]), trace
    )


def _compare_left_right(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
    generator: VqaGroundTruthGenerator,
) -> GroundTruthResult:
    if len(objects) != 1 or intent.comparison_query is None:
        return _rejected_result(frame, intent, objects, trace, "ambiguous_first_relation_object")
    other_objects, other_trace = generator.ground(frame, intent.comparison_query)
    trace = (*trace, *other_trace)
    if len(other_objects) != 1:
        return _rejected_result(
            frame, intent, [*objects, *other_objects], trace, "ambiguous_second_relation_object"
        )
    relation = generator.primitives.classify_horizontal_relation(objects[0], other_objects[0])
    trace = (
        *trace,
        ToolTrace(
            "classify_horizontal_relation",
            relation.relation or relation.rejection_reason or "rejected",
        ),
    )
    if relation.relation is None:
        return _rejected_result(
            frame,
            intent,
            [*objects, *other_objects],
            trace,
            relation.rejection_reason or "horizontal_relation_rejected",
        )
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-{intent.comparison_query}-left-right",
        _render_question(intent),
        relation.relation,
        "choice",
        (objects[0].id, other_objects[0].id),
        ("left", "right"),
    )
    return GroundTruthResult(
        intent,
        example,
        "answered",
        relation.relation,
        None,
        (objects[0], other_objects[0]),
        trace,
    )


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


def _classify_door_state(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
    primitives: FramePerceptionPrimitives,
) -> GroundTruthResult:
    if "door" not in intent.object_query.lower():
        return _rejected_result(frame, intent, objects, trace, "door_state_requires_door_query")
    if len(objects) != 1:
        return _rejected_result(frame, intent, objects, trace, "ambiguous_door_instances")
    result = primitives.classify_door_state(objects[0])
    trace = (
        *trace,
        ToolTrace("classify_door_state", result.state or result.rejection_reason or "rejected"),
    )
    if result.state is None:
        return _rejected_result(
            frame, intent, objects, trace, result.rejection_reason or "door_state_rejected"
        )
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-state",
        _render_question(intent),
        result.state,
        "choice",
        (objects[0].id,),
        ("open", "closed"),
    )
    return GroundTruthResult(intent, example, "answered", result.state, None, tuple(objects), trace)


def _select_closest_object(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
    generator: VqaGroundTruthGenerator,
) -> GroundTruthResult:
    if len(objects) != 1:
        return _rejected_result(frame, intent, objects, trace, "ambiguous_target_object")
    candidates: list[GroundedObject] = []
    for query in intent.candidate_queries:
        matches, candidate_trace = generator.ground(frame, query)
        trace = (*trace, *candidate_trace)
        if len(matches) != 1:
            return _rejected_result(
                frame,
                intent,
                [*objects, *candidates, *matches],
                trace,
                "ambiguous_candidate_object",
            )
        candidates.append(matches[0])
    selected = generator.primitives.select_closest_object(objects[0], candidates)
    trace = (
        *trace,
        ToolTrace(
            "select_closest_object",
            selected.object.id if selected.object else selected.rejection_reason or "rejected",
        ),
    )
    if selected.object is None:
        return _rejected_result(
            frame,
            intent,
            [*objects, *candidates],
            trace,
            selected.rejection_reason or "closest_object_rejected",
        )
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-closest-object",
        _render_question(intent),
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


def _classify_forward_path(
    frame: CalibratedFrame, intent: QuestionIntent, primitives: FramePerceptionPrimitives
) -> GroundTruthResult:
    result = primitives.classify_forward_path()
    trace = (
        ToolTrace("classify_forward_path", result.state or result.rejection_reason or "rejected"),
    )
    if result.state is None:
        return _rejected_result(
            frame, intent, [], trace, result.rejection_reason or "forward_path_rejected"
        )
    example = VqaExample(
        f"{frame.id}-forward-path",
        _render_question(intent),
        result.state,
        "choice",
        (),
        ("clear", "blocked"),
    )
    return GroundTruthResult(intent, example, "answered", result.state, None, (), trace)


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
