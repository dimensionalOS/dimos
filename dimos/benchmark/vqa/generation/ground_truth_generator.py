# Copyright 2026 Dimensional Inc.
"""Tool-driven private answer generation for single-frame VQA."""

from __future__ import annotations

from dimos.benchmark.vqa.generation.grounding import ground_segmented_objects
from dimos.benchmark.vqa.generation.questions import generate_questions
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
        self._groundings: dict[str, tuple[list[GroundedObject], tuple[ToolTrace, ...]]] = {}
        self._masks: dict[str, list[Detection2DSeg]] = {}
        self._detections: dict[str, list[Detection2DBBox]] = {}
        self._points: dict[str, list[Detection2DPoint]] = {}
        self._overlay_results: list[GroundTruthResult] = []

    def answer(self, frame: CalibratedFrame, intent: QuestionIntent) -> GroundTruthResult:
        cached = self._groundings.get(intent.object_query)
        if cached is not None:
            objects, prior_trace = cached
            result = self._answer_from_objects(
                frame,
                intent,
                objects,
                (ToolTrace("reuse_grounding", intent.object_query), *prior_trace),
            )
            self._overlay_results.append(result)
            return result
        trace: list[ToolTrace] = [ToolTrace("detect_objects", intent.object_query)]
        detections = self._detector.detect(frame.image, intent.object_query)
        self._detections[intent.object_query] = [
            item for item in detections if isinstance(item, Detection2DBBox)
        ]
        if len(detections):
            trace.append(ToolTrace("segment_objects", f"count={len(detections)}"))
            segmented = self._segmenter.segment(detections)
        elif self._localizer is not None and self._point_segmenter is not None:
            trace.append(ToolTrace("locate_object_point", intent.object_query))
            points = self._localizer.locate(frame.image, intent.object_query)
            self._points[intent.object_query] = [
                item for item in points if isinstance(item, Detection2DPoint)
            ]
            trace.append(ToolTrace("segment_object_point", f"count={len(points)}"))
            segmented = self._point_segmenter.segment_points(points)
        else:
            segmented = detections
        masks = [
            item
            for item in segmented
            if isinstance(item, Detection2DSeg)
            and int((item.mask > 0).sum()) >= self._config.min_mask_area_px
        ]
        trace.append(ToolTrace("get_foreground_geometry", f"masks={len(masks)}"))
        self._masks[intent.object_query] = masks
        objects = ground_segmented_objects(
            frame, masks, min_foreground_points=self._config.min_foreground_points
        )
        stored_trace = tuple(trace)
        self._groundings[intent.object_query] = (objects, stored_trace)
        result = self._answer_from_objects(frame, intent, objects, stored_trace)
        self._overlay_results.append(result)
        return result

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

    def write_overlay(self, frame: CalibratedFrame, path: str) -> None:
        """Save private masks, detector prompts, and a per-question audit legend."""
        import cv2
        import numpy as np

        overlay = frame.image.data.copy()
        colors = ((255, 128, 0), (0, 200, 255), (180, 0, 255), (0, 200, 80))
        question_labels: dict[str, list[str]] = {}
        for index, result in enumerate(self._overlay_results, start=1):
            question_labels.setdefault(result.intent.object_query, []).append(f"Q{index}")
        for index, query in enumerate(question_labels):
            color = colors[index % len(colors)]
            label = ",".join(question_labels[query])
            for mask in self._masks.get(query, []):
                overlay[mask.mask > 0] = color
                x1, y1, _, _ = map(int, mask.bbox)
                cv2.putText(
                    overlay,
                    f"{label} {query}",
                    (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1,
                )
            for detection in self._detections.get(query, []):
                x1, y1, x2, y2 = map(int, detection.bbox)
                cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    overlay,
                    f"{label} box",
                    (x1, min(y2 + 16, frame.image.height - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1,
                )
            for point in self._points.get(query, []):
                position = (int(point.x), int(point.y))
                cv2.drawMarker(overlay, position, color, cv2.MARKER_CROSS, 14, 2)
                cv2.putText(
                    overlay,
                    f"{label} point",
                    (position[0] + 8, max(position[1] - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1,
                )
        rendered = cv2.addWeighted(frame.image.data, 0.55, overlay, 0.45, 0)
        legend_lines = ["Private grounding audit"]
        for index, result in enumerate(self._overlay_results, start=1):
            status = result.answer if result.status == "answered" else f"rejected: {result.reason}"
            legend_lines.extend(_wrap_overlay_text(f"Q{index}: {result.question.question}", 52))
            legend_lines.extend(_wrap_overlay_text(f"  {status}", 52))
        line_height = 20
        panel_width = 440
        panel_height = max(rendered.shape[0], 16 + line_height * len(legend_lines))
        audit = np.full((panel_height, rendered.shape[1] + panel_width, 3), 32, dtype=np.uint8)
        audit[: rendered.shape[0], : rendered.shape[1]] = rendered
        for index, line in enumerate(legend_lines):
            cv2.putText(
                audit,
                line,
                (rendered.shape[1] + 12, 20 + index * line_height),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
            )
        if not cv2.imwrite(path, audit):
            raise RuntimeError(f"failed to write {path}")


def _render_question(intent: QuestionIntent) -> str:
    if intent.kind == "presence":
        return f"Is there a {intent.object_query} in the image? Answer yes or no."
    if intent.kind == "horizontal_direction":
        return f"Where is the nearest {intent.object_query}: left, center, or right?"
    if intent.kind == "compare_nearest_by_side":
        return f"Which {intent.object_query} is closer: the left one or the right one?"
    return f"Is the nearest {intent.object_query} within {intent.threshold_m or 3:g} meters? Answer yes or no."


def _wrap_overlay_text(text: str, width: int) -> list[str]:
    words = text.split()
    lines: list[str] = []
    line = ""
    for word in words:
        candidate = f"{line} {word}".strip()
        if line and len(candidate) > width:
            lines.append(line)
            line = word
        else:
            line = candidate
    if line:
        lines.append(line)
    return lines


def _compare_nearest_by_side(
    frame: CalibratedFrame,
    intent: QuestionIntent,
    objects: list[GroundedObject],
    trace: tuple[ToolTrace, ...],
) -> GroundTruthResult:
    left_matches = [item for item in objects if item.horizontal_direction == "left"]
    right_matches = [item for item in objects if item.horizontal_direction == "right"]
    if not left_matches or not right_matches:
        return _rejected_result(frame, intent, objects, trace, "missing_grounded_side")
    left = min(left_matches, key=lambda item: item.range_m)
    right = min(right_matches, key=lambda item: item.range_m)
    if left.range_m == right.range_m:
        return _rejected_result(frame, intent, objects, trace, "ambiguous_nearest_by_side")
    answer = "left" if left.range_m < right.range_m else "right"
    example = VqaExample(
        f"{frame.id}-{intent.object_query}-nearest-by-side",
        _render_question(intent),
        answer,
        "choice",
        (left.id, right.id),
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
