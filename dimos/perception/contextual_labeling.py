# Copyright 2025-2026 Dimensional Inc.
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

"""Contextual labeling: educated guesses for ambiguously-detected objects.

Given a detected :class:`~dimos.perception.detection.type.detection3d.object.Object`
(raw YOLO label + image crop + world-frame geometry) and the rest of the
observed scene, ask a vision-language model for a richer label. The VLM sees
the cropped detection image plus text context the detector doesn't use:
the detector's own guess and confidence, the object's real-world size, and
what else is nearby (a "mouse" next to a keyboard and monitor is more likely
a computer mouse than an animal).

Pure helpers — no module/stream wiring — so they are unit-testable with a
stub VLM and reusable outside ObjectSceneRegistrationModule.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Sequence

    from dimos.models.vl.base import VlModel
    from dimos.perception.detection.type.detection3d.object import Object

logger = setup_logger()

# Detector outputs that carry no real identification signal.
GENERIC_NAMES = frozenset({"object", "objects", "thing", "things", "item", "items", "unknown"})

DEFAULT_CONFIDENCE_THRESHOLD = 0.5


@dataclass(frozen=True)
class ContextualLabel:
    """A VLM's educated guess about what an object is."""

    label: str
    description: str
    confidence: float


def is_ambiguous(obj: Object, confidence_threshold: float = DEFAULT_CONFIDENCE_THRESHOLD) -> bool:
    """Whether a detection's label is worth refining with the VLM.

    True when the detector was unsure (low confidence) or produced a label
    with no identification value (empty, generic, or a raw class id).
    An already-refined object is never ambiguous — without this, every
    catalog_scene()/refine_observed_labels() pass would re-query the VLM for
    the same objects (repeat latency, churned labels between calls).
    identify_object() bypasses this gate for explicit re-refinement.
    """
    if obj.refined_name:
        return False
    if obj.confidence < confidence_threshold:
        return True
    name = (obj.name or "").strip().lower()
    return not name or name in GENERIC_NAMES or name.startswith("class_")


def describe_neighbors(obj: Object, scene: Sequence[Object], max_neighbors: int = 8) -> list[str]:
    """Human-readable nearest-neighbor context lines, e.g. ``keyboard (0.4m away)``.

    Skips the object itself and anything without a center; nearest first.
    """
    neighbors = [
        other
        for other in scene
        if other.object_id != obj.object_id and other.center is not None and obj.center is not None
    ]
    neighbors.sort(key=lambda other: obj.center.distance(other.center))
    return [
        f"{other.display_name} ({obj.center.distance(other.center):.1f}m away)"
        for other in neighbors[:max_neighbors]
    ]


def build_context_prompt(obj: Object, scene: Sequence[Object] = ()) -> str:
    """Build the VLM prompt describing everything known about the detection."""
    lines = [
        "You are helping a robot identify an object it detected. The image is",
        "cropped around the detected object.",
        f'The robot\'s detector guessed "{obj.name}" with confidence {obj.confidence:.2f}.',
    ]
    if obj.size is not None:
        lines.append(
            f"Measured real-world size: {obj.size.x:.2f}m x {obj.size.y:.2f}m x {obj.size.z:.2f}m."
        )
    neighbor_lines = describe_neighbors(obj, scene)
    if neighbor_lines:
        lines.append("Other objects observed nearby: " + ", ".join(neighbor_lines) + ".")
    lines += [
        "Using the image and this context, give your best guess of what the object",
        "actually is. Prefer a specific name over a generic one.",
        "Respond ONLY with JSON, no other text:",
        '{"label": "<short specific name>", "description": "<one sentence about the object>",',
        ' "confidence": <your confidence 0.0-1.0>}',
    ]
    return "\n".join(lines)


def contextual_label(
    vl_model: VlModel,
    obj: Object,
    scene: Sequence[Object] = (),
    crop_padding: int = 20,
) -> ContextualLabel | None:
    """Ask the VLM for an educated guess about one object.

    Returns None when the object has no stored image or the VLM response is
    unusable (query failure or malformed JSON) — the caller keeps the raw
    detector label in that case.
    """
    if obj.image is None:
        logger.debug(f"Object {obj.object_id} ({obj.name}) has no image; cannot label")
        return None

    prompt = build_context_prompt(obj, scene)
    try:
        crop = obj.cropped_image(padding=crop_padding)
        response = vl_model.query_json(crop, prompt)
    except Exception:
        logger.warning(f"VLM contextual labeling failed for {obj.object_id} ({obj.name})")
        return None

    label = str(response.get("label", "")).strip() if isinstance(response, dict) else ""
    if not label:
        logger.debug(f"VLM returned no label for {obj.object_id}: {response!r}")
        return None

    description = str(response.get("description", "")).strip()
    try:
        confidence = min(1.0, max(0.0, float(response.get("confidence", 0.5))))
    except (TypeError, ValueError):
        confidence = 0.5

    return ContextualLabel(label=label, description=description, confidence=confidence)
