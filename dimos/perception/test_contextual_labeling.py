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

"""Unit tests for contextual labeling (skill wishlist item 5).

Use a stub VLM so no model weights, GPU, or API access is needed.
"""

from typing import Any

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.contextual_labeling import (
    ContextualLabel,
    build_context_prompt,
    contextual_label,
    describe_neighbors,
    is_ambiguous,
)
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.test_object_scene_registration import _make_object


class FakeVlModel:
    """Stands in for VlModel: records the query, returns a canned response."""

    def __init__(self, response: Any) -> None:
        self.response = response
        self.queries: list[str] = []

    def query_json(self, image: Any, query: str) -> Any:
        self.queries.append(query)
        if isinstance(self.response, Exception):
            raise self.response
        return self.response

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


# ─────────────────────────────────────────────────────────────────
# is_ambiguous
# ─────────────────────────────────────────────────────────────────


def test_is_ambiguous_low_confidence() -> None:
    assert is_ambiguous(_make_object("cup", Vector3(0, 0, 0), confidence=0.3))


def test_is_ambiguous_generic_or_classid_name() -> None:
    assert is_ambiguous(_make_object("object", Vector3(0, 0, 0), confidence=0.9))
    assert is_ambiguous(_make_object("class_17", Vector3(0, 0, 0), confidence=0.9))
    assert is_ambiguous(_make_object("", Vector3(0, 0, 0), confidence=0.9))


def test_confident_specific_name_is_not_ambiguous() -> None:
    assert not is_ambiguous(_make_object("keyboard", Vector3(0, 0, 0), confidence=0.9))


# ─────────────────────────────────────────────────────────────────
# prompt building
# ─────────────────────────────────────────────────────────────────


def test_prompt_includes_detector_guess_size_and_neighbors() -> None:
    obj = _make_object("vase", Vector3(0.0, 0.0, 0.0), confidence=0.34)
    keyboard = _make_object("keyboard", Vector3(0.5, 0.0, 0.0), track_id=2)
    monitor = _make_object("monitor", Vector3(0.0, 1.0, 0.0), track_id=3)

    prompt = build_context_prompt(obj, [obj, keyboard, monitor])

    assert '"vase"' in prompt
    assert "0.34" in prompt
    assert "0.10m x 0.20m x 0.30m" in prompt  # _make_object size
    assert "keyboard (0.5m away)" in prompt
    assert "monitor (1.0m away)" in prompt
    assert "JSON" in prompt


def test_describe_neighbors_sorted_and_excludes_self() -> None:
    obj = _make_object("cup", Vector3(0.0, 0.0, 0.0))
    far = _make_object("chair", Vector3(3.0, 0.0, 0.0), track_id=2)
    near = _make_object("mouse", Vector3(0.2, 0.0, 0.0), track_id=3)

    lines = describe_neighbors(obj, [obj, far, near])

    assert lines == ["mouse (0.2m away)", "chair (3.0m away)"]


def test_describe_neighbors_caps_count() -> None:
    obj = _make_object("cup", Vector3(0.0, 0.0, 0.0))
    scene = [obj] + [
        _make_object(f"item{i}", Vector3(float(i), 0.0, 0.0), track_id=10 + i) for i in range(1, 12)
    ]
    assert len(describe_neighbors(obj, scene, max_neighbors=8)) == 8


# ─────────────────────────────────────────────────────────────────
# contextual_label
# ─────────────────────────────────────────────────────────────────


def test_contextual_label_happy_path() -> None:
    vlm = FakeVlModel(
        {"label": "coffee mug", "description": "A white ceramic mug.", "confidence": 0.85}
    )
    obj = _make_object("vase", Vector3(0.0, 0.0, 0.0), confidence=0.3)

    result = contextual_label(vlm, obj)  # type: ignore[arg-type]

    assert result == ContextualLabel(
        label="coffee mug", description="A white ceramic mug.", confidence=0.85
    )
    assert len(vlm.queries) == 1
    assert '"vase"' in vlm.queries[0]


def test_contextual_label_clamps_and_defaults_confidence() -> None:
    vlm = FakeVlModel({"label": "mug", "confidence": 3.0})
    result = contextual_label(vlm, _make_object("vase", Vector3(0, 0, 0)))  # type: ignore[arg-type]
    assert result is not None
    assert result.confidence == 1.0
    assert result.description == ""

    vlm = FakeVlModel({"label": "mug", "confidence": "high"})
    result = contextual_label(vlm, _make_object("vase", Vector3(0, 0, 0)))  # type: ignore[arg-type]
    assert result is not None
    assert result.confidence == 0.5


def test_contextual_label_rejects_bad_responses() -> None:
    obj = _make_object("vase", Vector3(0, 0, 0))
    assert contextual_label(FakeVlModel({}), obj) is None  # type: ignore[arg-type]
    assert contextual_label(FakeVlModel({"label": "  "}), obj) is None  # type: ignore[arg-type]
    assert contextual_label(FakeVlModel(["not", "a", "dict"]), obj) is None  # type: ignore[arg-type]
    assert contextual_label(FakeVlModel(RuntimeError("vlm down")), obj) is None  # type: ignore[arg-type]


def test_contextual_label_requires_image() -> None:
    obj = _make_object("vase", Vector3(0, 0, 0))
    obj.image = None
    assert contextual_label(FakeVlModel({"label": "mug"}), obj) is None  # type: ignore[arg-type]


# ─────────────────────────────────────────────────────────────────
# Object refined-name plumbing
# ─────────────────────────────────────────────────────────────────


def test_display_name_prefers_refined() -> None:
    obj = _make_object("vase", Vector3(0, 0, 0))
    assert obj.display_name == "vase"
    obj.refined_name = "coffee mug"
    assert obj.display_name == "coffee mug"


def test_locate_encode_surfaces_refined_name_and_detector_name() -> None:
    obj = _make_object("vase", Vector3(0, 0, 0))
    obj.refined_name = "coffee mug"
    obj.refined_description = "A white ceramic mug."

    encoded = obj.locate_encode()

    assert encoded["name"] == "coffee mug"
    assert encoded["detector_name"] == "vase"
    assert encoded["description"] == "A white ceramic mug."


def test_update_object_preserves_refinement() -> None:
    obj = _make_object("vase", Vector3(0, 0, 0))
    obj.refined_name = "coffee mug"

    obj.update_object(_make_object("cup", Vector3(0.01, 0, 0)))

    assert obj.name == "cup"  # raw detector label tracks the latest detection
    assert obj.refined_name == "coffee mug"  # educated guess sticks


# ─────────────────────────────────────────────────────────────────
# Module skills (VLM injected, no model download)
# ─────────────────────────────────────────────────────────────────


def _module_with_fake_vlm(response: Any) -> ObjectSceneRegistrationModule:
    module = ObjectSceneRegistrationModule(localization="lidar", min_detections_for_permanent=1)
    module._vl_model = FakeVlModel(response)
    return module


def test_identify_object_skill_refines_and_persists() -> None:
    module = _module_with_fake_vlm(
        {"label": "coffee mug", "description": "A white ceramic mug.", "confidence": 0.85}
    )
    try:
        inserted = module._object_db.add_objects(
            [_make_object("vase", Vector3(1.0, 2.0, 3.0), confidence=0.3)]
        )
        module._object_db.promote(inserted[0].object_id)

        result = module.identify_object(inserted[0].object_id)

        assert "coffee mug" in result
        assert "vase" in result
        # Refined name shows up in the scene inventory too.
        assert "coffee mug" in module.list_observed_items()
        assert module._object_db.find_by_name("coffee mug")
    finally:
        module.stop()


def test_identify_object_skill_unknown_id() -> None:
    module = _module_with_fake_vlm({"label": "mug"})
    try:
        assert "No object found" in module.identify_object("nope")
    finally:
        module.stop()


def test_refine_observed_labels_only_targets_ambiguous() -> None:
    module = _module_with_fake_vlm({"label": "coffee mug", "description": "", "confidence": 0.8})
    try:
        ambiguous = _make_object("vase", Vector3(0.0, 0.0, 0.0), confidence=0.3, track_id=1)
        confident = _make_object("keyboard", Vector3(2.0, 0.0, 0.0), confidence=0.95, track_id=2)
        for obj in module._object_db.add_objects([ambiguous, confident]):
            module._object_db.promote(obj.object_id)

        result = module.refine_observed_labels()

        assert "Relabeled 1 object(s):" in result
        assert "vase -> coffee mug" in result
        assert "keyboard" not in result
    finally:
        module.stop()


def test_refine_observed_labels_nothing_to_do() -> None:
    module = _module_with_fake_vlm({"label": "mug"})
    try:
        assert module.refine_observed_labels() == "No ambiguous objects to relabel."
    finally:
        module.stop()


def test_catalog_scene_refines_then_lists() -> None:
    """catalog_scene is the one-call find-and-catalog entry point: refine
    ambiguous labels with the VLM, then return the full located catalog."""
    module = _module_with_fake_vlm(
        {"label": "coffee mug", "description": "A white ceramic mug.", "confidence": 0.85}
    )
    try:
        ambiguous = _make_object("vase", Vector3(1.0, 2.0, 3.0), confidence=0.3, track_id=1)
        confident = _make_object("keyboard", Vector3(2.0, 0.0, 0.0), confidence=0.95, track_id=2)
        for obj in module._object_db.add_objects([ambiguous, confident]):
            module._object_db.promote(obj.object_id)

        result = module.catalog_scene()

        assert "Observing 2 item(s):" in result
        assert "coffee mug" in result and "A white ceramic mug." in result
        assert "keyboard" in result  # confident items still appear, just unrefined
        assert "pos=(1.00, 2.00, 3.00)m" in result
    finally:
        module.stop()


def test_catalog_scene_empty_scene() -> None:
    module = _module_with_fake_vlm({"label": "mug"})
    try:
        assert module.catalog_scene() == "No objects observed yet."
    finally:
        module.stop()
