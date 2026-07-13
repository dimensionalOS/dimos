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

"""Wishlist item 5 — contextual labeling / educated guesses.

Headless, stub VLM. Checks that the VLM prompt carries neighbor + size context,
that refine_observed_labels() only touches ambiguous items, and writes a
before/after label table for review. See manifest.yaml (05-contextual-labeling).
"""

from __future__ import annotations

from typing import Any

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.contextual_labeling import build_context_prompt, is_ambiguous
from dimos.perception.detection.type.detection3d.object import Object
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule

SKILL_ID = "05-contextual-labeling"


class _FakeVlModel:
    """Stands in for a VlModel: returns a canned refined label, records prompts."""

    def __init__(self, response: Any) -> None:
        self.response = response
        self.queries: list[str] = []

    def query_json(self, image: Any, query: str) -> Any:
        self.queries.append(query)
        return self.response

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


def _obj(name: str, center: Vector3, confidence: float, track_id: int, size: Vector3) -> Object:
    image = Image.from_numpy(
        np.zeros((8, 8, 3), dtype=np.uint8), format=ImageFormat.BGR, frame_id="camera_optical", ts=1.0
    )
    pointcloud = PointCloud2.from_numpy(
        np.array([[center.x, center.y, center.z]], dtype=float), frame_id="world", timestamp=1.0
    )
    return Object(
        bbox=(0.0, 0.0, 2.0, 2.0),
        track_id=track_id,
        class_id=0,
        confidence=confidence,
        name=name,
        ts=1.0,
        image=image,
        frame_id="world",
        center=center,
        size=size,
        pose=PoseStamped(
            ts=1.0, frame_id="world", position=center, orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
        ),
        pointcloud=pointcloud,
        detections_count=8,
    )


def test_prompt_carries_neighbor_and_size_context() -> None:
    """The VLM prompt for an ambiguous item names its detector guess, size, neighbors."""
    mouse = _obj("mouse", Vector3(0.0, 0.0, 0.8), 0.3, 1, Vector3(0.10, 0.06, 0.03))
    keyboard = _obj("keyboard", Vector3(0.3, 0.0, 0.8), 0.9, 2, Vector3(0.45, 0.15, 0.03))
    monitor = _obj("monitor", Vector3(0.1, -0.4, 1.1), 0.9, 3, Vector3(0.6, 0.4, 0.05))

    assert is_ambiguous(mouse)  # low confidence -> worth refining
    assert not is_ambiguous(keyboard)  # confident + specific -> leave alone

    prompt = build_context_prompt(mouse, [mouse, keyboard, monitor])
    assert "mouse" in prompt  # detector's own guess
    assert "keyboard" in prompt and "monitor" in prompt  # neighbor context
    assert "0.10m x 0.06m x 0.03m" in prompt  # measured real-world size


def test_refine_only_relabels_ambiguous_and_report(skill_output) -> None:
    """refine_observed_labels() refines the ambiguous item, leaves confident ones."""
    module = ObjectSceneRegistrationModule(localization="lidar", min_detections_for_permanent=1)
    module._vl_model = _FakeVlModel(
        {"label": "computer mouse", "description": "A small wired computer mouse.", "confidence": 0.82}
    )
    try:
        mouse = _obj("mouse", Vector3(0.0, 0.0, 0.8), 0.3, 1, Vector3(0.10, 0.06, 0.03))
        keyboard = _obj("keyboard", Vector3(0.3, 0.0, 0.8), 0.95, 2, Vector3(0.45, 0.15, 0.03))
        for obj in module._object_db.add_objects([mouse, keyboard]):
            module._object_db.promote(obj.object_id)

        before = {o.name: o.display_name for o in module._object_db.get_all_objects()}

        result = module.refine_observed_labels()

        after = {}
        for o in module._object_db.get_all_objects():
            # display_name prefers the refined label when present.
            after[o.name if o.name in before else o.display_name] = o.display_name

        # --- machine-checkable invariants ---------------------------------
        assert "Relabeled 1 object(s):" in result
        assert "mouse -> computer mouse" in result
        assert "keyboard" not in result  # confident item untouched
        assert module._object_db.find_by_name("computer mouse")

        # --- review artifact: before/after table --------------------------
        lines = [
            "Contextual labeling — refine_observed_labels() before/after",
            "=" * 58,
            "",
            "detector label        ->  reported label (display_name)",
            "-" * 58,
        ]
        for det_name, disp in sorted(before.items()):
            new_disp = next(
                (o.display_name for o in module._object_db.get_all_objects() if o.name == det_name),
                disp,
            )
            arrow = "  (refined)" if new_disp != det_name else ""
            lines.append(f"{det_name:<20}  ->  {new_disp}{arrow}")
        lines += [
            "",
            "VLM prompt sent for the ambiguous item:",
            "-" * 58,
            module._vl_model.queries[0] if module._vl_model.queries else "(none)",
            "",
        ]
        skill_output.path("labels_before_after.txt").write_text("\n".join(lines))
        skill_output.produced("labels_before_after.txt")
    finally:
        module.stop()
