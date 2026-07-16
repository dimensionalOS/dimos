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

"""Tests for dynamic robot-relationship video snapshots."""

from dimos.benchmark.spatiotemporal.models import BoundingBox2D, SpatialPredicate
from dimos.benchmark.spatiotemporal.ports import DetectedObject
from dimos.benchmark.spatiotemporal.relationship_video import derive_relationship_snapshot


def _detection(
    object_id: str,
    label: str,
    box: tuple[float, float, float, float],
    confidence: float,
) -> DetectedObject:
    return DetectedObject(
        object_id=object_id,
        label=label,
        box=BoundingBox2D(
            x_min=box[0],
            y_min=box[1],
            x_max=box[2],
            y_max=box[3],
        ),
        confidence=confidence,
    )


def test_relationship_snapshot_contract_is_deterministic_and_explicit() -> None:
    detections = (
        _detection("robot-low", "quadruped robot", (0.0, 0.0, 0.2, 0.2), 0.5),
        _detection("z-table", "table", (0.1, 0.6, 0.5, 0.9), 0.8),
        _detection("robot-high", "quadruped robot", (0.4, 0.4, 0.6, 0.6), 0.9),
        _detection("a-lamp", "lamp", (0.7, 0.1, 0.9, 0.3), 0.7),
    )

    snapshot = derive_relationship_snapshot(detections, robot_label="quadruped robot")

    assert snapshot.robot == detections[2]
    assert snapshot.message is None
    assert [relationship.object.object_id for relationship in snapshot.relationships] == [
        "a-lamp",
        "z-table",
    ]
    assert [
        (relationship.horizontal, relationship.vertical) for relationship in snapshot.relationships
    ] == [
        (SpatialPredicate.LEFT_OF, SpatialPredicate.BELOW),
        (SpatialPredicate.RIGHT_OF, SpatialPredicate.ABOVE),
    ]
    assert (
        derive_relationship_snapshot(detections[1:], robot_label="missing robot").message
        == "Robot not detected"
    )
    assert (
        derive_relationship_snapshot((detections[2],), robot_label="quadruped robot").message
        == "No other objects detected"
    )
