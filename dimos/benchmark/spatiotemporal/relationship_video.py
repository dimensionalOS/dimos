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

"""Pure robot-to-object image-plane relationship snapshots."""

from collections.abc import Sequence
from dataclasses import dataclass

from dimos.benchmark.spatiotemporal.models import SpatialPredicate
from dimos.benchmark.spatiotemporal.ports import DetectedObject


@dataclass(frozen=True)
class RobotRelationship:
    """Horizontal and vertical image-plane relations from one robot to one object."""

    object: DetectedObject
    horizontal: SpatialPredicate
    vertical: SpatialPredicate


@dataclass(frozen=True)
class RelationshipSnapshot:
    """One immutable robot-relationship overlay state."""

    robot: DetectedObject | None
    relationships: tuple[RobotRelationship, ...]
    message: str | None


def _box_center(detection: DetectedObject) -> tuple[float, float]:
    box = detection.box
    return ((box.x_min + box.x_max) / 2.0, (box.y_min + box.y_max) / 2.0)


def derive_relationship_snapshot(
    detections: Sequence[DetectedObject],
    robot_label: str = "quadruped robot",
) -> RelationshipSnapshot:
    """Select the configured robot and derive deterministic 2D relations."""
    robots = sorted(
        (detection for detection in detections if detection.label == robot_label),
        key=lambda detection: (-detection.confidence, detection.object_id, detection.label),
    )
    if not robots:
        return RelationshipSnapshot(robot=None, relationships=(), message="Robot not detected")

    robot = robots[0]
    other_objects = sorted(
        (detection for detection in detections if detection.label != robot_label),
        key=lambda detection: (detection.object_id, detection.label),
    )
    if not other_objects:
        return RelationshipSnapshot(
            robot=robot,
            relationships=(),
            message="No other objects detected",
        )

    robot_x, robot_y = _box_center(robot)
    relationships = tuple(
        RobotRelationship(
            object=detection,
            horizontal=(
                SpatialPredicate.LEFT_OF
                if robot_x <= _box_center(detection)[0]
                else SpatialPredicate.RIGHT_OF
            ),
            vertical=(
                SpatialPredicate.ABOVE
                if robot_y <= _box_center(detection)[1]
                else SpatialPredicate.BELOW
            ),
        )
        for detection in other_objects
    )
    return RelationshipSnapshot(robot=robot, relationships=relationships, message=None)
