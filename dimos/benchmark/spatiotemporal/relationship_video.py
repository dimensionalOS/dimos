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

"""Robot-to-object image-plane snapshots and annotated video rendering."""

from collections.abc import Sequence
from dataclasses import dataclass
import math
from pathlib import Path

import cv2
import numpy as np

from dimos.benchmark.spatiotemporal.models import SpatialPredicate
from dimos.benchmark.spatiotemporal.ports import DetectedObject, ObservationDetector
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


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


def _box_pixels(detection: DetectedObject, width: int, height: int) -> tuple[int, int, int, int]:
    box = detection.box
    return (
        max(0, min(width - 1, round(box.x_min * width))),
        max(0, min(height - 1, round(box.y_min * height))),
        max(0, min(width - 1, round(box.x_max * width))),
        max(0, min(height - 1, round(box.y_max * height))),
    )


def _draw_snapshot(frame: np.ndarray, snapshot: RelationshipSnapshot) -> None:
    height, width = frame.shape[:2]
    detections = (() if snapshot.robot is None else (snapshot.robot,)) + tuple(
        relationship.object for relationship in snapshot.relationships
    )
    for detection in detections:
        color = (80, 220, 80) if detection is snapshot.robot else (0, 170, 255)
        x_min, y_min, x_max, y_max = _box_pixels(detection, width, height)
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), color, 2)
        cv2.putText(
            frame,
            f"{detection.label} {detection.confidence:.2f}",
            (x_min, max(12, y_min - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            color,
            1,
            cv2.LINE_AA,
        )

    lines = (
        [snapshot.message]
        if snapshot.message is not None
        else [
            f"robot {relationship.horizontal.value} / {relationship.vertical.value} "
            f"{relationship.object.label} [{relationship.object.object_id}]"
            for relationship in snapshot.relationships
        ]
    )
    panel_height = min(height, 12 + 18 * len(lines))
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (width, panel_height), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.65, frame, 0.35, 0.0, frame)
    for index, line in enumerate(lines):
        cv2.putText(
            frame,
            line,
            (6, 17 + 18 * index),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )


def _verify_decoded_frame_count(video_path: Path, expected_frame_count: int) -> None:
    capture = cv2.VideoCapture(str(video_path))
    decoded_frame_count = 0
    try:
        while True:
            decoded, _ = capture.read()
            if not decoded:
                break
            decoded_frame_count += 1
    finally:
        capture.release()
    if decoded_frame_count != expected_frame_count:
        raise RuntimeError(
            f"encoded frame count mismatch: expected {expected_frame_count}, "
            f"decoded {decoded_frame_count}"
        )


def render_relationship_video(
    source_path: Path,
    output_path: Path,
    detector: ObservationDetector,
    robot_label: str = "quadruped robot",
    update_period_s: float = 1.0,
) -> None:
    """Render full-rate overlays while refreshing detections at a fixed period."""
    capture = cv2.VideoCapture(str(source_path))
    writer: cv2.VideoWriter | None = None
    frame_count = 0
    try:
        fps = capture.get(cv2.CAP_PROP_FPS)
        if not math.isfinite(fps) or fps <= 0.0:
            raise ValueError("video FPS must be finite and positive")
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        writer = cv2.VideoWriter(
            str(output_path),
            cv2.VideoWriter_fourcc(*"mp4v"),  # type: ignore[attr-defined]
            fps,
            (width, height),
        )
        next_refresh_s = 0.0
        snapshot: RelationshipSnapshot | None = None
        while True:
            decoded, frame = capture.read()
            if not decoded:
                break
            timestamp_s = frame_count / fps
            if timestamp_s + 1e-9 >= next_refresh_s:
                image = Image.from_numpy(
                    cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
                    format=ImageFormat.RGB,
                    frame_id=str(frame_count),
                    ts=timestamp_s,
                )
                snapshot = derive_relationship_snapshot(detector.detect(image), robot_label)
                next_refresh_s += update_period_s
            if snapshot is None:
                raise RuntimeError("first video frame did not produce an overlay snapshot")
            _draw_snapshot(frame, snapshot)
            writer.write(frame)
            frame_count += 1
    finally:
        try:
            capture.release()
        finally:
            try:
                if writer is not None:
                    writer.release()
            finally:
                detector.close()
    _verify_decoded_frame_count(output_path, frame_count)
