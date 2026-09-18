# Copyright 2026 Dimensional Inc.
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

"""Simulator-agnostic export of ground-truth boxes as ``Detection3DArray``.

:func:`top_down` flattens such an array to a metric ``Detection2DArray``: each
box's floor footprint as a rectangle in the same ``world`` frame, labels kept.

Each simulator measures world-axis-aligned boxes in its own frame (DimSim in
Three.js Y-up, Habitat in its Y-up world) and hands them here as
:class:`GroundTruthBox` values plus an axis converter into the DimOS Z-up
``world`` frame. The converters are axis permutations with sign flips, so an
axis-aligned box stays axis-aligned: both corners are converted and re-sorted
per axis, and the orientation stays the identity.

Consumers run over LCM. Publish the array like any other DimOS message, e.g.
``LCMTransport("/detections_3d", Detection3DArray).publish(detections)`` on the
channel ``/detections_3d#vision_msgs.Detection3DArray``; the file written by
:func:`write_detection3d_array` holds the identical LCM wire payload.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, cast

from dimos_lcm.vision_msgs import (
    BoundingBox2D,
    BoundingBox3D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseWithCovariance import PoseWithCovariance
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection2D import Detection2D
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Authored labels are ground truth, not classifier output.
GROUND_TRUTH_SCORE = 1.0
# A footprint spanning at least this fraction of the scene extent on both axes is a
# floor slab, roof or similar shell rather than an object: left out of the top-down view.
COVERING_FOOTPRINT_FRACTION = 0.9
TOP_DOWN_PROJECTION = "top_down_xy"

Point3 = tuple[float, float, float]
# Maps a point from the simulator's world frame to the DimOS Z-up world frame.
AxisConverter = Callable[[float, float, float], Point3]


@dataclass(frozen=True)
class GroundTruthBox:
    """Axis-aligned box in the simulator's native world frame.

    ``labels[0]`` becomes ``results[0].hypothesis.class_id``; further labels
    become further hypotheses in order, all scored :data:`GROUND_TRUTH_SCORE`.
    """

    id: str
    labels: tuple[str, ...]
    min: Point3
    max: Point3


def ros_box(lo: Point3, hi: Point3, to_ros: AxisConverter) -> tuple[Point3, Point3]:
    """Convert a box to the ROS frame, returning ``(center, size)``.

    Both corners go through ``to_ros`` and are re-sorted per axis, so a sign
    flip in the converter cannot produce a negative size.
    """
    (ax, ay, az), (bx, by, bz) = to_ros(*lo), to_ros(*hi)
    lo_r = (min(ax, bx), min(ay, by), min(az, bz))
    hi_r = (max(ax, bx), max(ay, by), max(az, bz))
    center = (
        (lo_r[0] + hi_r[0]) / 2.0,
        (lo_r[1] + hi_r[1]) / 2.0,
        (lo_r[2] + hi_r[2]) / 2.0,
    )
    size = (hi_r[0] - lo_r[0], hi_r[1] - lo_r[1], hi_r[2] - lo_r[2])
    return center, size


def boxes_to_detection3d_array(
    boxes: Sequence[GroundTruthBox], *, to_ros: AxisConverter, frame_id: str, ts: float
) -> Detection3DArray:
    """Build a ``Detection3DArray`` with one ``Detection3D`` per box in ``frame_id``.

    Raises ``ValueError`` on an invalid box rather than exporting a partial or
    placeholder result. Every nested message is freshly allocated so detections
    never alias each other or the generated classes' shared default instances.
    """
    detections = [_to_detection(box, to_ros, frame_id, ts) for box in boxes]
    return Detection3DArray(
        detections_length=len(detections),
        header=Header(ts, frame_id),
        detections=detections,
    )


def write_detection3d_array(detections: Detection3DArray, path: str | Path) -> Path:
    """Write one ``Detection3DArray`` as its LCM wire payload.

    The bytes are exactly what an LCM publisher sends for this type, so the
    file is one message (not an LCM event log) and decodes with
    :func:`read_detection3d_array` or ``Detection3DArray.lcm_decode``.
    """
    out = Path(path)
    out.write_bytes(detections.lcm_encode())
    return out


def read_detection3d_array(path: str | Path) -> Detection3DArray:
    """Read a file written by :func:`write_detection3d_array`."""
    return cast("Detection3DArray", Detection3DArray.lcm_decode(Path(path).read_bytes()))


def write_detection3d_json(
    detections: Detection3DArray,
    path: str | Path,
    *,
    provenance: Mapping[str, Any] | None = None,
) -> Path:
    """Write a readable JSON view of ``detections``, one entry per detection.

    For inspection and non-LCM tooling. The LCM payload written by
    :func:`write_detection3d_array` stays the typed, lossless format.
    ``provenance`` (e.g. dataset and scene id) is written ahead of the view.
    """
    out = Path(path)
    view = detection3d_array_to_dict(detections, provenance=provenance)
    out.write_text(json.dumps(view, indent=2) + "\n")
    return out


def detection3d_array_to_dict(
    detections: Detection3DArray, *, provenance: Mapping[str, Any] | None = None
) -> dict[str, Any]:
    """Plain-data view of a ``Detection3DArray``: label, center, size per detection."""
    view: dict[str, Any] = dict(provenance or {})
    view.update(
        {
            "frame_id": detections.frame_id,
            "timestamp": detections.ts,
            "count": detections.detections_length,
            "detections": [
                _detection_to_dict(d) for d in detections.detections[: detections.detections_length]
            ],
        }
    )
    return view


def top_down(
    detections: Detection3DArray,
    *,
    covering_fraction: float | None = COVERING_FOOTPRINT_FRACTION,
) -> Detection2DArray:
    """Project every box to its floor footprint as a ``Detection2D`` in the same frame.

    The boxes are world-axis-aligned, so the footprint is the rectangle with the
    same x/y center and size and ``theta = 0``; ids, labels, scores and the header
    carry over, the 3D center stays in each hypothesis pose, and only the height is
    dropped. Boxes whose footprint spans at least ``covering_fraction`` of the
    scene's extent on both axes (a floor slab, a roof) are left out and logged;
    pass ``None`` to keep them. Raises ``ValueError`` for an oriented box.
    """
    source = detections.detections[: detections.detections_length]
    dropped = set(covering_ids(source, covering_fraction))
    if dropped:
        logger.info("dropping scene-covering boxes from the top-down view", ids=sorted(dropped))
    projected = [_to_detection2d(d) for d in source if d.id not in dropped]
    return Detection2DArray(
        detections_length=len(projected),
        header=Header(detections.ts, detections.frame_id),
        detections=projected,
    )


def covering_ids(detections: Sequence[Detection3D], fraction: float | None) -> list[str]:
    """Ids of boxes spanning at least ``fraction`` of the joint XY extent on both axes."""
    if fraction is None or len(detections) < 2:
        return []
    lo_x = min(d.bbox.center.position.x - d.bbox.size.x / 2 for d in detections)
    hi_x = max(d.bbox.center.position.x + d.bbox.size.x / 2 for d in detections)
    lo_y = min(d.bbox.center.position.y - d.bbox.size.y / 2 for d in detections)
    hi_y = max(d.bbox.center.position.y + d.bbox.size.y / 2 for d in detections)
    extent_x, extent_y = hi_x - lo_x, hi_y - lo_y
    if extent_x <= 0 or extent_y <= 0:
        return []
    return [
        d.id
        for d in detections
        if d.bbox.size.x >= fraction * extent_x and d.bbox.size.y >= fraction * extent_y
    ]


def write_detection2d_json(
    detections: Detection2DArray,
    path: str | Path,
    *,
    provenance: Mapping[str, Any] | None = None,
) -> Path:
    """Write the JSON view of a top-down ``Detection2DArray`` (see :func:`top_down`)."""
    out = Path(path)
    view = detection2d_array_to_dict(detections, provenance=provenance)
    out.write_text(json.dumps(view, indent=2) + "\n")
    return out


def detection2d_array_to_dict(
    detections: Detection2DArray, *, provenance: Mapping[str, Any] | None = None
) -> dict[str, Any]:
    """Plain-data view of a top-down ``Detection2DArray``: metric XY rectangles with labels."""
    view: dict[str, Any] = dict(provenance or {})
    view.update(
        {
            "projection": TOP_DOWN_PROJECTION,
            "frame_id": str(detections.header.frame_id),
            "timestamp": detections.ts,
            "count": detections.detections_length,
            "detections": [
                _detection2d_to_dict(d)
                for d in detections.detections[: detections.detections_length]
            ],
        }
    )
    return view


def _to_detection2d(detection: Detection3D) -> Detection2D:
    q = detection.bbox.center.orientation
    if (q.x, q.y, q.z, q.w) != (0.0, 0.0, 0.0, 1.0):
        raise ValueError(f"box {detection.id!r} is oriented; only axis-aligned boxes project")
    center, size = detection.bbox.center.position, detection.bbox.size
    # Fresh copies throughout: nothing is shared with the 3D message.
    return Detection2D(
        results_length=detection.results_length,
        header=Header(detection.ts, detection.frame_id),
        results=[
            ObjectHypothesisWithPose(
                hypothesis=ObjectHypothesis(
                    class_id=result.hypothesis.class_id, score=result.hypothesis.score
                ),
                pose=PoseWithCovariance(Pose(Vector3(center.x, center.y, center.z), Quaternion())),
            )
            for result in detection.results[: detection.results_length]
        ],
        bbox=BoundingBox2D(
            center=Pose2D(position=Point2D(x=center.x, y=center.y), theta=0.0),
            size_x=size.x,
            size_y=size.y,
        ),
        id=detection.id,
    )


def _detection2d_to_dict(detection: Detection2D) -> dict[str, Any]:
    hypotheses = [r.hypothesis for r in detection.results[: detection.results_length]]
    first = hypotheses[0] if hypotheses else None
    center = detection.bbox.center
    entry: dict[str, Any] = {
        "id": detection.id,
        "label": first.class_id if first else "",
        "score": first.score if first else 0.0,
        "center_xy": [center.position.x, center.position.y],
        "size_xy": [detection.bbox.size_x, detection.bbox.size_y],
        "theta": center.theta,
    }
    if len(hypotheses) > 1:
        entry["labels"] = [h.class_id for h in hypotheses]
    return entry


def _detection_to_dict(detection: Detection3D) -> dict[str, Any]:
    hypotheses = [r.hypothesis for r in detection.results[: detection.results_length]]
    first = hypotheses[0] if hypotheses else None
    center, size = detection.bbox.center.position, detection.bbox.size
    q = detection.bbox.center.orientation
    entry: dict[str, Any] = {
        "id": detection.id,
        "label": first.class_id if first else "",
        "score": first.score if first else 0.0,
        "center_xyz": [center.x, center.y, center.z],
        "size_xyz": [size.x, size.y, size.z],
        "orientation_xyzw": [q.x, q.y, q.z, q.w],
    }
    if len(hypotheses) > 1:
        entry["labels"] = [h.class_id for h in hypotheses]
    return entry


def _to_detection(
    box: GroundTruthBox, to_ros: AxisConverter, frame_id: str, ts: float
) -> Detection3D:
    if not box.id:
        raise ValueError("box id must be a non-empty string")
    if not box.labels or not all(box.labels):
        raise ValueError(f"box {box.id!r} needs non-empty labels, got {box.labels!r}")
    for name, point in (("min", box.min), ("max", box.max)):
        if len(point) != 3 or not all(math.isfinite(v) for v in point):
            raise ValueError(f"box {box.id!r} {name} must be three finite numbers, got {point!r}")
    if any(h < l for l, h in zip(box.min, box.max, strict=True)):
        raise ValueError(f"box {box.id!r} has max {box.max} below min {box.min}")

    center, size = ros_box(box.min, box.max, to_ros)
    return Detection3D(
        results_length=len(box.labels),
        header=Header(ts, frame_id),
        results=[
            ObjectHypothesisWithPose(
                hypothesis=ObjectHypothesis(class_id=label, score=GROUND_TRUTH_SCORE),
                pose=PoseWithCovariance(_center_pose(center)),
            )
            for label in box.labels
        ],
        bbox=BoundingBox3D(center=_center_pose(center), size=Vector3(*size)),
        id=box.id,
    )


def _center_pose(center: Point3) -> Pose:
    return Pose(Vector3(*center), Quaternion())
