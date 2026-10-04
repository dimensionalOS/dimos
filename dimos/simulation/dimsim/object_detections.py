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

"""Typed export of DimSim object annotations as ``Detection3DArray``.

The browser measures world-axis-aligned bounds of identified scene assets and of
walls baked into the scene structure, in Three.js Y-up coordinates
(``misc/DimSim/src/objectAnnotations.js``). This module
turns that JSON snapshot into the repository's 3D detection types in the DimOS
Z-up ``world`` frame, the frame DimSim already publishes odometry in.

Consumers run over LCM. Publish the array like any other DimOS message, e.g.
``LCMTransport("/detections_3d", Detection3DArray).publish(detections)`` on the
channel ``/detections_3d#vision_msgs.Detection3DArray``; the file written by
:func:`write_detection3d_array` holds the identical LCM wire payload.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, TypeGuard, cast

from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseWithCovariance import PoseWithCovariance
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray

# DimSim odometry uses this frame (cli/bridge/physics.ts); annotations share it.
DIMSIM_WORLD_FRAME = "world"
# Authored labels are ground truth, not classifier output.
GROUND_TRUTH_SCORE = 1.0

Point3 = tuple[float, float, float]


def threejs_to_ros(x: float, y: float, z: float) -> Point3:
    """Three.js Y-up world to ROS Z-up world, the permutation used for odometry."""
    return (z, x, y)


def snapshot_to_detection3d_array(snapshot: dict[str, Any]) -> Detection3DArray:
    """Convert a ``getObjectAnnotations()`` snapshot into a ``Detection3DArray``.

    Each identified asset becomes one ``Detection3D``: ``id`` is the stable asset
    ID and the single hypothesis carries the authored label. Boxes are already
    world-axis-aligned, so orientation stays the identity and only the axes are
    permuted, for the size as well as the center.

    Raises ``ValueError`` on a malformed snapshot rather than exporting a
    partial or placeholder result.
    """
    ts = _captured_at_seconds(snapshot)
    detections = [_to_detection(obj, ts) for obj in _objects(snapshot)]
    # Fresh header and list: the generated constructors share mutable defaults.
    return Detection3DArray(
        detections_length=len(detections),
        header=Header(ts, DIMSIM_WORLD_FRAME),
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


def write_detection3d_json(detections: Detection3DArray, path: str | Path) -> Path:
    """Write a readable JSON view of ``detections``, one entry per detection.

    For inspection and non-LCM tooling. The LCM payload written by
    :func:`write_detection3d_array` stays the typed, lossless format.
    """
    out = Path(path)
    out.write_text(json.dumps(detection3d_array_to_dict(detections), indent=2) + "\n")
    return out


def detection3d_array_to_dict(detections: Detection3DArray) -> dict[str, Any]:
    """Plain-data view of a ``Detection3DArray``: label, center, size per detection."""
    return {
        "frame_id": detections.frame_id,
        "timestamp": detections.ts,
        "count": detections.detections_length,
        "detections": [
            _detection_to_dict(d) for d in detections.detections[: detections.detections_length]
        ],
    }


def _detection_to_dict(detection: Detection3D) -> dict[str, Any]:
    hypothesis = detection.results[0].hypothesis if detection.results_length else None
    center, size = detection.bbox.center.position, detection.bbox.size
    q = detection.bbox.center.orientation
    return {
        "id": detection.id,
        "label": hypothesis.class_id if hypothesis else "",
        "score": hypothesis.score if hypothesis else 0.0,
        "center_xyz": [center.x, center.y, center.z],
        "size_xyz": [size.x, size.y, size.z],
        "orientation_xyzw": [q.x, q.y, q.z, q.w],
    }


def _captured_at_seconds(snapshot: dict[str, Any]) -> float:
    captured_at = snapshot.get("capturedAt")
    if not _is_finite_number(captured_at):
        raise ValueError(
            f"snapshot capturedAt must be finite epoch milliseconds, got {captured_at!r}"
        )
    return captured_at / 1000.0


def _objects(snapshot: dict[str, Any]) -> list[dict[str, Any]]:
    objects = snapshot.get("objects")
    if not isinstance(objects, list):
        raise ValueError(f"snapshot objects must be a list, got {type(objects).__name__}")
    return objects


def _to_detection(obj: dict[str, Any], ts: float) -> Detection3D:
    obj_id = obj.get("id")
    if not isinstance(obj_id, str) or not obj_id:
        raise ValueError(f"object id must be a non-empty string, got {obj_id!r}")
    label = obj.get("label")
    if not isinstance(label, str) or not label:
        raise ValueError(f"object {obj_id!r} label must be a non-empty string, got {label!r}")
    lo = _point(obj.get("min"), obj_id, "min")
    hi = _point(obj.get("max"), obj_id, "max")
    if any(h < l for l, h in zip(lo, hi, strict=True)):
        raise ValueError(f"object {obj_id!r} has max {hi} below min {lo}")

    center = threejs_to_ros(*((l + h) / 2.0 for l, h in zip(lo, hi, strict=True)))
    size = threejs_to_ros(*(h - l for l, h in zip(lo, hi, strict=True)))

    # Every nested message is allocated here so detections never alias each
    # other or the generated classes' shared default instances.
    return Detection3D(
        results_length=1,
        header=Header(ts, DIMSIM_WORLD_FRAME),
        results=[
            ObjectHypothesisWithPose(
                hypothesis=ObjectHypothesis(class_id=label, score=GROUND_TRUTH_SCORE),
                pose=PoseWithCovariance(_center_pose(center)),
            )
        ],
        bbox=BoundingBox3D(center=_center_pose(center), size=Vector3(*size)),
        id=obj_id,
    )


def _center_pose(center: Point3) -> Pose:
    return Pose(Vector3(*center), Quaternion())


def _point(value: Any, obj_id: str, name: str) -> Point3:
    if (
        not isinstance(value, list | tuple)
        or len(value) != 3
        or not all(map(_is_finite_number, value))
    ):
        raise ValueError(f"object {obj_id!r} {name} must be three finite numbers, got {value!r}")
    return (float(value[0]), float(value[1]), float(value[2]))


def _is_finite_number(value: Any) -> TypeGuard[float]:
    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(value)
