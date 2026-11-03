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

"""Where a hand-eye calibration result lives on the machine, and how a blueprint reads it back.

One JSON file per camera under ``STATE_DIR/calibration``, named by the calibration's ``name``.
It holds the transform a blueprint mounts the camera with (parent link to the camera's
``camera_link``) and, for the record, the solved optical frame and the solver's confidence.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from dimos.constants import STATE_DIR
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3

HAND_EYE_DIR = STATE_DIR / "calibration"


def hand_eye_path(name: str) -> Path:
    return HAND_EYE_DIR / f"{name}.json"


def _transform_to_json(t: Transform) -> dict[str, Any]:
    return {
        "frame_id": t.frame_id,
        "child_frame_id": t.child_frame_id,
        "translation_m": [t.translation.x, t.translation.y, t.translation.z],
        "rotation_xyzw": [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w],
    }


def _transform_from_json(d: dict[str, Any]) -> Transform:
    x, y, z = d["translation_m"]
    qx, qy, qz, qw = d["rotation_xyzw"]
    return Transform(
        translation=Vector3(x, y, z),
        rotation=Quaternion(qx, qy, qz, qw),
        frame_id=d["frame_id"],
        child_frame_id=d["child_frame_id"],
    )


def save_hand_eye(
    name: str,
    mount: Transform,
    optical: Transform,
    confidence: dict[str, float],
    passed: bool,
) -> Path:
    """Write the result; returns the file written."""
    path = hand_eye_path(name)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                "name": name,
                "passed": passed,
                "mount": _transform_to_json(mount),
                "optical": _transform_to_json(optical),
                "confidence": confidence,
            },
            indent=2,
        )
    )
    return path


def load_hand_eye(name: str) -> Transform | None:
    """The mount transform of a calibration that passed, or None when there is none."""
    path = hand_eye_path(name)
    if not path.exists():
        return None
    data = json.loads(path.read_text())
    if not data.get("passed"):
        return None
    return _transform_from_json(data["mount"])
