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

from __future__ import annotations

from dataclasses import dataclass
from typing import cast

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


@dataclass(slots=True)
class GraspCandidate:
    """A scored gripper pose candidate."""

    pose: Pose
    jaw_width: float
    score: float
    id: str | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "pose": {
                "position": [self.pose.position.x, self.pose.position.y, self.pose.position.z],
                "orientation": [
                    self.pose.orientation.x,
                    self.pose.orientation.y,
                    self.pose.orientation.z,
                    self.pose.orientation.w,
                ],
            },
            "jaw_width": self.jaw_width,
            "score": self.score,
            "id": self.id,
        }

    @classmethod
    def from_dict(cls, value: object) -> GraspCandidate:
        if not isinstance(value, dict):
            raise ValueError("GraspCandidate must decode from a JSON object")
        payload = cast("dict[str, object]", value)
        pose_payload = _as_dict(payload["pose"])
        position = _as_float_list(pose_payload["position"], expected_len=3)
        orientation = _as_float_list(pose_payload["orientation"], expected_len=4)
        id_value = payload.get("id")
        return cls(
            pose=_make_pose(position, orientation),
            jaw_width=_as_float(payload["jaw_width"]),
            score=_as_float(payload["score"]),
            id=str(id_value) if id_value is not None else None,
        )


def _make_pose(position: list[float], orientation: list[float]) -> Pose:
    pose = Pose()  # type: ignore[call-arg]
    pose.position = Vector3(position)
    pose.orientation = Quaternion(orientation)
    return pose


def _as_dict(value: object) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError("Expected JSON object")
    return cast("dict[str, object]", value)


def _as_float_list(value: object, *, expected_len: int) -> list[float]:
    if not isinstance(value, list) or len(value) != expected_len:
        raise ValueError(f"Expected a list of {expected_len} floats")
    return [float(item) for item in value]


def _as_float(value: object) -> float:
    if not isinstance(value, int | float | str):
        raise ValueError(f"Expected a float-compatible value, got {type(value).__name__}")
    return float(value)
