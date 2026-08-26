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

"""Body-joint snapshots received from a WebXR client."""

from dataclasses import dataclass
from typing import Annotated, Literal, TypeAlias

from pydantic import BaseModel, ConfigDict, Field, StringConstraints, ValidationError

from dimos.msgs.geometry_msgs.Pose import Pose

BODY_TRACKING_MESSAGE_TYPE = "body_tracking_snapshot"

BodyTrackingMode: TypeAlias = Literal["off", "optional", "required"]
_FiniteFloat: TypeAlias = Annotated[float, Field(strict=True, allow_inf_nan=False)]
_NonEmptyString: TypeAlias = Annotated[
    str,
    StringConstraints(min_length=1, pattern=r".*\S.*"),
]


@dataclass(frozen=True)
class BodyTrackingSnapshot:
    """Named body-joint poses captured in one WebXR reference space."""

    capture_time_s: float
    frame_id: str
    joints: dict[str, Pose] | None


class _WirePose(BaseModel):
    model_config = ConfigDict(extra="forbid")

    position: tuple[_FiniteFloat, _FiniteFloat, _FiniteFloat]
    orientation: tuple[_FiniteFloat, _FiniteFloat, _FiniteFloat, _FiniteFloat]


class _WireBodyTrackingSnapshot(BaseModel):
    model_config = ConfigDict(extra="forbid")

    type: Literal["body_tracking_snapshot"]
    capture_time_s: _FiniteFloat
    frame_id: _NonEmptyString
    joints: dict[_NonEmptyString, _WirePose] | None


def decode_body_tracking_snapshot(payload: str) -> BodyTrackingSnapshot:
    """Decode and validate one JSON body-tracking snapshot."""
    try:
        wire = _WireBodyTrackingSnapshot.model_validate_json(payload)
    except ValidationError as exc:
        raise ValueError("invalid body-tracking snapshot") from exc

    joints = None
    if wire.joints is not None:
        joints = {
            name: Pose(position=pose.position, orientation=pose.orientation)
            for name, pose in wire.joints.items()
        }

    return BodyTrackingSnapshot(
        capture_time_s=wire.capture_time_s,
        frame_id=wire.frame_id,
        joints=joints,
    )
