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

from typing import Annotated, Literal, TypeAlias

from pydantic import BaseModel, ConfigDict, Field, StringConstraints

BodyTrackingMode: TypeAlias = Literal["off", "optional", "required"]
_FiniteFloat: TypeAlias = Annotated[float, Field(strict=True, allow_inf_nan=False)]
_NonEmptyString: TypeAlias = Annotated[
    str,
    StringConstraints(min_length=1, pattern=r".*\S.*"),
]


class BodyJointPose(BaseModel):
    """One body joint's pose in the snapshot's WebXR reference space."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    position: tuple[_FiniteFloat, _FiniteFloat, _FiniteFloat]
    orientation: tuple[_FiniteFloat, _FiniteFloat, _FiniteFloat, _FiniteFloat]


class BodyTrackingSnapshot(BaseModel):
    """Named body-joint poses captured in one WebXR reference space.

    ``joints=None`` means the body source is unavailable. An empty mapping
    means the source is available but did not resolve any joints.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    type: Literal["body_tracking_snapshot"]
    capture_time_s: _FiniteFloat
    frame_id: _NonEmptyString
    joints: dict[_NonEmptyString, BodyJointPose] | None
