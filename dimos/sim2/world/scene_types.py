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

"""The scene sidecar: names for bodies, fixture joints, regions and spawns, independent of MuJoCo indices."""

from __future__ import annotations

import math
from typing import Annotated, Any, Literal

from pydantic import BaseModel, BeforeValidator, ConfigDict, Field, FiniteFloat, PlainSerializer

from dimos.msgs.geometry_msgs.Pose import Pose

Vec3 = tuple[FiniteFloat, FiniteFloat, FiniteFloat]


def _pose(value: Any) -> Pose:
    pose = Pose(value)
    xyz = pose.position.to_tuple()
    xyzw = pose.orientation.to_tuple()
    if not all(math.isfinite(v) for v in (*xyz, *xyzw)):
        raise ValueError("pose must be finite")
    norm = math.sqrt(sum(v * v for v in xyzw))
    if abs(norm - 1) > 1e-3:
        raise ValueError("pose orientation must be a unit xyzw quaternion")
    return Pose(xyz, tuple(v / norm for v in xyzw))


PoseValue = Annotated[
    Pose,
    BeforeValidator(_pose),
    PlainSerializer(
        lambda p: {"position": p.position.to_tuple(), "orientation": p.orientation.to_tuple()},
        return_type=dict,
    ),
]


class SceneRecord(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid", arbitrary_types_allowed=True)


class SceneUpdate(SceneRecord):
    """World-frame poses of existing free/mocap bodies and named fixture joints.

    Robot IDs are allowed in poses; robot joints stay on the ordinary control
    interface. Omitted entities are unchanged for edits and baseline for reset.
    """

    poses: dict[str, PoseValue] = Field(default_factory=dict)
    joints: dict[str, FiniteFloat] = Field(default_factory=dict)


class SceneEntity(SceneRecord):
    body: str
    label: str
    kind: str
    movable: bool = False


class SceneJoint(SceneRecord):
    joint: str
    entity: str
    closed: FiniteFloat
    opened: FiniteFloat


class SceneRegion(SceneRecord):
    """An oriented volume attached to a model body, or the world frame."""

    body: str = "world"
    kind: Literal["support", "containment", "navigation"]
    pose: PoseValue = Field(default_factory=Pose)
    size: Vec3


class SceneDescription(SceneRecord):
    format: Literal["dimos.scene.v1"] = "dimos.scene.v1"
    id: str
    entities: dict[str, SceneEntity] = Field(default_factory=dict)
    joints: dict[str, SceneJoint] = Field(default_factory=dict)
    regions: dict[str, SceneRegion] = Field(default_factory=dict)
    initial: SceneUpdate = Field(default_factory=SceneUpdate)
    # Named support poses, independent of robot root height and identity.
    spawns: dict[str, PoseValue] = Field(default_factory=dict)
    hidden_geom_groups: tuple[int, ...] = ()
    provenance: dict[str, Any] = Field(default_factory=dict)
