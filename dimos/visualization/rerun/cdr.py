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

"""Viewer adapters live outside the generated plain value classes."""

from __future__ import annotations

from dataclasses import asdict
import json
from typing import TYPE_CHECKING, Any

from dimos.messages.geometry_msgs.msg import PointStamped, PoseStamped

if TYPE_CHECKING:
    from dimos.visualization.rerun.bridge import RerunMulti


def generated_archetypes(message: Any, entity: str) -> RerunMulti | None:
    """Render stamped geometry and its exact fields; other domains remain explicit."""
    import rerun as rr

    if type(message) is PoseStamped:
        point, rotation = message.pose.position, message.pose.orientation
        entries: RerunMulti = [
            (
                entity,
                rr.Transform3D(
                    translation=[point.x, point.y, point.z],
                    quaternion=rr.Quaternion(xyzw=[rotation.x, rotation.y, rotation.z, rotation.w]),
                    parent_frame=f"tf#/{message.header.frame_id}"
                    if message.header.frame_id
                    else None,
                ),
            ),
            (entity, rr.TransformAxes3D(0.35)),
        ]
    elif type(message) is PointStamped:
        point = message.point
        entries = [(entity, rr.Points3D([[point.x, point.y, point.z]]))]
        if message.header.frame_id:
            entries.append((entity, rr.Transform3D(parent_frame=f"tf#/{message.header.frame_id}")))
    else:
        return None
    fields = asdict(
        message,
        dict_factory=lambda items: {key: value for key, value in items if key != "__msgtype__"},
    )
    entries.append((entity + "/message", rr.TextDocument(json.dumps(fields, indent=2))))
    return entries
