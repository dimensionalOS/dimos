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

"""Rerun presentation helpers kept separate from generated wire types."""

from dimos_generated.tf2_msgs.msg import TFMessage
import matplotlib
import numpy as np
import rerun as rr


def tf_archetypes(message: TFMessage) -> list[tuple[str, rr.Transform3D]]:
    """Render named TF edges without embedding viewer behavior in messages."""
    result = []
    for edge in message.transforms:
        t, q = edge.transform.translation, edge.transform.rotation
        result.append(
            (
                f"tf_links/{rr.escape_entity_path_part(edge.child_frame_id)}",
                rr.Transform3D(
                    translation=[t.x, t.y, t.z],
                    quaternion=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
                    parent_frame="tf#/" + edge.header.frame_id,
                    child_frame="tf#/" + edge.child_frame_id,
                ),
            )
        )
    return result


def register_colormap_annotation(name: str = "turbo") -> None:
    """Register 256 class colors for clouds carrying colormap indices."""
    colors = (matplotlib.colormaps[name](np.linspace(0, 1, 256))[:, :3] * 255).astype(np.uint8)
    rr.log(
        "/",
        rr.AnnotationContext(
            [
                rr.datatypes.ClassDescription(
                    info=rr.datatypes.AnnotationInfo(id=i, color=color.tolist())
                )
                for i, color in enumerate(colors)
            ]
        ),
        static=True,
    )
