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

"""How a belief store should be drawn, for any robot that has one.

``dimos mem rerun`` is deliberately robot-agnostic: it walks streams and logs
whatever converts. That leaves two things it cannot know, and both are here
rather than in the record types, so ``import rerun`` stays out of them.

**Accumulation.** A viewer that shows only the newest record of each stream
shows a room one object at a time. Entities log per ``entity_id`` (in
``entity.py``) so they coexist; lidar logs per time chunk so the map builds up
instead of replacing itself.

**Where a detection belongs.** A frame's boxes go under the image they were
drawn on, not at the world origin.

Nothing here is static. Static means "true before it was seen", which is right
for a robot's own dimensions and wrong for everything this layer records.
Robot-shaped additions -- a camera frustum, a body box -- compose on top of
:func:`belief_rerun_config` from the robot's own package; see
``dimos/robot/unitree/go2/belief_rerun.py``.
"""

from __future__ import annotations

from typing import Any

#: Seconds of lidar per accumulated path. Every scan on its own path would
#: accumulate too, at thousands of paths; one path per chunk keeps the entity
#: panel usable while still revealing the map progressively as the robot drives.
MAP_CHUNK_S = 10.0


def map_chunk(cloud: Any) -> Any:
    """Send each scan to a path named for its time bucket.

    Keyed by the scan's own timestamp rather than a running counter, so the same
    recording always chunks the same way and the discovery pass that converts
    one observation to test it does not shift every boundary by one.
    """
    return [(f"lidar/chunk_{int(cloud.ts / MAP_CHUNK_S):05d}", cloud.to_rerun())]


def frame_boxes(annotation: Any) -> Any:
    """Put the frame's detection boxes under the image, where they were drawn."""
    return [("color_image/detections", annotation.to_rerun())]


def split_blueprint() -> Any:
    """Camera left, world right -- the split a robot's own viewer uses."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="color_image", name="Camera + detections"),
            rrb.Spatial3DView(
                origin="/",
                name="World",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.0)),
            ),
            column_shares=[1, 2],
        ),
        rrb.SelectionPanel(state="hidden"),
    )


def belief_rerun_config(*, static: dict[str, Any] | None = None) -> dict[str, Any]:
    """The robot-agnostic half, with room for a robot to add its own statics.

    Returns a fresh dict each call rather than a module-level constant: callers
    merge their own entries into it, and a shared dict would carry one robot's
    frustum into the next one's view.
    """
    return {
        "blueprint": split_blueprint,
        "tf_axes": 0.3,
        "visual_override": {
            "lidar": map_chunk,
            "belief_frame_annotation": frame_boxes,
        },
        "static": dict(static or {}),
    }
