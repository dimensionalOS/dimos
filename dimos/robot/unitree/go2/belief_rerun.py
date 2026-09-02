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

"""The go2's own body and camera, drawn around a replayed belief store.

A camera frustum and a robot-shaped box come from this machine's intrinsics and
this machine's dimensions, so they live with the robot rather than in
``dimos/experimental/memory_belief``. What they compose onto is
:func:`~dimos.experimental.memory_belief.rerun_config.belief_rerun_config`, which
carries everything true of any robot with a belief store.

Both entries are ``static``: they are facts about the hardware, not
observations. That is exactly why nothing derived from what the robot *saw* is
static -- a sighting that appears before the robot drove past it is a lie the
viewer tells for free.

The result has the same shape the live bridge's ``rerun_config`` does, so
replaying a recording looks like watching it being made.
"""

from __future__ import annotations

from typing import Any

from dimos.experimental.memory_belief.rerun_config import belief_rerun_config


def camera_frustum(rr: Any) -> Any:
    """The camera's intrinsics, drawn as a frustum around the live image.

    Logged at the image's own path rather than a path of its own: rerun draws
    the frustum wherever the ``Pinhole`` is and hangs that path's image on its
    plane, so splitting them puts the picture at the world origin.

    ``parent_frame`` belongs on the pinhole rather than being left to the
    image's own ``frame_id``: a pinhole implicitly parents its path to the view
    root, so letting both act gives the frame two parents and rerun drops one.
    Declaring it here is also what the live go2 config does.
    """
    from dimos.robot.unitree.go2.connection import GO2Connection

    info = GO2Connection.camera_info_static
    return [
        rr.Pinhole(
            focal_length=[info.K[0], info.K[4]],
            principal_point=[info.K[2], info.K[5]],
            width=info.width,
            height=info.height,
            image_plane_distance=0.6,
            parent_frame="tf#/camera_optical",
        )
    ]


def robot_body(rr: Any) -> list[Any]:
    """A go2-shaped box on base_link, so the trajectory has something driving it."""
    return [
        rr.Boxes3D(half_sizes=[0.35, 0.155, 0.2], colors=[(0, 255, 127)]),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def go2_belief_rerun_config() -> dict[str, Any]:
    """The belief view, with the go2's body and camera added."""
    return belief_rerun_config(static={"color_image": camera_frustum, "robot_body": robot_body})
