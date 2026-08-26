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

"""What the plan looks like on the robot: the body poses it expects to occupy.

The local plan already draws as a line, and a line cannot show the thing that
decides whether a plan fits — the BODY at each pose, at its planned heading.
This draws the body over the live stack, so a plan that looks fine as a line
and is actually threading the robot's corner through a wall looks wrong.

Colour is the required-precision profile, and it costs no extra channel: the
planner stamps that profile into the path's own per-waypoint timestamps
(``control/profile.py``), so :func:`render_plan_body` decodes it back out of
the message it is already being handed. Green = room to spare, amber = inside
the governor's ramp, red = at or under the embodiment's precision floor, which
is where a few centimetres of tracking error becomes contact.

Paired with :class:`~dimos.navigation.motion.adapter.planner.MotionPlanner`'s
``viz_publish_hz`` the same way the MLS planner's overrides are — one number in
the blueprint drives both the publishing and the drawing, so they cannot drift.
"""

from __future__ import annotations

from functools import partial
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.motion.control.profile import (
    ceilings_to_clearance,
    decode_ceilings,
)
from dimos.navigation.motion.embodiment import EMBODIMENTS, GO2, Embodiment

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype

# Lift the box off the floor so it does not z-fight the surface points.
_Z_LIFT = 0.02


def _body_centre(pose: PoseStamped, center_off: float) -> tuple[float, float]:
    """The body centre for a pose point, offset along the pose's own heading."""
    yaw = pose.yaw
    return (
        pose.position.x + center_off * np.cos(yaw),
        pose.position.y + center_off * np.sin(yaw),
    )


def plan_clearance(msg: Path, emb: Embodiment) -> np.ndarray | None:
    """Per-waypoint room (m) recovered from the plan's stamps, or None.

    A path from a producer that does not speak the precision dialect decodes to
    nothing, and an undecorated plan should draw as one flat colour rather than
    as a confident lie about clearance.
    """
    ceilings = decode_ceilings(msg, emb.min_speed, emb.max_speed)
    return None if ceilings is None else ceilings_to_clearance(ceilings, emb)


def render_plan_body(
    msg: Path,
    emb: Embodiment = GO2,
    height: float = 0.32,
    stride_m: float = 0.35,
    line_radius: float = 0.012,
) -> Archetype | None:
    """The plan's expected body poses as oriented boxes, coloured by room.

    The box is the embodiment's STRAIGHT-DRIFT row, not the all-gait union (see
    motion_visual_override), sitting ``center_off`` along the pose's +x so it is
    where the robot is rather than centred on the pose point. Room colours off
    the embodiment's own governor: red at or under its precision floor, amber
    on the ramp, green past ``speed_clearance``. ``stride_m`` subsamples along
    arc rather than by index: plans are discretised at 0.1 m, so drawing every
    waypoint is an opaque wall of boxes that hides the very geometry it is
    there to show. ``line_radius`` is the wireframe's own thickness in metres
    -- thin edges disappear against a dense point cloud at any useful zoom.
    """
    import rerun as rr

    length, width, center_off, _ = emb.box(0.0)

    n = len(msg.poses)
    if n == 0:
        # the planner emits an empty path when it finds no route; blanking the
        # last good picture on that is worse than holding it
        return None
    if n == 1:
        # a single-pose stub is the planner's VETO -- "no safe route, hold".
        # Draw it, in red: an empty viewport looks like a dead module, and the
        # difference between "refusing" and "crashed" is the whole question
        # when the robot will not move.
        p = msg.poses[0]
        cx, cy = _body_centre(p, center_off)
        return rr.Boxes3D(
            centers=[[cx, cy, p.position.z + height / 2.0 + _Z_LIFT]],
            half_sizes=[[length / 2.0, width / 2.0, height / 2.0]],
            rotation_axis_angles=[rr.RotationAxisAngle([0.0, 0.0, 1.0], rr.Angle(rad=p.yaw))],
            colors=[[255, 60, 60, 200]],
            radii=[line_radius],
            fill_mode="majorwireframe",
            labels=["VETO: no safe route"],
        )

    xy = np.array([[p.position.x, p.position.y] for p in msg.poses]).reshape(-1, 2)
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    arcs = np.concatenate([[0.0], np.cumsum(seg)])
    # one box per stride_m of arc, plus the last pose so the goal end is drawn
    picks = np.unique(np.searchsorted(arcs, np.arange(0.0, float(arcs[-1]) + 1e-9, stride_m)))
    picks = np.unique(np.append(np.clip(picks, 0, n - 1), n - 1))

    clear = plan_clearance(msg, emb)
    centers, angles, colors = [], [], []
    for i in picks:
        p = msg.poses[int(i)]
        cx, cy = _body_centre(p, center_off)
        centers.append([cx, cy, p.position.z + height / 2.0 + _Z_LIFT])
        angles.append(rr.RotationAxisAngle([0.0, 0.0, 1.0], rr.Angle(rad=p.yaw)))
        if clear is None:
            colors.append([100, 160, 255, 160])  # unstamped: the plan line's own blue
        else:
            room = float(clear[int(i)])
            if room <= emb.precision:
                colors.append([255, 60, 60, 220])
            elif room < emb.speed_clearance:
                colors.append([255, 200, 60, 200])
            else:
                colors.append([80, 220, 80, 160])
    return rr.Boxes3D(
        centers=centers,
        half_sizes=[[length / 2.0, width / 2.0, height / 2.0]] * len(centers),
        rotation_axis_angles=angles,
        colors=colors,
        radii=[line_radius] * len(centers),
        fill_mode="majorwireframe",
    )


def motion_visual_override(
    viz_publish_hz: float,
    embodiment: str = "go2",
    line_radius: float = 0.012,
    body_dilate_m: float = 0.0,
) -> dict[str, Any]:
    """rerun overrides for the motion stack, keyed off the planner's publish rate.

    Pass the same ``viz_publish_hz``, ``embodiment`` and ``body_dilate_m`` given
    to ``MotionPlanner.blueprint(...)``: a picture drawn from a body the planner
    did not plan with is a picture of the wrong question.

    The box is the STRAIGHT-DRIFT row, not the all-gait union. The union is what
    a turn-in-place is tested against and is ~0.18 m wider than the box a
    forward edge actually has to fit; drawing it everywhere makes every corridor
    look impassable and hides the margin the plan really has.
    """
    emb = EMBODIMENTS[embodiment].dilated(by=body_dilate_m)
    on = viz_publish_hz > 0.0
    body = partial(render_plan_body, emb=emb, line_radius=line_radius)
    return {"world/plan_body": body if on else None}
