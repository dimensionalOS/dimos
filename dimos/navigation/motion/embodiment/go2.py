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


"""The Unitree Go2, as measured: the one place its numbers live."""

from __future__ import annotations

from dataclasses import replace

from dimos.navigation.motion.control.controller import ControllerConfig

from .base import Embodiment

GO2 = Embodiment(
    # Moving-body union over a command sweep: the swinging legs, not the 0.31 m
    # trunk, set the width.
    length=0.883,
    width=0.593,
    center_off=0,  # body center relative to the pose point
    comfort=0.4,  # obstacles-we-care-about radius (m)
    precision=0.05,  # local control tracking accuracy; the clearance floor (m)
    max_speed=0.5,  # cruise, granted at speed_clearance of room (m/s)
    min_speed=0.2,  # creep at the precision floor (m/s)
    speed_clearance=0.35,  # room at which full speed is granted (m)
    max_yaw_rate=1.4,  # rad/s; prices a rotation in place
    command_slew=(2.5, 2.0, 5.0),  # d(vx, vy, wz)/dt the walking policy ramps at
    gait_band=(0.45, 0.95),  # commanded speeds it actually walks between (m/s)
    walk_gain=0.964,  # ground speed ~= walk_gain * cmd - walk_slip, probed open loop
    walk_slip=0.132,
    walk_slip_ramp=0.08,  # below this intended speed the slip inverse fades to identity
    strafe=1.3,  # planner cost of a metre sideways, forward = 1
    reverse=1.8,  # ...and backwards
    yaw_w=0.15,  # planner cost per rad of rotation
    steppable=0.20,  # legs negotiate obstacles below this (m) - at a cost (TODO)
    height=0.45,  # above this the body passes underneath; not an obstacle (m)
    base_height=0.29,  # base origin above support; frame plumbing, not semantics (m)
    # Baked by the fitted-sim envelope sweep over the governed slow band (stand +
    # 0.35 + 0.50 m/s). The sweep lives with the sim (README, "Not on this
    # branch"); this is its output, which is what the planner reads.
    envelope=(
        (0.0, 0.819, 0.416, -0.023, 0.000),
        (26.6, 0.802, 0.436, -0.032, -0.008),
        (45.0, 0.788, 0.472, -0.035, -0.018),
        (63.4, 0.781, 0.500, -0.039, -0.016),
        (90.0, 0.781, 0.507, -0.039, -0.009),
        (116.6, 0.781, 0.497, -0.039, 0.000),
        (135.0, 0.781, 0.463, -0.039, -0.001),
        (153.4, 0.781, 0.422, -0.039, -0.003),
        (180.0, 0.781, 0.416, -0.039, 0.000),
    ),
    arc_inflate=0.0334,  # extra width per rad/m of curvature, residuals <= 12 mm
    # The follower tuning searched on this body in the closed-loop lab (README,
    # "Not on this branch"). Fitted, unlike everything above, which is why it
    # is its own record.
    control=ControllerConfig(
        lookahead=0.35,  # carrot distance along the path (m)
        k_pos=2.0,  # body-frame position error gain (1/s)
        k_yaw=2.0,  # yaw error gain (1/s)
        fan_yaw_per_m=3.0,  # yaw-per-metre above this is a rotation in place, not a curve
        fan_yaw_done=0.25,  # a fan holds position until the yaw error is under this (rad)
        speed_lookahead=2.0,  # the governor reads room over this much path ahead (m)
    ),
)

# payload adds 8 cm in front: longer body, centre 4 cm further forward.
# No measured envelope of its own: it falls back to the union everywhere.
GO2_PAYLOAD = replace(
    GO2,
    length=0.963,
    center_off=0.042,
    comfort=0.5,
    envelope=(),
    arc_inflate=0.0,
)
