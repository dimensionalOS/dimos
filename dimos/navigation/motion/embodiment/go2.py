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


"""The Unitree Go2, as measured."""

from __future__ import annotations

from dataclasses import replace

from dimos.navigation.motion.control.controller import ControllerConfig

from .base import Embodiment

# Baked by the fitted-sim envelope sweep over the governed slow band (stand +
# 0.35 + 0.50 m/s). The sweep and the surface these fold out of are the sim's,
# and live with it (README, "What is not here") -- what is here is its measured
# output, which is what the planner and the judge actually read.
GO2_ENVELOPE: tuple[tuple[float, float, float, float, float], ...] = (
    (0.0, 0.819, 0.416, -0.023, 0.000),
    (26.6, 0.802, 0.436, -0.032, -0.008),
    (45.0, 0.788, 0.472, -0.035, -0.018),
    (63.4, 0.781, 0.500, -0.039, -0.016),
    (90.0, 0.781, 0.507, -0.039, -0.009),
    (116.6, 0.781, 0.497, -0.039, 0.000),
    (135.0, 0.781, 0.463, -0.039, -0.001),
    (153.4, 0.781, 0.422, -0.039, -0.003),
    (180.0, 0.781, 0.416, -0.039, 0.000),
)

# Measured 0.0334 m of extra width per rad/m, residuals <= 12 mm.
GO2_ARC_INFLATE = 0.0334

# The follower tuning searched on this body: gen40 of the hinted lab on the
# closed-loop referee (README, "What is not here"). Fitted, unlike everything
# above, which is why it is its own record.
GO2_CONTROL = ControllerConfig(
    lookahead=0.35,  # carrot distance along the path (m)
    k_pos=2.0,  # body-frame position error gain (1/s)
    k_yaw=2.0,  # yaw error gain (1/s)
    fan_yaw_per_m=3.0,  # yaw-per-metre above this is a rotation in place, not a curve
    fan_yaw_done=0.25,  # a fan holds position until the yaw error is under this (rad)
    speed_lookahead=2.0,  # the governor reads room over this much path ahead (m)
    tangent_preview=0.15,  # centred window the feedforward reads the plan's direction over
    escape_clearance=0.10,  # below this room the pinch-escape leg lifts the floor (m)
    escape_preview=1.00,  # ...read over this much arc ahead (m)
    escape_speed=0.75,  # ...up to this speed where the room has run out (m/s)
    brake_accel=0.8,  # deceleration a previewed waypoint credits the body with (m/s^2)
    brake_margin=0.15,  # within this arc a previewed waypoint binds in full (m)
)

GO2 = Embodiment(envelope=GO2_ENVELOPE, arc_inflate=GO2_ARC_INFLATE, control=GO2_CONTROL)

# payload adds 8 cm in front: longer body, centre 4 cm further forward.
# No measured envelope of its own: it falls back to the union everywhere.
GO2_PAYLOAD = replace(
    GO2,
    tag="go2-payload",
    length=0.963,
    center_off=0.042,
    comfort=0.5,
    envelope=(),
    arc_inflate=0.0,
)
