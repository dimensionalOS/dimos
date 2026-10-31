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

"""The Galaxea R1 Pro, as measured: the one place its numbers live."""

from __future__ import annotations

import math

from dimos.navigation.embodiment.base import ControllerConfig, Embodiment
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_CHASSIS_WIDTH_M,
    R1PRO_OVERHEAD_CLEARANCE_M,
    R1PRO_ROTATION_DIAMETER_M,
)

R1PRO = Embodiment(
    # The chassis box whose diagonal is the rotation diameter.
    length=math.sqrt(R1PRO_ROTATION_DIAMETER_M**2 - R1PRO_CHASSIS_WIDTH_M**2),
    width=R1PRO_CHASSIS_WIDTH_M,
    center_off=0,  # body center relative to the pose point
    comfort=0.4,  # obstacles-we-care-about radius (m)
    precision=0.05,  # local control tracking accuracy, the clearance floor (m)
    # Ground speed, so the walk inverse below asks 0.97 m/s of the 1.0 m/s R1PRO_PLANAR_BASE limit; the 10-28 drive only measured up to 0.55.
    max_speed=0.8,  # cruise (m/s)
    min_speed=0.2,  # creep at the precision floor (m/s)
    speed_clearance=0.35,  # room at which full speed is granted (m)
    # Under the planar base's 2.0 rad/s; the vendor gatekeeper caps angular accel at 1.0 rad/s^2, so 1.5 is reached in 1.5 s.
    max_yaw_rate=1.5,  # prices a rotation in place (rad/s)
    # Fit on the 10-28 R1 drive (~90 s per axis, chassis/cmd_vel vs Point-LIO): p95 accel; the base lags 0.27 s + tau 0.2 s (vx, vy), 0.4 s + 0.46 s (wz).
    command_slew=(1.1, 1.0, 1.3),  # d(vx, vy, wz)/dt
    gait_band=(0.1, 1.0),  # below ~0.1 commanded the base delivers about a third (m/s)
    # Measured ground ~= 0.845 * cmd - 0.023 (vx, 0.15-0.55 m/s). The follower's walk_command is cmd = gain * want + slip, so these are its inverse.
    walk_gain=1.18,
    walk_slip=0.027,
    walk_slip_ramp=0.08,
    strafe=1.5,  # holonomic, but priced above forward so it turns to face long legs
    reverse=1.8,  # backwards drives into what the torso hides from the head camera
    yaw_w=0.15,  # planner cost per rad of rotation
    steppable=0.07,  # one voxel layer of the nav map, which cannot resolve the wheels' 3 cm (m)
    height=R1PRO_OVERHEAD_CLEARANCE_M,  # anything above it passes overhead (m)
    base_height=0.0,  # base_link sits at the wheel contact (m)
    control=ControllerConfig(
        lookahead=0.35,  # carrot distance along the path (m)
        k_pos=1.7,  # body-frame position error gain (1/s); 45 deg phase margin over the measured 0.27 s + 0.2 s lag
        k_yaw=0.9,  # yaw error gain (1/s); 45 deg phase margin over the measured 0.4 s + 0.46 s lag
        fan_yaw_per_m=3.0,  # yaw-per-meter above this is a rotation in place, not a curve
        fan_yaw_done=0.25,  # a fan holds position until the yaw error is under this (rad)
        speed_lookahead=2.0,  # the governor reads room over this much path ahead (m)
    ),
)
