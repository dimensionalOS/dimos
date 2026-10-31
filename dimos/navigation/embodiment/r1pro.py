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
    # DanHolonomicTC's "walk" profile the R1 ran before: 0.55 m/s, 1 rad/s, 1 m/s^2; the 10-28 drive reached 0.56 m/s and 1.05 rad/s on it.
    max_speed=0.55,  # cruise (m/s)
    min_speed=0.2,  # creep at the precision floor (m/s)
    speed_clearance=0.35,  # room at which full speed is granted (m)
    max_yaw_rate=1.0,  # prices a rotation in place (rad/s)
    # Fit on the 10-28 R1 drive (~90 s per axis, chassis/cmd_vel vs Point-LIO): p95 accel, then the base lags ~0.3 s (vx, vy) / 0.4 s (wz) plus tau 0.2 / 0.46 s.
    command_slew=(1.1, 1.0, 1.3),  # d(vx, vy, wz)/dt
    gait_band=(0.0, 0.55),  # wheels hold any commanded speed up to cruise (m/s)
    # Measured ground/cmd ~0.8 (vx), ~0.87 (vy), ~0.95 (wz), but kept at identity: the inverse would command above 0.55.
    walk_gain=1.0,
    walk_slip=0.0,
    walk_slip_ramp=0.08,
    strafe=1.5,  # holonomic, but priced above forward so it turns to face long legs
    reverse=1.8,  # backwards drives into what the torso hides from the head camera
    yaw_w=0.15,  # planner cost per rad of rotation
    steppable=0.07,  # one voxel layer of the nav map, which cannot resolve the wheels' 3 cm (m)
    height=R1PRO_OVERHEAD_CLEARANCE_M,  # anything above it passes overhead (m)
    base_height=0.0,  # base_link sits at the wheel contact (m)
    control=ControllerConfig(
        lookahead=0.35,  # carrot distance along the path (m)
        k_pos=2.0,  # body-frame position error gain (1/s)
        k_yaw=0.8,  # yaw error gain (1/s); the swerve wheels turn before the body does
        fan_yaw_per_m=3.0,  # yaw-per-meter above this is a rotation in place, not a curve
        fan_yaw_done=0.25,  # a fan holds position until the yaw error is under this (rad)
        speed_lookahead=2.0,  # the governor reads room over this much path ahead (m)
    ),
)
