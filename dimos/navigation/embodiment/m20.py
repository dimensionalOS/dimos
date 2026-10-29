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


"""The Deep Robotics Lynx M20, provisional: nothing below is measured on the robot yet."""

from __future__ import annotations

from dimos.navigation.embodiment.base import ControllerConfig, Embodiment

M20 = Embodiment(
    # Vendor hardware guide body dimensions.
    length=0.82,
    width=0.506,
    center_off=0,  # body center relative to the pose point
    comfort=0.4,  # obstacles-we-care-about radius (m)
    precision=0.05,  # local control tracking accuracy, the clearance floor (m)
    max_speed=0.7,  # cruise (m/s)
    min_speed=0.2,  # creep at the precision floor (m/s)
    speed_clearance=0.35,  # room at which full speed is granted (m)
    max_yaw_rate=1.0,  # prices a rotation in place (rad/s)
    command_slew=(1.0, 1.0, 2.0),  # d(vx, vy, wz)/dt
    gait_band=(0.0, 0.7),  # assumed to hold any commanded speed up to cruise (m/s)
    walk_gain=1.0,  # assumed to deliver the command
    walk_slip=0.0,
    walk_slip_ramp=0.08,
    strafe=1.5,  # holonomic if /NAV_CMD honors vy, priced so the long body turns to face the path
    reverse=1.8,  # and backwards
    yaw_w=0.15,  # planner cost per rad of rotation
    steppable=0.25,  # the MLS planner's step threshold (m)
    height=0.65,  # MLS planning height, body top plus margin (m)
    base_height=0.48,  # the vendor SDK stand height (m)
    control=ControllerConfig(
        lookahead=0.35,  # carrot distance along the path (m)
        k_pos=2.0,  # body-frame position error gain (1/s)
        k_yaw=2.0,  # yaw error gain (1/s)
        fan_yaw_per_m=3.0,  # yaw-per-meter above this is a rotation in place, not a curve
        fan_yaw_done=0.25,  # a fan holds position until the yaw error is under this (rad)
        speed_lookahead=2.0,  # the governor reads room over this much path ahead (m)
    ),
)
