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


"""The Habitat sim body: an ideal kinematic point that slides along the navmesh."""

from __future__ import annotations

from dimos.navigation.embodiment.base import ControllerConfig, Embodiment

HABITAT = Embodiment(
    # Square around the navmesh agent's 0.1 m radius.
    length=0.2,
    width=0.2,
    center_off=0,  # body center relative to the pose point
    comfort=0.4,  # obstacles-we-care-about radius (m)
    precision=0.05,  # one voxel of the nav map, the clearance floor (m)
    max_speed=0.5,  # cruise (m/s)
    min_speed=0.2,  # creep at the precision floor (m/s)
    speed_clearance=0.35,  # room at which full speed is granted (m)
    max_yaw_rate=1.5,  # prices a rotation in place (rad/s)
    command_slew=(2.0, 2.0, 5.0),  # d(vx, vy, wz)/dt
    gait_band=(0.0, 0.5),  # executes any commanded speed up to cruise (m/s)
    walk_gain=1.0,  # the sim integrates the command exactly
    walk_slip=0.0,
    walk_slip_ramp=0.08,
    strafe=1.5,  # holonomic, but priced above forward so the camera faces long legs
    reverse=1.8,  # backwards drives into what the camera has not seen
    yaw_w=0.15,  # planner cost per rad of rotation
    steppable=0.2,  # the navmesh's default max climb (m)
    height=0.5,  # ROBOT_HEIGHT in simulation/habitat/blueprints.py (m)
    base_height=0.0,  # base_link sits on the navmesh (m)
    control=ControllerConfig(
        lookahead=0.35,  # carrot distance along the path (m)
        k_pos=2.0,  # body-frame position error gain (1/s)
        k_yaw=2.0,  # yaw error gain (1/s)
        fan_yaw_per_m=3.0,  # yaw-per-meter above this is a rotation in place, not a curve
        fan_yaw_done=0.25,  # a fan holds position until the yaw error is under this (rad)
        speed_lookahead=2.0,  # the governor reads room over this much path ahead (m)
    ),
)
