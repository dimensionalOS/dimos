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

"""Leg odometry: joint encoders + foot contact + IMU orientation -> hub pose and velocity.

When a foot touches down it is anchored on the ground (z = 0) at its flat-center position.
While it stays in contact, the hub position is the anchor minus the foot's current offset
from the hub (from forward kinematics, rotated by the IMU orientation). Offsets from all
contacting feet are averaged, weighted by contact force. Heading comes from the IMU.
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray

from tars_sdk.kinematics import SlabGeometry, heading, quat_to_mat, rot_y
from tars_sdk.types import N_SLABS, Measurement, Odometry


class LegOdometry:
    def __init__(
        self, geom: SlabGeometry, contact_threshold: float = 30.0, vel_filter_hz: float = 5.0
    ) -> None:
        self.g = geom
        self.contact_threshold = contact_threshold
        self.vel_filter_hz = vel_filter_hz
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)  # world frame
        self.anchors: list[NDArray[np.float64] | None] = [None] * N_SLABS
        self.odom = Odometry()

    def _foot_offset(
        self, R: NDArray[np.float64], i: int, hinge: float, slide: float
    ) -> NDArray[np.float64]:
        """World-frame vector from hub to foot i's flat center (on the ground when flat)."""
        c, r = self.g.half_depth, self.g.pivot_to_bottom + slide
        y = self.g.slab_y[i]
        edges = []
        for e in (c, -c):
            x, z = rot_y(hinge, e, -r)
            edges.append((R @ np.array([x, y, z]), e))
        edge_w, e = min(edges, key=lambda t: t[0][2])  # lowest edge touches the ground
        ax, az = rot_y(hinge, 1.0, 0.0)
        heading = R @ np.array([ax, 0.0, az])
        heading[2] = 0.0
        heading /= max(np.linalg.norm(heading), 1e-9)
        return edge_w - math.copysign(c, e) * heading

    def update(self, meas: Measurement, dt: float) -> Odometry:
        R = quat_to_mat(meas.imu_quat)
        num, den = np.zeros(3), 0.0
        for i in range(N_SLABS):
            f = float(meas.foot_force[i])
            if f < self.contact_threshold:
                self.anchors[i] = None
                continue
            off = self._foot_offset(
                R, i, float(meas.joint_q[2 * i]), float(meas.joint_q[2 * i + 1])
            )
            anchor = self.anchors[i]
            if anchor is None:
                anchor = np.array([self.pos[0] + off[0], self.pos[1] + off[1], 0.0])
                self.anchors[i] = anchor
            num += f * (anchor - off)
            den += f
        new_pos = num / den if den > 0 else self.pos + self.vel * dt
        if dt > 0:
            alpha = 1.0 - math.exp(-2 * math.pi * self.vel_filter_hz * dt)
            self.vel += alpha * ((new_pos - self.pos) / dt - self.vel)
        self.pos = new_pos

        yaw = heading(R)
        cy, sy = math.cos(yaw), math.sin(yaw)
        o = self.odom
        o.x, o.y, o.z, o.yaw = float(self.pos[0]), float(self.pos[1]), float(self.pos[2]), yaw
        o.vx = float(cy * self.vel[0] + sy * self.vel[1])
        o.vy = float(-sy * self.vel[0] + cy * self.vel[1])
        o.wz = float(meas.imu_gyro[2])
        o.quat = meas.imu_quat.copy()
        return o
