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

"""Agent-facing odometry measurements, without inferred motion or frame transforms."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.msgs.nav_msgs.Odometry import Odometry

LEGEND = (
    "Odometry: ts is seconds; position_m and orientation_xyzw describe child_frame_id "
    "in frame_id. Orientation is a quaternion in [x,y,z,w] order. "
    "linear_velocity_m_s and angular_velocity_rad_s are [x,y,z] in child_frame_id. "
    "No frame conversion or trajectory inference is performed. Covariances are not included."
)


def encode(odometry: Odometry) -> dict[str, Any]:
    """Return native-frame pose and twist with explicit units and axis ordering."""
    orientation = odometry.orientation
    return {
        "ts": float(odometry.ts),
        "frame_id": odometry.frame_id,
        "child_frame_id": odometry.child_frame_id,
        "position_m": [float(odometry.x), float(odometry.y), float(odometry.z)],
        "orientation_xyzw": [
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ],
        "linear_velocity_m_s": [float(odometry.vx), float(odometry.vy), float(odometry.vz)],
        "angular_velocity_rad_s": [float(odometry.wx), float(odometry.wy), float(odometry.wz)],
    }
