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

"""Stock ``align(..., interpolator=)`` helpers for the common geometry types.

``Stream.align`` pairs each primary observation with the nearest secondary
within tolerance; with an :data:`~dimos.memory2.stream.AlignInterpolator` it
instead synthesizes the secondary value at the exact primary timestamp from
the two bracketing samples - a LIDAR scan gets the pose *at scan time*, not
the pose captured closest to it. These are the shared interpolators for that
hook, so fusion modules do not re-derive slerp::

    image.align(self.streams.pose, tolerance=0.1, interpolator=lerp_pose)
    lidar.align(self.streams.imu, tolerance=0.02, interpolator=interp_odom)

Each helper follows the ``(prev_obs, nxt_obs, alpha) -> payload`` contract:
both sides are full :class:`Observation`\\s bracketing the primary and
``alpha`` in ``(0, 1)`` is the primary's fraction of the bracket span.
Positions and velocities interpolate linearly; orientations slerp along the
shortest arc. The returned payload is stamped at the interpolated time
(``prev.ts + alpha * (nxt.ts - prev.ts)`` - the primary ts by the align
contract) and keeps ``prev``'s frame ids.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry

if TYPE_CHECKING:
    from dimos.memory2.type.observation import Observation


def _slerp(q0: Quaternion, q1: Quaternion, alpha: float) -> Quaternion:
    """Spherical interpolation along the shortest arc between two rotations.

    ``q`` and ``-q`` encode the same rotation, so a negative dot product flips
    one side first - otherwise the path would swing the long way around.
    Nearly parallel inputs fall back to normalized lerp (``sin(theta) -> 0``
    would blow up the slerp weights).
    """
    q0, q1 = q0.normalize(), q1.normalize()
    dot = q0.dot(q1)
    if dot < 0.0:
        q1 = Quaternion(-q1.x, -q1.y, -q1.z, -q1.w)
        dot = -dot
    if dot > 0.9995:
        return Quaternion(
            q0.x + alpha * (q1.x - q0.x),
            q0.y + alpha * (q1.y - q0.y),
            q0.z + alpha * (q1.z - q0.z),
            q0.w + alpha * (q1.w - q0.w),
        ).normalize()
    theta = math.acos(min(1.0, dot))
    sin_theta = math.sin(theta)
    w0 = math.sin((1.0 - alpha) * theta) / sin_theta
    w1 = math.sin(alpha * theta) / sin_theta
    return Quaternion(
        w0 * q0.x + w1 * q1.x,
        w0 * q0.y + w1 * q1.y,
        w0 * q0.z + w1 * q1.z,
        w0 * q0.w + w1 * q1.w,
    )


def lerp_pose(
    prev: Observation[PoseStamped], nxt: Observation[PoseStamped], alpha: float
) -> PoseStamped:
    """Pose at the primary timestamp: lerp the position, slerp the orientation."""
    a, b = prev.data, nxt.data
    return PoseStamped(
        ts=prev.ts + alpha * (nxt.ts - prev.ts),
        frame_id=a.frame_id,
        position=a.position + (b.position - a.position) * alpha,
        orientation=_slerp(a.orientation, b.orientation, alpha),
    )


def interp_odom(
    prev: Observation[Odometry], nxt: Observation[Odometry], alpha: float
) -> Odometry:
    """Odometry at the primary timestamp: lerp position and twist, slerp orientation.

    Covariances are not interpolated - the result carries the default
    covariance, since blending two covariance matrices linearly would invent
    confidence neither sample had.
    """
    a, b = prev.data, nxt.data
    return Odometry(
        ts=prev.ts + alpha * (nxt.ts - prev.ts),
        frame_id=a.frame_id,
        child_frame_id=a.child_frame_id,
        pose=Pose(
            position=a.position + (b.position - a.position) * alpha,
            orientation=_slerp(a.orientation, b.orientation, alpha),
        ),
        twist=Twist(
            a.linear_velocity + (b.linear_velocity - a.linear_velocity) * alpha,
            a.angular_velocity + (b.angular_velocity - a.angular_velocity) * alpha,
        ),
    )
