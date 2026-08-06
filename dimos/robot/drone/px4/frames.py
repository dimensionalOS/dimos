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

"""Frame convention conversion helpers for MAVSDK velocity commands.

DIMOS uses FLU (Forward-Left-Up) internally for body-frame velocity.
MAVSDK v4, like PX4, uses FRD (Forward-Right-Down) in its Offboard
``set_velocity_body`` API.

Conversion signs::

    FLU axis   FRD axis   Sign
    forward    forward    +1  (same direction)
    left       right      -1  (opposite sign)
    up         down       -1  (opposite sign)
    yaw CCW    yaw CW     -1  (MAVSDK uses deg/s CW-positive)

MAVSDK's ``VelocityBodyYawspeed`` struct accepts ``right_m_s``,
``down_m_s``, and ``yawspeed_deg_s``. This module negates the DIMOS FLU
convention into those FRD fields and converts rad/s to deg/s.
"""

import math
from typing import TypeAlias

BodyVelocityFru: TypeAlias = tuple[float, float, float, float]


def flu_to_frd_body_velocity(
    forward_m_s: float,
    left_m_s: float,
    up_m_s: float,
    yaw_rate_ccw_rad_s: float,
) -> BodyVelocityFru:
    """Convert body FLU velocity and CCW yaw rate to MAVSDK body FRD values.

    Args:
        forward_m_s: Velocity along the forward (X) axis in the FLU frame.
        left_m_s: Velocity along the left (Y) axis in the FLU frame.
        up_m_s: Velocity along the up (Z) axis in the FLU frame.
        yaw_rate_ccw_rad_s: Counter-clockwise yaw angular velocity in rad/s.

    Returns:
        A 4-tuple ``(forward_m_s, right_m_s, down_m_s, yawspeed_deg_s)``
        suitable for constructing :class:`VelocityBodyYawspeed`.

    Raises:
        ValueError: Any input is not finite (inf or NaN).
    """
    values = (forward_m_s, left_m_s, up_m_s, yaw_rate_ccw_rad_s)
    if not all(math.isfinite(value) for value in values):
        msg = "frame conversion inputs must be finite"
        raise ValueError(msg)
    return (
        forward_m_s,
        -left_m_s,
        -up_m_s,
        -math.degrees(yaw_rate_ccw_rad_s),
    )
