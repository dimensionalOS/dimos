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

"""Pure Cartesian examples for offline curve plots, independent of the joint task."""

from __future__ import annotations

import math

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.teleop.policy.takeover import Weights6, distance_to_target, quadratic


def bottle_cap_takeover(pose: Transform) -> Weights6:
    """Human lifts along world Z; policy mainly rotates about world Z.

    Approaching the example cap at (0.4, 0, 0.4) increases policy authority.
    These are velocity weights, independent of the TCP's quaternion components.
    """
    alpha = quadratic(distance_to_target(pose, (0.4, 0.0, 0.4), outer_radius=0.5))
    return (alpha, alpha, 0.1 * alpha, alpha, alpha, 0.8 + 0.2 * alpha)


def bottle_cap_commands(
    elapsed: float, duration_s: float, world_frame: str
) -> tuple[TwistStamped, TwistStamped]:
    """One gentle lift-and-return and twist-and-return, then zero velocities."""
    speed = math.sin(2 * math.pi * elapsed / duration_s) if 0 <= elapsed < duration_s else 0.0
    return (
        TwistStamped(
            frame_id=world_frame, linear=[0, 0, 0.02 * speed], angular=[0, 0, 0.02 * speed]
        ),
        TwistStamped(
            frame_id=world_frame, linear=[0, 0, 0.002 * speed], angular=[0, 0, 0.3 * speed]
        ),
    )
