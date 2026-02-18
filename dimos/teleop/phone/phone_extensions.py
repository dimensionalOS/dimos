#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Phone teleop module extensions and subclasses.

Available subclasses:
    - SimplePhoneTeleop: Maps raw twist to ground robot axes (linear.x, linear.y, angular.z)
    - PhoneTwistTeleop: Outputs cmd_vel: Out[Twist] for direct autoconnect with any Twist consumer
"""

from dimos.core import Out
from dimos.msgs.geometry_msgs import Twist, TwistStamped, Vector3
from dimos.teleop.phone.phone_teleop_module import PhoneTeleopModule


class SimplePhoneTeleop(PhoneTeleopModule):
    """Phone teleop mapped to ground robot axes.

    Takes the raw 6-axis twist from the base module and extracts only the
    axes relevant for a mobile base:
        - linear.x  (forward/back from pitch tilt)
        - linear.y  (strafe from roll tilt)
        - angular.z (turn from yaw orientation delta)

    All other axes are zeroed.
    """

    def _get_output_twist(self) -> TwistStamped | None:
        raw = super()._get_output_twist()
        if raw is None:
            return None

        # raw.linear.z is the yaw orientation delta (already scaled by linear_gain
        # in the base class), remapped here to angular.z for ground-robot turning.
        return TwistStamped(
            ts=raw.ts,
            frame_id="mobile_base",
            linear=Vector3(x=raw.linear.x, y=raw.linear.y, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=raw.linear.z),
        )


class PhoneTwistTeleop(SimplePhoneTeleop):
    """Phone teleop that outputs Twist on cmd_vel.

    Extends SimplePhoneTeleop with cmd_vel: Out[Twist]
    for direct autoconnect wiring with any module that has cmd_vel: In[Twist].
    """

    cmd_vel: Out[Twist]

    def _publish_msg(self, output_msg: TwistStamped) -> None:
        """Publish as Twist on cmd_vel.
        Intentionally bypasses the base twist_output stream — only cmd_vel is used.
        """
        self.cmd_vel.publish(Twist(linear=output_msg.linear, angular=output_msg.angular))


simple_phone_teleop_module = SimplePhoneTeleop.blueprint
phone_twist_teleop_module = PhoneTwistTeleop.blueprint

__all__ = [
    "PhoneTwistTeleop",
    "SimplePhoneTeleop",
    "phone_twist_teleop_module",
    "simple_phone_teleop_module",
]
