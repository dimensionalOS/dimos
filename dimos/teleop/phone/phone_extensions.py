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

"""Phone teleop module extensions.

Available subclasses:
    - SimplePhoneTeleop: Filters to ground robot axes and outputs cmd_vel: Out[Twist]
"""

from dimos.core import Out
from dimos.msgs.geometry_msgs import Twist, TwistStamped, Vector3
from dimos.teleop.phone.phone_teleop_module import PhoneTeleopModule


class SimplePhoneTeleop(PhoneTeleopModule):
    """Phone teleop for ground robots.

    Filters the raw 6-axis twist to mobile base axes (linear.x, linear.y, angular.z)
    and publishes as Twist on cmd_vel for direct autoconnect wiring with any
    module that has cmd_vel: In[Twist].
    """

    cmd_vel: Out[Twist]

    def _publish_msg(self, output_msg: TwistStamped) -> None:
        self.cmd_vel.publish(
            Twist(
                linear=Vector3(x=output_msg.linear.x, y=output_msg.linear.y, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=output_msg.linear.z),
            )
        )


simple_phone_teleop_module = SimplePhoneTeleop.blueprint

__all__ = [
    "SimplePhoneTeleop",
    "simple_phone_teleop_module",
]
