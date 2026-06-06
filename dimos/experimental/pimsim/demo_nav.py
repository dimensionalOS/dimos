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

"""Debug harness: bring up pimsim + nav stack, publish a goal, watch every LCM
channel so we can see which link in the nav chain is (not) flowing."""

from __future__ import annotations

import os
import sys
import time

import lcm as lcmlib

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.e2e_tests.test_pimsim_cross_wall import _babylon_nav_blueprint
from dimos.experimental.pimsim.client import PimSimClient
from dimos.experimental.pimsim.headless import HeadlessBrowser
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.protocol.service.lcmservice import _DEFAULT_LCM_URL

GOAL_TOPIC = "/clicked_point#geometry_msgs.PointStamped"
ODOM_TOPIC = "/odom#geometry_msgs.PoseStamped"
CMD_TOPIC = "/nav_cmd_vel#geometry_msgs.Twist"


def main() -> int:
    coordinator = ModuleCoordinator.build(_babylon_nav_blueprint())
    state = {"x": 0.0, "y": 0.0, "cx": 0.0, "cy": 0.0, "cz": 0.0}

    def on_odom(_channel: str, data: bytes) -> None:
        msg = PoseStamped.lcm_decode(data)
        state["x"], state["y"] = msg.x, msg.y

    def on_cmd(_channel: str, data: bytes) -> None:
        msg = Twist.lcm_decode(data)
        state["cx"], state["cy"], state["cz"] = msg.linear.x, msg.linear.y, msg.angular.z

    lcm = lcmlib.LCM(_DEFAULT_LCM_URL)
    lcm.subscribe(ODOM_TOPIC, on_odom)
    lcm.subscribe(CMD_TOPIC, on_cmd)

    browser = HeadlessBrowser()
    client = PimSimClient()
    try:
        browser.start()
        client.start()
        client.set_agent_position(0.0, 0.0)
        client.add_wall(-3.0, 2.0, 1.2, 2.0)
        client.add_wall(2.7, 2.0, 3.0, 2.0)

        end = time.time() + 12
        while time.time() < end:
            lcm.handle_timeout(200)

        goal = PointStamped(x=0.0, y=4.0, z=0.0, ts=time.time(), frame_id="map")
        lcm.publish(GOAL_TOPIC, goal.lcm_encode())
        print("[nav] goal=(0,4) published; tracking odom + nav_cmd_vel")
        end = time.time() + 40
        next_print = time.time()
        while time.time() < end:
            lcm.handle_timeout(200)
            if time.time() >= next_print:
                next_print += 2.0
                print(
                    f"[nav] odom=({state['x']:.2f},{state['y']:.2f}) "
                    f"cmd_vel lin=({state['cx']:.2f},{state['cy']:.2f}) ang={state['cz']:.2f}"
                )
    finally:
        browser.stop()
        client.stop()
        coordinator.stop()
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.stdout.flush()
    os._exit(exit_code)
