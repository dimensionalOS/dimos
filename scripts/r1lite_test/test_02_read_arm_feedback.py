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

"""
Test 2: Read Arm Feedback
Subscribe to left arm joint feedback and print 5 messages.

Ported from scripts/r1pro_test/test_02_read_arm_feedback.py.

Run standalone:
    export ROS_DOMAIN_ID=<see r1lite_config>
    python3 scripts/r1lite_test/test_02_read_arm_feedback.py

Or via run_all_tests.py (preferred, single DDS session).

Pass condition: Prints 5 joint position arrays (ARM_DOF values each).
"""

from pathlib import Path
import sys
import time

import rclpy
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
from r1lite_config import ARM_DOF, FEEDBACK_ARM

MAX = 5
DISCOVERY_WAIT = 5.0
RECEIVE_TIMEOUT = 5.0

FEEDBACK_TOPIC = FEEDBACK_ARM.format(side="left")


def main() -> bool:
    """Run arm feedback test. Assumes rclpy.init() already called."""
    node = rclpy.create_node("dimos_arm_reader")
    count = [0]

    def cb(msg):
        count[0] += 1
        if len(msg.position) != ARM_DOF:
            print(
                f"NOTE: feedback has {len(msg.position)} joints, "
                f"r1lite_config.ARM_DOF says {ARM_DOF}, fix the config!"
            )
        print(f"[{count[0]}/{MAX}] positions: {[round(p, 4) for p in msg.position]}")
        print(f"       velocities: {[round(v, 4) for v in msg.velocity]}")
        print(f"       efforts:    {[round(e, 4) for e in msg.effort]}")

    node.create_subscription(JointState, FEEDBACK_TOPIC, cb, 10)

    print("Waiting for DDS peer discovery...")
    deadline = time.time() + DISCOVERY_WAIT
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"Waiting for {FEEDBACK_TOPIC} messages...")
    deadline = time.time() + RECEIVE_TIMEOUT
    while count[0] < MAX and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    passed = count[0] >= MAX
    if passed:
        print(f"\nPASS: Received {count[0]} messages")
    else:
        print(f"\nFAIL: Only received {count[0]}/{MAX} messages, is the robot stack running?")

    node.destroy_node()
    return passed


if __name__ == "__main__":
    rclpy.init()
    result = main()
    rclpy.shutdown()
    exit(0 if result else 1)
