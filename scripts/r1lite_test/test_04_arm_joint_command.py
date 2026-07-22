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
Test 4: Arm Joint Movement (wrist roll)
Reads current arm position, moves the LAST joint (wrist roll) by DELTA
radians, holds, then returns home.

This exact motion (left wrist roll +0.2 rad
via streamed targets, all other joints held at their current positions) ran
cleanly on the R1 Lite. The wrist roll rotates the gripper in place, the
safest arm motion in the tabletop fold configuration. Do NOT change
JOINT_INDEX to 0 (R1 Pro style): the base joint sweeps the whole arm
horizontally toward the camera tower / other arm.

Commands MUST be streamed continuously (jointTracker is a dead-man
follower, motion stops when the stream stops). Ctrl-C mid-run is always
safe: the arm simply stays where it is.

WARNING: Arm will physically move (gripper rotates ~11 degrees in place).

Run standalone:
    export ROS_DOMAIN_ID=<see r1lite_config>
    python3 scripts/r1lite_test/test_04_arm_joint_command.py

Or via run_all_tests.py (preferred, single DDS session).

Pass condition: Arm moves noticeably then returns to start position.
"""

from pathlib import Path
import sys
import time

import rclpy
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
from r1lite_config import ARM_DOF, CMD_ARM, FEEDBACK_ARM

SIDE = "left"  # change to "right" to test right arm
JOINT_INDEX = -1  # LAST joint = wrist roll, rotates in place (see header)
DELTA = 0.2  # radians (~11 degrees)
MOVE_DURATION = 4.0  # seconds of streaming per move (tracker needs the stream)
VELOCITY = 0.5  # rad/s tracking speed
DISCOVERY_WAIT = 5.0

FEEDBACK_TOPIC = FEEDBACK_ARM.format(side=SIDE)
CMD_TOPIC = CMD_ARM.format(side=SIDE)


def main(skip_prompt=False) -> bool:
    """Run arm movement test. Assumes rclpy.init() already called."""
    node = rclpy.create_node("dimos_arm_cmd_test")
    current_pos = [None]  # first feedback (captured home)
    latest_pos = [None]  # continuously updated feedback

    def fb_cb(msg):
        if len(msg.position) >= 1:
            latest_pos[0] = list(msg.position)
            if current_pos[0] is None:
                current_pos[0] = list(msg.position)

    node.create_subscription(JointState, FEEDBACK_TOPIC, fb_cb, 10)
    pub = node.create_publisher(JointState, CMD_TOPIC, 10)

    if not skip_prompt:
        print("WARNING: Arm will move. Keep clear.")
        if input("Type 'yes' to proceed: ").strip().lower() != "yes":
            print("Aborted.")
            node.destroy_node()
            return False

    print("Waiting for DDS peer discovery...")
    deadline = time.time() + DISCOVERY_WAIT
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"Waiting for {FEEDBACK_TOPIC}...")
    deadline = time.time() + 5.0
    while current_pos[0] is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if current_pos[0] is None:
        print(f"FAIL: No feedback from {FEEDBACK_TOPIC} within 5s")
        node.destroy_node()
        return False

    home = list(current_pos[0])
    dof = len(home)
    if dof != ARM_DOF:
        print(f"NOTE: arm has {dof} joints; r1lite_config.ARM_DOF says {ARM_DOF}, fix the config!")
    print(f"Home position ({dof} joints): {[round(p, 3) for p in home]}")

    def send_positions(positions, label):
        print(f"Moving to {label}: {[round(p, 3) for p in positions]}")
        deadline = time.time() + MOVE_DURATION
        while time.time() < deadline:
            cmd = JointState()
            cmd.header.stamp = node.get_clock().now().to_msg()
            cmd.name = [""]
            cmd.position = list(positions)
            cmd.velocity = [VELOCITY] * dof
            cmd.effort = [0.0]
            pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.02)

    target = list(home)
    target[JOINT_INDEX] += DELTA
    send_positions(target, f"wrist roll (joint {dof}) +{DELTA} rad")

    # Did the wrist actually track the target?
    moved = (
        latest_pos[0] is not None and abs(latest_pos[0][JOINT_INDEX] - target[JOINT_INDEX]) < 0.05
    )
    if latest_pos[0] is not None:
        print(
            f"After move: wrist at {latest_pos[0][JOINT_INDEX]:+.3f} (target {target[JOINT_INDEX]:+.3f})"
        )

    send_positions(home, "home")

    returned = (
        latest_pos[0] is not None and abs(latest_pos[0][JOINT_INDEX] - home[JOINT_INDEX]) < 0.05
    )
    if latest_pos[0] is not None:
        print(
            f"After return: wrist at {latest_pos[0][JOINT_INDEX]:+.3f} (home {home[JOINT_INDEX]:+.3f})"
        )

    if moved and returned:
        print("\nPASS: Wrist tracked target and returned home (verified via feedback).")
    elif not moved:
        print(
            "\nFAIL: Wrist did not track the target, motors enabled? "
            "(an e-stop latched at boot inhibits all motors for the session)"
        )
    else:
        print("\nFAIL: Wrist moved but did not return home, check feedback above.")
    node.destroy_node()
    return moved and returned


if __name__ == "__main__":
    rclpy.init()
    result = main()
    rclpy.shutdown()
    exit(0 if result else 1)
