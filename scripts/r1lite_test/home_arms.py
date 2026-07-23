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

"""Slowly return the arms to the folded home pose (all joints zero).

Reads current feedback, then streams a joint-space linear interpolation to
zeros at a conservative speed, one arm at a time (left first). Joint zero is
the folded tabletop pose, so the motion is always inward toward the body.

STOP THE DIMOS BLUEPRINT FIRST (Ctrl-C it): while a coordinator runs, its
servo task streams position targets continuously and the two streams fight.

Ctrl-C mid-run is always safe: the tracker is a dead-man follower, the arm
simply stays where it is.

Run on the robot (inside the dev container, ROS sourced), e-stop in hand:
    python3 scripts/r1lite_test/home_arms.py            # both arms
    python3 scripts/r1lite_test/home_arms.py --arm left # one arm
"""

import argparse
import math
from pathlib import Path
import sys
import time

import rclpy
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
from r1lite_config import ARM_DOF, CMD_ARM, FEEDBACK_ARM

SPEED = 0.12  # rad/s along the slowest-joint profile, deliberately stately
STREAM_HZ = 50.0
DISCOVERY_WAIT = 5.0
FEEDBACK_WAIT = 5.0
SETTLE_TOLERANCE = 0.05  # rad per joint to declare home reached
HOME = [0.0] * ARM_DOF


def home_arm(side: str) -> bool:
    node = rclpy.create_node(f"dimos_home_arm_{side}")
    feedback_topic = FEEDBACK_ARM.format(side=side)
    cmd_topic = CMD_ARM.format(side=side)
    latest = [None]

    def fb_cb(msg):
        if msg.position:
            latest[0] = list(msg.position)

    node.create_subscription(JointState, feedback_topic, fb_cb, 10)
    pub = node.create_publisher(JointState, cmd_topic, 10)

    print(f"[{side}] waiting for DDS discovery...")
    deadline = time.time() + DISCOVERY_WAIT
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    deadline = time.time() + FEEDBACK_WAIT
    while latest[0] is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if latest[0] is None:
        print(f"[{side}] FAIL: no feedback on {feedback_topic}")
        node.destroy_node()
        return False

    start = list(latest[0])
    dof = len(start)
    if dof != ARM_DOF:
        print(f"[{side}] FAIL: feedback has {dof} joints, expected {ARM_DOF}")
        node.destroy_node()
        return False

    worst = max(abs(p) for p in start)
    if worst < SETTLE_TOLERANCE:
        print(f"[{side}] already home ({[round(p, 3) for p in start]})")
        node.destroy_node()
        return True

    duration = worst / SPEED
    print(
        f"[{side}] homing from {[round(p, 3) for p in start]} "
        f"over {duration:.1f} s at {SPEED} rad/s"
    )

    period = 1.0 / STREAM_HZ
    t0 = time.time()
    try:
        while True:
            elapsed = time.time() - t0
            alpha = min(elapsed / duration, 1.0)
            target = [(1.0 - alpha) * p for p in start]
            cmd = JointState()
            cmd.header.stamp = node.get_clock().now().to_msg()
            cmd.name = [""]
            cmd.position = target
            cmd.velocity = [SPEED] * dof
            cmd.effort = [0.0]
            pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=period)
            if alpha >= 1.0:
                # Hold at zero briefly so the tracker settles there.
                if elapsed > duration + 1.0:
                    break
    finally:
        node.destroy_node()

    final = latest[0]
    residual = max(abs(p) for p in final) if final else math.inf
    if residual < SETTLE_TOLERANCE:
        print(f"[{side}] HOME ({[round(p, 3) for p in final]})")
        return True
    print(f"[{side}] finished streaming but residual {residual:.3f} rad; check feedback")
    return False


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", choices=["left", "right", "both"], default="both")
    parser.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    args = parser.parse_args()

    print("WARNING: arms will move slowly to the folded home pose.")
    print("Stop any running dimos blueprint first; keep the e-stop in hand.")
    if not args.yes and input("Type 'yes' to proceed: ").strip().lower() != "yes":
        print("Aborted.")
        return

    rclpy.init()
    try:
        sides = ["left", "right"] if args.arm == "both" else [args.arm]
        ok = all(home_arm(side) for side in sides)
    finally:
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
