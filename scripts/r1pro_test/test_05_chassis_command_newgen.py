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
Test 5: Chassis Command — NEW-GEN R1 Pro (firmware V2.3.0)

Supersedes test_03 for the new-gen robot. test_03 publishes a plain Twist to
/cmd_vel and trusts a robot-side gatekeeper to FORWARD it into the chassis
controller — that was the old-gen (V2.2.1) contract. The new-gen V2.3.0
gatekeeper does NOT forward /cmd_vel; it only HOLDS the gate preconditions
(mode=5 on /controller_unused, brake_mode=False, nonzero acc_limit). The
velocity command must go DIRECTLY to /motion_target/target_speed_chassis as a
TwistStamped at BEST_EFFORT — exactly what chassis_poke.py and
R1ProConnection do. This test mirrors that path.

Prerequisites (new-gen ~/galaxea-dimos tree booted):
  - chassis_gatekeeper started by the tree (holds mode/brake/acc gates)
  - ROS_DOMAIN_ID=1

Run with:
    source ~/galaxea-dimos/install/setup.bash
    export ROS_DOMAIN_ID=1
    python3 scripts/r1pro_test/test_05_chassis_command_newgen.py

Pass condition: /motion_target/target_speed_chassis has a subscriber AND the
robot's /motion_control/chassis_speed feedback goes nonzero while we command
motion (proves the controller accepted the command — not just "did it look
like it moved").
"""

import os
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

VELOCITY = 0.12  # m/s forward — small and safe
DURATION = 3.0  # seconds of commanded motion
PUBLISH_HZ = 50  # chassis_control_node runs its loop at 50 Hz
WARMUP_S = 1.0  # latch brake-release + acc_limit at zero velocity first
DISCOVERY_WAIT = 4.0
ACC_LIMIT = (2.5, 1.0, 1.0)  # vendor max ax, ay, alpha
FB_MOTION_THRESH = 0.02  # |vx| on chassis_speed feedback that counts as "moving"

CMD_TOPIC = "/motion_target/target_speed_chassis"
ACC_TOPIC = "/motion_target/chassis_acc_limit"
BRK_TOPIC = "/motion_target/brake_mode"
FB_TOPIC = "/motion_control/chassis_speed"

# BEST_EFFORT / VOLATILE — the profile chassis_control_node actually runs.
# RELIABLE (test_03) does not match the runtime contract.
QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


def _twist(node, x, y, z):
    m = TwistStamped()
    m.header.stamp = node.get_clock().now().to_msg()
    m.twist.linear.x = float(x)
    m.twist.linear.y = float(y)
    m.twist.angular.z = float(z)
    return m


def main() -> bool:
    node = rclpy.create_node("dimos_chassis_test_newgen")
    cmd_pub = node.create_publisher(TwistStamped, CMD_TOPIC, QOS)
    acc_pub = node.create_publisher(TwistStamped, ACC_TOPIC, QOS)
    brk_pub = node.create_publisher(Bool, BRK_TOPIC, QOS)

    fb = {"count": 0, "max_vx": 0.0, "last": None}

    def on_fb(msg):
        fb["count"] += 1
        fb["last"] = msg
        fb["max_vx"] = max(fb["max_vx"], abs(msg.twist.linear.x))

    node.create_subscription(TwistStamped, FB_TOPIC, on_fb, QOS)

    # Discovery FIRST — BEST_EFFORT samples sent before the subscriber is
    # discovered are silently dropped, so never rely on a one-shot publish.
    print("Waiting for DDS discovery...")
    deadline = time.time() + DISCOVERY_WAIT
    while time.time() < deadline:
        if node.count_publishers(FB_TOPIC) > 0 and node.count_subscribers(CMD_TOPIC) > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    cmd_subs = node.count_subscribers(CMD_TOPIC)
    fb_pubs = node.count_publishers(FB_TOPIC)
    print(f"  subscribers on {CMD_TOPIC}: {cmd_subs}  (need >=1 = chassis controller)")
    print(f"  publishers on {FB_TOPIC}: {fb_pubs}  (>=1 means the controller is alive)")
    if cmd_subs == 0:
        print(
            f"\nFAIL: nothing is subscribed to {CMD_TOPIC}. Is the ~/galaxea-dimos "
            f"tree booted (gatekeeper + chassis_control_node) and ROS_DOMAIN_ID=1?"
        )
        node.destroy_node()
        return False

    period = 1.0 / PUBLISH_HZ
    ax, ay, az = ACC_LIMIT

    def preconditions():
        # chassis_control_node must CONTINUOUSLY see brake-release + acc_limit;
        # one-shot publishes get lost. (The gatekeeper also holds these — this
        # is belt-and-suspenders so the test stands alone.)
        brk_pub.publish(Bool(data=False))
        acc_pub.publish(_twist(node, ax, ay, az))

    # Warm-up: establish unbraked + acc-limited state at zero velocity.
    print(f"Warmup {WARMUP_S}s: brake-release + acc_limit at zero velocity...")
    t0 = time.time()
    while time.time() - t0 < WARMUP_S:
        preconditions()
        cmd_pub.publish(_twist(node, 0.0, 0.0, 0.0))
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)

    fb_before = fb["max_vx"]
    print(f"Commanding vx={VELOCITY} m/s for {DURATION}s (Ctrl+C stops)...")
    t0 = time.time()
    try:
        while time.time() - t0 < DURATION and rclpy.ok():
            preconditions()
            cmd_pub.publish(_twist(node, VELOCITY, 0.0, 0.0))
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[stop] Ctrl+C")
    finally:
        # Always stop.
        for _ in range(10):
            cmd_pub.publish(_twist(node, 0.0, 0.0, 0.0))
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.02)
        print("[stop] zero velocity sent.")

    moved = fb["max_vx"] >= FB_MOTION_THRESH and fb["max_vx"] > fb_before
    print(f"\n[feedback] {fb['count']} msgs on {FB_TOPIC}, peak |vx|={fb['max_vx']:.3f} m/s")
    node.destroy_node()

    if fb["count"] == 0:
        print("FAIL: no chassis_speed feedback — Gate 1/discovery/QoS suspect.")
        return False
    if not moved:
        print(
            f"FAIL: controller feedback never exceeded {FB_MOTION_THRESH} m/s — "
            f"command was accepted on the wire but the chassis didn't move "
            f"(check gatekeeper: mode/brake/acc gates)."
        )
        return False
    print("PASS: chassis moved under direct target_speed_chassis command.")
    return True


if __name__ == "__main__":
    if "ROS_DOMAIN_ID" not in os.environ:
        os.environ["ROS_DOMAIN_ID"] = "1"
        print("Set ROS_DOMAIN_ID=1 (new-gen R1 Pro)")
    else:
        print(f"Using ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}")

    print("!!! SAFETY CHECK !!!")
    print("- Robot on flat ground with clear space ahead?")
    print("- Hand on e-stop?")
    print("- ~/galaxea-dimos tree booted (gatekeeper running)?")
    response = input("\nType 'yes' to proceed: ").strip().lower()
    if response != "yes":
        print("Aborted.")
        raise SystemExit(0)

    rclpy.init()
    try:
        ok = main()
    finally:
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)
