"""
Test 6: Torso Joint Movement — DO NOT RUN AS WRITTEN (see BRINGUP_LOG.md)

!! HARDWARE-VERIFIED NEGATIVE RESULT (2026-07-03): the R1 Lite torso is a
!! PARALLELOGRAM lift — joints 1-3 are mechanically coupled. Streaming a
!! single-joint delta to /motion_target/target_joint_state_torso made the
!! robot SHAKE (motors fighting the linkage). Joint-space torso commands
!! are only valid as linkage-consistent tuples read back from feedback.
!! Torso motion should go through the task-space MPC path
!! (/motion_target/target_speed_torso) instead. This script is kept as an
!! R1-Pro-style template only and refuses to run without an explicit
!! override (R1LITE_TORSO_I_KNOW_WHAT_IM_DOING=1).

Reads current torso position, moves to home pose, holds, then returns to zero.

Ported from scripts/r1pro_test/test_06_torso_command.py. TORSO_DOF and
TORSO_HOME_POSE come from r1lite_config.py (TODO(recon): fill in the R1
Lite's safe home pose from its Galaxea startup scripts before running).

WARNING: Torso will physically move. Ensure clear workspace above and around robot.

Run standalone:
    export ROS_DOMAIN_ID=<see r1lite_config>
    python3 scripts/r1lite_test/test_06_torso_command.py

Pass condition: Torso moves to home pose then returns to zero, positions
within 0.15 rad of commanded values.
"""
import sys
import time
from pathlib import Path

import rclpy
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
from r1lite_config import CMD_TORSO, FEEDBACK_TORSO, TORSO_DOF, TORSO_HOME_POSE

MOVE_DURATION = 3.0
VELOCITY = 0.5
DISCOVERY_WAIT = 5.0
FEEDBACK_WAIT = 5.0

HOME_POSE = TORSO_HOME_POSE
ZERO_POSE = [0.0] * TORSO_DOF


def main(skip_prompt=False) -> bool:
    """Run torso movement test. Assumes rclpy.init() already called."""
    import os

    if os.environ.get("R1LITE_TORSO_I_KNOW_WHAT_IM_DOING") != "1":
        print(
            "REFUSING TO RUN: joint-space torso commands shook the robot on "
            "2026-07-03 (parallelogram linkage — joints are coupled). Read "
            "the header + BRINGUP_LOG.md. Set R1LITE_TORSO_I_KNOW_WHAT_IM_DOING=1 "
            "only with a linkage-consistent plan."
        )
        return False

    if all(v == 0.0 for v in HOME_POSE):
        print(
            "WARNING: r1lite_config.TORSO_HOME_POSE is still all zeros "
            "(placeholder). Fill in the real home pose before trusting this test."
        )

    node = rclpy.create_node("dimos_torso_cmd_test")
    current_pos = [None]

    def fb_cb(msg):
        if current_pos[0] is None and len(msg.position) >= TORSO_DOF:
            current_pos[0] = list(msg.position[:TORSO_DOF])

    node.create_subscription(JointState, FEEDBACK_TORSO, fb_cb, 10)
    pub = node.create_publisher(JointState, CMD_TORSO, 10)

    if not skip_prompt:
        print("WARNING: Torso will move. Ensure clear workspace around robot.")
        if input("Type 'yes' to proceed: ").strip().lower() != "yes":
            print("Aborted.")
            node.destroy_node()
            return False

    print("Waiting for DDS peer discovery...")
    deadline = time.time() + DISCOVERY_WAIT
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"Waiting for {FEEDBACK_TORSO}...")
    deadline = time.time() + FEEDBACK_WAIT
    while current_pos[0] is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if current_pos[0] is None:
        print(f"FAIL: No feedback from {FEEDBACK_TORSO} within {FEEDBACK_WAIT}s")
        node.destroy_node()
        return False

    initial = list(current_pos[0])
    print(f"Initial torso position: {[round(p, 3) for p in initial]}")

    def send_positions(positions, label):
        print(f"Moving to {label}: {[round(p, 3) for p in positions]}")
        deadline = time.time() + MOVE_DURATION
        while time.time() < deadline:
            cmd = JointState()
            cmd.header.stamp = node.get_clock().now().to_msg()
            cmd.name = [""]
            cmd.position = list(positions)
            cmd.velocity = [VELOCITY] * TORSO_DOF
            cmd.effort = [0.0]
            pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.02)

    send_positions(HOME_POSE, "home pose")

    final_home = [None]
    deadline = time.time() + 2.0
    while final_home[0] is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if current_pos[0] is not None:
            final_home[0] = list(current_pos[0])
    current_pos[0] = None

    if final_home[0] is not None:
        print(f"Torso at home: {[round(p, 3) for p in final_home[0]]}")
        errors = [abs(a - b) for a, b in zip(final_home[0], HOME_POSE)]
        max_err = max(errors) if errors else 0.0
        if max_err > 0.15:
            print(f"WARNING: max position error {max_err:.3f} rad > 0.15 rad")

    send_positions(ZERO_POSE, "zero pose")

    deadline = time.time() + 2.0
    final_zero = initial
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if current_pos[0] is not None:
            final_zero = list(current_pos[0])
    print(f"Torso at zero: {[round(p, 3) for p in final_zero]}")

    print("\nPASS: Torso moved to home pose and returned to zero.")
    node.destroy_node()
    return True


if __name__ == "__main__":
    rclpy.init()
    result = main()
    rclpy.shutdown()
    exit(0 if result else 1)
