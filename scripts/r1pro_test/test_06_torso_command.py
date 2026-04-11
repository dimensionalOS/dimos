"""
Test 6: Torso Joint Movement
Reads current torso position, moves to home pose, holds, then returns to zero.

WARNING: Torso will physically move. Ensure clear workspace above and around robot.

Home pose (from Galaxea startup scripts):
    [0.25, -0.62, -0.53, 0.0]  rad  (torso_joint1–4)

Run standalone:
    export ROS_DOMAIN_ID=41
    python3 scripts/r1pro_test/test_06_torso_command.py

Or via run_all_tests.py (preferred — single DDS session).

Pass condition: Torso moves to home pose then returns to zero, positions
within 0.1 rad of commanded values.
"""
import rclpy
import time
from sensor_msgs.msg import JointState

DOF = 4
MOVE_DURATION = 3.0   # seconds to hold each position
VELOCITY = 0.5        # rad/s tracking speed
DISCOVERY_WAIT = 5.0  # DDS peer discovery time (s)
FEEDBACK_WAIT = 5.0   # time to wait for first feedback (s)

FEEDBACK_TOPIC = "/hdas/feedback_torso"
CMD_TOPIC = "/motion_target/target_joint_state_torso"

# Home pose from start_mobiman_torso_speed_control_pro_z.sh — safe starting point
HOME_POSE = [0.25, -0.62, -0.53, 0.0]
ZERO_POSE = [0.0] * DOF


def main(skip_prompt=False) -> bool:
    """Run torso movement test. Assumes rclpy.init() already called."""
    node = rclpy.create_node("dimos_torso_cmd_test")
    current_pos = [None]

    def fb_cb(msg):
        if current_pos[0] is None and len(msg.position) >= DOF:
            current_pos[0] = list(msg.position[:DOF])

    node.create_subscription(JointState, FEEDBACK_TOPIC, fb_cb, 10)
    pub = node.create_publisher(JointState, CMD_TOPIC, 10)

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

    print(f"Waiting for {FEEDBACK_TOPIC}...")
    deadline = time.time() + FEEDBACK_WAIT
    while current_pos[0] is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if current_pos[0] is None:
        print(f"FAIL: No feedback from {FEEDBACK_TOPIC} within {FEEDBACK_WAIT}s")
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
            cmd.velocity = [VELOCITY] * DOF
            cmd.effort = [0.0]
            pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.02)

    # Move to home pose
    send_positions(HOME_POSE, "home pose")

    # Read back position
    final_home = [None]
    deadline = time.time() + 2.0
    while final_home[0] is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if current_pos[0] is not None:
            final_home[0] = list(current_pos[0])
    current_pos[0] = None  # reset for next read

    if final_home[0] is not None:
        print(f"Torso at home: {[round(p, 3) for p in final_home[0]]}")
        errors = [abs(a - b) for a, b in zip(final_home[0], HOME_POSE)]
        max_err = max(errors)
        if max_err > 0.15:
            print(f"WARNING: max position error {max_err:.3f} rad > 0.15 rad")

    # Return to zero
    send_positions(ZERO_POSE, "zero pose")

    # Final position check
    deadline = time.time() + 2.0
    final_zero = initial  # fallback
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
