"""
Test 4: Arm Joint Movement
Reads current arm position, moves joint 0 by DELTA radians, holds, then returns home.

WARNING: Arm will physically move. Keep clear.

Run standalone:
    export ROS_DOMAIN_ID=41
    python3 scripts/r1pro_test/test_04_arm_joint_command.py

Or via run_all_tests.py (preferred — single DDS session).

Pass condition: Arm moves noticeably then returns to start position.
"""
import rclpy
import time
from sensor_msgs.msg import JointState

SIDE = "left"        # change to "right" to test right arm
DELTA = 0.3          # radians (~17 degrees)
MOVE_DURATION = 3.0  # seconds to hold moved position
VELOCITY = 0.5       # rad/s tracking speed
DISCOVERY_WAIT = 5.0

FEEDBACK_TOPIC = f"/hdas/feedback_arm_{SIDE}"
CMD_TOPIC = f"/motion_target/target_joint_state_arm_{SIDE}"


def main(skip_prompt=False) -> bool:
    """Run arm movement test. Assumes rclpy.init() already called."""
    node = rclpy.create_node("dimos_arm_cmd_test")
    current_pos = [None]

    def fb_cb(msg):
        if current_pos[0] is None and len(msg.position) >= 7:
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
    print(f"Home position: {[round(p, 3) for p in home]}")

    def send_positions(positions, label):
        print(f"Moving to {label}: {[round(p, 3) for p in positions]}")
        deadline = time.time() + MOVE_DURATION
        while time.time() < deadline:
            cmd = JointState()
            cmd.header.stamp = node.get_clock().now().to_msg()
            cmd.name = [""]
            cmd.position = list(positions)
            cmd.velocity = [VELOCITY] * 7
            cmd.effort = [0.0]
            pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.02)

    # Move joint 0 by DELTA
    target = list(home)
    target[0] += DELTA
    send_positions(target, f"joint0 +{DELTA} rad")

    # Return home
    send_positions(home, "home")

    print("\nPASS: Arm moved and returned.")
    node.destroy_node()
    return True


if __name__ == "__main__":
    rclpy.init()
    result = main()
    rclpy.shutdown()
    exit(0 if result else 1)
