"""
Test 2: Read Arm Feedback
Subscribe to left arm joint feedback and print 5 messages.

Run standalone:
    export ROS_DOMAIN_ID=41
    python3 scripts/r1pro_test/test_02_read_arm_feedback.py

Or via run_all_tests.py (preferred — single DDS session).

Pass condition: Prints 5 joint position arrays (7 values each).
"""
import rclpy
import time
from sensor_msgs.msg import JointState

MAX = 5
DISCOVERY_WAIT = 5.0
RECEIVE_TIMEOUT = 5.0


def main() -> bool:
    """Run arm feedback test. Assumes rclpy.init() already called."""
    node = rclpy.create_node("dimos_arm_reader")
    count = [0]

    def cb(msg):
        count[0] += 1
        print(f"[{count[0]}/{MAX}] positions: {[round(p, 4) for p in msg.position]}")
        print(f"       velocities: {[round(v, 4) for v in msg.velocity]}")
        print(f"       efforts:    {[round(e, 4) for e in msg.effort]}")

    node.create_subscription(JointState, "/hdas/feedback_arm_left", cb, 10)

    print("Waiting for DDS peer discovery...")
    deadline = time.time() + DISCOVERY_WAIT
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("Waiting for /hdas/feedback_arm_left messages...")
    deadline = time.time() + RECEIVE_TIMEOUT
    while count[0] < MAX and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    passed = count[0] >= MAX
    if passed:
        print(f"\nPASS: Received {count[0]} messages")
    else:
        print(f"\nFAIL: Only received {count[0]}/{MAX} messages — check mobiman is running")

    node.destroy_node()
    return passed


if __name__ == "__main__":
    rclpy.init()
    result = main()
    rclpy.shutdown()
    exit(0 if result else 1)
