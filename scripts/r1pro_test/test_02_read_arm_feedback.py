"""
Test 2: Read Arm Feedback
Subscribe to left arm joint feedback and print 5 messages.

Run with:
    export ROS_DOMAIN_ID=41
    python3 scripts/r1pro_test/test_02_read_arm_feedback.py

Pass condition: Prints 5 joint position arrays (7 values each).
"""
import rclpy
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("dimos_arm_reader")
count = [0]
MAX = 5

def cb(msg):
    count[0] += 1
    print(f"[{count[0]}/{MAX}] positions: {[round(p, 4) for p in msg.position]}")
    print(f"       velocities: {[round(v, 4) for v in msg.velocity]}")
    print(f"       efforts:    {[round(e, 4) for e in msg.effort]}")

node.create_subscription(JointState, "/hdas/feedback_arm_left", cb, 10)

print("Waiting for DDS peer discovery...")
import time
time.sleep(5.0)

print("Waiting for /hdas/feedback_arm_left messages (5 second timeout)...")
deadline = time.time() + 5.0
while count[0] < MAX and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

if count[0] >= MAX:
    print(f"\nPASS: Received {count[0]} messages")
else:
    print(f"\nFAIL: Only received {count[0]}/{MAX} messages — check mobiman is running on robot")

node.destroy_node()
rclpy.shutdown()
