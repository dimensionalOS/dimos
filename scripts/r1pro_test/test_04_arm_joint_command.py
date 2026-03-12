"""
Test 4: Arm Joint Command (Safe No-Op)
Read the current arm position and command it back to the same position.
This is a safe test — no movement expected.

Run with:
    export ROS_DOMAIN_ID=41
    python3 scripts/r1pro_test/test_04_arm_joint_command.py

Pass condition: Prints current position, arm stays still, no errors.
"""
import rclpy
import time
from sensor_msgs.msg import JointState

SIDE = "left"   # change to "right" to test right arm

rclpy.init()
node = rclpy.create_node("dimos_arm_cmd_test")
current_pos = [None]

def fb_cb(msg):
    if current_pos[0] is None:
        current_pos[0] = list(msg.position)

node.create_subscription(JointState, f"/hdas/feedback_arm_{SIDE}", fb_cb, 10)
pub = node.create_publisher(JointState, f"/motion_target/target_joint_state_arm_{SIDE}", 10)

# Wait for DDS peer discovery before starting timeout
print("Waiting for DDS peer discovery...")
time.sleep(5.0)

# Wait for feedback
print(f"Waiting for /hdas/feedback_arm_{SIDE}...")
deadline = time.time() + 5.0
while current_pos[0] is None and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

if current_pos[0] is None:
    print(f"FAIL: No feedback from /hdas/feedback_arm_{SIDE} within 5s")
    node.destroy_node()
    rclpy.shutdown()
    exit(1)

print(f"Current {SIDE} arm positions: {[round(p, 4) for p in current_pos[0]]}")
print(f"Sending hold command (same position)...")

# Send hold command: same position, low velocity limit
cmd = JointState()
cmd.header.stamp = node.get_clock().now().to_msg()
cmd.name = [f"joint{i+1}" for i in range(7)]
cmd.position = list(current_pos[0])
cmd.velocity = [0.5] * 7   # slow max velocity for safety

# Publish a few times
for i in range(5):
    cmd.header.stamp = node.get_clock().now().to_msg()
    pub.publish(cmd)
    time.sleep(0.1)
    rclpy.spin_once(node, timeout_sec=0.0)

print(f"\nPASS: Hold command sent. Arm should not have moved.")
print(f"      Verify by checking feedback_arm_{SIDE} is unchanged.")

node.destroy_node()
rclpy.shutdown()
