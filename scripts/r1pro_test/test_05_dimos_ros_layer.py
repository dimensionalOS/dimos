"""
Test 5: DiMOS ROS Layer
Test using DiMOS's existing RawROS infrastructure (from jeff/fix/rosnav3 branch).
This validates the full DiMOS transport stack works with the R1 Pro.

Prerequisites:
    - Must be on jeff/fix/rosnav3 branch (or have rospubsub.py available)
    - ROS2 Humble installed with rclpy

Run with:
    export ROS_DOMAIN_ID=41
    python3 scripts/r1pro_test/test_05_dimos_ros_layer.py

Pass condition: DiMOS ROS layer receives arm feedback messages.
"""
import os
import time

# Ensure ROS_DOMAIN_ID is set (can also be set in environment before running)
if "ROS_DOMAIN_ID" not in os.environ:
    os.environ["ROS_DOMAIN_ID"] = "41"
    print(f"Set ROS_DOMAIN_ID=41")
else:
    print(f"Using ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}")

try:
    from dimos.protocol.pubsub.impl.rospubsub import RawROS
    from sensor_msgs.msg import JointState
except ImportError as e:
    print(f"FAIL: Could not import DiMOS ROS layer: {e}")
    print("Make sure you are on jeff/fix/rosnav3 branch and ROS2 is installed.")
    exit(1)

received = []

def on_msg(msg):
    received.append(list(msg.position))
    print(f"[{len(received)}] DiMOS ROS layer received positions: {[round(p, 4) for p in msg.position]}")

print("Starting DiMOS RawROS node...")
ros = RawROS()
ros.start()
time.sleep(0.5)

print("Subscribing to /hdas/feedback_arm_left via DiMOS ROS layer...")
ros.subscribe("/hdas/feedback_arm_left", JointState, on_msg)

print("Waiting 5 seconds for messages...")
time.sleep(5.0)

ros.stop()

if len(received) >= 3:
    print(f"\nPASS: DiMOS ROS layer received {len(received)} messages from R1 Pro")
else:
    print(f"\nFAIL: Only received {len(received)} messages — check robot and ROS_DOMAIN_ID")
