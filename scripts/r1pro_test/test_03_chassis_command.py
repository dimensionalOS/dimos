"""
Test 3: Chassis Command
Publish a small forward velocity command for ~0.5 seconds then stop.

!!! SAFETY: Robot must be on flat ground with room to move. Have hand on e-stop. !!!

Run with:
    export ROS_DOMAIN_ID=41
    python3 scripts/r1pro_test/test_03_chassis_command.py

Pass condition: Robot moves forward briefly (~5cm) and stops cleanly.
"""
import rclpy
import time
from geometry_msgs.msg import TwistStamped

VELOCITY = 0.1   # m/s forward — small and safe
DURATION = 0.5   # seconds of movement

print("!!! SAFETY CHECK !!!")
print("- Robot on flat ground with clear space ahead?")
print("- Hand on e-stop?")
response = input("Type 'yes' to proceed: ").strip().lower()
if response != "yes":
    print("Aborted.")
    exit(0)

rclpy.init()
node = rclpy.create_node("dimos_chassis_test")
pub = node.create_publisher(TwistStamped, "/motion_target/target_speed_chassis", 10)

# Wait for publisher to connect to robot's subscriber
print("Connecting to robot...")
time.sleep(2.0)

# Send move command
print(f"Sending vx={VELOCITY} m/s for {DURATION}s...")
deadline = time.time() + DURATION
while time.time() < deadline:
    msg = TwistStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.twist.linear.x = VELOCITY
    pub.publish(msg)
    time.sleep(0.02)

# Send stop
print("Stopping...")
for _ in range(10):
    stop = TwistStamped()
    stop.header.stamp = node.get_clock().now().to_msg()
    pub.publish(stop)
    time.sleep(0.02)

print("\nPASS: Command sent. Did the robot move forward and stop?")

node.destroy_node()
rclpy.shutdown()
