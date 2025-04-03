import os
import sys
# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
import os
import time
import math

# Initialize robot
robot = UnitreeGo2(ip=os.getenv('ROBOT_IP'),
                  ros_control=UnitreeROSControl(),
                  skills=MyUnitreeSkills())

# Helper function to send pose commands continuously for a duration
def send_pose_for_duration(roll, pitch, yaw, duration, hz=10):
    """Send the same pose command repeatedly at specified frequency for the given duration"""
    start_time = time.time()
    while time.time() - start_time < duration:
        robot.pose_command(roll=roll, pitch=pitch, yaw=yaw)
        time.sleep(1.0/hz)  # Sleep to achieve the desired frequency

# Test pose commands

# First, make sure the robot is in a stable position
print("Setting default pose...")
send_pose_for_duration(0.0, 0.0, 0.0, 1)

# Test roll angle (lean left/right)
print("Testing roll angle - lean right...")
send_pose_for_duration(0.5, 0.0, 0.0, 1.5)  # Lean right

print("Testing roll angle - lean left...")
send_pose_for_duration(-0.5, 0.0, 0.0, 1.5)  # Lean left

# Test pitch angle (lean forward/backward)
print("Testing pitch angle - lean forward...")
send_pose_for_duration(0.0, 0.5, 0.0, 1.5)  # Lean forward

print("Testing pitch angle - lean backward...")
send_pose_for_duration(0.0, -0.5, 0.0, 1.5)  # Lean backward

# Test yaw angle (rotate body without moving feet)
print("Testing yaw angle - rotate clockwise...")
send_pose_for_duration(0.0, 0.0, 0.5, 1.5)  # Rotate body clockwise

print("Testing yaw angle - rotate counterclockwise...")
send_pose_for_duration(0.0, 0.0, -0.5, 1.5)  # Rotate body counterclockwise

# Reset to default pose
print("Resetting to default pose...")
send_pose_for_duration(0.0, 0.0, 0.0, 2)

print("Pose command test completed")

# Keep the program running (optional)
print("Press Ctrl+C to exit")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Test terminated by user")