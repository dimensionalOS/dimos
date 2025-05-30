import math

import pytest

from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.utils.testing import SensorReplay


def quaternion_to_euler_z(x, y, z, w):
    """Convert quaternion to Euler angles and return the Z (yaw) angle in radians."""
    # Calculate yaw (rotation around z-axis)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def test_calculate_rotation_degrees():
    """Calculate total degrees of rotation change from the odometry stream."""
    messages = list(
        SensorReplay(
            name="raw_odometry_rotate_walk",
            autocast=lambda x: x,  # Keep raw messages
        ).iterate()
    )

    if not messages:
        print("No messages found in the replay data")
        return

    print(f"Found {len(messages)} odometry messages")

    # Extract yaw angles from quaternions
    yaw_angles = []
    timestamps = []

    for msg in messages:
        orientation = msg["data"]["pose"]["orientation"]
        x, y, z, w = orientation["x"], orientation["y"], orientation["z"], orientation["w"]

        # Convert quaternion to yaw angle (rotation around z-axis)
        yaw = quaternion_to_euler_z(x, y, z, w)
        yaw_angles.append(yaw)

        # Extract timestamp
        stamp = msg["data"]["header"]["stamp"]
        timestamp = stamp["sec"] + stamp["nanosec"] / 1e9
        timestamps.append(timestamp)

    if len(yaw_angles) < 2:
        print("Need at least 2 messages to calculate rotation change")
        return

    # Calculate cumulative rotation change
    total_rotation_radians = 0
    rotation_changes = []

    for i in range(1, len(yaw_angles)):
        # Calculate the difference, handling wrap-around at ±π
        diff = yaw_angles[i] - yaw_angles[i - 1]

        # Normalize the difference to [-π, π]
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi

        rotation_changes.append(diff)
        total_rotation_radians += diff

    # Convert to degrees
    total_rotation_degrees = math.degrees(total_rotation_radians)

    # Calculate time span
    time_span = timestamps[-1] - timestamps[0]

    # Statistics
    rotation_changes_degrees = [math.degrees(r) for r in rotation_changes]
    max_rotation_rate = (
        max(abs(r) for r in rotation_changes_degrees) if rotation_changes_degrees else 0
    )
    avg_rotation_rate = (
        sum(abs(r) for r in rotation_changes_degrees) / len(rotation_changes_degrees)
        if rotation_changes_degrees
        else 0
    )

    print("\n=== ROTATION ANALYSIS ===")
    print(f"Total rotation change: {total_rotation_degrees:.2f} degrees")
    print(f"Time span: {time_span:.2f} seconds")
    print(f"Average rotation rate: {avg_rotation_rate:.2f} degrees per step")
    print(f"Maximum rotation rate: {max_rotation_rate:.2f} degrees per step")
    print(f"Number of rotation steps: {len(rotation_changes)}")

    if abs(total_rotation_degrees) > 360:
        full_rotations = abs(total_rotation_degrees) // 360
        remaining_degrees = abs(total_rotation_degrees) % 360
        direction = "clockwise" if total_rotation_degrees < 0 else "counter-clockwise"
        print(
            f"That's {full_rotations:.0f} full rotation(s) + {remaining_degrees:.2f}° {direction}!"
        )

    # Show first few and last few rotation changes for inspection
    print(f"\nFirst 5 rotation changes (degrees): {rotation_changes_degrees[:5]}")
    print(f"Last 5 rotation changes (degrees): {rotation_changes_degrees[-5:]}")

    return total_rotation_degrees


@pytest.mark.tool
def test_store_odometry_stream():
    import os
    import threading
    import time

    from dotenv import load_dotenv

    from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2
    from dimos.utils.testing import SensorStorage

    load_dotenv()

    robot = UnitreeGo2(ip=os.getenv("ROBOT_IP"), mode="ai")
    robot.standup()

    def streamlogger(element):
        print(element)
        return element

    SensorStorage("raw_odometry_rotate_walk").save_stream(robot.raw_odom_stream()).subscribe(print)

    shutdown_event = threading.Event()
    print("Robot is running. Press Ctrl+C to stop.")

    try:
        # Keep the main thread alive while the robot loop runs in the background
        while not shutdown_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Shutting down...")
    finally:
        print("Stopping robot...")
        robot.liedown()
        print("Robot stopped.")


def test_odometry_stream():
    counter = 0
    for message in SensorReplay(
        name="raw_odometry_rotate_walk", autocast=lambda x: Odometry.from_msg(x)
    ).iterate():
        counter += 1
        print(message)


def test_updated_odometry_rotation():
    """Test that the updated Odometry class now provides correct yaw angles in rot.z."""
    print("Testing updated Odometry class with proper quaternion conversion...")

    # Test with a few messages
    messages = list(
        SensorReplay(
            name="raw_odometry_rotate_walk", autocast=lambda x: Odometry.from_msg(x)
        ).iterate()
    )

    if len(messages) < 10:
        print("Need at least 10 messages for testing")
        return

    print("\nFirst 5 Odometry objects with corrected rotation:")
    for i, odom in enumerate(messages[:5]):
        yaw_degrees = math.degrees(odom.rot.z)
        print(f"  {i + 1}: {odom}")

    # Calculate total rotation using the new rot.z values
    total_rotation_radians = 0
    rotation_changes = []

    for i in range(1, len(messages)):
        # Now we can directly use rot.z which contains the yaw angle
        diff = messages[i].rot.z - messages[i - 1].rot.z

        # Normalize the difference to [-π, π]
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi

        rotation_changes.append(diff)
        total_rotation_radians += diff

    total_rotation_degrees = math.degrees(total_rotation_radians)

    print("\n=== SIMPLIFIED ROTATION ANALYSIS ===")
    print(f"Total rotation using Odometry.rot.z: {total_rotation_degrees:.2f} degrees")
    print(f"Number of messages: {len(messages)}")
    print(f"First few rotation changes: {[math.degrees(r) for r in rotation_changes[:5]]}")

    return total_rotation_degrees


def test_compare_rotation_methods():
    """Compare quaternion-based rotation calculation with Odometry.rot values."""
    messages = list(
        SensorReplay(
            name="raw_odometry_rotate_walk",
            autocast=lambda x: x,  # Keep raw messages
        ).iterate()
    )

    if not messages:
        print("No messages found in the replay data")
        return

    print("Analyzing first 10 messages to compare rotation methods...")

    for i, msg in enumerate(messages[:10]):
        # Extract quaternion from raw message
        orientation = msg["data"]["pose"]["orientation"]
        qx, qy, qz, qw = orientation["x"], orientation["y"], orientation["z"], orientation["w"]

        # Convert to Odometry object (which only uses x,y,z components)
        odom = Odometry.from_msg(msg)

        # Calculate yaw from full quaternion
        yaw_from_quaternion = quaternion_to_euler_z(qx, qy, qz, qw)
        yaw_degrees = math.degrees(yaw_from_quaternion)

        # Show what Odometry.rot contains (just x,y,z components of quaternion)
        rot_vector = odom.rot

        print(f"\nMessage {i + 1}:")
        print(f"  Full quaternion: x={qx:.6f}, y={qy:.6f}, z={qz:.6f}, w={qw:.6f}")
        print(f"  Odometry.rot (x,y,z only): {rot_vector}")
        print(f"  Yaw from quaternion: {yaw_degrees:.2f}°")
        print(f"  Z-component magnitude: {abs(qz):.6f}")
