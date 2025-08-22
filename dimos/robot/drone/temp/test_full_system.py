#!/usr/bin/env python3
"""Test the complete drone system."""

import sys
import os
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../..'))

from dimos.protocol import pubsub
from dimos.robot.drone.drone import Drone
from dimos.msgs.geometry_msgs import Vector3

def test_drone_system():
    print("Testing complete drone system...")
    
    # Configure LCM
    pubsub.lcm.autoconf()
    
    # Create drone
    drone = Drone(
        connection_string='udp:0.0.0.0:14550',
        video_port=5600
    )
    
    print("\n=== Starting Drone System ===")
    drone.start()
    
    # Wait for initialization
    time.sleep(3)
    
    print("\n=== Testing Drone Status ===")
    status = drone.get_status()
    print(f"Status: {status}")
    
    odom = drone.get_odom()
    print(f"Odometry: {odom}")
    
    print("\n=== Testing Camera ===")
    print("Waiting for video frame...")
    frame = drone.get_single_rgb_frame(timeout=5.0)
    if frame:
        print(f"✓ Got video frame: {frame.data.shape}")
    else:
        print("✗ No video frame received")
    
    print("\n=== Testing Movement Commands ===")
    print("Setting STABILIZE mode...")
    if drone.set_mode('STABILIZE'):
        print("✓ Mode set to STABILIZE")
    
    print("\nSending stop command...")
    drone.move(Vector3(0, 0, 0))
    print("✓ Stop command sent")
    
    print("\n=== System Running ===")
    print("Drone system is running. Press Ctrl+C to stop.")
    print("Foxglove visualization: http://localhost:8765")
    print("\nLCM Topics:")
    print("  /drone/odom - Odometry")
    print("  /drone/status - Status")
    print("  /drone/color_image - Video")
    print("  /drone/depth_image - Depth")
    print("  /drone/depth_colorized - Colorized depth")
    print("  /drone/cmd_vel - Movement commands")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        drone.stop()
        print("✓ System stopped")

if __name__ == "__main__":
    test_drone_system()