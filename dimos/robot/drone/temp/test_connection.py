#!/usr/bin/env python3
"""Test DroneConnection basic functionality."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../..'))

from dimos.robot.drone.connection import DroneConnection
import time

def test_connection():
    print("Testing DroneConnection...")
    
    # Create connection
    drone = DroneConnection('udp:0.0.0.0:14550')
    
    if not drone.connected:
        print("Failed to connect")
        return False
    
    print("Connected successfully!")
    
    # Test telemetry
    print("\n=== Testing Telemetry ===")
    for i in range(3):
        telemetry = drone.get_telemetry()
        print(f"Telemetry {i+1}: {telemetry}")
        time.sleep(1)
    
    # Test streams
    print("\n=== Testing Streams ===")
    odom_count = [0]
    status_count = [0]
    
    def on_odom(msg):
        odom_count[0] += 1
        print(f"Odom received: pos={msg.position}, orientation={msg.orientation}")
    
    def on_status(msg):
        status_count[0] += 1
        print(f"Status received: armed={msg.get('armed')}, mode={msg.get('mode')}")
    
    # Subscribe to streams
    odom_sub = drone.odom_stream().subscribe(on_odom)
    status_sub = drone.status_stream().subscribe(on_status)
    
    # Update telemetry to trigger stream updates
    for i in range(5):
        drone.update_telemetry(timeout=0.5)
        time.sleep(0.5)
    
    print(f"\nReceived {odom_count[0]} odom messages, {status_count[0]} status messages")
    
    # Cleanup
    odom_sub.dispose()
    status_sub.dispose()
    drone.disconnect()
    
    return True

if __name__ == "__main__":
    try:
        success = test_connection()
        if success:
            print("\nAll tests passed!")
        else:
            print("\nTests failed!")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()