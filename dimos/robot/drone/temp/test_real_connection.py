#!/usr/bin/env python3
"""Test real drone connection."""

import sys
import os
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../..'))

from dimos.robot.drone.connection import DroneConnection

def test_real_drone():
    print("Testing real drone connection...")
    
    # Create connection
    drone = DroneConnection('udp:0.0.0.0:14550')
    
    if not drone.connected:
        print("Failed to connect to drone")
        return False
    
    print("✓ Connected successfully!")
    
    # Get telemetry
    print("\n=== Telemetry ===")
    for i in range(3):
        telemetry = drone.get_telemetry()
        print(f"\nTelemetry update {i+1}:")
        for key, value in telemetry.items():
            print(f"  {key}: {value}")
        time.sleep(1)
    
    # Test arm command (will fail with safety on)
    print("\n=== Testing Arm Command ===")
    print("Attempting to arm (should fail with safety on)...")
    result = drone.arm()
    print(f"Arm result: {result}")
    
    # Test mode setting
    print("\n=== Testing Mode Changes ===")
    print("Current mode:", telemetry.get('mode'))
    
    print("Setting STABILIZE mode...")
    if drone.set_mode('STABILIZE'):
        print("✓ STABILIZE mode set")
    
    time.sleep(1)
    
    print("Setting GUIDED mode...")
    if drone.set_mode('GUIDED'):
        print("✓ GUIDED mode set")
    
    # Test takeoff (will fail without arm)
    print("\n=== Testing Takeoff Command ===")
    print("Attempting takeoff (should fail without arm)...")
    result = drone.takeoff(2.0)
    print(f"Takeoff result: {result}")
    
    # Disconnect
    drone.disconnect()
    print("\n✓ Test completed successfully!")
    return True

if __name__ == "__main__":
    try:
        test_real_drone()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()