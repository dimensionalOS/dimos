#!/usr/bin/env python3
"""Simple test of drone system components."""

import sys
import os
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../..'))

from dimos.protocol import pubsub
from dimos.robot.drone.connection import DroneConnection

def test_simple():
    print("Testing simple drone connection...")
    
    # Test basic connection
    drone = DroneConnection('udp:0.0.0.0:14550')
    
    if drone.connected:
        print("✓ Connected to drone!")
        
        # Get telemetry
        telemetry = drone.get_telemetry()
        print(f"\nTelemetry:")
        print(f"  Armed: {telemetry.get('armed', False)}")
        print(f"  Mode: {telemetry.get('mode', -1)}")
        print(f"  Altitude: {telemetry.get('relative_alt', 0):.1f}m")
        print(f"  Heading: {telemetry.get('heading', 0)}°")
        
        drone.disconnect()
    else:
        print("✗ Failed to connect")

if __name__ == "__main__":
    test_simple()