#!/usr/bin/env python3
"""Test DroneConnectionModule functionality."""

import sys
import os
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../..'))

from dimos import core
from dimos.robot.drone.connection_module import DroneConnectionModule
from dimos.msgs.geometry_msgs import Vector3

def test_module():
    print("Testing DroneConnectionModule...")
    
    # Start DimOS with 2 workers
    dimos = core.start(2)
    
    # Deploy the connection module
    print("Deploying connection module...")
    connection = dimos.deploy(DroneConnectionModule, connection_string='udp:0.0.0.0:14550')
    
    # Configure LCM transports with proper message types
    from dimos.msgs.geometry_msgs import PoseStamped, Vector3
    from dimos_lcm.std_msgs import String
    
    connection.odom.transport = core.LCMTransport("/drone/odom", PoseStamped)
    connection.status.transport = core.LCMTransport("/drone/status", String)
    connection.movecmd.transport = core.LCMTransport("/drone/cmd_vel", Vector3)
    
    # Get module info
    print("Module I/O configuration:")
    io_info = connection.io()
    print(io_info)
    
    # Try to start (will fail without drone, but tests the module)
    print("\nStarting module (will timeout without drone)...")
    try:
        result = connection.start()
        print(f"Start result: {result}")
    except Exception as e:
        print(f"Start failed as expected: {e}")
    
    # Test RPC methods
    print("\nTesting RPC methods...")
    
    odom = connection.get_odom()
    print(f"  get_odom(): {odom}")
    
    status = connection.get_status()
    print(f"  get_status(): {status}")
    
    # Shutdown
    print("\nShutting down...")
    dimos.shutdown()
    print("Test completed!")

if __name__ == "__main__":
    test_module()