#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Integration test for the full manipulation blueprint.

This tests the joint state flow from XArmDriver → ManipulationModule.

Usage:
    python test_blueprint_integration.py
"""

from __future__ import annotations

import logging
import time

# Set up logging to see all messages
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

# Enable debug logging for key modules
logging.getLogger("dimos.manipulation").setLevel(logging.DEBUG)
logging.getLogger("dimos.hardware.manipulators").setLevel(logging.DEBUG)

from dimos.manipulation.manipulation_blueprints import xarm6_manipulation


def main():
    """Run the integration test."""
    print("=" * 60)
    print("Blueprint Integration Test")
    print("=" * 60)

    print("\n1. Building the full xarm6_manipulation blueprint...")
    print("   This includes: XArmDriver + ManipulationModule + JointTrajectoryController")

    # Build the blueprint - this creates all modules, connects transports, and starts them
    coordinator = xarm6_manipulation.build()

    print("\n2. Blueprint built successfully!")
    print(f"   Active modules: {list(coordinator._deployed_modules.keys())}")

    # Wait for modules to initialize
    print("\n3. Waiting for modules to initialize...")
    time.sleep(2)

    # Try to get joint state from ManipulationModule
    print("\n4. Checking if joint states are being received...")

    # Get the ManipulationModule RPC proxy (Dask actor)
    from dimos.manipulation import ManipulationModule

    manip_module = coordinator.get_instance(ManipulationModule)

    if manip_module is None:
        print("   ERROR: ManipulationModule not found in coordinator!")
        return

    # Check debug info (RPC call)
    # Note: dimos RPCClient returns results directly, not futures
    debug_info = manip_module.get_debug_info()
    print("   Debug info:")
    for key, value in debug_info.items():
        print(f"     {key}: {value}")

    # Try to get current joints
    joints = manip_module.get_current_joints()
    print("\n5. Current joints from ManipulationModule:")
    if joints:
        print(f"   Positions: {[f'{j:.4f}' for j in joints]}")
    else:
        print("   WARNING: No joint positions available!")

    # Get EE pose
    ee_pose = manip_module.get_ee_pose()
    print("\n6. Current EE pose:")
    if ee_pose:
        x, y, z, roll, pitch, yaw = ee_pose
        print(f"   Position: ({x:.4f}, {y:.4f}, {z:.4f})")
        print(f"   Orientation: ({roll:.4f}, {pitch:.4f}, {yaw:.4f})")
    else:
        print("   WARNING: No EE pose available!")

    # Check visualization URL
    viz_url = manip_module.get_visualization_url()
    print("\n7. Visualization:")
    if viz_url:
        print(f"   Meshcat URL: {viz_url}")
    else:
        print("   Visualization not available")

    # Wait for more joint states
    print("\n8. Waiting 5 seconds to collect more joint states...")
    time.sleep(5)

    # Check debug info again
    debug_info = manip_module.get_debug_info()
    print("\n9. Debug info after waiting:")
    for key, value in debug_info.items():
        print(f"     {key}: {value}")

    joints = manip_module.get_current_joints()
    if joints:
        print(f"\n   Current joints: {[f'{j:.4f}' for j in joints]}")
    else:
        print("\n   WARNING: Still no joint positions after 5 seconds!")

    print("\n" + "=" * 60)
    print("Test complete!")
    print("=" * 60)

    # Keep running for interactive testing
    print("\nPress Ctrl+C to exit...")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        coordinator.stop()


if __name__ == "__main__":
    main()
