#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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
Simple test script for Piper driver.

Tests:
1. Initialize and start the driver
2. Read joint states
3. Send command to move to home position
4. Monitor joint states
5. Stop the driver

Usage:
    python3 test_piper_driver.py
"""

import math
import time

from dimos.hardware.manipulators.piper import PiperDriver


def main():
    print("=" * 80)
    print("Piper Driver Test")
    print("=" * 80)

    # Create driver instance
    print("\nCreating PiperDriver instance...")
    driver = PiperDriver(
        can_name="can0",
        enable_on_start=True,
        control_frequency=100.0,
        joint_state_rate=100.0,
        robot_state_rate=10.0,
    )

    try:
        # Start the driver
        print("\nStarting driver...")
        driver.start()
        print("✓ Driver started successfully!")

        # Wait for initial state
        print("\nWaiting for initial joint state...")
        time.sleep(2.0)

        # Get current joint state
        joint_state = driver.get_joint_state()
        if joint_state:
            print("\n✓ Joint state received!")
            print("\nCurrent Joint Positions (radians):")
            for i, pos in enumerate(joint_state.position):
                print(f"  Joint {i + 1}: {pos:7.4f} rad ({math.degrees(pos):7.2f}°)")
        else:
            print("\n✗ No joint state available")

        # Get robot state
        robot_state = driver.get_robot_state()
        if robot_state:
            print("\nRobot State:")
            print(f"  State: {robot_state.state}")
            print(f"  Mode: {robot_state.mode}")
            print(f"  Error code: {robot_state.error_code}")
            print(f"  Warning code: {robot_state.warn_code}")
        else:
            print("\n✗ No robot state available")

        # Move to home position (all zeros)
        print("\n" + "=" * 80)
        print("Moving to HOME position (all joints to 0.0 radians)...")
        print("=" * 80)

        home_position = [0.0] * driver.config.num_joints

        # Send command by setting internal state
        print(f"\nSending command: {home_position}")
        with driver._joint_cmd_lock:
            driver._joint_cmd_ = home_position
            driver._last_cmd_time = time.time()

        # Monitor motion for 5 seconds
        print("\nMonitoring motion (5 seconds)...")
        print("Time(s) | Joint Positions (degrees)")
        print("-" * 80)

        start_time = time.time()
        duration = 5.0

        while time.time() - start_time < duration:
            joint_state = driver.get_joint_state()
            if joint_state:
                elapsed = time.time() - start_time
                positions_deg = [math.degrees(p) for p in joint_state.position]
                positions_str = ", ".join([f"{p:6.2f}" for p in positions_deg])
                print(f"{elapsed:6.2f}  | {positions_str}")
            time.sleep(0.5)

        # Check final position
        print("\n" + "=" * 80)
        final_state = driver.get_joint_state()
        if final_state:
            tolerance = 0.05  # 0.05 radians (~2.86 degrees)
            errors = [abs(pos - 0.0) for pos in final_state.position]
            max_error = max(errors)

            print("\nFinal Joint Positions:")
            for i, (pos, err) in enumerate(zip(final_state.position, errors, strict=False)):
                status = "✓" if err < tolerance else "✗"
                print(
                    f"  {status} Joint {i + 1}: {pos:7.4f} rad ({math.degrees(pos):7.2f}°) [error: {math.degrees(err):5.2f}°]"
                )

            if max_error < tolerance:
                print(
                    f"\n✓ SUCCESS: Reached home position (max error: {math.degrees(max_error):.2f}°)"
                )
            else:
                print(
                    f"\n⚠ WARNING: Not fully at home position (max error: {math.degrees(max_error):.2f}°)"
                )

        print("\n✓ Test completed successfully!")

    except KeyboardInterrupt:
        print("\n\n✗ Interrupted by user (Ctrl+C)")

    except Exception as e:
        print(f"\n✗ Error during test: {e}")
        import traceback

        traceback.print_exc()

    finally:
        # Stop the driver
        print("\n" + "=" * 80)
        print("Stopping driver...")
        driver.stop()
        print("✓ Driver stopped")
        print("=" * 80)


if __name__ == "__main__":
    main()
