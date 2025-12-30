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
Integration test runner for manipulator drivers.

This is a standalone script (NOT a pytest test file) that tests the common
BaseManipulatorDriver interface that all arms implement.
Supports both mock mode (for CI/CD) and hardware mode (for real testing).

NOTE: This file is intentionally NOT named test_*.py to avoid pytest auto-discovery.
For pytest-based unit tests, see: dimos/hardware/manipulators/base/tests/test_driver_unit.py

Usage:
    # Run with mock (CI/CD safe, default)
    python -m dimos.hardware.manipulators.integration_test_runner

    # Run specific arm with mock
    python -m dimos.hardware.manipulators.integration_test_runner --arm piper

    # Run with real hardware (xArm)
    python -m dimos.hardware.manipulators.integration_test_runner --hardware --ip 192.168.1.210

    # Run with real hardware (Piper)
    python -m dimos.hardware.manipulators.integration_test_runner --hardware --arm piper --can can0

    # Run specific test
    python -m dimos.hardware.manipulators.integration_test_runner --test connection

    # Skip motion tests (safer for hardware)
    python -m dimos.hardware.manipulators.integration_test_runner --hardware --skip-motion
"""

import argparse
import math
import sys
import time

from dimos.core.transport import LCMTransport
from dimos.hardware.manipulators.base.sdk_interface import BaseManipulatorSDK, ManipulatorInfo
from dimos.msgs.sensor_msgs import JointState, RobotState


class MockSDK(BaseManipulatorSDK):
    """Mock SDK for testing without hardware. Works for any arm type."""

    def __init__(self, dof: int = 6, vendor: str = "Mock", model: str = "TestArm"):
        self._connected = True
        self._dof = dof
        self._vendor = vendor
        self._model = model
        self._positions = [0.0] * dof
        self._velocities = [0.0] * dof
        self._efforts = [0.0] * dof
        self._servos_enabled = False
        self._mode = 0
        self._state = 0
        self._error_code = 0

    def connect(self, config: dict) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def get_joint_positions(self) -> list[float]:
        return self._positions.copy()

    def get_joint_velocities(self) -> list[float]:
        return self._velocities.copy()

    def get_joint_efforts(self) -> list[float]:
        return self._efforts.copy()

    def set_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
        acceleration: float = 1.0,
        wait: bool = False,
    ) -> bool:
        if not self._servos_enabled:
            return False
        self._positions = list(positions)
        return True

    def set_joint_velocities(self, velocities: list[float]) -> bool:
        if not self._servos_enabled:
            return False
        self._velocities = list(velocities)
        return True

    def set_joint_efforts(self, efforts: list[float]) -> bool:
        return False  # Not supported in mock

    def stop_motion(self) -> bool:
        self._velocities = [0.0] * self._dof
        return True

    def enable_servos(self) -> bool:
        self._servos_enabled = True
        return True

    def disable_servos(self) -> bool:
        self._servos_enabled = False
        return True

    def are_servos_enabled(self) -> bool:
        return self._servos_enabled

    def get_robot_state(self) -> dict:
        return {
            "state": self._state,
            "mode": self._mode,
            "error_code": self._error_code,
            "is_moving": any(v != 0 for v in self._velocities),
        }

    def get_error_code(self) -> int:
        return self._error_code

    def get_error_message(self) -> str:
        return "" if self._error_code == 0 else f"Error {self._error_code}"

    def clear_errors(self) -> bool:
        self._error_code = 0
        return True

    def emergency_stop(self) -> bool:
        self._velocities = [0.0] * self._dof
        self._servos_enabled = False
        return True

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor=self._vendor,
            model=f"{self._model} (Mock)",
            dof=self._dof,
            firmware_version="mock-1.0.0",
            serial_number="MOCK-001",
        )

    def get_joint_limits(self) -> tuple[list[float], list[float]]:
        lower = [-2 * math.pi] * self._dof
        upper = [2 * math.pi] * self._dof
        return lower, upper

    def get_velocity_limits(self) -> list[float]:
        return [math.pi] * self._dof

    def get_acceleration_limits(self) -> list[float]:
        return [math.pi * 2] * self._dof


# =============================================================================
# Test Functions (work with any driver implementing BaseManipulatorDriver)
# =============================================================================


def check_connection(driver, hardware: bool) -> bool:
    """Test that driver connects to hardware/mock."""
    print("Testing connection...")

    if not driver.sdk.is_connected():
        print("  FAIL: SDK not connected")
        return False

    info = driver.sdk.get_info()
    print(f"  Connected to: {info.vendor} {info.model}")
    print(f"  DOF: {info.dof}")
    print(f"  Firmware: {info.firmware_version}")
    print(f"  Mode: {'HARDWARE' if hardware else 'MOCK'}")
    print("  PASS")
    return True


def check_read_joint_state(driver, hardware: bool) -> bool:
    """Test reading joint state."""
    print("Testing read joint state...")

    result = driver.get_joint_state()
    if not result.get("success"):
        print(f"  FAIL: {result.get('error')}")
        return False

    positions = result["positions"]
    velocities = result["velocities"]
    efforts = result["efforts"]

    print(f"  Positions (deg): {[f'{math.degrees(p):.1f}' for p in positions]}")
    print(f"  Velocities: {[f'{v:.3f}' for v in velocities]}")
    print(f"  Efforts: {[f'{e:.2f}' for e in efforts]}")

    if len(positions) != driver.capabilities.dof:
        print(f"  FAIL: Expected {driver.capabilities.dof} joints, got {len(positions)}")
        return False

    print("  PASS")
    return True


def check_get_robot_state(driver, hardware: bool) -> bool:
    """Test getting robot state."""
    print("Testing robot state...")

    result = driver.get_robot_state()
    if not result.get("success"):
        print(f"  FAIL: {result.get('error')}")
        return False

    print(f"  State: {result.get('state')}")
    print(f"  Mode: {result.get('mode')}")
    print(f"  Error code: {result.get('error_code')}")
    print(f"  Is moving: {result.get('is_moving')}")
    print("  PASS")
    return True


def check_servo_enable_disable(driver, hardware: bool) -> bool:
    """Test enabling and disabling servos."""
    print("Testing servo enable/disable...")

    # Enable
    result = driver.enable_servo()
    if not result.get("success"):
        print(f"  FAIL enable: {result.get('error')}")
        return False
    print("  Enabled servos")

    # Hardware needs more time for state to propagate
    time.sleep(1.0 if hardware else 0.01)

    # Check state with retry for hardware
    enabled = driver.sdk.are_servos_enabled()
    if not enabled and hardware:
        # Retry after additional delay
        time.sleep(0.5)
        enabled = driver.sdk.are_servos_enabled()

    if not enabled:
        print("  FAIL: Servos not enabled after enable_servo()")
        return False
    print("  Verified servos enabled")

    # # Disable
    # result = driver.disable_servo()
    # if not result.get("success"):
    #     print(f"  FAIL disable: {result.get('error')}")
    #     return False
    # print("  Disabled servos")

    print("  PASS")
    return True


def check_joint_limits(driver, hardware: bool) -> bool:
    """Test getting joint limits."""
    print("Testing joint limits...")

    result = driver.get_joint_limits()
    if not result.get("success"):
        print(f"  FAIL: {result.get('error')}")
        return False

    lower = result["lower"]
    upper = result["upper"]

    print(f"  Lower (deg): {[f'{math.degrees(l):.1f}' for l in lower]}")
    print(f"  Upper (deg): {[f'{math.degrees(u):.1f}' for u in upper]}")

    if len(lower) != driver.capabilities.dof:
        print("  FAIL: Wrong number of limits")
        return False

    print("  PASS")
    return True


def check_stop_motion(driver, hardware: bool) -> bool:
    """Test stop motion command."""
    print("Testing stop motion...")

    result = driver.stop_motion()
    # Note: stop_motion may return success=False if arm isn't moving,
    # which is expected behavior. We just verify no exception occurred.
    if result is None:
        print("  FAIL: stop_motion returned None")
        return False

    if result.get("error"):
        print(f"  FAIL: {result.get('error')}")
        return False

    # success=False when not moving is OK, success=True is also OK
    print(f"  stop_motion returned success={result.get('success')}")
    print("  PASS")
    return True


def check_small_motion(driver, hardware: bool) -> bool:
    """Test a small joint motion (5 degrees on joint 1).

    WARNING: With --hardware, this MOVES the real robot!
    """
    print("Testing small motion (5 deg on J1)...")
    if hardware:
        print("  WARNING: Robot will move!")

    # Get current position
    result = driver.get_joint_state()
    if not result.get("success"):
        print(f"  FAIL: Cannot read state: {result.get('error')}")
        return False

    current_pos = list(result["positions"])
    print(f"  Current J1: {math.degrees(current_pos[0]):.2f} deg")

    driver.clear_errors()
    # print(driver.get_state())

    # Enable servos
    result = driver.enable_servo()
    print(result)
    if not result.get("success"):
        print(f"  FAIL: Cannot enable servos: {result.get('error')}")
        return False

    time.sleep(0.5 if hardware else 0.01)

    # Move +5 degrees on joint 1
    target_pos = current_pos.copy()
    target_pos[0] += math.radians(5.0)
    print(f"  Target J1: {math.degrees(target_pos[0]):.2f} deg")

    result = driver.move_joint(target_pos, velocity=0.3, wait=True)
    if not result.get("success"):
        print(f"  FAIL: Motion failed: {result.get('error')}")
        return False

    time.sleep(1.0 if hardware else 0.01)

    # Verify position
    result = driver.get_joint_state()
    new_pos = result["positions"]
    error = abs(new_pos[0] - target_pos[0])
    print(
        f"  Reached J1: {math.degrees(new_pos[0]):.2f} deg (error: {math.degrees(error):.3f} deg)"
    )

    if hardware and error > math.radians(1.0):  # Allow 1 degree error for real hardware
        print("  FAIL: Position error too large")
        return False

    # Move back
    print("  Moving back to original position...")
    driver.move_joint(current_pos, velocity=0.3, wait=True)
    time.sleep(1.0 if hardware else 0.01)

    print("  PASS")
    return True


# =============================================================================
# Driver Factory
# =============================================================================


def create_driver(arm: str, hardware: bool, config: dict):
    """Create driver for the specified arm type.

    Args:
        arm: Arm type ('xarm', 'piper', etc.)
        hardware: If True, use real hardware; if False, use mock SDK
        config: Configuration dict (ip, dof, etc.)

    Returns:
        Driver instance
    """
    if arm == "xarm":
        from dimos.hardware.manipulators.xarm.xarm_driver import XArmDriver

        if hardware:
            return XArmDriver(config=config)
        else:
            # Create driver with mock SDK
            driver = XArmDriver.__new__(XArmDriver)
            # Manually initialize with mock
            from dimos.hardware.manipulators.base import (
                BaseManipulatorDriver,
                StandardMotionComponent,
                StandardServoComponent,
                StandardStatusComponent,
            )

            mock_sdk = MockSDK(dof=config.get("dof", 6), vendor="UFactory", model="xArm")
            components = [
                StandardMotionComponent(),
                StandardServoComponent(),
                StandardStatusComponent(),
            ]
            BaseManipulatorDriver.__init__(
                driver, sdk=mock_sdk, components=components, config=config, name="XArmDriver"
            )
            return driver

    elif arm == "piper":
        from dimos.hardware.manipulators.piper.piper_driver import PiperDriver

        if hardware:
            return PiperDriver(config=config)
        else:
            # Create driver with mock SDK
            driver = PiperDriver.__new__(PiperDriver)
            from dimos.hardware.manipulators.base import (
                BaseManipulatorDriver,
                StandardMotionComponent,
                StandardServoComponent,
                StandardStatusComponent,
            )

            mock_sdk = MockSDK(dof=6, vendor="Agilex", model="Piper")
            components = [
                StandardMotionComponent(),
                StandardServoComponent(),
                StandardStatusComponent(),
            ]
            BaseManipulatorDriver.__init__(
                driver, sdk=mock_sdk, components=components, config=config, name="PiperDriver"
            )
            return driver

    else:
        raise ValueError(f"Unknown arm type: {arm}. Supported: xarm, piper")


# =============================================================================
# Test Runner
# =============================================================================


def configure_transports(driver, arm: str):
    """Configure LCM transports for the driver (like production does).

    Args:
        driver: The driver instance
        arm: Arm type for topic naming
    """
    # Create LCM transports for state publishing
    joint_state_transport = LCMTransport(f"/test/{arm}/joint_state", JointState)
    robot_state_transport = LCMTransport(f"/test/{arm}/robot_state", RobotState)

    # Set transports on driver's Out streams
    if driver.joint_state:
        driver.joint_state._transport = joint_state_transport
    if driver.robot_state:
        driver.robot_state._transport = robot_state_transport


def run_tests(
    arm: str,
    hardware: bool,
    config: dict,
    test_name: str | None = None,
    skip_motion: bool = False,
):
    """Run integration tests."""
    mode = "HARDWARE" if hardware else "MOCK"
    print("=" * 60)
    print(f"Manipulator Driver Integration Tests ({mode})")
    print("=" * 60)
    print(f"Arm: {arm}")
    print(f"Config: {config}")
    print()

    # Create driver
    print("Creating driver...")
    try:
        driver = create_driver(arm, hardware, config)
    except Exception as e:
        print(f"FATAL: Failed to create driver: {e}")
        return False

    # Configure transports (like production does)
    print("Configuring transports...")
    configure_transports(driver, arm)

    # Start driver
    print("Starting driver...")
    try:
        driver.start()
        # Piper needs more initialization time before commands work
        wait_time = 3.0 if (hardware and arm == "piper") else (1.0 if hardware else 0.1)
        time.sleep(wait_time)
    except Exception as e:
        print(f"FATAL: Failed to start driver: {e}")
        return False

    # Define tests (stop_motion last since it leaves arm in stopped state)
    tests = [
        ("connection", check_connection),
        ("read_state", check_read_joint_state),
        ("robot_state", check_get_robot_state),
        ("joint_limits", check_joint_limits),
        # ("servo", check_servo_enable_disable),
    ]

    if not skip_motion:
        tests.append(("motion", check_small_motion))

    # Stop test always last (leaves arm in stopped state)
    tests.append(("stop", check_stop_motion))

    # Run tests
    results = {}
    print()
    print("-" * 60)

    for name, test_func in tests:
        if test_name and name != test_name:
            continue

        try:
            results[name] = test_func(driver, hardware)
        except Exception as e:
            print(f"  EXCEPTION: {e}")
            import traceback

            traceback.print_exc()
            results[name] = False

        print()

    # Stop driver
    print("Stopping driver...")
    try:
        driver.stop()
    except Exception as e:
        print(f"Warning: Error stopping driver: {e}")

    # Summary
    print("-" * 60)
    print("SUMMARY")
    print("-" * 60)
    passed = sum(1 for r in results.values() if r)
    total = len(results)

    for name, result in results.items():
        status = "PASS" if result else "FAIL"
        print(f"  {name}: {status}")

    print()
    print(f"Result: {passed}/{total} tests passed")

    return passed == total


def main():
    parser = argparse.ArgumentParser(
        description="Generic manipulator driver integration tests",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Mock mode (CI/CD safe, default)
  python -m dimos.hardware.manipulators.integration_test_runner

  # xArm hardware mode
  python -m dimos.hardware.manipulators.integration_test_runner --hardware --ip 192.168.1.210

  # Piper hardware mode
  python -m dimos.hardware.manipulators.integration_test_runner --hardware --arm piper --can can0

  # Skip motion tests
  python -m dimos.hardware.manipulators.integration_test_runner --hardware --skip-motion
""",
    )
    parser.add_argument(
        "--arm", default="xarm", choices=["xarm", "piper"], help="Arm type to test (default: xarm)"
    )
    parser.add_argument(
        "--hardware", action="store_true", help="Use real hardware (default: mock mode)"
    )
    parser.add_argument(
        "--ip", default="192.168.1.210", help="IP address for xarm (default: 192.168.1.210)"
    )
    parser.add_argument("--can", default="can0", help="CAN interface for piper (default: can0)")
    parser.add_argument(
        "--dof", type=int, help="Degrees of freedom (auto-detected in hardware mode)"
    )
    parser.add_argument("--test", help="Run specific test only")
    parser.add_argument("--skip-motion", action="store_true", help="Skip motion tests")
    args = parser.parse_args()

    # Build config - DOF auto-detected from hardware if not specified
    config = {}
    if args.arm == "xarm" and args.ip:
        config["ip"] = args.ip
    if args.arm == "piper" and args.can:
        config["can_port"] = args.can
    if args.dof:
        config["dof"] = args.dof
    elif not args.hardware:
        # Mock mode needs explicit DOF
        config["dof"] = 6

    success = run_tests(args.arm, args.hardware, config, args.test, args.skip_motion)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
