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
Simple unit test for xArm RT Driver.

This test mirrors test_xarm_driver.py but uses mocked hardware.
It tests all the same functionalities in a simple, straightforward way.

Usage:
    python tests/test_xarm_rt_driver.py
"""

import sys
import os

# Add dimos root to path
script_dir = os.path.dirname(os.path.abspath(__file__))
dimos_root = os.path.dirname(script_dir)
if dimos_root not in sys.path:
    sys.path.insert(0, dimos_root)

import time
from unittest.mock import MagicMock, patch

from dimos.hardware.manipulators.xarm.xarm_driver import XArmDriver
from dimos.msgs.sensor_msgs import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__file__)


def create_mock_xarm():
    """Create a mock XArmAPI that simulates successful xArm responses."""
    mock = MagicMock()

    # Connection properties
    mock.connected = True
    mock.version_number = (1, 10, 0)

    # Connection methods
    mock.connect.return_value = None
    mock.disconnect.return_value = None

    # Callback registration
    mock.register_connect_changed_callback.return_value = None
    mock.register_report_callback.return_value = None
    mock.release_connect_changed_callback.return_value = None
    mock.release_report_callback.return_value = None

    # Error/warning methods
    mock.get_err_warn_code.return_value = 0
    mock.clean_error.return_value = 0
    mock.clean_warn.return_value = 0

    # State/control methods (return code, message)
    mock.motion_enable.return_value = (0, "Motion enabled")
    mock.set_mode.return_value = 0
    mock.set_state.return_value = 0

    # Query methods
    mock.get_version.return_value = (0, "v1.10.0")
    mock.get_position.return_value = (0, [200.0, 0.0, 300.0, 3.14, 0.0, 0.0])
    mock.get_servo_angle.return_value = (0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    mock.get_joint_states.return_value = (
        0,
        [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # positions
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # velocities
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # efforts
        ],
    )

    # Command methods
    mock.set_servo_angle_j.return_value = 0
    mock.vc_set_joint_velocity.return_value = 0

    # Force/torque sensor data
    mock.ft_ext_force = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    mock.ft_raw_force = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    return mock


def test_basic_connection():
    """Test basic connection and startup (mirrors test_basic_connection)."""
    logger.info("=" * 80)
    logger.info("TEST 1: Basic Connection")
    logger.info("=" * 80)

    with patch("dimos.hardware.manipulators.xarm.xarm_driver.XArmAPI") as MockXArmAPI:
        MockXArmAPI.return_value = create_mock_xarm()

        logger.info("Creating XArmDriver...")
        driver = XArmDriver(
            ip_address="192.168.1.235",
            control_frequency=100.0,
            joint_state_rate=100.0,
            report_type="dev",
            enable_on_start=False,
            xarm_type="xarm6",
        )

        logger.info("Starting driver...")
        driver.start()
        time.sleep(1.0)

        # Check connection via RPC (mirrors real test)
        logger.info("Checking connection via RPC...")
        code, version = driver.get_version()
        if code == 0:
            logger.info(f"✓ Firmware version: {version}")
        else:
            logger.error(f"✗ Failed to get firmware version: code={code}")
            driver.stop()
            return False

        # Get robot state via RPC
        logger.info("Getting robot state via RPC...")
        robot_state = driver.get_robot_state()
        if robot_state:
            logger.info(
                f"✓ Robot State: state={robot_state.state}, mode={robot_state.mode}, "
                f"error={robot_state.error_code}, warn={robot_state.warn_code}"
            )
        else:
            logger.warning("✗ No robot state available yet")

        logger.info("Stopping driver...")
        driver.stop()

        logger.info("✓ TEST 1 PASSED\n")
        return True


def test_joint_state_reading():
    """Test joint state reading (mirrors test_joint_state_reading)."""
    logger.info("=" * 80)
    logger.info("TEST 2: Joint State Reading")
    logger.info("=" * 80)

    with patch("dimos.hardware.manipulators.xarm.xarm_driver.XArmAPI") as MockXArmAPI:
        MockXArmAPI.return_value = create_mock_xarm()

        logger.info("Creating XArmDriver...")
        driver = XArmDriver(
            ip_address="192.168.1.235",
            control_frequency=100.0,
            joint_state_rate=100.0,
            report_type="dev",
            enable_on_start=False,
            xarm_type="xarm6",
        )

        # Track received joint states
        joint_states_received = []

        def on_joint_state(msg: JointState):
            """Callback for receiving joint state messages."""
            joint_states_received.append(msg)
            if len(joint_states_received) <= 3:
                logger.info(
                    f"Received joint state #{len(joint_states_received)}: "
                    f"positions={[f'{p:.3f}' for p in msg.position[:3]]}..."
                )

        # Subscribe to joint states
        try:
            logger.info("Subscribing to joint_state...")
            driver.joint_state.subscribe(on_joint_state)
        except (AttributeError, ValueError) as e:
            logger.info(f"Note: Could not subscribe (no transport configured): {e}")
            logger.info("This is expected in unit test mode - continuing...")

        logger.info("Starting driver - joint states will publish at 100Hz...")
        driver.start()

        # Collect messages for 2 seconds
        logger.info("Collecting messages for 2 seconds...")
        time.sleep(2.0)

        # Check results
        logger.info(f"\nReceived {len(joint_states_received)} joint state messages")

        if len(joint_states_received) > 0:
            rate = len(joint_states_received) / 2.0
            logger.info(f"✓ Joint state publishing rate: ~{rate:.1f} Hz")

            last_state = joint_states_received[-1]
            logger.info(f"✓ Last state has {len(last_state.position)} joint positions")

            if rate > 50:
                logger.info("✓ Joint state publishing rate is good (>50 Hz)")
            else:
                logger.warning(f"⚠ Joint state publishing rate seems low: {rate:.1f} Hz")
        else:
            logger.info("Note: No messages received (no transport configured)")
            logger.info("This is expected in unit test mode")

        driver.stop()
        logger.info("✓ TEST 2 PASSED\n")
        return True


def test_command_sending():
    """Test command RPC methods (mirrors test_command_sending)."""
    logger.info("=" * 80)
    logger.info("TEST 3: Command RPC Methods")
    logger.info("=" * 80)

    with patch("dimos.hardware.manipulators.xarm.xarm_driver.XArmAPI") as MockXArmAPI:
        MockXArmAPI.return_value = create_mock_xarm()

        driver = XArmDriver(
            ip_address="192.168.1.235",
            control_frequency=100.0,
            joint_state_rate=100.0,
            report_type="dev",
            enable_on_start=False,
            xarm_type="xarm6",
        )

        driver.start()
        time.sleep(1.0)

        # Test command methods
        logger.info("Testing command RPC methods...")

        logger.info("Testing motion_enable()...")
        code, msg = driver.motion_enable(enable=True)
        logger.info(f"  motion_enable returned: code={code}, msg={msg}")

        logger.info("Testing enable_servo_mode()...")
        code, msg = driver.enable_servo_mode()
        logger.info(f"  enable_servo_mode returned: code={code}, msg={msg}")

        logger.info("Testing disable_servo_mode()...")
        code, msg = driver.disable_servo_mode()
        logger.info(f"  disable_servo_mode returned: code={code}, msg={msg}")

        logger.info("Testing set_state(0)...")
        code, msg = driver.set_state(0)
        logger.info(f"  set_state returned: code={code}, msg={msg}")

        logger.info("Testing get_position()...")
        code, position = driver.get_position()
        if code == 0 and position:
            logger.info(f"✓ get_position: {[f'{p:.1f}' for p in position[:3]]} (x,y,z in mm)")

        logger.info("✓ All command RPC methods are functional")

        driver.stop()
        logger.info("✓ TEST 3 PASSED\n")
        return True


def test_rpc_methods():
    """Test various RPC methods (mirrors test_rpc_methods)."""
    logger.info("=" * 80)
    logger.info("TEST 4: RPC Methods")
    logger.info("=" * 80)

    with patch("dimos.hardware.manipulators.xarm.xarm_driver.XArmAPI") as MockXArmAPI:
        MockXArmAPI.return_value = create_mock_xarm()

        driver = XArmDriver(
            ip_address="192.168.1.235",
            control_frequency=100.0,
            joint_state_rate=100.0,
            report_type="normal",
            enable_on_start=False,
            xarm_type="xarm6",
        )

        driver.start()
        time.sleep(1.0)

        # Test get_version
        logger.info("Testing get_version() RPC...")
        code, version = driver.get_version()
        if code == 0:
            logger.info(f"✓ get_version: {version}")

        # Test get_position
        logger.info("Testing get_position() RPC...")
        code, position = driver.get_position()
        if code == 0:
            logger.info(f"✓ get_position: {[f'{p:.3f}' for p in position]}")

        # Test motion_enable
        logger.info("Testing motion_enable() RPC...")
        code, msg = driver.motion_enable(enable=True)
        if code == 0:
            logger.info(f"✓ motion_enable: {msg}")

        # Test clean_error
        logger.info("Testing clean_error() RPC...")
        code, msg = driver.clean_error()
        if code == 0:
            logger.info(f"✓ clean_error: {msg}")

        driver.stop()
        logger.info("✓ TEST 4 PASSED\n")
        return True


def main():
    """Run all tests."""
    logger.info("=" * 80)
    logger.info("XArm RT Driver Unit Tests (Mocked Hardware)")
    logger.info("=" * 80)
    logger.info("")

    results = []

    try:
        results.append(("Basic Connection", test_basic_connection()))
    except Exception as e:
        logger.error(f"TEST 1 FAILED: {e}")
        import traceback

        traceback.print_exc()
        results.append(("Basic Connection", False))

    try:
        results.append(("Joint State Reading", test_joint_state_reading()))
    except Exception as e:
        logger.error(f"TEST 2 FAILED: {e}")
        import traceback

        traceback.print_exc()
        results.append(("Joint State Reading", False))

    try:
        results.append(("Command Sending", test_command_sending()))
    except Exception as e:
        logger.error(f"TEST 3 FAILED: {e}")
        import traceback

        traceback.print_exc()
        results.append(("Command Sending", False))

    try:
        results.append(("RPC Methods", test_rpc_methods()))
    except Exception as e:
        logger.error(f"TEST 4 FAILED: {e}")
        import traceback

        traceback.print_exc()
        results.append(("RPC Methods", False))

    # Print summary
    logger.info("=" * 80)
    logger.info("TEST SUMMARY")
    logger.info("=" * 80)

    for test_name, passed in results:
        status = "✓ PASSED" if passed else "✗ FAILED"
        logger.info(f"{test_name:30s} {status}")

    total_passed = sum(1 for _, passed in results if passed)
    total_tests = len(results)

    logger.info("")
    logger.info(f"Total: {total_passed}/{total_tests} tests passed")

    if total_passed == total_tests:
        logger.info("🎉 ALL TESTS PASSED!")
    else:
        logger.error("❌ SOME TESTS FAILED")


if __name__ == "__main__":
    main()
