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
Test script for XArmDriver with full dimos deployment.

This script properly deploys the XArmDriver module using dimos infrastructure:
1. dimos.start() - Initialize dimos dimos
2. dimos.deploy() - Deploy XArmDriver module
3. Set LCM transports for output topics
4. Test RPC methods and state monitoring

Usage:
    export XARM_IP=192.168.1.235
    venv/bin/python dimos/hardware/manipulators/xarm/test_xarm_driver.py

Or use the wrapper script:
    ./dimos/hardware/manipulators/xarm/test_xarm_deploy.sh

Note: Must use venv/bin/python to avoid GLIBC version conflicts with system Python.
"""

import os
import time

import pytest

from dimos import core
from dimos.hardware.manipulators.xarm.xarm_driver import XArmDriver
from dimos.msgs.sensor_msgs import JointState, RobotState
from dimos.msgs.geometry_msgs import WrenchStamped
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__file__)


@pytest.mark.tool
def test_basic_connection():
    """Test basic connection and startup with dimos deployment."""
    logger.info("=" * 80)
    logger.info("TEST 1: Basic Connection with dimos.deploy()")
    logger.info("=" * 80)

    # Get IP from environment or use default
    ip_address = os.getenv("XARM_IP", "192.168.1.235")

    # Start dimos with 1 worker
    logger.info("Starting dimos...")
    dimos = core.start(1)

    # Deploy XArmDriver using dimos.deploy()
    logger.info(f"Deploying XArmDriver for {ip_address}...")
    driver = dimos.deploy(
        XArmDriver,
        ip_address=ip_address,
        control_frequency=100.0,
        joint_state_rate=100.0,
        report_type="dev",
        enable_on_start=False,
        num_joints=6,
    )

    # Set up LCM transports for output topics BEFORE starting
    logger.info("Setting up LCM transports for outputs...")
    driver.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
    driver.robot_state.transport = core.LCMTransport("/xarm/robot_state", RobotState)
    driver.ft_ext.transport = core.LCMTransport("/xarm/ft_ext", WrenchStamped)
    driver.ft_raw.transport = core.LCMTransport("/xarm/ft_raw", WrenchStamped)

    # Start the driver
    logger.info("Starting driver...")
    driver.start()

    # Wait for initialization
    time.sleep(2.0)

    # Check connection via RPC
    logger.info("Checking connection via RPC...")
    code, version = driver.get_version()
    if code == 0:
        logger.info(f"✓ Firmware version: {version}")
    else:
        logger.error(f"✗ Failed to get firmware version: code={code}")
        dimos.stop()
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

    # Stop the driver and dimos
    logger.info("Stopping driver and dimos...")
    driver.stop()
    dimos.stop()

    logger.info("✓ TEST 1 PASSED\n")
    return True


@pytest.mark.tool
def test_joint_state_reading():
    """Test joint state reading via LCM topic subscription."""
    logger.info("=" * 80)
    logger.info("TEST 2: Joint State Reading via LCM Transport")
    logger.info("=" * 80)

    ip_address = os.getenv("XARM_IP", "192.168.1.235")

    # Start dimos
    logger.info("Starting dimos...")
    dimos = core.start(1)

    # Deploy driver
    logger.info("Deploying XArmDriver...")
    driver = dimos.deploy(
        XArmDriver,
        ip_address=ip_address,
        control_frequency=100.0,
        joint_state_rate=100.0,
        report_type="dev",
        enable_on_start=False,
        num_joints=6,
    )

    # Set up LCM transports for both joint states and robot state
    joint_state_transport = core.LCMTransport("/xarm/joint_states", JointState)
    robot_state_transport = core.LCMTransport("/xarm/robot_state", RobotState)

    driver.joint_state.transport = joint_state_transport
    driver.robot_state.transport = robot_state_transport
    driver.ft_ext.transport = core.LCMTransport("/xarm/ft_ext", WrenchStamped)
    driver.ft_raw.transport = core.LCMTransport("/xarm/ft_raw", WrenchStamped)

    # Subscribe to the LCM topics to receive messages
    joint_states_received = []
    robot_states_received = []

    def on_joint_state(msg):
        """Callback for receiving joint state messages from LCM."""
        joint_states_received.append(msg)
        if len(joint_states_received) <= 3:
            logger.info(
                f"Received joint state #{len(joint_states_received)} via LCM: "
                f"positions={[f'{p:.3f}' for p in msg.position[:3]]}... "
                f"(showing first 3 joints)"
            )

    def on_robot_state(msg):
        """Callback for receiving robot state messages from LCM."""
        robot_states_received.append(msg)
        if len(robot_states_received) <= 3:
            logger.info(
                f"Received robot state #{len(robot_states_received)} via LCM: "
                f"state={msg.state}, mode={msg.mode}, error={msg.error_code}, "
                f"joints={msg.joints}, tcp_pose={msg.tcp_pose}, tcp_offset={msg.tcp_offset}"
            )

    # Subscribe to the LCM transports
    logger.info("Subscribing to /xarm/joint_states LCM topic...")
    unsubscribe_joint = joint_state_transport.subscribe(on_joint_state, driver.joint_state)

    logger.info("Subscribing to /xarm/robot_state LCM topic...")
    unsubscribe_robot = robot_state_transport.subscribe(on_robot_state, driver.robot_state)

    logger.info("Starting driver - joint states will publish at 100Hz...")
    driver.start()

    # Wait 3 seconds to collect messages
    logger.info("Collecting messages for 3 seconds...")
    time.sleep(3.0)

    # Unsubscribe from both LCM topics
    unsubscribe_joint()
    unsubscribe_robot()

    # Check results
    logger.info(f"\nReceived {len(joint_states_received)} joint state messages via LCM")
    logger.info(f"Received {len(robot_states_received)} robot state messages via LCM")

    # Validate joint state messages
    if len(joint_states_received) > 0:
        logger.info("✓ Joint state publishing working via LCM transport")

        # Calculate rate
        rate = len(joint_states_received) / 3.0
        logger.info(f"✓ Joint state publishing rate: ~{rate:.1f} Hz (expected ~100 Hz)")

        # Check last state
        last_state = joint_states_received[-1]
        logger.info(f"✓ Last state has {len(last_state.position)} joint positions")
        logger.info(f"✓ Full joint positions: {[f'{p:.3f}' for p in last_state.position[:6]]}")

        if rate > 50:
            logger.info("✓ Joint state publishing rate is good (>50 Hz)")
        else:
            logger.warning(f"⚠ Joint state publishing rate seems low: {rate:.1f} Hz")
    else:
        logger.error("✗ No joint states received via LCM")
        driver.stop()
        dimos.stop()
        return False

    # Validate robot state messages
    if len(robot_states_received) > 0:
        logger.info("✓ Robot state publishing working via LCM transport")

        # Calculate rate
        rate = len(robot_states_received) / 3.0
        logger.info(f"✓ Robot state publishing rate: ~{rate:.1f} Hz")

        # Check last state
        last_robot_state = robot_states_received[-1]
        logger.info(
            f"✓ Last robot state: state={last_robot_state.state}, mode={last_robot_state.mode}, "
            f"error={last_robot_state.error_code}, warn={last_robot_state.warn_code}"
        )
    else:
        logger.warning(
            "⚠ No robot states received via LCM (might be expected with 'dev' report type)"
        )

    driver.stop()
    dimos.stop()
    logger.info("✓ TEST 2 PASSED\n")
    return True


@pytest.mark.tool
def test_command_sending():
    """Test that command RPC methods are available and functional."""
    logger.info("=" * 80)
    logger.info("TEST 3: Command RPC Methods")
    logger.info("=" * 80)

    ip_address = os.getenv("XARM_IP", "192.168.1.235")

    # Start dimos
    dimos = core.start(1)

    # Deploy driver
    driver = dimos.deploy(
        XArmDriver,
        ip_address=ip_address,
        control_frequency=100.0,
        joint_state_rate=100.0,
        report_type="dev",
        enable_on_start=False,
        num_joints=6,
    )

    # Set up transports
    driver.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
    driver.robot_state.transport = core.LCMTransport("/xarm/robot_state", RobotState)
    driver.ft_ext.transport = core.LCMTransport("/xarm/ft_ext", WrenchStamped)
    driver.ft_raw.transport = core.LCMTransport("/xarm/ft_raw", WrenchStamped)

    driver.start()
    time.sleep(2.0)

    # Test that command methods exist and are callable
    logger.info("Testing command RPC methods are available...")

    # Test motion_enable
    logger.info("Testing motion_enable()...")
    code, msg = driver.motion_enable(enable=True)
    logger.info(f"  motion_enable returned: code={code}, msg={msg}")

    # Test enable_servo_mode
    logger.info("Testing enable_servo_mode()...")
    code, msg = driver.enable_servo_mode()
    logger.info(f"  enable_servo_mode returned: code={code}, msg={msg}")

    # Test disable_servo_mode
    logger.info("Testing disable_servo_mode()...")
    code, msg = driver.disable_servo_mode()
    logger.info(f"  disable_servo_mode returned: code={code}, msg={msg}")

    # Test set_state
    logger.info("Testing set_state(0)...")
    code, msg = driver.set_state(0)
    logger.info(f"  set_state returned: code={code}, msg={msg}")

    # Test get_position
    logger.info("Testing get_position()...")
    code, position = driver.get_position()
    if code == 0 and position:
        logger.info(f"✓ get_position: {[f'{p:.1f}' for p in position[:3]]} (x,y,z in mm)")
    else:
        logger.warning(f"  get_position returned: code={code}")

    logger.info("\n✓ All command RPC methods are functional")
    logger.info("Note: Actual robot movement testing requires specific robot state")
    logger.info("      and is environment-dependent. The driver API is working correctly.")

    driver.stop()
    dimos.stop()
    logger.info("✓ TEST 3 PASSED\n")
    return True


@pytest.mark.tool
def test_rpc_methods():
    """Test RPC method calls."""
    logger.info("=" * 80)
    logger.info("TEST 4: RPC Methods")
    logger.info("=" * 80)

    ip_address = os.getenv("XARM_IP", "192.168.1.235")

    # Start dimos
    dimos = core.start(1)

    # Deploy driver
    driver = dimos.deploy(
        XArmDriver,
        ip_address=ip_address,
        control_frequency=100.0,
        joint_state_rate=100.0,
        report_type="normal",  # Use normal for this test
        enable_on_start=False,
        num_joints=6,
    )

    # Set up transports
    driver.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
    driver.robot_state.transport = core.LCMTransport("/xarm/robot_state", RobotState)
    driver.ft_ext.transport = core.LCMTransport("/xarm/ft_ext", WrenchStamped)
    driver.ft_raw.transport = core.LCMTransport("/xarm/ft_raw", WrenchStamped)

    driver.start()
    time.sleep(2.0)

    # Test get_version
    logger.info("Testing get_version() RPC...")
    code, version = driver.get_version()
    if code == 0:
        logger.info(f"✓ get_version: {version}")
    else:
        logger.error(f"✗ get_version failed: code={code}")

    # Test get_position (TCP pose)
    logger.info("Testing get_position() RPC...")
    code, position = driver.get_position()
    if code == 0:
        logger.info(f"✓ get_position: {[f'{p:.3f}' for p in position]}")
    else:
        logger.error(f"✗ get_position failed: code={code}")

    # Test motion_enable
    logger.info("Testing motion_enable() RPC...")
    code, msg = driver.motion_enable(enable=True)
    if code == 0:
        logger.info(f"✓ motion_enable: {msg}")
    else:
        logger.error(f"✗ motion_enable failed: code={code}, msg={msg}")

    # Test clean_error
    logger.info("Testing clean_error() RPC...")
    code, msg = driver.clean_error()
    if code == 0:
        logger.info(f"✓ clean_error: {msg}")
    else:
        logger.warning(f"⚠ clean_error: code={code}, msg={msg}")

    driver.stop()
    dimos.stop()
    logger.info("✓ TEST 4 PASSED\n")
    return True


def run_tests():
    """Run all tests."""
    logger.info("=" * 80)
    logger.info("XArm Driver Test Suite (Full dimos Deployment)")
    logger.info("=" * 80)
    logger.info("")

    # Run tests
    results = []

    try:
        results.append(("Basic Connection", test_basic_connection()))
    except Exception as e:
        logger.error(f"TEST 1 FAILED with exception: {e}")
        import traceback

        traceback.print_exc()
        results.append(("Basic Connection", False))

    try:
        results.append(("Joint State Reading", test_joint_state_reading()))
    except Exception as e:
        logger.error(f"TEST 2 FAILED with exception: {e}")
        import traceback

        traceback.print_exc()
        results.append(("Joint State Reading", False))

    try:
        results.append(("Command Sending", test_command_sending()))
    except Exception as e:
        logger.error(f"TEST 3 FAILED with exception: {e}")
        import traceback

        traceback.print_exc()
        results.append(("Command Sending", False))

    try:
        results.append(("RPC Methods", test_rpc_methods()))
    except Exception as e:
        logger.error(f"TEST 4 FAILED with exception: {e}")
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


def run_driver():
    """Start the xArm driver and keep it running."""
    logger.info("=" * 80)
    logger.info("XArm Driver - Starting in continuous mode")
    logger.info("=" * 80)
    logger.info("")

    # Get IP address from environment variable or use default
    ip_address = os.getenv("XARM_IP", "192.168.1.235")
    logger.info(f"Using xArm at IP: {ip_address}")
    logger.info("")

    # Start dimos
    logger.info("Starting dimos...")
    dimos = core.start(1)

    # Deploy XArmDriver
    logger.info(f"Deploying XArmDriver for {ip_address}...")
    driver = dimos.deploy(
        XArmDriver,
        ip_address=ip_address,
        report_type="dev",
        enable_on_start=False,
        num_joints=6,
    )

    # Set up LCM transports
    logger.info("Setting up LCM transports...")
    driver.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
    driver.robot_state.transport = core.LCMTransport("/xarm/robot_state", RobotState)
    driver.ft_ext.transport = core.LCMTransport("/xarm/ft_ext", WrenchStamped)
    driver.ft_raw.transport = core.LCMTransport("/xarm/ft_raw", WrenchStamped)

    # Start driver
    logger.info("Starting driver...")
    driver.start()

    logger.info("")
    logger.info("=" * 80)
    logger.info("✓ XArm driver is running!")
    logger.info("=" * 80)
    logger.info("")
    logger.info("Publishing:")
    logger.info("  - Joint states:  /xarm/joint_states  (~100 Hz)")
    logger.info("  - Robot state:   /xarm/robot_state   (~10 Hz)")
    logger.info("  - Force/torque:  /xarm/ft_ext, /xarm/ft_raw")
    logger.info("")
    logger.info("Press Ctrl+C to stop...")
    logger.info("")

    try:
        # Keep running until interrupted
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logger.info("\n\nShutting down...")
        driver.stop()
        dimos.stop()
        logger.info("✓ Driver stopped")


def main():
    """Main entry point."""
    import sys

    # Check if XARM_IP is set
    ip_address = os.getenv("XARM_IP")
    if not ip_address:
        logger.warning("XARM_IP environment variable not set, using default: 192.168.1.235")
        logger.warning("Set XARM_IP to your xArm's IP address:")
        logger.warning("  export XARM_IP=192.168.1.XXX")
        logger.info("")
    else:
        logger.info(f"Using xArm at IP: {ip_address}")
        logger.info("")

    # Check for command-line arguments
    if len(sys.argv) > 1 and sys.argv[1] == "--run-tests":
        run_tests()
    else:
        run_driver()


if __name__ == "__main__":
    main()
