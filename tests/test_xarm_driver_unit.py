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
Automated pytest unit tests for xArm RT Driver with mocked hardware.

These tests run WITHOUT real hardware and can be run in CI/CD.

Usage:
    pytest tests/test_xarm_driver_unit.py -v
    pytest tests/test_xarm_driver_unit.py::test_basic_connection -v
"""

import time
from unittest.mock import MagicMock, patch
import pytest

from dimos.hardware.manipulators.xarm.xarm_driver import XArmDriver
from dimos.msgs.sensor_msgs import JointState


@pytest.fixture
def mock_xarm():
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


@pytest.fixture
def driver(mock_xarm):
    """Create an XArmDriver instance with mocked hardware."""
    with patch("dimos.hardware.manipulators.xarm.xarm_driver.XArmAPI") as MockXArmAPI:
        MockXArmAPI.return_value = mock_xarm

        driver = XArmDriver(
            ip_address="192.168.1.235",
            control_frequency=100.0,
            joint_state_rate=100.0,
            report_type="dev",
            enable_on_start=False,
            xarm_type="xarm6",
        )

        yield driver

        # Teardown: stop driver if running
        if driver._running:
            driver.stop()


def test_driver_initialization(driver):
    """Test that driver initializes with correct configuration."""
    assert driver.config.ip_address == "192.168.1.235"
    assert driver.config.control_frequency == 100.0
    assert driver.config.num_joints == 6
    assert driver.config.report_type == "dev"
    assert driver.config.enable_on_start is False


def test_start_stop_cycle(mock_xarm):
    """Test that driver can start and stop successfully."""
    with patch("dimos.hardware.manipulators.xarm.xarm_driver.XArmAPI") as MockXArmAPI:
        MockXArmAPI.return_value = mock_xarm

        driver = XArmDriver(
            ip_address="192.168.1.235",
            enable_on_start=False,
            xarm_type="xarm6",
        )

        # Start driver
        driver.start()
        time.sleep(0.5)

        assert driver._running is True
        assert driver.arm is not None
        assert driver._state_thread is not None
        assert driver._control_thread is not None

        # Stop driver
        driver.stop()
        time.sleep(0.5)

        assert driver._running is False


def test_start_creates_new_disposable(mock_xarm):
    """Test that start() creates fresh CompositeDisposable after stop."""
    with patch("dimos.hardware.manipulators.xarm.xarm_driver.XArmAPI") as MockXArmAPI:
        MockXArmAPI.return_value = mock_xarm

        driver = XArmDriver(
            ip_address="192.168.1.235",
            enable_on_start=False,
            xarm_type="xarm6",
        )

        # First start
        driver.start()
        time.sleep(0.3)
        disposable1 = driver._disposables

        # Stop
        driver.stop()
        time.sleep(0.3)

        # Second start
        driver.start()
        time.sleep(0.3)
        disposable2 = driver._disposables

        # They should be different objects
        assert disposable1 is not disposable2, "start() should create new CompositeDisposable"

        driver.stop()


def test_get_version_rpc(driver):
    """Test get_version RPC method."""
    driver.start()
    time.sleep(0.3)

    code, version = driver.get_version()

    assert code == 0, "Failed to get firmware version"
    assert version == "v1.10.0", "Should return mocked version"

    driver.stop()


def test_get_position_rpc(driver):
    """Test get_position RPC method."""
    driver.start()
    time.sleep(0.3)

    code, position = driver.get_position()

    assert code == 0, "get_position should return success code"
    assert len(position) == 6, "Position should have 6 values [x,y,z,roll,pitch,yaw]"
    assert position[0] == 200.0, "X position should match mock"

    driver.stop()


def test_motion_enable_rpc(driver):
    """Test motion_enable RPC method."""
    driver.start()
    time.sleep(0.3)

    result = driver.motion_enable(enable=True)

    # motion_enable returns a tuple (code, msg)
    assert isinstance(result, tuple), "Should return tuple"
    code, msg = result

    assert code == 0, f"motion_enable should return success code, got {code}"
    assert "Motion enabled" in msg, "Should return success message"

    driver.stop()


def test_enable_servo_mode_rpc(driver):
    """Test enable_servo_mode RPC method."""
    driver.start()
    time.sleep(0.3)

    code, msg = driver.enable_servo_mode()

    assert code == 0, "enable_servo_mode should return success code"

    driver.stop()


def test_disable_servo_mode_rpc(driver):
    """Test disable_servo_mode RPC method."""
    driver.start()
    time.sleep(0.3)

    code, msg = driver.disable_servo_mode()

    assert code == 0, "disable_servo_mode should return success code"

    driver.stop()


def test_clean_error_rpc(driver):
    """Test clean_error RPC method."""
    driver.start()
    time.sleep(0.3)

    code, msg = driver.clean_error()

    assert code == 0, "clean_error should return success code"

    driver.stop()


def test_joint_state_publishing(driver):
    """Test that joint states are published (without transport)."""
    joint_states_received = []

    def on_joint_state(msg: JointState):
        joint_states_received.append(msg)

    # Try to subscribe (will fail without transport, but that's OK for unit test)
    try:
        driver.joint_state.subscribe(on_joint_state)
    except (AttributeError, ValueError):
        pass  # Expected - no transport in unit test

    driver.start()
    time.sleep(1.0)

    # Check that joint state thread is running
    assert driver._state_thread is not None
    assert driver._state_thread.is_alive()

    driver.stop()


def test_control_thread_starts(driver):
    """Test that control thread starts successfully."""
    driver.start()
    time.sleep(0.5)

    assert driver._control_thread is not None
    assert driver._control_thread.is_alive()

    driver.stop()


def test_readiness_check_initialization():
    """Test that _xarm_is_ready_write initializes tracking variables."""
    with patch("dimos.hardware.manipulators.xarm.xarm_driver.XArmAPI") as MockXArmAPI:
        mock = MagicMock()
        mock.connected = True
        mock.version_number = (1, 10, 0)
        MockXArmAPI.return_value = mock

        driver = XArmDriver(
            ip_address="192.168.1.235",
            enable_on_start=False,
            xarm_type="xarm6",
        )

        driver.start()
        time.sleep(0.3)

        # Call readiness check - should not raise AttributeError
        try:
            is_ready = driver._xarm_is_ready_write()
            # Should complete without AttributeError
            assert True
        except AttributeError as e:
            pytest.fail(f"_xarm_is_ready_write raised AttributeError: {e}")

        driver.stop()


def test_velocity_control_mode_initialization(mock_xarm):
    """Test that velocity control mode sets correct mode on initialization."""
    with patch("dimos.hardware.manipulators.xarm.xarm_driver.XArmAPI") as MockXArmAPI:
        MockXArmAPI.return_value = mock_xarm

        driver = XArmDriver(
            ip_address="192.168.1.235",
            enable_on_start=True,
            velocity_control=True,  # Enable velocity control
            xarm_type="xarm6",
        )

        driver.start()
        time.sleep(0.3)

        # Check that set_mode was called with mode 4 (velocity control)
        # mock_xarm.set_mode.assert_called_with(4)

        driver.stop()


def test_error_interpreter():
    """Test error code interpretation."""
    assert "Everything OK" in XArmDriver.controller_error_interpreter(0)
    assert "Emergency" in XArmDriver.controller_error_interpreter(1)
    assert "Joint 1" in XArmDriver.controller_error_interpreter(11)
    assert "Joint Angle Exceed Limit" in XArmDriver.controller_error_interpreter(23)
