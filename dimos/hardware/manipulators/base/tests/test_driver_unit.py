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

"""Unit tests for BaseManipulatorDriver.

These tests use MockSDK to test driver logic in isolation without hardware.
Run with: pytest dimos/hardware/manipulators/base/tests/test_driver_unit.py -v
"""

import math
import time

import pytest

from ..components import (
    StandardMotionComponent,
    StandardServoComponent,
    StandardStatusComponent,
)
from ..driver import BaseManipulatorDriver
from .conftest import MockSDK, MockSDKConfig

# =============================================================================
# Fixtures
# =============================================================================
# Note: mock_sdk and mock_sdk_with_positions fixtures are defined in conftest.py


@pytest.fixture
def standard_components():
    """Create standard component set."""
    return [
        StandardMotionComponent(),
        StandardServoComponent(),
        StandardStatusComponent(),
    ]


@pytest.fixture
def driver(mock_sdk, standard_components):
    """Create a driver with MockSDK and standard components."""
    config = {"dof": 6}
    driver = BaseManipulatorDriver(
        sdk=mock_sdk,
        components=standard_components,
        config=config,
        name="TestDriver",
    )
    yield driver
    # Cleanup - stop driver if running
    try:
        driver.stop()
    except Exception:
        pass


@pytest.fixture
def started_driver(driver):
    """Create and start a driver."""
    driver.start()
    time.sleep(0.05)  # Allow threads to start
    yield driver


# =============================================================================
# Connection Tests
# =============================================================================


class TestConnection:
    """Tests for driver connection behavior."""

    def test_driver_connects_on_init(self, mock_sdk, standard_components):
        """Driver should connect to SDK during initialization."""
        config = {"dof": 6}
        driver = BaseManipulatorDriver(
            sdk=mock_sdk,
            components=standard_components,
            config=config,
            name="TestDriver",
        )

        assert mock_sdk.connect_called
        assert mock_sdk.is_connected()
        assert driver.shared_state.is_connected

        driver.stop()

    @pytest.mark.skip(
        reason="Driver init failure leaks LCM threads - needs cleanup fix in Module base class"
    )
    def test_connection_failure_raises(self, standard_components):
        """Driver should raise if SDK connection fails."""
        config_fail = MockSDKConfig(connect_fails=True)
        mock_sdk = MockSDK(config=config_fail)

        with pytest.raises(RuntimeError, match="Failed to connect"):
            BaseManipulatorDriver(
                sdk=mock_sdk,
                components=standard_components,
                config={"dof": 6},
                name="TestDriver",
            )

    def test_disconnect_on_stop(self, started_driver, mock_sdk):
        """Driver should disconnect SDK on stop."""
        started_driver.stop()

        assert mock_sdk.disconnect_called
        assert not started_driver.shared_state.is_connected


# =============================================================================
# Joint State Tests
# =============================================================================


class TestJointState:
    """Tests for joint state reading."""

    def test_get_joint_state_returns_positions(self, driver):
        """get_joint_state should return current positions."""
        result = driver.get_joint_state()

        assert result["success"] is True
        assert len(result["positions"]) == 6
        assert len(result["velocities"]) == 6
        assert len(result["efforts"]) == 6

    def test_get_joint_state_with_custom_positions(self, standard_components):
        """get_joint_state should return SDK positions."""
        expected_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        mock_sdk = MockSDK(positions=expected_positions)

        driver = BaseManipulatorDriver(
            sdk=mock_sdk,
            components=standard_components,
            config={"dof": 6},
            name="TestDriver",
        )

        result = driver.get_joint_state()

        assert result["positions"] == expected_positions

        driver.stop()

    def test_shared_state_updated_on_joint_read(self, driver):
        """Shared state should be updated when reading joints."""
        # Manually trigger joint state update
        driver._update_joint_state()

        assert driver.shared_state.joint_positions is not None
        assert len(driver.shared_state.joint_positions) == 6


# =============================================================================
# Servo Control Tests
# =============================================================================


class TestServoControl:
    """Tests for servo enable/disable."""

    def test_enable_servo_calls_sdk(self, driver, mock_sdk):
        """enable_servo should call SDK's enable_servos."""
        result = driver.enable_servo()

        assert result["success"] is True
        assert mock_sdk.enable_servos_called

    def test_enable_servo_updates_shared_state(self, driver):
        """enable_servo should update shared state."""
        driver.enable_servo()

        # Trigger state update to sync
        driver._update_robot_state()

        assert driver.shared_state.is_enabled is True

    def test_disable_servo_calls_sdk(self, driver, mock_sdk):
        """disable_servo should call SDK's disable_servos."""
        driver.enable_servo()  # Enable first
        result = driver.disable_servo()

        assert result["success"] is True
        assert mock_sdk.disable_servos_called

    def test_enable_fails_with_error(self, standard_components):
        """enable_servo should return failure when SDK fails."""
        config = MockSDKConfig(enable_fails=True)
        mock_sdk = MockSDK(config=config)

        driver = BaseManipulatorDriver(
            sdk=mock_sdk,
            components=standard_components,
            config={"dof": 6},
            name="TestDriver",
        )

        result = driver.enable_servo()

        assert result["success"] is False

        driver.stop()


# =============================================================================
# Motion Control Tests
# =============================================================================


class TestMotionControl:
    """Tests for motion commands."""

    def test_move_joint_blocking_calls_sdk(self, driver, mock_sdk):
        """move_joint with wait=True should call SDK directly."""
        # Enable servos first (required for motion)
        driver.enable_servo()

        target = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        # Use wait=True to bypass queue and call SDK directly
        result = driver.move_joint(target, velocity=0.5, wait=True)

        assert result["success"] is True
        assert mock_sdk.set_joint_positions_called

        # Verify arguments
        call = mock_sdk.get_last_call("set_joint_positions")
        assert call is not None
        assert list(call.args[0]) == target
        assert call.kwargs["velocity"] == 0.5

    def test_move_joint_async_queues_command(self, driver, mock_sdk):
        """move_joint with wait=False should queue command."""
        # Enable servos first
        driver.enable_servo()

        target = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        # Default wait=False queues command
        result = driver.move_joint(target, velocity=0.5)

        assert result["success"] is True
        assert result.get("queued") is True
        # SDK not called yet (command is in queue)
        assert not mock_sdk.set_joint_positions_called
        # But command is in the queue
        assert not driver.command_queue.empty()

    def test_move_joint_fails_without_enable(self, driver, mock_sdk):
        """move_joint should fail if servos not enabled (blocking mode)."""
        target = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        # Use wait=True to test synchronous failure
        result = driver.move_joint(target, wait=True)

        assert result["success"] is False

    def test_move_joint_with_simulated_motion(self, standard_components):
        """With simulate_motion, positions should update (blocking mode)."""
        config = MockSDKConfig(simulate_motion=True)
        mock_sdk = MockSDK(config=config)

        driver = BaseManipulatorDriver(
            sdk=mock_sdk,
            components=standard_components,
            config={"dof": 6},
            name="TestDriver",
        )

        driver.enable_servo()
        target = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        # Use wait=True to execute directly
        driver.move_joint(target, wait=True)

        # Check SDK internal state updated
        assert mock_sdk.get_joint_positions() == target

        driver.stop()

    def test_stop_motion_calls_sdk(self, driver, mock_sdk):
        """stop_motion should call SDK's stop_motion."""
        result = driver.stop_motion()

        # stop_motion may return success=False if not moving, but should not error
        assert result is not None
        assert mock_sdk.stop_motion_called

    def test_process_command_calls_sdk(self, driver, mock_sdk):
        """_process_command should execute queued commands."""
        from ..driver import Command

        driver.enable_servo()

        # Create a position command directly
        command = Command(
            type="position",
            data={"positions": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], "velocity": 0.5},
        )

        # Process it directly
        driver._process_command(command)

        assert mock_sdk.set_joint_positions_called


# =============================================================================
# Robot State Tests
# =============================================================================


class TestRobotState:
    """Tests for robot state reading."""

    def test_get_robot_state_returns_state(self, driver):
        """get_robot_state should return state info."""
        result = driver.get_robot_state()

        assert result["success"] is True
        assert "state" in result
        assert "mode" in result
        assert "error_code" in result

    def test_get_robot_state_with_error(self, standard_components):
        """get_robot_state should report errors from SDK."""
        config = MockSDKConfig(error_code=42)
        mock_sdk = MockSDK(config=config)

        driver = BaseManipulatorDriver(
            sdk=mock_sdk,
            components=standard_components,
            config={"dof": 6},
            name="TestDriver",
        )

        result = driver.get_robot_state()

        assert result["error_code"] == 42

        driver.stop()

    def test_clear_errors_calls_sdk(self, driver, mock_sdk):
        """clear_errors should call SDK's clear_errors."""
        result = driver.clear_errors()

        assert result["success"] is True
        assert mock_sdk.clear_errors_called


# =============================================================================
# Joint Limits Tests
# =============================================================================


class TestJointLimits:
    """Tests for joint limit queries."""

    def test_get_joint_limits_returns_limits(self, driver):
        """get_joint_limits should return lower and upper limits."""
        result = driver.get_joint_limits()

        assert result["success"] is True
        assert len(result["lower"]) == 6
        assert len(result["upper"]) == 6

    def test_joint_limits_are_reasonable(self, driver):
        """Joint limits should be reasonable values."""
        result = driver.get_joint_limits()

        for lower, upper in zip(result["lower"], result["upper"], strict=False):
            assert lower < upper
            assert lower >= -2 * math.pi
            assert upper <= 2 * math.pi


# =============================================================================
# Capabilities Tests
# =============================================================================


class TestCapabilities:
    """Tests for driver capabilities."""

    def test_capabilities_from_sdk(self, driver):
        """Driver should get capabilities from SDK."""
        assert driver.capabilities.dof == 6
        assert len(driver.capabilities.max_joint_velocity) == 6
        assert len(driver.capabilities.joint_limits_lower) == 6

    def test_capabilities_with_different_dof(self, standard_components):
        """Driver should support different DOF arms."""
        mock_sdk = MockSDK(dof=7)

        driver = BaseManipulatorDriver(
            sdk=mock_sdk,
            components=standard_components,
            config={"dof": 7},
            name="TestDriver",
        )

        assert driver.capabilities.dof == 7
        assert len(driver.capabilities.max_joint_velocity) == 7

        driver.stop()


# =============================================================================
# Component API Exposure Tests
# =============================================================================


class TestComponentAPIExposure:
    """Tests for auto-exposed component APIs."""

    def test_motion_component_api_exposed(self, driver):
        """Motion component APIs should be exposed on driver."""
        assert hasattr(driver, "move_joint")
        assert hasattr(driver, "stop_motion")
        assert callable(driver.move_joint)

    def test_servo_component_api_exposed(self, driver):
        """Servo component APIs should be exposed on driver."""
        assert hasattr(driver, "enable_servo")
        assert hasattr(driver, "disable_servo")
        assert callable(driver.enable_servo)

    def test_status_component_api_exposed(self, driver):
        """Status component APIs should be exposed on driver."""
        assert hasattr(driver, "get_joint_state")
        assert hasattr(driver, "get_robot_state")
        assert hasattr(driver, "get_joint_limits")
        assert callable(driver.get_joint_state)


# =============================================================================
# Threading Tests
# =============================================================================


class TestThreading:
    """Tests for driver threading behavior."""

    def test_start_creates_threads(self, driver):
        """start() should create control threads."""
        driver.start()
        time.sleep(0.05)

        assert len(driver.threads) >= 2
        assert all(t.is_alive() for t in driver.threads)

        driver.stop()

    def test_stop_terminates_threads(self, started_driver):
        """stop() should terminate all threads."""
        started_driver.stop()
        time.sleep(0.1)

        assert all(not t.is_alive() for t in started_driver.threads)

    def test_stop_calls_sdk_stop_motion(self, started_driver, mock_sdk):
        """stop() should call SDK stop_motion."""
        started_driver.stop()

        assert mock_sdk.stop_motion_called


# =============================================================================
# Call Verification Tests (MockSDK features)
# =============================================================================


class TestMockSDKCallTracking:
    """Tests for MockSDK call tracking features."""

    def test_call_count(self, mock_sdk):
        """MockSDK should count method calls."""
        mock_sdk.get_joint_positions()
        mock_sdk.get_joint_positions()
        mock_sdk.get_joint_positions()

        assert mock_sdk.call_count("get_joint_positions") == 3

    def test_was_called(self, mock_sdk):
        """MockSDK.was_called should report if method called."""
        assert not mock_sdk.was_called("enable_servos")

        mock_sdk.enable_servos()

        assert mock_sdk.was_called("enable_servos")

    def test_get_last_call_args(self, mock_sdk):
        """MockSDK should record call arguments."""
        positions = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        mock_sdk.enable_servos()
        mock_sdk.set_joint_positions(positions, velocity=0.5, wait=True)

        call = mock_sdk.get_last_call("set_joint_positions")

        assert call is not None
        assert list(call.args[0]) == positions
        assert call.kwargs["velocity"] == 0.5
        assert call.kwargs["wait"] is True

    def test_reset_calls(self, mock_sdk):
        """MockSDK.reset_calls should clear call history."""
        mock_sdk.enable_servos()
        mock_sdk.get_joint_positions()

        mock_sdk.reset_calls()

        assert mock_sdk.call_count("enable_servos") == 0
        assert mock_sdk.call_count("get_joint_positions") == 0
        assert not mock_sdk.enable_servos_called


# =============================================================================
# Edge Case Tests
# =============================================================================


class TestEdgeCases:
    """Tests for edge cases and error handling."""

    def test_multiple_enable_calls_optimized(self, driver):
        """Multiple enable calls should only call SDK once (optimization)."""
        result1 = driver.enable_servo()
        result2 = driver.enable_servo()
        result3 = driver.enable_servo()

        # All calls succeed
        assert result1["success"] is True
        assert result2["success"] is True
        assert result3["success"] is True

        # But SDK only called once (component optimizes redundant calls)
        assert driver.sdk.call_count("enable_servos") == 1

        # Second and third calls should indicate already enabled
        assert result2.get("message") == "Servos already enabled"
        assert result3.get("message") == "Servos already enabled"

    def test_disable_when_already_disabled(self, driver):
        """Disable when already disabled should return success without SDK call."""
        # MockSDK starts with servos disabled
        result = driver.disable_servo()

        assert result["success"] is True
        assert result.get("message") == "Servos already disabled"
        # SDK not called since already disabled
        assert not driver.sdk.disable_servos_called

    def test_disable_after_enable(self, driver):
        """Disable after enable should call SDK."""
        driver.enable_servo()
        result = driver.disable_servo()

        assert result["success"] is True
        assert driver.sdk.disable_servos_called

    def test_emergency_stop(self, driver):
        """emergency_stop should disable servos."""
        driver.enable_servo()

        driver.sdk.emergency_stop()

        assert driver.sdk.emergency_stop_called
        assert not driver.sdk.are_servos_enabled()
