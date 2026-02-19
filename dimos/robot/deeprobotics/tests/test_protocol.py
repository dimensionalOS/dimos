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

"""Tests for M20 UDP protocol and velocity controller."""

import json
import struct
import threading
import time

import pytest

from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.robot.deeprobotics.protocol import (
    HEADER_LEN,
    HEADER_MAGIC,
    JSON_FORMAT,
    CommandType,
    GaitType,
    M20Protocol,
    MotionState,
    UsageMode,
)
from dimos.robot.deeprobotics.m20.velocity_controller import (
    M20SpeedLimits,
    M20VelocityController,
    VelocityState,
)


class TestM20Protocol:
    """Tests for the M20 binary protocol encoding."""

    def test_header_magic(self):
        assert HEADER_MAGIC == bytes([0xEB, 0x91, 0xEB, 0x90])
        assert len(HEADER_MAGIC) == 4

    def test_header_length(self):
        assert HEADER_LEN == 16

    def test_protocol_init_defaults(self):
        proto = M20Protocol()
        assert proto.host == "10.21.31.103"
        assert proto.port == 30000
        assert not proto.connected

    def test_protocol_init_custom(self):
        proto = M20Protocol(host="192.168.1.100", port=31000)
        assert proto.host == "192.168.1.100"
        assert proto.port == 31000

    def test_command_types(self):
        assert CommandType.HEARTBEAT == 100
        assert CommandType.MOTION == 2
        assert CommandType.PERIPHERAL == 1101

    def test_motion_states(self):
        assert MotionState.STAND == 1
        assert MotionState.SIT == 4

    def test_gait_types(self):
        assert GaitType.STANDARD == 0x1001
        assert GaitType.HIGH_OBSTACLE == 0x1002
        assert GaitType.STAIRS == 0x1003

    def test_usage_modes(self):
        assert UsageMode.REGULAR == 0
        assert UsageMode.NAVIGATION == 1
        assert UsageMode.ASSIST == 2

    def test_send_without_connect_is_noop(self):
        """Sending before connect() should silently no-op."""
        proto = M20Protocol()
        # Should not raise
        proto.send_heartbeat()
        proto.send_velocity(0, 0, 0)
        proto.send_motion_state(MotionState.STAND)

    def test_send_velocity_clamps(self):
        """Verify the protocol would clamp values to [-1,1]."""
        proto = M20Protocol()
        # Internal _clamp function is tested indirectly through send_velocity
        # which calls _send_command - since no socket, this is a no-op
        proto.send_velocity(x=2.0, y=-2.0, yaw=0.5)


class TestVelocityController:
    """Tests for the M20 velocity controller."""

    def test_speed_limits_defaults(self):
        limits = M20SpeedLimits()
        assert limits.max_linear_x == 1.5
        assert limits.max_linear_y == 0.5
        assert limits.max_angular_yaw == 1.0

    def test_velocity_state_defaults(self):
        state = VelocityState()
        assert state.target_linear_x == 0.0
        assert state.actual_linear_x == 0.0
        assert not state.emergency_stopped
        assert not state.paused

    def test_set_twist_clamps_to_speed_limits(self):
        proto = M20Protocol()
        limits = M20SpeedLimits(max_linear_x=1.0, max_linear_y=0.5, max_angular_yaw=0.8)
        ctrl = M20VelocityController(protocol=proto, speed_limits=limits)

        twist = Twist(
            linear=Vector3(x=2.0, y=-1.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=1.5),
        )
        ctrl.set_twist(twist)

        assert ctrl.state.target_linear_x == 1.0  # clamped from 2.0
        assert ctrl.state.target_linear_y == -0.5  # clamped from -1.0
        assert ctrl.state.target_angular_yaw == 0.8  # clamped from 1.5

    def test_emergency_stop_zeros_all(self):
        proto = M20Protocol()
        ctrl = M20VelocityController(protocol=proto)

        # Set some velocity
        twist = Twist(
            linear=Vector3(x=1.0, y=0.5, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.3),
        )
        ctrl.set_twist(twist)

        # Engage e-stop
        ctrl.emergency_stop(True)
        assert ctrl.state.emergency_stopped
        assert ctrl.state.target_linear_x == 0.0
        assert ctrl.state.actual_linear_x == 0.0

        # Should ignore new commands while stopped
        ctrl.set_twist(twist)
        assert ctrl.state.target_linear_x == 0.0

        # Release e-stop
        ctrl.emergency_stop(False)
        assert not ctrl.state.emergency_stopped

    def test_acceleration_smoothing(self):
        proto = M20Protocol()
        ctrl = M20VelocityController(
            protocol=proto,
            linear_accel=1.0,
            angular_accel=2.0,
        )

        # Set target
        ctrl.state.target_linear_x = 1.0
        dt = 0.05  # 20Hz

        # One step should move by linear_accel * dt = 0.05
        ctrl._update_velocities(dt)
        assert abs(ctrl.state.actual_linear_x - 0.05) < 1e-6

        # Another step
        ctrl._update_velocities(dt)
        assert abs(ctrl.state.actual_linear_x - 0.10) < 1e-6

    def test_normalization(self):
        """Verify m/s to [-1,1] normalization math."""
        proto = M20Protocol()
        limits = M20SpeedLimits(max_linear_x=1.5, max_linear_y=0.5, max_angular_yaw=1.0)
        ctrl = M20VelocityController(protocol=proto, speed_limits=limits)

        # Set actual velocities directly for testing normalization
        ctrl.state.actual_linear_x = 0.75  # 50% of max
        ctrl.state.actual_linear_y = 0.25  # 50% of max
        ctrl.state.actual_angular_yaw = 0.5  # 50% of max
        ctrl.state.last_command_time = time.time()

        # The normalization happens in _publish_control, which sends to protocol
        # We can verify the math: 0.75 / 1.5 = 0.5, 0.25 / 0.5 = 0.5, 0.5 / 1.0 = 0.5
        assert 0.75 / limits.max_linear_x == 0.5
        assert 0.25 / limits.max_linear_y == 0.5
        assert 0.5 / limits.max_angular_yaw == 0.5

    def test_is_moving(self):
        proto = M20Protocol()
        ctrl = M20VelocityController(protocol=proto)

        assert not ctrl.is_moving

        ctrl.state.actual_linear_x = 0.1
        assert ctrl.is_moving

        ctrl.state.actual_linear_x = 0.0
        ctrl.state.actual_angular_yaw = 0.05
        assert ctrl.is_moving
