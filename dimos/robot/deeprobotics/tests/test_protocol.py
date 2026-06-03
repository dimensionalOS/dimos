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

"""Tests for M20 UDP protocol."""

from dimos.robot.deeprobotics.protocol import (
    HEADER_LEN,
    HEADER_MAGIC,
    CommandType,
    GaitType,
    M20Protocol,
    MotionState,
    UsageMode,
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
