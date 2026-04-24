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

"""Smoke tests for UnitreeGo2LowLevelAdapter.

Covers the hardware-free surface: module constants, construction, and
the "not connected" guard rails on every public write/flush/read path.
Connect-time tests are out of scope here — they need a live DDS partner
and a real Go2 in a damped state, which belongs in manual hardware QA.
"""

from __future__ import annotations

import pytest

# Skip the whole module when the SDK / cyclonedds C lib aren't loadable.
# `cyclonedds` import touches the C library, so a missing CYCLONEDDS_HOME
# or an incompatible glibc surfaces here as ImportError / OSError.
pytest.importorskip("unitree_sdk2py")
pytest.importorskip("cyclonedds")

from dimos.hardware.drive_trains.unitree_go2.adapter_lowlevel import (
    GO2_JOINT_INDEX,
    GO2_NUM_ACTIVE_JOINTS,
    GO2_NUM_MOTORS,
    UnitreeGo2LowLevelAdapter,
)


@pytest.fixture
def adapter():
    """Adapter instance without connect() — exercises only the local state."""
    a = UnitreeGo2LowLevelAdapter()
    yield a
    # disconnect() must be safe on an unconnected adapter (no watchdog thread).
    a.disconnect()


class TestModuleConstants:
    def test_motor_array_length(self):
        assert GO2_NUM_MOTORS == 20

    def test_active_joint_count(self):
        assert GO2_NUM_ACTIVE_JOINTS == 12

    def test_joint_index_covers_all_active_joints(self):
        assert len(GO2_JOINT_INDEX) == GO2_NUM_ACTIVE_JOINTS

    def test_joint_index_values_unique_and_contiguous(self):
        # Indices must be 0..11 with no gaps or duplicates — the adapter
        # writes into motor_cmd[idx] without remapping.
        assert sorted(GO2_JOINT_INDEX.values()) == list(range(GO2_NUM_ACTIVE_JOINTS))

    def test_joint_index_leg_order(self):
        # Canonical Unitree ordering: FR, FL, RR, RL, each [hip, thigh, calf].
        expected = {
            ("FR", "hip"): 0,
            ("FR", "thigh"): 1,
            ("FR", "calf"): 2,
            ("FL", "hip"): 3,
            ("FL", "thigh"): 4,
            ("FL", "calf"): 5,
            ("RR", "hip"): 6,
            ("RR", "thigh"): 7,
            ("RR", "calf"): 8,
            ("RL", "hip"): 9,
            ("RL", "thigh"): 10,
            ("RL", "calf"): 11,
        }
        assert GO2_JOINT_INDEX == expected


class TestConstruction:
    def test_default_construction(self, adapter):
        assert adapter.is_connected() is False

    def test_assume_dds_initialized_flag(self):
        a = UnitreeGo2LowLevelAdapter(assume_dds_initialized=True)
        try:
            assert a._assume_dds_initialized is True
            assert a.is_connected() is False
        finally:
            a.disconnect()


class TestNotConnectedGuards:
    """Every public mutator must refuse cleanly when not connected.

    These guards prevent a caller from staging or publishing motor
    commands against a dead adapter — important because connect() can
    fail silently (returns False) and the user might not check.
    """

    def test_write_joint_cmd_refuses(self, adapter):
        ok = adapter.write_joint_cmd(idx=0, q=0.0, kp=0.0, kd=1.0)
        assert ok is False

    def test_write_joint_array_refuses(self, adapter):
        zeros = [0.0] * GO2_NUM_ACTIVE_JOINTS
        ok = adapter.write_joint_array(zeros, zeros, zeros, zeros, zeros)
        assert ok is False

    def test_flush_refuses(self, adapter):
        assert adapter.flush() is False

    def test_emergency_damp_refuses(self, adapter):
        assert adapter.emergency_damp() is False

    def test_disconnect_idempotent(self, adapter):
        # Adapter fixture already calls disconnect() in teardown — calling
        # it twice must not raise.
        adapter.disconnect()
        adapter.disconnect()


class TestJointIndexBoundsCheck:
    """write_joint_cmd validates idx range. We can't reach the validation
    branch without first being connected, so we test the bounds via the
    public constants — the validation uses GO2_NUM_ACTIVE_JOINTS."""

    def test_first_active_index_is_zero(self):
        assert min(GO2_JOINT_INDEX.values()) == 0

    def test_last_active_index_is_one_less_than_count(self):
        assert max(GO2_JOINT_INDEX.values()) == GO2_NUM_ACTIVE_JOINTS - 1


class TestLowCmdWireFormat:
    """CRC + default-payload checks. The robot's motor controller rejects
    LowCmd_ messages with a wrong CRC, so any change to the wire-format
    construction or the CRC helper would surface here."""

    def test_default_lowcmd_header_bytes(self, adapter):
        cmd = adapter._build_lowcmd_defaults()
        assert cmd.head[0] == 0xFE
        assert cmd.head[1] == 0xEF
        assert cmd.level_flag == 0xFF  # LOWLEVEL
        assert cmd.gpio == 0

    def test_default_lowcmd_motors_disabled(self, adapter):
        cmd = adapter._build_lowcmd_defaults()
        for i in range(GO2_NUM_MOTORS):
            m = cmd.motor_cmd[i]
            assert m.mode == 0
            assert m.q == 0.0
            assert m.dq == 0.0
            assert m.tau == 0.0
            assert m.kp == 0.0
            assert m.kd == 0.0

    def test_crc_is_deterministic(self, adapter):
        # Identical payloads must produce identical CRCs — otherwise the
        # robot would reject every other message.
        cmd_a = adapter._build_lowcmd_defaults()
        cmd_b = adapter._build_lowcmd_defaults()
        crc_a = adapter._compute_crc(cmd_a)
        crc_b = adapter._compute_crc(cmd_b)
        assert crc_a == crc_b

    def test_crc_changes_with_payload(self, adapter):
        # CRC must be sensitive to motor_cmd contents — otherwise a
        # corrupted payload could pass validation on the robot side.
        cmd_a = adapter._build_lowcmd_defaults()
        cmd_b = adapter._build_lowcmd_defaults()
        cmd_b.motor_cmd[0].q = 1.0  # any non-default value
        assert adapter._compute_crc(cmd_a) != adapter._compute_crc(cmd_b)

    def test_crc_field_is_written(self, adapter):
        cmd = adapter._build_lowcmd_defaults()
        cmd.crc = 0  # ensure it isn't already set
        returned = adapter._compute_crc(cmd)
        assert cmd.crc == returned
        assert returned != 0  # extremely unlikely the default payload CRCs to 0
