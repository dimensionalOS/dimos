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

"""Tests for SimMujocoAdapter and gripper integration."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from dimos.hardware.manipulators.sim.adapter import SimMujocoAdapter, register
from dimos.simulation.conftest import ARM_DOF


class TestSimMujocoAdapter:
    """Tests for SimMujocoAdapter with and without gripper."""

    @pytest.fixture
    def adapter_with_gripper(self, patched_mujoco_engine_with_gripper):
        """SimMujocoAdapter with ARM_DOF arm joints + 1 gripper joint."""
        return SimMujocoAdapter(dof=ARM_DOF, address="/fake/scene.xml", headless=True)

    @pytest.fixture
    def adapter_no_gripper(self, patched_mujoco_engine):
        """SimMujocoAdapter with ARM_DOF arm joints, no gripper."""
        return SimMujocoAdapter(dof=ARM_DOF, address="/fake/scene.xml", headless=True)

    def test_address_required(self, patched_mujoco_engine):
        with pytest.raises(ValueError, match="address"):
            SimMujocoAdapter(dof=ARM_DOF, address=None)

    def test_gripper_detected(self, adapter_with_gripper):
        assert adapter_with_gripper._gripper_idx == ARM_DOF

    def test_no_gripper_when_dof_matches(self, adapter_no_gripper):
        assert adapter_no_gripper._gripper_idx is None

    def test_read_gripper_position(self, adapter_with_gripper):
        pos = adapter_with_gripper.read_gripper_position()
        assert pos is not None

    def test_write_gripper_sets_target(self, adapter_with_gripper):
        """Write a gripper position and verify the control target was set."""
        assert adapter_with_gripper.write_gripper_position(0.42) is True
        target = adapter_with_gripper._engine._joint_position_targets[ARM_DOF]
        assert target != 0.0, "write_gripper_position should update the control target"

    def test_read_gripper_position_no_gripper(self, adapter_no_gripper):
        assert adapter_no_gripper.read_gripper_position() is None

    def test_write_gripper_position_no_gripper(self, adapter_no_gripper):
        assert adapter_no_gripper.write_gripper_position(0.5) is False

    def test_write_gripper_does_not_clobber_arm(self, adapter_with_gripper):
        """Gripper write must not overwrite arm joint targets."""
        engine = adapter_with_gripper._engine
        for i in range(ARM_DOF):
            engine._joint_position_targets[i] = float(i) + 1.0

        adapter_with_gripper.write_gripper_position(0.0)

        for i in range(ARM_DOF):
            assert engine._joint_position_targets[i] == pytest.approx(float(i) + 1.0)

    def test_read_joint_positions_excludes_gripper(self, adapter_with_gripper):
        positions = adapter_with_gripper.read_joint_positions()
        assert len(positions) == ARM_DOF

    def test_connect_and_disconnect(self, adapter_with_gripper):
        with patch("dimos.simulation.engines.mujoco_engine.mujoco.mj_step"):
            assert adapter_with_gripper.connect() is True
            adapter_with_gripper.disconnect()

    def test_register(self):
        registry = MagicMock()
        register(registry)
        registry.register.assert_called_once_with("sim_mujoco", SimMujocoAdapter)
