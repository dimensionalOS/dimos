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

"""Tests for SimMujocoAdapter."""

from __future__ import annotations

import threading
from unittest.mock import MagicMock, patch

import pytest

from dimos.simulation.engines.base import SimulationEngine


def _make_fake_engine(n_joints: int = 8) -> MagicMock:
    """Create a fake SimulationEngine with n_joints joints."""
    engine = MagicMock(spec=SimulationEngine)
    engine.joint_names = [f"joint{i}" for i in range(n_joints)]
    engine.connected = True
    engine.read_joint_positions.return_value = [float(i) * 0.1 for i in range(n_joints)]
    engine.read_joint_velocities.return_value = [0.0] * n_joints
    engine.read_joint_efforts.return_value = [0.0] * n_joints
    engine._lock = threading.Lock()
    engine._joint_position_targets = [0.0] * n_joints
    return engine


@pytest.fixture
def adapter_with_gripper():
    """SimMujocoAdapter with 7 arm DOF + 1 gripper joint (8 engine joints)."""
    with patch(
        "dimos.hardware.manipulators.sim.adapter.MujocoEngine",
        return_value=_make_fake_engine(8),
    ):
        from dimos.hardware.manipulators.sim.adapter import SimMujocoAdapter

        return SimMujocoAdapter(dof=7, address="/fake/scene.xml", headless=True)


@pytest.fixture
def adapter_no_gripper():
    """SimMujocoAdapter with 7 arm DOF, no extra gripper joint."""
    with patch(
        "dimos.hardware.manipulators.sim.adapter.MujocoEngine",
        return_value=_make_fake_engine(7),
    ):
        from dimos.hardware.manipulators.sim.adapter import SimMujocoAdapter

        return SimMujocoAdapter(dof=7, address="/fake/scene.xml", headless=True)


def test_address_required():
    with pytest.raises(ValueError, match="address"):
        with patch(
            "dimos.hardware.manipulators.sim.adapter.MujocoEngine",
            return_value=_make_fake_engine(7),
        ):
            from dimos.hardware.manipulators.sim.adapter import SimMujocoAdapter

            SimMujocoAdapter(dof=7, address=None)


def test_gripper_detected(adapter_with_gripper):
    assert adapter_with_gripper._gripper_idx == 7


def test_no_gripper_when_dof_matches(adapter_no_gripper):
    assert adapter_no_gripper._gripper_idx is None


def test_read_gripper_position(adapter_with_gripper):
    pos = adapter_with_gripper.read_gripper_position()
    # Engine returns [0.0, 0.1, 0.2, ..., 0.7] — index 7 = 0.7
    assert pos == pytest.approx(0.7)


def test_read_gripper_position_no_gripper(adapter_no_gripper):
    assert adapter_no_gripper.read_gripper_position() is None


def test_write_gripper_position(adapter_with_gripper):
    result = adapter_with_gripper.write_gripper_position(0.85)
    assert result is True
    assert adapter_with_gripper._engine._joint_position_targets[7] == pytest.approx(0.85)


def test_write_gripper_position_no_gripper(adapter_no_gripper):
    assert adapter_no_gripper.write_gripper_position(0.5) is False


def test_write_gripper_does_not_clobber_arm(adapter_with_gripper):
    """Gripper write must not overwrite arm joint targets."""
    engine = adapter_with_gripper._engine
    # Set arm targets to known values
    for i in range(7):
        engine._joint_position_targets[i] = float(i) + 1.0

    adapter_with_gripper.write_gripper_position(0.9)

    # Arm targets must remain unchanged
    for i in range(7):
        assert engine._joint_position_targets[i] == pytest.approx(float(i) + 1.0)
    # Gripper updated
    assert engine._joint_position_targets[7] == pytest.approx(0.9)


def test_read_joint_positions_excludes_gripper(adapter_with_gripper):
    positions = adapter_with_gripper.read_joint_positions()
    assert len(positions) == 7


def test_connect_and_disconnect(adapter_with_gripper):
    assert adapter_with_gripper.connect() is True
    adapter_with_gripper._engine.connect.assert_called_once()
    adapter_with_gripper.disconnect()
    adapter_with_gripper._engine.disconnect.assert_called_once()


def test_register():
    from dimos.hardware.manipulators.sim.adapter import register

    registry = MagicMock()
    register(registry)
    registry.register.assert_called_once_with("sim_mujoco", pytest.importorskip(
        "dimos.hardware.manipulators.sim.adapter"
    ).SimMujocoAdapter)
