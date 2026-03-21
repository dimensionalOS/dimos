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

from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from dimos.hardware.manipulators.sim.adapter import SimMujocoAdapter, register
from dimos.simulation.utils.xml_parser import JointMapping

ARM_DOF = 7


def _make_joint_mapping(name: str, idx: int) -> JointMapping:
    """Create a JointMapping for a simple revolute joint."""
    return JointMapping(
        name=name,
        joint_id=idx,
        actuator_id=idx,
        qpos_adr=idx,
        dof_adr=idx,
        tendon_qpos_adrs=(),
        tendon_dof_adrs=(),
    )


def _make_gripper_mapping(name: str, idx: int) -> JointMapping:
    """Create a JointMapping for a tendon-driven gripper."""
    return JointMapping(
        name=name,
        joint_id=None,
        actuator_id=idx,
        qpos_adr=None,
        dof_adr=None,
        tendon_qpos_adrs=(idx, idx + 1),
        tendon_dof_adrs=(idx, idx + 1),
    )


def _patch_mujoco_engine(n_joints: int):
    """Patch only the MuJoCo C-library and filesystem boundaries.

    Mocks ``_resolve_xml_path``, ``MjModel.from_xml_path``, ``MjData``, and
    ``build_joint_mappings`` — the rest of ``MujocoEngine.__init__`` runs as-is.
    """
    mappings = [_make_joint_mapping(f"joint{i}", i) for i in range(ARM_DOF)]
    if n_joints > ARM_DOF:
        mappings.append(_make_gripper_mapping(f"joint{ARM_DOF}", ARM_DOF))

    fake_model = MagicMock()
    fake_model.opt.timestep = 0.002
    fake_model.nu = n_joints
    fake_model.nq = n_joints
    fake_model.njnt = n_joints
    fake_model.actuator_ctrlrange = np.array(
        [[-6.28, 6.28]] * ARM_DOF + ([[0.0, 255.0]] if n_joints > ARM_DOF else [])
    )
    fake_model.jnt_range = np.array(
        [[-6.28, 6.28]] * ARM_DOF + ([[0.0, 0.85]] if n_joints > ARM_DOF else [])
    )
    fake_model.jnt_qposadr = np.arange(n_joints)

    fake_data = MagicMock()
    fake_data.qpos = np.zeros(n_joints + 4)  # extra for tendon qpos addresses
    fake_data.actuator_length = np.zeros(n_joints)

    patches = [
        patch(
            "dimos.simulation.engines.mujoco_engine.MujocoEngine._resolve_xml_path",
            return_value=Path("/fake/scene.xml"),
        ),
        patch(
            "dimos.simulation.engines.mujoco_engine.mujoco.MjModel.from_xml_path",
            return_value=fake_model,
        ),
        patch("dimos.simulation.engines.mujoco_engine.mujoco.MjData", return_value=fake_data),
        patch("dimos.simulation.engines.mujoco_engine.build_joint_mappings", return_value=mappings),
    ]
    return patches


class TestSimMujocoAdapter:
    """Tests for SimMujocoAdapter with and without gripper."""

    @pytest.fixture
    def adapter_with_gripper(self):
        """SimMujocoAdapter with ARM_DOF arm joints + 1 gripper joint."""
        patches = _patch_mujoco_engine(ARM_DOF + 1)
        for p in patches:
            p.start()
        try:
            adapter = SimMujocoAdapter(dof=ARM_DOF, address="/fake/scene.xml", headless=True)
        finally:
            for p in patches:
                p.stop()
        return adapter

    @pytest.fixture
    def adapter_no_gripper(self):
        """SimMujocoAdapter with ARM_DOF arm joints, no gripper."""
        patches = _patch_mujoco_engine(ARM_DOF)
        for p in patches:
            p.start()
        try:
            adapter = SimMujocoAdapter(dof=ARM_DOF, address="/fake/scene.xml", headless=True)
        finally:
            for p in patches:
                p.stop()
        return adapter

    def test_address_required(self):
        patches = _patch_mujoco_engine(ARM_DOF)
        for p in patches:
            p.start()
        try:
            with pytest.raises(ValueError, match="address"):
                SimMujocoAdapter(dof=ARM_DOF, address=None)
        finally:
            for p in patches:
                p.stop()

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
