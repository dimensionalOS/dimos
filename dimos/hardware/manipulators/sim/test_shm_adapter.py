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

"""Tests for ShmMujocoAdapter (the SHM-backed ManipulatorAdapter)."""

from __future__ import annotations

from types import SimpleNamespace
import uuid

import pytest

import dimos.hardware.manipulators.sim.adapter as adapter_mod
from dimos.hardware.manipulators.sim.adapter import ShmMujocoAdapter
from dimos.hardware.manipulators.spec import ControlMode, ManipulatorAdapter
from dimos.simulation.engines.mujoco_shm import ManipShmWriter
from dimos.simulation.engines.mujoco_sim_module import _WholeBodySimHooks

ARM_DOF = 7

# The xarm7 hand, which is what the fixtures below stand in for.
GRIPPER_RANGE = (0.0, 0.85)
GRIPPER_CTRL_RANGE = (0.0, 255.0)
GRIPPER_CLOSED, GRIPPER_OPEN = GRIPPER_RANGE


def sim_settles_at(command: float) -> float:
    """The raw MJCF joint value the sim reaches for a gripper ``command``.

    Runs the real command mapping, then the actuator. Measured on data/xarm7:
    ctrl 0 leaves the joint at 0.003 with the jaws 14cm apart, ctrl 255 drives
    it to 0.848 with the jaws closed, so the joint tracks the ctrl endpoints
    while the command mapping inverts.
    """
    sim = SimpleNamespace(
        _gripper_joint_range=GRIPPER_RANGE,
        _gripper_ctrl_range=GRIPPER_CTRL_RANGE,
    )
    ctrl = _WholeBodySimHooks._gripper_joint_to_ctrl(sim, command)
    low, high = GRIPPER_RANGE
    ctrl_low, ctrl_high = GRIPPER_CTRL_RANGE
    return low + (high - low) * (ctrl - ctrl_low) / (ctrl_high - ctrl_low)


@pytest.fixture
def shm_key():
    return f"test_{uuid.uuid4().hex[:10]}"


@pytest.fixture
def writer(shm_key, monkeypatch):
    """Pretend we're the sim module: create SHM, signal ready.

    We monkey-patch ``shm_key_from_path`` so the adapter under test resolves
    to our fixture's key regardless of the address string.
    """
    monkeypatch.setattr(adapter_mod, "shm_key_from_path", lambda _: shm_key)
    w = ManipShmWriter(shm_key)
    w.signal_ready(num_joints=ARM_DOF)
    yield w
    w.cleanup()


@pytest.fixture
def writer_with_gripper(shm_key, monkeypatch):
    monkeypatch.setattr(adapter_mod, "shm_key_from_path", lambda _: shm_key)
    w = ManipShmWriter(shm_key)
    w.signal_ready(num_joints=ARM_DOF + 1, arm_joints=ARM_DOF)
    # The sim module publishes the gripper joint's MJCF range at startup so the
    # adapter can declare it through get_limits().
    w.write_gripper_range(0.0, 0.85)
    yield w
    w.cleanup()


@pytest.fixture
def adapter(writer):
    a = ShmMujocoAdapter(dof=ARM_DOF, address="/fake/scene.xml")
    assert a.connect() is True
    yield a
    a.disconnect()


@pytest.fixture
def adapter_with_gripper(writer_with_gripper):
    a = ShmMujocoAdapter(dof=ARM_DOF + 1, address="/fake/scene.xml")
    assert a.connect() is True
    yield a
    a.disconnect()


class TestProtocolConformance:
    def test_implements_manipulator_adapter(self):
        a = ShmMujocoAdapter(dof=ARM_DOF, address="/fake/scene.xml")
        assert isinstance(a, ManipulatorAdapter)

    def test_address_required(self):
        with pytest.raises(ValueError, match="address"):
            ShmMujocoAdapter(dof=ARM_DOF, address=None)

    def test_registered(self):
        from dimos.hardware.manipulators.registry import adapter_registry

        assert adapter_registry._factory_paths["sim_mujoco"] == (
            "dimos.hardware.manipulators.sim.adapter:ShmMujocoAdapter"
        )


class TestReadState:
    def test_read_joint_positions(self, adapter, writer):
        writer.write_joint_state(
            positions=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            velocities=[0.0] * 7,
            efforts=[0.0] * 7,
        )
        assert adapter.read_joint_positions() == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]

    def test_read_joint_velocities(self, adapter, writer):
        writer.write_joint_state(
            positions=[0.0] * 7,
            velocities=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
            efforts=[0.0] * 7,
        )
        assert adapter.read_joint_velocities() == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]

    def test_read_joint_efforts(self, adapter, writer):
        writer.write_joint_state(
            positions=[0.0] * 7,
            velocities=[0.0] * 7,
            efforts=[-1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0],
        )
        assert adapter.read_joint_efforts() == [-1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0]

    def test_arm_comes_from_joints_and_gripper_from_its_own_slot(
        self, adapter_with_gripper, writer_with_gripper
    ):
        """The gripper's state is published separately, not read off the tail.

        The sim module writes the gripper into the ``grp`` segment, so the
        adapter must take the arm from the joint array and the gripper from
        ``grp`` — never ``positions[dof]``, which is the raw model value.
        """
        writer_with_gripper.write_joint_state(
            positions=list(range(8)),
            velocities=[0.0] * 8,
            efforts=[0.0] * 8,
        )
        writer_with_gripper.write_gripper_state(sim_settles_at(0.42))

        positions = adapter_with_gripper.read_joint_positions()
        assert len(positions) == ARM_DOF + 1
        assert positions[:ARM_DOF] == [float(i) for i in range(ARM_DOF)]
        assert positions[-1] == pytest.approx(0.42)


class TestWriteCommand:
    def test_write_joint_positions(self, adapter, writer):
        assert adapter.write_joint_positions([0.1] * 7) is True
        cmd = writer.read_position_command(7)
        assert cmd is not None
        assert cmd.tolist() == pytest.approx([0.1] * 7)

    def test_write_joint_velocities(self, adapter, writer):
        assert adapter.write_joint_velocities([0.5] * 7) is True
        cmd = writer.read_velocity_command(7)
        assert cmd is not None
        assert cmd.tolist() == pytest.approx([0.5] * 7)

    def test_write_when_disabled(self, adapter):
        adapter.write_enable(False)
        assert adapter.write_joint_positions([0.0] * 7) is False

    def test_control_mode_tracked(self, adapter):
        adapter.write_joint_positions([0.0] * 7)
        assert adapter.get_control_mode() == ControlMode.POSITION
        adapter.write_joint_velocities([0.0] * 7)
        assert adapter.get_control_mode() == ControlMode.VELOCITY


class TestGripper:
    def test_configured_joint_count_includes_gripper(self, adapter_with_gripper):
        assert adapter_with_gripper.get_dof() == ARM_DOF + 1

    def test_no_gripper_when_not_declared(self, adapter):
        assert adapter.get_dof() == ARM_DOF

    def test_limits_declare_the_mjcf_range(self, adapter_with_gripper):
        limits = adapter_with_gripper.get_limits()
        assert len(limits.position_lower) == ARM_DOF + 1
        assert limits.position_lower[-1] == pytest.approx(0.0)
        assert limits.position_upper[-1] == pytest.approx(0.85)

    def test_limits_have_no_gripper_entry_without_one(self, adapter):
        assert len(adapter.get_limits().position_lower) == ARM_DOF

    def test_gripper_trails_the_read_array(self, adapter_with_gripper, writer_with_gripper):
        writer_with_gripper.write_gripper_state(sim_settles_at(0.33))
        assert adapter_with_gripper.read_joint_positions() == pytest.approx(
            [0.0] * ARM_DOF + [0.33]
        )

    def test_reads_stay_index_aligned(self, adapter_with_gripper):
        assert adapter_with_gripper.read_joint_velocities() == [0.0] * (ARM_DOF + 1)
        assert adapter_with_gripper.read_joint_efforts() == [0.0] * (ARM_DOF + 1)

    def test_gripper_trails_the_write_array(self, adapter_with_gripper, writer_with_gripper):
        assert adapter_with_gripper.write_joint_positions([0.0] * ARM_DOF + [0.5]) is True
        # Unscaled — the sim module maps MJCF joint range to actuator ctrl.
        assert writer_with_gripper.read_gripper_command() == pytest.approx(0.5)


class TestGripperRoundTrip:
    """Reads and writes must agree on which end of the range is closed.

    ``get_limits()`` publishes the MJCF joint range, and PR #3381 normalizes
    against it as ``(p - lower) / (upper - lower)`` with 0.0 closed and 1.0
    open. So the position the adapter reads back has to be the position that
    was commanded, on that same coordinate.
    """

    @pytest.mark.parametrize(
        "command",
        [GRIPPER_CLOSED, GRIPPER_OPEN, 0.5 * (GRIPPER_CLOSED + GRIPPER_OPEN)],
        ids=["closed", "open", "midpoint"],
    )
    def test_commanded_position_reads_back_unchanged(
        self, adapter_with_gripper, writer_with_gripper, command
    ):
        assert adapter_with_gripper.write_joint_positions([0.0] * ARM_DOF + [command]) is True
        sent = writer_with_gripper.read_gripper_command()
        assert sent is not None
        writer_with_gripper.write_gripper_state(sim_settles_at(sent))

        assert adapter_with_gripper.read_joint_positions()[-1] == pytest.approx(command)

    def test_closing_does_not_read_back_as_fully_open(
        self, adapter_with_gripper, writer_with_gripper
    ):
        """The bug this guards: closed jaws normalized to 1.00 (fully open)."""
        adapter_with_gripper.write_joint_positions([0.0] * ARM_DOF + [GRIPPER_CLOSED])
        settled = sim_settles_at(writer_with_gripper.read_gripper_command())
        # Closing drives the raw MJCF joint to the top of its range...
        assert settled == pytest.approx(GRIPPER_OPEN)
        writer_with_gripper.write_gripper_state(settled)

        # ...which must still read back as the closed end of the range.
        lower, upper = (
            adapter_with_gripper.get_limits().position_lower[-1],
            (adapter_with_gripper.get_limits().position_upper[-1]),
        )
        position = adapter_with_gripper.read_joint_positions()[-1]
        normalized = (position - lower) / (upper - lower)
        assert normalized == pytest.approx(0.0)


class TestConnect:
    def test_connect_before_sim_ready_times_out(self, shm_key, monkeypatch):
        """If sim module never signals ready, connect() returns False after timeout."""
        monkeypatch.setattr(adapter_mod, "shm_key_from_path", lambda _: shm_key)
        # Shrink timeouts so the test runs fast.
        monkeypatch.setattr(adapter_mod, "_READY_WAIT_TIMEOUT_S", 0.2)
        monkeypatch.setattr(adapter_mod, "_READY_WAIT_POLL_S", 0.02)

        w = ManipShmWriter(shm_key)
        try:
            # Note: writer exists but signal_ready is NOT called.
            a = ShmMujocoAdapter(dof=ARM_DOF, address="/fake/scene.xml")
            assert a.connect() is False
        finally:
            w.cleanup()

    def test_connect_waits_for_shm(self, shm_key, monkeypatch):
        """If SHM buffers don't exist yet, connect() retries briefly."""
        monkeypatch.setattr(adapter_mod, "shm_key_from_path", lambda _: shm_key)
        monkeypatch.setattr(adapter_mod, "_ATTACH_RETRY_TIMEOUT_S", 0.2)
        monkeypatch.setattr(adapter_mod, "_ATTACH_RETRY_POLL_S", 0.02)

        a = ShmMujocoAdapter(dof=ARM_DOF, address="/fake/scene.xml")
        # SHM was never created — attach must time out.
        assert a.connect() is False
