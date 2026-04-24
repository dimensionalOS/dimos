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

"""Unit tests for GrootWBCTask.

ONNX runtime is monkey-patched with a stub that records which model
was called and returns a deterministic action — so the tests exercise
the obs-build, model-selection, decimation, and command-timeout logic
without depending on the actual GR00T ONNX weights.
"""

from __future__ import annotations

import math
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from dimos.control.components import make_humanoid_joints
from dimos.control.task import (
    ControlMode,
    CoordinatorState,
    JointStateSnapshot,
)
from dimos.control.tasks import groot_wbc_task
from dimos.control.tasks.groot_wbc_task import GrootWBCTask, GrootWBCTaskConfig
from dimos.hardware.whole_body.spec import IMUState

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


class _StubSession:
    """ONNX InferenceSession stub that tracks call count and returns a fixed action."""

    def __init__(
        self,
        model_path: str,
        *,
        label: str,
        action: np.ndarray,
        call_log: list[str],
    ) -> None:
        self.model_path = model_path
        self._label = label
        self._action = action
        self._call_log = call_log
        fake_input = MagicMock()
        fake_input.name = "obs"
        self._inputs = [fake_input]

    def get_inputs(self) -> list[Any]:
        return self._inputs

    def run(self, _outputs: Any, _feed: dict[str, np.ndarray]) -> list[np.ndarray]:
        self._call_log.append(self._label)
        return [self._action.reshape(1, -1)]


@pytest.fixture
def patched_ort(monkeypatch):
    """Patch onnxruntime so no real ONNX files are needed."""
    call_log: list[str] = []

    def _factory(path: str, providers: Any = None) -> _StubSession:
        label = "balance" if "balance" in str(path) else "walk"
        return _StubSession(
            str(path),
            label=label,
            action=np.full(15, 0.1, dtype=np.float32),
            call_log=call_log,
        )

    monkeypatch.setattr(groot_wbc_task.ort, "InferenceSession", _factory)
    monkeypatch.setattr(
        groot_wbc_task.ort, "get_available_providers", lambda: ["CPUExecutionProvider"]
    )
    return call_log


@pytest.fixture
def stub_adapter():
    """Stub WholeBodyAdapter returning a zeroed-out IMU (identity quat)."""
    adapter = MagicMock()
    adapter.read_imu.return_value = IMUState(
        quaternion=(1.0, 0.0, 0.0, 0.0),  # identity (w, x, y, z)
        gyroscope=(0.0, 0.0, 0.0),
        accelerometer=(0.0, 0.0, -9.81),
        rpy=(0.0, 0.0, 0.0),
    )
    return adapter


@pytest.fixture
def joints_29():
    return make_humanoid_joints("g1")


@pytest.fixture
def task(patched_ort, stub_adapter, joints_29) -> GrootWBCTask:
    """Test fixture: auto-armed with no ramp so the existing policy
    tests can run compute() immediately after start().  The arming/
    dry-run state-machine has its own dedicated tests below."""
    legs_waist = joints_29[:15]
    return GrootWBCTask(
        name="groot_wbc",
        config=GrootWBCTaskConfig(
            balance_onnx="/fake/balance.onnx",
            walk_onnx="/fake/walk.onnx",
            joint_names=legs_waist,
            all_joint_names=joints_29,
            priority=50,
            auto_arm=True,
            default_ramp_seconds=0.0,
        ),
        adapter=stub_adapter,
    )


@pytest.fixture
def unarmed_task(patched_ort, stub_adapter, joints_29) -> GrootWBCTask:
    """Fixture mirroring the real-hardware blueprint: active but
    unarmed on start(), so arm()/disarm()/set_dry_run() can be
    exercised explicitly."""
    legs_waist = joints_29[:15]
    return GrootWBCTask(
        name="groot_wbc",
        config=GrootWBCTaskConfig(
            balance_onnx="/fake/balance.onnx",
            walk_onnx="/fake/walk.onnx",
            joint_names=legs_waist,
            all_joint_names=joints_29,
            priority=50,
            auto_arm=False,
            default_ramp_seconds=0.0,
        ),
        adapter=stub_adapter,
    )


def _state_at(t_now: float, joint_names: list[str]) -> CoordinatorState:
    snap = JointStateSnapshot(
        joint_positions={n: 0.0 for n in joint_names},
        joint_velocities={n: 0.0 for n in joint_names},
        joint_efforts={n: 0.0 for n in joint_names},
        timestamp=t_now,
    )
    return CoordinatorState(joints=snap, t_now=t_now, dt=0.002)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_claim_shape(task, joints_29):
    claim = task.claim()
    assert claim.joints == frozenset(joints_29[:15])
    assert claim.priority == 50
    assert claim.mode == ControlMode.SERVO_POSITION


def test_inactive_returns_none(task, joints_29):
    state = _state_at(100.0, joints_29)
    assert task.compute(state) is None


def test_active_zero_cmd_routes_to_balance(task, joints_29, patched_ort):
    task.start()
    # Decimation=10 → run compute 10 times to force first inference.
    state = _state_at(100.0, joints_29)
    result = None
    for _ in range(10):
        result = task.compute(state)
    assert result is not None
    assert len(result.positions) == 15
    assert patched_ort == ["balance"]


def test_nonzero_cmd_routes_to_walk(task, joints_29, patched_ort):
    task.start()
    task.set_velocity_command(0.5, 0.0, 0.0, t_now=100.0)
    state = _state_at(100.0, joints_29)
    for _ in range(10):
        task.compute(state)
    assert patched_ort == ["walk"]


def test_decimation_reemits_last_targets(task, joints_29, patched_ort):
    """Between inference ticks, the task should repeat the last output."""
    task.start()
    state = _state_at(100.0, joints_29)
    # First 9 ticks pre-inference: no targets yet.
    for _ in range(9):
        assert task.compute(state) is None
    # 10th tick: inference fires.
    first = task.compute(state)
    assert first is not None
    assert len(patched_ort) == 1
    # Next 9 ticks: no inference, same targets echoed.
    for _ in range(9):
        echo = task.compute(state)
        assert echo is not None
        assert echo.positions == first.positions
    assert len(patched_ort) == 1
    # 20th tick: second inference.
    task.compute(state)
    assert len(patched_ort) == 2


def test_velocity_command_timeout(task, joints_29, patched_ort):
    task.start()
    task.set_velocity_command(0.5, 0.0, 0.0, t_now=100.0)
    # Still inside the 1.0s timeout — walk.
    state_inside = _state_at(100.5, joints_29)
    for _ in range(10):
        task.compute(state_inside)
    # Past the timeout — command goes to zero → balance.
    state_outside = _state_at(102.0, joints_29)
    for _ in range(10):
        task.compute(state_outside)
    assert patched_ort == ["walk", "balance"]


def test_projected_gravity_identity_quat():
    g = GrootWBCTask._projected_gravity((1.0, 0.0, 0.0, 0.0))
    np.testing.assert_allclose(g, np.array([0.0, 0.0, -1.0]), atol=1e-6)


def test_projected_gravity_roll_90():
    """+90° roll around body-X: body-Y now points world-up, body-Z world-right.
    World gravity (0,0,-1) expressed in body frame is (0, -1, 0)."""
    s = math.sin(math.pi / 4.0)
    c = math.cos(math.pi / 4.0)
    g = GrootWBCTask._projected_gravity((c, s, 0.0, 0.0))
    np.testing.assert_allclose(g, np.array([0.0, -1.0, 0.0]), atol=1e-6)


def test_projected_gravity_pitch_90():
    """+90° pitch around body-Y: body-X now points world-down, body-Z world-forward.
    World gravity (0,0,-1) expressed in body frame is (+1, 0, 0)."""
    s = math.sin(math.pi / 4.0)
    c = math.cos(math.pi / 4.0)
    g = GrootWBCTask._projected_gravity((c, 0.0, s, 0.0))
    np.testing.assert_allclose(g, np.array([1.0, 0.0, 0.0]), atol=1e-6)


def test_obs_build_layout(task):
    """Verify the 86-dim obs respects the documented slot layout."""
    cmd = np.array([1.0, 0.5, 0.25], dtype=np.float32)
    gyro = np.array([0.1, 0.2, 0.3], dtype=np.float32)
    gravity = np.array([0.0, 0.0, -1.0], dtype=np.float32)
    q = np.zeros(29, dtype=np.float32)
    dq = np.ones(29, dtype=np.float32)
    obs = task._build_obs(cmd=cmd, gyro=gyro, gravity=gravity, q=q, dq=dq)
    assert obs.shape == (86,)
    np.testing.assert_allclose(obs[0:3], cmd * np.array([2.0, 2.0, 0.5]))
    assert obs[3] == pytest.approx(0.74)
    np.testing.assert_array_equal(obs[4:7], np.zeros(3))
    np.testing.assert_allclose(obs[7:10], gyro * 0.5)
    np.testing.assert_array_equal(obs[10:13], gravity)
    # q - default_29 → legs/waist get nonzero offsets from DEFAULT_15,
    # arms (indices 15..28 in DEFAULT_29) are zero, so obs[28:42] == 0.
    np.testing.assert_array_equal(obs[28:42], np.zeros(14))
    np.testing.assert_allclose(obs[42:71], dq * 0.05)
    np.testing.assert_array_equal(obs[71:86], np.zeros(15))


def test_first_inference_fills_history(task, joints_29, patched_ort):
    """First inference should tile current obs across all 6 history slots."""
    task.start()
    state = _state_at(100.0, joints_29)
    for _ in range(10):
        task.compute(state)
    # History has 6 identical 86-dim slices.
    buf = task._obs_buf[0]
    assert buf.shape == (86 * 6,)
    slice0 = buf[0:86]
    for k in range(1, 6):
        np.testing.assert_array_equal(buf[86 * k : 86 * (k + 1)], slice0)


def test_start_resets_state(task, joints_29, patched_ort):
    task.start()
    state = _state_at(100.0, joints_29)
    for _ in range(10):
        task.compute(state)
    assert np.any(task._last_action != 0.0)
    assert task._last_targets is not None

    task.stop()
    assert task._last_targets is None

    task.start()
    # After restart, tick counter is zero, last_action cleared, first-inference flag set.
    assert task._tick_count == 0
    np.testing.assert_array_equal(task._last_action, np.zeros(15, dtype=np.float32))
    assert task._first_inference is True


def test_on_twist_routes_to_velocity_cmd(task):
    msg = MagicMock()
    msg.linear.x = 0.7
    msg.linear.y = -0.2
    msg.angular.z = 0.4
    task.on_twist(msg, t_now=12.34)
    np.testing.assert_allclose(task._cmd, np.array([0.7, -0.2, 0.4], dtype=np.float32), atol=1e-6)
    assert task._last_cmd_time == 12.34


def test_joint_count_validation(patched_ort, stub_adapter, joints_29):
    with pytest.raises(ValueError, match="15 joint names"):
        GrootWBCTask(
            name="bad",
            config=GrootWBCTaskConfig(
                balance_onnx="/fake/balance.onnx",
                walk_onnx="/fake/walk.onnx",
                joint_names=joints_29[:10],  # wrong size
                all_joint_names=joints_29,
            ),
            adapter=stub_adapter,
        )
    with pytest.raises(ValueError, match="29 all_joint_names"):
        GrootWBCTask(
            name="bad",
            config=GrootWBCTaskConfig(
                balance_onnx="/fake/balance.onnx",
                walk_onnx="/fake/walk.onnx",
                joint_names=joints_29[:15],
                all_joint_names=joints_29[:20],  # wrong size
            ),
            adapter=stub_adapter,
        )


# ---------------------------------------------------------------------------
# Arming / dry-run state machine
# ---------------------------------------------------------------------------


def test_unarmed_holds_current_pose(unarmed_task, joints_29, patched_ort):
    """Active but unarmed → compute() echoes current joint positions
    every tick.  Downstream PD with q_tgt == q_actual → damping only."""
    unarmed_task.start()
    snap = JointStateSnapshot(
        joint_positions={n: 0.0 for n in joints_29},
        joint_velocities={n: 0.0 for n in joints_29},
        joint_efforts={n: 0.0 for n in joints_29},
        timestamp=100.0,
    )
    # Set some non-zero current positions for the 15 claimed joints.
    for i, n in enumerate(joints_29[:15]):
        snap.joint_positions[n] = 0.1 * (i + 1)
    state = CoordinatorState(joints=snap, t_now=100.0, dt=0.002)
    for _ in range(30):
        out = unarmed_task.compute(state)
        assert out is not None
        # No inference while unarmed.
    assert patched_ort == []
    # Output tracks current pose exactly.
    np.testing.assert_allclose(out.positions, [0.1 * (i + 1) for i in range(15)], atol=1e-6)


def test_arm_no_ramp_goes_straight_to_policy(unarmed_task, joints_29, patched_ort):
    """arm(0.0) → immediately armed → policy runs on the next decimation tick."""
    unarmed_task.start()
    unarmed_task.arm(ramp_seconds=0.0)
    state = _state_at(100.0, joints_29)
    # First compute after arm(): snapshots ramp_start, flips armed=True (ramp=0).
    unarmed_task.compute(state)
    assert unarmed_task._armed
    # 9 more ticks to hit decimation threshold (10th is inference).
    for _ in range(9):
        unarmed_task.compute(state)
    assert patched_ort == ["balance"]


def test_arm_with_ramp_lerps_over_duration(unarmed_task, joints_29, patched_ort):
    """arm(1.0) → lerp from current pose to default_15 over 1 second."""
    unarmed_task.start()
    unarmed_task.arm(ramp_seconds=1.0)
    # First tick: snapshot ramp_start (all zeros).
    state0 = _state_at(0.0, joints_29)
    out0 = unarmed_task.compute(state0)
    assert out0 is not None
    assert unarmed_task._arming
    # alpha=0 → output == ramp_start (all zeros here).
    np.testing.assert_allclose(out0.positions, [0.0] * 15, atol=1e-6)
    # Halfway through: alpha=0.5.
    state_mid = _state_at(0.5, joints_29)
    out_mid = unarmed_task.compute(state_mid)
    default_15 = list(groot_wbc_task._DEFAULT_POSITIONS_29[:15])
    expected_mid = [0.5 * d for d in default_15]
    np.testing.assert_allclose(out_mid.positions, expected_mid, atol=1e-6)
    # End: alpha=1 → armed flips, output == default_15.
    state_end = _state_at(1.0, joints_29)
    unarmed_task.compute(state_end)
    assert unarmed_task._armed
    assert not unarmed_task._arming
    # Policy has NOT run yet — ramp completion doesn't trigger inference.
    assert patched_ort == []


def test_dry_run_suppresses_output_but_runs_inference(task, joints_29, patched_ort):
    """Dry-run: policy still computes (obs history stays hot), but
    compute() returns None so the adapter sees no command."""
    task.start()  # fixture has auto_arm=True, so armed immediately
    task.set_dry_run(True)
    state = _state_at(100.0, joints_29)
    # 10 ticks → first inference fires under the hood, but output is None.
    for _ in range(10):
        out = task.compute(state)
    assert out is None
    # Policy DID run — obs buffer is hot.
    assert patched_ort == ["balance"]


def test_dry_run_toggle_off_resumes_output(task, joints_29, patched_ort):
    """Flipping dry_run from True → False resumes normal output."""
    task.start()
    task.set_dry_run(True)
    state = _state_at(100.0, joints_29)
    for _ in range(10):
        task.compute(state)
    assert patched_ort == ["balance"]  # ran during dry-run
    task.set_dry_run(False)
    # Next inference tick: output is non-None.
    for _ in range(10):
        out = task.compute(state)
    assert out is not None
    assert len(out.positions) == 15


def test_disarm_returns_to_hold_pose(unarmed_task, joints_29, patched_ort):
    """Disarm after policy has run → compute() falls back to echoing pose."""
    unarmed_task.start()
    unarmed_task.arm(ramp_seconds=0.0)
    state = _state_at(100.0, joints_29)
    for _ in range(10):
        unarmed_task.compute(state)
    assert patched_ort == ["balance"]
    assert unarmed_task._armed

    unarmed_task.disarm()
    assert not unarmed_task._armed
    # Policy should NOT run again.
    for _ in range(30):
        unarmed_task.compute(state)
    assert patched_ort == ["balance"]  # still just one call
