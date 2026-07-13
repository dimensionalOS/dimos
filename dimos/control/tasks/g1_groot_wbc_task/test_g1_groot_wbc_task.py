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

"""Behavioral tests for the G1 GR00T WBC task.

ONNX runtime is stubbed so these tests exercise the policy input contract,
safety state transitions, and partial-state cache behavior without depending
on the actual GR00T weights.
"""

from __future__ import annotations

from collections.abc import Iterator
import math
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from dimos.control.components import make_humanoid_joints
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.g1_groot_wbc_task import g1_groot_wbc_task
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import (
    G1GrootWBCTask,
    G1GrootWBCTaskConfig,
)
from dimos.hardware.whole_body.spec import IMUState


class _StubSession:
    """ONNX InferenceSession stub that records calls and returns a fixed action."""

    def __init__(
        self,
        model_path: str,
        *,
        label: str,
        action: np.ndarray,
        call_log: list[str],
        providers: Any = None,
    ) -> None:
        self.model_path = model_path
        self._label = label
        self._action = action
        self._call_log = call_log
        self._providers = list(providers or [])
        fake_input = MagicMock()
        fake_input.name = "obs"
        self._inputs = [fake_input]

    def get_inputs(self) -> list[Any]:
        return self._inputs

    def get_providers(self) -> list[str]:
        return self._providers

    def run(self, _outputs: Any, _feed: dict[str, np.ndarray]) -> list[np.ndarray]:
        self._call_log.append(self._label)
        return [self._action.reshape(1, -1)]


@pytest.fixture
def patched_ort(monkeypatch: pytest.MonkeyPatch) -> list[str]:
    call_log: list[str] = []

    def _factory(path: str, providers: Any = None) -> _StubSession:
        _ = providers
        label = "balance" if "balance" in str(path) else "walk"
        return _StubSession(
            str(path),
            label=label,
            action=np.full(15, 0.1, dtype=np.float32),
            call_log=call_log,
            providers=providers,
        )

    monkeypatch.setattr(g1_groot_wbc_task.ort, "InferenceSession", _factory)
    monkeypatch.setattr(
        g1_groot_wbc_task.ort,
        "get_available_providers",
        lambda: ["CPUExecutionProvider"],
    )
    return call_log


@pytest.fixture
def stub_adapter() -> MagicMock:
    adapter = MagicMock()
    adapter.read_imu.return_value = IMUState(
        quaternion=(1.0, 0.0, 0.0, 0.0),
        gyroscope=(0.0, 0.0, 0.0),
        accelerometer=(0.0, 0.0, -9.81),
        rpy=(0.0, 0.0, 0.0),
    )
    return adapter


@pytest.fixture
def joints_29() -> list[str]:
    return make_humanoid_joints("g1")


@pytest.fixture
def task(
    patched_ort: list[str], stub_adapter: MagicMock, joints_29: list[str]
) -> Iterator[G1GrootWBCTask]:
    _ = patched_ort
    task = G1GrootWBCTask(
        name="groot_wbc",
        config=G1GrootWBCTaskConfig(
            balance_onnx="/fake/balance.onnx",
            walk_onnx="/fake/walk.onnx",
            joint_names=joints_29[:15],
            all_joint_names=joints_29,
            priority=50,
            auto_arm=True,
            default_ramp_seconds=0.0,
        ),
        adapter=stub_adapter,
    )
    try:
        yield task
    finally:
        task.stop()


@pytest.fixture
def unarmed_task(
    patched_ort: list[str], stub_adapter: MagicMock, joints_29: list[str]
) -> Iterator[G1GrootWBCTask]:
    _ = patched_ort
    task = G1GrootWBCTask(
        name="groot_wbc",
        config=G1GrootWBCTaskConfig(
            balance_onnx="/fake/balance.onnx",
            walk_onnx="/fake/walk.onnx",
            joint_names=joints_29[:15],
            all_joint_names=joints_29,
            priority=50,
            auto_arm=False,
            default_ramp_seconds=0.0,
        ),
        adapter=stub_adapter,
    )
    try:
        yield task
    finally:
        task.stop()


def _state_at(t_now: float, joint_names: list[str]) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={n: 0.0 for n in joint_names},
            joint_velocities={n: 0.0 for n in joint_names},
            joint_efforts={n: 0.0 for n in joint_names},
            timestamp=t_now,
        ),
        t_now=t_now,
        dt=0.002,
    )


def test_nonzero_cmd_uses_walk_until_timeout(
    task: G1GrootWBCTask, joints_29: list[str], patched_ort: list[str]
) -> None:
    task.start()
    task.set_velocity_command(0.5, 0.0, 0.0, t_now=100.0)

    for _ in range(10):
        task.compute(_state_at(100.5, joints_29))
    for _ in range(10):
        task.compute(_state_at(102.0, joints_29))

    assert patched_ort == ["walk", "balance"]


def test_observation_layout_matches_policy_contract(task: G1GrootWBCTask) -> None:
    cmd = np.array([1.0, 0.5, 0.25], dtype=np.float32)
    gyro = np.array([0.1, 0.2, 0.3], dtype=np.float32)
    gravity = np.array([0.0, 0.0, -1.0], dtype=np.float32)
    q = np.zeros(g1_groot_wbc_task._NUM_MOTORS, dtype=np.float32)
    dq = np.ones(g1_groot_wbc_task._NUM_MOTORS, dtype=np.float32)

    obs = task._build_obs(cmd=cmd, gyro=gyro, gravity=gravity, q=q, dq=dq)

    assert obs.shape == (86,)
    np.testing.assert_allclose(obs[0:3], cmd * np.array([2.0, 2.0, 0.5]))
    assert obs[3] == pytest.approx(0.74)
    np.testing.assert_array_equal(obs[4:7], np.zeros(3))
    np.testing.assert_allclose(obs[7:10], gyro * 0.5)
    np.testing.assert_array_equal(obs[10:13], gravity)
    np.testing.assert_allclose(
        obs[13:42],
        -np.asarray(g1_groot_wbc_task._DEFAULT_POSITIONS_29, dtype=np.float32),
    )
    np.testing.assert_allclose(obs[42:71], dq * 0.05)
    np.testing.assert_array_equal(obs[71:86], np.zeros(15))


def test_partial_state_keeps_claimed_cache_consistent_with_full_cache(
    unarmed_task: G1GrootWBCTask, joints_29: list[str]
) -> None:
    assert unarmed_task._refresh_state_caches(_state_at(0.0, joints_29))

    positions = {n: 0.0 for n in joints_29}
    velocities = {n: 0.0 for n in joints_29[1:]}
    positions[joints_29[0]] = 0.42
    partial = CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions=positions,
            joint_velocities=velocities,
            joint_efforts={n: 0.0 for n in joints_29},
            timestamp=0.1,
        ),
        t_now=0.1,
        dt=0.002,
    )

    assert not unarmed_task._refresh_state_caches(partial)
    assert unarmed_task._cached_q_29[0] == pytest.approx(0.42)
    assert unarmed_task._cached_q_15[0] == pytest.approx(0.42)
    assert unarmed_task._cached_dq_29[0] == pytest.approx(0.0)


def test_unarmed_task_holds_current_pose_without_running_policy(
    unarmed_task: G1GrootWBCTask, joints_29: list[str], patched_ort: list[str]
) -> None:
    unarmed_task.start()
    snap = JointStateSnapshot(
        joint_positions={n: 0.0 for n in joints_29},
        joint_velocities={n: 0.0 for n in joints_29},
        joint_efforts={n: 0.0 for n in joints_29},
        timestamp=100.0,
    )
    for i, name in enumerate(joints_29[:15]):
        snap.joint_positions[name] = 0.1 * (i + 1)

    out = None
    for _ in range(30):
        out = unarmed_task.compute(CoordinatorState(joints=snap, t_now=100.0, dt=0.002))

    assert out is not None
    np.testing.assert_allclose(out.positions, [0.1 * (i + 1) for i in range(15)], atol=1e-6)
    assert patched_ort == []


def test_arm_with_ramp_lerps_from_current_pose_to_policy_default(
    unarmed_task: G1GrootWBCTask, joints_29: list[str], patched_ort: list[str]
) -> None:
    unarmed_task.start()
    assert unarmed_task.arm(ramp_seconds=1.0)

    out0 = unarmed_task.compute(_state_at(0.0, joints_29))
    assert out0 is not None
    np.testing.assert_allclose(out0.positions, [0.0] * 15, atol=1e-6)

    out_mid = unarmed_task.compute(_state_at(0.5, joints_29))
    assert out_mid is not None
    expected_mid = [0.5 * value for value in g1_groot_wbc_task._DEFAULT_POSITIONS_29[:15]]
    np.testing.assert_allclose(out_mid.positions, expected_mid, atol=1e-6)

    out_end = unarmed_task.compute(_state_at(1.0, joints_29))
    assert out_end is not None
    np.testing.assert_allclose(out_end.positions, g1_groot_wbc_task._DEFAULT_POSITIONS_29[:15])
    assert unarmed_task._armed
    assert not unarmed_task._arming
    assert patched_ort == []


def test_dry_run_suppresses_output_but_keeps_policy_hot(
    task: G1GrootWBCTask, joints_29: list[str], patched_ort: list[str]
) -> None:
    task.start()
    task.set_dry_run(True)

    out = None
    for _ in range(10):
        out = task.compute(_state_at(100.0, joints_29))

    assert out is None
    assert patched_ort == ["balance"]
    assert np.any(task._obs_buf != 0.0)


def test_reset_runtime_state_clears_policy_state_and_rearms(
    task: G1GrootWBCTask, joints_29: list[str]
) -> None:
    task.start()
    task.set_velocity_command(0.5, 0.0, 0.0, t_now=100.0)
    for _ in range(10):
        task.compute(_state_at(100.0, joints_29))

    assert task.state_snapshot()["armed"]
    assert np.any(task._obs_buf != 0.0)
    assert np.any(task._cmd != 0.0)

    assert task.reset_runtime_state(reactivate=True)

    snapshot = task.state_snapshot()
    assert not snapshot["armed"]
    assert snapshot["arm_pending"]
    np.testing.assert_array_equal(task._obs_buf, np.zeros_like(task._obs_buf))
    np.testing.assert_array_equal(task._last_action, np.zeros_like(task._last_action))
    np.testing.assert_array_equal(task._cmd, np.zeros_like(task._cmd))
    assert task._last_cmd_time == 0.0

    task.compute(_state_at(101.0, joints_29))
    assert task.state_snapshot()["armed"]


def test_projected_gravity_matches_reference_quaternion_order() -> None:
    np.testing.assert_allclose(
        G1GrootWBCTask._projected_gravity((1.0, 0.0, 0.0, 0.0)),
        np.array([0.0, 0.0, -1.0]),
        atol=1e-6,
    )

    s = math.sin(math.pi / 4.0)
    c = math.cos(math.pi / 4.0)
    np.testing.assert_allclose(
        G1GrootWBCTask._projected_gravity((c, s, 0.0, 0.0)),
        np.array([0.0, -1.0, 0.0]),
        atol=1e-6,
    )
