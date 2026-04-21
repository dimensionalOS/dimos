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

"""Unit tests for PathFollowerTask."""

from __future__ import annotations

import pytest

from dimos.control.task import (
    ControlMode,
    CoordinatorState,
    JointStateSnapshot,
)
from dimos.control.tasks.path_follower_task import (
    PathFollowerTask,
    PathFollowerTaskConfig,
)
from dimos.core.global_config import GlobalConfig, global_config
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path


def _make_pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def _make_path(points: list[tuple[float, float]]) -> Path:
    return Path(poses=[_make_pose(x, y) for x, y in points])


def _make_state(t_now: float = 0.0, dt: float = 0.01) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(),
        t_now=t_now,
        dt=dt,
    )


@pytest.fixture()
def config() -> PathFollowerTaskConfig:
    return PathFollowerTaskConfig(
        joint_names=["go2_vx", "go2_vy", "go2_wz"],
        priority=20,
        control_frequency=10.0,
        ct_kp=1.0,
        ct_ki=0.0,
        ct_kd=0.0,
    )


@pytest.fixture()
def task(config: PathFollowerTaskConfig) -> PathFollowerTask:
    return PathFollowerTask("test_follower", config, global_config)


class TestStateTransitions:
    def test_initial_state_is_idle(self, task: PathFollowerTask) -> None:
        assert task.get_state() == "idle"
        assert not task.is_active()

    def test_start_path_transitions_to_following(self, task: PathFollowerTask) -> None:
        path = _make_path([(0, 0), (1, 0), (2, 0)])
        odom = _make_pose(0, 0)
        assert task.start_path(path, odom)
        assert task.get_state() == "following"
        assert task.is_active()

    def test_start_path_rejects_short_path(self, task: PathFollowerTask) -> None:
        path = _make_path([(0, 0)])
        assert not task.start_path(path, _make_pose(0, 0))
        assert task.get_state() == "idle"

    def test_cancel_transitions_to_aborted(self, task: PathFollowerTask) -> None:
        path = _make_path([(0, 0), (1, 0), (2, 0)])
        task.start_path(path, _make_pose(0, 0))
        assert task.cancel()
        assert task.get_state() == "aborted"
        assert not task.is_active()

    def test_cancel_fails_when_not_following(self, task: PathFollowerTask) -> None:
        assert not task.cancel()

    def test_reset_from_aborted(self, task: PathFollowerTask) -> None:
        path = _make_path([(0, 0), (1, 0), (2, 0)])
        task.start_path(path, _make_pose(0, 0))
        task.cancel()
        assert task.reset()
        assert task.get_state() == "idle"

    def test_reset_fails_while_following(self, task: PathFollowerTask) -> None:
        path = _make_path([(0, 0), (1, 0), (2, 0)])
        task.start_path(path, _make_pose(0, 0))
        assert not task.reset()


class TestCompute:
    def test_returns_none_when_idle(self, task: PathFollowerTask) -> None:
        state = _make_state(t_now=0.0)
        assert task.compute(state) is None

    def test_returns_velocity_output(self, task: PathFollowerTask) -> None:
        path = _make_path([(0, 0), (1, 0), (2, 0), (3, 0)])
        task.start_path(path, _make_pose(0, 0))

        state = _make_state(t_now=1.0)
        output = task.compute(state)

        assert output is not None
        assert output.mode == ControlMode.VELOCITY
        assert output.joint_names == ["go2_vx", "go2_vy", "go2_wz"]
        assert output.velocities is not None
        assert len(output.velocities) == 3

    def test_decimation_returns_cached_output(self, task: PathFollowerTask) -> None:
        path = _make_path([(0, 0), (1, 0), (2, 0), (3, 0)])
        task.start_path(path, _make_pose(0, 0))

        # First compute at t=1.0
        out1 = task.compute(_make_state(t_now=1.0))
        assert out1 is not None

        # Second compute at t=1.05 (within 0.1s period) should return cached
        out2 = task.compute(_make_state(t_now=1.05))
        assert out2 is out1

        # Third compute at t=1.11 (past 0.1s period) should recompute
        out3 = task.compute(_make_state(t_now=1.11))
        assert out3 is not None

    def test_goal_detection(self, task: PathFollowerTask) -> None:
        path = _make_path([(0, 0), (0.1, 0)])
        task.start_path(path, _make_pose(0, 0))

        # Place robot at the goal
        task.update_odom(_make_pose(0.1, 0))

        output = task.compute(_make_state(t_now=1.0))
        assert output is not None
        # Should transition to completed
        assert task.get_state() == "completed"
        assert output.velocities == [0.0, 0.0, 0.0]


class TestResourceClaim:
    def test_claim_joints_and_priority(self, task: PathFollowerTask) -> None:
        claim = task.claim()
        assert claim.joints == frozenset({"go2_vx", "go2_vy", "go2_wz"})
        assert claim.priority == 20
        assert claim.mode == ControlMode.VELOCITY


class TestPreemption:
    def test_preemption_aborts_following(self, task: PathFollowerTask) -> None:
        path = _make_path([(0, 0), (1, 0), (2, 0)])
        task.start_path(path, _make_pose(0, 0))

        task.on_preempted("teleop", frozenset({"go2_vx"}))
        assert task.get_state() == "aborted"

    def test_preemption_ignored_for_unrelated_joints(self, task: PathFollowerTask) -> None:
        path = _make_path([(0, 0), (1, 0), (2, 0)])
        task.start_path(path, _make_pose(0, 0))

        task.on_preempted("arm_task", frozenset({"arm_joint1"}))
        assert task.get_state() == "following"


class TestInit:
    def test_rejects_wrong_joint_count(self) -> None:
        config = PathFollowerTaskConfig(joint_names=["vx", "wz"])
        with pytest.raises(ValueError, match="exactly 3 joints"):
            PathFollowerTask("bad", config, GlobalConfig())
