# Copyright 2026 Dimensional Inc.
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

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks.eef_twist_task.eef_twist_task import EEFTwistTask, EEFTwistTaskConfig
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped


@dataclass
class FakePose:
    translation: NDArray[np.float64]
    rotation: NDArray[np.float64]

    def copy(self) -> FakePose:
        return FakePose(self.translation.copy(), self.rotation.copy())


class FakeIK:
    def __init__(self) -> None:
        self.nq = 3
        self.fk_calls: list[np.ndarray] = []
        self.solve_calls: list[FakePose] = []
        self.solution = np.array([0.01, 0.02, 0.03], dtype=np.float64)
        self.converged = True
        self.final_error = 0.0

    def forward_kinematics(self, q_current: NDArray[np.float64]) -> FakePose:
        self.fk_calls.append(q_current.copy())
        return FakePose(q_current.copy(), np.eye(3, dtype=np.float64))

    def solve(
        self, pose: FakePose, q_current: NDArray[np.float64]
    ) -> tuple[NDArray[np.float64], bool, float]:
        self.solve_calls.append(pose.copy())
        return self.solution.copy(), self.converged, self.final_error


@pytest.fixture
def fake_ik(mocker) -> FakeIK:
    ik = FakeIK()
    mocker.patch(
        "dimos.control.tasks.eef_twist_task.eef_twist_task.PinocchioIK.from_model_path",
        return_value=ik,
    )
    return ik


@pytest.fixture
def task(fake_ik: FakeIK) -> EEFTwistTask:
    return EEFTwistTask(
        "eef",
        EEFTwistTaskConfig(
            joint_names=["arm/joint1", "arm/joint2", "arm/joint3"],
            model_path="fake.urdf",
            ee_joint_id=3,
            timeout=0.3,
            max_joint_delta_deg=15.0,
        ),
    )


def _state(
    t_now: float, positions: list[float] | None = None, dt: float = 0.01
) -> CoordinatorState:
    values = [0.0, 0.0, 0.0] if positions is None else positions
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={f"arm/joint{i + 1}": value for i, value in enumerate(values)},
        ),
        t_now=t_now,
        dt=dt,
    )


def _twist(x: float = 0.1) -> TwistStamped:
    return TwistStamped(frame_id="eef", linear=[x, 0.0, 0.0], angular=[0.0, 0.0, 0.0])


def test_first_nonzero_command_activates_seeds_from_fk_and_outputs_servo_position(
    task: EEFTwistTask, fake_ik: FakeIK
) -> None:
    assert task.on_ee_twist_command(_twist(), t_now=1.0)
    assert task.is_active()

    output = task.compute(_state(1.01))

    assert output is not None
    assert output.mode == ControlMode.SERVO_POSITION
    assert output.joint_names == ["arm/joint1", "arm/joint2", "arm/joint3"]
    assert output.positions == [0.01, 0.02, 0.03]
    assert fake_ik.solve_calls[0].translation[0] > 0.0


def test_jog_integrates_from_commanded_anchor_not_live_state(
    task: EEFTwistTask, fake_ik: FakeIK
) -> None:
    # The jog must build on the previous commanded target (anchor), NOT the live
    # joint state — so a sagging arm can't drag the target down. Even when the
    # reported live positions jump around, consecutive jogs advance the EE target.
    assert task.on_ee_twist_command(_twist(1.0), t_now=1.0)

    first = task.compute(_state(1.01, dt=0.01))
    second = task.compute(_state(1.04, positions=[0.5, 0.0, 0.0], dt=0.01))

    assert first is not None
    assert second is not None
    # Second jog's FK anchor is the first jog's solution, then +twist*dt again →
    # its target advances further in +X than the first.
    assert fake_ik.solve_calls[1].translation[0] > fake_ik.solve_calls[0].translation[0]


def test_non_converged_ik_solution_is_accepted_when_joint_delta_is_safe(
    task: EEFTwistTask, fake_ik: FakeIK
) -> None:
    fake_ik.converged = False
    fake_ik.final_error = 1.0

    assert task.on_ee_twist_command(_twist(), t_now=1.0)
    output = task.compute(_state(1.01))

    assert output is not None
    assert output.positions == [0.01, 0.02, 0.03]


def test_non_finite_ik_solution_is_rejected(task: EEFTwistTask, fake_ik: FakeIK) -> None:
    fake_ik.solution = np.array([np.nan, 0.0, 0.0], dtype=np.float64)

    assert task.on_ee_twist_command(_twist(), t_now=1.0)
    output = task.compute(_state(1.01))

    assert output is None


def test_non_finite_twist_is_rejected(task: EEFTwistTask) -> None:
    accepted = task.on_ee_twist_command(
        TwistStamped(frame_id="eef", linear=[np.nan, 0.0, 0.0], angular=[0.0, 0.0, 0.0]),
        t_now=1.0,
    )

    assert accepted is False


def test_missing_joint_state_skips_fk_and_ik(task: EEFTwistTask, fake_ik: FakeIK) -> None:
    assert task.on_ee_twist_command(_twist(), t_now=1.0)

    output = task.compute(_state(1.01, positions=[0.0, 0.0]))

    assert output is None
    assert fake_ik.fk_calls == []
    assert fake_ik.solve_calls == []


def test_joint_delta_rejection_returns_none(task: EEFTwistTask, fake_ik: FakeIK) -> None:
    assert task.on_ee_twist_command(_twist(), t_now=1.0)
    fake_ik.solution = np.array([10.0, 0.0, 0.0], dtype=np.float64)

    rejected = task.compute(_state(1.01))

    assert rejected is None


def test_active_from_spawn_holds_current_pose_when_idle(
    task: EEFTwistTask, fake_ik: FakeIK
) -> None:
    # Active before any command — holds the current joints (no IK) so a gravity-
    # loaded arm doesn't sag on spawn.
    assert task.is_active()
    held = task.compute(_state(0.5, positions=[0.1, 0.2, 0.3]))
    assert held is not None
    assert held.mode == ControlMode.SERVO_POSITION
    assert held.positions == [0.1, 0.2, 0.3]
    assert fake_ik.solve_calls == []  # hold path skips IK


def test_zero_twist_holds_the_commanded_anchor_not_live_state(
    task: EEFTwistTask, fake_ik: FakeIK
) -> None:
    # After a jog, the anchor is the jogged solution. A zero twist then holds
    # THAT commanded anchor — not the live (possibly sagging) joint state.
    assert task.on_ee_twist_command(_twist(), t_now=1.0)
    assert task.compute(_state(1.01)) is not None  # anchor advances to solution

    assert task.on_ee_twist_command(_twist(0.0), t_now=1.5)
    assert task.is_active()
    # Live state reports [0.4,0,0] (e.g. sagged), but hold commands the anchor.
    held = task.compute(_state(1.51, positions=[0.4, 0.0, 0.0]))
    assert held is not None
    assert held.positions == fake_ik.solution.tolist()  # the commanded anchor

    # A new non-zero twist resumes integration from the anchor.
    prev_calls = len(fake_ik.solve_calls)
    assert task.on_ee_twist_command(_twist(), t_now=2.0)
    assert task.compute(_state(2.01, positions=[1.0, 0.0, 0.0])) is not None
    assert len(fake_ik.solve_calls) > prev_calls


def test_stale_jog_times_out_and_holds(task: EEFTwistTask, fake_ik: FakeIK) -> None:
    # A jog is active but the command stream goes silent past timeout (operator
    # disconnected mid-jog): the task drops the twist and holds instead of
    # integrating the stale twist forever.
    assert task.on_ee_twist_command(_twist(), t_now=1.0)
    assert task.compute(_state(1.01)) is not None  # jog runs, anchor advances
    jog_calls = len(fake_ik.solve_calls)

    # No new command for > timeout (0.3s): the next compute holds (no new IK).
    held = task.compute(_state(1.5))
    assert held is not None
    assert len(fake_ik.solve_calls) == jog_calls  # hold path skips IK
    assert held.positions == fake_ik.solution.tolist()  # holds the commanded anchor


def test_preempt_reseeds_anchor_from_live_pose(task: EEFTwistTask, fake_ik: FakeIK) -> None:
    # After a jog sets the anchor, a higher-priority task preempts and moves the
    # arm. When this task regains control it must hold the arm's ACTUAL current
    # pose, not the stale pre-preempt anchor (else it would snap the arm back).
    assert task.on_ee_twist_command(_twist(), t_now=1.0)
    assert task.compute(_state(1.01)) is not None  # anchor = jogged solution

    task.on_preempted("teleop_xarm", frozenset(["arm/joint1"]))

    # The other task moved the arm to a new pose; on the next idle tick we hold
    # THAT live pose (anchor was cleared → re-seeded from current joints).
    held = task.compute(_state(2.0, positions=[0.7, 0.8, 0.9]))
    assert held is not None
    assert held.positions == [0.7, 0.8, 0.9]
