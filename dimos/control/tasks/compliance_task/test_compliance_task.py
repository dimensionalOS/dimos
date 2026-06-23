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

import pytest

from dimos.control.task import ControlMode, CoordinatorState, JointCommandOutput, JointStateSnapshot
from dimos.control.tasks.compliance_task.compliance_task import (
    JointComplianceTask,
    JointComplianceTaskConfig,
)
from dimos.control.tasks.compliance_task.compliant_trajectory_task import (
    create_compliant_joint_trajectory_task,
)
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState

JOINTS = ["arm/j1", "arm/j2"]


def make_state(
    positions: dict[str, float] | None = None,
    efforts: dict[str, float] | None = None,
    dt: float = 0.01,
    t_now: float = 1.0,
) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions=positions or {"arm/j1": 0.0, "arm/j2": 0.0},
            joint_velocities={"arm/j1": 0.0, "arm/j2": 0.0},
            joint_efforts=efforts or {},
        ),
        t_now=t_now,
        dt=dt,
    )


def make_reference(positions: list[float] | None = None) -> JointCommandOutput:
    return JointCommandOutput(
        joint_names=JOINTS,
        positions=positions or [0.0, 0.0],
        mode=ControlMode.SERVO_POSITION,
    )


def make_trajectory() -> JointTrajectory:
    return JointTrajectory(
        joint_names=JOINTS,
        points=[
            TrajectoryPoint(time_from_start=0.0, positions=[0.0, 0.0]),
            TrajectoryPoint(time_from_start=1.0, positions=[1.0, 2.0]),
        ],
    )


def make_invalid_trajectory() -> JointTrajectory:
    return JointTrajectory(joint_names=JOINTS, points=[])


def test_compliance_passes_through_free_space_with_near_zero_offsets() -> None:
    task = JointComplianceTask("compliance", JointComplianceTaskConfig(joint_names=JOINTS))

    output = task.compute_from_reference(make_state(), make_reference([0.2, -0.1]))

    assert output is not None
    assert output.positions == pytest.approx([0.2, -0.1], abs=1e-4)
    diagnostics = task.get_diagnostics()
    assert max(abs(offset) for offset in diagnostics.offsets.values()) < 1e-4
    assert max(abs(velocity) for velocity in diagnostics.offset_velocities.values()) < 1e-2


def test_compliance_bounds_offset_and_velocity_under_tracking_resistance() -> None:
    task = JointComplianceTask(
        "compliance",
        JointComplianceTaskConfig(
            joint_names=JOINTS,
            mass=0.1,
            damping=0.0,
            stiffness=0.0,
            max_offset=0.02,
            max_offset_velocity=0.03,
        ),
    )

    output: JointCommandOutput | None = None
    for step in range(10):
        output = task.compute_from_reference(
            make_state(positions={"arm/j1": 1.0, "arm/j2": -1.0}, dt=0.1, t_now=float(step)),
            make_reference([0.0, 0.0]),
        )

    assert output is not None
    assert output.positions == pytest.approx([0.02, -0.02])
    diagnostics = task.get_diagnostics()
    assert diagnostics.offset_velocities == pytest.approx({"arm/j1": 0.03, "arm/j2": -0.03})
    assert diagnostics.saturated == {"arm/j1": True, "arm/j2": True}


def test_effort_feedback_is_opt_in_and_zero_placeholder_effort_is_ignored_by_default() -> None:
    default_task = JointComplianceTask(
        "default",
        JointComplianceTaskConfig(joint_names=JOINTS, mass=1.0, damping=0.0, stiffness=0.0),
    )
    effort_task = JointComplianceTask(
        "effort",
        JointComplianceTaskConfig(
            joint_names=JOINTS, mass=1.0, damping=0.0, stiffness=0.0, use_effort_feedback=True
        ),
    )

    state = make_state(
        positions={"arm/j1": 1.0, "arm/j2": 0.0}, efforts={"arm/j1": 0.0, "arm/j2": 0.0}, dt=0.1
    )

    default_output = default_task.compute_from_reference(state, make_reference([0.0, 0.0]))
    effort_output = effort_task.compute_from_reference(state, make_reference([0.0, 0.0]))

    assert default_output is not None
    assert effort_output is not None
    assert default_output.positions == pytest.approx([0.01, 0.0])
    assert effort_output.positions == pytest.approx([0.0, 0.0])
    assert default_task.get_diagnostics().feedback_mode == "position_error"
    assert effort_task.get_diagnostics().feedback_mode == "effort"


def test_compliance_invalid_inputs_record_safe_action() -> None:
    task = JointComplianceTask("compliance", JointComplianceTaskConfig(joint_names=JOINTS))

    output = task.compute_from_reference(make_state(dt=0.0), make_reference([0.1, 0.2]))
    assert output is not None
    assert output.positions == [0.1, 0.2]
    assert task.get_diagnostics().last_safe_action == "invalid_dt"

    assert (
        task.compute_from_reference(make_state(positions={"arm/j1": 0.0}), make_reference()) is None
    )
    assert task.get_diagnostics().last_safe_action == "missing_state"

    incompatible = JointCommandOutput(
        joint_names=JOINTS, velocities=[0.0, 0.0], mode=ControlMode.VELOCITY
    )
    assert task.compute_from_reference(make_state(), incompatible) is None
    assert task.get_diagnostics().last_safe_action == "incompatible_reference"


def test_compliance_rejects_non_finite_config_and_runtime_values() -> None:
    with pytest.raises(ValueError, match="must be finite"):
        JointComplianceTask("bad", JointComplianceTaskConfig(joint_names=JOINTS, mass=float("nan")))

    task = JointComplianceTask("compliance", JointComplianceTaskConfig(joint_names=JOINTS))

    assert task.compute_from_reference(make_state(), make_reference([float("nan"), 0.0])) is None
    assert task.get_diagnostics().last_safe_action == "invalid_reference"

    assert (
        task.compute_from_reference(
            make_state(positions={"arm/j1": float("inf"), "arm/j2": 0.0}), make_reference()
        )
        is None
    )
    assert task.get_diagnostics().last_safe_action == "invalid_state"

    effort_task = JointComplianceTask(
        "effort",
        JointComplianceTaskConfig(joint_names=JOINTS, use_effort_feedback=True),
    )
    assert (
        effort_task.compute_from_reference(
            make_state(efforts={"arm/j1": float("nan"), "arm/j2": 0.0}), make_reference()
        )
        is None
    )
    assert effort_task.get_diagnostics().last_safe_action == "invalid_effort"


def test_compliant_trajectory_samples_and_shapes_output() -> None:
    task = create_compliant_joint_trajectory_task(
        "compliant",
        JOINTS,
        compliance_config=JointComplianceTaskConfig(
            joint_names=JOINTS, mass=1.0, damping=0.0, stiffness=0.0
        ),
    )
    assert task.execute(make_trajectory()) is True

    first = task.compute(make_state(t_now=10.0, dt=0.1))
    second = task.compute(make_state(positions={"arm/j1": 1.0, "arm/j2": 0.0}, t_now=10.5, dt=0.1))

    assert first is not None
    assert second is not None
    assert first.positions == pytest.approx([0.0, 0.0])
    assert second.positions == pytest.approx([0.505, 0.99])


def test_compliant_trajectory_completion_cancel_and_preemption_match_rigid_task() -> None:
    task = create_compliant_joint_trajectory_task("compliant", JOINTS)
    assert task.execute(make_trajectory()) is True
    completed = task.compute(make_state(t_now=1.0, dt=0.01))
    assert completed is not None
    assert task.compute(make_state(t_now=2.1, dt=0.01)) is not None
    assert task.get_state() == TrajectoryState.COMPLETED
    assert task.cancel() is False
    assert task.reset() is True
    assert task.get_state() == TrajectoryState.IDLE

    assert task.execute(make_trajectory()) is True
    assert task.cancel() is True
    assert task.get_state() == TrajectoryState.ABORTED

    assert task.reset() is True
    assert task.execute(make_trajectory()) is True
    task.on_preempted("higher", frozenset({"arm/j1"}))
    assert task.get_state() == TrajectoryState.ABORTED
    assert task.get_compliance_diagnostics().last_safe_action == "reset"


def test_compliant_trajectory_completion_returns_nominal_final_and_resets_compliance() -> None:
    task = create_compliant_joint_trajectory_task(
        "compliant",
        JOINTS,
        compliance_config=JointComplianceTaskConfig(
            joint_names=JOINTS, mass=0.1, damping=0.0, stiffness=0.0
        ),
    )
    assert task.execute(make_trajectory()) is True

    shaped = task.compute(make_state(positions={"arm/j1": 1.0, "arm/j2": -1.0}, t_now=0.0, dt=0.1))
    assert shaped is not None
    assert any(abs(position) > 0.0 for position in shaped.positions or [])

    completed = task.compute(
        make_state(positions={"arm/j1": 1.0, "arm/j2": -1.0}, t_now=1.1, dt=0.1)
    )

    assert completed is not None
    assert completed.positions == pytest.approx([1.0, 2.0])
    assert task.get_state() == TrajectoryState.COMPLETED
    diagnostics = task.get_compliance_diagnostics()
    assert diagnostics.offsets == pytest.approx({"arm/j1": 0.0, "arm/j2": 0.0})
    assert diagnostics.last_safe_action == "reset"


def test_rejected_execute_and_forbidden_reset_preserve_compliance_state() -> None:
    task = create_compliant_joint_trajectory_task(
        "compliant",
        JOINTS,
        compliance_config=JointComplianceTaskConfig(
            joint_names=JOINTS, mass=0.1, damping=0.0, stiffness=0.0
        ),
    )
    assert task.execute(make_trajectory()) is True
    output = task.compute(make_state(positions={"arm/j1": 1.0, "arm/j2": -1.0}, t_now=0.0, dt=0.1))
    assert output is not None
    before = task.get_compliance_diagnostics()
    assert any(abs(offset) > 0.0 for offset in before.offsets.values())

    assert task.execute(make_invalid_trajectory()) is False
    after_rejected_execute = task.get_compliance_diagnostics()
    assert after_rejected_execute.offsets == pytest.approx(before.offsets)
    assert task.get_state() == TrajectoryState.EXECUTING

    assert task.reset() is False
    after_rejected_reset = task.get_compliance_diagnostics()
    assert after_rejected_reset.offsets == pytest.approx(before.offsets)
    assert task.get_state() == TrajectoryState.EXECUTING
