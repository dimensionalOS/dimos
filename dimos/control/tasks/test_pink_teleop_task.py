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

"""Tests for Pink-backed teleop IK control tasks."""

from __future__ import annotations

from typing import Any

import numpy as np
from pink.limits import AccelerationLimit
from pink.tasks import DampingTask, FrameTask, ManipulabilityTask, PostureTask
import pytest

from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig, TaskConfig
from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks import pink_teleop_task
from dimos.control.tasks.pink_teleop_task import (
    BasePinkIKTask,
    PiperPinkIKTask,
    PiperPinkIKTaskConfig,
    SingleArmPinkIKTask,
    SingleFramePinkIKTask,
    XArm7IKTask,
    XArm7IKTaskConfig,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.catalog.piper import PIPER_FK_MODEL
from dimos.robot.catalog.ufactory import XARM7_FK_MODEL

XARM7_JOINTS = [f"arm/joint{i}" for i in range(1, 8)]
PIPER_JOINTS = [f"arm/joint{i}" for i in range(1, 7)]


def _state(*, t_now: float = 1.0, dt: float = 0.01) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions={name: 0.0 for name in XARM7_JOINTS}),
        t_now=t_now,
        dt=dt,
    )


def _task(
    *,
    timeout: float = 0.5,
    max_joint_delta_deg: float = 5.0,
    gripper_joint: str | None = None,
    posture_cost: float = 0.0,
    posture_reference: dict[str, float] | None = None,
    posture_lm_damping: float = 0.0,
    posture_gain: float = 1.0,
    damping_task_cost: float = 0.0,
    gain: float = 1.0,
    max_joint_acceleration: float = 0.0,
    velocity_smoothing_alpha: float = 1.0,
    manipulability_cost: float = 0.0,
    manipulability_lm_damping: float = 1e-3,
    manipulability_gain: float = 1.0,
    manipulability_rate: float = 1.5,
) -> XArm7IKTask:
    config = XArm7IKTaskConfig(
        joint_names=XARM7_JOINTS,
        model_path=XARM7_FK_MODEL,
        timeout=timeout,
        max_joint_delta_deg=max_joint_delta_deg,
        gripper_joint=gripper_joint,
        gain=gain,
        posture_cost=posture_cost,
        posture_reference=posture_reference or {},
        posture_lm_damping=posture_lm_damping,
        posture_gain=posture_gain,
        damping_task_cost=damping_task_cost,
        max_joint_acceleration=max_joint_acceleration,
        velocity_smoothing_alpha=velocity_smoothing_alpha,
        manipulability_cost=manipulability_cost,
        manipulability_lm_damping=manipulability_lm_damping,
        manipulability_gain=manipulability_gain,
        manipulability_rate=manipulability_rate,
    )
    return XArm7IKTask("teleop_xarm", config)


def _piper_task(**overrides: Any) -> SingleArmPinkIKTask:
    kwargs: dict[str, Any] = {
        "joint_names": PIPER_JOINTS,
        "model_path": PIPER_FK_MODEL,
        "end_effector_frame": "gripper_base",
        "hand": "left",
    }
    kwargs.update(overrides)
    config = PiperPinkIKTaskConfig(**kwargs)
    return PiperPinkIKTask("teleop_piper", config)


def test_missing_joint_state_returns_no_command() -> None:
    task = _task()
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)
    state = CoordinatorState(
        joints=JointStateSnapshot(joint_positions={name: 0.0 for name in XARM7_JOINTS[:-1]}),
        t_now=1.0,
        dt=0.01,
    )

    assert task.compute(state) is None


def test_timeout_clears_active_target_state() -> None:
    task = _task(timeout=0.1)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)

    assert task.compute(_state(t_now=1.2)) is None
    assert not task.is_active()


def test_timeout_clears_active_target_state_before_joint_extraction() -> None:
    task = _task(timeout=0.1)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)
    state = CoordinatorState(
        joints=JointStateSnapshot(joint_positions={name: 0.0 for name in XARM7_JOINTS[:-1]}),
        t_now=1.2,
        dt=0.01,
    )

    assert task.compute(state) is None
    assert not task.is_active()


def test_inactive_state_returns_no_command() -> None:
    task = _task()

    assert task.compute(_state()) is None


def test_single_target_state_lives_in_single_frame_layer() -> None:
    assert issubclass(XArm7IKTask, SingleFramePinkIKTask)
    assert not hasattr(BasePinkIKTask, "_get_live_target")
    assert not hasattr(BasePinkIKTask, "_single_frame_task")
    assert not hasattr(BasePinkIKTask, "_primary_frame_name")


def test_unsafe_joint_delta_is_rejected(monkeypatch: pytest.MonkeyPatch) -> None:
    task = _task(max_joint_delta_deg=1.0)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)

    def large_velocity(*_args: object, **_kwargs: object) -> np.ndarray:
        return np.ones(7) * 100.0

    monkeypatch.setattr(pink_teleop_task, "solve_ik", large_velocity)

    assert task.compute(_state(t_now=1.0, dt=0.1)) is None


def test_posture_task_is_pink_posture_task() -> None:
    task = _task(posture_cost=0.2)
    posture_task = task._posture_task
    assert isinstance(posture_task, PostureTask)

    task._configuration.update(np.zeros(7, dtype=float))
    posture_task.set_target(np.ones(7, dtype=float))

    error = posture_task.compute_error(task._configuration)
    jacobian = posture_task.compute_jacobian(task._configuration)

    assert error.shape == (7,)
    assert jacobian.shape == (7, 7)
    assert np.all(np.isfinite(error))
    assert np.all(np.isfinite(jacobian))


def test_posture_disabled_keeps_frame_task_only() -> None:
    task = _task(posture_cost=0.0)

    assert task._posture_task is None
    assert len(task._pink_tasks) == 1


def test_damping_disabled_by_default_keeps_frame_task_only() -> None:
    task = _task()

    assert task._posture_task is None
    assert len(task._pink_tasks) == 1
    assert isinstance(task._pink_tasks[0], FrameTask)


def test_damping_enabled_adds_damping_task_to_solve(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = _task(damping_task_cost=0.001)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)
    solve_tasks: list[object] = []

    def capture_tasks(
        _configuration: object, tasks: list[object], *_args: object, **_kwargs: object
    ) -> np.ndarray:
        solve_tasks.extend(tasks)
        return np.zeros(7, dtype=float)

    monkeypatch.setattr(pink_teleop_task, "solve_ik", capture_tasks)

    assert task.compute(_state(t_now=1.0, dt=0.1)) is not None
    assert task._frame_tasks[0] in solve_tasks
    assert any(isinstance(solve_task, DampingTask) for solve_task in solve_tasks)


def test_posture_and_damping_tasks_are_both_included() -> None:
    task = _task(posture_cost=0.2, damping_task_cost=0.001)

    assert task._posture_task is not None
    assert task._posture_task in task._pink_tasks
    assert any(isinstance(pink_task, DampingTask) for pink_task in task._pink_tasks)


def test_posture_enabled_preserves_frame_task_in_solve(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = _task(posture_cost=0.2)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)
    solve_tasks: list[object] = []

    def capture_tasks(
        _configuration: object, tasks: list[object], *_args: object, **_kwargs: object
    ) -> np.ndarray:
        solve_tasks.extend(tasks)
        return np.zeros(7, dtype=float)

    monkeypatch.setattr(pink_teleop_task, "solve_ik", capture_tasks)

    assert task.compute(_state(t_now=1.0, dt=0.1)) is not None
    assert task._frame_tasks[0] in solve_tasks
    assert task._posture_task in solve_tasks


def test_pink_ik_uses_single_official_solve_and_integrate_step(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = _task()
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)
    calls: list[float] = []

    def constant_velocity(
        _configuration: object,
        _tasks: list[object],
        dt: float,
        **_kwargs: object,
    ) -> np.ndarray:
        calls.append(dt)
        return np.ones(7, dtype=float) * 0.1

    monkeypatch.setattr(pink_teleop_task, "solve_ik", constant_velocity)

    output = task.compute(_state(t_now=1.0, dt=0.1))

    assert output is not None
    assert calls == [0.1]
    assert output.positions is not None
    assert np.allclose(output.positions, np.full(7, 0.01))


def test_posture_reference_changes_observable_ik_output() -> None:
    state = _state(t_now=1.0, dt=0.01)
    frame_only = _task(max_joint_delta_deg=180.0)
    frame_only.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)

    posture = _task(
        max_joint_delta_deg=180.0,
        posture_cost=0.1,
        posture_reference={"arm/joint3": 0.5},
    )
    posture.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)

    frame_only_output = frame_only.compute(state)
    posture_output = posture.compute(state)

    assert frame_only_output is not None
    assert posture_output is not None
    assert frame_only_output.positions is not None
    assert posture_output.positions is not None
    assert not np.allclose(frame_only_output.positions, posture_output.positions)


def test_posture_reference_maps_joint_targets() -> None:
    task = _task(
        posture_cost=0.2,
        posture_reference={"arm/joint3": 1.25, "joint5": -0.5},
    )

    assert task._posture_task is not None

    q_current = np.arange(7, dtype=float)
    assert task._update_extra_task_targets(q_current)

    assert task._posture_task.target_q is not None
    assert np.allclose(task._posture_task.target_q, [0.0, 1.0, 1.25, 3.0, -0.5, 5.0, 6.0])


def test_posture_target_refreshes_from_current_joint_state() -> None:
    task = _task(posture_cost=0.2, posture_reference={"arm/joint3": 1.25})
    assert task._posture_task is not None

    first = np.zeros(7, dtype=float)
    second = np.arange(7, dtype=float) * 0.1

    assert task._update_extra_task_targets(first)
    assert task._posture_task.target_q is not None
    assert np.allclose(task._posture_task.target_q, [0.0, 0.0, 1.25, 0.0, 0.0, 0.0, 0.0])

    assert task._update_extra_task_targets(second)
    assert task._posture_task.target_q is not None
    assert np.allclose(task._posture_task.target_q, [0.0, 0.1, 1.25, 0.3, 0.4, 0.5, 0.6])


def test_posture_reference_rejects_unknown_joint_name() -> None:
    with pytest.raises(ValueError, match="posture joint"):
        _task(posture_cost=0.2, posture_reference={"missing_joint": 0.0})


@pytest.mark.parametrize(
    ("field", "value", "match"),
    [
        ("posture_cost", -0.1, "posture_cost"),
        ("posture_lm_damping", float("nan"), "posture_lm_damping"),
        ("posture_gain", float("inf"), "posture_gain"),
        ("posture_gain", 1.1, "posture_gain"),
        ("damping_task_cost", -0.1, "damping_task_cost"),
        ("damping_task_cost", float("nan"), "damping_task_cost"),
        ("gain", 1.1, "gain"),
        ("max_joint_delta_deg", float("inf"), "max_joint_delta_deg"),
        ("timeout", -1.0, "timeout"),
        ("max_joint_acceleration", -1.0, "max_joint_acceleration"),
        ("max_joint_acceleration", float("nan"), "max_joint_acceleration"),
        ("velocity_smoothing_alpha", -0.1, "velocity_smoothing_alpha"),
        ("velocity_smoothing_alpha", 1.5, "velocity_smoothing_alpha"),
        ("manipulability_cost", -0.1, "manipulability_cost"),
        ("manipulability_lm_damping", -1e-3, "manipulability_lm_damping"),
        ("manipulability_gain", 1.1, "manipulability_gain"),
        ("manipulability_rate", float("nan"), "manipulability_rate"),
    ],
)
def test_invalid_numeric_config_is_rejected(field: str, value: float, match: str) -> None:
    with pytest.raises(ValueError, match=match):
        if field == "posture_cost":
            _task(posture_cost=value)
        elif field == "posture_lm_damping":
            _task(posture_lm_damping=value)
        elif field == "posture_gain":
            _task(posture_gain=value)
        elif field == "damping_task_cost":
            _task(damping_task_cost=value)
        elif field == "gain":
            _task(gain=value)
        elif field == "max_joint_delta_deg":
            _task(max_joint_delta_deg=value)
        elif field == "timeout":
            _task(timeout=value)
        elif field == "max_joint_acceleration":
            _task(max_joint_acceleration=value)
        elif field == "velocity_smoothing_alpha":
            _task(velocity_smoothing_alpha=value)
        elif field == "manipulability_cost":
            _task(manipulability_cost=value)
        elif field == "manipulability_lm_damping":
            _task(manipulability_lm_damping=value)
        elif field == "manipulability_gain":
            _task(manipulability_gain=value)
        elif field == "manipulability_rate":
            _task(manipulability_rate=value)
        else:
            raise AssertionError(f"Unhandled field: {field}")


def test_out_of_limit_posture_reference_is_rejected() -> None:
    with pytest.raises(ValueError, match="outside model limits"):
        _task(posture_cost=0.2, posture_reference={"arm/joint2": 1000.0})


def test_joint_state_just_past_urdf_limit_does_not_fail_ik() -> None:
    task = _task()
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)

    # Padded upper limit is original + joint_limit_margin (default 1e-3 rad).
    # Sit a joint a few hundred µrad past the original URDF upper bound —
    # well inside the margin — and confirm Pink's Configuration.update no
    # longer raises "violates configuration limits".
    padded_upper = float(task._model.upperPositionLimit[0])
    overshoot = padded_upper - 5e-4
    positions = {name: 0.0 for name in XARM7_JOINTS}
    positions[XARM7_JOINTS[0]] = overshoot
    state = CoordinatorState(
        joints=JointStateSnapshot(joint_positions=positions),
        t_now=1.0,
        dt=0.01,
    )

    output = task.compute(state)
    assert output is not None
    assert output.mode == ControlMode.SERVO_POSITION


def test_negative_joint_limit_margin_is_rejected() -> None:
    config = XArm7IKTaskConfig(
        joint_names=XARM7_JOINTS,
        model_path=XARM7_FK_MODEL,
        joint_limit_margin=-0.1,
    )
    with pytest.raises(ValueError, match="joint_limit_margin"):
        XArm7IKTask("teleop_xarm", config)


def test_posture_enabled_solver_failure_returns_no_command(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = _task(posture_cost=0.2)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)

    def fail_solve(*_args: object, **_kwargs: object) -> np.ndarray:
        raise RuntimeError("solver failed")

    monkeypatch.setattr(pink_teleop_task, "solve_ik", fail_solve)

    assert task.compute(_state(t_now=1.0, dt=0.1)) is None


def test_damping_enabled_solver_failure_returns_no_command(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = _task(damping_task_cost=0.001)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)

    def fail_solve(*_args: object, **_kwargs: object) -> np.ndarray:
        raise RuntimeError("solver failed")

    monkeypatch.setattr(pink_teleop_task, "solve_ik", fail_solve)

    assert task.compute(_state(t_now=1.0, dt=0.1)) is None


def test_posture_enabled_unsafe_joint_delta_is_rejected(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = _task(posture_cost=0.2, max_joint_delta_deg=1.0)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)

    def large_velocity(*_args: object, **_kwargs: object) -> np.ndarray:
        return np.ones(7) * 100.0

    monkeypatch.setattr(pink_teleop_task, "solve_ik", large_velocity)

    assert task.compute(_state(t_now=1.0, dt=0.1)) is None


def test_damping_enabled_unsafe_joint_delta_is_rejected(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = _task(damping_task_cost=0.001, max_joint_delta_deg=1.0)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)

    def large_velocity(*_args: object, **_kwargs: object) -> np.ndarray:
        return np.ones(7) * 100.0

    monkeypatch.setattr(pink_teleop_task, "solve_ik", large_velocity)

    assert task.compute(_state(t_now=1.0, dt=0.1)) is None


def test_non_finite_ik_output_is_rejected(monkeypatch: pytest.MonkeyPatch) -> None:
    task = _task(posture_cost=0.2)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)

    def non_finite_velocity(*_args: object, **_kwargs: object) -> np.ndarray:
        return np.full(7, np.nan)

    monkeypatch.setattr(pink_teleop_task, "solve_ik", non_finite_velocity)

    assert task.compute(_state(t_now=1.0, dt=0.1)) is None


def test_xarm7_task_claims_arm_and_gripper() -> None:
    task = _task(gripper_joint="arm/gripper")

    claim = task.claim()

    assert claim.mode == ControlMode.SERVO_POSITION
    assert claim.joints == frozenset([*XARM7_JOINTS, "arm/gripper"])


def test_xarm7_construction_rejects_mismatched_joint_names() -> None:
    with pytest.raises(ValueError, match="joint names"):
        XArm7IKTask(
            "teleop_xarm",
            XArm7IKTaskConfig(
                joint_names=[f"arm/bad{i}" for i in range(1, 8)],
                model_path=XARM7_FK_MODEL,
            ),
        )


def test_xarm7_construction_rejects_missing_end_effector_frame() -> None:
    with pytest.raises(ValueError, match="no frame"):
        XArm7IKTask(
            "teleop_xarm",
            XArm7IKTaskConfig(
                joint_names=XARM7_JOINTS,
                model_path=XARM7_FK_MODEL,
                end_effector_frame="missing_frame",
            ),
        )


def test_single_arm_pink_task_constructs_configurable_piper_frame_task() -> None:
    task = _piper_task()

    assert task._config.model_path == PIPER_FK_MODEL
    assert task._config.end_effector_frame == "gripper_base"
    assert task._config.hand == "left"
    assert task._model.nq == len(PIPER_JOINTS)
    assert task._model.existFrame("gripper_base")
    assert [str(frame_task.frame) for frame_task in task._frame_tasks] == ["gripper_base"]
    assert isinstance(task._frame_tasks[0], FrameTask)


def test_single_arm_pink_task_rejects_piper_mismatched_joint_names() -> None:
    with pytest.raises(ValueError, match="joint names"):
        _piper_task(joint_names=[f"arm/bad{i}" for i in range(1, 7)])


def test_single_arm_pink_task_rejects_missing_piper_end_effector_frame() -> None:
    with pytest.raises(ValueError, match="no frame"):
        _piper_task(end_effector_frame="missing_frame")


def test_single_arm_pink_task_claims_piper_arm_and_gripper() -> None:
    task = _piper_task(
        gripper_joint="arm/gripper",
        gripper_open_pos=0.0,
        gripper_closed_pos=0.035,
    )

    claim = task.claim()
    assert claim.mode == ControlMode.SERVO_POSITION
    assert claim.joints == frozenset([*PIPER_JOINTS, "arm/gripper"])

    assert task.on_gripper_trigger(1.0)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_piper"), t_now=1.0)
    output = task.compute(
        CoordinatorState(
            joints=JointStateSnapshot(joint_positions={name: 0.0 for name in PIPER_JOINTS}),
            t_now=1.0,
            dt=0.01,
        )
    )
    assert output is not None
    assert output.joint_names[-1] == "arm/gripper"
    assert output.positions[-1] == 0.035


def test_right_controller_pose_activates_without_left_controller_data() -> None:
    task = _task()

    assert task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)
    assert task.is_active()


def test_coordinator_creates_xarm7_pink_task_with_teleop_route_key() -> None:
    coordinator = ControlCoordinator.__new__(ControlCoordinator)

    task = coordinator._create_task_from_config(
        TaskConfig(
            name="teleop_xarm",
            type="xarm7_pink_ik",
            joint_names=XARM7_JOINTS,
            model_path=XARM7_FK_MODEL,
            hand="right",
        )
    )

    assert isinstance(task, XArm7IKTask)
    assert task.name == "teleop_xarm"
    assert task._config.end_effector_frame == "link7"


def test_coordinator_requires_end_effector_frame_for_single_arm_pink_task() -> None:
    coordinator = ControlCoordinator.__new__(ControlCoordinator)

    for end_effector_frame in (None, ""):
        with pytest.raises(ValueError, match="end_effector_frame"):
            coordinator._create_task_from_config(
                TaskConfig(
                    name="teleop_piper",
                    type="single_arm_pink_ik",
                    joint_names=PIPER_JOINTS,
                    model_path=PIPER_FK_MODEL,
                    end_effector_frame=end_effector_frame,
                    hand="left",
                )
            )


def test_coordinator_creates_xarm7_pink_task_with_task_local_defaults() -> None:
    coordinator = ControlCoordinator.__new__(ControlCoordinator)

    task = coordinator._create_task_from_config(
        TaskConfig(
            name="teleop_xarm",
            type="xarm7_pink_ik",
            joint_names=XARM7_JOINTS,
            model_path=XARM7_FK_MODEL,
            hand="right",
        )
    )

    assert isinstance(task, XArm7IKTask)
    assert task._config.damping == 1e-12
    assert task._config.position_cost == 1.0
    assert task._config.orientation_cost == 1.0
    assert task._config.lm_damping == 1.0
    assert task._config.gain == 1.0
    assert task._config.posture_cost == 1e-3
    assert task._config.posture_reference == {}
    assert task._config.posture_lm_damping == 0.0
    assert task._config.posture_gain == 1.0
    assert task._config.damping_task_cost == 1e-3
    assert task._posture_task is not None
    assert any(isinstance(pink_task, DampingTask) for pink_task in task._pink_tasks)


def test_coordinator_rejects_removed_openarm_bimanual_pink_task_type() -> None:
    coordinator = ControlCoordinator.__new__(ControlCoordinator)

    with pytest.raises(ValueError, match="Unknown task type"):
        coordinator._create_task_from_config(
            TaskConfig(
                name="teleop_openarm",
                type="openarm_bimanual_pink_ik",
                model_path=XARM7_FK_MODEL,
            )
        )


def test_xarm7_pink_task_requests_cartesian_and_button_subscriptions() -> None:
    coordinator = ControlCoordinator.__new__(ControlCoordinator)
    coordinator.config = ControlCoordinatorConfig(
        tasks=[
            TaskConfig(
                name="teleop_xarm",
                type="xarm7_pink_ik",
                joint_names=XARM7_JOINTS,
                model_path=XARM7_FK_MODEL,
                hand="right",
            )
        ]
    )

    assert coordinator._has_cartesian_target_task()
    assert coordinator._has_teleop_task()


def test_single_arm_pink_task_requests_cartesian_and_button_subscriptions() -> None:
    coordinator = ControlCoordinator.__new__(ControlCoordinator)
    coordinator.config = ControlCoordinatorConfig(
        tasks=[
            TaskConfig(
                name="teleop_piper",
                type="piper_pink_ik",
                joint_names=PIPER_JOINTS,
                model_path=PIPER_FK_MODEL,
                hand="right",
            )
        ]
    )

    assert coordinator._has_cartesian_target_task()
    assert coordinator._has_teleop_task()


def test_acceleration_limit_disabled_when_max_joint_acceleration_zero(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = _task(max_joint_acceleration=0.0)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)
    captured_kwargs: dict[str, Any] = {}

    def capture(
        _configuration: object, _tasks: list[object], _dt: float, **kwargs: Any
    ) -> np.ndarray:
        captured_kwargs.update(kwargs)
        return np.zeros(7, dtype=float)

    monkeypatch.setattr(pink_teleop_task, "solve_ik", capture)
    assert task.compute(_state(t_now=1.0, dt=0.1)) is not None
    assert "limits" not in captured_kwargs
    assert task._acceleration_limit is None


def test_acceleration_limit_passed_to_solve_when_enabled(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = _task(max_joint_acceleration=20.0)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)
    captured_kwargs: dict[str, Any] = {}

    def capture(
        _configuration: object, _tasks: list[object], _dt: float, **kwargs: Any
    ) -> np.ndarray:
        captured_kwargs.update(kwargs)
        return np.zeros(7, dtype=float)

    monkeypatch.setattr(pink_teleop_task, "solve_ik", capture)
    assert task.compute(_state(t_now=1.0, dt=0.1)) is not None

    assert "limits" in captured_kwargs
    limits = list(captured_kwargs["limits"])
    accel_limits = [lim for lim in limits if isinstance(lim, AccelerationLimit)]
    assert len(accel_limits) == 1
    assert np.allclose(accel_limits[0].a_max, np.full(7, 20.0))


def test_velocity_smoothing_alpha_passthrough_when_one(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = _task(velocity_smoothing_alpha=1.0)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)
    velocities = iter([np.full(7, 0.10), np.full(7, 0.30)])

    def stream(*_args: object, **_kwargs: object) -> np.ndarray:
        return next(velocities)

    monkeypatch.setattr(pink_teleop_task, "solve_ik", stream)

    first = task.compute(_state(t_now=1.0, dt=0.1))
    second = task.compute(_state(t_now=1.1, dt=0.1))
    assert first is not None
    assert second is not None
    assert first.positions is not None
    assert second.positions is not None
    # No EMA. State snapshot resets joints to zero each tick, so position is v*dt.
    assert np.allclose(first.positions, np.full(7, 0.01))
    assert np.allclose(second.positions, np.full(7, 0.03))


def test_velocity_smoothing_alpha_blends_previous_velocity(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = _task(velocity_smoothing_alpha=0.5)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)
    velocities = iter([np.full(7, 0.10), np.full(7, 0.30)])

    def stream(*_args: object, **_kwargs: object) -> np.ndarray:
        return next(velocities)

    monkeypatch.setattr(pink_teleop_task, "solve_ik", stream)

    first = task.compute(_state(t_now=1.0, dt=0.1))
    second = task.compute(_state(t_now=1.1, dt=0.1))
    assert first is not None
    assert second is not None
    assert first.positions is not None
    assert second.positions is not None
    # First tick: no prev -> raw 0.10 * 0.1 = 0.01 from zero state snapshot.
    assert np.allclose(first.positions, np.full(7, 0.01))
    # Second tick: blended v = 0.5*0.30 + 0.5*0.10 = 0.20; pos = 0 + 0.20 * 0.1 = 0.02
    assert np.allclose(second.positions, np.full(7, 0.02))


def test_velocity_smoothing_state_resets_on_disengage(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = _task(timeout=0.1, velocity_smoothing_alpha=0.5)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)
    velocities = iter([np.full(7, 0.10), np.full(7, 0.30)])

    def stream(*_args: object, **_kwargs: object) -> np.ndarray:
        return next(velocities)

    monkeypatch.setattr(pink_teleop_task, "solve_ik", stream)

    first = task.compute(_state(t_now=1.0, dt=0.1))
    assert first is not None
    assert task._prev_velocity is not None

    # Force a timeout-driven disengage; the task should clear smoothing state.
    assert task.compute(_state(t_now=1.5, dt=0.1)) is None
    assert task._prev_velocity is None

    # Re-engage: next velocity must be applied raw (no carryover from prev engage).
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=2.0)
    resumed = task.compute(_state(t_now=2.0, dt=0.1))
    assert resumed is not None
    assert resumed.positions is not None
    # State snapshot is zero again; first post-reengage velocity is applied raw (no prev).
    assert np.allclose(resumed.positions, np.full(7, 0.03))


def test_manipulability_disabled_by_default() -> None:
    task = _task()

    assert not any(isinstance(t, ManipulabilityTask) for t in task._pink_tasks)


def test_manipulability_enabled_adds_task() -> None:
    task = _task(manipulability_cost=0.3, manipulability_rate=1.5)

    manip = next((t for t in task._pink_tasks if isinstance(t, ManipulabilityTask)), None)
    assert manip is not None
    assert manip.cost == 0.3
    assert manip.manipulability_rate == 1.5


def test_manipulability_enabled_is_passed_to_solve(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = _task(manipulability_cost=0.3)
    task.on_cartesian_command(PoseStamped(frame_id="teleop_xarm"), t_now=1.0)
    solve_tasks: list[object] = []

    def capture(
        _configuration: object, tasks: list[object], *_args: object, **_kwargs: object
    ) -> np.ndarray:
        solve_tasks.extend(tasks)
        return np.zeros(7, dtype=float)

    monkeypatch.setattr(pink_teleop_task, "solve_ik", capture)

    assert task.compute(_state(t_now=1.0, dt=0.1)) is not None
    assert any(isinstance(t, ManipulabilityTask) for t in solve_tasks)
