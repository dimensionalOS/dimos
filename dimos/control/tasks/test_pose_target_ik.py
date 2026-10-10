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

"""Behavior tests for the shared pose-target IK control core."""

from pathlib import Path

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.sensor_msgs.msg import JointState
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.pose_target_ik import (
    FrameTargetSnapshot,
    PinkJointLimitError,
    PinkPoseTargetSolver,
    PoseTargetIKTask,
    PoseTargetIKTaskConfig,
    _StreamingStepResult,
)
from dimos.manipulation.planning.kinematics.pink_solver import _PinkSolverCore
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel


class _Task(PoseTargetIKTask):
    def __init__(
        self,
        config: PoseTargetIKTaskConfig,
        solver: PinkPoseTargetSolver,
        snapshot: FrameTargetSnapshot | None,
        additional_joints: tuple[str, ...] = (),
    ) -> None:
        self.snapshot = snapshot
        self.timed_out = False
        super().__init__(
            "pose_target",
            config,
            additional_claimed_joints=additional_joints,
            solver=solver,
        )

    def is_active(self) -> bool:
        return self.snapshot is not None

    def _frame_target_snapshot(self, state: CoordinatorState) -> FrameTargetSnapshot | None:
        return self.snapshot

    def _on_target_timeout(self) -> None:
        self.timed_out = True


def _robot_model() -> RobotModelConfig:
    return RobotModelConfig(
        model=RobotModel.from_file(Path("fake.urdf")),
        joint_names=["arm/a", "arm/b"],
    )


def _config(
    *,
    joint_names: tuple[str, ...] = ("arm/a", "arm/b"),
    target_frames: tuple[str, ...] = ("tool",),
    timeout: float = 0.5,
    max_joint_velocity_rad_s: float = 5.0,
    joint_velocity_limits_rad_s: dict[str, float] | None = None,
    joint_command_filter_cutoff_hz: float | None = 5.0,
    max_command_tracking_error_deg: float = 10.0,
) -> PoseTargetIKTaskConfig:
    return PoseTargetIKTaskConfig(
        joint_names=joint_names,
        robot_model=_robot_model(),
        target_frames=target_frames,
        timeout=timeout,
        max_joint_velocity_rad_s=max_joint_velocity_rad_s,
        joint_velocity_limits_rad_s=joint_velocity_limits_rad_s or {},
        joint_command_filter_cutoff_hz=joint_command_filter_cutoff_hz,
        max_command_tracking_error_deg=max_command_tracking_error_deg,
    )


def _solver(mocker: MockerFixture, positions: list[float] | None = None) -> PinkPoseTargetSolver:
    solver = mocker.Mock(spec=PinkPoseTargetSolver)
    solver.step.return_value = JointState(
        name=["arm/a", "arm/b"],
        position=np.asarray(positions or [0.01, -0.01], dtype=np.float64),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        velocity=np.array([], dtype=np.float64),
        effort=np.array([], dtype=np.float64),
    )
    solver.frame_poses.return_value = {
        "tool": PoseStamped(
            header=Header(frame_id="base", stamp=Time(sec=0, nanosec=0)),
            pose=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
    }
    return solver


def _state(*, t_now: float = 1.0, positions: dict[str, float] | None = None) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions=positions or {"arm/a": 0.0, "arm/b": 0.0}),
        t_now=t_now,
        dt=0.01,
    )


def _snapshot(
    *,
    targets: dict[str, PoseStamped] | None = None,
    last_update_time: float = 1.0,
    extra_joint_positions: dict[str, float] | None = None,
) -> FrameTargetSnapshot:
    return FrameTargetSnapshot(
        targets=targets
        or {
            "tool": PoseStamped(
                header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
                pose=Pose(
                    position=Point(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            )
        },
        last_update_time=last_update_time,
        extra_joint_positions=extra_joint_positions or {},
    )


def _stateful_solver(mocker: MockerFixture) -> PinkPoseTargetSolver:
    mocker.patch.object(_PinkSolverCore, "__init__", return_value=None)
    mocker.patch.object(PinkPoseTargetSolver, "_validate_frame_targets")
    return PinkPoseTargetSolver(_config())


def test_pose_target_solver_advances_from_last_command_not_delayed_feedback(
    mocker: MockerFixture,
) -> None:
    solver = _stateful_solver(mocker)

    def advance(**kwargs: object) -> _StreamingStepResult:
        command = kwargs["command_state"]
        assert isinstance(command, JointState)
        return _StreamingStepResult(
            command=JointState(
                name=list(command.name),
                position=np.asarray(
                    [position + 0.1 for position in command.position], dtype=np.float64
                ),
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
                velocity=np.array([], dtype=np.float64),
                effort=np.array([], dtype=np.float64),
            ),
            bounded_increment=np.array([0.1, 0.1]),
        )

    step = mocker.patch.object(solver, "_step_frame_targets", side_effect=advance)
    targets = {
        "tool": PoseStamped(
            header=Header(frame_id="", stamp=Time(sec=0, nanosec=0)),
            pose=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
    }

    assert solver.step(
        targets,
        JointState(
            name=["arm/a", "arm/b"],
            position=np.array([0.0, 0.0], dtype=np.float64),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            velocity=np.array([], dtype=np.float64),
            effort=np.array([], dtype=np.float64),
        ),
        0.01,
    )
    assert solver.step(
        targets,
        JointState(
            name=["arm/a", "arm/b"],
            position=np.array([-0.3, -0.3], dtype=np.float64),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            velocity=np.array([], dtype=np.float64),
            effort=np.array([], dtype=np.float64),
        ),
        0.01,
    )

    np.testing.assert_array_equal(
        step.call_args_list[0].kwargs["command_state"].position, [0.0, 0.0]
    )
    np.testing.assert_array_equal(
        step.call_args_list[1].kwargs["command_state"].position, [0.1, 0.1]
    )
    np.testing.assert_array_equal(
        step.call_args_list[1].kwargs["measured_state"].position, [-0.3, -0.3]
    )
    assert step.call_args_list[1].kwargs["command_increment_history"][0] == pytest.approx(
        [0.1, 0.1]
    )
    assert step.call_args_list[1].kwargs["joint_command_filter_cutoff_hz"] == 5.0
    assert step.call_args_list[1].kwargs["joint_velocity_limits_rad_s"] == {}


def test_pose_target_solver_reset_reseeds_from_feedback(mocker: MockerFixture) -> None:
    solver = _stateful_solver(mocker)
    step = mocker.patch.object(
        solver,
        "_step_frame_targets",
        side_effect=[
            _StreamingStepResult(
                command=JointState(
                    name=["arm/a", "arm/b"],
                    position=np.array([0.1, 0.1], dtype=np.float64),
                    header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
                    velocity=np.array([], dtype=np.float64),
                    effort=np.array([], dtype=np.float64),
                ),
                bounded_increment=np.array([0.1, 0.1]),
            ),
            _StreamingStepResult(
                command=JointState(
                    name=["arm/a", "arm/b"],
                    position=np.array([-0.2, -0.2], dtype=np.float64),
                    header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
                    velocity=np.array([], dtype=np.float64),
                    effort=np.array([], dtype=np.float64),
                ),
                bounded_increment=np.array([0.1, 0.1]),
            ),
        ],
    )
    targets = {
        "tool": PoseStamped(
            header=Header(frame_id="", stamp=Time(sec=0, nanosec=0)),
            pose=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
    }

    assert solver.step(
        targets,
        JointState(
            name=["arm/a", "arm/b"],
            position=np.array([0.0, 0.0], dtype=np.float64),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            velocity=np.array([], dtype=np.float64),
            effort=np.array([], dtype=np.float64),
        ),
        0.01,
    )
    solver.reset()
    assert solver.step(
        targets,
        JointState(
            name=["arm/a", "arm/b"],
            position=np.array([-0.3, -0.3], dtype=np.float64),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            velocity=np.array([], dtype=np.float64),
            effort=np.array([], dtype=np.float64),
        ),
        0.01,
    )

    np.testing.assert_array_equal(
        step.call_args_list[1].kwargs["command_state"].position, [-0.3, -0.3]
    )
    assert step.call_args_list[1].kwargs["command_increment_history"] == ()


def test_pose_target_solver_reset_during_step_discards_command_and_filter_history(
    mocker: MockerFixture,
) -> None:
    solver = _stateful_solver(mocker)

    def reset_during_step(**_kwargs: object) -> _StreamingStepResult:
        solver.reset()
        return _StreamingStepResult(
            command=JointState(
                name=["arm/a", "arm/b"],
                position=np.array([0.1, 0.1], dtype=np.float64),
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
                velocity=np.array([], dtype=np.float64),
                effort=np.array([], dtype=np.float64),
            ),
            bounded_increment=np.array([0.1, 0.1]),
        )

    mocker.patch.object(solver, "_step_frame_targets", side_effect=reset_during_step)

    result = solver.step(
        {
            "tool": PoseStamped(
                header=Header(frame_id="", stamp=Time(sec=0, nanosec=0)),
                pose=Pose(
                    position=Point(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            )
        },
        JointState(
            name=["arm/a", "arm/b"],
            position=np.array([0.0, 0.0], dtype=np.float64),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            velocity=np.array([], dtype=np.float64),
            effort=np.array([], dtype=np.float64),
        ),
        0.01,
    )

    assert result is None
    assert solver._command_state is None
    assert tuple(solver._command_increment_history) == ()


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("joint_names", (), "at least one joint"),
        ("joint_names", ("arm/a", "arm/a"), "unique joint names"),
        ("target_frames", (), "at least one target frame"),
        ("target_frames", ("tool", "tool"), "unique target frames"),
    ],
)
def test_constructor_rejects_invalid_common_configuration(
    mocker: MockerFixture, field: str, value: tuple[str, ...], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        config = (
            _config(joint_names=value) if field == "joint_names" else _config(target_frames=value)
        )
        _Task(config, _solver(mocker), _snapshot())


def test_compute_calls_one_pink_step_and_preserves_output_order(
    mocker: MockerFixture,
) -> None:
    solver = _solver(mocker, [0.02, -0.03])
    task = _Task(
        _config(),
        solver,
        _snapshot(extra_joint_positions={"arm/gripper": 0.4}),
        additional_joints=("arm/gripper",),
    )

    output = task.compute(_state())

    assert output is not None
    assert output.joint_names == ["arm/a", "arm/b", "arm/gripper"]
    np.testing.assert_array_equal(output.positions, [0.02, -0.03, 0.4])
    solver.step.assert_called_once()
    assert solver.step.call_args.args[2] == 0.01
    assert task.claim().joints == frozenset({"arm/a", "arm/b", "arm/gripper"})


@pytest.mark.parametrize("max_joint_velocity_rad_s", [0.0, -1.0, float("inf"), float("nan")])
def test_constructor_rejects_invalid_joint_velocity_limit(
    mocker: MockerFixture, max_joint_velocity_rad_s: float
) -> None:
    with pytest.raises(ValueError, match="positive finite joint velocity limit"):
        _Task(
            _config(max_joint_velocity_rad_s=max_joint_velocity_rad_s),
            _solver(mocker),
            _snapshot(),
        )


@pytest.mark.parametrize("cutoff_hz", [0.0, -1.0, float("inf"), float("nan")])
def test_constructor_rejects_invalid_joint_command_filter_cutoff(
    cutoff_hz: float,
) -> None:
    with pytest.raises(ValueError, match="positive finite joint command filter cutoff"):
        _config(joint_command_filter_cutoff_hz=cutoff_hz)


@pytest.mark.parametrize("limit", [0.0, -1.0, float("inf"), float("nan")])
def test_constructor_rejects_invalid_per_joint_velocity_limit(limit: float) -> None:
    with pytest.raises(ValueError, match="positive finite velocity limit"):
        _config(joint_velocity_limits_rad_s={"arm/a": limit})


def test_constructor_rejects_velocity_limit_for_uncontrolled_joint() -> None:
    with pytest.raises(ValueError, match="unknown joints.*arm/missing"):
        _config(joint_velocity_limits_rad_s={"arm/missing": 1.0})


@pytest.mark.parametrize("max_command_tracking_error_deg", [0.0, -1.0, float("inf"), float("nan")])
def test_constructor_rejects_invalid_command_tracking_error(
    mocker: MockerFixture, max_command_tracking_error_deg: float
) -> None:
    with pytest.raises(ValueError, match="positive finite command tracking error"):
        _Task(
            _config(max_command_tracking_error_deg=max_command_tracking_error_deg),
            _solver(mocker),
            _snapshot(),
        )


def test_compute_without_complete_joint_state_skips_pink(mocker: MockerFixture) -> None:
    solver = _solver(mocker)
    task = _Task(_config(), solver, _snapshot())

    output = task.compute(_state(positions={"arm/a": 0.0}))

    assert output is None
    solver.step.assert_not_called()


def test_compute_skips_tick_when_pink_solver_raises(mocker: MockerFixture) -> None:
    solver = _solver(mocker)
    solver.step.side_effect = Exception("QP solver found no solution")
    task = _Task(_config(), solver, _snapshot())

    assert task.compute(_state()) is None


def test_feedback_limit_warnings_are_rate_limited(mocker: MockerFixture) -> None:
    solver = _solver(mocker)
    solver.step.side_effect = PinkJointLimitError(
        joint_name="arm/a",
        value=-1.0011,
        lower=-1.0,
        upper=1.0,
        tolerance=1e-3,
    )
    warning = mocker.patch("dimos.control.tasks.pose_target_ik.logger.warning")
    task = _Task(_config(timeout=0.0), solver, _snapshot())

    assert task.compute(_state(t_now=1.0)) is None
    assert task.compute(_state(t_now=1.1)) is None
    assert task.compute(_state(t_now=2.0)) is None

    assert warning.call_count == 2


def test_compute_rejects_non_finite_pink_result(mocker: MockerFixture) -> None:
    task = _Task(_config(), _solver(mocker, [float("nan"), 0.0]), _snapshot())

    assert task.compute(_state()) is None


def test_stale_snapshot_times_out_without_calling_pink(mocker: MockerFixture) -> None:
    solver = _solver(mocker)
    task = _Task(_config(timeout=0.2), solver, _snapshot(last_update_time=1.0))

    output = task.compute(_state(t_now=1.3))

    assert output is None
    assert task.timed_out
    solver.step.assert_not_called()


def test_current_frame_poses_uses_live_coordinator_seed(mocker: MockerFixture) -> None:
    solver = _solver(mocker)
    task = _Task(_config(), solver, _snapshot())

    poses = task.current_frame_poses(_state(positions={"arm/a": 0.2, "arm/b": 0.3}), ["tool"])

    assert poses == {
        "tool": PoseStamped(
            header=Header(frame_id="base", stamp=Time(sec=0, nanosec=0)),
            pose=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
    }
    seed = solver.frame_poses.call_args.args[0]
    assert seed.name == ["arm/a", "arm/b"]
    np.testing.assert_array_equal(seed.position, [0.2, 0.3])


def test_measured_frame_poses_follow_measured_joints_not_the_target(mocker: MockerFixture) -> None:
    solver = _solver(mocker)
    target = PoseStamped(frame_id="world", position=(9.0, 9.0, 9.0))  # commanded, never reached
    task = _Task(_config(), solver, _snapshot(targets={"tool": target}))

    poses = task.measured_frame_poses(_state(positions={"arm/a": 0.2, "arm/b": 0.3}))

    assert poses == {"tool": PoseStamped(frame_id="base")}  # FK result, not the target
    seed, frames = solver.frame_poses.call_args.args
    assert seed.position == [0.2, 0.3] and list(frames) == ["tool"]


def test_measured_frame_poses_are_empty_without_full_feedback(mocker: MockerFixture) -> None:
    task = _Task(_config(), _solver(mocker), _snapshot())
    assert task.measured_frame_poses(_state(positions={"arm/a": 0.2})) == {}
