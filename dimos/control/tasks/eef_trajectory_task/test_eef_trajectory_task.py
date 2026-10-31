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

"""Deterministic EEF playback tests at the coordinator/IK boundary."""

from pathlib import Path

import attrs
import pytest

from dimos.control._control_test_helpers import execute_with_state
from dimos.control.coordinator import ControlCoordinator
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.eef_trajectory_task.eef_trajectory_task import (
    EefTrajectoryTask,
    EefTrajectoryTaskConfig,
)
from dimos.control.tasks.eef_trajectory_task.trajectory import EefTrajectory, EefTrajectoryPoint
from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
    TrajectoryCancellationStatus,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
from dimos.robot.assets.model import RobotModel


def pose(x=0.0):
    return PoseStamped(ts=0.0, frame_id="world", position=[x, 0, 0])


def trajectory():
    return EefTrajectory(
        [
            EefTrajectoryPoint(0, {"left": pose(), "right": pose()}, {"gripper": 0.0}),
            EefTrajectoryPoint(1, {"left": pose(0.1), "right": pose(0.1)}, {"gripper": 1.0}),
        ]
    )


def state(t=1.0, opening=0.0):
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions={"joint": 0.0, "gripper": opening}),
        t_now=t,
        dt=0.01,
    )


@pytest.fixture
def setup_task(mocker):
    solver = mocker.Mock(spec=PinkPoseTargetSolver)
    solver.step.return_value = JointState(name=["joint"], position=[0.01])
    solver.frame_poses.return_value = {"left": pose(), "right": pose()}
    config = EefTrajectoryTaskConfig(
        joint_names=("joint",),
        robot_model=RobotModelConfig(
            model=RobotModel.from_file(Path("fake.urdf")), joint_names=["joint"]
        ),
        target_frames=("left", "right"),
        gripper_joint_names=("gripper",),
    )
    return EefTrajectoryTask(config, solver=solver), solver


def test_eef_tick_samples_both_frames_once_and_rate_bounds_gripper(setup_task, mocker):
    task, solver = setup_task
    assert (
        execute_with_state(task, trajectory(), state().joints.joint_positions).status
        == TrajectoryExecutionStatus.ACCEPTED
    )
    sample = mocker.spy(EefTrajectory, "sample")
    assert task.compute(state(t=10)) is not None
    output = task.compute(state(t=10.5))
    assert sample.call_count == 2
    assert output is not None
    targets = solver.step.call_args.args[0]
    assert {frame: target.x for frame, target in targets.items()} == pytest.approx(
        {"left": 0.05, "right": 0.05}
    )
    assert output.positions == pytest.approx([0.01, 0.01])
    assert task.get_status(99999).time_elapsed == 0.5
    assert task.claim().joints == frozenset({"joint", "gripper"})


def test_eef_completion_requires_measured_pose_and_gripper_not_only_elapsed(setup_task):
    task, solver = setup_task
    execute_with_state(task, trajectory(), state().joints.joint_positions)
    task.compute(state(t=10))
    assert task.compute(state(t=11)) is not None
    assert task.get_state() == TrajectoryState.EXECUTING
    solver.frame_poses.return_value = {"left": pose(0.1), "right": pose(0.1)}
    output = task.compute(state(t=11.1, opening=1))
    assert output is not None
    assert output.positions == [0.0, 1.0]
    assert task.get_state() == TrajectoryState.COMPLETED
    assert task.compute(state(t=12)) is None


def test_eef_unreached_final_target_faults_after_settling_timeout(setup_task):
    task, _ = setup_task
    execute_with_state(task, trajectory(), state().joints.joint_positions)
    task.compute(state(t=10))
    assert task.compute(state(t=13.1)) is None
    assert task.get_state() == TrajectoryState.FAULT
    assert "completion" in task.get_status().error


@pytest.mark.parametrize("stop", ["cancel", "estop", "preempt", "reset"])
def test_eef_stop_clears_chunk_and_never_resumes_old_motion(setup_task, stop):
    task, solver = setup_task
    execute_with_state(task, trajectory(), state().joints.joint_positions)
    task.compute(state())
    if stop == "cancel":
        assert task.cancel().status == TrajectoryCancellationStatus.CANCELLED
    elif stop == "estop":
        task.set_estop(True)
        assert (
            execute_with_state(task, trajectory(), state().joints.joint_positions).status
            == TrajectoryExecutionStatus.ESTOPPED
        )
        task.set_estop(False)
    elif stop == "preempt":
        task.on_preempted("teleop", frozenset({"gripper"}))
    else:
        assert task.reset_runtime_state() is True
    assert not task.is_active()
    assert task.compute(state(t=2)) is None
    assert task.cancel().status == TrajectoryCancellationStatus.ALREADY_STOPPED
    assert solver.reset.call_count >= 2


def test_eef_rejects_replacement_and_mismatched_or_missing_start(setup_task):
    task, solver = setup_task
    assert (
        execute_with_state(task, trajectory(), {}).status
        == TrajectoryExecutionStatus.START_STATE_UNAVAILABLE
    )
    solver.frame_poses.return_value = {"left": pose(1), "right": pose(1)}
    assert (
        execute_with_state(task, trajectory(), state().joints.joint_positions).status
        == TrajectoryExecutionStatus.START_STATE_MISMATCH
    )
    solver.frame_poses.return_value = {"left": pose(), "right": pose()}
    assert (
        execute_with_state(task, trajectory(), state().joints.joint_positions).status
        == TrajectoryExecutionStatus.ACCEPTED
    )
    assert (
        execute_with_state(task, trajectory(), state().joints.joint_positions).status
        == TrajectoryExecutionStatus.ALREADY_EXECUTING
    )


@pytest.mark.parametrize(
    "failure",
    [RuntimeError("QP failure"), None, JointState(name=["joint"], position=[float("nan")])],
)
def test_eef_ik_failure_faults_whole_chunk(setup_task, failure):
    task, solver = setup_task
    execute_with_state(task, trajectory(), state().joints.joint_positions)
    if isinstance(failure, Exception):
        solver.step.side_effect = failure
    else:
        solver.step.return_value = failure
    assert task.compute(state()) is None
    assert task.get_state() == TrajectoryState.FAULT
    assert not task.is_active()


def test_eef_tracking_bound_aborts_without_emitting_joint_commands(setup_task):
    task, solver = setup_task
    execute_with_state(task, trajectory(), state().joints.joint_positions)
    solver.frame_poses.return_value = {"left": pose(1), "right": pose()}
    assert task.compute(state()) is None
    assert task.get_state() == TrajectoryState.FAULT
    solver.step.assert_not_called()


@pytest.fixture
async def coordinator(mocker):
    mocker.patch("dimos.control.coordinator.TickLoop")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    instance = ControlCoordinator(publish_joint_state=False)
    try:
        yield instance
    finally:
        instance.stop()


def test_coordinator_eef_rpc_uses_tick_feedback_and_removal_clears_route(
    coordinator, setup_task, mocker
):
    task, _ = setup_task
    with pytest.raises(KeyError, match="eef_trajectory"):
        coordinator.task_invoke("eef_trajectory", "execute", {"trajectory": trajectory()})
    coordinator.add_task(task, task_type="eef_trajectory")
    coordinator._set_latest_state(state())
    assert (
        coordinator.task_invoke("eef_trajectory", "execute", {"trajectory": trajectory()}).status
        == TrajectoryExecutionStatus.ACCEPTED
    )
    assert (
        coordinator.task_invoke("eef_trajectory", "cancel").status
        == TrajectoryCancellationStatus.CANCELLED
    )
    assert coordinator.task_invoke("eef_trajectory", "get_status").state == TrajectoryState.ABORTED
    coordinator.remove_task("eef_trajectory")
    with pytest.raises(KeyError, match="eef_trajectory"):
        coordinator.task_invoke("eef_trajectory", "cancel")


def test_eef_rechecks_anchor_at_first_tick_before_emitting_commands(setup_task):
    task, solver = setup_task
    assert (
        execute_with_state(task, trajectory(), state().joints.joint_positions).status
        == TrajectoryExecutionStatus.ACCEPTED
    )
    solver.frame_poses.return_value = {"left": pose(0.08), "right": pose()}
    assert task.compute(state()) is None
    assert task.get_state() == TrajectoryState.FAULT
    assert "anchor changed" in task.get_status().error
    solver.step.assert_not_called()


def test_model_root_targets_are_explicitly_transformed_into_world(setup_task):
    old_task, solver = setup_task
    config = attrs.evolve(
        old_task._config,
        robot_model=old_task._config.robot_model.model_copy(
            update={"base_pose": PoseStamped(frame_id="world", position=[1, 2, 3])}
        ),
    )
    task = EefTrajectoryTask(config, solver=solver)
    root_pose = PoseStamped(frame_id="base_link")
    chunk = EefTrajectory(
        [
            EefTrajectoryPoint(0, {"left": root_pose, "right": root_pose}, {"gripper": 0.0}),
            EefTrajectoryPoint(1, {"left": root_pose, "right": root_pose}, {"gripper": 0.0}),
        ]
    )
    solver.frame_poses.return_value = {
        frame: PoseStamped(frame_id="base_link", position=[1, 2, 3]) for frame in ("left", "right")
    }
    assert (
        execute_with_state(task, chunk, state().joints.joint_positions).status
        == TrajectoryExecutionStatus.ACCEPTED
    )
    assert task.compute(state()) is not None
    targets = solver.step.call_args.args[0]
    assert {frame: [target.x, target.y, target.z] for frame, target in targets.items()} == {
        "left": [1, 2, 3],
        "right": [1, 2, 3],
    }
    assert all(target.frame_id == "world" for target in targets.values())
    assert root_pose.frame_id == "base_link" and root_pose.x == 0


def test_eef_tick_path_uses_real_bounded_pink_solver_and_measured_completion(tmp_path):
    model_path = tmp_path / "slider.urdf"
    model_path.write_text("""<robot name="slider">
      <link name="base_link"/><link name="tool"/>
      <joint name="joint" type="prismatic">
        <parent link="base_link"/><child link="tool"/>
        <axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/>
      </joint>
    </robot>""")
    task = EefTrajectoryTask(
        EefTrajectoryTaskConfig(
            joint_names=("joint",),
            robot_model=RobotModelConfig(
                model=RobotModel.from_file(model_path).with_default_joint_acceleration_limit(1.0),
                joint_names=["joint"],
            ),
            target_frames=("tool",),
            max_joint_velocity_rad_s=0.5,
            joint_command_filter_cutoff_hz=None,
        )
    )
    chunk = EefTrajectory(
        [
            EefTrajectoryPoint(0, {"tool": pose()}),
            EefTrajectoryPoint(1, {"tool": pose(0.05)}),
        ]
    )
    assert (
        execute_with_state(task, chunk, {"joint": 0.0}).status == TrajectoryExecutionStatus.ACCEPTED
    )
    position = 0.0
    outputs = []
    for tick in range(301):
        tick_state = CoordinatorState(
            joints=JointStateSnapshot(joint_positions={"joint": position}),
            t_now=tick * 0.01,
            dt=0.01,
        )
        output = task.compute(tick_state)
        if output is not None:
            next_position = output.positions[0]
            assert abs(next_position - position) <= 0.005 + 1e-9
            position = next_position
            outputs.append(position)
        if not task.is_active():
            break
    assert task.get_state() == TrajectoryState.COMPLETED, task.get_status().error
    assert position == pytest.approx(0.05, abs=0.005)
    assert len(outputs) >= 100


def test_eef_new_episode_restarts_clock_at_its_first_tick(setup_task):
    task, _ = setup_task
    assert (
        execute_with_state(task, trajectory(), state().joints.joint_positions).status
        == TrajectoryExecutionStatus.ACCEPTED
    )
    assert task.compute(state(t=10.0)) is not None
    assert task.compute(state(t=10.5)) is not None
    assert task.get_status().time_elapsed == 0.5
    task.cancel()
    assert (
        execute_with_state(task, trajectory(), state().joints.joint_positions).status
        == TrajectoryExecutionStatus.ACCEPTED
    )
    pending = task.get_status(1000.0)
    assert (pending.progress, pending.time_elapsed, pending.time_remaining) == (0.0, 0.0, 1.0)
    assert task.compute(state(t=100.0)) is not None
    assert task.get_status().time_elapsed == 0.0


@pytest.mark.parametrize(
    "keepalive, expected",
    [
        (False, TrajectoryState.ABORTED),
        (True, TrajectoryState.EXECUTING),
    ],
)
def test_eef_deadman_aborts_when_caller_stops_refreshing(setup_task, keepalive, expected, mocker):
    task, _ = setup_task
    assert (
        execute_with_state(
            task, trajectory(), state().joints.joint_positions, deadman_timeout=0.5
        ).status
        == TrajectoryExecutionStatus.ACCEPTED
    )
    receipt_clock = mocker.patch("dimos.control.tasks.trajectory_clock.time.perf_counter")
    for tick in range(101):
        t_now = tick * 0.01
        if keepalive:
            receipt_clock.return_value = t_now
            task.keepalive()
        task.compute(state(t=t_now))
    assert task.get_state() == expected


def test_eef_deadman_clears_solver_and_does_not_leak_into_next_episode(setup_task):
    task, solver = setup_task
    assert (
        execute_with_state(
            task, trajectory(), state().joints.joint_positions, deadman_timeout=0.5
        ).status
        == TrajectoryExecutionStatus.ACCEPTED
    )
    assert task.compute(state(t=10.0)) is not None
    assert task.compute(state(t=10.5)) is not None
    assert task.compute(state(t=10.5001)) is None
    assert task.get_state() == TrajectoryState.ABORTED
    assert "deadman expired" in task.get_status().error
    assert task.compute(state(t=10.6)) is None
    assert solver.reset.call_count >= 2
    assert (
        execute_with_state(task, trajectory(), state().joints.joint_positions).status
        == TrajectoryExecutionStatus.ACCEPTED
    )
    assert task.compute(state(t=20.0)) is not None
    assert task.compute(state(t=21.0)) is not None
    assert task.get_state() == TrajectoryState.EXECUTING


@pytest.mark.parametrize("timeout", [0.0, -1.0, float("nan"), float("inf")])
def test_eef_rejects_invalid_deadman_before_start(setup_task, timeout):
    task, _ = setup_task
    assert (
        execute_with_state(
            task, trajectory(), state().joints.joint_positions, deadman_timeout=timeout
        ).status
        == TrajectoryExecutionStatus.INVALID_TRAJECTORY
    )
    assert not task.is_active()


def test_coordinator_passes_eef_deadman_and_exposes_keepalive(coordinator, setup_task, mocker):
    task, _ = setup_task
    coordinator.add_task(task, task_type="eef_trajectory")
    coordinator._set_latest_state(state())
    assert (
        coordinator.task_invoke(
            "eef_trajectory", "execute", {"trajectory": trajectory(), "deadman_timeout": 0.5}
        ).status
        == TrajectoryExecutionStatus.ACCEPTED
    )
    assert task.compute(state(t=10.0)) is not None
    mocker.patch("dimos.control.tasks.trajectory_clock.time.perf_counter", return_value=10.4)
    with pytest.raises(TypeError):
        coordinator.task_invoke("eef_trajectory", "keepalive", {"t_now": 1_700_000_000.0})
    status = coordinator.task_invoke("eef_trajectory", "keepalive")
    assert status.state == TrajectoryState.EXECUTING
    assert task.compute(state(t=10.6)) is not None
    assert task.compute(state(t=10.9001)) is None
    assert task.get_state() == TrajectoryState.ABORTED


def test_coordinator_passes_joint_deadman_and_exposes_keepalive(coordinator, mocker):
    task = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=["joint"]))
    coordinator.add_task(task, task_type="trajectory")
    coordinator._set_latest_state(state())
    chunk = JointTrajectory(
        joint_names=["joint"],
        points=[
            TrajectoryPoint(positions=[0.0]),
            TrajectoryPoint(positions=[1.0], time_from_start=2.0),
        ],
    )
    assert (
        coordinator.task_invoke(
            "joint_trajectory", "execute", {"trajectory": chunk, "deadman_timeout": 0.5}
        ).status
        == TrajectoryExecutionStatus.ACCEPTED
    )
    assert task.compute(state(t=10.0)) is not None
    mocker.patch("dimos.control.tasks.trajectory_clock.time.perf_counter", return_value=10.4)
    with pytest.raises(TypeError):
        coordinator.task_invoke("joint_trajectory", "keepalive", {"t_now": 1_700_000_000.0})
    status = coordinator.task_invoke("joint_trajectory", "keepalive")
    assert status.state == TrajectoryState.EXECUTING
    assert task.compute(state(t=10.6)) is not None
    assert task.compute(state(t=10.9001)) is None
    assert task.get_state() == TrajectoryState.ABORTED
