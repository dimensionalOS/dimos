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

"""Exercise PTM with the real trajectory, twist task, routing and arbitration."""

from pathlib import Path

import pytest

from dimos.control.coordinator import TaskConfig
from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks.eef_twist_task.eef_twist_task import EEFTwistTask, EEFTwistTaskConfig
from dimos.control.tasks.policy_teleop_task.policy_teleop_task import PolicyTeleopTask
from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.control.tasks.registry import control_task_registry
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionStatus,
    joint_trajectory_task,
)
from dimos.control.tick_loop import TickLoop
from dimos.core.transport_factory import rpc_backend
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator

JOINTS = ["arm/joint1", "arm/joint2"]


def _state(t_now=1.0, dt=0.01):
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions=dict.fromkeys(JOINTS, 0.0)),
        t_now=t_now,
        dt=dt,
    )


@pytest.fixture
async def coordinator(mocker):
    for method in ("start", "serve_module_rpc", "stop"):
        mocker.patch.object(rpc_backend(), method)
    instance = ArmTwistCoordinator(publish_joint_state=False)
    try:
        solver = mocker.Mock(spec=PinkPoseTargetSolver)
        solver.frame_poses.return_value = {"tool": PoseStamped()}
        solver.step.return_value = JointState(name=JOINTS, position=[-0.2, -0.4])
        operator = EEFTwistTask(
            "operator",
            EEFTwistTaskConfig(
                joint_names=tuple(JOINTS),
                target_frames=("tool",),
                command_timeout=0.3,
                robot_model=RobotModelConfig(
                    model=RobotModel.from_file(Path("fake.urdf")), joint_names=JOINTS
                ),
            ),
            solver=solver,
        )
        instance.add_task(operator, task_type="eef_twist")
        policy_cfg = joint_trajectory_task(JOINTS, velocity_limits=dict.fromkeys(JOINTS, 100.0))
        policy = instance._create_task_from_config(policy_cfg)
        instance.add_task(policy, task_type=policy_cfg.type)
        blend_cfg = TaskConfig(
            name="blend",
            type="policy_teleop",
            joint_names=JOINTS,
            priority=20,
            params={"operator_tasks": [operator.name], "policy_tasks": [policy.name]},
        )
        blend = instance._create_task_from_config(blend_cfg)
        instance.add_task(blend, task_type=blend_cfg.type)
        instance._tick_loop = TickLoop(
            tick_rate=100,
            hardware={},
            hardware_lock=instance._hardware_lock,
            tasks=instance._tasks,
            task_lock=instance._task_lock,
            joint_to_hardware={},
        )
        yield instance, operator, policy, blend, solver
    finally:
        instance.stop()


def _start_policy(coordinator):
    result = coordinator.execute_trajectory(
        JointTrajectory(
            joint_names=JOINTS,
            points=[TrajectoryPoint(positions=[0.4, 0.8])],
        )
    )
    assert result.status is TrajectoryExecutionStatus.ACCEPTED


def test_sources_compute_once_and_never_preempt_each_other(coordinator, mocker):
    instance, operator, policy, blend, solver = coordinator
    compute_operator = mocker.spy(operator, "compute")
    compute_policy = mocker.spy(policy, "compute")
    preempt_policy = mocker.spy(policy, "on_preempted")
    _start_policy(instance)
    operator.on_ee_twist_command(TwistStamped(linear=[0.1, 0, 0]), t_now=1)

    results = instance._tick_loop._compute_all_tasks(_state(dt=0.5))
    winners, preemptions = instance._tick_loop._arbitrate(results)
    instance._tick_loop._notify_preemptions(preemptions)

    assert [task.name for task, _, _ in results] == ["blend"]
    assert winners == {
        "arm/joint1": (0.1, ControlMode.SERVO_POSITION, "blend"),
        "arm/joint2": (0.2, ControlMode.SERVO_POSITION, "blend"),
    }
    assert preemptions == {}
    compute_operator.assert_called_once()
    compute_policy.assert_called_once()
    preempt_policy.assert_not_called()
    # One real twist integration on this tick; the solver sees 0.1 m/s * 0.5 s.
    assert solver.step.call_args.args[0]["tool"].position.x == pytest.approx(0.05)
    assert instance.task_invoke("blend", "get_status") == {
        "weights": dict.fromkeys(JOINTS, 0.5),
        "operator_present": JOINTS,
        "policy_present": JOINTS,
    }


def test_policy_keeps_executing_when_operator_joins(coordinator, mocker):
    instance, operator, policy, _, _ = coordinator
    preempt_policy = mocker.spy(policy, "on_preempted")
    trajectory = JointTrajectory(
        joint_names=JOINTS,
        points=[
            TrajectoryPoint(positions=[0, 0]),
            TrajectoryPoint(time_from_start=2, positions=[0.4, 0.8]),
        ],
    )
    instance._tick_loop._compute_all_tasks(_state())
    assert instance.execute_trajectory(trajectory).status is TrajectoryExecutionStatus.ACCEPTED
    instance._tick_loop._compute_all_tasks(_state())
    operator.on_ee_twist_command(TwistStamped(linear=[0.1, 0, 0]), t_now=1.1)

    results = instance._tick_loop._compute_all_tasks(_state(1.1))
    _, preemptions = instance._tick_loop._arbitrate(results)
    instance._tick_loop._notify_preemptions(preemptions)

    assert policy.get_state() is TrajectoryState.EXECUTING
    preempt_policy.assert_not_called()
    assert instance.task_invoke("blend", "get_status")["operator_present"] == JOINTS


def test_streams_still_route_to_registered_sources_and_status_has_no_inputs(coordinator):
    instance, operator, policy, blend, _ = coordinator
    instance._dispatch("ee_twist_command", TwistStamped(linear=[0.1, 0, 0]))
    instance._dispatch("joint_command", JointState(name=JOINTS, position=[0.4, 0.8]))

    assert operator.is_active()
    assert policy.is_active()
    assert instance.get_task(policy.name) is policy
    assert isinstance(blend, PolicyTeleopTask)
    assert control_task_registry.bindings_for("policy_teleop").consumes == ()
    assert control_task_registry.bindings_for("policy_teleop").exposes == frozenset({"get_status"})


def test_estop_resets_sources_and_cached_blend_targets(coordinator):
    instance, operator, policy, blend, solver = coordinator
    _start_policy(instance)
    operator.on_ee_twist_command(TwistStamped(linear=[0.1, 0, 0]), t_now=1)
    instance._tick_loop._compute_all_tasks(_state())

    instance.set_estop(True)
    assert not operator.is_active()
    assert not policy.is_active()
    assert instance.task_invoke("blend", "get_status") == {
        "weights": dict.fromkeys(JOINTS, 1.0),
        "operator_present": [],
        "policy_present": [],
    }
    instance.set_estop(False)
    blend.compute(_state(1.01))
    # The twist source can hold after clear, but it re-anchors to feedback.
    assert solver.step.call_args.args[0]["tool"].position.x == 0
    assert blend.get_status()["policy_present"] == []
