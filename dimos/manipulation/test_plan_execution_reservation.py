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

"""Tests for planned-path execution reservation and result propagation."""

from pathlib import Path
import threading
from unittest.mock import MagicMock

from dimos.manipulation._test_manipulation_helpers import make_module
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationState
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus, PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan, IKResult, PlanningResult
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


def _plan(group_id: str = "arm/manipulator") -> GeneratedPlan:
    return GeneratedPlan(
        group_ids=(group_id,),
        status=PlanningStatus.SUCCESS,
        path=[
            JointState(name=["arm/j0"], position=[0.0]),
            JointState(name=["arm/j0"], position=[1.0]),
        ],
        trajectory=JointTrajectory(
            joint_names=["arm/j0"],
            points=[
                TrajectoryPoint(time_from_start=0.0, positions=[0.0], velocities=[0.0]),
                TrajectoryPoint(time_from_start=1.0, positions=[1.0], velocities=[0.0]),
            ],
        ),
    )


def _module_with_current(current: JointState) -> ManipulationModule:
    module = make_module()
    robot_config = RobotModelConfig(
        name="arm",
        model_path=Path("/path/to/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["j0"],
        base_link="base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("j0",),
                base_link="base",
                tip_link="tool",
            )
        ],
        max_velocity=1.0,
        max_acceleration=1.0,
        coordinator_task_name="traj_arm",
    )
    module._robots = {"arm": ("robot_id", robot_config, MagicMock())}
    module._world_monitor = MagicMock()
    module._world_monitor.planning_groups = PlanningGroupRegistry([robot_config])
    module._world_monitor.current_global_joint_state.return_value = current
    module._world_monitor.is_state_stale.return_value = False
    client = MagicMock()
    client.execute_task.return_value = True
    client.cancel_task.return_value = False
    module._initialize_execution(client)
    return module


def _install_current_global_state(module: ManipulationModule, current: JointState) -> None:
    robot_config = RobotModelConfig(
        name="arm",
        model_path=Path("/path/to/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["j0"],
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator", joint_names=("j0",), base_link="base", tip_link="tool"
            )
        ],
        coordinator_task_name="traj_arm",
    )
    module._robots = {"arm": ("robot_id", robot_config, MagicMock())}
    module._world_monitor = MagicMock()
    module._world_monitor.planning_groups = PlanningGroupRegistry([robot_config])
    module._world_monitor.current_global_joint_state.return_value = current
    client = MagicMock()
    client.execute_task.return_value = True
    client.cancel_task.return_value = False
    module._initialize_execution(client)


def test_plan_to_pose_targets_propagates_failed_plan_as_false():
    module = make_module()
    robot_config = RobotModelConfig(
        name="test_arm",
        model_path=Path("/path/to/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["joint1", "joint2", "joint3"],
        base_link="link_base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("joint1", "joint2", "joint3"),
                base_link="link_base",
                tip_link="link_tcp",
            )
        ],
        max_velocity=1.0,
        max_acceleration=2.0,
        coordinator_task_name="traj_arm",
    )
    registry = PlanningGroupRegistry([robot_config])
    module._robots = {"test_arm": ("robot_id", robot_config, MagicMock())}
    module._world_monitor = MagicMock()
    module._world_monitor.world = MagicMock()
    module._world_monitor.planning_groups = registry
    module._world_monitor.current_global_joint_state.return_value = JointState(
        name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
        position=[0.0, 0.0, 0.0],
    )
    module._world_monitor.get_current_joint_state.return_value = JointState(
        name=robot_config.joint_names,
        position=[0.0, 0.0, 0.0],
    )
    module._kinematics = MagicMock()
    module._kinematics.solve_pose_targets.return_value = IKResult(
        status=IKStatus.SUCCESS,
        joint_state=JointState(
            name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
            position=[0.1, 0.2, 0.3],
        ),
    )
    module._planner = MagicMock()
    module._planner.plan_selected_joint_path.return_value = PlanningResult(
        status=PlanningStatus.NO_SOLUTION
    )
    pose = Pose(position=Vector3(x=0.45, y=0.0, z=0.25), orientation=Quaternion())

    assert module.plan_to_pose_targets({"test_arm/manipulator": pose}) is False
    assert module._state == ManipulationState.FAULT


def test_execute_plan_replaces_tracked_execution_and_can_then_cancel() -> None:
    module = make_module()
    plan = _plan()
    module._last_plan = plan
    module._state = ManipulationState.COMPLETED
    _install_current_global_state(module, JointState(name=["arm/j0"], position=[0.0]))

    assert module.execute_plan() is True
    assert module.execute_plan() is True
    assert module.cancel() is True
    assert [item[0] for item in module._coordinator_client.mock_calls] == [
        "execute_task",
        "cancel_task",
        "execute_task",
        "cancel_task",
    ]
    assert module._state is ManipulationState.IDLE


def test_execute_and_execute_plan_race_without_consuming_latest_plan():
    module = make_module()
    plan = _plan()
    module._last_plan = plan
    module._state = ManipulationState.COMPLETED
    _install_current_global_state(module, JointState(name=["arm/j0"], position=[0.0]))
    dispatch_started = threading.Event()
    allow_dispatch_return = threading.Event()
    results: list[bool] = []

    def dispatch(*_args, **_kwargs):
        dispatch_started.set()
        assert allow_dispatch_return.wait(timeout=1.0)
        return True

    module._coordinator_client.execute_task.side_effect = dispatch
    legacy = threading.Thread(target=lambda: results.append(module.execute()))
    legacy.start()
    assert dispatch_started.wait(timeout=1.0)

    current_result = module.execute_plan()

    allow_dispatch_return.set()
    legacy.join(timeout=1.0)

    assert not legacy.is_alive()
    assert results == [True]
    assert current_result is False
    assert module._coordinator_client.execute_task.call_count == 1


def test_execute_started_during_cancel_cannot_dispatch_after_cancel():
    module = _module_with_current(JointState(name=["arm/j0"], position=[0.0]))
    module._last_plan = _plan()
    module._state = ManipulationState.COMPLETED
    assert module.execute_plan()
    cancel_started = threading.Event()
    execute_finished = threading.Event()
    release_cancel = threading.Event()
    cancel_result: list[bool] = []
    execute_result: list[bool] = []

    def cancel_task(task_name: str) -> bool:
        assert task_name == "traj_arm"
        cancel_started.set()
        assert release_cancel.wait(timeout=1.0)
        return False

    module._coordinator_client.cancel_task.side_effect = cancel_task
    module._coordinator_client.reset_mock()
    cancelling = threading.Thread(target=lambda: cancel_result.append(module.cancel()))
    cancelling.start()
    assert cancel_started.wait(timeout=1.0)

    def run_execute() -> None:
        execute_result.append(module.execute_plan())
        execute_finished.set()

    executing = threading.Thread(target=run_execute)
    executing.start()
    assert execute_finished.wait(timeout=1.0)
    assert execute_result == [False]
    module._coordinator_client.execute_task.assert_not_called()
    release_cancel.set()
    cancelling.join(timeout=1.0)
    executing.join(timeout=1.0)

    assert cancel_result == [True]
    module._coordinator_client.cancel_task.assert_called_once_with("traj_arm")


def test_execute_plan_accepts_a_direct_plan_without_reserving_stored_plan():
    module = make_module()
    direct_plan = _plan()
    module._last_plan = _plan("other/manipulator")
    _install_current_global_state(module, JointState(name=["arm/j0"], position=[0.0]))

    assert module.execute_plan(plan=direct_plan) is True
    module._coordinator_client.execute_task.assert_called_once()


def test_execute_plan_uses_current_global_state_without_consuming_cached_plan():
    module = _module_with_current(JointState(name=["arm/j0"], position=[0.0]))
    cached_plan = _plan("cached/manipulator")
    module._last_plan = cached_plan
    assert module.execute_plan(plan=_plan()) is True
    module._coordinator_client.execute_task.assert_called_once()
    assert module._last_plan is cached_plan


def test_execute_plan_rejects_missing_direct_plan_start_before_dispatch_without_reserving_cached_plan():
    module = _module_with_current(JointState(name=[], position=[]))
    cached_plan = _plan("cached/manipulator")
    module._last_plan = cached_plan

    assert module.execute_plan(plan=_plan()) is False
    module._coordinator_client.execute_task.assert_not_called()
    assert module._last_plan is cached_plan


def test_execute_plan_rejects_malformed_direct_plan_before_dispatch_without_reserving_cached_plan():
    module = _module_with_current(JointState(name=["arm/j0"], position=[0.0]))
    cached_plan = _plan("cached/manipulator")
    direct_plan = _plan()
    direct_plan.trajectory.points[0].positions = []
    module._last_plan = cached_plan

    assert module.execute_plan(plan=direct_plan) is False
    module._coordinator_client.execute_task.assert_not_called()
    assert module._last_plan is cached_plan


def test_execute_plan_pre_dispatch_exception_restores_previous_state_without_faulting(mocker):
    module = _module_with_current(JointState(name=["arm/j0"], position=[0.0]))
    module._last_plan = _plan()
    module._state = ManipulationState.COMPLETED
    mocker.patch.object(
        module._execution_manager,
        "_prepare_plan",
        side_effect=RuntimeError("split exploded"),
    )

    assert module.execute_plan() is False

    assert module._state == ManipulationState.COMPLETED
    assert module._error_message == "Failed to dispatch generated plan: split exploded"
    module._coordinator_client.execute_task.assert_not_called()


def test_execute_plan_dispatch_exception_with_safe_rollback_restores_prior_state():
    module = _module_with_current(JointState(name=["arm/j0"], position=[0.0]))
    module._last_plan = _plan()
    module._coordinator_client.execute_task.side_effect = RuntimeError("rpc exploded")

    assert module.execute_plan() is False

    assert module._state == ManipulationState.IDLE
    assert module._error_message.startswith("Coordinator task 'traj_arm' dispatch is uncertain")


def test_execute_plan_unresolved_rollback_projects_uncertainty_to_fault():
    module = _module_with_current(JointState(name=["arm/j0"], position=[0.0]))
    module._last_plan = _plan()
    module._coordinator_client.execute_task.side_effect = RuntimeError("rpc exploded")
    module._coordinator_client.cancel_task.return_value = None

    assert module.execute_plan() is False

    assert module._state == ManipulationState.FAULT
    assert "safety is uncertain" in module._error_message
