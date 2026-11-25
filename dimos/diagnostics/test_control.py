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

"""The cached target used by a later control tick must remain identifiable."""

from pathlib import Path

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.cartesian_ik_task.cartesian_ik_task import (
    CartesianIKTask,
    CartesianIKTaskConfig,
)
from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.diagnostics.analysis import load_bundle
from dimos.diagnostics.model import context_scope, current_context
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel


def test_cached_pose_links_ik_output_and_timeout_reason(trace_runtime, mocker):
    solver = mocker.Mock(spec=PinkPoseTargetSolver)
    solver.step.return_value = JointState(name=["joint"], position=[0.2])
    task = CartesianIKTask(
        "arm",
        CartesianIKTaskConfig(
            joint_names=("joint",),
            target_frames=("tool",),
            robot_model=RobotModelConfig(
                model=RobotModel.from_file(Path("unused.urdf")), joint_names=["joint"]
            ),
        ),
        solver=solver,
    )
    parent = trace_runtime.sample("hand")
    with context_scope(parent):
        task.on_cartesian_command(PoseStamped(), 1.0)
    assert current_context() is None
    state = CoordinatorState(
        joints=JointStateSnapshot(joint_positions={"joint": 0.0}), t_now=1.1, dt=0.01
    )
    output = task.compute(state)
    assert output.trace_context.parents == (parent.sample_id,)
    assert output.positions == [0.2]
    # Input remains fresh while IK cannot produce a command. The fault must
    # identify the solver boundary and retain the input, not blame transport.
    solver.step.return_value = None
    assert task.compute(state) is None
    state.t_now = 2.0
    assert task.compute(state) is None
    trace_runtime.close()
    _, events = load_bundle(trace_runtime.collector.directory)
    assert [e["reason"] for e in events if e["event"] == "control.ik_no_output"] == [
        "solver_returned_none",
        "target_timeout",
    ]
    fault = next(e for e in events if e.get("reason") == "solver_returned_none")
    assert fault["context"]["parents"] == [parent.sample_id]
