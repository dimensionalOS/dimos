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

from contextlib import ExitStack

import pytest

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.transport_factory import rpc_backend
from dimos.imitation.policy.lerobot.module import PolicyControlSpec
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.xarm.blueprints.policy_teleop_sim import (
    JointPolicyDemo,
    demo_policy_teleop_human_sim,
    demo_policy_teleop_policy_sim,
    policy_teleop_sim,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule


@pytest.mark.parametrize(
    "blueprint, alpha",
    [
        (policy_teleop_sim, 0.5),
        (demo_policy_teleop_human_sim, 0.0),
        (demo_policy_teleop_policy_sim, 1.0),
    ],
)
def test_simulation_blends_registered_joint_policy_and_twist_tasks(blueprint, alpha):
    parsed = BlueprintConfigParser(blueprint.disabled_modules(ArmTwistCoordinator)).parse(
        ["--mujocosimmodule.headless=true"], environ={}
    )
    assert parsed.global_config["simulation"] == "mujoco"
    atoms = {atom.module: atom for atom in blueprint.active_blueprints}
    tasks = atoms[ArmTwistCoordinator].kwargs["tasks"]
    assert [task.type for task in tasks] == ["eef_twist", "trajectory", "policy_teleop"]
    assert tasks[2].params["operator_tasks"] == [tasks[0].name]
    assert tasks[2].params["policy_tasks"] == [tasks[1].name]
    assert tasks[2].params["alpha"] == alpha
    assert set(tasks[2].joint_names) >= set(tasks[0].joint_names) | set(tasks[1].joint_names)
    assert set(atoms) == {
        ArmTwistCoordinator,
        MujocoSimModule,
        JointPolicyDemo,
        KeyboardTeleopModule,
    }


@pytest.fixture
async def policy(mocker):
    for method in ("start", "serve_module_rpc", "stop"):
        mocker.patch.object(rpc_backend(), method)
    with ExitStack() as stack:
        module = JointPolicyDemo(
            joint_names=["j1", "j2"], amplitude_rad=0.6, duration_s=12, cycles=2
        )
        stack.callback(module.stop)
        control = mocker.Mock(spec=PolicyControlSpec)
        control.execute_trajectory.return_value = TrajectoryExecutionResult(
            TrajectoryExecutionStatus.ACCEPTED
        )
        mocker.patch.object(module, "_control", control, create=True)
        yield module, control


async def test_demo_policy_submits_joint_trajectory_from_measured_feedback_once(policy):
    module, control = policy
    await module.handle_coordinator_joint_state(JointState(name=["j1"], position=[0.2]))
    control.execute_trajectory.assert_not_called()

    await module.handle_coordinator_joint_state(JointState(name=["j2", "j1"], position=[0.4, 0.2]))
    await module.handle_coordinator_joint_state(JointState(name=["j1", "j2"], position=[0.3, 0.5]))

    control.execute_trajectory.assert_called_once()
    trajectory = control.execute_trajectory.call_args.args[0]
    assert trajectory.joint_names == ["j1", "j2"]
    assert trajectory.duration == 24
    assert trajectory.points[0].positions == [0.2, 0.4]
    # Repeated sweeps stay centered on the first measurement, without drift.
    assert trajectory.sample(3)[0] == pytest.approx([0.8, 0.4])
    assert trajectory.sample(9)[0] == pytest.approx([-0.4, 0.4])
    assert trajectory.sample(12)[0] == pytest.approx([0.2, 0.4])
    assert trajectory.sample(15)[0] == pytest.approx([0.8, 0.4])
    assert trajectory.sample(21)[0] == pytest.approx([-0.4, 0.4])
    assert trajectory.points[-1].positions == [0.2, 0.4]


async def test_demo_policy_retries_submission_after_a_rejected_start(policy):
    module, control = policy
    control.execute_trajectory.side_effect = [
        TrajectoryExecutionResult(TrajectoryExecutionStatus.START_STATE_MISMATCH),
        TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED),
    ]
    await module.handle_coordinator_joint_state(JointState(name=["j1", "j2"], position=[0, 0]))
    await module.handle_coordinator_joint_state(JointState(name=["j1", "j2"], position=[0.2, 0.3]))
    assert control.execute_trajectory.call_count == 2
    assert control.execute_trajectory.call_args.args[0].points[0].positions == [0.2, 0.3]
