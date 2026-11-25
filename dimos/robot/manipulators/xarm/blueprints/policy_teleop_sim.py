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

"""xArm7 joint policy blended with keyboard EEF teleoperation in one tick loop."""

from __future__ import annotations

from collections.abc import AsyncIterator

from pydantic import Field

from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.imitation.policy.lerobot.module import PolicyControlSpec
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.manipulators.common.blueprints import (
    coordinator,
    eef_twist_task,
    policy_teleop_task,
    trajectory_task,
)
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule


class JointPolicyDemoConfig(ModuleConfig):
    joint_names: list[str] = Field(min_length=1)
    amplitude_rad: float = Field(default=0.15, gt=0, allow_inf_nan=False)
    duration_s: float = Field(default=8.0, gt=0, allow_inf_nan=False)


class JointPolicyDemo(Module):
    """Submit one gentle out-and-back joint trajectory, using the real policy RPC.

    This synthetic policy needs no checkpoint. It starts after measured joint
    feedback arrives; operator commands can join at any time during execution.
    """

    config: JointPolicyDemoConfig
    coordinator_joint_state: In[JointState]
    _control: PolicyControlSpec
    _submitted: bool = False

    async def main(self) -> AsyncIterator[None]:
        self._submitted = False
        yield

    async def handle_coordinator_joint_state(self, message: JointState) -> None:
        if self._submitted:
            return
        measured = dict(zip(message.name, message.position, strict=True))
        if not measured.keys() >= set(self.config.joint_names):
            return
        positions = [measured[joint] for joint in self.config.joint_names]
        target = positions.copy()
        target[0] += self.config.amplitude_rad
        trajectory = JointTrajectory(
            joint_names=self.config.joint_names,
            points=[
                TrajectoryPoint(positions=positions),
                TrajectoryPoint(time_from_start=self.config.duration_s / 2, positions=target),
                TrajectoryPoint(time_from_start=self.config.duration_s, positions=positions),
            ],
        )
        result = self._control.execute_trajectory(trajectory)
        self._submitted = result.status is TrajectoryExecutionStatus.ACCEPTED


def _make_policy_teleop_sim(alpha: float) -> Blueprint:
    hardware = make_xarm7_sim_hardware()
    model = make_xarm7_sim_robot_config()
    operator = eef_twist_task(hardware, robot_model=model, target_frame="link_tcp")
    policy = trajectory_task(hardware)
    return autoconnect(
        MujocoSimModule.blueprint(
            **make_xarm7_sim_module_kwargs(XARM7_SIM_PATH),
            enable_color=False,
            enable_depth=False,
        ),
        coordinator(
            hardware=[hardware],
            tasks=[
                operator,
                policy,
                policy_teleop_task(
                    hardware.joints,
                    operator_tasks=[operator.name],
                    policy_tasks=[policy.name],
                    alpha=alpha,
                ),
            ],
            publish_frame_poses=True,
            cls=ArmTwistCoordinator,
            instance_name="ControlCoordinator",
        ),
        KeyboardTeleopModule.blueprint(),
        JointPolicyDemo.blueprint(joint_names=list(model.joint_names)),
    )


policy_teleop_sim = _make_policy_teleop_sim(0.5).global_config(simulation="mujoco")
demo_policy_teleop_human_sim = _make_policy_teleop_sim(0.0).global_config(simulation="mujoco")
demo_policy_teleop_policy_sim = _make_policy_teleop_sim(1.0).global_config(simulation="mujoco")
