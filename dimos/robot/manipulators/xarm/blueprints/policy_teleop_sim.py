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

"""Minimal xArm7 simulation driven exclusively by blended TCP velocities."""

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.manipulators.common.blueprints import coordinator, eef_twist_task
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.teleop.policy.adapters import TcpPoseAdapter
from dimos.teleop.policy.demo_sources import SyntheticVelocitySources, bottle_cap_takeover
from dimos.teleop.policy.module import PolicyTeleopModule
from dimos.teleop.policy.takeover import TakeoverFn, human_only, policy_only


def _make_policy_teleop_sim(takeover_fn: TakeoverFn) -> Blueprint:
    hardware = make_xarm7_sim_hardware()
    model = make_xarm7_sim_robot_config()
    return autoconnect(
        MujocoSimModule.blueprint(
            **make_xarm7_sim_module_kwargs(XARM7_SIM_PATH),
            enable_color=False,
            enable_depth=False,
        ),
        coordinator(
            hardware=[hardware],
            # The only motion task: a command timeout cannot resume a parallel
            # trajectory. Its own deadman also stops motion if the blender dies.
            tasks=[eef_twist_task(hardware, robot_model=model, target_frame="link_tcp")],
            publish_frame_poses=True,
            cls=ArmTwistCoordinator,
            instance_name="ControlCoordinator",
        ),
        TcpPoseAdapter.blueprint(),
        SyntheticVelocitySources.blueprint(),
        PolicyTeleopModule.blueprint(takeover_fn=takeover_fn),
    )


policy_teleop_sim = _make_policy_teleop_sim(bottle_cap_takeover).global_config(simulation="mujoco")
demo_policy_teleop_human_sim = _make_policy_teleop_sim(human_only).global_config(
    simulation="mujoco"
)
demo_policy_teleop_policy_sim = _make_policy_teleop_sim(policy_only).global_config(
    simulation="mujoco"
)
