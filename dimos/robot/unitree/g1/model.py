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

"""G1 model assets and connection bindings, independent of the external controller.

The velocity surrogate and torque-control models are existing, distinct training
plants. They are kept explicit until their physical differences are reconciled.
"""

from __future__ import annotations

from functools import partial
from pathlib import Path
from typing import TYPE_CHECKING

from dimos.control.components import make_humanoid_joints
from dimos.sim2.spec import ModelAsset, RobotInstanceSpec
from dimos.utils.data import LfsPath

if TYPE_CHECKING:
    from dimos.robot.unitree.g1.connection import G1ConnectionConfig


def describe_simulated_robot(config: G1ConnectionConfig) -> RobotInstanceSpec:
    """Describe one instance; loading assets and starting physics happen in the world."""
    stock = config.control_mode == "stock"
    names = tuple(make_humanoid_joints(config.source))
    velocity_controller = None
    timestep = 0.005
    if stock:
        # ONNX is optional; hardware descriptions never load the simulation policy.
        from dimos.robot.unitree.g1.velocity_policy import G1_PHYSICS_DT, G1VelocityController

        timestep = G1_PHYSICS_DT

        velocity_controller = partial(
            G1VelocityController,
            policy_path=str(LfsPath("mujoco_sim/unitree_g1_policy.onnx")),
        )
    model = ModelAsset(
        path=(
            LfsPath("mujoco_sim/unitree_g1.xml")
            if stock
            else Path(__file__).parent / "assets" / "g1_29dof.xml"
        ),
        meshdir=LfsPath("g1_urdf/meshes"),
        initial_qpos=config.initial_qpos,
        keyframe="home" if stock else None,
        control_imu="imu_in_pelvis" if stock else "control_imu",
    )
    return RobotInstanceSpec(
        robot_id=config.source,
        model=model,
        joint_names=names,
        frame_id=config.frame_id,
        command_kind="velocity" if stock else "motor",
        base_name=f"{config.source}/base",
        velocity_controller=velocity_controller,
        timestep=timestep,
        spawn=config.sim_spawn,
        position=config.sim_position,
        rpy=config.sim_orientation_rpy,
        startup_kp=config.startup_kp,
        startup_kd=config.startup_kd,
        wait_for_first_command=config.wait_for_first_command,
        position_only_joints=names if stock else (),
        command_timeout=config.deadman_timeout_s,
        halt_damping=config.halt_damping,
    )
