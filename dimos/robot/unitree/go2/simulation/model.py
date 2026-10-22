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

"""Go2's physical device definition and the existing compiled-model dynamics fit."""

import mujoco
import numpy as np

from dimos.robot.unitree.go2.controllers.locomotion.policy import LEGS
from dimos.robot.unitree.go2.simulation.calibration import FITTED_ACTUATOR_TAU, FITTED_PHYSICS
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.composition import Sensor
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.lidar.models.fibonacci import Fibonacci
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.sensors.spec import Mount
from dimos.sim2.spec import Joint, RobotConfig
from dimos.utils.data import LfsPath


def configure_physics(model: mujoco.MjModel, prefix: str) -> None:
    """Apply Andrew's compiled-model fit by robot-local names, not global indices."""
    p = FITTED_PHYSICS
    dofs = [
        model.jnt_dofadr[model.joint(f"{prefix}{leg}_{part}_joint").id]
        for leg in LEGS
        for part in ("hip", "thigh", "calf")
    ]
    model.dof_armature[dofs] = p["armature"]
    model.dof_damping[dofs] = p["damping"]
    model.dof_frictionloss[dofs] = p["frictionloss"]
    trunk = model.body(prefix + "base").id
    model.body_mass[trunk] *= p["trunk_mass_scale"]
    model.body_inertia[trunk] *= p["trunk_inertia_scale"]
    model.body_ipos[trunk, 0] += p["trunk_com_x"]
    for leg in LEGS:
        for part in ("thigh", "calf"):
            body = model.body(f"{prefix}{leg}_{part}").id
            model.body_mass[body] *= p["leg_mass_scale"]
            model.body_inertia[body] *= p["leg_mass_scale"]
        foot = model.geom(prefix + leg).id
        model.geom_friction[foot, :2] = (p["foot_friction"], p["foot_friction_torsional"])


def go2_definition(
    *,
    camera: bool = True,
    lidar: bool = True,
) -> RobotConfig:
    sensors: list[Sensor] = [Imu("imu", Mount("base"))]
    if camera:
        sensors.append(
            Camera(
                "camera",
                Mount("base", xyz=(0.3, 0, 0), rpy=(np.pi / 2, 0, -np.pi / 2)),
                fovy=60,
            )
        )
    if lidar:
        sensors.append(
            Lidar(
                "lidar",
                Mount("base", xyz=(0.2, 0, 0.15)),
                Fibonacci,
                model_kwargs={"ray_count": 15000},
                maximum_world_elevation=0,
            )
        )
    return RobotConfig(
        model=LfsPath("go2_sim/unitree_go2/go2.xml"),
        root_body="base",
        floating=True,
        spawn_height=0.335,
        max_timestep=0.002,
        configure_physics=configure_physics,
        joints=tuple(
            Joint(
                name=name,
                model_name=name,
                actuator=name.removesuffix("_joint"),
                actuator_time_constant=FITTED_ACTUATOR_TAU,
            )
            for name in (f"{leg}_{part}_joint" for leg in LEGS for part in ("hip", "thigh", "calf"))
        ),
        sensors=tuple(sensors),
    )
