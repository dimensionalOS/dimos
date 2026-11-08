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

"""The walking G1 and its Mid-360 in the simulated world."""

from __future__ import annotations

from functools import partial
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from dimos.core.core import rpc
from dimos.robot.unitree.g1.g1_tf_publisher import base_to_torso, torso_to_mid360
from dimos.robot.unitree.go2.constants import CMD_VEL_TIMEOUT
from dimos.simulation.g1_sim.body import BASE_BODY, G1Chassis, LeggedG1, g1_spec
from dimos.simulation.g1_sim.policy import G1Policy
from dimos.simulation.scenes.procedural import Scene
from dimos.simulation.sensors.mid360.lidar import HOUSING_HALF_EXTENTS, sim_mid360
from dimos.simulation.sim_world.body import BodyShape, Rig, Sensor
from dimos.simulation.sim_world.mujoco_sim import MujocoSim
from dimos.simulation.sim_world.world import Sim, SimWorld, SimWorldConfig

WORLD_FRAME = "odom"
SENSOR_FRAME = "mid360_link"
# the policy steps over a kerb, not a stair
STEP_LIMIT = 0.1
# the pelvis's height standing in the policy's default pose, from the vendor MJCF's home keyframe
STAND_HEIGHT = 0.755
# the Mid-360 on its torso mount is the highest point of the standing G1
CLEARANCE_HEIGHT = 1.3
# shoulder to shoulder, which is what has to fit through a door
BODY_WIDTH = 0.45
# past the G1's own arms, which swing through the lower part of the Mid-360's view
LIO_BLIND_RANGE = 0.5


def _mount() -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """pelvis -> mid360_link as a translation and a rotation, off the real robot's frame tree.

    The sim mounts a sensor on the base body and the real mount is stated against torso_link, so
    the waist chain between them is composed here rather than written out a second time. It is
    taken at the waist's zero, which is where the policy's default pose puts it.
    """
    matrix = base_to_torso(0.0, 0.0, 0.0).to_matrix() @ torso_to_mid360().to_matrix()
    return matrix[:3, 3].copy(), matrix[:3, :3].copy()


MOUNT_XYZ, MOUNT_R = _mount()
# The body whose shell the lens looks out of. The G1's Mid-360 *is* the top of its head: the
# vendor URDF puts mid360_link 0.416 m above torso_link, which is 70 mm **inside** the head mesh,
# and that mesh is a geom on torso_link because the head is welded to the torso. Cast without
# skipping it and the robot scans the inside of its own head: every return lands at 0.02 m,
# under the Mid-360's 0.16 m blind range, so a frame comes back with nothing in it at all.
HOUSING_BODY = "torso_link"
MID360 = Sensor(
    "mid360",
    MOUNT_XYZ,
    MOUNT_R,
    sim_mid360,
    HOUSING_HALF_EXTENTS,
    lio_blind_range=LIO_BLIND_RANGE,
    sees_base=False,
    housing_body=HOUSING_BODY,
)

SHAPE = BodyShape(
    half_width=BODY_WIDTH / 2,
    headroom=CLEARANCE_HEIGHT,
    step=STEP_LIMIT,
    base_height=STAND_HEIGHT,
    top_center=(float(MOUNT_XYZ[0]), float(MOUNT_XYZ[1]), float(MOUNT_XYZ[2])),
    top_rotation=(
        (float(MOUNT_R[0, 0]), float(MOUNT_R[0, 1]), float(MOUNT_R[0, 2])),
        (float(MOUNT_R[1, 0]), float(MOUNT_R[1, 1]), float(MOUNT_R[1, 2])),
        (float(MOUNT_R[2, 0]), float(MOUNT_R[2, 1]), float(MOUNT_R[2, 2])),
    ),
    top_half_extents=HOUSING_HALF_EXTENTS,
    lying_parts=("trunk",),
)

G1_RIG = Rig(
    spec=g1_spec,
    chassis=G1Chassis,
    base_body=BASE_BODY,
    sensors=(MID360,),
    shape=SHAPE,
    command_timeout=CMD_VEL_TIMEOUT,
    world_frame=WORLD_FRAME,
    odom_child_frame=SENSOR_FRAME,
    cloud_frame="sensor",
)


class SimG1WorldConfig(SimWorldConfig):
    policy: Path | None = None


class SimG1World(SimWorld):
    """A generated scene with the walking G1 and its simulated Mid-360, paced to the wall clock."""

    config: SimG1WorldConfig
    rig = G1_RIG

    @rpc
    def start(self) -> None:
        path = self.config.policy
        self._policy = G1Policy.load(LeggedG1.dt) if path is None else G1Policy(path, LeggedG1.dt)
        super().start()

    def _build(self, scene: Scene, seed: int) -> Sim:
        return Sim(scene, seed, self.rig, partial(LeggedG1, policy=self._policy))


class SimG1Many(MujocoSim):
    """A generated scene with several G1s in it, each driven by its own G1Sim module.

    The world and the actors are separate processes' worth of work: this one owns the model and
    steps it, and every robot's walking actor lives in the module that publishes that robot's
    frames. See `MujocoSim` for the lockstep the two keep.
    """

    rig = G1_RIG
