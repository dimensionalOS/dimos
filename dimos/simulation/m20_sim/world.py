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

"""The walking M20 and its two Airys in the simulated world."""

from __future__ import annotations

from functools import partial
from itertools import pairwise
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.robot.deeprobotics.m20.constants import (
    BODY_WIDTH_M,
    CMD_VEL_TIMEOUT_S,
    FRONT_LIDAR_XYZ_M,
    REAR_LIDAR_XYZ_M,
    WORLD_FRAME,
)
from dimos.simulation.m20_sim.body import LeggedM20, m20_spec
from dimos.simulation.m20_sim.policy import M20Policy
from dimos.simulation.scenes.procedural import Scene
from dimos.simulation.sensors.airy.lidar import HOUSING_HALF_EXTENTS, sim_airy
from dimos.simulation.sim_world.body import COLLISION_GROUP, BodyShape, Rig, Sensor
from dimos.simulation.sim_world.world import Sim, SimWorld, SimWorldConfig

# the walking policy climbs 2.5 cm and stalls at 3 cm
STEP_LIMIT = 0.02
STAND_HEIGHT = 0.566
# the trunk top, at its highest while walking
CLEARANCE_HEIGHT = 0.66
TRUNK_HALF_EXTENTS = (0.41, 0.121, 0.087)
VENDOR_COLLISION_GROUP = 1
# the driver's minimum distance
LIO_BLIND_RANGE = 0.2
# the vendor LIO's skip_num
LIO_KEEP_EVERY = 5
COLLISION_RGBA = (0.3, 0.3, 0.3, 1.0)
# in the lidar's frame, fitted to where the real lidars get no returns
GUARD_CENTERLINE = (
    (0.041, 0.0, 0.126),
    (0.038, 0.018, 0.125),
    (0.036, 0.043, 0.116),
    (0.023, 0.064, 0.104),
    (0.0, 0.083, 0.081),
    (-0.038, 0.082, 0.043),
)
GUARD_RADII = (0.015, 0.02, 0.02, 0.022, 0.028)
SHELL_CENTERLINE = ((0.09, 0.0, 0.041), (0.078, 0.045, 0.041))
SHELL_RADIUS = 0.03


FRONT = Sensor(
    "front_lidar",
    np.array(FRONT_LIDAR_XYZ_M),
    Rotation.from_euler("ZY", (180, -90), degrees=True).as_matrix(),
    sim_airy,
    HOUSING_HALF_EXTENTS,
    lio_blind_range=LIO_BLIND_RANGE,
    lio_keep_every=LIO_KEEP_EVERY,
    sees_base=False,
)
REAR = Sensor(
    "rear_lidar",
    np.array(REAR_LIDAR_XYZ_M),
    Rotation.from_euler("y", -90, degrees=True).as_matrix(),
    sim_airy,
    HOUSING_HALF_EXTENTS,
    lio_blind_range=LIO_BLIND_RANGE,
    lio_keep_every=LIO_KEEP_EVERY,
    sees_base=False,
)

SHAPE = BodyShape(
    half_width=BODY_WIDTH_M / 2,
    headroom=CLEARANCE_HEIGHT,
    step=STEP_LIMIT,
    base_height=STAND_HEIGHT,
    top_center=(0.0, 0.0, 0.0),
    top_rotation=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    top_half_extents=TRUNK_HALF_EXTENTS,
    lying_parts=("trunk",),
)


def _add_guard(spec: mujoco.MjSpec, sensor: Sensor) -> None:
    """The guard and shell around a lidar, seen by the lidars but with no mass or contacts."""
    body = spec.body("base_link").add_body()
    body.name = f"{sensor.name}_guard"
    body.pos = sensor.mount_xyz
    body.quat = Rotation.from_matrix(sensor.mount_r).as_quat(scalar_first=True)
    capsules = [
        *zip(pairwise(GUARD_CENTERLINE), GUARD_RADII, strict=True),
        (SHELL_CENTERLINE, SHELL_RADIUS),
    ]
    for (start, end), radius in capsules:
        for side in (1.0, -1.0):
            geom = body.add_geom()
            geom.type = mujoco.mjtGeom.mjGEOM_CAPSULE
            geom.fromto = [start[0], side * start[1], start[2], end[0], side * end[1], end[2]]
            geom.size = [radius, 0.0, 0.0]
            geom.group = COLLISION_GROUP
            geom.contype = 0
            geom.conaffinity = 0
            geom.mass = 0.0


def _spec() -> mujoco.MjSpec:
    """The vendor M20 with its collision shapes opaque, in the sim's hidden collision group."""
    spec = m20_spec()
    spec.material("collision_material").rgba = COLLISION_RGBA
    for geom in spec.geoms:
        if geom.group == VENDOR_COLLISION_GROUP:
            geom.group = COLLISION_GROUP
    _add_guard(spec, FRONT)
    _add_guard(spec, REAR)
    return spec


M20_RIG = Rig(
    spec=_spec,
    base_body="base_link",
    sensors=(FRONT, REAR),
    shape=SHAPE,
    command_timeout=CMD_VEL_TIMEOUT_S,
    world_frame=WORLD_FRAME,
    odom_child_frame="base_link",
    cloud_frame="base",
    lidar_group=COLLISION_GROUP,
)


class SimM20WorldConfig(SimWorldConfig):
    policy: Path | None = None


class SimM20World(SimWorld):
    """A generated scene with the walking M20 and its two simulated Airys, paced to the wall clock."""

    config: SimM20WorldConfig
    rig = M20_RIG

    @rpc
    def start(self) -> None:
        path = self.config.policy
        self._policy = M20Policy.load() if path is None else M20Policy(path)
        super().start()

    def _build(self, scene: Scene, seed: int) -> Sim:
        return Sim(scene, seed, self.rig, partial(LeggedM20, policy=self._policy))
