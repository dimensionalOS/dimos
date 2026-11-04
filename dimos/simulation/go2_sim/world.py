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


"""The legged Go2 and its Mid-360 in the simulated world."""

from __future__ import annotations

from functools import partial
import math
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from dimos.core.core import rpc
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.protocol.tf.static_tf_publisher import (
    FrameSpec,
    StaticTfPublisher,
    frames_to_edge_transforms,
)
from dimos.robot.unitree.go2.config import CMD_VEL_TIMEOUT, ROBOT_WIDTH
from dimos.simulation.go2_legged.policy import load_policy
from dimos.simulation.go2_legged.robot import LeggedGo2, apply_fitted_physics, go2_spec
from dimos.simulation.scenes.procedural import Scene
from dimos.simulation.sensors.lidar import OcclusionMap, Raycaster, SimLidar
from dimos.simulation.sensors.mid360.lidar import HOUSING_HALF_EXTENTS, sim_mid360
from dimos.simulation.sim_world.body import BodyShape, Rig, Sensor
from dimos.simulation.sim_world.world import Sim, SimWorld, SimWorldConfig
from dimos.utils.data import get_data

ODOM_FRAME_ID = "odom"
SENSOR_FRAME_ID = "mid360_link"
STEP_LIMIT = 0.2
STAND_HEIGHT = 0.3
# the Mid-360 on its mount is the highest point of the standing sim Go2
CLEARANCE_HEIGHT = 0.6


# The Go2 + Mid-360 rig, measured: base_link -> front_camera (the URDF camera mount), then the lidar 3.2 cm back,
# 12 cm up and pitched 60 deg down.
FRAMES: list[FrameSpec] = [
    ("base_link", None, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("front_camera", "base_link", (0.32715, -0.00003, 0.04297), (0.0, 0.0, 0.0)),
    ("mid360_link", "front_camera", (-0.032, 0.0, 0.12), (0.0, math.radians(60.0), 0.0)),
]


class SimGo2MountTf(StaticTfPublisher):
    """mid360_link -> front_camera -> base_link, so base_link hangs off the odometry's mid360_link."""

    def transforms(self) -> list[Transform]:
        edges = {t.child_frame_id: t for t in frames_to_edge_transforms(FRAMES)}
        return [-edges["mid360_link"], -edges["front_camera"]]


def _mount() -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """base_link -> mid360_link as a translation and rotation matrix, from the rig's frame tree."""
    edges = {t.child_frame_id: t for t in frames_to_edge_transforms(FRAMES)}
    matrix = edges["front_camera"].to_matrix() @ edges["mid360_link"].to_matrix()
    return matrix[:3, 3].copy(), matrix[:3, :3].copy()


def _lidar(raycaster: Raycaster, seed: int) -> SimLidar:
    """The Mid-360 shadowed by the Go2 mount."""
    occlusion = OcclusionMap.load(get_data("go2_sim") / "go2_mid360_occlusion.npz")
    return sim_mid360(raycaster, seed, occlusion)


MOUNT_XYZ, MOUNT_R = _mount()
MID360 = Sensor("mid360", MOUNT_XYZ, MOUNT_R, _lidar, HOUSING_HALF_EXTENTS)

SHAPE = BodyShape(
    half_width=ROBOT_WIDTH / 2,
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

GO2 = Rig(
    spec=go2_spec,
    base_body="base",
    sensors=(MID360,),
    shape=SHAPE,
    command_timeout=CMD_VEL_TIMEOUT,
    tune=apply_fitted_physics,
    world_frame=ODOM_FRAME_ID,
    odom_child_frame=SENSOR_FRAME_ID,
    cloud_frame="sensor",
)


class SimGo2WorldConfig(SimWorldConfig):
    policy: Path | None = None


class SimGo2World(SimWorld):
    """A generated scene with the legged Go2 and its simulated Mid-360, paced to the wall clock."""

    config: SimGo2WorldConfig
    rig = GO2

    @rpc
    def start(self) -> None:
        self._policy = load_policy(self.config.policy)
        super().start()

    def _build(self, scene: Scene, seed: int) -> Sim:
        return Sim(scene, seed, self.rig, partial(LeggedGo2, policy=self._policy))
