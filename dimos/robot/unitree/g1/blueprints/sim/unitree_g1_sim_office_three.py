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


"""Three simulated G1s in office 555 on MuJoCo, on its main, left and right spawn points, each walked by its own GR00T walker, and a room camera that sees all three.

Only ``main`` takes ``cmd_vel`` from outside its namespace, so what drives one G1 drives that one;
``left`` and ``right`` stand under their own walkers.
"""

import math

from pydantic import Field

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.hardware_module import Mount, Mounts
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.g1.body_sim import G1_ODOM_TF, G1BodySim
from dimos.robot.unitree.g1.walker import G1Walker
from dimos.sim2.camera import SimCamera
from dimos.sim2.livox import LivoxSim
from dimos.sim2.mujoco.sim import MujocoSim
from dimos.sim2.odom_tf import OdomTf
from dimos.sim2.scene import SpawnMounts, scene_usd, spawn_frames_in_file
from dimos.sim2.sim import on_mount, sim
from dimos.simulation.scenes.office_555.room import SCENE, Office555, Office555Config

# A room camera (x forward) off the spots' corner, turned to see all three apart and 22 degrees down at their waists.
WATCHING = ((-3.5, -10.0, 2.4), (0.0, math.radians(22), math.radians(42)))


class Office555WatchedConfig(Office555Config):
    usd: UsdModel = Field(
        default_factory=lambda: scene_usd(SCENE).with_frames(
            {"watching": ("office_555", *WATCHING)}
        )
    )


class _WatchedMounts(SpawnMounts):
    """Office 555's spawn points and the room camera's mount, read from the scene the first time one is asked for."""

    def _read(self) -> None:
        if "_frames" not in vars(self):
            Mounts.__init__(self, *spawn_frames_in_file(self._path), watching="office_555/watching")


class Office555Watched(Office555):
    """Office 555 with a room camera mount, ``watching``, that sees its main, left and right spots."""

    config: Office555WatchedConfig
    mounts = _WatchedMounts(SCENE)


def _g1(spot: Mount, driven: bool) -> Blueprint:
    return on_mount(
        autoconnect(
            G1BodySim.blueprint(attached_to=spot.name),
            OdomTf.blueprint(**G1_ODOM_TF),
            G1Walker.blueprint(),
            LivoxSim.attach(G1BodySim.mounts.lidar),
            SimCamera.attach(G1BodySim.mounts.camera),
        ),
        spot,
        shared={"cmd_vel", "odom"} if driven else set(),
    )


unitree_g1_sim_office_three = autoconnect(
    sim({"mujoco": MujocoSim.blueprint()}, default="mujoco"),
    Office555Watched.blueprint(),
    SimCamera.attach(Office555Watched.mounts.watching),
    _g1(Office555Watched.mounts.main, driven=True),
    _g1(Office555Watched.mounts.left, driven=False),
    _g1(Office555Watched.mounts.right, driven=False),
).global_config(robot_model="unitree_g1")
