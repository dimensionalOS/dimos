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

import math
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("pxr")
pytest.importorskip("mujoco")

SCENE = Path(__file__).parents[3] / "sim2" / "mujoco" / "assets" / "scene.usda"
MOUNT_XYZ = (1.0, -2.0, 0.5)
MOUNT_YAW = 0.7


def test_odom_is_from_the_mount_so_the_pelvis_stands_above_it() -> None:
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.robot.assets.model import UsdModel
    from dimos.robot.assets.usd_utils.layers import graft_usd
    from dimos.robot.unitree.g1.sim2 import G1_USD, G1Sim
    from dimos.sim2.mujoco.backend import MujocoBackend
    from dimos.sim2.spec import SimulatedRobot
    from dimos.sim2.world_observations import WorldMirror

    scene = UsdModel.from_file(SCENE).with_frames(
        {"g1": ("scene", MOUNT_XYZ, (0.0, 0.0, MOUNT_YAW))}
    )
    sim = G1Sim(_mount="g1", instance_name="g1/G1Sim")
    try:
        g1 = sim.get_usd()
        assert g1 is not None
        world = UsdModel.from_layer(graft_usd(scene.usd, {"g1": g1}))
        backend = MujocoBackend(world, [SimulatedRobot("g1", G1_USD)])
        sim._world = WorldMirror()
        sim._world.apply(backend.frame())
        odom = sim._odom(sim._world.stamp())
        assert odom is not None
        odom_pelvis = Transform.from_pose("pelvis", odom).to_matrix()
        mount = Transform.from_matrix(np.eye(4)).to_matrix()
        mount[:3, 3] = MOUNT_XYZ
        c, s = math.cos(MOUNT_YAW), math.sin(MOUNT_YAW)
        mount[:2, :2] = [[c, -s], [s, c]]
        pelvis = backend.data.xpos[backend.model.body("g1/pelvis").id]
        # tf: the mount holds odom by identity, so mount @ odom is where the pelvis is.
        assert (mount @ odom_pelvis)[:3, 3] == pytest.approx(pelvis, abs=1e-4)
        assert odom.position.z == pytest.approx(0.793, abs=1e-4)
    finally:
        sim.stop()
