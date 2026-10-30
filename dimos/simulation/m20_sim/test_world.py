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

from __future__ import annotations

from functools import partial

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.msgs.sim_msgs.Contacts import Contact
from dimos.robot.deeprobotics.m20.blueprints._nav import VOXEL_SIZE_M
from dimos.robot.deeprobotics.m20.constants import FRONT_LIDAR_XYZ_M, REAR_LIDAR_XYZ_M
from dimos.simulation.m20_sim.body import LeggedM20
from dimos.simulation.m20_sim.policy import M20Policy
from dimos.simulation.m20_sim.world import M20_RIG
from dimos.simulation.scenes.procedural import Scene, office
from dimos.simulation.sim_world.world import FRAME_DT, LidarFrame, Sim

STILL = np.zeros(3)
FORWARD = np.array([0.5, 0.0, 0.0])


@pytest.fixture(scope="module")
def policy() -> M20Policy:
    return M20Policy.load()


@pytest.fixture(scope="module")
def scene() -> Scene:
    return office(1)


@pytest.fixture
def sim(scene: Scene, policy: M20Policy) -> Sim:
    sim = Sim(scene, 1, M20_RIG, partial(LeggedM20, policy=policy))
    sim.reset(*scene.start, 0.0)
    return sim


def _next_frame(sim: Sim, command: NDArray[np.float64]) -> LidarFrame:
    for _ in range(sim.ticks_per_frame):
        if (frame := sim.tick(command)) is not None:
            return frame
    raise AssertionError("no lidar frame within one frame period")


def _floor_is_flat(frame: LidarFrame, sim: Sim) -> None:
    floor = sim.scene.params["z0"]
    position, rotation = sim.odom_pose()
    z = (position + frame.points.astype(np.float64) @ rotation.T)[:, 2]
    assert z.min() > floor - 0.05
    assert abs(np.quantile(z, 0.1) - floor) < 0.03
    assert np.std(z[np.abs(z - floor) < 0.05]) < 0.02


def test_frames_come_out_at_10hz_in_base_link_from_both_lidars(sim: Sim) -> None:
    frames = [f for _ in range(10) if (f := sim.tick(STILL)) is not None]
    assert [f.t for f in frames] == pytest.approx([FRAME_DT, 2 * FRAME_DT])
    for got, want in zip(sim.odom_pose(), sim.base_pose(), strict=True):
        assert np.array_equal(got, want)
    x = frames[-1].points[:, 0]
    assert (x > FRONT_LIDAR_XYZ_M[0]).sum() > 20_000
    assert (x < REAR_LIDAR_XYZ_M[0]).sum() > 20_000
    _floor_is_flat(frames[-1], sim)


def test_deskew_keeps_the_floor_flat_while_walking(sim: Sim) -> None:
    for _ in range(75):
        sim.tick(FORWARD)
    _floor_is_flat(_next_frame(sim, FORWARD), sim)


def test_no_return_off_the_robot_rises_out_of_the_floor_voxel(sim: Sim) -> None:
    for command in (STILL, FORWARD):
        for _ in range(25):
            sim.tick(command)
        p = _next_frame(sim, command).points
        floor = sim.scene.params["z0"] - sim.base_pose()[0][2]
        inside = (
            (np.abs(p[:, 0]) < 0.5) & (np.abs(p[:, 1]) < 0.32) & (p[:, 2] > floor + VOXEL_SIZE_M)
        )
        assert inside.sum() == 0


def test_standing_robot_touches_the_floor_only_with_its_wheels(sim: Sim) -> None:
    for _ in range(25):
        sim.tick(STILL)
    assert sim.contacts() == [Contact("wheel", "floor")]


def test_the_lidars_see_the_robot_by_its_collision_shapes(sim: Sim) -> None:
    position, _ = sim.sensor_pose(M20_RIG.sensors[0])
    left = sim.base_pose()[1] @ np.array([0.0, 1.0, 0.0])
    dist, _ = sim.lidars[0].raycaster.cast(position, left[None, :], 10.0)
    assert 0.03 < dist[0] < 0.1
