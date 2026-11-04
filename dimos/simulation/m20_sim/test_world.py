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

from dataclasses import replace
from functools import partial

import mujoco
import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.msgs.sim_msgs.Contacts import Contact
from dimos.robot.deeprobotics.m20.constants import FRONT_LIDAR_XYZ_M, REAR_LIDAR_XYZ_M
from dimos.simulation.m20_sim.body import LeggedM20
from dimos.simulation.m20_sim.policy import M20Policy
from dimos.simulation.m20_sim.world import (
    LIO_BLIND_RANGE,
    M20_RIG,
    SimM20WorldConfig,
    with_blind_range,
)
from dimos.simulation.scenes.procedural import Scene, office
from dimos.simulation.sensors.airy.lidar import LENS_HEIGHT
from dimos.simulation.sensors.airy.pattern import FRAME_RATE, POINT_RATE, AiryPattern
from dimos.simulation.sensors.mujoco_raycaster import MujocoRaycaster
from dimos.simulation.sim_world.world import FRAME_DT, LidarFrame, Sim

pytestmark = pytest.mark.self_hosted

STILL = np.zeros(3)
FORWARD = np.array([0.5, 0.0, 0.0])
WHEEL_TOP = 0.22
# the guards sit within the vendor driver's own minimum distance
GUARD_REACH = 0.2
# turning in place and walking while turning swing a far wheel furthest into each Airy's view
GAIT = (np.array([0.0, 0.0, -0.7]), np.array([0.7, 0.0, 0.7]))


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
    assert (x > FRONT_LIDAR_XYZ_M[0]).sum() > 5_000
    assert (x < REAR_LIDAR_XYZ_M[0]).sum() > 5_000
    _floor_is_flat(frames[-1], sim)


def test_deskew_keeps_the_floor_flat_while_walking(sim: Sim) -> None:
    for _ in range(75):
        sim.tick(FORWARD)
    _floor_is_flat(_next_frame(sim, FORWARD), sim)


def test_returns_off_the_robot_stay_at_wheel_height(sim: Sim) -> None:
    for command in (STILL, FORWARD):
        for _ in range(25):
            sim.tick(command)
        p = _next_frame(sim, command).points
        floor = sim.scene.params["z0"] - sim.base_pose()[0][2]
        inside = (np.abs(p[:, 0]) < 0.5) & (np.abs(p[:, 1]) < 0.32) & (p[:, 2] > floor + WHEEL_TOP)
        assert inside.sum() == 0


def test_the_guards_block_the_lidars_without_touching_the_physics(sim: Sim) -> None:
    model = sim.model
    per_frame = round(POINT_RATE / FRAME_RATE)
    directions = AiryPattern().directions(0, per_frame)
    for sensor, lidar in zip(M20_RIG.sensors, sim.lidars, strict=True):
        guard = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{sensor.name}_guard")
        geoms = model.geom_bodyid == guard
        assert geoms.any()
        assert model.body_mass[guard] == 0.0
        assert not model.geom_contype[geoms].any() and not model.geom_conaffinity[geoms].any()
        position, rotation = sim.sensor_pose(sensor)
        lens = position + rotation @ np.array([0.0, 0.0, LENS_HEIGHT])
        keeps_own = MujocoRaycaster(
            model, sim.data, sim.lidar_groups, lidar.raycaster.excluded_body
        )
        dist, _ = keeps_own.cast(lens, directions @ rotation.T, 10.0)
        assert 0.2 < ((dist >= 0.0) & (dist < GUARD_REACH)).mean() < 0.35


def test_the_cloud_keeps_one_return_in_five(scene: Scene, policy: M20Policy) -> None:
    every_return = replace(
        M20_RIG, sensors=tuple(replace(s, lio_keep_every=1) for s in M20_RIG.sensors)
    )
    counts = []
    for rig in (M20_RIG, every_return):
        sim = Sim(scene, 1, rig, partial(LeggedM20, policy=policy))
        sim.reset(*scene.start, 0.0)
        counts.append(len(_next_frame(sim, STILL).points))
    assert counts[0] == pytest.approx(counts[1] / 5, rel=0.01)


def test_standing_robot_touches_the_floor_only_with_its_wheels(sim: Sim) -> None:
    for _ in range(25):
        sim.tick(STILL)
    assert sim.contacts() == [Contact("wheel", "floor")]


def test_the_lidars_see_the_robot_by_its_collision_shapes(sim: Sim) -> None:
    position, _ = sim.sensor_pose(M20_RIG.sensors[0])
    left = sim.base_pose()[1] @ np.array([0.0, 1.0, 0.0])
    keeps_own = MujocoRaycaster(
        sim.model, sim.data, sim.lidar_groups, sim.lidars[0].raycaster.excluded_body
    )
    dist, _ = keeps_own.cast(position, left[None, :], 10.0)
    assert 0.03 < dist[0] < 0.1


def test_the_lidars_never_return_their_own_robot(sim: Sim) -> None:
    directions = np.random.default_rng(0).normal(size=(20_000, 3))
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    robot_hits = 0
    for command in (STILL, *GAIT):
        for tick in range(60):
            sim.tick(command)
            if tick % 10:
                continue
            for sensor, lidar in zip(M20_RIG.sensors, sim.lidars, strict=True):
                position, rotation = sim.sensor_pose(sensor)
                world_directions = directions @ rotation.T
                unfiltered = MujocoRaycaster(
                    sim.model, sim.data, sim.lidar_groups, lidar.raycaster.excluded_body
                )
                raw, _ = unfiltered.cast(position, world_directions, 10.0)
                dist, _ = lidar.raycaster.cast(position, world_directions, 10.0)
                hit_robot = (raw >= 0.0) & (dist < 0.0)
                robot_hits += int(hit_robot.sum())
                # every kept return lands on the scene, at the range the unfiltered ray found
                assert np.array_equal(dist[dist >= 0.0], raw[dist >= 0.0])
    assert robot_hits > 0


def test_the_blind_range_is_the_worlds_config() -> None:
    assert SimM20WorldConfig().lio_blind_range == LIO_BLIND_RANGE
    rig = with_blind_range(M20_RIG, SimM20WorldConfig(lio_blind_range=0.7).lio_blind_range)
    assert [s.lio_blind_range for s in rig.sensors] == [0.7, 0.7]
