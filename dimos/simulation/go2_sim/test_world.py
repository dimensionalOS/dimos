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

from collections.abc import Iterator
from dataclasses import replace
from functools import partial
import math
from pathlib import Path
import time

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.sim_msgs.Contacts import Contact, Contacts
from dimos.simulation.go2_legged.policy import OnnxGo2Policy
from dimos.simulation.go2_legged.robot import CONTROL_DT, LeggedGo2
from dimos.simulation.go2_sim.world import GO2, MID360, MOUNT_R, SHAPE, SimGo2World
from dimos.simulation.scenes.procedural import Box, Scene, block, office
from dimos.simulation.sensors.mujoco_raycaster import MujocoRaycaster
from dimos.simulation.sim_world.premap import premap_cloud, without_objects
from dimos.simulation.sim_world.world import (
    CONTACTS_HEARTBEAT_DT,
    FRAME_DT,
    LIDAR_GROUPS,
    LidarFrame,
    Sim,
    display_xml,
    scene_edges,
)

pytestmark = pytest.mark.self_hosted

STILL = np.zeros(3)
FORWARD = np.array([0.8, 0.0, 0.0])


@pytest.fixture(scope="module")
def policy() -> OnnxGo2Policy:
    return OnnxGo2Policy.load()


@pytest.fixture(scope="module")
def scene() -> Scene:
    return office(1)


@pytest.fixture
def sim(scene: Scene, policy: OnnxGo2Policy) -> Sim:
    sim = Sim(scene, 1, GO2, partial(LeggedGo2, policy=policy))
    sim.reset(*scene.start, 0.0)
    return sim


@pytest.fixture
def waiting_world() -> Iterator[SimGo2World]:
    world = SimGo2World(wait_for_scene=True)
    world.cmd_vel.transport = LCMTransport("/test_go2_sim_world_wait/cmd_vel", Twist)
    yield world
    world.stop()


def _yaw(sim: Sim) -> float:
    _, rotation = sim.base_pose()
    return math.atan2(rotation[1, 0], rotation[0, 0])


def _next_frame(sim: Sim, command: NDArray[np.float64]) -> LidarFrame:
    for _ in range(sim.ticks_per_frame):
        if (frame := sim.tick(command)) is not None:
            return frame
    raise AssertionError("no lidar frame within one frame period")


def _floor_is_flat(frame: LidarFrame, sim: Sim, floor: float) -> None:
    position, rotation = sim.odom_pose()
    world = position + frame.points.astype(np.float64) @ rotation.T
    z = world[:, 2]
    assert z.min() > floor - 0.05
    assert abs(np.quantile(z, 0.1) - floor) < 0.03
    assert np.std(z[np.abs(z - floor) < 0.05]) < 0.02


def test_frames_come_out_at_10hz_in_the_sensor_frame(sim: Sim) -> None:
    frames = [f for _ in range(10) if (f := sim.tick(STILL)) is not None]
    assert [f.t for f in frames] == pytest.approx([FRAME_DT, 2 * FRAME_DT])
    assert 12_000 < len(frames[-1].points) < 20_000
    _floor_is_flat(frames[-1], sim, sim.scene.params["z0"])


def test_deskew_keeps_the_floor_flat_while_walking(sim: Sim) -> None:
    for _ in range(75):
        sim.tick(FORWARD)
    _floor_is_flat(_next_frame(sim, FORWARD), sim, sim.scene.params["z0"])


def test_standing_robot_touches_the_floor_only_with_its_feet(sim: Sim) -> None:
    for _ in range(25):
        sim.tick(STILL)
    contacts = sim.contacts()
    assert Contact("foot", "floor") in contacts
    assert {c.kind for c in contacts} == {"floor"}
    assert "trunk" not in {c.part for c in contacts}


def test_contacts_name_the_scene_kind_and_the_robot_part(sim: Sim) -> None:
    sim.reset(-0.05, 3.0, sim.scene.start[2], 0.0)
    assert Contact("trunk", "wall") in sim.contacts()
    clutter = max(
        (b for b in sim.scene.boxes if b.kind == "clutter"), key=lambda b: min(b.half[:2])
    )
    sim.reset(clutter.center[0], clutter.center[1], clutter.center[2] + clutter.half[2], 0.0)
    for _ in range(25):
        sim.tick(STILL)
    assert Contact("foot", "clutter") in sim.contacts()


def test_sensor_velocity_matches_the_motion(sim: Sim) -> None:
    for _ in range(75):
        sim.tick(FORWARD)
    before, _ = sim.odom_pose()
    linear = np.zeros(3)
    for _ in range(10):
        sim.tick(FORWARD)
        linear += sim.odom_velocity()[0] / 10
    after, _ = sim.odom_pose()
    assert linear == pytest.approx((after - before) / (10 * CONTROL_DT), abs=0.1)
    assert linear[0] > 0.1
    turn = np.array([0.0, 0.0, 0.8])
    for _ in range(50):
        sim.tick(turn)
    yaw_before = _yaw(sim)
    rate = 0.0
    for _ in range(10):
        sim.tick(turn)
        rate += (MOUNT_R @ sim.odom_velocity()[1])[2] / 10
    yaw_delta = (_yaw(sim) - yaw_before + np.pi) % (2 * np.pi) - np.pi
    assert rate == pytest.approx(yaw_delta / (10 * CONTROL_DT), abs=0.1)
    assert rate > 0.05


def test_the_lidar_sees_nothing_overhead(sim: Sim) -> None:
    position, _ = sim.sensor_pose(MID360)
    raycaster = MujocoRaycaster(sim.model, sim.data, LIDAR_GROUPS)
    dist, _ = raycaster.cast(position, np.array([[0.0, 0.0, 1.0]]), 10.0)
    assert dist[0] < 0


def test_sensor_sits_on_the_mount_above_the_base(sim: Sim) -> None:
    base, _ = sim.base_pose()
    sensor, rotation = sim.sensor_pose(MID360)
    assert 0.1 < sensor[2] - base[2] < 0.2
    forward = rotation @ np.array([1.0, 0.0, 0.0])
    assert forward[2] < -0.8


def test_a_rig_with_two_lidars_reports_both_at_its_base(
    scene: Scene, policy: OnnxGo2Policy, sim: Sim
) -> None:
    rear = replace(
        MID360, name="mid360_rear", mount_xyz=MID360.mount_xyz + np.array([-0.3, 0.0, 0.05])
    )
    rig = replace(GO2, sensors=(MID360, rear), odom_child_frame="base_link", cloud_frame="base")
    both = Sim(scene, 1, rig, partial(LeggedGo2, policy=policy))
    both.reset(*scene.start, 0.0)
    frame = _next_frame(both, STILL)
    assert len(frame.points) > 1.5 * len(_next_frame(sim, STILL).points)
    _floor_is_flat(frame, both, scene.params["z0"])
    for got, want in zip(both.odom_pose(), both.base_pose(), strict=True):
        assert np.array_equal(got, want)
    for got, want in zip(both.odom_velocity(), both.robot.base_velocity(), strict=True):
        assert np.array_equal(got, want)
    rear_on_the_wall = (-float(rear.mount_xyz[0]), 3.0, scene.start[2], 0.0)
    both.reset(*rear_on_the_wall)
    assert Contact("lidar", "wall") in both.contacts()
    sim.reset(*rear_on_the_wall)
    assert Contact("lidar", "wall") not in sim.contacts()


def test_reset_pose_moves_the_robot_and_the_contacts_keep_their_heartbeat(scene: Scene) -> None:
    world = SimGo2World()
    world.cmd_vel.transport = LCMTransport("/test_go2_sim_world/cmd_vel", Twist)
    poses: list[PoseStamped] = []
    heard: list[Contacts] = []
    world.ground_truth.subscribe(poses.append)
    world.contacts.subscribe(heard.append)
    world.start()
    try:
        deadline = time.monotonic() + 10.0
        while len(poses) < 5 and time.monotonic() < deadline:
            time.sleep(0.05)
        assert poses[-1].position.x == pytest.approx(scene.start[0], abs=0.1)
        time.sleep(2.5)
        assert heard[-1].ts - heard[-2].ts == pytest.approx(CONTACTS_HEARTBEAT_DT, abs=0.1)
        assert heard[-1].contacts == [Contact("foot", "floor")]
        world.reset_pose(3.0, 2.0, scene.params["z0"], 1.0)
        # the bundled policy lurches a few decimeters as it is set down, so take the first pose there
        moved = [p for p in poses if abs(p.position.x - 3.0) < 0.5]
        while not moved and time.monotonic() < deadline:
            time.sleep(0.02)
            moved = [p for p in poses if abs(p.position.x - 3.0) < 0.5]
        first = moved[0]
        assert (first.position.x, first.position.y) == pytest.approx((3.0, 2.0), abs=0.15)
        assert first.orientation.to_euler().z == pytest.approx(1.0, abs=0.2)
        assert first.ts > poses[0].ts
        while not any(c.ts >= first.ts for c in heard) and time.monotonic() < deadline:
            time.sleep(0.05)
        after = min(c.ts for c in heard if c.ts >= first.ts)
        assert after - first.ts < CONTACTS_HEARTBEAT_DT + 0.5
    finally:
        world.stop()


def test_a_waiting_world_is_silent_until_it_loads_its_one_scene(
    waiting_world: SimGo2World,
) -> None:
    poses: list[PoseStamped] = []
    drawn: list[LineSegments3D] = []
    waiting_world.ground_truth.subscribe(poses.append)
    waiting_world.scene.subscribe(drawn.append)
    waiting_world.start()
    time.sleep(1.0)
    assert not poses and not drawn
    assert waiting_world.shape() == SHAPE
    waiting_world.load_scene("office", 2, {})
    deadline = time.monotonic() + 10.0
    while not drawn and time.monotonic() < deadline:
        time.sleep(0.05)
    assert len(drawn[0].segments) == len(scene_edges(office(2)))
    with pytest.raises(RuntimeError, match="already runs a scene"):
        waiting_world.load_scene("office", 3, {})


def test_the_world_publishes_its_premap_and_exports_its_robot(
    waiting_world: SimGo2World, tmp_path: Path
) -> None:
    maps: list[PointCloud2] = []
    waiting_world.loaded_map.subscribe(maps.append)
    waiting_world.start()
    params = {"block_height": 0.4}
    waiting_world.load_scene("block", 1, params)
    scene = block(1, **params)
    x, y, z0 = (scene.params[k] for k in ("block_x", "block_y", "z0"))
    walk = np.array([[x - 2.0, y, z0], [x - 1.0, y, z0]])
    for objects, seen in ((True, scene), (False, without_objects(scene))):
        count = waiting_world.publish_premap(walk, objects)
        assert maps[-1].frame_id == "odom" and len(maps[-1].points_f32()) == count
        assert np.array_equal(maps[-1].points_f32(), premap_cloud(seen, GO2, walk))
    robot = tmp_path / "robot.xml"
    waiting_world.export_model(robot)
    assert robot.read_text() == display_xml(GO2)


def test_scene_edges_cover_every_box(scene: Scene) -> None:
    edges = scene_edges(scene)
    assert edges.shape == (12 * len(scene.boxes), 2, 3)
    lo, hi = scene.bounds()
    assert np.allclose(edges.reshape(-1, 3).min(0), lo)
    assert np.allclose(edges.reshape(-1, 3).max(0), hi)
    turned = Scene("turned", boxes=[Box((1.0, 2.0, 0.5), (2.0, 0.5, 0.5), "wall", math.pi / 2)])
    corners = scene_edges(turned).reshape(-1, 3)
    assert np.allclose(corners.min(0), [0.5, 0.0, 0.0])
    assert np.allclose(corners.max(0), [1.5, 4.0, 1.0])
