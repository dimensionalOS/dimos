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

import time

import mujoco
import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sim_msgs.Contacts import Contact, Contacts
from dimos.simulation.go2_legged.policy import OnnxGo2Policy
from dimos.simulation.go2_legged.robot import CONTROL_DT
from dimos.simulation.go2_sim.world import (
    CEILING_GROUP,
    CONTACTS_HEARTBEAT_DT,
    FRAME_DT,
    LIDAR_GROUPS,
    MOUNT_R,
    TICKS_PER_FRAME,
    Go2Sim,
    LidarFrame,
    SimGo2World,
    scene_edges,
)
from dimos.simulation.scenes.procedural import Scene, office
from dimos.simulation.sensors.mujoco_raycaster import MujocoRaycaster

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
def sim(scene: Scene, policy: OnnxGo2Policy) -> Go2Sim:
    sim = Go2Sim(scene, seed=1, policy=policy)
    sim.reset(*scene.start, 0.0)
    return sim


def _next_frame(sim: Go2Sim, command: NDArray[np.float64]) -> LidarFrame:
    for _ in range(TICKS_PER_FRAME):
        if (frame := sim.tick(command)) is not None:
            return frame
    raise AssertionError("no lidar frame within one frame period")


def _floor_is_flat(frame: LidarFrame, sim: Go2Sim, floor: float) -> None:
    position, rotation = sim.sensor_pose()
    world = position + frame.points.astype(np.float64) @ rotation.T
    z = world[:, 2]
    assert z.min() > floor - 0.05
    assert abs(np.quantile(z, 0.1) - floor) < 0.03
    assert np.std(z[np.abs(z - floor) < 0.05]) < 0.02


def test_frames_come_out_at_10hz_in_the_sensor_frame(sim: Go2Sim) -> None:
    frames = [f for _ in range(10) if (f := sim.tick(STILL)) is not None]
    assert [f.t for f in frames] == pytest.approx([FRAME_DT, 2 * FRAME_DT])
    assert 12_000 < len(frames[-1].points) < 20_000
    _floor_is_flat(frames[-1], sim, sim.scene.params["z0"])


def test_deskew_keeps_the_floor_flat_while_walking(sim: Go2Sim) -> None:
    for _ in range(75):
        sim.tick(FORWARD)
    _floor_is_flat(_next_frame(sim, FORWARD), sim, sim.scene.params["z0"])


def test_standing_robot_touches_the_floor_only_with_its_feet(sim: Go2Sim) -> None:
    for _ in range(25):
        sim.tick(STILL)
    contacts = sim.contacts()
    assert Contact("foot", "floor") in contacts
    assert {c.kind for c in contacts} == {"floor"}
    assert "trunk" not in {c.part for c in contacts}


def test_contacts_name_the_scene_kind_and_the_robot_part(sim: Go2Sim) -> None:
    sim.reset(-0.05, 3.0, sim.scene.start[2], 0.0)
    assert Contact("trunk", "wall") in sim.contacts()
    clutter = max(
        (b for b in sim.scene.boxes if b.kind == "clutter"), key=lambda b: min(b.half[:2])
    )
    sim.reset(clutter.center[0], clutter.center[1], clutter.center[2] + clutter.half[2], 0.0)
    for _ in range(25):
        sim.tick(STILL)
    assert Contact("foot", "clutter") in sim.contacts()


def test_sensor_velocity_matches_the_motion(sim: Go2Sim) -> None:
    for _ in range(75):
        sim.tick(FORWARD)
    before, _ = sim.sensor_pose()
    linear = np.zeros(3)
    for _ in range(10):
        sim.tick(FORWARD)
        linear += sim.sensor_velocity()[0] / 10
    after, _ = sim.sensor_pose()
    assert linear == pytest.approx((after - before) / (10 * CONTROL_DT), abs=0.1)
    assert linear[0] > 0.1
    turn = np.array([0.0, 0.0, 0.8])
    for _ in range(50):
        sim.tick(turn)
    yaw_before = sim.robot.yaw()
    rate = 0.0
    for _ in range(10):
        sim.tick(turn)
        rate += (MOUNT_R @ sim.sensor_velocity()[1])[2] / 10
    yaw_delta = (sim.robot.yaw() - yaw_before + np.pi) % (2 * np.pi) - np.pi
    assert rate == pytest.approx(yaw_delta / (10 * CONTROL_DT), abs=0.1)
    assert rate > 0.05


def test_ceiling_is_hidden_from_the_viewer_but_not_the_lidar(sim: Go2Sim) -> None:
    index, ceiling = next(
        (i, box) for i, box in enumerate(sim.scene.boxes) if box.kind == "ceiling"
    )
    underside = ceiling.center[2] - ceiling.half[2]
    position, _ = sim.sensor_pose()
    raycaster = MujocoRaycaster(sim.model, sim.data, LIDAR_GROUPS)
    dist, _ = raycaster.cast(position, np.array([[0.0, 0.0, 1.0]]), 10.0)
    assert abs(dist[0] - (underside - position[2])) < 0.01
    geom = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, f"ceiling_{index}")
    assert sim.model.geom_group[geom] == CEILING_GROUP


def test_sensor_sits_on_the_mount_above_the_base(sim: Go2Sim) -> None:
    base, _ = sim.base_pose()
    sensor, rotation = sim.sensor_pose()
    assert 0.1 < sensor[2] - base[2] < 0.2
    forward = rotation @ np.array([1.0, 0.0, 0.0])
    assert forward[2] < -0.8


def test_reset_pose_moves_the_robot_through_the_sim_thread(scene: Scene) -> None:
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
        time.sleep(2.0)
        world.reset_pose(3.0, 2.0, scene.params["z0"], 1.0)

        # the bundled policy lurches a few decimeters as it is set down, so take the first pose there
        def near() -> PoseStamped | None:
            return next((p for p in poses if abs(p.position.x - 3.0) < 0.5), None)

        while near() is None and time.monotonic() < deadline:
            time.sleep(0.02)
        first = near()
        assert first is not None
        assert (first.position.x, first.position.y) == pytest.approx((3.0, 2.0), abs=0.15)
        assert first.orientation.to_euler().z == pytest.approx(1.0, abs=0.2)
        assert first.ts > poses[0].ts
        while not any(c.ts >= first.ts for c in heard) and time.monotonic() < deadline:
            time.sleep(0.05)
        after = min(c.ts for c in heard if c.ts >= first.ts)
        assert after - first.ts < CONTACTS_HEARTBEAT_DT + 0.5
    finally:
        world.stop()


def test_joint_state_names_every_leg_joint(sim: Go2Sim) -> None:
    positions, velocities = sim.robot.joint_state()
    assert len(positions) == len(velocities) == len(sim.robot.policy.joint_names) == 12
    assert positions == pytest.approx(sim.robot.policy.default_pose, abs=0.3)


def test_contacts_are_republished_on_a_heartbeat() -> None:
    world = SimGo2World()
    world.cmd_vel.transport = LCMTransport("/test_go2_sim_world/cmd_vel_heartbeat", Twist)
    heard: list[Contacts] = []
    world.contacts.subscribe(heard.append)
    world.start()
    try:
        deadline = time.monotonic() + 10.0
        while len(heard) < 3 and time.monotonic() < deadline:
            time.sleep(0.05)
    finally:
        world.stop()
    assert len(heard) >= 3
    assert heard[-1].ts - heard[-2].ts == pytest.approx(CONTACTS_HEARTBEAT_DT, abs=0.1)
    assert heard[-1].contacts == [Contact("foot", "floor")]


def test_scene_edges_cover_every_box(scene: Scene) -> None:
    edges = scene_edges(scene)
    assert edges.shape == (12 * len(scene.boxes), 2, 3)
    lo, hi = scene.bounds()
    assert np.allclose(edges.reshape(-1, 3).min(0), lo)
    assert np.allclose(edges.reshape(-1, 3).max(0), hi)
