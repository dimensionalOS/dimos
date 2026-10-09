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

import json
import math
from pathlib import Path
import threading
import time

import numpy as np
import pytest

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Bool import Bool
from dimos.navigation.sim_eval.driver import TERMINAL_FILE, EpisodeDriver
from dimos.navigation.sim_eval.ground_truth import GroundTruth
from dimos.navigation.sim_eval.suite import Case, Manifest, Rules
from dimos.simulation.scenes.procedural import office

RULES = Rules(premap="walk", stuck_s=1.0, stalled_s=1.0)


class FakeWorld:
    """Moves a point robot by the teleop commands it is given and publishes its pose at 20 Hz."""

    def __init__(self, poses: LCMTransport, z: float) -> None:
        self.resets: list[tuple[float, float, float, float]] = []
        self.xy = np.zeros(2)
        self.yaw = 0.0
        self.z = z
        self.command = np.zeros(3)
        self._poses = poses
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def reset_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        self.resets.append((x, y, z, yaw))
        self.xy, self.yaw = np.array([x, y]), yaw

    def on_teleop(self, msg: Twist) -> None:
        self.command = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def _run(self) -> None:
        while not self._stop.is_set():
            dt = 0.05
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            self.xy = self.xy + dt * np.array(
                [
                    c * self.command[0] - s * self.command[1],
                    s * self.command[0] + c * self.command[1],
                ]
            )
            self.yaw += dt * self.command[2]
            self._poses.publish(
                PoseStamped(
                    float(self.xy[0]),
                    float(self.xy[1]),
                    self.z,
                    0.0,
                    0.0,
                    math.sin(self.yaw / 2),
                    math.cos(self.yaw / 2),
                    ts=time.time(),
                    frame_id="odom",
                )
            )
            time.sleep(dt)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)


def _manifest(path: Path, rules: Rules) -> Path:
    scene = office(1)
    gt = GroundTruth(scene)
    start = (scene.start[0], scene.start[1], 0.0)
    goal = gt.center(*gt.index((scene.start[0] + 3.0, scene.start[1])))
    route = gt.route(start, goal)
    assert route is not None
    case = Case(
        family="office",
        seed=1,
        params={},
        start=start,
        goal=(float(goal[0]), float(goal[1]), float(goal[2])),
        tag="mined",
        id="mined-s1-test",
        split="dev",
        difficulty=gt.difficulty(route),
        route_length=route.length,
        scene_digest=scene.digest(),
    )
    Manifest("test", rules, 0, None, False, [case], []).save(path)
    return path


@pytest.fixture(scope="module")
def manifest_path(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return _manifest(tmp_path_factory.mktemp("suite") / "suite.json", RULES)


@pytest.fixture(scope="module")
def seeded_manifest_path(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return _manifest(tmp_path_factory.mktemp("seeded") / "suite.json", Rules(premap="seed"))


def _driver(
    manifest_path: Path, out_dir: Path, topic: str
) -> tuple[EpisodeDriver, FakeWorld, dict]:
    poses = LCMTransport(f"/test_driver/{topic}/ground_truth", PoseStamped)
    goals = LCMTransport(f"/test_driver/{topic}/goal", PointStamped)
    reached = LCMTransport(f"/test_driver/{topic}/goal_reached", Bool)
    commands = LCMTransport(f"/test_driver/{topic}/cmd_vel", Twist)
    driver = EpisodeDriver(
        manifest=manifest_path,
        case_id="mined-s1-test",
        out_dir=out_dir,
        settle_s=0.2,
        seed_settle_s=0.2,
        goal_resend_s=0.2,
        goal_wait_s=3.0,
    )
    driver.ground_truth.transport = poses
    driver.goal.transport = goals
    driver.goal_reached.transport = reached
    driver.cmd_vel.transport = commands
    world = FakeWorld(poses, z=office(1).params["z0"])
    driver._world = world
    driver.tele_cmd_vel.subscribe(world.on_teleop)
    clicks: list[PointStamped] = []
    driver.clicked_point.subscribe(clicks.append)
    seeds: list[PointCloud2] = []
    driver.loaded_map.subscribe(seeds.append)
    return (
        driver,
        world,
        {
            "poses": poses,
            "goals": goals,
            "reached": reached,
            "commands": commands,
            "clicks": clicks,
            "seeds": seeds,
        },
    )


def _close(io: dict) -> None:
    for transport in (io["poses"], io["goals"], io["reached"], io["commands"]):
        transport.stop()


def _terminal(out_dir: Path, timeout_s: float) -> dict:
    deadline = time.time() + timeout_s
    while not (out_dir / TERMINAL_FILE).exists() and time.time() < deadline:
        time.sleep(0.05)
    return json.loads((out_dir / TERMINAL_FILE).read_text())


def test_driver_premaps_resets_sends_the_goal_and_ends_on_arrival(
    manifest_path: Path, tmp_path: Path
) -> None:
    driver, world, io = _driver(manifest_path, tmp_path, "arrive")
    world.start()
    driver.start()
    try:
        deadline = time.time() + 30.0
        while not io["clicks"] and time.time() < deadline:
            time.sleep(0.05)
        assert len(world.resets) == 2
        assert world.resets[0][:2] == (1.0, 1.0)
        assert np.linalg.norm(world.xy - (1.0, 1.0)) < 0.1
        click = io["clicks"][0]
        echo = PointStamped(click.x, click.y, click.z, ts=time.time(), frame_id="odom")
        io["goals"].publish(echo)
        time.sleep(0.5)
        io["reached"].publish(Bool(True))
        record = _terminal(tmp_path, 5.0)
    finally:
        driver.stop()
        world.stop()
        _close(io)
    assert record["reason"] == "arrived"
    assert record["premap_walked"] is True
    assert record["t0"] == pytest.approx(echo.ts)
    assert record["case_id"] == "mined-s1-test"


def test_driver_seeds_the_map_instead_of_walking(
    seeded_manifest_path: Path, tmp_path: Path
) -> None:
    driver, world, io = _driver(seeded_manifest_path, tmp_path, "seed")
    teleops: list[Twist] = []
    driver.tele_cmd_vel.subscribe(teleops.append)
    world.start()
    driver.start()
    try:
        deadline = time.time() + 30.0
        while not io["clicks"] and time.time() < deadline:
            time.sleep(0.05)
        assert len(io["seeds"]) == 1
        assert io["seeds"][0].frame_id == "odom"
        assert len(io["seeds"][0].points_f32()) > 10_000
        assert teleops == []
        assert len(world.resets) == 1
        click = io["clicks"][0]
        io["goals"].publish(
            PointStamped(click.x, click.y, click.z, ts=time.time(), frame_id="odom")
        )
        time.sleep(0.5)
        io["reached"].publish(Bool(True))
        record = _terminal(tmp_path, 5.0)
    finally:
        driver.stop()
        world.stop()
        _close(io)
    assert record["reason"] == "arrived"
    assert record["premap"] == "seed"
    assert record["premap_points"] > 10_000


def test_driver_reports_a_lost_goal(manifest_path: Path, tmp_path: Path) -> None:
    driver, world, io = _driver(manifest_path, tmp_path, "lost")
    world.start()
    driver.start()
    try:
        record = _terminal(tmp_path, 40.0)
    finally:
        driver.stop()
        world.stop()
        _close(io)
    assert record["reason"] == "goal_lost"
    assert len(io["clicks"]) >= 10
