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

from collections.abc import Callable, Iterator
from dataclasses import dataclass
from itertools import count
import json
from pathlib import Path
import threading
import time

import numpy as np
import pytest

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path as PathMsg
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Bool import Bool
from dimos.navigation.bench.driver import TERMINAL_FILE, EpisodeDriver
from dimos.navigation.bench.ground_truth import Difficulty
from dimos.navigation.bench.suite import Case, Manifest, Rules
from dimos.simulation.scenes.procedural import office

CASE_ID = "mined-s1-test"
TOPICS = count()


class FakeWorld:
    """Stands where it was last reset and publishes that pose at 20 Hz."""

    def __init__(self, poses: LCMTransport, z: float) -> None:
        self.resets: list[tuple[float, float, float, float]] = []
        self.xy = np.zeros(2)
        self.z = z
        self._poses = poses
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def reset_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        self.resets.append((x, y, z, yaw))
        self.xy = np.array([x, y])

    def _run(self) -> None:
        while not self._stop.is_set():
            pose = PoseStamped(
                float(self.xy[0]), float(self.xy[1]), self.z, ts=time.time(), frame_id="odom"
            )
            self._poses.publish(pose)
            time.sleep(0.05)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)


@pytest.fixture(scope="module")
def manifest_path(tmp_path_factory: pytest.TempPathFactory) -> Path:
    scene = office(1)
    start = (scene.start[0], scene.start[1], 0.0)
    case = Case(
        family="office",
        seed=1,
        params={},
        start=start,
        goal=(start[0] + 3.0, start[1], float(scene.params["z0"])),
        tag="mined",
        id=CASE_ID,
        split="dev",
        difficulty=Difficulty(0.5, 0, 1.0, 0),
        route_length=3.0,
        scene_digest=scene.digest(),
    )
    path = tmp_path_factory.mktemp("suite") / "suite.json"
    Manifest("test", Rules(), 0, None, False, [case], []).save(path)
    return path


@dataclass
class Harness:
    driver: EpisodeDriver
    world: FakeWorld
    goals: LCMTransport
    reached: LCMTransport
    clicks: list[PointStamped]
    seeds: list[PointCloud2]
    routes: list[PathMsg]
    phases: list[str]


@pytest.fixture
def harness(manifest_path: Path, tmp_path: Path) -> Iterator[Harness]:
    topic = next(TOPICS)
    transports = {
        name: LCMTransport(f"/test_driver/{topic}/{name}", kind)
        for name, kind in (
            ("ground_truth", PoseStamped),
            ("goal", PointStamped),
            ("goal_reached", Bool),
            ("cmd_vel", Twist),
        )
    }
    driver = EpisodeDriver(
        manifest=manifest_path,
        case_id=CASE_ID,
        out_dir=tmp_path,
        settle_s=0.2,
        seed_settle_s=0.2,
        goal_resend_s=0.2,
        goal_wait_s=3.0,
    )
    for name, transport in transports.items():
        getattr(driver, name).transport = transport
    world = FakeWorld(transports["ground_truth"], z=office(1).params["z0"])
    driver._world = world
    harness = Harness(driver, world, transports["goal"], transports["goal_reached"], [], [], [], [])
    driver.clicked_point.subscribe(harness.clicks.append)
    driver.loaded_map.subscribe(harness.seeds.append)
    driver.reference_path.subscribe(harness.routes.append)
    driver.phase.subscribe(lambda msg: harness.phases.append(msg.data))
    world.start()
    driver.start()
    try:
        yield harness
    finally:
        driver.stop()
        world.stop()
        for transport in transports.values():
            transport.stop()


def _terminal(out_dir: Path, timeout_s: float) -> dict[str, object]:
    deadline = time.time() + timeout_s
    while not (out_dir / TERMINAL_FILE).exists() and time.time() < deadline:
        time.sleep(0.05)
    record: dict[str, object] = json.loads((out_dir / TERMINAL_FILE).read_text())
    return record


def _until(done: Callable[[], bool], timeout_s: float) -> None:
    deadline = time.time() + timeout_s
    while not done() and time.time() < deadline:
        time.sleep(0.05)
    assert done()


def test_driver_resets_seeds_sends_the_goal_and_ends_on_arrival(
    harness: Harness, tmp_path: Path
) -> None:
    _until(lambda: bool(harness.clicks), 30.0)
    assert len(harness.seeds) == 1
    assert harness.seeds[0].frame_id == "odom"
    assert len(harness.seeds[0].points_f32()) > 10_000
    assert len(harness.world.resets) == 1
    assert harness.world.resets[0][:2] == (1.0, 1.0)
    click = harness.clicks[0]
    echo = PointStamped(click.x, click.y, click.z, ts=time.time(), frame_id="odom")
    harness.goals.publish(echo)
    _until(lambda: harness.driver._echo is not None, 5.0)
    harness.reached.publish(Bool(True))
    record = _terminal(tmp_path, 5.0)
    assert record["reason"] == "arrived"
    assert record["t0"] == pytest.approx(echo.ts)
    assert record["case_id"] == CASE_ID
    assert record["premap_points"] == len(harness.seeds[0].points_f32())
    assert len(harness.routes) == 1 and len(harness.routes[0].poses) > 10
    assert harness.routes[0].poses[-1].position.x == pytest.approx(4.0, abs=0.05)
    assert harness.phases == ["reset", "premap", "goal", "navigate", "arrived"]
