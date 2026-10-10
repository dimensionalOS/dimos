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
from pathlib import Path
import threading
import time

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


class FakeWorld:
    """Stands where it was last reset and publishes that pose at 20 Hz."""

    def __init__(self, poses: LCMTransport, z: float) -> None:
        self.resets: list[tuple[float, float, float, float]] = []
        self.xy, self.z = (0.0, 0.0), z
        self.stop = threading.Event()
        self.thread = threading.Thread(target=lambda: self._publish(poses), daemon=True)

    def reset_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        self.resets.append((x, y, z, yaw))
        self.xy = (x, y)

    def _publish(self, poses: LCMTransport) -> None:
        while not self.stop.is_set():
            poses.publish(PoseStamped(*self.xy, self.z, ts=time.time(), frame_id="odom"))
            time.sleep(0.05)


def _until(done: object, timeout_s: float) -> None:
    deadline = time.time() + timeout_s
    while not done() and time.time() < deadline:  # type: ignore[operator]
        time.sleep(0.05)
    assert done()  # type: ignore[operator]


def test_driver_resets_seeds_sends_the_goal_and_ends_on_arrival(tmp_path: Path) -> None:
    scene = office(1)
    start = (scene.start[0], scene.start[1], 0.0)
    case = Case(
        family="office",
        seed=1,
        params={},
        start=start,
        goal=(start[0] + 3.0, start[1], float(scene.params["z0"])),
        tag="mined",
        id="mined-s1-test",
        split="dev",
        difficulty=Difficulty(0.5, 0, 1.0, 0),
        route_length=3.0,
        scene_digest=scene.digest(),
    )
    Manifest("test", Rules(), 0, None, False, [case], []).save(tmp_path / "suite.json")
    transports = {
        name: LCMTransport(f"/test_driver/{name}", kind)
        for name, kind in (
            ("ground_truth", PoseStamped),
            ("goal", PointStamped),
            ("goal_reached", Bool),
            ("cmd_vel", Twist),
        )
    }
    driver = EpisodeDriver(
        manifest=tmp_path / "suite.json",
        case_id=case.id,
        out_dir=tmp_path,
        settle_s=0.2,
        seed_settle_s=0.2,
        goal_resend_s=0.2,
        goal_wait_s=3.0,
    )
    for name, transport in transports.items():
        getattr(driver, name).transport = transport
    world = FakeWorld(transports["ground_truth"], z=scene.params["z0"])
    driver._world = world
    clicks: list[PointStamped] = []
    seeds: list[PointCloud2] = []
    routes: list[PathMsg] = []
    phases: list[str] = []
    driver.clicked_point.subscribe(clicks.append)
    driver.loaded_map.subscribe(seeds.append)
    driver.reference_path.subscribe(routes.append)
    driver.phase.subscribe(lambda msg: phases.append(msg.data))
    world.thread.start()
    driver.start()
    try:
        _until(lambda: clicks, 30.0)
        assert len(seeds) == 1 and seeds[0].frame_id == "odom"
        assert len(seeds[0].points_f32()) > 10_000
        assert world.resets[0][:2] == (1.0, 1.0) and len(world.resets) == 1
        echo = PointStamped(clicks[0].x, clicks[0].y, clicks[0].z, ts=time.time(), frame_id="odom")
        transports["goal"].publish(echo)
        _until(lambda: driver._echo is not None, 5.0)
        transports["goal_reached"].publish(Bool(True))
        _until(lambda: (tmp_path / TERMINAL_FILE).exists(), 5.0)
    finally:
        driver.stop()
        world.stop.set()
        for transport in transports.values():
            transport.stop()
    record = json.loads((tmp_path / TERMINAL_FILE).read_text())
    assert record["reason"] == "arrived" and record["case_id"] == case.id
    assert record["t0"] == pytest.approx(echo.ts)
    assert record["premap_points"] == len(seeds[0].points_f32())
    assert len(routes) == 1 and routes[0].poses[-1].position.x == pytest.approx(4.0, abs=0.05)
    assert phases == ["reset", "premap", "goal", "navigate", "arrived"]
