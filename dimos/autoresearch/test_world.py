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

from collections.abc import Iterator, Mapping
import threading
import time

import pytest

from dimos.autoresearch.judge import BodyPose
from dimos.autoresearch.program import RunStoppedError
from dimos.autoresearch.world import MujocoWorld, PlacementBounds
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.simulation.engines.sim_scene_spec import BodyPlacement


class FakeSim:
    def __init__(self) -> None:
        self.placements: list[Mapping[str, BodyPlacement]] = []

    def reset(self, placements: Mapping[str, BodyPlacement] | None = None) -> bool:
        self.placements.append(dict(placements or {}))
        return True


class FakeArm:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def home(self) -> None:
        self.calls.append("home")

    def open_gripper(self) -> None:
        self.calls.append("open")

    def cancel(self) -> None:
        self.calls.append("cancel")


class Feeder:
    """Publishes a still arm and two body poses at 50 Hz through the world's handlers."""

    def __init__(self, world: MujocoWorld) -> None:
        self.world = world
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        while not self.stop.is_set():
            self.world.on_joint_state(JointState(name=["j"], position=[0.5]))
            self.world.on_body_poses(
                TFMessage(
                    Transform(Vector3(0.2, 0.1, 0.02), Quaternion(), "world", "cube"),
                    Transform(
                        Vector3(0.36, 0.0, 0.0), Quaternion(0.0, 0.0, 1.0, 0.0), "world", "bowl"
                    ),
                )
            )
            time.sleep(0.02)


@pytest.fixture
def world() -> Iterator[tuple[MujocoWorld, FakeSim, FakeArm, threading.Event]]:
    sim, arm, stop = FakeSim(), FakeArm(), threading.Event()
    world = MujocoWorld(sim, arm, "cube", PlacementBounds(), stop)
    feeder = Feeder(world)
    feeder.thread.start()
    yield world, sim, arm, stop
    feeder.stop.set()
    feeder.thread.join()


def test_snapshot_holds_every_body_pose_from_the_tf_stream(
    world: tuple[MujocoWorld, FakeSim, FakeArm, threading.Event],
) -> None:
    scene, sim, arm, _ = world
    snapshot = scene.snapshot()
    assert snapshot.bodies == {
        "cube": BodyPose(position=(0.2, 0.1, 0.02), orientation=(0.0, 0.0, 0.0, 1.0)),
        "bowl": BodyPose(position=(0.36, 0.0, 0.0), orientation=(0.0, 0.0, 1.0, 0.0)),
    }
    assert arm.calls == ["cancel"]

    scene.reset(seed=3)
    assert sim.placements == [{"cube": PlacementBounds().sample(3)}]
    assert arm.calls == ["cancel", "cancel", "home", "open"]


def test_a_stopped_world_gives_up_its_waits(
    world: tuple[MujocoWorld, FakeSim, FakeArm, threading.Event],
) -> None:
    scene, _, _, stop = world
    stop.set()
    with pytest.raises(RunStoppedError):
        scene.snapshot()
