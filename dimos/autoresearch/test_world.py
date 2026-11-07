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
from dimos.autoresearch.program import RunStoppedError, program_box
from dimos.autoresearch.world import MujocoWorld, PlacementBounds
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.simulation.engines.sim_scene_spec import BodyPlacement

HOME = JointState(name=["j"], position=[0.5])


class Bench:
    """The sim, the arm, the controller and the box filter as the world drives
    them; every call lands in ``events`` in the order it was made."""

    def __init__(self) -> None:
        self.events: list[str] = []
        self.placements: list[Mapping[str, BodyPlacement]] = []
        self.trajectories: list[tuple[list[str], list[float]]] = []

    def reset(self, placements: Mapping[str, BodyPlacement] | None = None) -> bool:
        self.placements.append(dict(placements or {}))
        self.events.append("reset")
        return True

    def open_gripper(self) -> None:
        self.events.append("open")

    def cancel(self) -> None:
        self.events.append("cancel")

    def execute_trajectory(self, trajectory: JointTrajectory) -> TrajectoryExecutionResult:
        self.trajectories.append((trajectory.joint_names, trajectory.points[-1].positions))
        self.events.append("home")
        return TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)

    def set_box(self, name: str, pose: object, size: object, frame_id: str | None = None) -> bool:
        return True

    def clear_box(self, name: str) -> bool:
        self.events.append(f"clear {name}")
        return True


class Feeder:
    """Publishes a still arm, two body poses and a map at 50 Hz through the world's handlers."""

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
            self.world.on_voxel_map(PointCloud2())
            time.sleep(0.02)


@pytest.fixture
def world() -> Iterator[tuple[MujocoWorld, Bench, threading.Event]]:
    bench, stop = Bench(), threading.Event()
    world = MujocoWorld(bench, bench, bench, bench, HOME, "cube", PlacementBounds(), stop)
    feeder = Feeder(world)
    feeder.thread.start()
    yield world, bench, stop
    feeder.stop.set()
    feeder.thread.join()


def test_snapshot_holds_every_body_pose_from_the_tf_stream(
    world: tuple[MujocoWorld, Bench, threading.Event],
) -> None:
    scene, bench, _ = world
    snapshot = scene.snapshot()
    assert snapshot.bodies == {
        "cube": BodyPose(position=(0.2, 0.1, 0.02), orientation=(0.0, 0.0, 0.0, 1.0)),
        "bowl": BodyPose(position=(0.36, 0.0, 0.0), orientation=(0.0, 0.0, 1.0, 0.0)),
    }
    assert bench.events == ["cancel"]


def test_reset_places_the_body_then_homes_the_arm_without_the_planner_before_restoring_the_map(
    world: tuple[MujocoWorld, Bench, threading.Event],
) -> None:
    scene, bench, _ = world
    scene.reset(seed=3)
    assert bench.placements == [{"cube": PlacementBounds().sample(3)}]
    assert bench.trajectories == [(["j"], [0.5])]
    assert bench.events == [
        "cancel",
        "reset",
        "home",
        "open",
        f"clear {program_box('bowl')}",
        f"clear {program_box('cube')}",
    ]


def test_a_stopped_world_gives_up_its_waits(
    world: tuple[MujocoWorld, Bench, threading.Event],
) -> None:
    scene, _, stop = world
    stop.set()
    with pytest.raises(RunStoppedError):
        scene.snapshot()
