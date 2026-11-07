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

"""The simulated bench as the trial loop sees it: seeded resets and snapshots
of the tracked bodies, built from the simulator's pose and joint streams and
the obstacle map."""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
import math
import random
import threading
import time
from typing import Protocol

from pydantic import BaseModel

from dimos.autoresearch.judge import BodyPose, Snapshot
from dimos.autoresearch.program import RunStoppedError, program_box
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.sdk import Arm
from dimos.mapping.pointclouds.box_filter_spec import PointCloudBoxFilterSpec
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.simulation.engines.sim_scene_spec import BodyPlacement, SimSceneSpec
from dimos.spec.utils import Spec

# The arm is at rest when every joint stayed within REST_TOLERANCE (rad or m)
# over the last REST_WINDOW_S seconds of samples.
REST_WINDOW_S = 0.5
REST_TOLERANCE = 0.002
# How long to wait for the first message after a reset, and for the arm to come to rest.
MESSAGE_TIMEOUT_S = 10.0
REST_TIMEOUT_S = 30.0


class TrajectoryControlSpec(Spec, Protocol):
    """The controller's trajectory entry point, as the ControlCoordinator has it."""

    def execute_trajectory(self, trajectory: JointTrajectory) -> TrajectoryExecutionResult: ...


class PlacementBounds(BaseModel):
    """Where the seeded object may start: ``x`` and ``y`` ranges on the bench and
    its resting height ``z``, metres in the world frame, and a ``yaw`` range in
    radians. The defaults keep the Metal bench's cube in the wrist camera's
    view and clear of the cylinder and the bowl."""

    x: tuple[float, float] = (0.18, 0.30)
    y: tuple[float, float] = (0.08, 0.20)
    z: float = 0.02
    yaw: tuple[float, float] = (-math.pi / 4, math.pi / 4)

    def sample(self, seed: int) -> BodyPlacement:
        """The placement for ``seed``; the same seed always gives the same place."""
        rng = random.Random(seed)
        return (rng.uniform(*self.x), rng.uniform(*self.y), self.z, rng.uniform(*self.yaw))


class MujocoWorld:
    """``reset(seed)`` puts the simulation back with ``body`` where ``bounds``
    says for that seed, drives the arm to ``home`` through the controller, waits
    for it to hold still and for an obstacle map of the new bench, then puts
    back any object the last program cleared from that map and waits for poses
    taken after the reset; ``snapshot()`` stops the arm and, once it holds still,
    reports every tracked body's pose. Feed ``on_body_poses``, ``on_joint_state``
    and ``on_voxel_map`` from the stack's streams. Every wait gives up with
    ``RunStoppedError`` once ``stop`` is set."""

    def __init__(
        self,
        sim: SimSceneSpec,
        arm: Arm,
        control: TrajectoryControlSpec,
        box_filter: PointCloudBoxFilterSpec,
        home: JointState,
        body: str,
        bounds: PlacementBounds,
        stop: threading.Event,
    ) -> None:
        self._sim = sim
        self._arm = arm
        self._control = control
        self._box_filter = box_filter
        self._home = home
        self._body = body
        self._bounds = bounds
        self._stop = stop
        self._changed = threading.Condition()
        self._poses: dict[str, BodyPose] = {}
        self._pose_counts: dict[str, int] = {}
        self._joints: deque[tuple[float, list[float]]] = deque(maxlen=1000)
        self._joint_count = 0
        self._map_count = 0

    def on_body_poses(self, msg: TFMessage) -> None:
        with self._changed:
            for transform in msg.transforms:
                t, q = transform.translation, transform.rotation
                self._poses[transform.child_frame_id] = BodyPose(
                    position=(t.x, t.y, t.z), orientation=(q.x, q.y, q.z, q.w)
                )
                self._pose_counts[transform.child_frame_id] = (
                    self._pose_counts.get(transform.child_frame_id, 0) + 1
                )
            self._changed.notify_all()

    def on_joint_state(self, msg: JointState) -> None:
        with self._changed:
            self._joints.append((time.monotonic(), [float(value) for value in msg.position]))
            self._joint_count += 1
            self._changed.notify_all()

    def on_voxel_map(self, msg: PointCloud2) -> None:
        with self._changed:
            self._map_count += 1
            self._changed.notify_all()

    def reset(self, seed: int) -> None:
        self._arm.cancel()
        with self._changed:
            pose_counts = dict(self._pose_counts)
            joint_count = self._joint_count
        if not self._sim.reset(placements={self._body: self._bounds.sample(seed)}):
            raise RuntimeError("the simulation did not apply the reset")
        # The reset puts the arm at home, but the controller still drives it
        # toward its last target until it is given home as the target.
        self._drive_home()
        self._wait(
            lambda: self._joint_count > joint_count,
            "a joint state after the reset",
            MESSAGE_TIMEOUT_S,
        )
        self.wait_for_rest()
        self._wait_for_map()
        # With the arm away from the objects, every object is an obstacle again.
        with self._changed:
            bodies = sorted(set(self._poses) | {self._body})
        for body in bodies:
            self._box_filter.clear_box(program_box(body))
        self._wait_for_poses(pose_counts)

    def home(self) -> None:
        """Stop any motion, drive the arm to its home joints with the gripper open and wait until it holds still."""
        self._wait(lambda: self._joint_count > 0, "the first joint state", MESSAGE_TIMEOUT_S)
        self._arm.cancel()
        self._drive_home()
        self.wait_for_rest()

    def _drive_home(self) -> None:
        """Give the controller the home joints as its target, with no plan, and open the gripper."""
        # One point: the controller ramps every joint from wherever it is to the
        # target at its velocity limit and checks no start state, so this works
        # from any pose, even one the planner calls a collision.
        trajectory = JointTrajectory(
            joint_names=list(self._home.name),
            points=[TrajectoryPoint(positions=[float(value) for value in self._home.position])],
        )
        result = self._control.execute_trajectory(trajectory)
        if result.status is not TrajectoryExecutionStatus.ACCEPTED:
            raise RuntimeError(
                f"the controller refused the home trajectory: {result.message or result.status.name}"
            )
        self._arm.open_gripper()

    def wait_for_rest(self) -> None:
        self._wait(self._at_rest, "the arm to come to rest", REST_TIMEOUT_S)

    def snapshot(self) -> Snapshot:
        with self._changed:
            pose_counts = dict(self._pose_counts)
        self._arm.cancel()
        self.wait_for_rest()
        self._wait_for_poses(pose_counts)
        with self._changed:
            return Snapshot(bodies=dict(self._poses))

    def _wait_for_poses(self, counts: dict[str, int]) -> None:
        """Wait for every tracked body, and the seeded one, to report a pose newer than ``counts``."""
        tracked = set(counts) | {self._body}
        self._wait(
            lambda: all(self._pose_counts.get(name, 0) > counts.get(name, 0) for name in tracked),
            "fresh body poses",
            MESSAGE_TIMEOUT_S,
        )

    def _wait_for_map(self) -> None:
        """Wait for an obstacle map published from now on. The arm has held still
        for REST_WINDOW_S since the reset, longer than the map lags the camera,
        so that map shows the bench as reset, not the objects where they were."""
        with self._changed:
            count = self._map_count
        self._wait(
            lambda: self._map_count > count, "an obstacle map of the reset bench", MESSAGE_TIMEOUT_S
        )

    def _at_rest(self) -> bool:
        now = time.monotonic()
        recent = [(t, positions) for t, positions in self._joints if t >= now - REST_WINDOW_S]
        if len(recent) < 2 or recent[-1][0] - recent[0][0] < 0.8 * REST_WINDOW_S:
            return False
        newest = recent[-1][1]
        return all(
            abs(value - latest) < REST_TOLERANCE
            for _, positions in recent
            for value, latest in zip(positions, newest, strict=True)
        )

    def _wait(self, ready: Callable[[], bool], what: str, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        with self._changed:
            while not ready():
                if self._stop.is_set():
                    raise RunStoppedError(f"the run was stopped while waiting for {what}")
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(f"waited {timeout_s} s for {what}")
                self._changed.wait(min(remaining, 0.1))
