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


"""The physics module: one MuJoCo world, several robots, and nothing that decides what they do.

`SimWorld` is the single-robot shape, where the scene, the robot and its actor all live in one
module. This is the other shape: the world runs here and each robot's actor runs in its own
module, so the two talk over the bus like a driver and a controller do on a real robot.

Per control tick the world publishes what every robot can feel -- `sim_odom` (its base pose and
twist, ground truth standing in for odometry) and `sim_state` (every joint, under prefixed
names) -- and then waits for each robot's `joint_target` stamped with that same sim time before
stepping. That is lockstep: correctness is in sim time, and a controller that cannot keep up
makes the wall clock slower rather than the physics wrong. A robot that misses the deadline has
its last target held, which is what a real actuator does, and the miss is counted and logged.
"""

from __future__ import annotations

from collections import defaultdict
from threading import Condition, Event, Thread
import time
from typing import Any, ClassVar

import mujoco
import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.files.File import File
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.simulation.scenes.procedural import Scene, generate
from dimos.simulation.sim_world.body import Rig
from dimos.simulation.sim_world.scene_data import scene_files
from dimos.simulation.sim_world.shared import Occupant, Placement, SharedWorld
from dimos.simulation.sim_world.world import SimWorldConfig, open_viewer
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The frame a robot's base pose is reported in, under its prefix: `a_base`, `b_base`.
BASE_SUFFIX = "base"
Spot = tuple[float, float, float]


class MujocoSimConfig(SimWorldConfig):
    # prefix -> where that robot stands, as (x, y, yaw). The prefix is what every one of its
    # names takes in the shared model, and what its controller answers under.
    robots: dict[str, Spot] = Field(default_factory=dict)
    # Wall seconds to wait for one tick's targets before holding what the actuators have. The
    # sim is in lockstep, so this is a stuck-controller timeout and not a pacing knob: raise
    # real_time_factor to give a slow controller more wall clock per tick.
    control_deadline: float = 1.0


class MujocoSim(Module):
    """A scene with several robots in it, stepped in lockstep with their controllers."""

    config: MujocoSimConfig
    rig: ClassVar[Rig]

    # every robot's targets, routed by the prefix on the joint names
    joint_target: In[JointState]
    render_file_request: In[String]

    sim_odom: Out[Odometry]
    sim_state: Out[JointState]
    sim_lidar: Out[PointCloud2]
    file_info: Out[File]

    _thread: Thread | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = Event()
        self._targets: dict[str, tuple[float, np.ndarray]] = {}
        self._arrived = Condition()
        self._misses: defaultdict[str, int] = defaultdict(int)

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.joint_target.subscribe(self._on_target)))
        self.register_disposable(
            Disposable(self.render_file_request.subscribe(self._on_render_file_request))
        )
        self._world = self._load()
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        # Guarded because a world that never started still has to be stoppable: the module's
        # own threads are up from construction, so anything holding one has to let it go.
        if (stopping := getattr(self, "_stop_event", None)) is not None:
            stopping.set()
        with self._arrived:
            self._arrived.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    @rpc
    def misses(self) -> dict[str, int]:
        """How many ticks each robot's controller was late for. Zero is the healthy answer."""
        return dict(self._misses)

    def _load(self) -> SharedWorld:
        if not self.config.robots:
            raise ValueError(
                "a MujocoSim needs at least one robot: set robots={prefix: (x, y, yaw)}"
            )
        scene: Scene = generate(self.config.family, self.config.seed, **self.config.scene_params)
        placements = [
            Placement(prefix, self.rig, spot) for prefix, spot in self.config.robots.items()
        ]
        world = SharedWorld(scene, self.config.seed, placements)
        world.reset()
        for prefix in world.occupants:
            blocked = [c for c in world.contacts(prefix) if c.kind != "floor"]
            if blocked:
                raise RuntimeError(f"{prefix} starts inside scene geometry: {blocked}")
        logger.info("Shared world ready", scene=scene.name, robots=sorted(world.occupants))
        return world

    def _on_target(self, msg: JointState) -> None:
        """One robot's targets for one tick, recognised by the prefix on its joint names."""
        if not msg.name:
            return
        prefix = self._prefix_of(msg.name[0])
        if prefix is None:
            logger.warning("joint_target for no robot here", first=msg.name[0])
            return
        with self._arrived:
            self._targets[prefix] = (msg.ts, np.asarray(msg.position, dtype=np.float64))
            self._arrived.notify_all()

    def _prefix_of(self, joint: str) -> str | None:
        for prefix in self._world.occupants:
            if joint.startswith(prefix):
                return prefix
        return None

    def _await_targets(self, stamp: float) -> None:
        """Wait until every robot has answered for this tick, or the deadline passes."""
        deadline = time.monotonic() + self.config.control_deadline
        with self._arrived:
            while not self._stop_event.is_set():
                late = [
                    p
                    for p in self._world.occupants
                    if self._targets.get(p, (-1.0, None))[0] != stamp
                ]
                if not late:
                    return
                if not self._arrived.wait(timeout=max(0.0, deadline - time.monotonic())):
                    for prefix in late:
                        self._misses[prefix] += 1
                    logger.warning(
                        "controllers missed a tick; holding the last targets",
                        late=sorted(late),
                        sim_t=round(stamp, 3),
                    )
                    return

    def _apply_targets(self, stamp: float) -> None:
        with self._arrived:
            for prefix, (ts, target) in self._targets.items():
                if ts == stamp:
                    self._world.apply(prefix, target)

    def _run(self) -> None:
        PointCloud2.from_numpy(np.zeros((0, 3), np.float32), frame_id="sim")
        viewer = (
            open_viewer(self._world.model, self._world.data, self._world.data.xpos[1].copy())
            if self.config.mujoco_viewer
            else None
        )
        try:
            self._simulate(viewer)
        except Exception:
            logger.exception("Shared sim thread failed")
        finally:
            if viewer is not None:
                viewer.close()

    def _simulate(self, viewer: Any) -> None:
        world = self._world
        t0 = time.time()
        while not self._stop_event.is_set():
            stamp = t0 + world.t / self.config.real_time_factor
            self._publish_state(stamp)
            self._await_targets(world.t)
            self._apply_targets(world.t)
            frames = world.tick()
            if viewer is not None:
                viewer.sync()
            for prefix, frame in frames.items():
                if frame is None:
                    continue
                child = f"{prefix}{world.occupants[prefix].rig.odom_child_frame}"
                self.sim_lidar.publish(
                    PointCloud2.from_numpy(frame.points, frame_id=child, timestamp=stamp)
                )
            delay = (t0 + world.t / self.config.real_time_factor) - time.time()
            if delay > 0:
                self._stop_event.wait(delay)

    def _publish_state(self, stamp: float) -> None:
        """What every robot can feel this tick, stamped with sim time so a reply can name it."""
        world = self._world
        names: list[str] = []
        positions: list[float] = []
        velocities: list[float] = []
        for prefix, occupant in world.occupants.items():
            self.sim_odom.publish(base_odometry(occupant, prefix, world.t))
            joint_positions, joint_velocities = occupant.chassis.joint_state()
            names.extend(occupant.chassis.joint_names)
            positions.extend(joint_positions.tolist())
            velocities.extend(joint_velocities.tolist())
        self.sim_state.publish(
            JointState(
                ts=world.t,
                frame_id="sim",
                name=names,
                position=positions,
                velocity=velocities,
            )
        )
        assert stamp  # the wall-clock stamp paces the loop; the messages carry sim time

    def _on_render_file_request(self, msg: String) -> None:
        world = getattr(self, "_world", None)
        if world is None:
            return
        data = self._scene_files(world).get(msg.data)
        if data is not None:
            self.file_info.publish(File(msg.data, data))

    def _scene_files(self, world: SharedWorld) -> dict[str, bytes]:
        scene_id = f"{world.scene.name}-{id(world)}"
        if getattr(self, "_files_for", None) != scene_id:
            # Every robot is drawn from its own description, so none of them belongs in the
            # scene index. scene_files leaves out one root's subtree, so the robots are left
            # out by their prefix instead.
            self._files = scene_files(
                world.model, world.data, scene_id, self.rig.world_frame, _no_robot(world)
            )
            self._files_for = scene_id
        return self._files


def base_odometry(occupant: Occupant, prefix: str, t: float) -> Odometry:
    """One robot's base pose and twist as the world reports them, stamped with sim time.

    Ground truth standing in for odometry, in the conventions odometry uses: the pose in the
    world frame, the linear twist in the world frame and the angular twist in the base frame.
    A controller turns it back into what its actor was trained on.
    """
    position, rotation = occupant.chassis.base_pose()
    linear, angular = occupant.chassis.base_velocity()
    q = Rotation.from_matrix(rotation).as_quat()
    return Odometry(
        ts=t,
        frame_id=occupant.rig.world_frame,
        child_frame_id=f"{prefix}{BASE_SUFFIX}",
        pose=Pose(*map(float, position), *map(float, q)),
        twist=Twist(Vector3(linear.tolist()), Vector3(angular.tolist())),
    )


def _no_robot(world: SharedWorld) -> int:
    """A body id whose subtree scene_files drops. With several robots there is no single one."""
    return mujoco.mj_name2id(
        world.model,
        mujoco.mjtObj.mjOBJ_BODY,
        f"{next(iter(world.occupants))}{world.placements[0].rig.base_body}",
    )
