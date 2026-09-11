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

"""Whole-map KronkNav inputs and a physical Twist base for the packing sim."""

from itertools import pairwise, product
import time
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.galaxea.r1pro.grasping_sim import VIRTUAL_BASE_JOINTS
from dimos.robot.galaxea.r1pro.grasping_transport import PlanarTransport
from dimos.robot.galaxea.r1pro.navigation_base import PlanarVelocityServo
from dimos.robot.galaxea.r1pro.packing_blueprint import R1ProPackingSim
from dimos.simulation.engines.mujoco_engine import MujocoEngine

NAV_BASE_ID = "r1pro_nav_base"
NAV_TASK = "tray_navigation"
NAV_FRAME = "carrying_footprint"


def pose_message(pose: list[float], ts: float | None = None) -> PoseStamped:
    return PoseStamped(
        position=Vector3(pose[0], pose[1], 0),
        orientation=Quaternion.from_euler(Vector3(0, 0, pose[2])),
        frame_id="world",
        ts=ts if ts is not None else time.time(),
    )


def carrying_offset(
    model: mujoco.MjModel, data: mujoco.MjData, bodies: set[int]
) -> NDArray[np.float64]:
    """Centre of collidable carrying bounds, expressed in the base frame."""
    pose = np.array([data.joint(n).qpos[0] for n in VIRTUAL_BASE_JOINTS])
    c, s = np.cos(pose[2]), np.sin(pose[2])
    rotation = np.array([[c, -s], [s, c]])
    points = []
    signs = np.array(list(product((-1, 1), repeat=3)))
    for gid in range(model.ngeom):
        if int(model.geom_bodyid[gid]) not in bodies:
            continue
        if not (model.geom_contype[gid] or model.geom_conaffinity[gid]):
            continue
        corners = model.geom_aabb[gid, :3] + signs * model.geom_aabb[gid, 3:]
        world = corners @ data.geom_xmat[gid].reshape(3, 3).T + data.geom_xpos[gid]
        points.append((world[:, :2] - pose[:2]) @ rotation)
    cloud = np.concatenate(points)
    return np.asarray((cloud.min(axis=0) + cloud.max(axis=0)) / 2, dtype=np.float64)


class R1ProNavigationSim(R1ProPackingSim):
    """Expose perfect sim odometry and the complete map as a simulated lidar."""

    base_cmd_vel: In[Twist]
    base_odom: Out[PoseStamped]
    navigation_tf: Out[TFMessage]
    global_map: Out[PointCloud2]
    goal: Out[PointStamped]
    planned_path: In[Path]
    surface_map: In[PointCloud2]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._servo: PlanarVelocityServo | None = None
        self._last_odom = float("-inf")
        self._offset = np.zeros(2)
        self._map_points = 0
        self._surface_points = 0
        self._request_time = float("inf")
        self._path: Path | None = None
        self._goal_pose: NDArray[np.float64] | None = None

    @rpc
    def start(self) -> None:
        self.register_disposable(Disposable(self.base_cmd_vel.subscribe(self._on_twist)))
        self.register_disposable(Disposable(self.planned_path.subscribe(self._on_path)))
        self.register_disposable(Disposable(self.surface_map.subscribe(self._on_surface)))
        super().start()

    def _on_surface(self, cloud: PointCloud2) -> None:
        self._surface_points = len(cloud)

    def _on_path(self, path: Path) -> None:
        if path.frame_id == "world" and path.ts >= self._request_time:
            self._path = path

    def _on_twist(self, twist: Twist) -> None:
        if self._engine is None:
            return
        with self._engine._lock:
            if self._servo is not None:
                self._servo.command_twist(
                    np.array([twist.linear.x, twist.linear.y, twist.angular.z]), time.monotonic()
                )

    def _publish_shm_and_lcm(self, engine: MujocoEngine) -> None:
        super()._publish_shm_and_lcm(engine)
        with engine._lock:
            pose = np.array([engine.data.joint(n).qpos[0] for n in VIRTUAL_BASE_JOINTS])
            if self._servo is None:
                self._servo = PlanarVelocityServo(pose)
            targets = self._servo.step(pose, float(engine.model.opt.timestep), time.monotonic())
            for name, target in zip(VIRTUAL_BASE_JOINTS, targets, strict=True):
                engine.data.actuator(name).ctrl[0] = target
            now = time.monotonic()
            if now - self._last_odom >= 0.02:
                self._last_odom = now
                odom = pose_message(pose.tolist())
                self.base_odom.publish(odom)
                self._publish_navigation_tf(odom)

    def _publish_navigation_tf(self, odom: PoseStamped) -> None:
        yaw = odom.orientation.euler[2]
        c, s = np.cos(yaw), np.sin(yaw)
        xy = (
            np.array([odom.position.x, odom.position.y])
            + np.array([[c, -s], [s, c]]) @ self._offset
        )
        self.navigation_tf.publish(
            TFMessage(
                Transform(
                    translation=Vector3(float(xy[0]), float(xy[1]), 0),
                    rotation=odom.orientation,
                    frame_id="world",
                    child_frame_id=NAV_FRAME,
                    ts=odom.ts,
                )
            )
        )

    @rpc
    def stop_navigation_base(self) -> None:
        """Hold the measured actuator pose immediately after cancelling the task."""
        if self._engine is not None:
            with self._engine._lock:
                if self._servo is not None:
                    self._servo.stop(
                        np.array([self._engine.data.joint(n).qpos[0] for n in VIRTUAL_BASE_JOINTS])
                    )

    @rpc
    def reset(self) -> bool:
        result = super().reset()
        self.stop_navigation_base()
        return result

    @rpc
    def task_state(self) -> dict[str, Any]:
        if self._engine is None:
            raise RuntimeError("Simulation has not started")
        with self._engine._lock:
            return {
                **super().task_state(),
                "base_pose": [
                    float(self._engine.data.joint(n).qpos[0]) for n in VIRTUAL_BASE_JOINTS
                ],
                "base_velocity": [
                    float(self._engine.data.joint(n).qvel[0]) for n in VIRTUAL_BASE_JOINTS
                ],
            }

    @rpc
    def publish_navigation_map(self, cloud_path: str) -> None:
        """Publish the complete static environment once all native inputs are live."""
        points = np.load(cloud_path, allow_pickle=False)
        self._map_points = len(points)
        self.global_map.publish(
            PointCloud2.from_numpy(points, frame_id="world", timestamp=time.time())
        )

    @rpc
    def plan_departure(self, yaw: float) -> list[list[float]]:
        """Find a clear backing/turn manoeuvre before constant-heading MLS travel."""
        if self._engine is None or not np.isfinite(yaw):
            raise ValueError("A live simulation and finite heading are required")
        with self._engine._lock:
            checker = PlanarTransport(
                self._engine.model, self._engine.data, cargo_bodies=self.cargo_bodies
            )
            self._offset = carrying_offset(
                self._engine.model, self._engine.data, checker.robot_bodies
            )
        for dx, dy in ((-0.2, 0.0), (-0.1, 0.0), (-0.3, 0.0), (-0.2, -0.1)):
            departure = checker.start + np.array([dx, dy, 0])
            turned = np.r_[departure[:2], yaw]
            if checker.clear_pose_segment(checker.start, departure) and checker.clear_pose_segment(
                departure, turned
            ):
                return [checker.start.tolist(), departure.tolist(), turned.tolist()]
        raise RuntimeError("No collision-free departure turn for the loaded tray")

    @rpc
    def request_navigation_path(self, target: list[float]) -> None:
        """Request KronkNav travel in the loaded-footprint frame, preserving heading."""
        pose = np.asarray(target, dtype=float)
        if pose.shape != (3,) or not np.isfinite(pose).all():
            raise ValueError("Navigation goal must be a finite planar pose")
        self._goal_pose = pose
        self._path = None
        self._request_time = time.time()
        c, s = np.cos(pose[2]), np.sin(pose[2])
        xy = pose[:2] + np.array([[c, -s], [s, c]]) @ self._offset
        self.goal.publish(PointStamped(x=float(xy[0]), y=float(xy[1]), z=0, frame_id="world"))

    @rpc
    def navigation_status(self) -> dict[str, Any]:
        """Return native planner evidence; partial or invalid paths never execute."""
        result: dict[str, Any] = {
            "map_points": self._map_points,
            "surface_points": self._surface_points,
            "footprint_offset": self._offset.tolist(),
            "path": None,
        }
        path, goal = self._path, self._goal_pose
        if path is None or goal is None:
            return result
        if len(path.poses) < 2:
            return {**result, "error": "KronkNav found no traversable route"}
        c, s = np.cos(goal[2]), np.sin(goal[2])
        offset = np.array([[c, -s], [s, c]]) @ self._offset
        poses = np.array(
            [[p.position.x - offset[0], p.position.y - offset[1], goal[2]] for p in path.poses]
        )
        if not np.isfinite(poses).all() or np.linalg.norm(poses[-1, :2] - goal[:2]) > 0.015:
            return {**result, "error": "KronkNav returned a partial route; destination not reached"}
        return {**result, "path": poses.tolist(), "planner_path_ts": path.ts}

    @rpc
    def validate_navigation_path(self, path: list[list[float]]) -> None:
        """Independently reject paths that collide with the complete held geometry."""
        poses = np.asarray(path, dtype=float)
        if poses.ndim != 2 or poses.shape[1] != 3 or len(poses) < 2 or not np.isfinite(poses).all():
            raise ValueError("Expected at least two finite planar poses")
        if self._engine is None:
            raise RuntimeError("Simulation has not started")
        with self._engine._lock:
            checker = PlanarTransport(
                self._engine.model, self._engine.data, cargo_bodies=self.cargo_bodies
            )
        for index, (a, b) in enumerate(pairwise(np.vstack([checker.start, poses]))):
            if not checker.clear_pose_segment(a, b):
                raise RuntimeError(f"Navigation segment {index} obstructs the loaded robot")
