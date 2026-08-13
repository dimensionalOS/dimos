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

"""Referee worlds in MuJoCo: the bridge between planner and simulation.

A planner :class:`Scenario` describes truth as oriented boxes. This module
rebuilds that truth as static collision geoms in the matched Go2 scene, spawns
the robot at the scenario start, and hands the planner the same analytic
surface cloud the referee feeds it — perception stays perfect on purpose, so
episode scores measure execution, not mapping.
"""

from __future__ import annotations

import math
from pathlib import Path as FilePath

import mujoco
import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path, Path as RefereePath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2 as RefereeCloud
from dimos.navigation.motion.embodiment import Embodiment
from dimos.navigation.motion.scenarios import Scenario
from dimos.navigation.motion.simulation import model as go2_model
from dimos.navigation.motion.simulation.evaluate import apply_physics

# Planner's view of the world, identical to the referee (sim.py CLOUD_STEP).
CLOUD_STEP = 0.05

WALL_PREFIX = "wall_"
WALL_RGBA = (0.55, 0.55, 0.58, 1.0)
GOAL_RGBA = (0.2, 1.0, 0.2, 0.4)


def _yaw_quat_wxyz(yaw: float) -> tuple[float, float, float, float]:
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def load_world(
    sc: Scenario,
    menagerie: FilePath | None = None,
    physics: dict[str, float] | None = None,
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """The go2 scene plus one static box geom per scenario obstacle.

    Walls collide with the robot (that is the gate the judge reads); the goal
    marker is a contact-free translucent cylinder. ``physics`` patches the
    compiled model via :func:`apply_physics` — pass ``FITTED_PHYSICS`` for the
    matched sim.
    """
    spec = mujoco.MjSpec.from_file(str(go2_model.scene_path(menagerie)))
    for i, b in enumerate(sc.boxes):
        body = spec.worldbody.add_body(name=f"{WALL_PREFIX}{i}")
        body.pos = (b.cx, b.cy, b.height / 2.0)
        body.quat = _yaw_quat_wxyz(b.yaw)
        geom = body.add_geom()
        geom.type = mujoco.mjtGeom.mjGEOM_BOX
        geom.name = f"{WALL_PREFIX}{i}"
        geom.size = (b.sx / 2.0, b.sy / 2.0, b.height / 2.0)
        geom.rgba = WALL_RGBA
    goal = spec.worldbody.add_body(name="goal_marker")
    goal.pos = (sc.goal[0], sc.goal[1], 0.005)
    marker = goal.add_geom()
    marker.type = mujoco.mjtGeom.mjGEOM_CYLINDER
    marker.size = (0.12, 0.005, 0.0)
    marker.rgba = GOAL_RGBA
    marker.contype = 0
    marker.conaffinity = 0
    model = spec.compile()
    if physics:
        apply_physics(model, physics)
    return model, mujoco.MjData(model)


def reset_to_start(
    model: mujoco.MjModel, data: mujoco.MjData, sc: Scenario, default_pose: np.ndarray
) -> None:
    """Keyframe stand at the scenario's start pose (position + yaw)."""
    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")  # type: ignore[attr-defined]
    if kid >= 0:
        mujoco.mj_resetDataKeyframe(model, data, kid)
    data.qpos[7:19] = default_pose
    data.qpos[0] = sc.start[0]
    data.qpos[1] = sc.start[1]
    data.qpos[3:7] = _yaw_quat_wxyz(sc.start[2])
    mujoco.mj_forward(model, data)


def wall_geom_ids(model: mujoco.MjModel) -> set[int]:
    return {
        gid
        for gid in range(model.ngeom)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or "").startswith(WALL_PREFIX)
    }


def wall_contact(model: mujoco.MjModel, data: mujoco.MjData, walls: set[int]) -> bool:
    """True when any active contact touches a wall geom.

    Walls and floor are both static (welded to the world), so MuJoCo never
    generates wall-floor contacts — a wall in a contact pair means the robot.
    """
    for i in range(data.ncon):
        con = data.contact[i]
        if con.geom1 in walls or con.geom2 in walls:
            return True
    return False


def planner_cloud(sc: Scenario) -> RefereeCloud:
    """The referee's own planner view: box surfaces sampled at CLOUD_STEP."""
    pts = (
        np.concatenate([b.surface(CLOUD_STEP) for b in sc.boxes]) if sc.boxes else np.empty((0, 3))
    )
    return RefereeCloud.from_numpy(pts.astype(np.float32), frame_id="world")


def path_clearance(ref: RefereePath, obstacles: np.ndarray, emb: Embodiment) -> np.ndarray:
    """Per-waypoint room hint (m), derived from the planner's own obstacles.

    Nearest obstacle minus the half-width — a speed HINT for the controller,
    not a safety contract: it is what a path annotator on the robot would
    compute from the local map, and the judge measures truth separately.
    Nothing to hit = infinite room.

    `obstacles` is the obstacle model's hard set (motion/obstacles.py), every
    row of which is an obstacle; z rides along and is ignored. There is no
    second z rule here, because the planner is priced against the SAME set and
    a band of our own would price a different world than the one planned.
    """
    xy = np.array([[p.position.x, p.position.y] for p in ref.poses]).reshape(-1, 2)
    band = np.asarray(obstacles, dtype=np.float32).reshape(-1, 3)[:, :2]
    if not len(band) or not len(xy):
        return np.full(len(xy), np.inf)
    from scipy.spatial import cKDTree

    d, _ = cKDTree(band).query(xy)
    return np.asarray(d, dtype=float) - emb.width / 2.0


def to_nav_path(ref: RefereePath, ts: float = 0.0, frame_id: str = "world") -> Path:
    """Referee path -> dimos nav_msgs Path (the type the controller consumes)."""
    poses = [
        PoseStamped(
            ts=ts,
            frame_id=frame_id,
            position=Vector3(p.position.x, p.position.y, p.position.z),
            orientation=Quaternion(
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
            ),
        )
        for p in ref.poses
    ]
    return Path(ts=ts, frame_id=frame_id, poses=poses)
