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

"""Classical right-arm unloading from a supported tray onto a real surface."""

from typing import Any

import mujoco
import numpy as np

from dimos.robot.galaxea.r1pro.grasping_sim import BOTTLE_HALF_HEIGHT
from dimos.robot.galaxea.r1pro.packing_sim import PACKING_BODIES, PACKING_JOINTS
from dimos.robot.galaxea.r1pro.tray_motion import TrayMotion, TrayWaypoint


def bottle_contacts(model: mujoco.MjModel, data: mujoco.MjData, index: int) -> dict[str, Any]:
    """Measure current gripper forces, support, pose and velocity of one bottle."""
    body = model.body(PACKING_BODIES[index]).id
    pads = {model.geom(f"right_finger_pad{i}").id for i in (1, 2)}
    touching: set[int] = set()
    support: set[str] = set()
    force = np.zeros(6)
    for i, contact in enumerate(data.contact):
        a, b = map(int, contact.geom)
        if body not in (model.geom_bodyid[a], model.geom_bodyid[b]):
            continue
        mujoco.mj_contactForce(model, data, i, force)
        if contact.dist > 0 or force[0] < 0.02:
            continue
        other = b if model.geom_bodyid[a] == body else a
        if other in pads:
            touching.add(other)
        else:
            # Contact normals point from geom1 to geom2. Side contact with a
            # wall or neighbour must not authorize releasing a held bottle.
            upward = contact.frame[:3] * (1 if int(model.geom_bodyid[b]) == body else -1)
            if upward[2] > 0.7:
                support.add(model.geom(other).name)
    axis = data.body(body).xmat.reshape(3, 3)[:, 2]
    return {
        "bottle": index + 1,
        "position": data.body(body).xpos.tolist(),
        "grasped": touching == pads,
        "touching_pads": sorted(model.geom(g).name for g in touching),
        "released": bool(not touching and data.joint("r1pro/right_gripper").qpos[0] > 0.04),
        "support_geoms": sorted(support),
        "upright": bool(axis[2] > np.cos(np.deg2rad(15))),
        "velocity_norm": float(np.linalg.norm(data.joint(PACKING_JOINTS[index]).qvel)),
    }


def plan_unload(
    model: mujoco.MjModel, data: mujoco.MjData, index: int, target: list[float]
) -> list[TrayWaypoint]:
    """Plan a vertical grasp, lift clear of neighbours, then release on a surface.

    Target is the bottle's bottom centre on the tabletop. All planning qpos
    writes are confined to TrayMotion's copied kinematic state.
    """
    motion = TrayMotion(model, data, cargo_bodies=(PACKING_BODIES[index],))
    source = data.body(PACKING_BODIES[index]).xpos.copy()
    grasp = source + np.array([0, 0, 0.03])
    placement = np.asarray(target) + np.array([0, 0, BOTTLE_HALF_HEIGHT + 0.020])
    clearance = max(grasp[2], placement[2]) + 0.16
    above = np.r_[source[:2], clearance]
    over = np.r_[placement[:2], clearance]
    left_home = np.array([0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0])
    points = []
    if np.max(np.abs(motion.initial[4:11] - left_home)) > 0.03:
        rotation = data.body("base_link").xmat.reshape(3, 3)
        left_clear = data.site("left_tcp").xpos + rotation @ np.array([-0.18, 0.0, 0.10])
        motion.body_pose(
            left_clear,
            data.site("right_tcp").xpos.copy(),
            position_tolerance=0.005,
            orientation_tolerance=0.03,
        )
        points.append(
            TrayWaypoint("unload_clear_left", motion.probe.qpos[motion.qids].tolist(), 2.0)
        )
        parked = motion.probe.qpos[motion.qids].copy()
        parked[4:11] = left_home
        parked[-2] = 0.05
        motion.probe.qpos[motion.qids] = parked
        points.append(TrayWaypoint("unload_park_left", parked.tolist(), 3.0))
    for phase, xyz, opening, seconds in (
        ("unload_approach", above, 0.05, 2.0),
        ("unload_lower", grasp, 0.05, 1.5),
        ("unload_grasp", grasp, 0.0, 1.0),
        ("unload_lift", above, 0.0, 2.0),
        ("unload_transfer", over, 0.0, 2.0),
        ("unload_place", placement, 0.0, 2.0),
        ("unload_release", placement, 0.05, 1.0),
        ("unload_retreat", over, 0.05, 1.5),
    ):
        motion.body_pose(None, xyz)
        motion.probe.qpos[motion.qids[-1]] = opening
        points.append(TrayWaypoint(phase, motion.probe.qpos[motion.qids].tolist(), seconds))
    return motion._checked(points, data, allow_tray_contact=False)
