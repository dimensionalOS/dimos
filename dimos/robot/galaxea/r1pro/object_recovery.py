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

"""Collision-checked empty-hand recovery; never a fallback grasp policy."""

import copy
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.robot.galaxea.r1pro.grasping_task import HOME_TCP
from dimos.robot.galaxea.r1pro.home_kinematics import HomeKinematics
from dimos.robot.galaxea.r1pro.object_packing_scene import ObjectLayout
from dimos.robot.galaxea.r1pro.object_packing_state import ObjectPackingState


def check_recovery_path(
    model: mujoco.MjModel,
    snapshot: mujoco.MjData,
    state: ObjectPackingState,
    points: list[dict[str, Any]],
) -> None:
    probe = copy.copy(snapshot)
    robot = state.guard.robot_bodies - state.guard.cargo_ids - {state.guard.tray_id}
    start = snapshot.qpos[state.qids].copy()
    for point in points:
        goal = np.asarray(point["positions"])
        allowed = {model.body(n).id for n in point.get("allowed_contacts", [])}
        count = max(1, int(np.ceil(np.max(np.abs(goal - start)) / 0.02)))
        for fraction in np.linspace(0, 1, count + 1):
            probe.qpos[state.qids] = start + (goal - start) * fraction
            # The follower is constrained in physics, but forward kinematics
            # does not solve the equality constraint after changing a leader.
            for side in ("left", "right"):
                probe.joint(f"{side}_gripper_follower").qpos[:] = probe.joint(
                    f"r1pro/{side}_gripper"
                ).qpos
            mujoco.mj_forward(model, probe)
            for contact in probe.contact:
                if contact.dist > 0 or contact.pos[2] < 0.06:
                    continue
                a, b = map(int, model.geom_bodyid[contact.geom])
                if (a in robot) != (b in robot):
                    other = b if a in robot else a
                    if other not in allowed:
                        raise RuntimeError(f"{point['phase']} blocked by {model.body(other).name}")
        start = goal


def plan_object_recovery(
    model: mujoco.MjModel, snapshot: mujoco.MjData, layout: ObjectLayout, home: NDArray[Any]
) -> list[dict[str, Any]]:
    """Open only supported contacts, retreat, and restore the trained home posture."""
    # mj_step advances qpos after computing xpos. Refresh a private copy so
    # the SDK compares FK from the same instant without altering live state.
    snapshot = copy.copy(snapshot)
    mujoco.mj_forward(model, snapshot)
    state = ObjectPackingState(model, snapshot, layout, home)
    rows = state.inventory()
    if any(not row["upright"] or not row["support_geoms"] for row in rows):
        raise RuntimeError(
            "Recovery requires supported upright objects; holding position. Use an explicit scene reset if needed."
        )
    allowed = [row["object"] for row in rows if not row["released"]]
    if np.max(np.abs(snapshot.qpos[state.qids] - home)) < 0.01 and not allowed:
        return []
    kinematics = HomeKinematics(model, snapshot, lock_lower_torso=True)
    errors = []
    for clearance in (0.98, 0.94, 1.04):
        probe = copy.copy(snapshot)
        released = snapshot.qpos[state.qids].copy()
        released[-2:] = 0.05
        points = [
            dict(
                phase="recovery_release",
                positions=released.tolist(),
                seconds=0.8,
                allowed_contacts=allowed,
            )
        ]
        probe.qpos[state.qids] = released
        above = snapshot.site("right_tcp").xpos.copy()
        above[2] = max(float(above[2]), clearance)
        try:
            for phase, target in (
                ("recovery_retreat", above),
                ("recovery_home", np.asarray(HOME_TCP)),
            ):
                mujoco.mj_forward(model, probe)
                q = kinematics.solve(
                    probe, {"right": target}, position_tolerance=0.002, orientation_tolerance=0.005
                )
                q[-2:] = 0.05
                probe.qpos[state.qids] = q
                points.append(dict(phase=phase, positions=q.tolist(), seconds=2.0))
            points.append(dict(phase="recovery_posture", positions=home.tolist(), seconds=3.0))
            check_recovery_path(model, snapshot, state, points)
            return points
        except RuntimeError as exc:
            errors.append(str(exc))
    raise RuntimeError("No clear recovery path: " + "; ".join(errors))
