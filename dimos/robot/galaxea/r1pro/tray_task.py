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

"""Physical evidence and destination geometry for two-handed tray delivery."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

import mujoco
import numpy as np


@dataclass(frozen=True)
class TrayDestination:
    support_geom: str
    tray_position: list[float]
    base_position: list[float]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def laptop_destination(model: mujoco.MjModel, data: mujoco.MjData) -> TrayDestination:
    """Find the real support below the laptop and an empty spot beside it.

    This demo expects the packaged house's axis-aligned rectangular desktop.
    Collision planning and physical placement checks still validate the target.
    """
    laptop = model.body("entity:laptop")
    position = data.body(laptop.id).xpos.copy()
    gid = np.array([-1], dtype=np.int32)
    distance = mujoco.mj_ray(
        model,
        data,
        position - [0, 0, 0.005],
        np.array([0.0, 0.0, -1.0]),
        None,
        True,
        laptop.id,
        gid,
    )
    if distance < 0 or distance > 0.15:
        raise RuntimeError("Could not identify the tabletop supporting the laptop")
    geom = model.geom(int(gid[0]))
    rotation = data.geom_xmat[geom.id].reshape(3, 3)
    if geom.type[0] != mujoco.mjtGeom.mjGEOM_BOX or abs(rotation[2, 2]) < 0.999:
        raise RuntimeError("The laptop delivery requires a horizontal box tabletop")
    extent = np.abs(rotation) @ geom.size
    centre = data.geom_xpos[geom.id]
    low, high = centre - extent, centre + extent
    # The left-hand portion is clear of the laptop in this packaged house.
    target = np.array([low[0] + 0.20, high[1] - 0.18, high[2]])
    if high[0] - low[0] < 1.0 or not low[1] + 0.15 < target[1] < high[1] - 0.15:
        raise RuntimeError("The laptop tabletop has insufficient room for this tray")
    if abs(target[0] - position[0]) < 0.4:
        raise RuntimeError("The proposed tray position is too close to the laptop")
    return TrayDestination(
        geom.name, target.tolist(), [float(target[0]), float(target[1] + 0.48), -float(np.pi / 2)]
    )


def tray_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    cargo_bodies: tuple[str, ...] = ("task_bottle",),
) -> dict[str, Any]:
    """Measure support and finger forces; no object state is changed."""
    tray = model.body("task_bin").id
    cargo = {model.body(name).id for name in cargo_bodies}
    pads = {
        model.geom(f"{side}_finger_pad{i}").id: f"{side}_{i}"
        for side in ("left", "right")
        for i in (1, 2)
    }
    handles = {model.geom(f"tray_{side}_handle").id for side in ("left", "right")}
    touching: set[str] = set()
    support: set[str] = set()
    forces = np.zeros(6)
    for index, contact in enumerate(data.contact):
        first, second = map(int, contact.geom)
        if tray not in (model.geom_bodyid[first], model.geom_bodyid[second]):
            continue
        mujoco.mj_contactForce(model, data, index, forces)
        if contact.dist > 0 or forces[0] < 0.02:
            continue
        if first in handles and second in pads:
            touching.add(pads[second])
        if second in handles and first in pads:
            touching.add(pads[first])
        other = second if model.geom_bodyid[first] == tray else first
        if other not in pads and model.geom_bodyid[other] not in cargo:
            support.add(model.geom(other).name)
    rotation = data.body(tray).xmat.reshape(3, 3)
    opening = [float(data.joint(f"r1pro/{side}_gripper").qpos[0]) for side in ("left", "right")]
    return {
        "position": data.body(tray).xpos.tolist(),
        "tilt_radians": float(np.arccos(np.clip(rotation[2, 2], -1, 1))),
        "velocity_norm": float(np.linalg.norm(data.joint("task_tray_free").qvel)),
        "finger_contacts": sorted(touching),
        "bimanual_grasp": len(touching) == 4,
        "support_geoms": sorted(support),
        "released": not touching and min(opening) > 0.04,
    }
