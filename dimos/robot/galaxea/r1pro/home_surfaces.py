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

"""Ground named house destinations in collidable geometry, including low supports."""

import copy
from typing import Any

import mujoco
import numpy as np

from dimos.robot.galaxea.r1pro.packing_sim import PACKING_BODIES


def station_name(name: str) -> str:
    """Normalize common descriptions without inventing another destination."""
    normalized = name.strip().lower().replace(" ", "_")
    return {
        "desk": "dining_table",
        "laptop": "dining_table",
        "laptop_table": "dining_table",
        "dining": "dining_table",
        "kitchen_counter": "kitchen",
        "counter": "kitchen",
        "bedroom": "bed",
        "ground": "floor",
    }.get(normalized, normalized)


def low_surface_destination(
    model: mujoco.MjModel, data: mujoco.MjData, name: str
) -> dict[str, Any]:
    """Measure a patch on the packaged bed or open floor; validate docking separately.

    These anchors identify fixtures in this house package, not learned detections.
    All heights and allowed contacts come from the actual collidable surface.
    """
    if name == "bed":
        xy, base = [0.99, -2.50], [0.05, -2.50, 0.0]
        height_range = (0.35, 0.65)
        support_body = "58b7f3eee922fe43fe9f6156c0942be5f8e72e96-001"
    elif name == "floor":
        xy, base = [-1.0, -2.0], [-1.60, -2.0, 0.0]
        height_range = (-0.03, 0.06)
        support_body = None
    else:
        raise ValueError(f"Unknown low support: {name}")
    probe_model = copy.copy(model)
    excluded = {model.body(n).id for n in ("base_link", "task_bin", *PACKING_BODIES)}
    for body in range(1, model.nbody):
        if int(model.body_parentid[body]) in excluded:
            excluded.add(body)
    for gid in range(model.ngeom):
        collision = bool(model.geom_contype[gid] or model.geom_conaffinity[gid])
        probe_model.geom_group[gid] = (
            0 if collision and int(model.geom_bodyid[gid]) not in excluded else 5
        )
    probe = mujoco.MjData(probe_model)
    probe.qpos[:] = data.qpos
    mujoco.mj_forward(probe_model, probe)
    mask = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
    hits = []
    for dx in (-0.12, 0.0, 0.12):
        for dy in (-0.17, 0.0, 0.17):
            geom_id = np.array([-1], dtype=np.int32)
            normal = np.zeros(3)
            distance = mujoco.mj_ray(  # type: ignore[call-arg]
                probe_model,
                probe,
                np.array([xy[0] + dx, xy[1] + dy, 1.5]),
                np.array([0.0, 0.0, -1.0]),
                mask,
                True,
                -1,
                geom_id,
                normal,
            )
            z = 1.5 - distance
            if distance < 0 or not height_range[0] <= z <= height_range[1] or normal[2] < 0.7:
                raise RuntimeError(
                    f"{name} has no sufficiently level support under the tray footprint"
                )
            geom = model.geom(int(geom_id[0]))
            if support_body is not None and model.body(int(geom.bodyid[0])).name != support_body:
                raise RuntimeError("The expected mattress is missing or obstructed")
            hits.append((z, geom.name, int(geom.bodyid[0])))
    if max(h[0] for h in hits) - min(h[0] for h in hits) > 0.045:
        raise RuntimeError(f"{name} is too uneven for this tray")
    highest = max(hits)
    support = sorted({h[1] for h in hits})
    if support_body is not None:
        bid = model.body(support_body).id
        support = [
            model.geom(g).name
            for g in range(model.ngeom)
            if int(model.geom_bodyid[g]) == bid
            and (model.geom_contype[g] or model.geom_conaffinity[g])
        ]
    return {
        **({"approach_position": [-0.5, -2.0, -float(np.pi / 2)]} if name == "bed" else {}),
        "name": name,
        "support_geom": highest[1],
        "support_geoms": support,
        "tray_position": [*xy, highest[0]],
        "base_position": base,
    }
