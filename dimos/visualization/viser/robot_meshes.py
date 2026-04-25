# Copyright 2025-2026 Dimensional Inc.
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

"""MuJoCo-based robot mesh extractor + FK helper for the Viser viewer.

Loads an MJCF (the same one the sim subprocess loads), pulls out visual
mesh geoms with their parent-body indices and local poses, and runs FK
on demand to give world poses for every body in the model.

Lets the Viser render module display the same robot the simulation is
stepping, without forcing a separate URDF or duplicating geometry.
"""

from __future__ import annotations

from dataclasses import dataclass
import re
from typing import TYPE_CHECKING

import mujoco
import numpy as np

if TYPE_CHECKING:
    from pathlib import Path

# dimos joint names look like ``g1_LeftHipPitch``.  MJCF joint names look
# like ``left_hip_pitch_joint``.  Strip the hardware prefix, snake-case
# the camel suffix, append ``_joint``.
_CAMEL_RE = re.compile(r"(?<!^)(?=[A-Z])")


def dimos_joint_to_mjcf(name: str) -> str:
    parts = name.split("_", 1)
    suffix = parts[1] if len(parts) > 1 else parts[0]
    snake = _CAMEL_RE.sub("_", suffix).lower()
    return f"{snake}_joint"


@dataclass
class GeomInstance:
    """A single visual mesh geom, parented to a body."""

    body_name: str
    vertices: np.ndarray  # (V, 3)
    faces: np.ndarray  # (F, 3)
    local_pos: np.ndarray  # (3,)
    local_wxyz: np.ndarray  # (4,)
    rgba: tuple[float, float, float, float]


@dataclass
class RobotMeshes:
    model: mujoco.MjModel
    data: mujoco.MjData
    geoms: list[GeomInstance]
    # joint-name (in MJCF order) -> qpos address.  Used to splice incoming
    # joint_state values into the right slots of qpos.
    qpos_addr_by_mjcf_name: dict[str, int]
    # body_id -> body name (for viser entity paths).
    body_names: list[str]


def load_robot_meshes(
    mjcf_path: str | Path,
    *,
    visual_groups: tuple[int, ...] = (0, 1, 2),
    assets: dict[str, bytes] | None = None,
) -> RobotMeshes:
    """Parse the MJCF, pull visual mesh geoms into Python arrays.

    ``visual_groups`` defaults to MuJoCo's convention where group 0-2 are
    visual and group 3+ are collision.  Most menagerie / dimos models
    follow this; if a model uses different groups, override it.

    ``assets`` is an optional ``{filename: bytes}`` map for mesh files
    referenced by bare name in the MJCF (e.g. menagerie meshes).
    Pass ``dimos.simulation.mujoco.model.get_assets()`` for G1.
    """
    with open(mjcf_path) as f:
        xml = f.read()
    model = mujoco.MjModel.from_xml_string(xml, assets or {})
    data = mujoco.MjData(model)

    geoms: list[GeomInstance] = []
    for gid in range(model.ngeom):
        if int(model.geom_type[gid]) != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        if int(model.geom_group[gid]) not in visual_groups:
            continue
        mesh_id = int(model.geom_dataid[gid])
        if mesh_id < 0:
            continue

        v_start = int(model.mesh_vertadr[mesh_id])
        v_count = int(model.mesh_vertnum[mesh_id])
        f_start = int(model.mesh_faceadr[mesh_id])
        f_count = int(model.mesh_facenum[mesh_id])
        vertices = np.array(
            model.mesh_vert[v_start : v_start + v_count],
            dtype=np.float32,
        ).copy()
        faces = np.array(
            model.mesh_face[f_start : f_start + f_count],
            dtype=np.int32,
        ).copy()

        body_id = int(model.geom_bodyid[gid])
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or f"body_{body_id}"
        rgba = tuple(float(x) for x in model.geom_rgba[gid])
        geoms.append(
            GeomInstance(
                body_name=body_name,
                vertices=vertices,
                faces=faces,
                local_pos=np.array(model.geom_pos[gid], dtype=np.float32).copy(),
                local_wxyz=np.array(model.geom_quat[gid], dtype=np.float32).copy(),
                rgba=rgba,  # type: ignore[arg-type]
            )
        )

    qpos_addr_by_mjcf_name: dict[str, int] = {}
    for jid in range(model.njnt):
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        if jname is None:
            continue
        qpos_addr_by_mjcf_name[jname] = int(model.jnt_qposadr[jid])

    body_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid) or f"body_{bid}"
        for bid in range(model.nbody)
    ]

    return RobotMeshes(
        model=model,
        data=data,
        geoms=geoms,
        qpos_addr_by_mjcf_name=qpos_addr_by_mjcf_name,
        body_names=body_names,
    )


def apply_state(
    rm: RobotMeshes,
    *,
    base_pos: np.ndarray | None = None,
    base_wxyz: np.ndarray | None = None,
    joint_positions: dict[str, float] | None = None,
) -> None:
    """Splice base + joint values into qpos and run FK in-place.

    After this returns, ``rm.data.xpos`` and ``rm.data.xquat`` hold each
    body's world pose.  Cheap (~50 us for G1).
    """
    if base_pos is not None:
        rm.data.qpos[0:3] = base_pos
    if base_wxyz is not None:
        rm.data.qpos[3:7] = base_wxyz
    if joint_positions:
        for name, q in joint_positions.items():
            adr = rm.qpos_addr_by_mjcf_name.get(name)
            if adr is not None:
                rm.data.qpos[adr] = q
    mujoco.mj_kinematics(rm.model, rm.data)
