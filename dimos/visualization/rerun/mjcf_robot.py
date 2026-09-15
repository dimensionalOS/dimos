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

"""Generic Rerun helpers for visualizing MJCF robots."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np

from dimos.utils.data import get_data


def default_mjcf_joint_name_mapper(name: str) -> str:
    """Map a namespaced DimOS joint name to its MJCF joint name."""

    return name.rsplit("/", 1)[-1]


def _resolve_mjcf_path(path: str | Path) -> Path:
    candidate = Path(path).expanduser()
    if candidate.is_absolute() or candidate.exists():
        return candidate
    return get_data(candidate)


def _rerun_path_part(name: str) -> str:
    return name.replace("/", "_").replace(" ", "_")


class MjcfRobotRerun:
    """Log an MJCF robot's meshes once and animate its body transforms."""

    def __init__(
        self,
        *,
        mjcf_path: str | Path,
        root_path: str,
        root_body_name: str,
        initial_joint_positions: Mapping[str, float] | None = None,
        visual_geom_group: int = 2,
    ) -> None:
        self.mjcf_path = mjcf_path
        self.root_path = root_path.rstrip("/")
        self.root_body_name = root_body_name
        self.initial_joint_positions = dict(initial_joint_positions or {})
        self.visual_geom_group = visual_geom_group

        self._mujoco: Any | None = None
        self._model: Any | None = None
        self._data: Any | None = None
        self._body_ids: tuple[int, ...] = ()
        self._body_paths: dict[int, str] = {}
        self._joint_qpos_addresses: dict[str, int] = {}

    def static(self, rr: Any) -> list[tuple[str, Any]]:
        """Return static visual meshes attached to the MJCF body hierarchy."""

        self._load()
        assert self._model is not None
        assert self._mujoco is not None

        model = self._model
        entities: list[tuple[str, Any]] = []
        body_ids = set(self._body_ids)

        for geom_id in range(model.ngeom):
            body_id = int(model.geom_bodyid[geom_id])
            if body_id not in body_ids:
                continue
            if int(model.geom_group[geom_id]) != self.visual_geom_group:
                continue
            if int(model.geom_type[geom_id]) != int(self._mujoco.mjtGeom.mjGEOM_MESH):
                continue

            mesh_id = int(model.geom_dataid[geom_id])
            if mesh_id < 0:
                continue
            mesh_name = (
                self._mujoco.mj_id2name(
                    model,
                    self._mujoco.mjtObj.mjOBJ_MESH,
                    mesh_id,
                )
                or f"mesh_{mesh_id}"
            )
            entity_path = (
                f"{self._body_paths[body_id]}/visual/{geom_id:03d}_{_rerun_path_part(mesh_name)}"
            )

            entities.append(
                (
                    entity_path,
                    rr.Transform3D(
                        translation=np.asarray(model.geom_pos[geom_id], dtype=float).tolist(),
                        rotation=rr.Quaternion(
                            xyzw=self._xyzw(model.geom_quat[geom_id]),
                        ),
                    ),
                )
            )
            entities.append((entity_path, self._mesh_archetype(rr, mesh_id, geom_id)))

        return entities

    def joint_state(self, msg: Any) -> list[tuple[str, Any]]:
        """Convert a JointState-like message into local MJCF body transforms."""

        import rerun as rr

        self._load()
        assert self._model is not None
        assert self._data is not None
        assert self._mujoco is not None

        for name, position in zip(msg.name, msg.position, strict=False):
            mjcf_name = default_mjcf_joint_name_mapper(str(name))
            address = self._joint_qpos_addresses.get(mjcf_name)
            if address is not None:
                self._data.qpos[address] = float(position)

        self._mujoco.mj_forward(self._model, self._data)
        entities: list[tuple[str, Any]] = []
        for body_id in self._body_ids:
            parent_id = int(self._model.body_parentid[body_id])
            parent_rotation = np.asarray(self._data.xmat[parent_id], dtype=float).reshape(3, 3)
            body_rotation = np.asarray(self._data.xmat[body_id], dtype=float).reshape(3, 3)
            translation = parent_rotation.T @ (
                np.asarray(self._data.xpos[body_id], dtype=float)
                - np.asarray(self._data.xpos[parent_id], dtype=float)
            )
            rotation = parent_rotation.T @ body_rotation
            quaternion = np.empty(4, dtype=float)
            self._mujoco.mju_mat2Quat(quaternion, rotation.ravel())
            entities.append(
                (
                    self._body_paths[body_id],
                    self._transform(rr, translation, quaternion),
                )
            )
        return entities

    def _load(self) -> None:
        if self._model is not None:
            return

        import mujoco

        model = mujoco.MjModel.from_xml_path(str(_resolve_mjcf_path(self.mjcf_path)))
        root_body_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            self.root_body_name,
        )
        if root_body_id < 0:
            raise ValueError(f"MJCF body not found: {self.root_body_name}")

        body_ids = tuple(
            body_id
            for body_id in range(1, model.nbody)
            if self._is_descendant(model, body_id, root_body_id)
        )
        body_paths: dict[int, str] = {}
        for body_id in body_ids:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            path_part = _rerun_path_part(name or f"body_{body_id}")
            parent_id = int(model.body_parentid[body_id])
            parent_path = body_paths.get(parent_id, self.root_path)
            body_paths[body_id] = f"{parent_path}/{path_part}"

        joint_qpos_addresses: dict[str, int] = {}
        root_freejoint_address: int | None = None
        free_joint = int(mujoco.mjtJoint.mjJNT_FREE)  # type: ignore[attr-defined]
        ball_joint = int(mujoco.mjtJoint.mjJNT_BALL)  # type: ignore[attr-defined]
        for joint_id in range(model.njnt):
            joint_type = int(model.jnt_type[joint_id])
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            if joint_type == free_joint:
                if int(model.jnt_bodyid[joint_id]) == root_body_id:
                    root_freejoint_address = int(model.jnt_qposadr[joint_id])
                continue
            if joint_type == ball_joint or joint_name is None:
                continue
            joint_qpos_addresses[joint_name] = int(model.jnt_qposadr[joint_id])

        data = mujoco.MjData(model)
        data.qpos[:] = model.qpos0
        if root_freejoint_address is not None:
            data.qpos[root_freejoint_address : root_freejoint_address + 7] = (
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
            )
        for name, position in self.initial_joint_positions.items():
            address = joint_qpos_addresses.get(name)
            if address is not None:
                data.qpos[address] = float(position)
        mujoco.mj_forward(model, data)

        self._mujoco = mujoco
        self._model = model
        self._data = data
        self._body_ids = body_ids
        self._body_paths = body_paths
        self._joint_qpos_addresses = joint_qpos_addresses

    @staticmethod
    def _is_descendant(model: Any, body_id: int, root_body_id: int) -> bool:
        current = body_id
        while current != 0:
            if current == root_body_id:
                return True
            current = int(model.body_parentid[current])
        return False

    def _mesh_archetype(self, rr: Any, mesh_id: int, geom_id: int) -> Any:
        assert self._model is not None
        model = self._model
        vertex_address = int(model.mesh_vertadr[mesh_id])
        vertex_count = int(model.mesh_vertnum[mesh_id])
        normal_address = int(model.mesh_normaladr[mesh_id])
        normal_count = int(model.mesh_normalnum[mesh_id])
        face_address = int(model.mesh_faceadr[mesh_id])
        face_count = int(model.mesh_facenum[mesh_id])
        material_id = int(model.geom_matid[geom_id])
        rgba = model.mat_rgba[material_id] if material_id >= 0 else model.geom_rgba[geom_id]
        color = np.clip(np.rint(np.asarray(rgba) * 255.0), 0, 255).astype(np.uint8)

        return rr.Mesh3D(
            vertex_positions=np.asarray(
                model.mesh_vert[vertex_address : vertex_address + vertex_count],
                dtype=np.float32,
            ),
            triangle_indices=np.asarray(
                model.mesh_face[face_address : face_address + face_count],
                dtype=np.uint32,
            ),
            vertex_normals=(
                np.asarray(
                    model.mesh_normal[normal_address : normal_address + normal_count],
                    dtype=np.float32,
                )
                if normal_count == vertex_count
                else None
            ),
            albedo_factor=color.tolist(),
        )

    @staticmethod
    def _xyzw(quaternion_wxyz: Any) -> list[float]:
        quaternion = np.asarray(quaternion_wxyz, dtype=float)
        return [
            float(quaternion[1]),
            float(quaternion[2]),
            float(quaternion[3]),
            float(quaternion[0]),
        ]

    def _transform(self, rr: Any, translation: Any, quaternion_wxyz: Any) -> Any:
        return rr.Transform3D(
            translation=np.asarray(translation, dtype=float).tolist(),
            rotation=rr.Quaternion(xyzw=self._xyzw(quaternion_wxyz)),
        )
