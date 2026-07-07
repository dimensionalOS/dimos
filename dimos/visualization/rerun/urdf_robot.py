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

"""Generic Rerun helpers for visualizing URDF robots."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from importlib import import_module
from pathlib import Path
from typing import Any

import numpy as np

from dimos.utils.data import get_data

JointNameMapper = Callable[[str], str]
RERUN_URDF_INSTALL_HINT = (
    "Rerun URDF robot visualization requires yourdfpy. Install it with: "
    "uv sync --extra visualization. Linux aarch64 is currently unsupported "
    "because yourdfpy depends on embreex, which does not publish Linux "
    "aarch64 wheels."
)


def default_joint_name_mapper(name: str) -> str:
    """Map a DimOS hardware joint name to the common URDF joint name form."""
    short = name.rsplit("/", 1)[-1]
    return short if short.endswith("_joint") else f"{short}_joint"


def _resolve_urdf_path(path: str | Path) -> Path:
    candidate = Path(path).expanduser()
    if candidate.is_absolute() or candidate.exists():
        return candidate
    return get_data(candidate)


def _matrix_to_rerun_transform(rr: Any, matrix: np.ndarray) -> Any:
    return rr.Transform3D(
        translation=matrix[:3, 3].tolist(),
        mat3x3=matrix[:3, :3].tolist(),
    )


def _mesh_to_rerun(rr: Any, mesh: Any) -> Any:
    color: tuple[int, int, int, int] = (178, 178, 178, 255)
    face_colors = getattr(getattr(mesh, "visual", None), "face_colors", None)
    if face_colors is not None and len(face_colors):
        rgba = np.asarray(face_colors[0], dtype=np.uint8).tolist()
        if len(rgba) >= 4:
            color = (int(rgba[0]), int(rgba[1]), int(rgba[2]), int(rgba[3]))

    return rr.Mesh3D(
        vertex_positions=np.asarray(mesh.vertices, dtype=np.float32),
        triangle_indices=np.asarray(mesh.faces, dtype=np.uint32),
        vertex_normals=np.asarray(mesh.vertex_normals, dtype=np.float32),
        albedo_factor=color,
    )


def _rerun_path_part(name: str) -> str:
    return name.replace("/", "_").replace(" ", "_")


def _yourdfpy_urdf() -> Any:
    try:
        return import_module("yourdfpy").URDF
    except ModuleNotFoundError as e:
        if e.name not in {"yourdfpy", "trimesh", "embreex"}:
            raise
        raise ModuleNotFoundError(RERUN_URDF_INSTALL_HINT) from e


def _build_link_paths(
    root_path: str,
    root_link: str,
    joints: list[Any],
) -> dict[str, str]:
    link_paths = {root_link: f"{root_path}/{_rerun_path_part(root_link)}"}
    remaining = list(joints)

    while remaining:
        progressed = False
        for joint in remaining[:]:
            parent = str(joint.parent)
            child = str(joint.child)
            parent_path = link_paths.get(parent)
            if parent_path is None:
                continue
            link_paths[child] = f"{parent_path}/{_rerun_path_part(child)}"
            remaining.remove(joint)
            progressed = True
        if not progressed:
            for joint in remaining:
                link_paths[str(joint.child)] = f"{root_path}/{_rerun_path_part(joint.child)}"
            break

    return link_paths


_MOVABLE_JOINT_TYPES = frozenset({"revolute", "continuous", "prismatic"})


def _load_urdf_tree_and_paths(urdf_path: str | Path, root_path: str) -> tuple[Any, dict[str, str]]:
    """Load the rerun ``UrdfTree`` and map every joint name to its child-link path.

    Shared by the static rest-pose logger and the per-frame animator so both
    target *identical* entity paths -- if they diverged, static fixed-joint
    transforms and per-frame updates would land on different entities and the
    mesh would break.
    """
    import rerun.urdf as rr_urdf

    resolved = _resolve_urdf_path(urdf_path)
    robot = _yourdfpy_urdf().load(str(resolved))
    link_paths = _build_link_paths(root_path, str(robot.base_link), list(robot.robot.joints))
    tree = rr_urdf.UrdfTree.from_file_path(resolved)
    joint_paths = {
        joint.name: link_paths.get(
            joint.child_link, f"{root_path}/{_rerun_path_part(joint.child_link)}"
        )
        for joint in tree.joints()
    }
    return tree, joint_paths


@dataclass
class UrdfRobotStaticJointsRerunFactory:
    """Log every URDF joint's rest transform once, as static.

    Fixed joints are constant, so this is their permanent transform; movable
    joints get an initial rest pose that ``UrdfRobotJointStateRerunFactory``
    then overrides per frame. Logging fixed joints here (rather than re-sending
    them every message from the animator) is what lets the animator skip them
    while keeping the transform tree complete.
    """

    urdf_path: str | Path
    root_path: str
    clamp_joint_limits: bool = False

    def __call__(self, rr: Any) -> list[tuple[str, Any]]:
        tree, joint_paths = _load_urdf_tree_and_paths(self.urdf_path, self.root_path)
        out: list[tuple[str, Any]] = []
        for joint in tree.joints():
            result = joint._inner.compute_transform(0.0, clamp=self.clamp_joint_limits)
            out.append(
                (
                    joint_paths[joint.name],
                    rr.Transform3D(
                        translation=result["translation"],
                        rotation=rr.Quaternion(xyzw=result["quaternion_xyzw"]),
                    ),
                )
            )
        return out


@dataclass
class UrdfRobotStaticRerunFactory:
    """Log a URDF robot's static visual meshes under a Rerun root path."""

    urdf_path: str | Path
    root_path: str
    _robot: Any = field(default=None, init=False, repr=False)

    def __call__(self, rr: Any) -> list[tuple[str, Any]]:
        robot = self._load_robot()
        link_paths = _build_link_paths(
            self.root_path,
            str(robot.base_link),
            list(robot.robot.joints),
        )
        entities: list[tuple[str, Any]] = [
            (self.root_path, rr.Transform3D()),
            (link_paths[str(robot.base_link)], rr.Transform3D()),
        ]

        geometry_nodes = set(robot.scene.graph.nodes_geometry)
        for parent, node, edge_data in robot.scene.graph.to_edgelist():
            if node not in geometry_nodes:
                continue

            geometry_name = edge_data.get("geometry")
            mesh = robot.scene.geometry.get(geometry_name)
            if mesh is None:
                continue

            parent_path = link_paths.get(
                str(parent), f"{self.root_path}/{_rerun_path_part(parent)}"
            )
            path = f"{parent_path}/{_rerun_path_part(node)}"
            matrix = np.asarray(edge_data.get("matrix", np.eye(4)), dtype=float)
            entities.append((path, _matrix_to_rerun_transform(rr, matrix)))
            entities.append((path, _mesh_to_rerun(rr, mesh)))

        return entities

    def _load_robot(self) -> Any:
        if self._robot is None:
            URDF = _yourdfpy_urdf()
            self._robot = URDF.load(str(_resolve_urdf_path(self.urdf_path)))
        return self._robot


@dataclass
class UrdfRobotJointStateRerunFactory:
    """Convert JointState-like messages into animated URDF link transforms."""

    urdf_path: str | Path
    root_path: str
    joint_name_mapper: JointNameMapper = default_joint_name_mapper
    clamp_joint_limits: bool = False
    _tree: Any = field(default=None, init=False, repr=False)
    _joints: list[Any] = field(default_factory=list, init=False, repr=False)
    _joint_paths: dict[str, str] = field(default_factory=dict, init=False, repr=False)
    _movable_by_name: dict[str, Any] = field(default_factory=dict, init=False, repr=False)

    def __call__(self, msg: Any) -> list[tuple[str, Any]]:
        self._load_tree()

        import rerun as rr

        # Animate only the joints present in this message, straight from the
        # Rust FK result. Two wins over the original:
        #   * skip the old joint.compute_transform() -> Arrow round-trip
        #     (as_arrow_array().to_pylist()) -> rebuild dance (~3x the work).
        #   * skip fixed joints and any undriven movable joints (e.g. the hands):
        #     they're seeded once as static by UrdfRobotStaticJointsRerunFactory
        #     and Rerun holds each entity's last transform, so we log one
        #     transform per commanded joint (~29) instead of one per URDF joint
        #     (52). Both cut the per-message load on the bridge's LCM thread.
        out: list[tuple[str, Any]] = []
        for name, position in zip(msg.name, msg.position, strict=False):
            joint = self._movable_by_name.get(self.joint_name_mapper(str(name)))
            if joint is None:
                continue
            result = joint._inner.compute_transform(float(position), clamp=self.clamp_joint_limits)
            out.append(
                (
                    self._joint_paths[joint.name],
                    rr.Transform3D(
                        translation=result["translation"],
                        rotation=rr.Quaternion(xyzw=result["quaternion_xyzw"]),
                    ),
                )
            )
        return out

    def _load_tree(self) -> None:
        if self._tree is None:
            self._tree, self._joint_paths = _load_urdf_tree_and_paths(
                self.urdf_path, self.root_path
            )
            self._joints = self._tree.joints()
            self._movable_by_name = {
                joint.name: joint
                for joint in self._joints
                if joint.joint_type in _MOVABLE_JOINT_TYPES
            }
