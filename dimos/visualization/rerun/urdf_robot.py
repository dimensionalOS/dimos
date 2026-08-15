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
from xml.etree import ElementTree

import numpy as np

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.data import get_data

JointNameMapper = Callable[[str], str]
_TransformState = tuple[
    str,
    tuple[float, float, float],
    tuple[float, float, float, float],
]
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


def _rerun_transform_without_frames(rr: Any, transform: Any) -> Any:
    translation = transform.translation.as_arrow_array().to_pylist()[0]
    quaternion = transform.quaternion.as_arrow_array().to_pylist()[0]
    return rr.Transform3D(
        translation=translation,
        rotation=rr.Quaternion(xyzw=quaternion),
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


@dataclass
class UrdfRobotTransformFilter:
    """Drop duplicate robot TF and forward only changed non-robot frames."""

    urdf_path: str | Path
    child_frame_prefix: str
    translation_epsilon_m: float = 1e-5
    quaternion_epsilon: float = 1e-7
    _robot_link_names: frozenset[str] | None = field(default=None, init=False, repr=False)
    _states: dict[str, _TransformState] = field(default_factory=dict, init=False, repr=False)

    def __post_init__(self) -> None:
        if not self.child_frame_prefix.strip("/"):
            raise ValueError("child_frame_prefix must not be empty")
        if self.translation_epsilon_m < 0.0:
            raise ValueError("translation_epsilon_m must be non-negative")
        if self.quaternion_epsilon < 0.0:
            raise ValueError("quaternion_epsilon must be non-negative")
        self.child_frame_prefix = self.child_frame_prefix.rstrip("/")

    def __call__(self, message: TFMessage) -> TFMessage:
        robot_link_names = self._load_robot_link_names()
        changed: list[Transform] = []
        for transform in message:
            if self._is_robot_link(transform.child_frame_id, robot_link_names):
                continue
            state = _transform_state(transform)
            previous = self._states.get(transform.child_frame_id)
            if previous is None or self._changed(previous, state):
                changed.append(transform)
                self._states[transform.child_frame_id] = state
        return TFMessage(*changed)

    def reset(self) -> None:
        """Make the next non-robot TF snapshot seed a fresh recording."""

        self._states.clear()

    def _load_robot_link_names(self) -> frozenset[str]:
        if self._robot_link_names is None:
            root = ElementTree.parse(_resolve_urdf_path(self.urdf_path)).getroot()
            self._robot_link_names = frozenset(
                name for link in root.findall(".//link") if (name := link.attrib.get("name"))
            )
        return self._robot_link_names

    def _is_robot_link(self, child_frame_id: str, robot_link_names: frozenset[str]) -> bool:
        prefix = f"{self.child_frame_prefix}/"
        if not child_frame_id.startswith(prefix):
            return False
        return child_frame_id.removeprefix(prefix) in robot_link_names

    def _changed(self, previous: _TransformState, current: _TransformState) -> bool:
        if previous[0] != current[0]:
            return True
        translation_delta = max(
            abs(first - second) for first, second in zip(previous[1], current[1], strict=True)
        )
        if translation_delta > self.translation_epsilon_m:
            return True
        direct = max(
            abs(first - second) for first, second in zip(previous[2], current[2], strict=True)
        )
        negated = max(
            abs(first + second) for first, second in zip(previous[2], current[2], strict=True)
        )
        return min(direct, negated) > self.quaternion_epsilon


def _transform_state(transform: Transform) -> _TransformState:
    translation = transform.translation
    rotation = transform.rotation
    return (
        transform.frame_id,
        (translation.x, translation.y, translation.z),
        (rotation.x, rotation.y, rotation.z, rotation.w),
    )


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
    joint_position_epsilon: float = 1e-6
    _tree: Any = field(default=None, init=False, repr=False)
    _joints: list[Any] = field(default_factory=list, init=False, repr=False)
    _joint_paths: dict[str, str] = field(default_factory=dict, init=False, repr=False)
    _joint_values: dict[str, float] = field(default_factory=dict, init=False, repr=False)
    _logged_joint_values: dict[str, float] = field(default_factory=dict, init=False, repr=False)
    _movable_joints: list[Any] = field(default_factory=list, init=False, repr=False)
    _fixed_joints_logged: bool = field(default=False, init=False, repr=False)

    def __post_init__(self) -> None:
        if self.joint_position_epsilon < 0.0:
            raise ValueError("joint_position_epsilon must be non-negative")

    def __call__(self, msg: Any) -> list[tuple[str, Any]]:
        self._load_tree()

        for name, position in zip(msg.name, msg.position, strict=False):
            urdf_joint = self.joint_name_mapper(str(name))
            if urdf_joint in self._joint_values:
                self._joint_values[urdf_joint] = float(position)

        import rerun as rr

        joints = (
            self._joints
            if not self._fixed_joints_logged
            else [
                joint
                for joint in self._movable_joints
                if abs(
                    self._joint_values[joint.name]
                    - self._logged_joint_values.get(joint.name, float("inf"))
                )
                > self.joint_position_epsilon
            ]
        )
        entities = [
            (
                self._joint_paths[joint.name],
                _rerun_transform_without_frames(
                    rr,
                    joint.compute_transform(
                        self._joint_values.get(joint.name, 0.0),
                        clamp=self.clamp_joint_limits,
                    ),
                ),
            )
            for joint in joints
        ]
        for joint in joints:
            if joint.name in self._joint_values:
                self._logged_joint_values[joint.name] = self._joint_values[joint.name]
        self._fixed_joints_logged = True
        return entities

    def reset(self) -> None:
        """Re-emit fixed joints when Rerun starts a fresh recording."""

        self._fixed_joints_logged = False
        self._logged_joint_values.clear()

    def _load_tree(self) -> None:
        if self._tree is None:
            import rerun.urdf as rr_urdf

            URDF = _yourdfpy_urdf()
            urdf_path = _resolve_urdf_path(self.urdf_path)
            robot = URDF.load(str(urdf_path))
            link_paths = _build_link_paths(
                self.root_path,
                str(robot.base_link),
                list(robot.robot.joints),
            )

            self._tree = rr_urdf.UrdfTree.from_file_path(urdf_path)
            self._joints = self._tree.joints()
            self._joint_paths = {
                joint.name: link_paths.get(
                    joint.child_link,
                    f"{self.root_path}/{_rerun_path_part(joint.child_link)}",
                )
                for joint in self._joints
            }
            self._joint_values = {
                joint.name: 0.0
                for joint in self._joints
                if joint.joint_type in {"revolute", "continuous", "prismatic"}
            }
            self._movable_joints = [
                joint for joint in self._joints if joint.name in self._joint_values
            ]
