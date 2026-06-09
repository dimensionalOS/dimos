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

from __future__ import annotations

from collections.abc import Callable, Sequence
import itertools
from pathlib import Path
from typing import Any

from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.manipulation.viser_panel.animation import PreviewAnimator
from dimos.manipulation.viser_panel.state import PanelSession
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.JointState import JointState

GOAL_ROBOT_FEASIBLE_COLOR = (255, 122, 0)
GOAL_ROBOT_INFEASIBLE_COLOR = (255, 30, 30)
GOAL_ROBOT_FEASIBLE_OPACITY = 0.7
GOAL_ROBOT_INFEASIBLE_OPACITY = 0.75
GOAL_ROBOT_MESH_COLOR = (*GOAL_ROBOT_FEASIBLE_COLOR, GOAL_ROBOT_FEASIBLE_OPACITY)


class PanelScene:
    def __init__(
        self,
        server: Any | None,
        session: PanelSession,
        handles: dict[str, Any],
        urdfs: dict[str, Any],
        viser_urdf: Any | None,
    ) -> None:
        self.server = server
        self.session = session
        self.handles = handles
        self.urdfs = urdfs
        self.viser_urdf = viser_urdf

    def ensure_scene_nodes(
        self,
        robot_name: str,
        info: dict[str, Any],
        on_target_update: Callable[[Any], None],
        prepared_urdf_path: Callable[[dict[str, Any]], Path] | None = None,
    ) -> None:
        if self.server is None:
            return
        if "ee_control" not in self.handles:
            self.handles["ee_control"] = self.server.scene.add_transform_controls(
                f"/targets/{robot_name}/ee_control", scale=0.25
            )
            self.handles["ee_control"].on_update(lambda event: on_target_update(event.target))
        ViserUrdf = self.viser_urdf
        if ViserUrdf is None or not info.get("model_path"):
            return
        for kind in ("current", "ghost"):
            key = f"{robot_name}:{kind}"
            if key not in self.urdfs:
                root_node_name = (
                    f"/robots/{robot_name}/current"
                    if kind == "current"
                    else f"/targets/{robot_name}/ghost"
                )
                mesh_color_override = GOAL_ROBOT_MESH_COLOR if kind == "ghost" else None
                path_factory = prepared_urdf_path or self.prepared_urdf_path
                self.urdfs[key] = ViserUrdf(
                    self.server,
                    path_factory(info),
                    root_node_name=root_node_name,
                    mesh_color_override=mesh_color_override,
                )
                if kind == "ghost":
                    self.set_urdf_mesh_material(
                        self.urdfs[key], GOAL_ROBOT_FEASIBLE_COLOR, GOAL_ROBOT_FEASIBLE_OPACITY
                    )

    def update_current_robot(self, robot_name: str) -> None:
        current = self.urdfs.get(f"{robot_name}:current")
        if current is not None and self.session.current_joints is not None:
            self.set_urdf_joints(current, self.session.current_joints)

    def set_selected_ghost_joints(self, joints: Sequence[float]) -> bool:
        robot = self.session.selected_robot
        if robot is None:
            return False
        ghost = self.urdfs.get(f"{robot}:ghost")
        if ghost is None:
            return False
        self.set_urdf_joints(ghost, joints)
        return True

    def render_plan_path(self, path: Sequence[JointState], poses: Sequence[Pose | None]) -> None:
        if self.server is None:
            return
        positions = [
            [float(pose.position.x), float(pose.position.y), float(pose.position.z)]
            for pose in poses
            if pose is not None
        ]
        if "plan_path" in self.handles:
            self.handles["plan_path"].remove()
            self.handles.pop("plan_path", None)
        if len(positions) >= 2:
            self.handles["plan_path"] = self.server.scene.add_line_segments(
                "/plans/path",
                points=[[start, end] for start, end in itertools.pairwise(positions)],
                colors=(80, 180, 255),
            )

    def animate_ghost_path(self, path: Sequence[JointState], duration: float, fps: float) -> bool:
        if self.session.selected_robot is None or not path:
            return False
        return PreviewAnimator(self.set_selected_ghost_joints).animate(path, duration, fps)

    def prepared_urdf_path(self, info: dict[str, Any]) -> Path:
        package_paths = {
            package: Path(path) for package, path in (info.get("package_paths") or {}).items()
        }
        return Path(
            prepare_urdf_for_drake(
                Path(str(info["model_path"])),
                package_paths=package_paths,
                xacro_args={
                    str(key): str(value) for key, value in (info.get("xacro_args") or {}).items()
                },
            )
        )

    def set_urdf_joints(self, urdf: Any, joints: Sequence[float]) -> None:
        joint_names = list((self.session.robot_info or {}).get("joint_names") or [])
        named_joints = self.viser_joint_configuration(urdf, joint_names, joints)
        if not named_joints:
            return
        if hasattr(urdf, "update_cfg"):
            urdf.update_cfg(named_joints)
        elif hasattr(urdf, "update_configuration"):
            urdf.update_configuration(named_joints)

    def viser_joint_configuration(
        self, urdf: Any, joint_names: Sequence[str], joints: Sequence[float]
    ) -> list[float]:
        allowed_names = list(self.viser_actuated_joint_names(urdf))
        if not allowed_names:
            return []
        values_by_name: dict[str, float] = {}
        for name, value in zip(joint_names, joints, strict=False):
            values_by_name[name] = float(value)
            values_by_name[name.rsplit("/", 1)[-1]] = float(value)
        return [values_by_name.get(name, 0.0) for name in allowed_names]

    def viser_actuated_joint_names(self, urdf: Any) -> tuple[str, ...]:
        wrapped_urdf = getattr(urdf, "_urdf", None)
        names = getattr(wrapped_urdf, "actuated_joint_names", None)
        if names is not None:
            return tuple(names)
        joint_map = getattr(wrapped_urdf, "joint_map", None)
        if isinstance(joint_map, dict):
            return tuple(joint_map)
        return ()

    def set_target_visual_state(self, feasible: bool) -> None:
        color = (0, 180, 255) if feasible else (255, 40, 40)
        mesh_color = GOAL_ROBOT_FEASIBLE_COLOR if feasible else GOAL_ROBOT_INFEASIBLE_COLOR
        mesh_opacity = GOAL_ROBOT_FEASIBLE_OPACITY if feasible else GOAL_ROBOT_INFEASIBLE_OPACITY
        robot = self.session.selected_robot
        handles = [self.handles.get("ee_control")]
        if robot is not None:
            ghost = self.urdfs.get(f"{robot}:ghost")
            handles.append(ghost)
            self.set_urdf_mesh_material(ghost, mesh_color, mesh_opacity)
        for handle in handles:
            if handle is None:
                continue
            for attr in ("color", "material_color"):
                if hasattr(handle, attr):
                    try:
                        setattr(handle, attr, color)
                    except Exception:
                        pass

    def set_urdf_mesh_material(
        self, urdf: Any | None, color: tuple[int, int, int], opacity: float
    ) -> None:
        if urdf is None:
            return
        for mesh in getattr(urdf, "_meshes", ()):
            for attr in ("color", "material_color"):
                if hasattr(mesh, attr):
                    try:
                        setattr(mesh, attr, color)
                    except Exception:
                        pass
            if hasattr(mesh, "opacity"):
                try:
                    mesh.opacity = opacity
                except Exception:
                    pass
