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

"""Rerun scene wiring helpers (static attachments, URDF, pinholes).

This module is intentionally *not* a TF visualizer.
It only provides static Rerun scene setup:
- view coordinates
- attach semantic entity paths (world/robot/...) under named TF frames (base_link, camera_optical, ...)
- optional URDF logging
- optional axes gizmo + camera pinhole(s)

Dynamic TF visualization remains the responsibility of `TFRerunModule`.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any, Protocol

import rerun as rr

from dimos.core import Module, rpc
from dimos.core.global_config import GlobalConfig
from dimos.dashboard.rerun_init import connect_rerun

if TYPE_CHECKING:
    from collections.abc import Sequence


class _HasToRerun(Protocol):
    def to_rerun(self) -> Any: ...


def _attach_entity(entity_path: str, parent_frame: str) -> None:
    """Attach an entity path's implicit frame (tf#/...) under a named frame."""
    rr.log(
        entity_path,
        rr.Transform3D(
            translation=[0.0, 0.0, 0.0],
            rotation=rr.Quaternion(xyzw=[0.0, 0.0, 0.0, 1.0]),
            parent_frame=parent_frame,  # type: ignore[call-arg]
        ),
        static=True,
    )


class RerunSceneWiringModule(Module):
    """Static Rerun scene wiring for semantic entity paths."""

    _global_config: GlobalConfig

    # Semantic entity roots
    world_entity: str
    robot_entity: str
    robot_axes_entity: str

    # Named TF frames to attach to
    world_frame: str
    robot_frame: str

    # Optional assets
    urdf_path: str | Path | None
    axes_size: float | None

    # Multi-camera wiring:
    # tuple = (camera_entity_path, camera_named_frame, camera_info_static)
    cameras: Sequence[tuple[str, str, _HasToRerun]]
    camera_rgb_suffix: str

    def __init__(
        self,
        *,
        global_config: GlobalConfig | None = None,
        world_entity: str = "world",
        robot_entity: str = "world/robot",
        robot_axes_entity: str = "world/robot/axes",
        world_frame: str = "world",
        robot_frame: str = "base_link",
        urdf_path: str | Path | None = None,
        axes_size: float | None = 0.5,
        cameras: Sequence[tuple[str, str, _HasToRerun]] = (),
        camera_rgb_suffix: str = "rgb",
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._global_config = global_config or GlobalConfig()

        self.world_entity = world_entity
        self.robot_entity = robot_entity
        self.robot_axes_entity = robot_axes_entity

        self.world_frame = world_frame
        self.robot_frame = robot_frame

        self.urdf_path = urdf_path
        self.axes_size = axes_size

        self.cameras = cameras
        self.camera_rgb_suffix = camera_rgb_suffix

    @rpc
    def start(self) -> None:
        super().start()

        if not self._global_config.viewer_backend.startswith("rerun"):
            return

        connect_rerun(global_config=self._global_config)

        # Global view coordinates (applies to views at/under this origin).
        rr.log(self.world_entity, rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

        # Attach semantic entity paths to named TF frames.
        _attach_entity(self.world_entity, self.world_frame)
        _attach_entity(self.robot_entity, self.robot_frame)

        if self.axes_size is not None:
            rr.log(self.robot_axes_entity, rr.TransformAxes3D(self.axes_size), static=True)  # type: ignore[attr-defined]

        # Optional URDF load (purely visual).
        if self.urdf_path is not None:
            p = Path(self.urdf_path)
            if p.exists():
                rr.log_file_from_path(
                    str(p),
                    entity_path_prefix=self.robot_entity,
                    static=True,
                )

        # Multi-camera: attach camera entities + log static pinholes.
        for cam_entity, cam_frame, cam_info in self.cameras:
            _attach_entity(cam_entity, cam_frame)
            rr.log(cam_entity, cam_info.to_rerun(), static=True)  # type: ignore[no-untyped-call]

    @rpc
    def stop(self) -> None:
        super().stop()


rerun_scene_wiring = RerunSceneWiringModule.blueprint
