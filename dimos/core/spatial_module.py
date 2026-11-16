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

"""A module whose tf frames are declared in its config, so a blueprint can namespace and check them."""

from typing import TypeAlias

from pydantic import Field, model_validator

from dimos.core.module import ModuleBase, ModuleConfig
from dimos.msgs.geometry_msgs.Transform import Transform

# A frame that belongs to one robot (base_link, lidar); a namespaced blueprint prefixes it.
TransformFrameLocal: TypeAlias = str
# A frame shared by everything in the blueprint (world, map); never prefixed.
TransformFrameGlobal: TypeAlias = str


class SpatialModuleConfig(ModuleConfig):
    # Role -> frame id, e.g. {"body": "base_link"}. Code reads frames from here, never from literals.
    tf_frames_local: dict[str, TransformFrameLocal] = Field(default_factory=dict)
    tf_frames_global: dict[str, TransformFrameGlobal] = Field(default_factory=dict)
    # (parent role, child role) for every edge this module publishes; tf_publish refuses any other.
    tf_edges: list[tuple[str, str]] = Field(default_factory=list)

    @model_validator(mode="after")
    def _check_roles(self) -> "SpatialModuleConfig":
        shared = sorted(self.tf_frames_local.keys() & self.tf_frames_global.keys())
        if shared:
            raise ValueError(f"tf roles {shared} are in both tf_frames_local and tf_frames_global")
        roles = self.tf_frames_local.keys() | self.tf_frames_global.keys()
        unknown = sorted({role for edge in self.tf_edges for role in edge} - roles)
        if unknown:
            raise ValueError(
                f"tf_edges use undeclared tf roles {unknown} (declared: {sorted(roles)})"
            )
        return self

    @property
    def tf_edge_frames(self) -> set[tuple[str, str]]:
        frames = {**self.tf_frames_global, **self.tf_frames_local}
        return {(frames[parent], frames[child]) for parent, child in self.tf_edges}


class SpatialModule(ModuleBase):
    """Looks up only the tf frames it declares, and publishes only the edges in ``tf_edges``."""

    config: SpatialModuleConfig

    @property
    def tf_frames(self) -> set[str]:
        return {*self.config.tf_frames_local.values(), *self.config.tf_frames_global.values()}

    def _require_declared(self, *frames: str) -> None:
        undeclared = sorted(set(frames) - self.tf_frames)
        if undeclared:
            raise ValueError(
                f"{type(self).__name__} uses undeclared tf frames {undeclared}; "
                f"add them to tf_frames_local or tf_frames_global (declared: {sorted(self.tf_frames)})"
            )

    def tf_publish(self, *transforms: Transform) -> None:
        self._require_declared(*(f for t in transforms for f in (t.frame_id, t.child_frame_id)))
        undeclared = sorted(
            {(t.frame_id, t.child_frame_id) for t in transforms} - self.config.tf_edge_frames
        )
        if undeclared:
            raise ValueError(
                f"{type(self).__name__} publishes undeclared tf edges {undeclared}; add them to tf_edges"
            )
        self.tfbuffer.publish(*transforms)

    def tf_get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
        *,
        forward_tolerance: float = 0.0,
        warn: bool = True,
    ) -> Transform | None:
        self._require_declared(parent_frame, child_frame)
        return self.tfbuffer.get(
            parent_frame,
            child_frame,
            time_point,
            time_tolerance,
            forward_tolerance=forward_tolerance,
            warn=warn,
        )
