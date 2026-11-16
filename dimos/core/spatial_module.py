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

from pydantic import Field

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


class SpatialModule(ModuleBase):
    """Publishes and looks up only the tf frames it declares in ``tf_frames_local``/``tf_frames_global``."""

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
