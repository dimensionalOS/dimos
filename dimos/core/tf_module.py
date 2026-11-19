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

from typing import Any, TypeAlias

from pydantic import Field, model_validator

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import TF

# A frame that belongs to one robot (base_link, lidar); a namespaced blueprint prefixes it.
TransformFrameLocal: TypeAlias = str
# A frame shared by everything in the blueprint (world, map); never prefixed.
TransformFrameGlobal: TypeAlias = str


class TfModuleConfig(ModuleConfig):
    # Role -> frame id, e.g. {"body": "base_link"}. Code reads frames from here, never from literals.
    tf_frames_local: dict[str, TransformFrameLocal] = Field(default_factory=dict)
    tf_frames_global: dict[str, TransformFrameGlobal] = Field(default_factory=dict)
    # (parent role, child role) for every edge this module publishes; its tfbuffer refuses any other.
    tf_edges: list[tuple[str, str]] = Field(default_factory=list)
    # Opt this module out of tf checks (its tfbuffer publishes any edge).
    skip_tf_checks: bool = False

    @model_validator(mode="after")
    def _check_roles(self) -> "TfModuleConfig":
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
    def tf_frames(self) -> dict[str, str]:
        """Every role -> frame id, local and global together."""
        return {**self.tf_frames_global, **self.tf_frames_local}

    @property
    def tf_edge_frames(self) -> set[tuple[str, str]]:
        frames = self.tf_frames
        return {(frames[parent], frames[child]) for parent, child in self.tf_edges}


class TfFrames:
    """A module's frame ids by role, as attributes: ``self.frame.body``."""

    def __init__(self, frames: dict[str, str], owner: str) -> None:
        self._frames = frames
        self._owner = owner

    def __getattr__(self, role: str) -> str:
        if role.startswith("_"):
            raise AttributeError(role)
        return self[role]

    def __getitem__(self, role: str) -> str:
        if role not in self._frames:
            raise AttributeError(
                f"{self._owner} has no tf frame for role {role!r}; "
                f"declare it in tf_frames_local or tf_frames_global (declared: {sorted(self._frames)})"
            )
        return self._frames[role]


class TfModule(Module):
    """Its ``tfbuffer`` publishes only the edges in ``tf_edges``, unless ``skip_tf_checks``."""

    config: TfModuleConfig
    tf: IO[TFMessage]

    @property
    def frame(self) -> TfFrames:
        """Frame ids by role, local and global together: ``self.frame.body``."""
        return TfFrames(self.config.tf_frames, self.config.instance_name or type(self).__name__)

    def _create_tfbuffer(self, port: Any) -> TF:
        return TF(
            port,
            edges=None if self.config.skip_tf_checks else self.config.tf_edge_frames,
            module_name=self.config.instance_name or type(self).__name__,
        )
