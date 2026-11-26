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

from functools import cached_property
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


class TfModule(Module):
    """Its ``tf`` port and ``tfbuffer`` publish only the edges in ``tf_edges`` and look up only
    declared frames, unless ``skip_tf_checks``."""

    config: TfModuleConfig
    tf: IO[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        if self.config.skip_tf_checks:
            return
        # The port stays the IO Module bound; only its publish gains the edge check.
        publish = self.tf.publish

        def checked_publish(msg: TFMessage) -> None:
            for transform in msg.transforms:
                if (transform.frame_id, transform.child_frame_id) not in self.tf_edge_ids:
                    raise ValueError(
                        f"{self.config.instance_name or type(self).__name__} publishes undeclared "
                        f"tf edge ({transform.frame_id!r}, {transform.child_frame_id!r}); "
                        "add it to tf_edges"
                    )
            publish(msg)

        self.tf.publish = checked_publish  # type: ignore[method-assign]

    @cached_property
    def tf_frames(self) -> dict[str, str]:
        """Every role -> frame id, local and global together."""
        return {**self.config.tf_frames_global, **self.config.tf_frames_local}

    @cached_property
    def tf_edge_ids(self) -> frozenset[tuple[str, str]]:
        """The declared edges as (parent frame id, child frame id)."""
        frames = self.tf_frames
        return frozenset((frames[parent], frames[child]) for parent, child in self.config.tf_edges)

    def _create_tfbuffer(self, port: Any) -> TF:
        name = self.config.instance_name or type(self).__name__
        buffer = TF(
            port, edges=None if self.config.skip_tf_checks else self.tf_edge_ids, module_name=name
        )
        if self.config.skip_tf_checks:
            return buffer
        get = buffer.get

        def checked_get(parent_frame: str, child_frame: str, *args: Any, **kwargs: Any) -> Any:
            for frame in (parent_frame, child_frame):
                if frame not in self.tf_frames.values():
                    raise ValueError(
                        f"{name} looks up undeclared tf frame {frame!r}; "
                        "add it to tf_frames_local or tf_frames_global"
                    )
            return get(parent_frame, child_frame, *args, **kwargs)

        buffer.get = checked_get  # type: ignore[method-assign]
        return buffer
