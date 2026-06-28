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

"""Repeatedly publish a fixed set of transforms onto the tf stream.

``PubSubTF`` has no ``publish_static`` (latched) path, so a one-shot publish would
be missed by anything that subscribed later — including a recorder that wants the
mount geometry captured in its tf stream. This module works around that by
re-publishing the transforms on a fixed interval from a background task, each cycle
re-stamped with the current time. Subclass and override :meth:`transforms` with the
rig's mount frames (see the go2 / realsense recording blueprints).
"""

from __future__ import annotations

import asyncio
import time

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# (name, parent_name, translation_xyz, fixed-axis rpy) — parent None marks the tree root.
FrameSpec = tuple[str, str | None, tuple[float, float, float], tuple[float, float, float]]


def frames_to_edge_transforms(frames: list[FrameSpec]) -> list[Transform]:
    """Build a ``parent -> child`` Transform for each non-root edge of a frame tree.

    This is the static mount tree (the rigid sensor offsets); a tf buffer composes
    these edges to answer any ``world <- frame`` query once odometry supplies the
    moving ``world <- root`` edge.
    """
    transforms: list[Transform] = []
    for name, parent, translation, rpy in frames:
        if parent is None:
            continue
        transforms.append(
            Transform(
                translation=Vector3(*translation),
                rotation=Quaternion.from_euler(Vector3(*rpy)),
                frame_id=parent,
                child_frame_id=name,
            )
        )
    return transforms


class StaticTfPublisherConfig(ModuleConfig):
    # How often to re-publish the static transforms onto the tf stream.
    publish_hz: float = Field(default=5.0, gt=0.0)


class StaticTfPublisher(Module):
    config: StaticTfPublisherConfig

    _running: bool = False
    _transforms: list[Transform] = []

    def transforms(self) -> list[Transform]:
        """The static transforms to publish. Override in a rig-specific subclass."""
        raise NotImplementedError(
            f"{type(self).__name__} must override transforms() with its mount frames"
        )

    @rpc
    def start(self) -> None:
        super().start()
        self._transforms = self.transforms()
        if not self._transforms:
            logger.warning("%s: no transforms to publish", type(self).__name__)
            return
        self._running = True
        self.spawn(self._publish_loop())
        logger.info(
            "%s publishing %d static transform(s) at %.1f Hz",
            type(self).__name__,
            len(self._transforms),
            self.config.publish_hz,
        )

    async def _publish_loop(self) -> None:
        period = 1.0 / self.config.publish_hz
        while self._running:
            now = time.time()
            for transform in self._transforms:
                transform.ts = now
            self.tf.publish(*self._transforms)
            await asyncio.sleep(period)

    @rpc
    def stop(self) -> None:
        self._running = False
        super().stop()
