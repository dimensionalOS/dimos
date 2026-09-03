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

"""Relocalization: place the live ``world`` frame inside a prior map's ``map`` frame.

This file is the contract: every relocalizer, whatever it matches, loads a
prior map and answers with a ``world -> map`` transform, so it publishes the
same two things - that transform on ``tf`` and the placed prior map on
``loaded_map``. All of that lives here: ``map_file`` is read into
``self.premap``, republished on ``loaded_map`` once a fix can resolve its
frame, and :meth:`RelocalizationModule.submit` takes the transform.

An implementation reads ``self.premap`` in its own ``start()`` (after
``super().start()``, and ``None`` means no map was configured), builds
whatever it matches against, and drives itself from its own inputs. It owns
what the base cannot know: which ports it listens on, when to attempt a fix,
and how good a fix must be. Matching lidar against the premap's points,
apriltags against tag poses baked into it and GPS against a datum share none
of that. See ``lidar/module.py``, the pointcloud runtime.

Whether a fix is good enough is the implementation's call, made against its
own config. A second threshold here would be a second place to configure one
decision, and the two would drift.

A dual strategy is a subclass of two implementations: ports merge across the
MRO and ``start()`` chains through ``super()``.
"""

from __future__ import annotations

from typing import Any

import reactivex as rx
from reactivex import Subject, operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

FRAME_MAP = "map"

PUBLISH_INTERVAL = 2.0  # TF and loaded_map republish period
MAP_SUFFIX = ".pc2.lcm"


class Config(ModuleConfig):
    # Premap stem or path, e.g. `--map-file=go2_hongkong_office_twopass_map`;
    # `.pc2.lcm` is appended if absent. Without one the module runs but never
    # attempts a fix.
    map_file: str | None = None
    # What the live fixed frame is called - whatever the odometry that feeds
    # this stack roots its tf at. The fix is published as an edge from it.
    world_frame: str = "world"
    publish_loaded_map: bool = False
    # Stop attempting once a fix is accepted. A premap fix does not go stale
    # the way odometry does - the accepted transform keeps being republished
    # either way - so carrying on only spends CPU and gives a later, worse
    # attempt a chance to overwrite a good answer. Set False where the robot
    # is expected to drift far enough that the fix must be re-earned.
    relocalize_once: bool = False


class RelocalizationModule(Module):
    config: Config
    tf: Out[TFMessage]
    loaded_map: Out[PointCloud2]

    _placed: bool = False

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._world_to_map: Subject[Transform] = Subject()
        # The prior map, for implementations to match against. Set by start().
        self.premap: PointCloud2 | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            rx.interval(PUBLISH_INTERVAL)
            .pipe(ops.with_latest_from(self._world_to_map))
            .subscribe(lambda pair: self.tf.publish(TFMessage(pair[1].now())))
        )
        if not self.config.map_file:
            logger.info("Relocalization module disabled (no map_file configured)")
            return
        self._load_premap(self.config.map_file)
        logger.info(f"Relocalization module started: map_file={self.config.map_file!r}")

    def _load_premap(self, map_file: str) -> None:
        premap = PointCloud2.lcm_decode(resolve_named_path(map_file, MAP_SUFFIX).read_bytes())
        # The premap *is* the map frame - it defines where `map` is.
        premap.frame_id = FRAME_MAP
        self.premap = premap
        if self.config.publish_loaded_map:
            # Gated on a fix: until one ties `map` to `world` there is nothing
            # downstream that can resolve the frame this cloud is stamped with.
            self.register_disposable(
                rx.interval(PUBLISH_INTERVAL)
                .pipe(ops.with_latest_from(self._world_to_map))
                .subscribe(lambda _: self.loaded_map.publish(premap))
            )

    @property
    def placed(self) -> bool:
        """Whether any fix has been accepted yet."""
        return self._placed

    def keep_relocalizing(self) -> bool:
        """Whether to keep attempting. Implementations gate their input on this."""
        return not (self._placed and self.config.relocalize_once)

    def submit(self, tf: Transform, source: str = "") -> None:
        """Publish a ``world_frame -> map`` fix the implementation already decided to believe."""
        world = self.config.world_frame
        assert (tf.frame_id, tf.child_frame_id) == (world, FRAME_MAP), (
            f"relocalize {source}: expected {world!r} -> {FRAME_MAP!r}, "
            f"got {tf.frame_id!r} -> {tf.child_frame_id!r}"
        )
        logger.info(f"relocalize {source}: TF {world!r} -> {FRAME_MAP!r} t={tf.translation}")
        self._world_to_map.on_next(tf)
        if not self._placed and self.config.relocalize_once:
            logger.info(f"relocalize {source}: placed, no further attempts (relocalize_once)")
        self._placed = True
