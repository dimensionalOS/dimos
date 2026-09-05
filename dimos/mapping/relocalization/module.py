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
from reactivex import Observable, Subject, operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MAP_SUFFIX = ".pc2.lcm"


def fix_stream(fixes: Observable[Transform], interval: float) -> Observable[Transform]:
    """Every accepted fix as it lands, then again every ``interval`` s (once only if <= 0)."""

    def now_and_again(fix: Transform) -> Observable[Transform]:
        again = rx.interval(interval).pipe(ops.map(lambda _: fix)) if interval > 0 else rx.empty()
        return rx.concat(rx.of(fix), again)

    return fixes.pipe(ops.switch_map(now_and_again))


class Config(ModuleConfig):
    # Premap stem or path, e.g. `--map-file=go2_hongkong_office_twopass_map`;
    # `.pc2.lcm` is appended if absent. Without one the module runs but never
    # attempts a fix.
    map_file: str | None = None
    # What the live fixed frame is called, whatever the odometry is in
    world_frame: str = "world"
    # frame for a loaded premap
    map_frame: str = "map"
    # Seconds between tf republishes of the accepted fix. The fix itself does
    # not change but tf is not latched,
    tf_interval: float = 10.0
    # Seconds between `loaded_map` republishes. A premap does not change, so
    # one publish is all the information there is - but the topic is not
    # latched. Zero for the one publish.
    republish_loaded_map: float = 0.0
    # Stop attempting once a fix is accepted.
    relocalize_once: bool = True


class RelocalizationModule(Module):
    config: Config
    tf: Out[TFMessage]
    loaded_map: Out[PointCloud2]

    _placed: bool = False

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.fixes: Subject[Transform] = Subject()
        self.premap: PointCloud2 | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            fix_stream(self.fixes, self.config.tf_interval).subscribe(
                lambda tf: self.tf.publish(TFMessage(tf.now()))
            )
        )
        if not self.config.map_file:
            logger.info("Relocalization module disabled (no map_file configured)")
            return
        self._load_premap(self.config.map_file)
        logger.info(f"Relocalization module started: map_file={self.config.map_file!r}")

    def _load_premap(self, map_file: str) -> None:
        # A path, a stem in the working directory or project root, or an LFS
        # name, which is pulled and decompressed rather than reported missing.
        path = resolve_named_path(map_file, MAP_SUFFIX)
        premap = PointCloud2.lcm_decode(path.read_bytes())

        premap.frame_id = self.config.map_frame
        self.premap = premap

        self.register_disposable(
            fix_stream(self.fixes, self.config.republish_loaded_map).subscribe(
                lambda _: self.loaded_map.publish(premap)
            )
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
        world, map_frame = self.config.world_frame, self.config.map_frame
        assert (tf.frame_id, tf.child_frame_id) == (world, map_frame), (
            f"relocalize {source}: expected {world!r} -> {map_frame!r}, "
            f"got {tf.frame_id!r} -> {tf.child_frame_id!r}"
        )
        logger.info(f"relocalize {source}: TF {world!r} -> {map_frame!r} t={tf.translation}")
        self.fixes.on_next(tf)
        if not self._placed and self.config.relocalize_once:
            logger.info(f"relocalize {source}: placed, no further attempts (relocalize_once)")
        self._placed = True
