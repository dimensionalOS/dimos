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

"""Replay a recording's lidar and tf onto the transports, as the sensors would publish them."""

from __future__ import annotations

from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.memory.replay import ReplayStream
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class RecordingPlayerConfig(ModuleConfig):
    dataset: str = "go2_mid360_sf_office_outdoors_2026-05-29"  # recording stem or path; `.db`, LFS-fetched on miss
    stream: str = "pointlio_lidar"
    speed: float = 1.0
    seek: float | None = None
    duration: float | None = None
    # Frame to stamp the scans with, for a recording whose scan frame is not in its tf tree.
    lidar_frame: str | None = None


class RecordingPlayer(Module):
    """Replay a recording's lidar and tf exactly as the sensors published them.

    Each scan is stamped with the time it was recorded at, which is the clock
    the tf stream is on. Transforms the recording holds a single sample of are
    static mounts, and they ride along with every tf message so a consumer that
    matches transforms to scans by time keeps resolving them.
    """

    config: RecordingPlayerConfig
    lidar: Out[PointCloud2]
    tf: Out[TFMessage]

    @rpc
    def start(self) -> None:
        super().start()
        path = resolve_named_path(self.config.dataset, ".db")
        store = self.register_disposable(SqliteStore(path=str(path), must_exist=True))
        store.start()
        replay = store.replay(
            speed=self.config.speed,
            seek=self.config.seek,
            duration=self.config.duration,
        )
        lidar: ReplayStream[PointCloud2] = replay.stream(self.config.stream)
        logger.info(
            f"Replaying {path.name}:{self.config.stream} "
            f"({lidar.count()} frames at {self.config.speed}x)"
        )
        self.register_disposable(lidar.observable_ts().subscribe(self._publish_scan))

        statics = self._static_transforms(store.streams.tf)
        if statics:
            logger.info(
                "re-announcing %d static transforms with every tf message: %s",
                len(statics),
                ", ".join(f"{t.frame_id}->{t.child_frame_id}" for t in statics),
            )
        tf: ReplayStream[TFMessage] = replay.stream("tf")
        self.register_disposable(
            tf.observable().subscribe(lambda msg: self._publish_tf(msg, statics))
        )

    def _publish_scan(self, stamped: tuple[float, PointCloud2]) -> None:
        ts, cloud = stamped
        cloud.ts = ts
        if self.config.lidar_frame:
            cloud.frame_id = self.config.lidar_frame
        self.lidar.publish(cloud)

    def _publish_tf(self, msg: TFMessage, statics: list[Transform]) -> None:
        if statics and msg.transforms:
            stamp = msg.transforms[0].ts
            msg = TFMessage(
                *msg.transforms,
                *(
                    Transform(t.translation, t.rotation, t.frame_id, t.child_frame_id, ts=stamp)
                    for t in statics
                ),
            )
        self.tf.publish(msg)

    @staticmethod
    def _static_transforms(tf_stream: Any) -> list[Transform]:
        """The transforms with exactly one sample in the recording."""
        samples: dict[tuple[str, str], int] = {}
        first: dict[tuple[str, str], Transform] = {}
        for obs in tf_stream:
            for transform in obs.data.transforms:
                key = (transform.frame_id, transform.child_frame_id)
                samples[key] = samples.get(key, 0) + 1
                first.setdefault(key, transform)
        return [first[key] for key, count in samples.items() if count == 1]
