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

"""Replay a memory2 recording's camera streams as if the camera were live."""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
import itertools
import time
from typing import Any

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.memory2.replay import resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# tf messages scanned for the mount tree before the replay starts.
MOUNT_SCAN_MESSAGES = 200
MOUNT_REPUBLISH_SECONDS = 1.0


class RecordingReplayConfig(ModuleConfig):
    # Recording name resolved by resolve_db_path (an absolute path also works).
    dataset: str = "sf_office1"
    speed: float = 1.0
    loop: bool = False
    # Recorded streams replayed onto the `image` output; the consumer tells the
    # imagers apart by frame_id, exactly as with a live camera.
    image_streams: list[str] = [
        "realsense_infra_left",
        "realsense_infra_right",
    ]
    camera_info_streams: list[str] = [
        "realsense_infra_left_camera_info",
        "realsense_infra_right_camera_info",
    ]
    # Camera infos and mount tf repeat this long before frames start, so a consumer
    # that builds state off them (a tracker's rig) is ready for the first frame.
    settle_s: float = 3.0


class RecordingReplay(Module):
    """Publish a recording's images, camera infos and mount tf on the live topics.

    All replayed streams share one wall-clock anchor (:meth:`Store.replay`), so the
    stereo pair stays paired. The mount tree is taken from the recording's own tf
    stream, dropping every moving ``world -> *`` edge: that is odometry, not
    calibration. Mount stamps are wall clock because a static edge stamped with the
    recording's time would age out of any live tf buffer instantly.
    """

    config: RecordingReplayConfig

    image: Out[Image]
    camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    async def main(self) -> AsyncIterator[None]:
        store = SqliteStore(path=str(resolve_db_path(self.config.dataset)), must_exist=True)
        await asyncio.to_thread(store.start)
        self._store = store
        # Typed handles created first, so the replay view reuses them for decoding.
        for name in self.config.image_streams:
            store.stream(name, Image)
        for name in self.config.camera_info_streams:
            store.stream(name, CameraInfo)
        store.stream("tf", TFMessage)

        infos = [
            store.stream(name, CameraInfo).first().data for name in self.config.camera_info_streams
        ]
        mount = await asyncio.to_thread(self._mount_edges, store)
        logger.info(
            "replaying %s: %d imagers, mount %s",
            self.config.dataset,
            len(self.config.image_streams),
            [edge.child_frame_id for edge in mount],
        )

        settle_rounds = 5
        for _ in range(settle_rounds):
            self._publish_mount(mount)
            for info in infos:
                info.ts = time.time()
                self.camera_info.publish(info)
            await asyncio.sleep(self.config.settle_s / settle_rounds)

        replay = self._store.replay(speed=self.config.speed, loop=self.config.loop)
        for name in self.config.image_streams:
            self.register_disposable(replay.stream(name).observable().subscribe(self.image.publish))
        for name in self.config.camera_info_streams:
            self.register_disposable(
                replay.stream(name).observable().subscribe(self.camera_info.publish)
            )
        self._mount_task = asyncio.create_task(self._republish_mount(mount))
        yield
        self._mount_task.cancel()
        await asyncio.to_thread(store.stop)

    def _mount_edges(self, store: SqliteStore) -> list[Transform]:
        by_child: dict[str, Transform] = {}
        tf_stream: Any = iter(store.stream("tf", TFMessage))
        for observation in itertools.islice(tf_stream, MOUNT_SCAN_MESSAGES):
            for transform in observation.data.transforms:
                if transform.frame_id != "world":
                    by_child[transform.child_frame_id] = transform
        return list(by_child.values())

    def _publish_mount(self, edges: list[Transform]) -> None:
        now = time.time()
        for edge in edges:
            edge.ts = now
        self.tf.publish(TFMessage(*edges))

    async def _republish_mount(self, edges: list[Transform]) -> None:
        while True:
            self._publish_mount(edges)
            await asyncio.sleep(MOUNT_REPUBLISH_SECONDS)
