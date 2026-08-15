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

"""Replay a D455 recording .db as live camera/depth/tf module streams.

Each stream gets one persistent pump thread sleeping toward the next
message's wall target on a shared anchor. (ReplayStream.observable()'s
per-message timer scheduling cannot sustain a 400+ msg/s tf stream next to
50 MB/s of image pickling — tf drifts seconds behind and TF lookups fail.)
"""

from __future__ import annotations

import threading
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

#: Image frames further behind schedule than this are dropped to catch up.
_IMAGE_STALE_S = 0.25


class D455ReplayModuleConfig(ModuleConfig):
    db_path: str
    speed: float = 1.0
    loop: bool = False
    #: Recording stream name -> output port name.
    stream_map: dict[str, str] = {
        "realsense_color_image": "color_image",
        "realsense_depth_image": "depth_image",
        "realsense_color_image_camera_info": "camera_info",
        "realsense_depth_image_camera_info": "depth_camera_info",
        "tf": "tf",
    }


class D455ReplayModule(Module):
    """Publish the camera streams of a recording .db in (scaled) real time."""

    config: D455ReplayModuleConfig

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    _stores: list[SqliteStore]
    _threads: list[threading.Thread]
    _stop: threading.Event
    _wall_t0: float
    _replay_t0: float

    def _pump(self, store: SqliteStore, stream_name: str, port_name: str) -> None:
        port = getattr(self, port_name)
        speed = self.config.speed
        may_drop = stream_name.endswith("_image")
        published = dropped = 0
        obs: Any
        while not self._stop.is_set():
            for obs in store.stream(stream_name):
                if self._stop.is_set():
                    return
                target = self._wall_t0 + (obs.ts - self._replay_t0) / speed
                delay = target - time.time()
                if delay > 0:
                    if self._stop.wait(delay):
                        return
                elif may_drop and -delay > _IMAGE_STALE_S:
                    dropped += 1
                    continue
                try:
                    port.publish(obs.data)
                    published += 1
                except Exception:
                    logger.exception("replay: publish failed on %s", stream_name)
            if not self.config.loop:
                break
            # Re-anchor and play the stream again from the top.
            self._wall_t0 = time.time()
        logger.info(
            "replay %s done: %d published, %d dropped-late", stream_name, published, dropped
        )

    @rpc
    def start(self) -> None:
        super().start()
        self._stop = threading.Event()
        self._stores = []
        self._threads = []

        index_store = SqliteStore(path=self.config.db_path)
        available = set(index_store.list_streams())
        streams = {name: port for name, port in self.config.stream_map.items() if name in available}
        for missing in set(self.config.stream_map) - set(streams):
            logger.warning("replay: stream %r not in %s", missing, self.config.db_path)

        first_ts = [index_store.stream(name).first().ts for name in streams]
        index_store.stop()
        self._replay_t0 = min(first_ts)
        self._wall_t0 = time.time() + 0.5  # grace so all pumps start in sync

        for stream_name, port_name in streams.items():
            # One store (and sqlite connection) per stream: pumps run on
            # different threads and must not share a connection.
            store = SqliteStore(path=self.config.db_path)
            self._stores.append(store)
            thread = threading.Thread(
                target=self._pump,
                args=(store, stream_name, port_name),
                name=f"replay_{stream_name}",
                daemon=True,
            )
            self._threads.append(thread)
            thread.start()
        logger.info(
            "replaying %s at %gx (%s)",
            self.config.db_path,
            self.config.speed,
            "loop" if self.config.loop else "once",
        )

    @rpc
    def stop(self) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=5.0)
        self._threads = []
        for store in self._stores:
            store.stop()
        self._stores = []
        super().stop()
