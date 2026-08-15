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

"""Replay a D455 recording .db as live camera/depth/tf module streams."""

from __future__ import annotations

from reactivex import operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


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

    @rpc
    def start(self) -> None:
        super().start()
        self._stores = []
        index_store = SqliteStore(path=self.config.db_path)
        available = set(index_store.list_streams())
        index_store.stop()

        def _on_error(e: Exception) -> None:
            logger.error("replay pipeline error: %s", e, exc_info=True)

        for stream_name, port_name in self.config.stream_map.items():
            if stream_name not in available:
                logger.warning("replay: stream %r not in %s", stream_name, self.config.db_path)
                continue
            # One store (and sqlite connection) per stream: the observables
            # iterate on different threads and must not share a connection.
            store = SqliteStore(path=self.config.db_path)
            self._stores.append(store)
            replay = store.replay(speed=self.config.speed, loop=self.config.loop)
            port = getattr(self, port_name)
            # ReplayStream.observable() emits decoded payloads on the replay
            # clock — feed them straight into the module port. retry() rides
            # out transient read errors: the resubscribe skips forward to the
            # current replay position via the shared anchor.
            self.register_disposable(
                replay.streams[stream_name]
                .observable()
                .pipe(ops.retry(3))
                .subscribe(on_next=port.publish, on_error=_on_error)
            )
        logger.info(
            "replaying %s at %gx (%s)",
            self.config.db_path,
            self.config.speed,
            "loop" if self.config.loop else "once",
        )

    @rpc
    def stop(self) -> None:
        super().stop()  # dispose stream subscriptions before closing the dbs
        for store in self._stores:
            store.stop()
        self._stores = []
