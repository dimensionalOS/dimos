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

"""Replay a G1 recording (``g1_sf_office.db``) onto the G1 sensor streams."""

from __future__ import annotations

from functools import partial
from typing import TYPE_CHECKING, Any

from pydantic import Field
from reactivex import timer

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.memory.cli.dataset import resolve_dataset
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory.replay import ReplayStream

logger = setup_logger()

DEFAULT_G1_DATASET = "g1_sf_office.db"


def _default_g1_dataset(model: dict[str, Any]) -> str:
    replay_db = str(getattr(model.get("g"), "replay_db", "") or "")
    if replay_db in ("", "go2_short"):
        return DEFAULT_G1_DATASET
    return replay_db


class G1ReplayConfig(ModuleConfig):
    dataset: str = Field(default_factory=_default_g1_dataset)
    speed: float = 1.0
    loop: bool = False
    seek: float | None = None
    duration: float | None = None
    replay_lidar: bool = False


class G1Replay(Module):
    """Replays a ``G1Recorder`` SQLite recording onto the live G1 stream names."""

    config: G1ReplayConfig
    dedicated_worker = True

    color_image: Out[Image]
    realsense_depth_image: Out[Image]
    realsense_camera_info: Out[CameraInfo]
    realsense_depth_camera_info: Out[CameraInfo]
    pointlio_odometry: Out[Odometry]
    pointlio_lidar: Out[PointCloud2]
    tf: Out[TFMessage]

    _store: SqliteStore | None = None

    @rpc
    def start(self) -> None:
        super().start()
        path = resolve_dataset(self.config.dataset)
        store = SqliteStore(path=str(path), must_exist=True)
        store.start()
        self._store = store
        replay = store.replay(
            speed=self.config.speed,
            loop=self.config.loop,
            seek=self.config.seek,
            duration=self.config.duration,
        )
        available = set(replay.list_streams())
        streams: dict[str, ReplayStream[Any]] = {}
        for name in self.outputs:
            if name == "pointlio_lidar" and not self.config.replay_lidar:
                continue
            if name in available:
                streams[name] = replay.stream(name)

        starts = [t for s in streams.values() if (t := s.first_ts()) is not None]
        replay.pin_anchor(min(starts) if starts else None)

        for info_name in ("realsense_camera_info", "realsense_depth_camera_info"):
            if info_name in streams:
                first_info = streams[info_name].first()
                if first_info is not None:
                    port = getattr(self, info_name)
                    port.publish(first_info)
                    self.register_disposable(
                        timer(0.5).subscribe(partial(_publish_once, port, first_info))
                    )

        for name, stream in streams.items():
            port = getattr(self, name)
            logger.info(
                "G1Replay: %s:%s (%d frames at %.1fx)",
                path.name,
                name,
                stream.count(),
                self.config.speed,
            )
            self.register_disposable(stream.observable().subscribe(port.publish))

    @rpc
    def stop(self) -> None:
        super().stop()
        if self._store is not None:
            self._store.stop()
            self._store = None


def _publish_once(port: Out[Any], msg: Any, _tick: int) -> None:
    port.publish(msg)
