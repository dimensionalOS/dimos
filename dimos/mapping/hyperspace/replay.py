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

"""Replay a recorded RGB-D session onto the streams hyperspace listens to.

Plays a memory recording's colour, depth, both camera infos and tf back onto
Out ports named exactly like a live camera driver's, so ``HyperspacePatches``
wires up by name with no robot present.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.memory.replay import resolve_db_path
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

logger = setup_logger()

# Out port -> recorded stream name.
DEFAULT_STREAMS = {
    "color_image": "color_image",
    "depth_image": "depth_image",
    "camera_info": "camera_info",
    "depth_camera_info": "depth_camera_info",
    "tf": "tf",
}


class HyperspaceReplayConfig(ModuleConfig):
    """Where to read the recording from and how to play it back."""

    db_path: str = ""
    speed: float = 1.0
    loop: bool = False
    seek: float | None = None
    duration: float | None = None
    # Rename recorded streams: {port name: stream name in the db}.
    streams: dict[str, str] = {}


class HyperspaceReplay(Module):
    """Replays colour, depth, camera infos and tf from a memory recording."""

    config: HyperspaceReplayConfig
    dedicated_worker = True

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    async def main(self) -> AsyncIterator[None]:
        if not self.config.db_path:
            raise ValueError("HyperspaceReplay needs db_path (a memory .db recording)")
        db_path = resolve_db_path(Path(self.config.db_path).expanduser())
        logger.info(f"hyperspace replay: {db_path} at {self.config.speed}x")
        store = SqliteStore(path=str(db_path), must_exist=True)
        store.start()
        self.register_disposable(store)
        replay = store.replay(
            speed=self.config.speed,
            loop=self.config.loop,
            seek=self.config.seek,
            duration=self.config.duration,
        )
        available = set(replay.list_streams())
        for port, default_name in DEFAULT_STREAMS.items():
            name = self.config.streams.get(port, default_name)
            if name not in available:
                logger.warning(
                    f"hyperspace replay: stream {name!r} missing from recording; {port} stays silent"
                )
                continue
            self.register_disposable(
                replay.stream(name).observable().subscribe(getattr(self, port).publish)
            )
        yield
