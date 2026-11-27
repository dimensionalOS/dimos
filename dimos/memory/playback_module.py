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

"""Base class for modules that publish a recording onto their ``Out`` ports."""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.memory.replay import Playback, Replay

if TYPE_CHECKING:
    from dimos.memory.store.base import Store


class PlaybackModuleConfig(ModuleConfig):
    speed: float = 1.0
    loop: bool = False
    seek: float | None = None
    duration: float | None = None
    # Decoded frames read ahead of the publisher; start() returns once this many are ready.
    buffer_size: int = Field(default=128, gt=0)
    # Seconds the publisher may fall behind schedule before it warns.
    lag_warning: float = 0.5


class PlaybackModule(Module):
    """Plays the :class:`Playback` that :meth:`open_playback` builds.

    ``start()`` fills its buffer before returning, the clock starts in
    ``on_system_modules_ready`` once every subscriber is listening, and
    ``stop()`` stops it before a subclass closes its store.
    """

    config: PlaybackModuleConfig
    _playback: Playback | None = None

    def replay_of(self, store: Store) -> Replay:
        """*store* played with this module's speed, loop, window and buffer."""
        config = self.config
        return store.replay(
            speed=config.speed,
            loop=config.loop,
            seek=config.seek,
            duration=config.duration,
            buffer_size=config.buffer_size,
            lag_warning=config.lag_warning,
        )

    def open_playback(self) -> Playback:
        """Open the recording and add its streams, each subscribed to its port."""
        raise NotImplementedError

    @rpc
    def start(self) -> None:
        super().start()
        self._playback = self.open_playback()
        self._playback.load()

    @rpc
    def on_system_modules_ready(self, _modules: list[RPCClient]) -> None:
        if self._playback is not None:
            self._playback.play()

    @rpc
    def stop(self) -> None:
        if self._playback is not None:
            self._playback.stop()
        super().stop()
