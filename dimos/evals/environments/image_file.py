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

"""A standalone image question."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Any

from dimos.evals.environment import Environment
from dimos.evals.types import RunningEnvironment
from dimos.protocol.service.spec import BaseConfig

if TYPE_CHECKING:
    from dimos.evals.agent import Agent


class ImageFileConfig(BaseConfig):
    path: Path


class ImageFile(Environment):
    """A standalone image exposed as one ``image`` stream."""

    config: ImageFileConfig

    def __init__(self, path: Path, **kwargs: Any) -> None:
        super().__init__(path=path, **kwargs)

    def preflight(self, agent: Agent) -> None:
        if agent.config.modules:
            raise RuntimeError(
                f"ImageFile({self.config.path}) launches nothing; "
                f"{type(agent).__name__} adds modules {agent.config.modules!r}"
            )
        if not self.config.path.is_file():
            raise FileNotFoundError(f"image does not exist: {self.config.path}")

    def start(self, modules: Sequence[str]) -> RunningEnvironment:
        from dimos.memory.store.memory import MemoryStore
        from dimos.msgs.sensor_msgs.Image import Image

        image = Image.from_file(self.config.path)
        recording = MemoryStore()
        self._resources.callback(recording.stop)
        stream = recording.stream("image", Image)
        stream.append(image, ts=image.ts)
        return RunningEnvironment(
            mcp_url="", streams=(stream,), artifacts={"image": self.config.path}
        )
