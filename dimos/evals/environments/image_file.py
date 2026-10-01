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

from dimos_generated.sensor_msgs.msg import Image
import numpy as np
from PIL import Image as PILImage

from dimos.evals.environments.base import Environment
from dimos.evals.types import RunningEnvironment
from dimos.memory.store.memory import MemoryStore
from dimos.msgs.image import image_from_array
from dimos.msgs.time import header_now, to_seconds
from dimos.protocol.service.spec import BaseConfig

if TYPE_CHECKING:
    from dimos.evals.agents.base import Agent


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
        with PILImage.open(self.config.path) as source:
            pixels = np.asarray(source.convert("RGB"))
        image = image_from_array(pixels, encoding="rgb8", header=header_now())
        recording = MemoryStore()
        self._resources.callback(recording.stop)
        stream = recording.stream("image", Image)
        stream.append(image, ts=to_seconds(image.header.stamp))
        return RunningEnvironment(
            mcp_url="", streams=(stream,), artifacts={"image": self.config.path}
        )
