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

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, ClassVar

from dimos.evals.types import Agent, RunningEnvironment

if TYPE_CHECKING:
    from dimos.memory.store.base import Store


@dataclass
class ImageFile:
    """A standalone image question: the recording holds one ``image`` stream
    with the file in it."""

    path: Path

    artifacts: ClassVar[tuple[str, ...]] = ("image",)
    has_robot: ClassVar[bool] = False
    _recording: Store | None = field(default=None, init=False, repr=False, compare=False)

    def preflight(self, agent: Agent) -> None:
        if agent.modules:
            raise RuntimeError(
                f"ImageFile({self.path}) launches nothing; "
                f"{type(agent).__name__} adds modules {agent.modules!r}"
            )
        if not self.path.is_file():
            raise FileNotFoundError(f"image does not exist: {self.path}")

    def start(self, modules: str) -> RunningEnvironment:
        from dimos.memory.store.memory import MemoryStore
        from dimos.msgs.sensor_msgs.Image import Image

        image = Image.from_file(self.path)
        self._recording = MemoryStore()
        self._recording.stream("image", Image).append(image, ts=image.ts or 0.0)
        return RunningEnvironment(
            mcp_url="", recording=self._recording, artifacts={"image": self.path}
        )

    def stop(self) -> None:
        if self._recording is not None:
            self._recording.stop()
            self._recording = None
