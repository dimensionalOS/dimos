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
"""Configuration-resolved module IO demo.

Run:

    uv run python examples/configuration_resolved_io.py

This demo uses one reusable module implementation whose final config selects the
stream direction/type pairing:

- ``text_to_number`` exposes ``text: In[str]`` and ``number: Out[int]``
- ``number_to_text`` exposes ``number: In[int]`` and ``text: Out[str]``

Two configured instances connect into a small loop. The script lets the
blueprint run briefly, then shuts the whole blueprint down with
``coordinator.stop()``.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from contextlib import suppress
import time
from typing import Literal

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module, ModuleConfig, ModuleIOContract, StreamDecl
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

Mode = Literal["text_to_number", "number_to_text"]


class AlternatingIOConfig(ModuleConfig):
    mode: Mode
    interval_s: float = 0.25


class AlternatingIO(Module):
    """Module whose input/output stream types are selected by config."""

    config: AlternatingIOConfig  # type: ignore[assignment]

    @classmethod
    def io_contract(cls, config: AlternatingIOConfig) -> ModuleIOContract:
        if config.mode == "text_to_number":
            return ModuleIOContract(
                streams=(
                    StreamDecl(name="text", direction="in", type=str),
                    StreamDecl(name="number", direction="out", type=int),
                )
            )

        return ModuleIOContract(
            streams=(
                StreamDecl(name="number", direction="in", type=int),
                StreamDecl(name="text", direction="out", type=str),
            )
        )

    async def main(self) -> AsyncIterator[None]:
        """Start a background publisher without overriding start()/stop()."""
        publisher = asyncio.create_task(self._publish_loop())
        try:
            yield
        finally:
            publisher.cancel()
            with suppress(asyncio.CancelledError):
                await publisher

    async def _publish_loop(self) -> None:
        counter = 0
        while True:
            counter += 1
            outputs = self.outputs
            if "text" in outputs:
                outputs["text"].publish(f"tick-{counter}")
            if "number" in outputs:
                outputs["number"].publish(counter)
            await asyncio.sleep(self.config.interval_s)

    async def handle_text(self, message: str) -> None:
        logger.info("Received text", module=type(self).__name__, message=message)

    async def handle_number(self, message: int) -> None:
        logger.info("Received number", module=type(self).__name__, message=message)


class TextToNumber(AlternatingIO):
    pass


class NumberToText(AlternatingIO):
    pass


def main() -> None:
    blueprint = autoconnect(
        TextToNumber.blueprint(mode="text_to_number"),
        NumberToText.blueprint(mode="number_to_text"),
    )

    coordinator = ModuleCoordinator.build(blueprint, {"g": {"viewer": "none"}})
    try:
        print("Blueprint running for 2 seconds...")
        time.sleep(2.0)
    finally:
        print("Stopping blueprint...")
        coordinator.stop()


if __name__ == "__main__":
    main()
