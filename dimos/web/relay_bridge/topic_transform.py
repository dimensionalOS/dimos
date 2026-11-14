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

"""Bounded optional media worker boundary; raw forwarding imports no codecs."""

import asyncio
from collections.abc import Callable, Mapping
from contextlib import suppress
import importlib
import time
from typing import TYPE_CHECKING, Any, TypeVar, cast

if TYPE_CHECKING:
    from dimos.web.relay_bridge.topic_media import MediaOutput

_T = TypeVar("_T")


def _encode(profile: str, data: bytes, settings: Mapping[str, Any] | None) -> "MediaOutput":
    # Fixed module, never a class or module name supplied by a browser.
    codec = importlib.import_module("dimos.web.relay_bridge.topic_media")
    return cast("MediaOutput", codec.encode_media(profile, data, settings))


class TopicTransforms:
    """A provider shares two worker permits across all active routes.

    Acquire a permit *before* selecting the sample to encode, so jobs waiting
    for a worker do not retain an obsolete camera frame; release it after each
    encode, so one route cannot hold both while an event route waits.
    Cancellation waits for an active worker before releasing its permit.
    """

    def __init__(self) -> None:
        self.slots = asyncio.Semaphore(2)

    async def run(self, fn: Callable[..., _T], *args: Any) -> _T:
        """One worker job; the caller holds a permit from ``slots``."""
        task = asyncio.create_task(asyncio.to_thread(fn, *args))
        try:
            return await asyncio.shield(task)
        except asyncio.CancelledError:
            with suppress(Exception):
                await task
            raise

    async def encode(
        self, profile: str, data: bytes, settings: Mapping[str, Any] | None = None
    ) -> "MediaOutput":
        started = time.monotonic()
        result = await self.run(_encode, profile, data, settings)
        result.metadata["encodeMs"] = (time.monotonic() - started) * 1000
        return result
