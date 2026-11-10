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
from contextlib import suppress
import importlib
import time
from typing import TYPE_CHECKING, cast

if TYPE_CHECKING:
    from dimos.web.relay_bridge.topic_media import MediaOutput


def _encode(profile: str, data: bytes) -> "MediaOutput":
    # Fixed module, never a class or module name supplied by a browser.
    codec = importlib.import_module("dimos.web.relay_bridge.topic_media")
    return cast("MediaOutput", codec.encode_media(profile, data))


class TopicTransforms:
    """A provider shares two worker permits across all active routes.

    Acquire a permit *before* taking from the raw latest mailbox, so jobs
    waiting for a worker do not retain an obsolete selected camera frame.
    Cancellation waits for an active worker before releasing its permit.
    """

    def __init__(self) -> None:
        self.slots = asyncio.Semaphore(2)

    async def encode(self, profile: str, data: bytes) -> "MediaOutput":
        started = time.monotonic()
        task = asyncio.create_task(asyncio.to_thread(_encode, profile, data))
        try:
            result = await asyncio.shield(task)
        except asyncio.CancelledError:
            with suppress(Exception):
                await task
            raise
        result.metadata["encodeMs"] = (time.monotonic() - started) * 1000
        return result
