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

"""A simulated world's files as a viewer asks for them on render_file_request: `*.usd` is the world, the rest its assets."""

import asyncio
from collections.abc import Mapping
from pathlib import Path

from dimos.robot.assets.usd_utils.assets import publish_usd_assets

# A request for either is a request for the world's layer, answered under the name it asked by.
USD_SUFFIXES = (".usd", ".usda")


class WorldFiles:
    def __init__(self, usd: str, files: Mapping[str, Path | bytes] | None = None) -> None:
        """`usd` is the composed world (assets as absolute paths); `files` are further names answered as they are."""
        self.usd = usd
        self._files = dict(files or {})
        self._published: asyncio.Future[tuple[str, dict[str, str]]] | None = None

    async def file(self, name: str) -> bytes | None:
        """What a request for `name` is answered with, or None for a name this world does not have."""
        if self._published is None:
            self._published = asyncio.ensure_future(asyncio.to_thread(publish_usd_assets, self.usd))
        usd, published = await asyncio.shield(self._published)
        found: Path | bytes | None = (
            Path(published[name]) if name in published else self._files.get(name)
        )
        if found is None:
            return usd.encode() if name.endswith(USD_SUFFIXES) else None
        return found if isinstance(found, bytes) else await asyncio.to_thread(found.read_bytes)
