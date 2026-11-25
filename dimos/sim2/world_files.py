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
from collections.abc import Callable, Mapping
from pathlib import Path
import posixpath

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.msgs.files.File import File
from dimos.robot.assets.usd_utils.assets import publish_usd_assets
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# A request for either is a request for the world's layer, answered under the name it asked by.
USD_SUFFIXES = (".usd", ".usda")


def published_world(usd: str) -> tuple[str, dict[str, str]]:
    """publish_usd_assets, plus every file the layers it publishes reach, each named relative to the name of the layer naming it."""
    usd, published = publish_usd_assets(usd)
    for name, real in list(published.items()):
        _publish_reached(name, Path(real), Path(real).resolve().parent, published, set())
    return usd, published


def _publish_reached(
    name: str, real: Path, scene_dir: Path, published: dict[str, str], walked: set[Path]
) -> None:
    """Add what the layer at `real` (asked for as `name`) names, at any depth, staying inside `scene_dir` or the repo."""
    from pxr import Ar, Sdf, Tf, UsdUtils

    if real.resolve() in walked or Sdf.FileFormat.FindByExtension(real.suffix[1:]) is None:
        return
    walked.add(real.resolve())
    try:
        layer = Sdf.Layer.FindOrOpen(str(real))
        sublayers, references, payloads = UsdUtils.ExtractExternalReferences(str(real))
    except Tf.ErrorException as error:
        logger.warning(f"{real} does not open as a layer, so nothing it names is served: {error}")
        return
    if layer is None:
        return
    for authored in [*sublayers, *references, *payloads]:
        resolved = str(Ar.GetResolver().Resolve(layer.ComputeAbsolutePath(authored)))
        if not resolved or not Path(resolved).is_file():
            logger.debug(f"{real} names {authored!r}, which is not a file here")
            continue
        found = Path(resolved).resolve()
        if not any(
            found.is_relative_to(root) for root in (scene_dir, DIMOS_PROJECT_ROOT.resolve())
        ):
            logger.warning(
                f"{real} names {authored!r}, outside {scene_dir} and the repo: not served"
            )
            continue
        asked_as = posixpath.normpath(posixpath.join(posixpath.dirname(name), authored))
        if published.setdefault(asked_as, str(found)) != str(found):
            logger.warning(
                f"{real} names {authored!r} as {asked_as!r}, already another file's name: not served"
            )
            continue
        _publish_reached(asked_as, found, scene_dir, published, walked)


class WorldFiles:
    def __init__(self, usd: str, files: Mapping[str, Path | bytes] | None = None) -> None:
        """`usd` is the composed world (assets as absolute paths); `files` are further names answered as they are."""
        self.usd = usd
        self._files = dict(files or {})
        self._published: asyncio.Future[tuple[str, dict[str, str]]] | None = None
        self._answering: set[str] = set()

    async def answer(self, name: str, publish: Callable[[File], None]) -> None:
        """Publish the reply to a request for `name`; a repeat asked while one is on its way is that same reply."""
        if name in self._answering:
            return
        self._answering.add(name)
        try:
            data = await self.file(name)
        finally:
            self._answering.discard(name)
        if data is None:
            logger.debug(f"render_file_request for {name!r}, which the world does not name")
            return
        publish(File(name, data))

    async def file(self, name: str) -> bytes | None:
        """What a request for `name` is answered with, or None for a name this world does not have."""
        if self._published is None:
            self._published = asyncio.ensure_future(asyncio.to_thread(published_world, self.usd))
        usd, published = await asyncio.shield(self._published)
        found: Path | bytes | None = (
            Path(published[name]) if name in published else self._files.get(name)
        )
        if found is None:
            return usd.encode() if name.endswith(USD_SUFFIXES) else None
        return found if isinstance(found, bytes) else await asyncio.to_thread(found.read_bytes)
