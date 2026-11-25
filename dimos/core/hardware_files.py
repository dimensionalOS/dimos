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

"""What a HardwareModule serves: its USD as ``hardware.usda`` and every file it reaches, named relative to it."""

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
import posixpath
import threading

from dimos.msgs.files.FileInfo import FileInfo
from dimos.robot.assets.usd_utils.assets import (
    asset_hash,
    layer_asset_paths,
    publish_usd_assets,
    resolve_usd_assets,
)
from dimos.robot.assets.usd_utils.lfs import pull_usd_assets
from dimos.robot.assets.usd_utils.pxr import Pxr, load_pxr
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

HARDWARE_USD = "hardware.usda"

# A file's sha256 by (path, size, mtime in ns), so a file is hashed again only once it changes.
_hashes: dict[tuple[str, int, int], str] = {}
_hashes_lock = threading.Lock()


@dataclass(frozen=True)
class _Layer:
    """A layer served rewritten, so the names in it are relative to where it is served."""

    content: bytes
    sha256: str
    mtime: float | None


def hardware_file_topics(namespace: str) -> tuple[str, str]:
    """The request and response topics of the HardwareModule in ``namespace``, as a blueprint names them."""
    prefix = f"/{namespace}/" if namespace else "/"
    return f"{prefix}hardware_file_request", f"{prefix}hardware_file_response"


def is_relative_uri(uri: str) -> bool:
    """Whether a uri stays inside the module: not absolute, and never stepping up with ``..``."""
    return bool(uri) and not uri.startswith("/") and ".." not in uri.split("/")


def file_hash(path: Path) -> tuple[str, float]:
    """The file's sha256 and mtime, hashed only when its size or mtime moved since the last time."""
    stat = path.stat()
    key = (str(path), stat.st_size, stat.st_mtime_ns)
    with _hashes_lock:
        known = _hashes.get(key)
    if known is None:
        known = asset_hash(path.read_bytes())
        with _hashes_lock:
            _hashes[key] = known
    return known, stat.st_mtime


class HardwareFiles:
    def __init__(self, usd: str, mtime: float | None = None) -> None:
        """``usd`` names its files by absolute path; what is served names them relative to it."""
        # Before any layer is opened or hashed: a clone without LFS smudging has pointers, not meshes.
        pull_usd_assets([usd])
        self.usd, published = publish_usd_assets(usd)
        self._usd = _Layer(self.usd.encode(), asset_hash(self.usd.encode()), mtime)
        # None: named by a served layer but not servable, which is said in a header with no sha256.
        self._files: dict[str, Path | _Layer | None] = {}
        self._serve_reached("", published, frozenset())

    @property
    def uris(self) -> list[str]:
        return [HARDWARE_USD, *sorted(self._files)]

    def announcement(self) -> FileInfo:
        """The header of ``hardware.usda``, published when the module starts."""
        return FileInfo(HARDWARE_USD, self._usd.sha256, self._usd.mtime)

    def reply(self, request: FileInfo) -> FileInfo | None:
        """The whole file, or only its header when the request already has its sha256; None for a uri this does not serve."""
        uri = request.uri
        if not is_relative_uri(uri):
            logger.debug(f"hardware_file_request for {uri!r}, which is not relative to the module")
            return None
        if uri != HARDWARE_USD and uri not in self._files:
            logger.debug(f"hardware_file_request for {uri!r}, which this hardware does not serve")
            return None
        found = self._usd if uri == HARDWARE_USD else self._files[uri]
        if found is None:
            return FileInfo(uri)
        if isinstance(found, _Layer):
            if request.sha256 == found.sha256:
                return FileInfo(uri, found.sha256, found.mtime)
            return FileInfo(uri, found.sha256, found.mtime, found.content)
        try:
            sha256, mtime = file_hash(found)
            if request.sha256 == sha256:
                return FileInfo(uri, sha256, mtime)
            content = found.read_bytes()
        except OSError as error:
            logger.warning(f"hardware_file_request for {uri!r}: {found} cannot be read: {error}")
            return FileInfo(uri)
        # Hashed as read, so the content and its sha256 never disagree.
        return FileInfo(uri, asset_hash(content), mtime, content)

    def _serve_reached(
        self, directory: str, published: Mapping[str, str], walking: frozenset[Path]
    ) -> None:
        """Serve each published file under ``directory``, and what each layer among them names, rewritten the same way."""
        pxr = load_pxr()
        for name, real in published.items():
            uri = posixpath.join(directory, name)
            path = Path(real)
            if uri in self._files:
                logger.warning(f"{path} would be served as {uri!r}, already another file's name")
                continue
            self._files[uri] = self._served(pxr, uri, path, walking)

    def _served(
        self, pxr: Pxr, uri: str, path: Path, walking: frozenset[Path]
    ) -> Path | _Layer | None:
        if pxr.Sdf.FileFormat.FindByExtension(path.suffix[1:]) is None or not path.is_file():
            return path
        if path.resolve() in walking:
            logger.warning(f"{path} reaches itself through what it names, so {uri!r} is not served")
            return None
        try:
            layer = pxr.Sdf.Layer.FindOrOpen(str(path))
        except pxr.Tf.ErrorException as error:
            logger.warning(f"{path} does not open as a layer, so {uri!r} is not served: {error}")
            return None
        if not layer_asset_paths(pxr, layer):
            return path
        if path.suffix == ".usdc":
            logger.warning(
                f"{path} is a binary layer naming other files, so its names cannot be made "
                f"relative and {uri!r} is not served (export it as .usda)"
            )
            return None
        text = resolve_usd_assets(str(layer.ExportToString()), str(path.parent))
        pull_usd_assets([text])
        served, reached = publish_usd_assets(text)
        content = served.encode()
        self._serve_reached(posixpath.dirname(uri), reached, walking | {path.resolve()})
        return _Layer(content, asset_hash(content), path.stat().st_mtime)
