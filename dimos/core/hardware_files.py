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

"""What a HardwareModule provides: its USD as ``hardware.usda`` and every file it reaches, named relative to it."""

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
import posixpath

from dimos.msgs.files.FileInfo import FileInfo
from dimos.robot.assets.usd_utils.assets import (
    asset_hash,
    is_asset_hash,
    layer_asset_paths,
    publish_usd_assets,
    rename_assets,
    resolve_usd_assets,
)
from dimos.robot.assets.usd_utils.lfs import pull_usd_assets
from dimos.robot.assets.usd_utils.pxr import Pxr, layer_from, load_pxr
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

HARDWARE_USD = "hardware.usda"


@dataclass(frozen=True)
class _Layer:
    """A layer as provided: rewritten so the names in it are relative to where it is provided."""

    content: bytes
    sha256: str
    mtime: float | None


@dataclass(frozen=True)
class _File:
    """A file as it is on disk, with the sha256 the layer naming it declares, so it is never hashed here."""

    path: Path
    sha256: str


def _layer(text: str, mtime: float | None) -> _Layer:
    content = text.encode()
    return _Layer(content, asset_hash(content), mtime)


def hardware_file_topics(namespace: str) -> tuple[str, str]:
    """The request and response topics of the HardwareModule in ``namespace``, as a blueprint names them."""
    prefix = f"/{namespace}/" if namespace else "/"
    return f"{prefix}hardware_file_request", f"{prefix}hardware_file_response"


def is_relative_uri(uri: str) -> bool:
    """Whether a uri stays inside the module: not absolute, and never stepping up with ``..``."""
    return bool(uri) and not uri.startswith("/") and ".." not in uri.split("/")


def declared_hashes(usd: str) -> dict[str, str | None]:
    """Each file a layer names, with the sha256 the assetInfo of the prim naming it declares, if it declares one."""
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a provided layer", None)
    named: dict[str, str | None] = {}

    def record(asset: str, where: str, required: bool) -> None:
        if not required:
            return
        spec = layer.GetPrimAtPath(where) if where != "/" else None
        info = (spec.assetInfo or {}) if spec else {}
        identifier = info.get("identifier")
        version = info.get("version")
        if identifier is not None and str(identifier.path) == asset and is_asset_hash(version):
            named[asset] = str(version)
        else:
            named.setdefault(asset, None)

    rename_assets(pxr, layer, record)
    return named


def inline_meshes(pxr: Pxr, usd: str) -> list[str]:
    """The Mesh prims whose points are written in ``usd`` itself rather than in a file it references."""
    layer = pxr.Sdf.Layer.CreateAnonymous(".usda")
    layer.ImportFromString(usd)
    found: list[str] = []

    def visit(path: object) -> None:
        spec = layer.GetPrimAtPath(path)
        if spec is not None and spec.typeName == "Mesh" and "points" in spec.attributes:
            found.append(str(path))

    layer.Traverse(layer.pseudoRoot.path, visit)
    return found


class HardwareFiles:
    def __init__(self, usd: str, mtime: float | None = None) -> None:
        """``usd`` names its files by absolute path; what is provided names them relative to it.

        Every file a layer names, other than a layer, needs its sha256 declared in the assetInfo of
        the prim naming it (urdf_to_usd writes it), so nothing is hashed here but the rewritten layers.
        """
        # Before any layer is opened: a clone without LFS smudging has pointers, not meshes.
        pull_usd_assets([usd])
        # Reserved, so a sublayer that is also called hardware.usda never takes the root's name.
        provided, published = publish_usd_assets(usd, reserved={HARDWARE_USD})
        inline = inline_meshes(load_pxr(), provided)
        if inline:
            logger.warning(
                f"{HARDWARE_USD} holds mesh geometry itself ({', '.join(inline)}); keep it to "
                f"links, joints and sensors and reference meshes from their own files"
            )
        self._usd = _layer(provided, mtime)
        # None: named by a provided layer but not providable, which is said in a header with no sha256.
        self._files: dict[str, _File | _Layer | None] = {}
        self._unhashed: dict[str, Path] = {}
        self._provide_reached("", provided, published, frozenset())
        if self._unhashed:
            listed = "\n".join(f"  {uri}  ({path})" for uri, path in sorted(self._unhashed.items()))
            raise ValueError(
                f"{len(self._unhashed)} file(s) this hardware provides have no sha256 declared, "
                f'so a renderer cannot tell whether its copy is current. Set assetInfo["version"] '
                f'to "sha256:<64 hex digits>" (and assetInfo["identifier"] to the file) on the '
                f"prim naming each one, one file per prim:\n{listed}"
            )

    @property
    def usd(self) -> str:
        """``hardware.usda`` as provided."""
        return self._usd.content.decode()

    @property
    def uris(self) -> list[str]:
        return [HARDWARE_USD, *sorted(self._files)]

    def announcement(self) -> FileInfo:
        """The header of ``hardware.usda``, published when the module starts."""
        return FileInfo(HARDWARE_USD, self._usd.sha256, self._usd.mtime)

    def reply(self, request: FileInfo) -> FileInfo | None:
        """The whole file, or only its header when the request already has its sha256; None for a uri this does not provide."""
        uri = request.uri
        if not is_relative_uri(uri):
            logger.debug(f"hardware_file_request for {uri!r}, which is not relative to the module")
            return None
        if uri != HARDWARE_USD and uri not in self._files:
            logger.debug(f"hardware_file_request for {uri!r}, which this hardware does not provide")
            return None
        found = self._usd if uri == HARDWARE_USD else self._files[uri]
        if found is None:
            return FileInfo(uri)
        if isinstance(found, _Layer):
            if request.sha256 == found.sha256:
                return FileInfo(uri, found.sha256, found.mtime)
            return FileInfo(uri, found.sha256, found.mtime, found.content)
        try:
            mtime = found.path.stat().st_mtime
            if request.sha256 == found.sha256:
                return FileInfo(uri, found.sha256, mtime)
            content = found.path.read_bytes()
        except OSError as error:
            logger.warning(
                f"hardware_file_request for {uri!r}: {found.path} cannot be read: {error}"
            )
            return FileInfo(uri)
        # The declared sha256, not one computed here: the renderer checks the content against it.
        return FileInfo(uri, found.sha256, mtime, content)

    def _provide_reached(
        self, directory: str, layer: str, published: Mapping[str, str], walking: frozenset[Path]
    ) -> None:
        """Provide each file ``layer`` (provided under ``directory``) names, and what each layer among them names, rewritten the same way."""
        pxr = load_pxr()
        declared = declared_hashes(layer)
        for name, real in published.items():
            uri = posixpath.join(directory, name)
            path = Path(real)
            if uri in self._files:
                logger.warning(f"{path} would be provided as {uri!r}, already another file's name")
                continue
            self._files[uri] = self._provided(pxr, uri, path, declared.get(name), walking)

    def _provided(
        self, pxr: Pxr, uri: str, path: Path, sha256: str | None, walking: frozenset[Path]
    ) -> _File | _Layer | None:
        if pxr.Sdf.FileFormat.FindByExtension(path.suffix[1:]) is None or not path.is_file():
            if sha256 is None:
                self._unhashed[uri] = path
                return None
            return _File(path, sha256)
        if path.resolve() in walking:
            logger.warning(
                f"{path} reaches itself through what it names, so {uri!r} is not provided"
            )
            return None
        try:
            layer = pxr.Sdf.Layer.FindOrOpen(str(path))
        except pxr.Tf.ErrorException as error:
            logger.warning(f"{path} does not open as a layer, so {uri!r} is not provided: {error}")
            return None
        if not layer_asset_paths(pxr, layer):
            # A layer is read whole when it is opened anyway, so it needs no declared sha256.
            content = path.read_bytes()
            return _Layer(content, asset_hash(content), path.stat().st_mtime)
        if path.suffix == ".usdc":
            logger.warning(
                f"{path} is a binary layer naming other files, so its names cannot be made "
                f"relative and {uri!r} is not provided (export it as .usda)"
            )
            return None
        text = resolve_usd_assets(str(layer.ExportToString()), str(path.parent))
        pull_usd_assets([text])
        directory = posixpath.dirname(uri)
        provided, reached = publish_usd_assets(
            text, reserved={HARDWARE_USD} if not directory else ()
        )
        self._provide_reached(directory, provided, reached, walking | {path.resolve()})
        return _layer(provided, path.stat().st_mtime)
