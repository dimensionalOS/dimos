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

"""Fetches a HardwareModule's files over its hardware_file_request/response into a local cache that pxr and mujoco open.

Every file is kept once at ``<cache>/<sha256 hex>/<name>``. A module's ``hardware.usda`` is kept the
same way, and the directory holding it is laid out as the module serves it: each file it reaches
is linked in at its relative path, so the names in the layers resolve as they do on the module.
"""

from collections.abc import Callable
from concurrent.futures import Future
import os
from pathlib import Path
import posixpath
import threading
import uuid

from dimos.constants import CACHE_DIR
from dimos.core.hardware_files import HARDWARE_USD, hardware_file_topics, is_relative_uri
from dimos.core.transport_factory import make_transport
from dimos.msgs.files.FileInfo import FileInfo
from dimos.robot.assets.usd_utils.assets import asset_hash, is_asset_hash, rename_assets
from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

HARDWARE_FILES_CACHE = CACHE_DIR / "hardware_files"


def fetch_hardware_files(namespace: str, cache: Path = HARDWARE_FILES_CACHE) -> Path:
    """The local ``hardware.usda`` of the HardwareModule in ``namespace``, with every file it reaches beside it."""
    request_topic, response_topic = hardware_file_topics(namespace)
    requests = make_transport(request_topic, FileInfo)
    responses = make_transport(response_topic, FileInfo)
    requests.start()
    responses.start()
    try:
        asking = _Asking(requests.publish)
        unsubscribe = responses.subscribe(asking.on_reply)
        try:
            usd = asking.ask(HARDWARE_USD, None)
            if usd.sha256 is None:
                raise RuntimeError(f"the HardwareModule in {namespace!r} cannot serve its USD here")
            stored = _store(cache, usd)
            _fetch_named(asking, cache, stored.parent, HARDWARE_USD, stored)
            return stored
        finally:
            unsubscribe()
    finally:
        requests.stop()
        responses.stop()


class _Asking:
    """One request at a time, waiting for its own reply with no deadline."""

    def __init__(self, publish: Callable[[FileInfo], None]) -> None:
        self._publish = publish
        self._lock = threading.Lock()
        self._waiting: tuple[FileInfo, Future[FileInfo]] | None = None

    def ask(self, uri: str, held: str | None) -> FileInfo:
        """The reply to a request for ``uri`` from one already holding the file with sha256 ``held``."""
        request = FileInfo(uri, held)
        reply: Future[FileInfo] = Future()
        with self._lock:
            self._waiting = (request, reply)
        self._publish(request)
        try:
            return reply.result()
        finally:
            with self._lock:
                self._waiting = None

    def on_reply(self, reply: FileInfo) -> None:
        with self._lock:
            waiting = self._waiting
        if waiting is None:
            return
        request, future = waiting
        # A header answers only a request holding its sha256 (or names a file the module cannot serve).
        if reply.uri == request.uri and (
            reply.content is not None or reply.sha256 in (request.sha256, None)
        ):
            future.set_result(reply)
        elif reply.uri == HARDWARE_USD and reply.content is None:
            # The module announces hardware.usda when it starts, so a request sent before then is sent again.
            self._publish(request)


def _hex(sha256: str | None) -> str:
    if not is_asset_hash(sha256):
        raise ValueError(f"a hardware file came with {sha256!r}, not a sha256")
    return str(sha256).partition(":")[2]


def _store(cache: Path, reply: FileInfo) -> Path:
    """Where the reply's file is kept, written there if it was not yet."""
    stored = cache / _hex(reply.sha256) / posixpath.basename(reply.uri)
    if stored.is_file():
        return stored
    if reply.content is None:
        raise ValueError(f"{reply.uri!r} came as a header, but {stored} is not kept here")
    if asset_hash(reply.content) != reply.sha256:
        raise ValueError(f"{reply.uri!r} does not hash to the {reply.sha256} it came with")
    stored.parent.mkdir(parents=True, exist_ok=True)
    partial = stored.with_name(f".{stored.name}.{uuid.uuid4().hex}")
    partial.write_bytes(reply.content)
    os.replace(partial, stored)
    return stored


def _link(tree: Path, uri: str, stored: Path) -> Path:
    """``uri`` under the module's tree, pointing at the kept file."""
    linked = tree / uri
    if linked.is_symlink() and linked.resolve() == stored.resolve():
        return linked
    linked.parent.mkdir(parents=True, exist_ok=True)
    partial = linked.with_name(f".{linked.name}.{uuid.uuid4().hex}")
    partial.symlink_to(stored)
    os.replace(partial, linked)
    return linked


def _held(tree: Path, uri: str) -> str | None:
    """The sha256 of the file this tree last had at ``uri``, so an unchanged one comes back as a header."""
    linked = tree / uri
    if not linked.is_symlink():
        return None
    held = f"sha256:{linked.resolve().parent.name}"
    return held if is_asset_hash(held) else None


def _fetch_named(asking: _Asking, cache: Path, tree: Path, uri: str, stored: Path) -> None:
    """Fetch what the layer at ``uri`` names that is not kept yet, link it into the tree, and do the same for each layer among it."""
    pxr = load_pxr()
    if pxr.Sdf.FileFormat.FindByExtension(Path(uri).suffix[1:]) is None:
        return
    directory = posixpath.dirname(uri)
    named = {}
    for name, sha256 in _named_files(stored.read_text()).items():
        reached = posixpath.normpath(posixpath.join(directory, name))
        if not is_relative_uri(reached):
            logger.warning(f"{uri!r} names {name!r}, which is not relative to the module")
            continue
        named[reached] = sha256
    kept = {
        reached: cache / _hex(sha256) / posixpath.basename(reached)
        for reached, sha256 in named.items()
        if sha256 is not None and (cache / _hex(sha256) / posixpath.basename(reached)).is_file()
    }
    for reached in named:
        if reached in kept:
            continue
        reply = asking.ask(reached, _held(tree, reached))
        if reply.sha256 is None:
            logger.warning(f"{uri!r} names {reached!r}, which its module cannot serve")
            continue
        kept[reached] = _store(cache, reply)
    for reached, kept_at in kept.items():
        _link(tree, reached, kept_at)
        _fetch_named(asking, cache, tree, reached, kept_at)


def _named_files(usd: str) -> dict[str, str | None]:
    """Each file a served layer names, with the sha256 its prim's assetInfo gives it, if any."""
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a served layer", None)
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
