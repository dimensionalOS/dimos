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

"""Materialize a USD manager's composed scene and referenced assets through RPC."""

from collections.abc import Callable
import hashlib
from pathlib import Path
import tempfile

from dimos.constants import CACHE_DIR
from dimos.core.rpc_client import RPCClient
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.assets import asset_attributes, asset_hash, resolve_usd_assets
from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr


def fetch_scene(remote_name: str, rpc: RPCSpec) -> UsdModel:
    client = RPCClient(None, None, remote_name, {"get_scene", "get_asset"}, rpc=rpc)
    try:
        answer = client.get_scene([])
        if (
            not isinstance(answer, list)
            or len(answer) != 2
            or not all(isinstance(v, str) for v in answer)
        ):
            raise ValueError(f"{remote_name}.get_scene() must return [asset_root, usd]")
        return materialize_scene(resolve_usd_assets(answer[1], answer[0]), client.get_asset)
    finally:
        client.stop_rpc_client()


def materialize_scene(
    usd: str, fetch: Callable[[str], bytes], *, cache: Path | None = None
) -> UsdModel:
    """Use content-addressed, relative filenames so every reader gets the same revision."""
    cache = cache if cache is not None else Path(CACHE_DIR) / "usd"
    cache.mkdir(parents=True, exist_ok=True)
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a served scene", None)
    files: dict[str, str] = {}

    def local(name: str, declared: str | None) -> str:
        if name in files:
            return files[name]
        data = fetch(name)
        digest = asset_hash(data)
        if declared is not None and declared != digest:
            raise ValueError(
                f"asset hash mismatch for {name}: expected {declared}, received {digest}"
            )
        filename = digest.replace(":", "-") + Path(name).suffix
        _write(cache / filename, data)
        files[name] = filename
        return filename

    stack = [layer.pseudoRoot]
    while stack:
        spec = stack.pop()
        info = dict(spec.assetInfo) if spec.assetInfo else {}
        digest = str(info["version"]) if "version" in info else None
        refs = spec.referenceList.explicitItems or spec.referenceList.prependedItems
        if refs:
            spec.referenceList.explicitItems = [
                pxr.Sdf.Reference(local(ref.assetPath, None), ref.primPath)
                if ref.assetPath
                else ref
                for ref in refs
            ]
        identifier = info.get("identifier")
        if identifier is not None and str(identifier.path):
            info["identifier"] = pxr.Sdf.AssetPath(local(str(identifier.path), digest))
            spec.assetInfo = info
        for attribute, value in asset_attributes(spec):
            attribute.default = pxr.Sdf.AssetPath(local(str(value.path), None))
        stack.extend(spec.nameChildren)
    text = str(layer.ExportToString())
    path = cache / (hashlib.sha256(text.encode()).hexdigest() + ".usda")
    _write(path, text.encode())
    return UsdModel.from_file(path)


def _write(path: Path, data: bytes) -> None:
    if path.is_file() and path.read_bytes() == data:
        return
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as temporary:
        temporary.write(data)
        pending = Path(temporary.name)
    try:
        pending.replace(path)
    finally:
        pending.unlink(missing_ok=True)
