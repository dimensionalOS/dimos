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


from collections.abc import Callable
import hashlib
import os
from pathlib import Path
import tempfile

from dimos.robot.assets.model import UsdModel
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_CACHE = Path(tempfile.gettempdir()) / "dimos-sim2-usd"
_ASSETS = Path(tempfile.gettempdir()) / "dimos-usd-assets"


def usd_layer(model: UsdModel) -> Path:
    from dimos.robot.assets.usd_assets import resolve_usd_assets

    source = model.source_path
    layer = resolve_usd_assets(model.usd, str(source.parent) if source is not None else "")
    return layer_file(layer, called=_name_of(model))


def layer_file(layer: str, called: str = "layer") -> Path:
    out = _CACHE / f"{called}-{hashlib.sha256(layer.encode()).hexdigest()[:16]}.usda"
    if not out.exists():
        out.parent.mkdir(parents=True, exist_ok=True)
        partial = out.with_suffix(f".{os.getpid()}.partial")
        partial.write_text(layer)
        partial.replace(out)
    return out


def _name_of(model: UsdModel) -> str:
    source = model.source_path
    return source.stem if source is not None else "layer"


def scene_as_files(usd: str, fetch: Callable[[str], bytes]) -> Path:
    from dimos.robot.assets.usd import layer_from, load_pxr
    from dimos.robot.assets.usd_assets import asset_attributes

    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a scene", None)
    _ASSETS.mkdir(parents=True, exist_ok=True)
    here: dict[str, str] = {}
    stack, rewritten = [layer.pseudoRoot], 0

    def on_disk(name: str, declared: str | None) -> str:
        if name not in here:
            here[name] = str(_cached(name, declared, fetch))
        return here[name]

    while stack:
        spec = stack.pop()
        info = dict(spec.assetInfo) if spec.assetInfo else {}
        declared = str(info["version"]) if "version" in info else None
        items = spec.referenceList.explicitItems or spec.referenceList.prependedItems
        if items:
            spec.referenceList.explicitItems = [
                pxr.Sdf.Reference(on_disk(ref.assetPath, declared), ref.primPath)
                if ref.assetPath
                else ref
                for ref in items
            ]
            rewritten += len(items)
        identifier = info.get("identifier")
        if identifier is not None and str(identifier.path):
            info["identifier"] = pxr.Sdf.AssetPath(on_disk(str(identifier.path), declared))
            spec.assetInfo = info
        for attribute, value in asset_attributes(spec):
            attribute.default = pxr.Sdf.AssetPath(on_disk(str(value.path), None))
            rewritten += 1
        stack.extend(spec.nameChildren)

    logger.info(
        f"sim2: {rewritten} asset reference(s) over {len(here)} file(s) of this scene are on "
        f"disk under {_ASSETS}"
    )
    return layer_file(layer.ExportToString(), called="scene")


def _cached(name: str, digest: str | None, fetch: Callable[[str], bytes]) -> Path:
    suffix = Path(name).suffix
    if digest:
        promised = _ASSETS / f"{digest}{suffix}"
        if promised.exists():
            return promised
    data = fetch(name)
    from dimos.robot.assets.usd_assets import asset_hash

    out = _ASSETS / f"{asset_hash(data)}{suffix}"
    if not out.exists():
        partial = out.with_suffix(f"{suffix}.{os.getpid()}.partial")
        partial.write_bytes(data)
        partial.replace(out)
    return out
