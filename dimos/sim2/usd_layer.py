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
from typing import Any

from dimos.robot.assets.model import UsdModel
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_CACHE = Path(tempfile.gettempdir()) / "dimos-sim2-usd"
_ASSETS = Path(tempfile.gettempdir()) / "dimos-usd-assets"


def usd_layer(model: UsdModel, transform: Callable[[str], str] | None = None) -> Path:
    """Write the model out as a layer file, optionally rewritten for whoever is about to read it."""
    from dimos.robot.assets.usd_utils.assets import resolve_usd_assets

    source = model.source_path
    layer = resolve_usd_assets(model.usd, str(source.parent) if source is not None else "")
    if transform is not None:
        layer = transform(layer)
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
    from dimos.robot.assets.usd_utils.assets import rename_assets
    from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr

    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a scene", None)
    _ASSETS.mkdir(parents=True, exist_ok=True)
    here: dict[str, str] = {}
    rewritten = 0

    def on_disk(name: str, where: str, _required: bool) -> str | None:
        nonlocal rewritten
        if not name:
            return None
        if name not in here:
            here[name] = str(_cached(name, _declared_hash(layer, where, name), fetch))
        rewritten += 1
        return here[name]

    rename_assets(pxr, layer, on_disk)
    logger.info(
        f"sim2: {rewritten} asset reference(s) over {len(here)} file(s) of this scene are on "
        f"disk under {_ASSETS}"
    )
    return layer_file(layer.ExportToString(), called="scene")


def _declared_hash(layer: Any, where: str, name: str) -> str | None:
    """The hash the prim's assetInfo promises for `name`, when it names that file."""
    spec = layer.GetPrimAtPath(where)
    info = dict(spec.assetInfo) if spec is not None and spec.assetInfo else {}
    identifier = info.get("identifier")
    if identifier is None or str(identifier.path) != name or "version" not in info:
        return None
    return str(info["version"])


def _cached(name: str, digest: str | None, fetch: Callable[[str], bytes]) -> Path:
    suffix = Path(name).suffix
    if digest:
        promised = _ASSETS / f"{digest}{suffix}"
        if promised.exists():
            return promised
    data = fetch(name)
    from dimos.robot.assets.usd_utils.assets import asset_hash

    out = _ASSETS / f"{asset_hash(data)}{suffix}"
    if not out.exists():
        partial = out.with_suffix(f"{suffix}.{os.getpid()}.partial")
        partial.write_bytes(data)
        partial.replace(out)
    return out
