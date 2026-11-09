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


from __future__ import annotations

from collections.abc import Mapping
import hashlib
import itertools
from pathlib import Path
from typing import Any

from dimos.robot.assets.usd import Prim, Pxr, Spec, layer_from, load_pxr
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


_HASH_ALGORITHM = "sha256"
_ASSET_ROOT_PREFIX = "/dimos/assets/"


def resolve_usd_assets(usd: str, root: str) -> str:
    if not root:
        return usd
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a subtree", root)
    base = Path(root)
    changed = False
    stack = [layer.pseudoRoot]
    while stack:
        spec = stack.pop()
        items = spec.referenceList.explicitItems or spec.referenceList.prependedItems
        rebased = []
        for ref in items:
            asset = ref.assetPath
            if asset and not Path(asset).is_absolute():
                rebased.append(pxr.Sdf.Reference(str(base / asset), ref.primPath))
                changed = True
            else:
                rebased.append(ref)
        if rebased and changed:
            if spec.referenceList.explicitItems:
                spec.referenceList.explicitItems = rebased
            else:
                spec.referenceList.prependedItems = rebased
        identifier = spec.assetInfo.get("identifier") if spec.assetInfo else None
        if identifier is not None and not Path(str(identifier.path)).is_absolute():
            spec.assetInfo["identifier"] = pxr.Sdf.AssetPath(str(base / str(identifier.path)))
            changed = True
        for attribute, value in asset_attributes(spec):
            if not Path(str(value.path)).is_absolute():
                attribute.default = pxr.Sdf.AssetPath(str(base / str(value.path)))
                changed = True
        stack.extend(spec.nameChildren)
    return str(layer.ExportToString()) if changed else usd


def publish_usd_assets(usd: str) -> tuple[str, dict[str, str]]:
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a scene")
    public: dict[str, str] = {}
    taken: set[str] = set()
    changed = False

    stack = [layer.pseudoRoot]
    while stack:
        spec = stack.pop(0)
        for real in _asset_paths_of(spec):
            if real not in public:
                public[real] = _public_asset_path(str(spec.path), real, taken)
                taken.add(public[real])
        if _rewrite_assets(pxr, spec, public):
            changed = True
        stack.extend(spec.nameChildren)

    rewritten = str(layer.ExportToString()) if changed else usd
    return rewritten, {name: real for real, name in public.items()}


def _asset_paths_of(spec: Spec) -> list[str]:
    items = spec.referenceList.explicitItems or spec.referenceList.prependedItems
    paths = [ref.assetPath for ref in items if ref.assetPath]
    identifier = spec.assetInfo.get("identifier") if spec.assetInfo else None
    if identifier is not None and str(identifier.path):
        paths.append(str(identifier.path))
    paths += [str(value.path) for _, value in asset_attributes(spec)]
    return paths


def asset_attributes(spec: Spec) -> list[tuple[Any, Any]]:
    found = []
    for attribute in spec.attributes:
        value = attribute.default
        if value is not None and type(value).__name__ == "AssetPath" and str(value.path):
            found.append((attribute, value))
    return found


def _public_asset_path(prim_path: str, real: str, taken: set[str]) -> str:
    stem = f"{_ASSET_ROOT_PREFIX}{prim_path.lstrip('/')}/{Path(real).name}"
    if stem not in taken:
        return stem
    base, dot, extension = stem.rpartition(".")
    for index in itertools.count(2):
        candidate = f"{base}_{index}{dot}{extension}" if dot else f"{stem}_{index}"
        if candidate not in taken:
            return candidate
    raise AssertionError("unreachable")


def _rewrite_assets(pxr: Pxr, spec: Spec, public: Mapping[str, str]) -> bool:
    changed = False
    items = spec.referenceList.explicitItems or spec.referenceList.prependedItems
    rebased = [
        pxr.Sdf.Reference(public[ref.assetPath], ref.primPath) if ref.assetPath in public else ref
        for ref in items
    ]
    if rebased and rebased != list(items):
        if spec.referenceList.explicitItems:
            spec.referenceList.explicitItems = rebased
        else:
            spec.referenceList.prependedItems = rebased
        changed = True
    identifier = spec.assetInfo.get("identifier") if spec.assetInfo else None
    if identifier is not None and str(identifier.path) in public:
        spec.assetInfo["identifier"] = pxr.Sdf.AssetPath(public[str(identifier.path)])
        changed = True
    for attribute, value in asset_attributes(spec):
        if str(value.path) in public:
            attribute.default = pxr.Sdf.AssetPath(public[str(value.path)])
            changed = True
    return changed


def asset_hash(data: bytes) -> str:
    return f"{_HASH_ALGORITHM}:{hashlib.new(_HASH_ALGORITHM, data).hexdigest()}"


def hash_of_file(path: str, base_dir: Path | None = None) -> str | None:
    resolved = Path(path)
    if not resolved.is_absolute() and base_dir is not None:
        resolved = base_dir / resolved
    try:
        return asset_hash(resolved.read_bytes())
    except OSError:
        return None


def write_asset_info(pxr: Pxr, prim: Prim, filename: str, base_dir: Path | None) -> None:
    prim.SetAssetInfoByKey("identifier", pxr.Sdf.AssetPath(filename))
    digest = hash_of_file(filename, base_dir)
    if digest is None:
        logger.warning(f"cannot hash {filename!r}: not readable from here, so it gets no hash")
        return
    prim.SetAssetInfoByKey("version", digest)


def referenced_asset_paths(usd: str, root: Path | str | None = None) -> set[str]:
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a layer", root)
    found: set[str] = set()
    stack = [layer.pseudoRoot]
    while stack:
        spec = stack.pop()
        found.update(_asset_paths_of(spec))
        stack.extend(spec.nameChildren)
    return found


def unhashed_assets(usd: str) -> dict[str, str]:
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer"))
    missing: dict[str, str] = {}
    for prim in stage.Traverse():
        info = prim.GetAssetInfo() or {}
        for asset in _referenced_assets(pxr, prim):
            if not info.get("version"):
                missing[asset] = str(prim.GetPath())
    return missing


def _referenced_assets(pxr: Pxr, prim: Prim) -> list[str]:
    assets = []
    for spec in prim.GetPrimStack():
        items = spec.referenceList.explicitItems or spec.referenceList.prependedItems
        assets += [ref.assetPath for ref in items if ref.assetPath]
        assets += [str(value.path) for _, value in asset_attributes(spec)]
    identifier = (prim.GetAssetInfo() or {}).get("identifier")
    if identifier is not None and str(identifier.path):
        assets.append(str(identifier.path))
    assert pxr
    return assets


def require_asset_hashes(usd: str, *, what: str) -> None:
    missing = unhashed_assets(usd)
    if not missing:
        return
    lines = []
    for asset, prim_path in sorted(missing.items()):
        digest = hash_of_file(asset)
        lines.append(
            f"  {asset}  (at {prim_path})\n"
            + (
                f"    assetInfo version: {digest}"
                if digest
                else "    unreadable from here, so hash it where the file lives"
            )
        )
    listed = "\n".join(lines)
    raise ValueError(
        f"{what} references {len(missing)} asset(s) with no hash. Every asset needs one: a "
        f"path alone cannot say whether the file fetched is the file meant. Set "
        f'assetInfo["version"] on each prim below (urdf_to_usd does this for you):\n{listed}'
    )
