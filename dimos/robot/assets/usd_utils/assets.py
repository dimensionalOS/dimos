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

from collections.abc import Callable, Iterator, Mapping
import hashlib
import itertools
from pathlib import Path
import re
from typing import Any

from dimos.robot.assets.usd_utils.pxr import Layer, Prim, Pxr, Spec, layer_from, load_pxr
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


_HASH_ALGORITHM = "sha256"
_ASSET_ROOT_PREFIX = "/dimos/assets/"


def is_published_asset(path: str) -> bool:
    """Whether this asset name is one publish_usd_assets minted for the bus, not a file path.

    A published name says nothing about where the file is; only the module that published it
    can still hand over the bytes.
    """
    return path.startswith(_ASSET_ROOT_PREFIX)


def resolve_usd_assets(usd: str, root: str) -> str:
    if not root:
        return usd
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a subtree", root)
    base = Path(root)

    def anchored(asset: str, _where: str, _required: bool) -> str | None:
        return None if Path(asset).is_absolute() else str(base / asset)

    return str(layer.ExportToString()) if rename_assets(pxr, layer, anchored) else usd


def publish_usd_assets(usd: str, root: Path | str | None = None) -> tuple[str, dict[str, str]]:
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a scene", root)
    public: dict[str, str] = {}
    taken: set[str] = set()
    for real, where in _required_assets(pxr, layer):
        if real not in public:
            public[real] = _public_asset_path(where, real, taken)
            taken.add(public[real])

    changed = rename_assets(pxr, layer, lambda asset, _where, _required: public.get(asset))
    rewritten = str(layer.ExportToString()) if changed else usd
    return rewritten, {name: real for real, name in public.items()}


# (asset path, prim path naming it, whether it needs the file) -> its new path, or None to keep it.
AssetRename = Callable[[str, str, bool], str | None]

_LISTED_ARC_FIELDS = ("explicitItems", "addedItems", "prependedItems", "appendedItems")
# A delete or a reorder names an arc stated elsewhere: it moves with that arc but needs no file.
_MATCHING_ARC_FIELDS = ("deletedItems", "orderedItems")


def repointed_arc(pxr: Pxr, arc: Any, asset_path: str, prim_path: Any) -> Any:
    """The same reference or payload aimed elsewhere, keeping its time offset and custom data."""
    if isinstance(arc, pxr.Sdf.Payload):
        return pxr.Sdf.Payload(asset_path, prim_path, arc.layerOffset)
    return pxr.Sdf.Reference(asset_path, prim_path, arc.layerOffset, arc.customData)


def _required_assets(pxr: Pxr, layer: Layer) -> list[tuple[str, str]]:
    found: list[tuple[str, str]] = []

    def record(asset: str, where: str, required: bool) -> None:
        if required:
            found.append((asset, where))

    rename_assets(pxr, layer, record)
    return found


def rename_assets(pxr: Pxr, layer: Layer, rename: AssetRename) -> bool:
    """Offer every asset path the layer states to rename, writing back what it returns."""
    changed = False
    offsets = list(layer.subLayerOffsets)
    for index, sublayer in enumerate(list(layer.subLayerPaths)):
        renamed = rename(sublayer, "/", True)
        if renamed is not None:
            layer.subLayerPaths[index] = renamed
            layer.subLayerOffsets[index] = offsets[index]
            changed = True
    for spec in _prim_specs(layer):
        changed = _rename_spec_assets(pxr, spec, rename) or changed
    return changed


def _prim_specs(layer: Layer) -> Iterator[Spec]:
    """Every prim spec in the layer, breadth first, variants included."""
    queue = [layer.pseudoRoot]
    while queue:
        spec = queue.pop(0)
        yield spec
        queue.extend(spec.nameChildren)
        for variant_set in spec.variantSets.values():
            queue.extend(variant.primSpec for variant in variant_set.variants.values())


def _rename_spec_assets(pxr: Pxr, spec: Spec, rename: AssetRename) -> bool:
    where = str(spec.path.StripAllVariantSelections())
    changed = False
    for has_arcs, list_name in (
        (spec.hasReferences, "referenceList"),
        (spec.hasPayloads, "payloadList"),
    ):
        if not has_arcs:
            continue
        arcs = getattr(spec, list_name)
        for field in _LISTED_ARC_FIELDS + _MATCHING_ARC_FIELDS:
            items = list(getattr(arcs, field))
            moved = []
            for arc in items:
                renamed = (
                    rename(arc.assetPath, where, field in _LISTED_ARC_FIELDS)
                    if arc.assetPath
                    else None
                )
                moved.append(
                    arc if renamed is None else repointed_arc(pxr, arc, renamed, arc.primPath)
                )
            if moved != items:
                setattr(arcs, field, moved)
                changed = True
    identifier = spec.assetInfo.get("identifier") if spec.assetInfo else None
    renamed_identifier = _renamed_value(pxr, identifier, where, rename)
    if renamed_identifier is not None:
        spec.assetInfo["identifier"] = renamed_identifier
        changed = True
    asset_types = (pxr.Sdf.ValueTypeNames.Asset, pxr.Sdf.ValueTypeNames.AssetArray)
    for attribute in spec.attributes:
        # Only an asset-typed attribute holds paths; reading any other's value copies its data (a mesh's points) for nothing.
        if attribute.typeName not in asset_types:
            continue
        value = _renamed_value(pxr, attribute.default, where, rename)
        if value is not None:
            attribute.default = value
            changed = True
        for time in spec.layer.ListTimeSamplesForPath(attribute.path):
            sample = spec.layer.QueryTimeSample(attribute.path, time)
            value = _renamed_value(pxr, sample, where, rename)
            if value is not None:
                spec.layer.SetTimeSample(attribute.path, time, value)
                changed = True
    return changed


def _renamed_value(pxr: Pxr, value: Any, where: str, rename: AssetRename) -> Any:
    """An asset-valued value with its paths renamed, or None when nothing in it changes."""
    if isinstance(value, pxr.Sdf.AssetPath):
        renamed = rename(str(value.path), where, True) if str(value.path) else None
        return None if renamed is None else pxr.Sdf.AssetPath(renamed)
    if isinstance(value, pxr.Sdf.AssetPathArray):
        paths = [str(asset.path) for asset in value]
        moved = [(rename(path, where, True) if path else None) or path for path in paths]
        return None if moved == paths else pxr.Sdf.AssetPathArray(moved)
    return None


def _public_asset_path(prim_path: str, real: str, taken: set[str]) -> str:
    where = prim_path.strip("/")
    stem = f"{_ASSET_ROOT_PREFIX}{where + '/' if where else ''}{Path(real).name}"
    if stem not in taken:
        return stem
    base, dot, extension = stem.rpartition(".")
    for index in itertools.count(2):
        candidate = f"{base}_{index}{dot}{extension}" if dot else f"{stem}_{index}"
        if candidate not in taken:
            return candidate
    raise AssertionError("unreachable")


def asset_hash(data: bytes) -> str:
    return f"{_HASH_ALGORITHM}:{hashlib.new(_HASH_ALGORITHM, data).hexdigest()}"


_HASH_PATTERN = re.compile(rf"^{_HASH_ALGORITHM}:[0-9a-f]{{64}}$")


def is_asset_hash(version: object) -> bool:
    """Whether an assetInfo version is the digest this API promises its consumers.

    USD's `version` is free-form, so a layer authored elsewhere may carry a release number
    like "1.0" there. Accepting that would let an asset through the hash check without giving
    anyone a way to tell the file they fetched from the file that was meant.
    """
    return bool(version) and _HASH_PATTERN.match(str(version)) is not None


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


_LFS_POINTER_PREFIX = b"version https://git-lfs.github.com/spec/v1"
LFS_POINTER = "an unfetched Git LFS pointer, not the file itself"


def unopenable_assets(usd: str, root: Path | str | None) -> dict[str, str]:
    unopenable: dict[str, str] = {}
    base = Path(root) if root else None
    for asset in referenced_asset_paths(usd, root):
        if is_published_asset(asset):
            continue
        path = Path(asset)
        if not path.is_absolute():
            if base is None:
                unopenable[asset] = "relative, and this layer was opened with no asset root"
                continue
            path = base / path
        try:
            head = path.open("rb").read(len(_LFS_POINTER_PREFIX))
        except OSError as error:
            unopenable[asset] = f"{type(error).__name__}: {error.strerror or error}"
            continue
        if head == _LFS_POINTER_PREFIX:
            unopenable[asset] = LFS_POINTER
    return unopenable


def require_fetched_assets(
    unopenable: Mapping[str, str], root: Path | str | None, *, what: str
) -> None:
    stubs = {asset: why for asset, why in unopenable.items() if why == LFS_POINTER}
    if not stubs:
        return
    raise FileNotFoundError(
        f"{describe_unopenable_assets(stubs, root, what=what)}\nNothing has to be downloaded "
        f"twice: a checkout under GIT_LFS_SKIP_SMUDGE=1 writes pointers, and `git lfs checkout` "
        f"turns the objects already in .git/lfs/objects back into files."
    )


def describe_unopenable_assets(
    unopenable: Mapping[str, str], root: Path | str | None, *, what: str
) -> str:
    shown = "\n".join(f"  {asset}\n    {why}" for asset, why in sorted(unopenable.items())[:5])
    more = f"\n  ... and {len(unopenable) - 5} more" if len(unopenable) > 5 else ""
    where = _common_parent(unopenable, root)
    fetch = (
        f'\nFetch them with: git lfs checkout "{where}"'
        if any(why == LFS_POINTER for why in unopenable.values()) and where
        else ""
    )
    return (
        f"{what} names {len(unopenable)} asset(s) USD cannot open, so that geometry will be "
        f"missing and every reference to it costs a USD warning on every open of this "
        f"layer:\n{shown}{more}{fetch}"
    )


def _common_parent(assets: Mapping[str, str], root: Path | str | None) -> str:
    base = Path(root) if root else None
    parents = {
        str(
            (base / asset).parent
            if base is not None and not Path(asset).is_absolute()
            else Path(asset).parent
        )
        for asset in assets
    }
    return parents.pop() if len(parents) == 1 else ""


def referenced_asset_paths(usd: str, root: Path | str | None = None) -> set[str]:
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a layer", root)
    return {asset for asset, _ in _required_assets(pxr, layer)}


def unhashed_assets(usd: str, root: Path | str | None = None) -> dict[str, str]:
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer", root))
    missing: dict[str, str] = {}
    for prim in stage.Traverse():
        info = prim.GetAssetInfo() or {}
        for asset in _referenced_assets(pxr, prim):
            if not is_asset_hash(info.get("version")):
                missing[asset] = str(prim.GetPath())
    return missing


def _referenced_assets(pxr: Pxr, prim: Prim) -> list[str]:
    assets: list[str] = []

    def record(asset: str, _where: str, required: bool) -> None:
        if required:
            assets.append(asset)

    for spec in prim.GetPrimStack():
        _rename_spec_assets(pxr, spec, record)
    return assets


def require_asset_hashes(usd: str, *, what: str, root: Path | str | None = None) -> None:
    missing = unhashed_assets(usd, root)
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
        f"{what} references {len(missing)} asset(s) with no usable hash. Every asset needs "
        f"one: a path alone cannot say whether the file fetched is the file meant. Set "
        f'assetInfo["version"] on each prim below to "{_HASH_ALGORITHM}:<64 hex digits>" '
        f"(urdf_to_usd does this for you):\n{listed}"
    )
