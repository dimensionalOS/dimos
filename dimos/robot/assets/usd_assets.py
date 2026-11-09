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

"""The files a layer names: hashing them, publishing them by hash, and resolving them back.

A layer travels as text and the files it refers to do not travel with it, so a consumer gets
asset paths it cannot open. Here they are rewritten to content-addressed names that a receiver
can ask for one at a time and resolve against its own copy.
"""

from __future__ import annotations

from collections.abc import Mapping
import hashlib
import itertools
from pathlib import Path
from typing import Any

from dimos.robot.assets.usd import Prim, Pxr, Spec, layer_from, load_pxr
from dimos.utils.logging_config import setup_logger

Vector = tuple[float, float, float]

logger = setup_logger()


# The algorithm is inside the value, so it can change without the old ones becoming unreadable.
_HASH_ALGORITHM = "sha256"
# Not paths on any disk: names the manager that assembled the scene answers for.
_ASSET_ROOT_PREFIX = "/dimos/assets/"


def resolve_usd_assets(usd: str, root: str) -> str:
    """``usd`` with every relative asset path rewritten to sit under ``root``.

    Only the module that answered for a layer knows what its mesh paths are relative to, so a
    subtree is rebased as it arrives. Absolute paths, and everything when ``root`` is empty, are
    left alone. The layer is read *under* that root too, so USD resolves while composing instead
    of warning once per mesh prim.
    """
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
        # assetInfo names the same file, so it moves with it or the two disagree.
        identifier = spec.assetInfo.get("identifier") if spec.assetInfo else None
        if identifier is not None and not Path(str(identifier.path)).is_absolute():
            # assetInfo is a proxy, not a dict: set the key on it rather than assigning a copy.
            spec.assetInfo["identifier"] = pxr.Sdf.AssetPath(str(base / str(identifier.path)))
            changed = True
        # A texture names its image in an ordinary asset attribute, neither a reference nor assetInfo.
        # Rebasing only those two left every texture relative to a directory that no longer figures.
        for attribute, value in asset_attributes(spec):
            if not Path(str(value.path)).is_absolute():
                attribute.default = pxr.Sdf.AssetPath(str(base / str(value.path)))
                changed = True
        stack.extend(spec.nameChildren)
    return str(layer.ExportToString()) if changed else usd


def publish_usd_assets(usd: str) -> tuple[str, dict[str, str]]:
    """``usd`` with every asset path replaced by a stable public one, and ``{public: real}``.

    The paths a scene is built with are absolute on the machine that assembled it, which says
    nothing to whoever asks for the file. A name comes from where the asset first appears -- its
    prim path -- so it is stable across runs; later references reuse it, because two copies of one
    arm are one mesh.
    """
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a scene")
    public: dict[str, str] = {}
    taken: set[str] = set()
    changed = False

    stack = [layer.pseudoRoot]
    while stack:
        spec = stack.pop(0)  # breadth-first, so "first" means nearest the root
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
    """Every asset path this prim spec names: a reference, assetInfo, or an asset attribute."""
    items = spec.referenceList.explicitItems or spec.referenceList.prependedItems
    paths = [ref.assetPath for ref in items if ref.assetPath]
    identifier = spec.assetInfo.get("identifier") if spec.assetInfo else None
    if identifier is not None and str(identifier.path):
        paths.append(str(identifier.path))
    paths += [str(value.path) for _, value in asset_attributes(spec)]
    return paths


def asset_attributes(spec: Spec) -> list[tuple[Any, Any]]:
    """This prim spec's asset-valued attributes, with their paths.

    A texture is neither a reference nor a mesh: a `UsdUVTexture` names its image in
    `inputs:file`, an ordinary attribute of type `asset`. Walking only arcs and assetInfo left
    every texture unpublished and unhashed, and silently so.
    """
    found = []
    for attribute in spec.attributes:
        value = attribute.default
        if value is not None and type(value).__name__ == "AssetPath" and str(value.path):
            found.append((attribute, value))
    return found


def _public_asset_path(prim_path: str, real: str, taken: set[str]) -> str:
    """A path under the fictional root, from where this asset first appears.

    The prim path is unique within a scene, so two assets collide only on one prim -- a link with
    two meshes. The suffix counts rather than hashes, to stay readable.
    """
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
    """The hash an asset is identified by, in the form assetInfo["version"] carries."""
    return f"{_HASH_ALGORITHM}:{hashlib.new(_HASH_ALGORITHM, data).hexdigest()}"


def hash_of_file(path: str, base_dir: Path | None = None) -> str | None:
    """``path``'s asset hash, or None when it cannot be read from here."""
    resolved = Path(path)
    if not resolved.is_absolute() and base_dir is not None:
        resolved = base_dir / resolved
    try:
        return asset_hash(resolved.read_bytes())
    except OSError:
        return None


def write_asset_info(pxr: Pxr, prim: Prim, filename: str, base_dir: Path | None) -> None:
    """Identify the asset this prim references, and hash it if it can be read. assetInfo is USD's
    own metadata for this, so a consumer can check what it fetched against what was meant.
    """
    prim.SetAssetInfoByKey("identifier", pxr.Sdf.AssetPath(filename))
    digest = hash_of_file(filename, base_dir)
    if digest is None:
        # The converter runs where the description lives, so this is worth saying out loud.
        logger.warning(f"cannot hash {filename!r}: not readable from here, so it gets no hash")
        return
    prim.SetAssetInfoByKey("version", digest)


def referenced_asset_paths(usd: str, root: Path | str | None = None) -> set[str]:
    """Every asset path ``usd`` names, as authored. What a renderer may ask to be served."""
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
    """Every asset ``usd`` references without a hash, mapped to the prim path carrying it."""
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
    """The asset paths this prim names, as authored.

    Three ways a prim can name one: a reference arc, assetInfo (a mesh's file) and an asset-valued
    attribute (a texture's image). Only arcs made the hash check pass vacuously on every mesh;
    only those two passed over every texture.
    """
    assets = []
    for spec in prim.GetPrimStack():
        items = spec.referenceList.explicitItems or spec.referenceList.prependedItems
        assets += [ref.assetPath for ref in items if ref.assetPath]
        assets += [str(value.path) for _, value in asset_attributes(spec)]
    identifier = (prim.GetAssetInfo() or {}).get("identifier")
    if identifier is not None and str(identifier.path):
        assets.append(str(identifier.path))
    assert pxr  # the pxr bundle is what opened the stage these specs came from
    return assets


def require_asset_hashes(usd: str, *, what: str) -> None:
    """Refuse a layer whose assets are not identified by content. The error carries the hash each
    one should have had, computed wherever the file can still be read.
    """
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
