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

"""Layer surgery: nesting a layer under a namespace, repointing the paths that move with it."""

from __future__ import annotations

from collections.abc import Mapping
import itertools
from pathlib import Path
from typing import Any

from dimos.protocol.tf.frame_paths import check_frame_path
from dimos.robot.assets.usd_utils.assets import repointed_arc
from dimos.robot.assets.usd_utils.pxr import Layer, Pxr, layer_from, load_pxr


def _free_root_name(layer: Layer, name: str) -> str:
    """A root-level prim name this layer does not already use."""
    taken = itertools.chain([name], (f"{name}_{n}" for n in itertools.count(1)))
    return next(candidate for candidate in taken if not layer.GetPrimAtPath(f"/{candidate}"))


_LIST_OP_FIELDS = ("addedItems", "prependedItems", "appendedItems", "deletedItems", "orderedItems")


def usd_under_namespace(usd: str, namespace: str, root: Path | str | None = None) -> str:
    """The layer with every root prim moved under ``/<namespace>``, so each frame's prim path is its tf frame.

    The namespace's own prims are typeless ``def``s (an ``over`` would hide what is under it from a
    traversal), the first of them is the defaultPrim, and every path the layer states into a moved
    prim (targets, connections, internal references, inherits) follows it.
    """
    if not namespace:
        return usd
    check_frame_path(namespace, "the namespace")
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a layer", root)
    parent = pxr.Sdf.Path(f"/{namespace}")
    # Parked first, so a root prim named like the namespace's first segment can move under it.
    parked = pxr.Sdf.Path(f"/{_free_root_name(layer, '_dimos_namespace')}")
    moves = {spec.path: parent.AppendChild(spec.name) for spec in layer.rootPrims}
    pxr.Sdf.CreatePrimInLayer(layer, parked)
    _apply_moves(pxr, layer, {old: parked.AppendChild(old.name) for old in moves})
    for prefix in parent.GetPrefixes():
        pxr.Sdf.CreatePrimInLayer(layer, prefix).specifier = pxr.Sdf.SpecifierDef
    _apply_moves(pxr, layer, {parked.AppendChild(old.name): new for old, new in moves.items()})
    edit = pxr.Sdf.BatchNamespaceEdit()
    edit.Add(parked, pxr.Sdf.Path.emptyPath)
    layer.Apply(edit)
    _repath(pxr, layer, moves)
    layer.defaultPrim = parent.GetPrefixes()[0].name
    return str(layer.ExportToString())


def usd_moved_under(usd: str, namespace: str, under: str) -> str:
    """A layer namespaced under ``/<namespace>`` (``usd_under_namespace``), with what it holds there moved to under ``/<under>`` instead, every path it states following."""
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a layer", None)
    holder = layer.GetPrimAtPath(f"/{namespace}") if namespace else layer.pseudoRoot
    if holder is None:
        raise ValueError(f"the layer holds nothing under /{namespace}")
    parent = pxr.Sdf.Path(f"/{under}")
    # Parked first, so what moves can land under a prim named like a namespace prim it leaves.
    parked = pxr.Sdf.Path(f"/{_free_root_name(layer, '_dimos_moving')}")
    moves = {spec.path: parent.AppendChild(spec.name) for spec in holder.nameChildren}
    pxr.Sdf.CreatePrimInLayer(layer, parked)
    _apply_moves(pxr, layer, {old: parked.AppendChild(old.name) for old in moves})
    edit = pxr.Sdf.BatchNamespaceEdit()
    for spec in list(layer.rootPrims):
        if spec.path != parked:
            edit.Add(spec.path, pxr.Sdf.Path.emptyPath)
    layer.Apply(edit)
    for prefix in parent.GetPrefixes():
        pxr.Sdf.CreatePrimInLayer(layer, prefix).specifier = pxr.Sdf.SpecifierDef
    _apply_moves(pxr, layer, {parked.AppendChild(old.name): new for old, new in moves.items()})
    edit = pxr.Sdf.BatchNamespaceEdit()
    edit.Add(parked, pxr.Sdf.Path.emptyPath)
    layer.Apply(edit)
    _repath(pxr, layer, moves)
    layer.defaultPrim = parent.GetPrefixes()[0].name
    return str(layer.ExportToString())


def _apply_moves(pxr: Pxr, layer: Layer, moves: Mapping[Any, Any]) -> None:
    edit = pxr.Sdf.BatchNamespaceEdit()
    for old, new in moves.items():
        edit.Add(old, new)
    if not layer.Apply(edit):
        raise ValueError(f"could not move {sorted(map(str, moves))} in the layer")


def _repath(pxr: Pxr, layer: Layer, moves: Mapping[Any, Any]) -> None:
    """Point every path the layer states into a moved prim at where it went (Sdf moves do not)."""

    def moved(path: Any) -> Any:
        for old, new in moves.items():
            if path.HasPrefix(old):
                return path.ReplacePrefix(old, new)
        return path

    def arc(item: Any) -> Any:
        # Only an internal arc (no asset) names a prim of this layer.
        if item.assetPath or item.primPath.isEmpty:
            return item
        return repointed_arc(pxr, item, item.assetPath, moved(item.primPath))

    def fix(list_op: Any, change: Any) -> None:
        fields = ("explicitItems",) if list_op.isExplicit else _LIST_OP_FIELDS
        for field in fields:
            items = list(getattr(list_op, field))
            changed = [change(item) for item in items]
            if changed != items:
                setattr(list_op, field, changed)

    def visit(path: Any) -> None:
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, pxr.Sdf.RelationshipSpec):
            fix(spec.targetPathList, moved)
        elif isinstance(spec, pxr.Sdf.AttributeSpec):
            fix(spec.connectionPathList, moved)
        elif isinstance(spec, pxr.Sdf.PrimSpec):
            fix(spec.referenceList, arc)
            fix(spec.payloadList, arc)
            fix(spec.inheritPathList, moved)
            fix(spec.specializesList, moved)

    layer.Traverse(pxr.Sdf.Path.absoluteRootPath, visit)
