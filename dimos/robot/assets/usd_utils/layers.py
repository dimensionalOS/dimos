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

"""Layer surgery: nesting a layer under a namespace (repointing the paths that move with it), and
overlaying namespaced layers into one world."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import itertools
from pathlib import Path
from typing import Any

from dimos.protocol.tf.frame_paths import check_frame_path
from dimos.robot.assets.usd_utils.assets import repointed_arc
from dimos.robot.assets.usd_utils.pxr import (
    Layer,
    Prim,
    Pxr,
    Stage,
    frame_name,
    layer_from,
    load_pxr,
    prim_identifier,
)


def overlay_usds(root_usd: str, mounted: Sequence[tuple[str, str]]) -> str:
    """The root layer with each mounted (usd, mount frame) layer under it, flattened into one layer.

    Each layer is already namespaced, so its prims sit at their tf frames and they overlay as they
    are; the root's opinions are the strongest and its metadata (defaultPrim, units) the world's.
    Only the root's PhysicsScene survives, and each mounted part with a rigid body is welded to its
    mount prim by a joint (a physics backend frees a floating robot itself).
    """
    from pxr import UsdUtils

    pxr = load_pxr()
    root_layer = layer_from(pxr, root_usd, "the root", None)
    # Kept in a name: a sublayer is found by its identifier, and an anonymous layer lives only as long as its handle.
    mounted_layers = [(_without_physics_scenes(pxr, usd), frame) for usd, frame in mounted]
    root_layer.subLayerPaths = [layer.identifier for layer, _ in mounted_layers]
    flattened = UsdUtils.FlattenLayerStack(pxr.Usd.Stage.Open(root_layer))
    world = pxr.Usd.Stage.Open(flattened)
    for layer, frame in mounted_layers:
        _weld_to_mount(pxr, world, pxr.Usd.Stage.Open(layer), frame)
    return str(flattened.ExportToString())


def _without_physics_scenes(pxr: Pxr, usd: str) -> Layer:
    # A mounted part's PhysicsScene would be a second one for the world, so it goes.
    layer = layer_from(pxr, usd, "a mounted part", None)
    stage = pxr.Usd.Stage.Open(layer)
    for path in [prim.GetPath() for prim in stage.Traverse() if prim.IsA(pxr.UsdPhysics.Scene)]:
        stage.RemovePrim(path)
    return layer


def _weld_to_mount(pxr: Pxr, world: Stage, part: Stage, mount_frame: str) -> None:
    mount_path = pxr.Sdf.Path(f"/{mount_frame}")
    root_link = _first_link(pxr, part, mount_path)
    if root_link is None:
        return
    # The part's own top prim, the child of its mount on the way to its first link.
    top = next(
        path for path in root_link.GetPath().GetPrefixes() if path.GetParentPath() == mount_path
    )
    name = prim_identifier(pxr, f"{mount_frame}_mount")
    joint = pxr.UsdPhysics.Joint.Define(world, top.AppendChild(name))
    joint.CreateBody0Rel().SetTargets([mount_path])
    joint.CreateBody1Rel().SetTargets([root_link.GetPath()])
    joint.CreateLocalPos0Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.GetPrim().SetDisplayName(f"{frame_name(root_link)}_mount")


def _first_link(pxr: Pxr, stage: Stage, path: Any) -> Prim:
    root = stage.GetPrimAtPath(path)
    if not root:
        return None
    for prim in pxr.Usd.PrimRange(root):
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            return prim
    return None


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
