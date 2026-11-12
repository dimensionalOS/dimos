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

"""Layer surgery: grafting one layer's subtree into another, and repointing the arcs and
relationships that move with it."""

from __future__ import annotations

from collections.abc import Iterator, Mapping
import itertools
from pathlib import Path
from typing import Any

from dimos.robot.assets.usd_utils.frames import link_prim_paths
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


def graft_usd(root_usd: str, subtrees: Mapping[str, str], root: Path | str | None = None) -> str:
    pxr = load_pxr()
    root_layer = layer_from(pxr, root_usd, "root", root)
    root_stage = pxr.Usd.Stage.Open(root_layer)
    by_link = link_prim_paths(root_stage)

    for frame_id, subtree_usd in subtrees.items():
        attach_path = by_link.get(frame_id)
        if attach_path is None:
            raise ValueError(
                f"cannot attach a subtree at {frame_id!r}: the root USD has no prim for that "
                f"frame (it has {sorted(by_link)})"
            )
        sub_layer = layer_from(pxr, subtree_usd, frame_id, root)
        sub_stage = pxr.Usd.Stage.Open(sub_layer)
        source = sub_stage.GetDefaultPrim()
        if not source:
            raise ValueError(f"the subtree USD for {frame_id!r} declares no defaultPrim")
        target = pxr.Sdf.Path(attach_path).AppendChild(source.GetName())
        if root_stage.GetPrimAtPath(target):
            raise ValueError(
                f"cannot attach a subtree at {frame_id!r}: {target} is taken. Two copies of one "
                f"description mount under the same prim name, because prefixing renames frames "
                f"and not prims -- so grafting both would keep only the last, silently. Give "
                f"the scene a frame per copy."
            )
        if not pxr.Sdf.CopySpec(sub_layer, source.GetPath(), root_layer, target):
            raise ValueError(f"could not copy the subtree USD for {frame_id!r} into the scene")
        _rebase_relationships(pxr, root_layer, target, str(source.GetPath()), str(target))
        _carry_outside_references(pxr, sub_layer, str(source.GetPath()), root_layer, str(target))
        _mount_joint(pxr, root_stage, attach_path, str(target), frame_id)

    return str(root_layer.ExportToString())


def _mount_joint(pxr: Pxr, stage: Stage, mount_path: str, subtree_path: str, frame_id: str) -> None:
    root_link = _first_link(pxr, stage, subtree_path)
    if root_link is None:
        return
    mount = stage.GetPrimAtPath(pxr.Sdf.Path(mount_path))
    name = prim_identifier(pxr, f"{frame_id}_mount")
    joint = pxr.UsdPhysics.Joint.Define(stage, f"{subtree_path}/{name}")
    joint.CreateBody0Rel().SetTargets([mount.GetPath()])
    joint.CreateBody1Rel().SetTargets([root_link.GetPath()])
    joint.CreateLocalPos0Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.GetPrim().SetDisplayName(f"{frame_name(root_link)}_mount")


def _first_link(pxr: Pxr, stage: Stage, path: str) -> Prim:
    root = stage.GetPrimAtPath(pxr.Sdf.Path(path))
    if not root:
        return None
    for prim in pxr.Usd.PrimRange(root):
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            return prim
    return None


def _specs_under(layer: Layer, root: Any) -> Iterator[Any]:
    """Every prim spec at or below root, in this layer alone."""
    stack = [layer.GetPrimAtPath(root)]
    while stack:
        spec = stack.pop()
        if spec is None:
            continue
        yield spec
        stack.extend(spec.nameChildren)


def _rebase_relationships(
    pxr: Pxr, layer: Layer, root: Any, old_prefix: str, new_prefix: str
) -> None:
    # By prim, not by text: a sibling named "arm_sub_2" also starts with "arm_sub".
    old_path, new_path = pxr.Sdf.Path(old_prefix), pxr.Sdf.Path(new_prefix)
    for spec in _specs_under(layer, root):
        for prop in spec.properties:
            if not isinstance(prop, pxr.Sdf.RelationshipSpec):
                continue
            targets = list(prop.targetPathList.explicitItems)
            rebased = [
                t.ReplacePrefix(old_path, new_path) if t.HasPrefix(old_path) else t for t in targets
            ]
            if rebased != targets:
                prop.targetPathList.explicitItems = rebased


_REFERENCE_FIELDS = ("explicitItems", "addedItems", "prependedItems", "appendedItems")


def _free_root_name(layer: Layer, name: str) -> str:
    """A root-level prim name this layer does not already use."""
    taken = itertools.chain([name], (f"{name}_{n}" for n in itertools.count(1)))
    return next(candidate for candidate in taken if not layer.GetPrimAtPath(f"/{candidate}"))


def _carry_one(pxr: Pxr, sub_layer: Layer, prim_path: Any, root_layer: Layer) -> str:
    """Copy a prim the subtree referenced from outside itself, under a free name."""
    at = pxr.Sdf.Path.absoluteRootPath.AppendChild(_free_root_name(root_layer, prim_path.name))
    if not pxr.Sdf.CopySpec(sub_layer, prim_path, root_layer, at):
        raise ValueError(f"could not carry over {prim_path}, which the subtree references")
    _rebase_relationships(pxr, root_layer, at, str(prim_path), str(at))
    return str(at)


def _carry_outside_references(
    pxr: Pxr, sub_layer: Layer, source: str, root_layer: Layer, target: str
) -> None:
    """Repoint the arcs of a subtree we just copied, carrying what they reach outside it.

    A converted asset often keeps its shader networks as root-level prims beside its
    default prim, and the prims inside reach them by absolute path. Copying the default
    prim alone leaves those arcs pointing at whatever happens to own that path in the
    scene, which is how a grafted robot ends up wearing the room's materials -- so bring
    the referenced prims along under names the scene has not taken. Arcs that point back
    inside the subtree just move with it.
    """
    inside, landed = pxr.Sdf.Path(source), pxr.Sdf.Path(target)
    carried: dict[str, str] = {}
    pending = [target]
    while pending:
        for spec in _specs_under(root_layer, pending.pop()):
            for field in _REFERENCE_FIELDS:
                arcs = list(getattr(spec.referenceList, field))
                moved = [
                    _moved_reference(
                        pxr, arc, sub_layer, root_layer, inside, landed, carried, pending
                    )
                    for arc in arcs
                ]
                if moved != arcs:
                    setattr(spec.referenceList, field, moved)


def _moved_reference(
    pxr: Pxr,
    arc: Any,
    sub_layer: Layer,
    root_layer: Layer,
    inside: Any,
    landed: Any,
    carried: dict[str, str],
    pending: list[str],
) -> Any:
    if arc.assetPath or not arc.primPath:
        # A file of its own, which brings its own namespace with it.
        return arc
    if arc.primPath.HasPrefix(inside):
        at = str(arc.primPath.ReplacePrefix(inside, landed))
    elif str(arc.primPath) in carried:
        at = carried[str(arc.primPath)]
    else:
        at = _carry_one(pxr, sub_layer, arc.primPath, root_layer)
        carried[str(arc.primPath)] = at
        pending.append(at)
    return pxr.Sdf.Reference(arc.assetPath, at, arc.layerOffset, arc.customData)
