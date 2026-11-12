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

"""Frames: the display names dimos treats as a robot's links and joints, and the prims that
carry them. A frame is a prim with RigidBodyAPI or a physics joint, named by its display name."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.robot.assets.usd_utils.pxr import (
    Prim,
    Pxr,
    Stage,
    Vector,
    layer_from,
    load_pxr,
    prim_identifier,
    usd_matrix,
)


def carries_a_frame(pxr: Pxr, prim: Prim) -> bool:
    """Whether this prim is one of the frames dimos names: a rigid body or a physics joint."""
    return bool(prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI) or prim.IsA(pxr.UsdPhysics.Joint))


def usd_with_frames(
    usd: str, frames: Mapping[str, tuple[str, Vector, Vector]], root: Path | str | None = None
) -> str:
    if not frames:
        return usd
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a scene", root)
    stage = pxr.Usd.Stage.Open(layer)
    for name, (parent, xyz, rpy) in frames.items():
        by_link = link_prim_paths(stage)
        parent_path = by_link.get(parent)
        if parent_path is None:
            raise ValueError(
                f"cannot add the frame {name!r}: the layer has no frame {parent!r} to hang it "
                f"off (it has {sorted(by_link)[:8]}...)"
            )
        path = pxr.Sdf.Path(parent_path).AppendChild(prim_identifier(pxr, name))
        if stage.GetPrimAtPath(path):
            raise ValueError(f"cannot add the frame {name!r}: {path} is taken")
        prim = pxr.UsdGeom.Xform.Define(stage, path)
        pxr.UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
        prim.GetPrim().SetDisplayName(name)
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        transform[:3, 3] = xyz
        prim.AddTransformOp().Set(usd_matrix(pxr, transform))
        _weld_frame(pxr, stage, parent_path, str(path), name)
    return str(layer.ExportToString())


def _weld_frame(pxr: Pxr, stage: Stage, parent_path: str, path: str, name: str) -> None:
    parent = stage.GetPrimAtPath(pxr.Sdf.Path(parent_path))
    joint = pxr.UsdPhysics.FixedJoint.Define(
        stage, pxr.Sdf.Path(path).AppendChild(prim_identifier(pxr, f"{name}_joint"))
    )
    if parent and parent.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
        joint.CreateBody0Rel().SetTargets([parent.GetPath()])
    joint.CreateBody1Rel().SetTargets([pxr.Sdf.Path(path)])
    joint.CreateLocalPos0Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.GetPrim().SetDisplayName(f"{name}_joint")


def prefix_usd_frames(usd: str, prefix: str, root: Path | str | None = None) -> str:
    if not prefix:
        return usd
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a subtree", root)
    stage = pxr.Usd.Stage.Open(layer)
    root = stage.GetDefaultPrim()
    for prim in stage.Traverse():
        if carries_a_frame(pxr, prim) or (root and prim == root):
            prim.SetDisplayName(f"{prefix}/{prim.GetDisplayName() or prim.GetName()}")
    return str(layer.ExportToString())


def usd_frame_names(usd: str, root: Path | str | None = None) -> set[str]:
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer", root))
    names = {
        prim.GetDisplayName() or prim.GetName()
        for prim in stage.Traverse()
        if carries_a_frame(pxr, prim)
    }
    default = stage.GetDefaultPrim()
    if default:
        names.add(default.GetDisplayName() or default.GetName())
    return names


def rename_usd_frames(usd: str, renames: Mapping[str, str], root: Path | str | None = None) -> str:
    if not renames:
        return usd
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a subtree", root)
    stage = pxr.Usd.Stage.Open(layer)
    frames = [prim for prim in stage.Traverse() if carries_a_frame(pxr, prim)]
    default = stage.GetDefaultPrim()
    if default and default not in frames:
        frames.append(default)
    named = [(prim, prim.GetDisplayName() or prim.GetName()) for prim in frames]
    changed = False
    for prim, name in named:
        if name in renames:
            prim.SetDisplayName(renames[name])
            changed = True
    return str(layer.ExportToString()) if changed else usd


def link_prim_paths(stage: Stage) -> dict[str, str]:
    pxr = load_pxr()
    paths: dict[str, str] = {}
    for prim in stage.Traverse():
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            paths[prim.GetDisplayName() or prim.GetName()] = str(prim.GetPath())
    root = stage.GetDefaultPrim()
    if root:
        paths.setdefault(root.GetDisplayName() or root.GetName(), str(root.GetPath()))
    return paths
