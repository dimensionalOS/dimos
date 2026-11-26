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

"""Frames: the prims dimos treats as tf frames, each named by its prim path (``frame_path``).

A frame is the default prim, a prim with RigidBodyAPI or named by a joint, or a prim on the path
between them (a ``Geometry`` scope, say), so every frame's path-parent is a frame too."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.protocol.tf.frame_paths import check_frame_path
from dimos.robot.assets.usd_utils.pxr import (
    Prim,
    Pxr,
    Stage,
    Vector,
    frame_path,
    layer_from,
    load_pxr,
    prim_identifier,
    usd_matrix,
)


def frame_prims(stage: Stage) -> list[Prim]:
    """Every prim that carries a frame, parents before children."""
    pxr = load_pxr()
    framed = set()
    default = stage.GetDefaultPrim()
    if default:
        framed.add(default.GetPath())
    bodies = []
    for prim in stage.Traverse():
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            bodies.append(prim)
        elif prim.IsA(pxr.UsdPhysics.Joint):
            # A joint's bodies are frames even when static (no RigidBodyAPI): the joint is an edge between them.
            for relationship in ("physics:body0", "physics:body1"):
                bodies += [
                    stage.GetPrimAtPath(t) for t in prim.GetRelationship(relationship).GetTargets()
                ]
    for prim in bodies:
        while prim and prim.GetPath() != pxr.Sdf.Path.absoluteRootPath:
            framed.add(prim.GetPath())
            prim = prim.GetParent()
    return [prim for prim in stage.Traverse() if prim.GetPath() in framed]


def usd_with_frames(
    usd: str, frames: Mapping[str, tuple[str, Vector, Vector]], root: Path | str | None = None
) -> str:
    if not frames:
        return usd
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a scene", root)
    stage = pxr.Usd.Stage.Open(layer)
    for name, (parent, xyz, rpy) in frames.items():
        if "/" in name:
            raise ValueError(
                f"cannot add the frame {name!r}: a frame is named on its own and hung off its "
                f"parent ({parent!r}), so its name has no '/'"
            )
        check_frame_path(name, "the added frame")
        by_link = link_prim_paths(stage)
        parent_path = by_link.get(parent)
        if parent_path is None:
            raise ValueError(
                f"cannot add the frame {name!r}: the layer has no frame {parent!r} to hang it "
                f"off (it has {sorted(by_link)[:8]}...)"
            )
        path = pxr.Sdf.Path(parent_path).AppendChild(name)
        if stage.GetPrimAtPath(path):
            raise ValueError(f"cannot add the frame {name!r}: {path} is taken")
        prim = pxr.UsdGeom.Xform.Define(stage, path)
        pxr.UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        transform[:3, 3] = xyz
        prim.AddTransformOp().Set(usd_matrix(pxr, transform))
        _weld_frame(pxr, stage, parent_path, str(path), name, xyz, rpy)
    return str(layer.ExportToString())


def _weld_frame(
    pxr: Pxr, stage: Stage, parent_path: str, path: str, name: str, xyz: Vector, rpy: Vector
) -> None:
    """Weld the added frame where its own transform puts it.

    A joint's local frames are what a reader has to go on -- the prim transform places the
    body, but anything rebuilding the kinematics (usd_to_urdf, and so tf) reads body0's local
    pose. Leaving them identity says the child sits on top of its parent, which silently drops
    the offset and the rotation the frame was added for.
    """
    parent = stage.GetPrimAtPath(pxr.Sdf.Path(parent_path))
    joint = pxr.UsdPhysics.FixedJoint.Define(
        stage, pxr.Sdf.Path(path).AppendChild(prim_identifier(pxr, f"{name}_joint"))
    )
    if parent and parent.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
        joint.CreateBody0Rel().SetTargets([parent.GetPath()])
    joint.CreateBody1Rel().SetTargets([pxr.Sdf.Path(path)])
    x, y, z, w = Rotation.from_euler("xyz", rpy).as_quat()
    joint.CreateLocalPos0Attr(pxr.Gf.Vec3f(*(float(value) for value in xyz)))
    joint.CreateLocalRot0Attr(pxr.Gf.Quatf(float(w), float(x), float(y), float(z)))
    joint.CreateLocalPos1Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.GetPrim().SetDisplayName(f"{name}_joint")


def usd_frame_names(usd: str, root: Path | str | None = None) -> set[str]:
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer", root))
    return {frame_path(prim) for prim in frame_prims(stage)}


def usd_top_frame(usd: str, root: Path | str | None = None) -> str:
    """The frame of the layer's default prim, the top of every frame in it."""
    pxr = load_pxr()
    # The stage is kept in a name: a prim of a stage already collected reads as absent.
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer", root))
    default = stage.GetDefaultPrim()
    if not default:
        raise ValueError("the layer declares no defaultPrim, so it has no top frame")
    return frame_path(default)


def usd_frame_edges(
    usd: str, root: Path | str | None = None
) -> list[tuple[str, str, NDArray[np.float64]]]:
    """Every frame's (path-parent, frame, 4x4 local transform): where its own prim transform puts it."""
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer", root))
    framed = frame_prims(stage)
    paths = {prim.GetPath() for prim in framed}
    edges = []
    for prim in framed:
        parent = prim.GetParent()
        if parent.GetPath() not in paths:
            continue
        local = np.eye(4)
        if prim.IsA(pxr.UsdGeom.Xformable):
            matrix = pxr.UsdGeom.Xformable(prim).GetLocalTransformation(pxr.Usd.TimeCode.Default())
            # Gf matrices are row-vector, so transposed.
            local = np.array(matrix, dtype=np.float64).T
        edges.append((frame_path(parent), frame_path(prim), local))
    return edges


def usd_frame_pose(usd: str, frame: str, root: Path | str | None = None) -> NDArray[np.float64]:
    """Where ``frame`` (a frame, or any prim's path, such as a site's) sits in the frame this USD is mounted on (its default prim's parent), as a 4x4."""
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer", root))
    prim = stage.GetPrimAtPath(link_prim_paths(stage).get(frame, f"/{frame}"))
    if not prim.IsValid():
        raise KeyError(f"the USD has no frame or prim {frame!r}")
    pose = pxr.UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(pxr.Usd.TimeCode.Default())
    # Gf matrices are row-vector, so transposed.
    return np.array(pose, dtype=np.float64).T


def link_prim_paths(stage: Stage) -> dict[str, str]:
    """Each frame's name (its path without the leading "/") -> its prim path."""
    return {frame_path(prim): str(prim.GetPath()) for prim in frame_prims(stage)}
