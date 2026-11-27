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

"""Frames: the prims dimos treats as tf frames, each named by its prim's own name.

A frame is the default prim, a prim with RigidBodyAPI, or a prim a joint names. Prims between
them (a ``Geometry`` scope, say) are not frames: a frame's parent is its nearest frame ancestor."""

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
    layer_from,
    load_pxr,
    prim_identifier,
    usd_matrix,
)


def frame_prims(stage: Stage) -> dict[str, Prim]:
    """Every prim that carries a frame, by frame name, parents before children.

    Raises ValueError if two frame prims share a name.
    """
    pxr = load_pxr()
    framed = set()
    default = stage.GetDefaultPrim()
    if default:
        framed.add(default.GetPath())
    for prim in stage.Traverse():
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            framed.add(prim.GetPath())
        elif prim.IsA(pxr.UsdPhysics.Joint):
            # A joint's bodies are frames even when static (no RigidBodyAPI): the joint is an edge between them.
            for relationship in ("physics:body0", "physics:body1"):
                framed.update(prim.GetRelationship(relationship).GetTargets())
    frames: dict[str, Prim] = {}
    for prim in stage.Traverse():
        if prim.GetPath() not in framed:
            continue
        name = str(prim.GetName())
        if name in frames:
            raise ValueError(
                f"the frame prims {frames[name].GetPath()} and {prim.GetPath()} are both named "
                f"{name!r}; a frame is named by its prim's own name, so rename one"
            )
        frames[name] = prim
    return frames


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
        frames_here = frame_prims(stage)
        if parent not in frames_here:
            raise ValueError(
                f"cannot add the frame {name!r}: the layer has no frame {parent!r} to hang it "
                f"off (it has {sorted(frames_here)[:8]}...)"
            )
        parent_path = frames_here[parent].GetPath()
        path = parent_path.AppendChild(name)
        if name in frames_here or stage.GetPrimAtPath(path):
            taken = frames_here[name].GetPath() if name in frames_here else path
            raise ValueError(f"cannot add the frame {name!r}: the name is taken by {taken}")
        prim = pxr.UsdGeom.Xform.Define(stage, path)
        pxr.UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        transform[:3, 3] = xyz
        prim.AddTransformOp().Set(usd_matrix(pxr, transform))
        _weld_frame(pxr, stage, str(parent_path), str(path), name, xyz, rpy)
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


def usd_frame_paths(usd: str, root: Path | str | None = None) -> dict[str, str]:
    """Each frame's prim path (``/go2/Geometry/base_link``), by frame name (``base_link``)."""
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer", root))
    return {name: str(prim.GetPath()) for name, prim in frame_prims(stage).items()}


def usd_frame_names(usd: str, root: Path | str | None = None) -> set[str]:
    return set(usd_frame_paths(usd, root))


def usd_top_frame(usd: str, root: Path | str | None = None) -> str:
    """The frame of the layer's default prim, the top of every frame in it."""
    pxr = load_pxr()
    # The stage is kept in a name: a prim of a stage already collected reads as absent.
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer", root))
    default = stage.GetDefaultPrim()
    if not default:
        raise ValueError("the layer declares no defaultPrim, so it has no top frame")
    return str(default.GetName())


def usd_frame_edges(
    usd: str, root: Path | str | None = None
) -> list[tuple[str, str, NDArray[np.float64]]]:
    """Every frame's (parent frame, frame, 4x4 transform from parent to frame).

    The parent is the frame's nearest frame ancestor; the transform includes the prims skipped
    on the way (a ``Geometry`` scope, say).
    """
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer", root))
    frames = frame_prims(stage)
    names = {prim.GetPath(): name for name, prim in frames.items()}
    edges = []
    for name, prim in frames.items():
        local = _local_transform(pxr, prim)
        parent = prim.GetParent()
        while parent and not parent.IsPseudoRoot() and parent.GetPath() not in names:
            local = _local_transform(pxr, parent) @ local
            parent = parent.GetParent()
        if parent and parent.GetPath() in names:
            edges.append((names[parent.GetPath()], name, local))
    return edges


def _local_transform(pxr: Pxr, prim: Prim) -> NDArray[np.float64]:
    if not prim.IsA(pxr.UsdGeom.Xformable):
        return np.eye(4)
    matrix = pxr.UsdGeom.Xformable(prim).GetLocalTransformation(pxr.Usd.TimeCode.Default())
    # Gf matrices are row-vector, so transposed.
    return np.array(matrix, dtype=np.float64).T


def usd_frame_pose(usd: str, frame: str, root: Path | str | None = None) -> NDArray[np.float64]:
    """Where ``frame`` sits in the frame this USD is mounted on (its default prim's parent), as a 4x4."""
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer", root))
    prim = frame_prims(stage)[frame]
    pose = pxr.UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(pxr.Usd.TimeCode.Default())
    # Gf matrices are row-vector, so transposed.
    return np.array(pose, dtype=np.float64).T
