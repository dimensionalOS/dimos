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
between them (a ``Geometry`` scope, say), so every frame's path-parent is a frame too.

A HardwareModule's frame is called on tf ``<namespace>/<role>`` when a role (a mount's name or a
tf_frames_local key) names it, else ``<namespace>/<prim path>``. It serves its USD with
``DimosFramesAPI`` on the top prim: ``dimos:namespace``, and ``rel dimos:frame:<namespace>:<role> =
<prim>`` per named frame, so whoever holds a world USD (its parts moved under their mounts) turns a
tf frame id back into its prim (``world_frame_prims``)."""

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
        frames_here = {frame_path(prim) for prim in frame_prims(stage)}
        if parent not in frames_here:
            raise ValueError(
                f"cannot add the frame {name!r}: the layer has no frame {parent!r} to hang it "
                f"off (it has {sorted(frames_here)[:8]}...)"
            )
        path = pxr.Sdf.Path(f"/{parent}").AppendChild(name)
        if stage.GetPrimAtPath(path):
            raise ValueError(f"cannot add the frame {name!r}: {path} is taken")
        prim = pxr.UsdGeom.Xform.Define(stage, path)
        pxr.UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        transform[:3, 3] = xyz
        prim.AddTransformOp().Set(usd_matrix(pxr, transform))
        _weld_frame(pxr, stage, f"/{parent}", str(path), name, xyz, rpy)
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
    """Where ``frame`` sits in the frame this USD is mounted on (its default prim's parent), as a 4x4."""
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer", root))
    if frame not in {frame_path(prim) for prim in frame_prims(stage)}:
        raise KeyError(frame)
    prim = stage.GetPrimAtPath(f"/{frame}")
    pose = pxr.UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(pxr.Usd.TimeCode.Default())
    # Gf matrices are row-vector, so transposed.
    return np.array(pose, dtype=np.float64).T


FRAMES_API = "DimosFramesAPI"
_FRAME_REL = "dimos:frame:"
_NAMESPACE_ATTR = "dimos:namespace"


def has_frames_api(prim: Prim) -> bool:
    """Whether ``prim`` applies DimosFramesAPI, read off its apiSchemas (the schema is never registered)."""
    listed = prim.GetMetadata("apiSchemas")
    return listed is not None and FRAMES_API in listed.ApplyOperations([])


def usd_with_frame_names(
    usd: str, namespace: str, named: Mapping[str, str], root: Path | str | None = None
) -> str:
    """``usd`` with DimosFramesAPI on its top prim: the ``namespace`` its frames' tf ids are under, and ``rel dimos:frame:<tf frame id, ':' for '/'> = <prim>`` for each named (prim path -> tf frame id)."""
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a layer", root)
    stage = pxr.Usd.Stage.Open(layer)
    top = stage.GetDefaultPrim()
    if not top:
        raise ValueError(
            "the layer declares no defaultPrim, so it has no prim to name its frames on"
        )
    if not has_frames_api(top):
        top.AddAppliedSchema(FRAMES_API)
    top.CreateAttribute(_NAMESPACE_ATTR, pxr.Sdf.ValueTypeNames.String, custom=False).Set(namespace)
    for path, frame_id in sorted(named.items()):
        rel = top.CreateRelationship(_FRAME_REL + frame_id.replace("/", ":"), custom=False)
        rel.SetTargets([pxr.Sdf.Path(f"/{path}")])
    return str(layer.ExportToString())


def world_frame_prims(stage: Stage) -> dict[str, str]:
    """Every tf frame id of a stage -> its prim path (no leading "/"): a part's (DimosFramesAPI prim's) named frame by its rel, its other prims by its namespace and their path under where it was put, and a prim no part holds by its own path."""
    pxr = load_pxr()
    found: dict[str, str] = {}

    def add(frame_id: str, path: str) -> None:
        if found.setdefault(frame_id, path) != path:
            raise ValueError(f"the tf frame {frame_id!r} is both {found[frame_id]!r} and {path!r}")

    # What no part holds (a scene layer written by hand) is called by its prim path.
    prims = iter(pxr.Usd.PrimRange(stage.GetPseudoRoot()))
    for prim in prims:
        if prim.IsPseudoRoot():
            continue
        if has_frames_api(prim):
            prims.PruneChildren()
            continue
        add(str(prim.GetPath())[1:], str(prim.GetPath())[1:])
    tops = [prim for prim in stage.Traverse() if has_frames_api(prim)]
    for top in tops:
        attribute = top.GetAttribute(_NAMESPACE_ATTR)
        namespace = str(attribute.Get()) if attribute and attribute.HasAuthoredValue() else ""
        named: set[str] = set()
        for rel in top.GetRelationships():
            if not rel.GetName().startswith(_FRAME_REL):
                continue
            targets = rel.GetTargets()
            if len(targets) != 1:
                raise ValueError(
                    f"{rel.GetPath()} names {len(targets)} prims; a tf frame is one prim"
                )
            add(rel.GetName()[len(_FRAME_REL) :].replace(":", "/"), str(targets[0])[1:])
            named.add(str(targets[0]))
        placed = top.GetParent().GetPath()
        prims = iter(pxr.Usd.PrimRange(top))
        for prim in prims:
            if prim != top and has_frames_api(prim):
                prims.PruneChildren()
                continue
            if str(prim.GetPath()) in named:
                continue
            relative = str(prim.GetPath().MakeRelativePath(placed))
            add(f"{namespace}/{relative}" if namespace else relative, str(prim.GetPath())[1:])
    return found


def usd_world_frame_prims(usd: str, root: Path | str | None = None) -> dict[str, str]:
    pxr = load_pxr()
    return world_frame_prims(pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer", root)))
