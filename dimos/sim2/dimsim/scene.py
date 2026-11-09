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

"""A USD scene as Rapier holds it: one body per rigid body, its shapes as colliders in its frame.

`misc/DimSim/cli/sim2/world.ts` takes `scene_data.v1` bodies on its `init` op -- a pose, and
colliders placed in that pose's frame -- so this is the USD side of that one wire format. A prim
carrying `UsdPhysics.RigidBodyAPI` is a dynamic body and every gprim under it is one of its
colliders; a gprim under no such prim is a fixed body of its own, which is what a wall is.

Z is up in USD here and Z is up in world.ts, so no axis moves. The quaternions do: world.ts
reads a quat as x, y, z, w, which is neither USD's order nor MuJoCo's.
"""

import base64
from collections.abc import Iterable, Sequence
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.sim2.mujoco.names import short_names

if TYPE_CHECKING:
    from dimos.robot.assets.model import UsdModel

# What a dynamic body weighs where its layer never said. Rapier needs a number, and a kilogram is
# light enough that walking into the thing moves it, which is the point of a prop.
_DEFAULT_MASS = 1.0
# Every purpose, so a prim modelled as a proxy still has a bound to fall back on.
_PURPOSES = ["default", "proxy", "render", "guide"]


def dimsim_bodies(
    usd: "UsdModel", exclude: Iterable[str] = ()
) -> tuple[list[dict[str, Any]], dict[str, dict[str, str]]]:
    """The scene's bodies and the meshes they name, as world.ts's `init` op wants them.

    `exclude` holds robot ids. A composed scene arrives with the robots already grafted into it
    and this engine replaces a whole robot with one kinematic box, so every prim belonging to one
    is left out.
    """
    from dimos.robot.assets.usd import _layer_from, _pxr

    pxr = _pxr()
    stage = pxr.Usd.Stage.Open(_layer_from(pxr, usd.usd, "a scene", usd._asset_root()))
    cache = pxr.UsdGeom.BBoxCache(pxr.Usd.TimeCode.Default(), _PURPOSES)
    owners = tuple(exclude)
    bodies: dict[str, dict[str, Any]] = {}
    frames: dict[str, Any] = {}
    meshes: dict[str, dict[str, str]] = {}
    for prim in stage.Traverse():
        if not prim.IsA(pxr.UsdGeom.Gprim) or _owned(prim.GetPath().pathString, owners):
            continue
        rigid = _rigid_body(pxr, prim)
        owner = rigid if rigid is not None else prim
        path = owner.GetPath().pathString
        if path not in bodies:
            pose, frames[path] = _pose(pxr, owner)
            bodies[path] = {"name": path, **pose, "geoms": []}
            if rigid is not None:
                bodies[path]["mass"] = _mass(pxr, rigid)
        geom = _geom(pxr, prim, frames[path], cache, meshes)
        if geom is not None:
            bodies[path]["geoms"].append(geom)
    found = [body for body in bodies.values() if body["geoms"]]
    # Only the dynamic ones, and for the reason `short_names` gives: a crate that can be pushed is
    # an entity an action addresses, and a fixed wall is never reported at all.
    short = short_names(body["name"] for body in found if "mass" in body)
    for body in found:
        body["name"] = short.get(body["name"], body["name"])
    return found, meshes


def _owned(path: str, owners: Sequence[str]) -> bool:
    """Whether this prim belongs to one of the robots.

    The convention `dimos/sim2/mujoco/names.py::_wanted` states and documents: a grafted robot's
    subtree is nested under a prim named after its id.
    """
    return any(f"/{owner}/" in path or path.startswith(f"{owner}/") for owner in owners)


def _rigid_body(pxr: Any, prim: Any) -> Any:
    """The nearest prim at or above this one that is a rigid body, or None where there is none."""
    while prim and not prim.IsPseudoRoot():
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            return prim
        prim = prim.GetParent()
    return None


def _pose(pxr: Any, prim: Any) -> tuple[dict[str, Any], Any]:
    """A prim's world pose, and the inverse of it so its shapes can be put in its frame.

    Rotation and translation only: a Rapier body has no scale, so a scale from anywhere above a
    shape is left in the shape's own transform and baked into its size there.
    """
    world = pxr.UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(pxr.Usd.TimeCode.Default())
    placed = pxr.Gf.Transform(world)
    translation = placed.GetTranslation()
    quat = placed.GetRotation().GetQuat()
    rigid = pxr.Gf.Matrix4d().SetRotate(quat) * pxr.Gf.Matrix4d().SetTranslate(translation)
    return (
        {"pos": [float(v) for v in translation], "quat": _xyzw(quat)},
        rigid.GetInverse(),
    )


def _geom(pxr: Any, prim: Any, inverse: Any, cache: Any, meshes: dict[str, Any]) -> Any:
    """One gprim as a collider in its body's frame, or None where it has no shape at all."""
    local = (
        pxr.UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(pxr.Usd.TimeCode.Default())
        * inverse
    )
    placed = pxr.Gf.Transform(local)
    name = prim.GetPath().pathString
    shape = _shape(pxr, prim, [abs(float(v)) for v in placed.GetScale()], meshes, name)
    if shape is not None:
        return {
            "name": name,
            "pos": [float(v) for v in placed.GetTranslation()],
            "quat": _xyzw(placed.GetRotation().GetQuat()),
            **shape,
        }
    # Nothing Rapier has a shape for: the box it fits in, which is what the character controller
    # has to walk around either way.
    aligned = cache.ComputeWorldBound(prim).Transform(inverse).ComputeAlignedRange()
    if aligned.IsEmpty():
        return None
    low, high = np.array(aligned.GetMin()), np.array(aligned.GetMax())
    return {
        "name": name,
        "pos": ((low + high) / 2).tolist(),
        "quat": [0.0, 0.0, 0.0, 1.0],
        "type": "box",
        "size": ((high - low) / 2).tolist(),
    }


def _shape(
    pxr: Any, prim: Any, scale: list[float], meshes: dict[str, Any], name: str
) -> dict[str, Any] | None:
    """The `type` and `size` world.ts's `shape()` reads, with the scale above the prim baked in.

    A capsule's or a cylinder's `size` is (radius, half height) and z-aligned, which is what
    world.ts rotates y-to-z for itself. One whose USD axis is not Z falls through to the bounding
    box rather than being silently laid on its side.
    """
    sx, sy, sz = scale
    if prim.IsA(pxr.UsdGeom.Cube):
        half = float(pxr.UsdGeom.Cube(prim).GetSizeAttr().Get() or 2.0) / 2
        return {"type": "box", "size": [half * sx, half * sy, half * sz]}
    if prim.IsA(pxr.UsdGeom.Sphere):
        radius = float(pxr.UsdGeom.Sphere(prim).GetRadiusAttr().Get() or 1.0)
        return {"type": "sphere", "size": [radius * max(scale)]}
    for schema, kind in ((pxr.UsdGeom.Capsule, "capsule"), (pxr.UsdGeom.Cylinder, "cylinder")):
        if not prim.IsA(schema):
            continue
        shaped = schema(prim)
        if str(shaped.GetAxisAttr().Get() or "Z") != "Z":
            return None
        radius = float(shaped.GetRadiusAttr().Get() or 1.0) * max(sx, sy)
        half = float(shaped.GetHeightAttr().Get() or 1.0) / 2 * sz
        return {"type": kind, "size": [radius, half]}
    if prim.IsA(pxr.UsdGeom.Plane):
        plane = pxr.UsdGeom.Plane(prim)
        if str(plane.GetAxisAttr().Get() or "Z") != "Z":
            return None
        width = float(plane.GetWidthAttr().Get() or 0.0) / 2 * sx
        length = float(plane.GetLengthAttr().Get() or 0.0) / 2 * sy
        return {"type": "plane", "size": [width, length, 0.0]}
    if prim.IsA(pxr.UsdGeom.Mesh):
        mesh = _trimesh(pxr, prim, scale)
        if mesh is None:
            return None
        meshes[name] = mesh
        return {"type": "mesh", "size": [0.0, 0.0, 0.0], "mesh": name}
    return None


def _trimesh(pxr: Any, prim: Any, scale: list[float]) -> dict[str, str] | None:
    """The mesh as world.ts reads one: base64 little-endian float32 points and uint32 triangles.

    The scale is baked into the points because a Rapier trimesh has none, and a polygon of more
    than three sides is fanned, because Rapier takes triangles only.
    """
    mesh = pxr.UsdGeom.Mesh(prim)
    points = np.asarray(mesh.GetPointsAttr().Get() or [], dtype="<f4")
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get() or [], dtype=np.int64)
    indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get() or [], dtype=np.int64)
    if points.size == 0 or counts.size == 0:
        return None
    triangles = []
    at = 0
    for count in counts:
        fan = indices[at : at + count]
        at += count
        for corner in range(1, len(fan) - 1):
            triangles.append((fan[0], fan[corner], fan[corner + 1]))
    if not triangles:
        return None
    scaled = (points.reshape(-1, 3) * np.asarray(scale, dtype="<f4")).astype("<f4")
    return {
        "vertices": _b64(scaled),
        "faces": _b64(np.asarray(triangles, dtype="<u4")),
    }


def _mass(pxr: Any, prim: Any) -> float:
    """What a dynamic body weighs: its own authored mass, else its shapes' summed, else the default.

    The shapes are summed because a converted layer puts `physics:mass` on the gprim and not on
    the body carrying the RigidBodyAPI, which is where the toy robot's box and head have theirs.
    """
    own = pxr.UsdPhysics.MassAPI(prim).GetMassAttr()
    if own and own.HasAuthoredValue() and float(own.Get() or 0.0) > 0:
        return float(own.Get())
    total = 0.0
    for child in pxr.Usd.PrimRange(prim):
        attribute = pxr.UsdPhysics.MassAPI(child).GetMassAttr()
        if attribute and attribute.HasAuthoredValue():
            total += float(attribute.Get() or 0.0)
    return total if total > 0 else _DEFAULT_MASS


def _xyzw(quat: Any) -> list[float]:
    imaginary = quat.GetImaginary()
    return [float(imaginary[0]), float(imaginary[1]), float(imaginary[2]), float(quat.GetReal())]


def _b64(array: np.ndarray) -> str:
    return base64.b64encode(np.ascontiguousarray(array).tobytes()).decode()
