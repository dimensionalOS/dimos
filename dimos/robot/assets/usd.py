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

"""URDF to USD: one robot as a USD stage, and the grafting that joins subtrees onto it.

USD is becoming the description format, so this is the bridge while the robots are still
authored as URDF. A converted stage is a string (a ``.usda`` layer), never a file: the
description already travels as one string through ``UrdfManager.file_info``, and a stage
that names external layers would need a second resolution contract on the renderer side.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.robot.assets.model import JointDescription, LoadedRobotModel

logger = setup_logger()

# A prim's displayName carries the original URDF spelling of the link or joint it stands for.
# USD prim names are identifiers, so "base-link" and "base:link" would both come out
# "base_link"; displayName is the standard field for a name that is not so restricted, and it
# is what a frame_id is matched against.

_NO_USD = (
    "URDF to USD needs usd-core (pxr), which is in the 'scene-cooking' extra and ships no "
    "linux-aarch64 wheel. Install it with: uv sync --extra scene-cooking"
)


def _pxr() -> Any:
    """The pxr modules, imported on first use.

    Deferred because usd-core is an optional extra with no linux-aarch64 wheel: importing a
    UrdfManager on a Jetson must not need it, only asking one for a USD scene does.
    """
    try:
        from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics
    except ImportError as exc:  # pragma: no cover - exercised only where the extra is absent
        raise ImportError(_NO_USD) from exc
    return _Pxr(Gf=Gf, Sdf=Sdf, Tf=Tf, Usd=Usd, UsdGeom=UsdGeom, UsdPhysics=UsdPhysics)


@dataclass(frozen=True)
class _Pxr:
    Gf: Any
    Sdf: Any
    Tf: Any
    Usd: Any
    UsdGeom: Any
    UsdPhysics: Any


@dataclass(frozen=True)
class _Geometry:
    """One ``<visual>`` or ``<collision>`` of a link."""

    kind: str  # box | cylinder | sphere | mesh
    origin: NDArray[np.float64]
    # box: the three extents; cylinder: (radius, length); sphere: (radius,); mesh: ()
    size: tuple[float, ...]
    filename: str | None
    mesh_scale: tuple[float, float, float]
    color: tuple[float, float, float] | None
    collision: bool


def urdf_to_usd(
    model: LoadedRobotModel,
    *,
    name: str | None = None,
    include_collision: bool = True,
) -> str:
    """``model`` as one ``.usda`` layer.

    Links become nested Xforms down the kinematic tree, each posed at the URDF zero pose, so
    a viewer with no physics engine still shows the robot assembled. The joints are also
    written as ``UsdPhysics`` prims under ``/<name>/joints`` for consumers that articulate it.

    A ``<mesh>`` becomes a reference to the path the URDF named. Nothing is converted here:
    UsdManager already serves exactly those paths over ``render_file_request``, so resolving
    them is the consumer's job, the same as it already is for the URDF.

    ``name`` is the defaultPrim; it defaults to the URDF's own ``<robot name>``.
    """
    pxr = _pxr()
    root_element = ET.fromstring(model.xml)
    name = name or root_element.get("name") or "robot"
    links = _links_of(model.xml, include_collision=include_collision)
    children = _children_by_parent(model.joints)

    stage = pxr.Usd.Stage.CreateInMemory(f"{name}.usda")
    pxr.UsdGeom.SetStageUpAxis(stage, pxr.UsdGeom.Tokens.z)
    # URDF is metres and USD is unitless, so say so rather than leave a viewer to assume cm.
    pxr.UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = pxr.UsdGeom.Xform.Define(stage, f"/{_identifier(pxr, name)}")
    stage.SetDefaultPrim(root.GetPrim())

    paths: dict[str, str] = {}
    _write_link_tree(
        pxr,
        stage,
        parent_path=str(root.GetPath()),
        link=model.root_link,
        links=links,
        children=children,
        paths=paths,
    )
    _write_joints(pxr, stage, root_path=str(root.GetPath()), joints=model.joints, paths=paths)
    return str(stage.GetRootLayer().ExportToString())


def graft_usd(root_usd: str, subtrees: Mapping[str, str]) -> str:
    """``root_usd`` with each subtree copied in under the prim standing for its frame.

    Each key is a frame_id in ``root_usd`` (a URDF link, matched on its ``displayName``) and each value is a USD layer whose default prim is that subtree's root. The
    result is one layer again: specs are copied, not referenced, because the scene leaves as
    a string with no layer the far side could resolve.
    """
    pxr = _pxr()
    root_layer = _layer_from(pxr, root_usd, "root")
    root_stage = pxr.Usd.Stage.Open(root_layer)
    by_link = _link_prim_paths(root_stage)

    for frame_id, subtree_usd in subtrees.items():
        attach_path = by_link.get(frame_id)
        if attach_path is None:
            raise ValueError(
                f"cannot attach a subtree at {frame_id!r}: the root USD has no prim for that "
                f"frame (it has {sorted(by_link)})"
            )
        sub_layer = _layer_from(pxr, subtree_usd, frame_id)
        sub_stage = pxr.Usd.Stage.Open(sub_layer)
        source = sub_stage.GetDefaultPrim()
        if not source:
            raise ValueError(f"the subtree USD for {frame_id!r} declares no defaultPrim")
        target = pxr.Sdf.Path(attach_path).AppendChild(source.GetName())
        if not pxr.Sdf.CopySpec(sub_layer, source.GetPath(), root_layer, target):
            raise ValueError(f"could not copy the subtree USD for {frame_id!r} into the scene")
        # Copied specs keep their authored target paths, so a joint's body rels would still
        # point at the subtree's old root. Rebase them onto where the subtree now lives.
        _rebase_relationships(pxr, root_layer, target, str(source.GetPath()), str(target))

    return str(root_layer.ExportToString())


def prefix_usd_frames(usd: str, prefix: str) -> str:
    """``usd`` with every link and joint name it carries moved under ``prefix``.

    A description's link names are its tf frame_ids, and they come from the URDF, so every copy
    of one arm names the same frames. Two of that arm on one robot therefore need one of them
    renamed, and the namespace it is deployed under ("left" out of "left/xarm") is the name that
    is already unique. Joint names move with the links: they are what a JointState keys on, and
    two arms answering to one joint name is the same collision one layer down.

    Only ``displayName`` changes. Prim names cannot hold a "/" and prim paths are already unique
    by nesting, so neither needs to move. A prim with no displayName of its own is given one
    from its prim name, because the prefixed name is exactly the case the prim name cannot hold.
    """
    if not prefix:
        return usd
    pxr = _pxr()
    layer = _layer_from(pxr, usd, "a subtree")
    stage = pxr.Usd.Stage.Open(layer)
    for prim in stage.Traverse():
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI) or prim.IsA(pxr.UsdPhysics.Joint):
            prim.SetDisplayName(f"{prefix}/{prim.GetDisplayName() or prim.GetName()}")
    return str(layer.ExportToString())


def resolve_usd_assets(usd: str, root: str) -> str:
    """``usd`` with every relative asset path rewritten to sit under ``root``.

    A layer only says where its meshes are relative to something; the module that answered for
    it is the only thing that knows what. So a subtree is rebased as it arrives, before it joins
    a scene whose other subtrees were rooted somewhere else entirely.

    An absolute path is left alone, and so is everything when ``root`` is empty.
    """
    if not root:
        return usd
    pxr = _pxr()
    layer = _layer_from(pxr, usd, "a subtree")
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
        stack.extend(spec.nameChildren)
    return str(layer.ExportToString()) if changed else usd


def _layer_from(pxr: Any, usd: str, what: str) -> Any:
    layer = pxr.Sdf.Layer.CreateAnonymous(".usda")
    if not layer.ImportFromString(usd):
        raise ValueError(f"the USD for {what!r} is not a layer this build of USD can read")
    return layer


def _rebase_relationships(
    pxr: Any, layer: Any, root: Any, old_prefix: str, new_prefix: str
) -> None:
    """Rewrite every relationship target under ``root`` that still names the old subtree root."""
    stack = [layer.GetPrimAtPath(root)]
    while stack:
        spec = stack.pop()
        if spec is None:
            continue
        for prop in spec.properties:
            if not isinstance(prop, pxr.Sdf.RelationshipSpec):
                continue
            targets = list(prop.targetPathList.explicitItems)
            rebased = [
                pxr.Sdf.Path(str(t).replace(old_prefix, new_prefix, 1))
                if str(t).startswith(old_prefix)
                else t
                for t in targets
            ]
            if rebased != targets:
                prop.targetPathList.explicitItems = rebased
        stack.extend(spec.nameChildren)


def _link_prim_paths(stage: Any) -> dict[str, str]:
    """URDF link name -> prim path, for every link prim in ``stage``.

    A link is exactly a prim carrying ``UsdPhysics.RigidBodyAPI``, which is what this converter
    applies to each one. That is narrower than "any Xform": it leaves out the defaultPrim
    standing for the robot itself, and the joint prims, either of which could otherwise shadow
    a link that shares its name.

    The name comes from displayName, falling back to the prim name for a layer that authored
    none (they agree whenever the URDF name was already a USD identifier).
    """
    pxr = _pxr()
    paths: dict[str, str] = {}
    for prim in stage.Traverse():
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            paths[prim.GetDisplayName() or prim.GetName()] = str(prim.GetPath())
    return paths


def _write_link_tree(
    pxr: Any,
    stage: Any,
    *,
    parent_path: str,
    link: str,
    links: Mapping[str, Sequence[_Geometry]],
    children: Mapping[str, Sequence[JointDescription]],
    paths: dict[str, str],
    joint: JointDescription | None = None,
) -> None:
    """Define the prim for ``link`` under ``parent_path``, then recurse into its children."""
    prim_path = f"{parent_path}/{_identifier(pxr, link)}"
    if link in paths:
        # A URDF is a tree, so a second arrival means a malformed description; keep the first.
        logger.warning(f"URDF link {link!r} is reached more than once; ignoring the later edge")
        return
    xform = pxr.UsdGeom.Xform.Define(stage, prim_path)
    prim = xform.GetPrim()
    paths[link] = prim_path
    prim.SetDisplayName(link)
    pxr.UsdPhysics.RigidBodyAPI.Apply(prim)
    if joint is not None:
        xform.AddTransformOp().Set(_matrix(pxr, joint.parent_from_child(0.0)))

    for index, geometry in enumerate(links.get(link, ())):
        _write_geometry(
            pxr,
            stage,
            f"{prim_path}/{_geometry_name(geometry, index)}",
            geometry,
        )

    for child_joint in children.get(link, ()):
        _write_link_tree(
            pxr,
            stage,
            parent_path=prim_path,
            link=child_joint.child_link,
            links=links,
            children=children,
            paths=paths,
            joint=child_joint,
        )


def _geometry_name(geometry: _Geometry, index: int) -> str:
    return f"{'collision' if geometry.collision else 'visual'}_{index}"


def _write_geometry(pxr: Any, stage: Any, path: str, geometry: _Geometry) -> None:
    gprim = _define_gprim(pxr, stage, path, geometry)
    if gprim is None:
        return
    gprim.AddTransformOp().Set(_matrix(pxr, geometry.origin))
    if geometry.color is not None:
        gprim.CreateDisplayColorAttr([pxr.Gf.Vec3f(*geometry.color)])
    if geometry.collision:
        # Collision geometry is carried for planners and self-filters, not for rendering.
        pxr.UsdGeom.Imageable(gprim).CreateVisibilityAttr(pxr.UsdGeom.Tokens.invisible)
        pxr.UsdPhysics.CollisionAPI.Apply(gprim.GetPrim())


def _define_gprim(pxr: Any, stage: Any, path: str, geometry: _Geometry) -> Any:
    if geometry.kind == "box":
        cube = pxr.UsdGeom.Cube.Define(stage, path)
        # A Cube is a unit cube scaled to the box's extents: USD has no three-sided box.
        cube.CreateSizeAttr(1.0)
        cube.AddScaleOp().Set(pxr.Gf.Vec3f(*geometry.size))
        return cube
    if geometry.kind == "cylinder":
        cylinder = pxr.UsdGeom.Cylinder.Define(stage, path)
        cylinder.CreateAxisAttr(pxr.UsdGeom.Tokens.z)  # URDF cylinders are along z too
        cylinder.CreateRadiusAttr(geometry.size[0])
        cylinder.CreateHeightAttr(geometry.size[1])
        return cylinder
    if geometry.kind == "sphere":
        sphere = pxr.UsdGeom.Sphere.Define(stage, path)
        sphere.CreateRadiusAttr(geometry.size[0])
        return sphere
    if geometry.kind == "mesh" and geometry.filename is not None:
        return _define_mesh(pxr, stage, path, geometry)
    logger.warning(f"skipping URDF geometry of unsupported kind {geometry.kind!r}")
    return None


def _define_mesh(pxr: Any, stage: Any, path: str, geometry: _Geometry) -> Any:
    """A reference to the file the URDF named. Nothing is read, parsed or converted here.

    A reference is USD's own way to say "this geometry lives over there", so the mesh needs no
    attribute of our own and no triangles of ours in the layer. Resolving it is the consumer's
    job: UsdManager serves exactly these paths over render_file_request, and USD resolves a
    non-USD mesh through a file format plugin for that extension.
    """
    assert geometry.filename is not None
    mesh = pxr.UsdGeom.Mesh.Define(stage, path)
    mesh.AddScaleOp().Set(pxr.Gf.Vec3f(*geometry.mesh_scale))
    # The path the URDF named, kept verbatim: those are the names UsdManager answers for.
    mesh.GetPrim().GetReferences().AddReference(geometry.filename)
    return mesh


def _write_joints(
    pxr: Any,
    stage: Any,
    *,
    root_path: str,
    joints: Sequence[JointDescription],
    paths: Mapping[str, str],
) -> None:
    scope = pxr.UsdGeom.Scope.Define(stage, f"{root_path}/joints")
    for joint in joints:
        parent = paths.get(joint.parent_link)
        child = paths.get(joint.child_link)
        if parent is None or child is None:
            logger.warning(f"skipping joint {joint.name!r}: it names a link outside the tree")
            continue
        prim = _define_joint(pxr, stage, f"{scope.GetPath()}/{_identifier(pxr, joint.name)}", joint)
        if prim is None:
            continue
        prim.CreateBody0Rel().SetTargets([pxr.Sdf.Path(parent)])
        prim.CreateBody1Rel().SetTargets([pxr.Sdf.Path(child)])
        prim.GetPrim().SetDisplayName(joint.name)
        _set_joint_frames(pxr, prim, joint)
        _set_joint_limits(prim, joint)


def _define_joint(pxr: Any, stage: Any, path: str, joint: JointDescription) -> Any:
    if joint.type in {"revolute", "continuous"}:
        return pxr.UsdPhysics.RevoluteJoint.Define(stage, path)
    if joint.type == "prismatic":
        return pxr.UsdPhysics.PrismaticJoint.Define(stage, path)
    if joint.type == "fixed":
        return pxr.UsdPhysics.FixedJoint.Define(stage, path)
    # floating and planar have no UsdPhysics equivalent that matches URDF's semantics, and
    # their pose comes from odometry rather than the model (see UrdfManager).
    logger.debug(f"joint {joint.name!r} of type {joint.type!r} is left out of the USD")
    return None


def _set_joint_frames(pxr: Any, prim: Any, joint: JointDescription) -> None:
    """The joint's frame in each body.

    ``physics:axis`` is only ever X, Y or Z, so a URDF axis that is none of those is folded
    into both local rotations: the joint is authored about z with the bodies turned instead.
    """
    to_z = _rotation_to_z(joint.axis) if joint.type != "fixed" else Rotation.identity()
    origin = Rotation.from_euler("xyz", joint.origin_rpy)
    prim.CreateLocalPos0Attr(pxr.Gf.Vec3f(*joint.origin_xyz))
    prim.CreateLocalRot0Attr(_quat(pxr, origin * to_z))
    prim.CreateLocalPos1Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    prim.CreateLocalRot1Attr(_quat(pxr, to_z))
    if joint.type != "fixed":
        prim.CreateAxisAttr(pxr.UsdPhysics.Tokens.z)


def _set_joint_limits(prim: Any, joint: JointDescription) -> None:
    if joint.type == "continuous" or joint.lower is None or joint.upper is None:
        return  # no limit attributes at all means unlimited, which is what continuous is
    # A revolute limit is degrees in USD and radians in URDF; a prismatic limit is a distance.
    scale = 180.0 / np.pi if joint.type in {"revolute", "continuous"} else 1.0
    prim.CreateLowerLimitAttr(float(joint.lower * scale))
    prim.CreateUpperLimitAttr(float(joint.upper * scale))


def _matrix(pxr: Any, transform: NDArray[np.float64]) -> Any:
    """A 4x4 row-vector transform as USD's column-vector Gf.Matrix4d."""
    return pxr.Gf.Matrix4d(*np.asarray(transform, dtype=np.float64).T.reshape(-1).tolist())


def _quat(pxr: Any, rotation: Rotation) -> Any:
    x, y, z, w = rotation.as_quat()
    return pxr.Gf.Quatf(float(w), float(x), float(y), float(z))


def _rotation_to_z(axis: tuple[float, float, float]) -> Rotation:
    """A rotation taking +z onto ``axis``, so a joint authored about z turns about ``axis``."""
    vector = np.asarray(axis, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        return Rotation.identity()
    vector = vector / norm
    z = np.array([0.0, 0.0, 1.0])
    cross = np.cross(z, vector)
    sin = float(np.linalg.norm(cross))
    cos = float(np.dot(z, vector))
    if sin < 1e-9:
        # Parallel or antiparallel: a half turn about any perpendicular axis for the latter.
        return Rotation.identity() if cos > 0 else Rotation.from_rotvec([np.pi, 0.0, 0.0])
    return Rotation.from_rotvec(cross / sin * np.arctan2(sin, cos))


def _identifier(pxr: Any, name: str) -> str:
    """``name`` as a USD prim name; the original spelling stays on the prim as its displayName."""
    return str(pxr.Tf.MakeValidIdentifier(name))


def _children_by_parent(
    joints: Sequence[JointDescription],
) -> dict[str, list[JointDescription]]:
    children: dict[str, list[JointDescription]] = {}
    for joint in joints:
        children.setdefault(joint.parent_link, []).append(joint)
    return children


def _links_of(xml: str, *, include_collision: bool) -> dict[str, list[_Geometry]]:
    """Every link's visual (and optionally collision) geometry, by link name."""
    links: dict[str, list[_Geometry]] = {}
    for link in ET.fromstring(xml).iter("link"):
        name = link.get("name")
        if name is None:
            continue
        geometries = [_geometry_of(element, collision=False) for element in link.iter("visual")]
        if include_collision:
            geometries += [
                _geometry_of(element, collision=True) for element in link.iter("collision")
            ]
        links[name] = [geometry for geometry in geometries if geometry is not None]
    return links


def _geometry_of(element: ET.Element, *, collision: bool) -> _Geometry | None:
    shape = element.find("geometry")
    if shape is None:
        return None
    origin = _origin_of(element.find("origin"))
    color = _color_of(element.find("material"))
    if (box := shape.find("box")) is not None:
        size = _floats(box.get("size"), 3, (1.0, 1.0, 1.0))
        return _Geometry("box", origin, size, None, (1.0, 1.0, 1.0), color, collision)
    if (cylinder := shape.find("cylinder")) is not None:
        radius = float(cylinder.get("radius", 1.0))
        length = float(cylinder.get("length", 1.0))
        return _Geometry(
            "cylinder", origin, (radius, length), None, (1.0, 1.0, 1.0), color, collision
        )
    if (sphere := shape.find("sphere")) is not None:
        radius = float(sphere.get("radius", 1.0))
        return _Geometry("sphere", origin, (radius,), None, (1.0, 1.0, 1.0), color, collision)
    if (mesh := shape.find("mesh")) is not None:
        filename = mesh.get("filename")
        if filename is None:
            return None
        sx, sy, sz = _floats(mesh.get("scale"), 3, (1.0, 1.0, 1.0))
        return _Geometry("mesh", origin, (), filename, (sx, sy, sz), color, collision)
    return None


def _origin_of(element: ET.Element | None) -> NDArray[np.float64]:
    transform = np.eye(4)
    if element is None:
        return transform
    transform[:3, :3] = Rotation.from_euler(
        "xyz", _floats(element.get("rpy"), 3, (0.0,) * 3)
    ).as_matrix()
    transform[:3, 3] = _floats(element.get("xyz"), 3, (0.0,) * 3)
    return transform


def _color_of(element: ET.Element | None) -> tuple[float, float, float] | None:
    if element is None:
        return None
    color = element.find("color")
    if color is None:
        return None
    rgba = _floats(color.get("rgba"), 4, (0.8, 0.8, 0.8, 1.0))
    return (rgba[0], rgba[1], rgba[2])


def _floats(text: str | None, count: int, default: tuple[float, ...]) -> tuple[float, ...]:
    if not text:
        return default
    values = [float(part) for part in text.replace(",", " ").split()]
    if len(values) != count:
        logger.warning(f"expected {count} numbers in {text!r}, using {default}")
        return default
    return tuple(values)
