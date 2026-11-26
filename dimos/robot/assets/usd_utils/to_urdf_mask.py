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


from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any
import warnings
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.robot.assets.usd_utils.from_urdf import OUT_OF_PLANE, PASSTHROUGH_KEY
from dimos.robot.assets.usd_utils.pxr import Prim, Pxr, Stage, frame_path, layer_from, load_pxr
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()

_DEGREES = 180.0 / np.pi

_JOINT_TYPES = {
    "PhysicsRevoluteJoint": "revolute",
    "PhysicsPrismaticJoint": "prismatic",
    "PhysicsFixedJoint": "fixed",
    "PhysicsJoint": "floating",
}


def usd_to_urdf(usd: str, *, name: str | None = None, root: Path | str | None = None) -> str:
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a layer", root)
    stage = pxr.Usd.Stage.Open(layer)

    links = _usd_links_of(pxr, stage)
    world = _world_body(stage)
    joint_prims = _joint_prims(pxr, stage)
    # Every name the layer already uses: a link or joint this converter adds must not take one.
    taken = set(links) | {str(prim.GetDisplayName() or prim.GetName()) for prim in joint_prims}
    joints = [joint for prim in joint_prims for joint in _joints_of(pxr, prim, taken, world)]
    links |= {joint.child: None for joint in joints if joint.child_is_frame}
    links |= _static_bodies_of(pxr, stage, links, joints)

    default_prim = stage.GetDefaultPrim()
    robot_name = name or (default_prim.GetName() if default_prim else "robot")
    robot = ET.Element("robot", name=robot_name)

    for link_name, prim in links.items():
        _write_link(pxr, robot, link_name, prim)
    joints = joints + _welds_of(pxr, links, joints, taken)
    for joint in joints:
        _write_joint(robot, joint)

    _warn_about_orphans(links, joints)
    return ET.tostring(robot, encoding="unicode")


def usd_root_link(usd: str, root: Path | str | None = None) -> str:
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer", root))
    links = _usd_links_of(pxr, stage)
    joints = list(_joint_prims(pxr, stage))
    children = {_body_name(prim, "physics:body1") for prim in joints} | _nested_link_names(
        pxr, links
    )
    roots = [name for name in links if name not in children]
    if not roots:
        return _world_root(pxr, stage, joints) or next(iter(links), "")
    if len(roots) > 1:
        logger.warning(f"this layer has more than one root link ({roots}); taking the first")
    return roots[0]


def _world_root(pxr: Pxr, stage: Stage, joints: list[Any]) -> str:
    parents = {name for prim in joints if (name := _body_name(prim, "physics:body0"))}
    assert pxr and stage
    return next(iter(parents)) if len(parents) == 1 else ""


def _nested_link_names(pxr: Pxr, links: dict[str, Any]) -> set[str]:
    nested = set()
    for name, prim in links.items():
        if _link_ancestor(pxr, prim) is not None:
            nested.add(name)
    return nested


def _link_ancestor(pxr: Pxr, prim: Prim) -> Any | None:
    parent = prim.GetParent()
    while parent and parent.GetPath() != pxr.Sdf.Path.absoluteRootPath:
        if parent.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            return parent
        parent = parent.GetParent()
    return None


def _usd_links_of(pxr: Pxr, stage: Stage) -> dict[str, Any]:
    links: dict[str, Any] = {}
    for prim in stage.Traverse():
        if not prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            continue
        links[frame_path(prim)] = prim
    return links


def _world_body(stage: Stage) -> str | None:
    root = stage.GetDefaultPrim()
    return frame_path(root) if root else None


def _static_bodies_of(
    pxr: Pxr, stage: Stage, links: dict[str, Any], joints: list[_Joint]
) -> dict[str, Any]:
    named = {joint.parent for joint in joints} | {joint.child for joint in joints}
    wanted = named - set(links)
    if not wanted:
        return {}
    found: dict[str, Any] = {}
    for prim in stage.Traverse():
        name = frame_path(prim)
        if name in wanted:
            found[name] = prim
    return found


def _joint_prims(pxr: Pxr, stage: Stage) -> list[Any]:
    return [prim for prim in stage.Traverse() if prim.IsA(pxr.UsdPhysics.Joint)]


class _Joint:
    def __init__(
        self,
        name: str,
        kind: str,
        parent: str,
        child: str,
        origin_xyz: NDArray[np.float64],
        origin_rpy: NDArray[np.float64],
        axis: NDArray[np.float64],
        lower: float | None,
        upper: float | None,
        extras: list[str],
        child_is_frame: bool = False,
    ) -> None:
        self.name = name
        self.kind = kind
        self.parent = parent
        self.child = child
        self.origin_xyz = origin_xyz
        self.origin_rpy = origin_rpy
        self.axis = axis
        self.lower = lower
        self.upper = upper
        self.extras = extras
        # The child is a frame this reader made up (a joint's anchor), so no prim backs it.
        self.child_is_frame = child_is_frame


def _unused_name(base: str, taken: set[str]) -> str:
    """``base``, or ``base_2``, ``base_3``... if the layer already uses it; claimed in ``taken``."""
    name, index = base, 1
    while name in taken:
        index += 1
        name = f"{base}_{index}"
    taken.add(name)
    return name


def _joints_of(pxr: Pxr, prim: Prim, taken: set[str], world: str | None = None) -> list[_Joint]:
    """The URDF joints one USD joint becomes: itself, or itself and the weld off its anchor.

    UsdPhysics puts the joint at localPose0 in body0 and at localPose1 in body1, so at rest
    the child sits at parent_from_joint * inverse(child_from_joint). URDF has no localPose1:
    its joint frame is the child's frame. A fixed joint folds the two into one origin, but a
    moving joint anchored off the child's origin turns about the anchor, so the joint ends at
    a frame on the anchor (oriented like the child) and the child is welded off it.
    """
    joint = pxr.UsdPhysics.Joint(prim)
    named = _body_name(prim, "physics:body0")
    parent = named or world
    child = _body_name(prim, "physics:body1")
    if parent is None or child is None:
        logger.warning(f"skipping joint {prim.GetPath()}: it does not name both bodies")
        return []

    kind = _JOINT_TYPES.get(str(prim.GetTypeName()), "fixed")
    if kind == "floating":
        kind = _free_kind(prim)
    lower, upper = _limits_of(prim, kind)
    if kind == "revolute" and lower is None and upper is None:
        kind = "continuous"

    rot0 = _quat_rotation(joint.GetLocalRot0Attr().Get())
    rot1 = _quat_rotation(joint.GetLocalRot1Attr().Get())
    pos0 = _vector_of(joint.GetLocalPos0Attr().Get())
    pos1 = _vector_of(joint.GetLocalPos1Attr().Get())
    axis = rot1.apply(_axis_of(prim))

    child_rotation = rot0 * rot1.inv()
    child_xyz = pos0 - child_rotation.apply(pos1)
    if named is None:
        # Welded to the world: the child's own transform is where it sits, and the joint
        # carries no offset of its own.
        world_prim = prim.GetStage().GetDefaultPrim()
        child_xyz, child_rotation, _ = _decompose(
            pxr, _pose_in(pxr, _body_prim(prim, "physics:body1"), world_prim)
        )

    name = prim.GetDisplayName() or prim.GetName()
    anchored = kind != "fixed" and bool(np.any(pos1))
    origin_xyz = child_xyz + child_rotation.apply(pos1) if anchored else child_xyz
    moving = _Joint(
        name=name,
        kind=kind,
        parent=parent,
        child=_unused_name(f"{name}_anchor", taken) if anchored else child,
        origin_xyz=origin_xyz,
        origin_rpy=_rpy_of(child_rotation),
        axis=axis,
        lower=lower,
        upper=upper,
        extras=_passthrough_of(prim),
        child_is_frame=anchored,
    )
    if not anchored:
        return [moving]
    weld = _Joint(
        name=_unused_name(f"{name}_anchor_weld", taken),
        kind="fixed",
        parent=moving.child,
        child=child,
        origin_xyz=-pos1,
        origin_rpy=np.zeros(3),
        axis=np.array([0.0, 0.0, 1.0]),
        lower=None,
        upper=None,
        extras=[],
    )
    return [moving, weld]


def _vector_of(value: Any) -> NDArray[np.float64]:
    if value is None:
        return np.zeros(3)
    return np.asarray([value[0], value[1], value[2]], dtype=np.float64)


def _free_kind(prim: Prim) -> str:
    applied = {str(name) for name in prim.GetAppliedSchemas()}
    if all(f"PhysicsLimitAPI:{axis}" in applied for axis in OUT_OF_PLANE):
        return "planar"
    return "floating"


def _body_prim(prim: Prim, relationship: str) -> Prim:
    targets = prim.GetRelationship(relationship).GetTargets()
    return prim.GetStage().GetPrimAtPath(targets[0])


def _body_name(prim: Prim, relationship: str) -> str | None:
    targets = prim.GetRelationship(relationship).GetTargets()
    if not targets:
        return None
    body = prim.GetStage().GetPrimAtPath(targets[0])
    if not body:
        logger.warning(f"{prim.GetPath()} {relationship} points at {targets[0]}, which is absent")
        return None
    return frame_path(body)


def _limits_of(prim: Prim, kind: str) -> tuple[float | None, float | None]:
    lower_attr = prim.GetAttribute("physics:lowerLimit")
    upper_attr = prim.GetAttribute("physics:upperLimit")
    if not lower_attr or not upper_attr:
        return None, None
    if not lower_attr.HasAuthoredValue() or not upper_attr.HasAuthoredValue():
        return None, None
    lower, upper = lower_attr.Get(), upper_attr.Get()
    if lower is None or upper is None:
        return None, None
    scale = 1.0 / _DEGREES if kind in {"revolute", "continuous"} else 1.0
    return float(lower) * scale, float(upper) * scale


def _quat_rotation(quat: Any) -> Rotation:
    if quat is None:
        return Rotation.identity()
    imaginary = quat.GetImaginary()
    return Rotation.from_quat([imaginary[0], imaginary[1], imaginary[2], quat.GetReal()])


def _passthrough_of(prim: Prim) -> list[str]:
    data = prim.GetCustomData() or {}
    urdf = data.get("urdf") or {}
    return [str(fragment) for fragment in urdf.get(PASSTHROUGH_KEY.split(":", 1)[1], ())]


def _write_link(pxr: Pxr, root: ET.Element, link_name: str, prim: Prim | None) -> None:
    element = ET.SubElement(root, "link", name=link_name)
    if prim is None:
        return
    for shape in _shapes_under(pxr, prim):
        _write_geometry(pxr, element, shape, prim)
    for fragment in _passthrough_of(prim):
        _append_fragment(element, fragment)


def _shapes_under(pxr: Pxr, prim: Prim) -> list[Prim]:
    """The geometry a body owns: its gprims, looking through any Xform or Scope that groups
    them, and stopping at a nested body or joint, which own their own."""
    shapes = []
    for child in prim.GetChildren():
        if child.HasAPI(pxr.UsdPhysics.RigidBodyAPI) or child.IsA(pxr.UsdPhysics.Joint):
            continue
        if child.IsA(pxr.UsdGeom.Gprim):
            shapes.append(child)
        else:
            shapes.extend(_shapes_under(pxr, child))
    return shapes


def _write_geometry(pxr: Pxr, link: ET.Element, prim: Prim, owner: Prim) -> None:
    xyz, rotation, group_scale = _decompose(pxr, _pose_in(pxr, prim, owner))
    shape = _shape_of(pxr, prim, _scale_of(prim) * group_scale)
    if shape is None:
        return
    tag = "collision" if prim.HasAPI(pxr.UsdPhysics.CollisionAPI) else "visual"
    element = ET.SubElement(link, tag)
    rpy = _rpy_of(rotation * _shape_axis_rotation(prim))
    if np.any(xyz) or np.any(rpy):
        ET.SubElement(element, "origin", xyz=_triple(xyz), rpy=_triple(rpy))
    geometry = ET.SubElement(element, "geometry")
    geometry.append(shape)


# What turns a URDF cylinder, which always lies along z, onto each axis a USD shape can name.
_SHAPE_AXIS_ROTATIONS = {
    "X": Rotation.from_euler("y", np.pi / 2),
    "Y": Rotation.from_euler("x", -np.pi / 2),
    "Z": Rotation.identity(),
}


def _shape_axis(prim: Prim) -> str:
    """The axis a Cylinder or Capsule lies along; anything else is z, where a URDF has it."""
    if str(prim.GetTypeName()) not in {"Cylinder", "Capsule"}:
        return "Z"
    attribute = prim.GetAttribute("axis")
    token = str(attribute.Get()) if attribute and attribute.Get() is not None else "Z"
    if token not in _SHAPE_AXIS_ROTATIONS:
        logger.warning(f"{prim.GetPath()} lies along {token!r}, which is no axis; taking Z")
        return "Z"
    return token


def _shape_axis_rotation(prim: Prim) -> Rotation:
    return _SHAPE_AXIS_ROTATIONS[_shape_axis(prim)]


def _round_scales(prim: Prim, scale: NDArray[np.float64]) -> tuple[float, float]:
    """The scale across a round shape (its first other axis) and the scale along it."""
    along = "XYZ".index(_shape_axis(prim))
    across = next(index for index in range(3) if index != along)
    return float(scale[across]), float(scale[along])


def _shape_of(pxr: Pxr, prim: Prim, scale: NDArray[np.float64]) -> ET.Element | None:
    type_name = str(prim.GetTypeName())
    if type_name == "Cube":
        size = prim.GetAttribute("size").Get() if prim.GetAttribute("size") else 1.0
        extents = scale * float(size if size is not None else 1.0)
        return ET.Element("box", size=_triple(extents))
    if type_name == "Sphere":
        radius = _float_attr(prim, "radius", 1.0)
        return ET.Element("sphere", radius=_number(radius * float(scale[0])))
    if type_name == "Cylinder":
        radius = _float_attr(prim, "radius", 1.0)
        height = _float_attr(prim, "height", 1.0)
        across, along = _round_scales(prim, scale)
        return ET.Element(
            "cylinder", radius=_number(radius * across), length=_number(height * along)
        )
    if type_name == "Capsule":
        radius = _float_attr(prim, "radius", 1.0)
        height = _float_attr(prim, "height", 1.0)
        across, along = _round_scales(prim, scale)
        return ET.Element(
            "cylinder",
            radius=_number(radius * across),
            length=_number((height + 2.0 * radius) * along),
        )
    if type_name == "Mesh":
        filename = _referenced_path(prim)
        if filename is None:
            logger.warning(f"skipping mesh {prim.GetPath()}: it references no file")
            return None
        return ET.Element("mesh", filename=filename, scale=_triple(scale))
    logger.warning(f"skipping geometry {prim.GetPath()} of type {type_name!r}")
    return None


def _referenced_path(prim: Prim) -> str | None:
    identifier = (prim.GetAssetInfo() or {}).get("identifier")
    if identifier is not None and str(identifier.path):
        return str(identifier.path)
    for spec in prim.GetPrimStack():
        items = spec.referenceList.explicitItems or spec.referenceList.prependedItems
        for reference in items:
            path = str(reference.assetPath)
            if path:
                return path
    return None


_AXIS_TOKENS = {
    "X": (1.0, 0.0, 0.0),
    "Y": (0.0, 1.0, 0.0),
    "Z": (0.0, 0.0, 1.0),
}


def _axis_of(prim: Prim) -> NDArray[np.float64]:
    """The axis a single-DOF joint turns about, in its own local frame.

    UsdPhysics states it as a token and lets the local rotations carry anything that is not a
    principal axis. ``urdf_to_usd`` writes "Z" and puts the direction in ``localRot1``, so
    reading only that frame looked right; a layer from MuJoCo's converter writes the token
    ("Y" for a knee) with identity rotations, and reading only the frame then turned every
    joint about z -- a waist that pitches drawn as a waist that twists.
    """
    attribute = prim.GetAttribute("physics:axis")
    if not attribute or not attribute.IsDefined():
        # A multi-DOF joint has no axis attribute at all, and for one of those the direction
        # is whatever localRot1 does to z -- which is how urdf_to_usd writes a planar normal.
        return np.asarray(_AXIS_TOKENS["Z"], dtype=np.float64)
    token = str(attribute.Get())
    if token not in _AXIS_TOKENS:
        logger.warning(f"{prim.GetPath()} turns about {token!r}, which is no axis; taking X")
    return np.asarray(_AXIS_TOKENS.get(token, _AXIS_TOKENS["X"]), dtype=np.float64)


def _pose_in(pxr: Pxr, prim: Prim, owner: Prim | None) -> Any:
    """Where a prim sits in `owner` (an ancestor), through every grouping prim between them.

    The prim's own placement drops its scale (see _placement_matrix); a group's does not,
    since a scaled group moves what it holds. Row-vector Gf matrices, so child-first.
    """
    pose = _placement_matrix(pxr, prim)
    parent = prim.GetParent()
    while parent and parent != owner and parent.GetPath() != pxr.Sdf.Path.absoluteRootPath:
        xformable = pxr.UsdGeom.Xformable(parent)
        if xformable:
            pose = pose * xformable.GetLocalTransformation(pxr.Usd.TimeCode.Default())
        parent = parent.GetParent()
    return pose


def _decompose(pxr: Pxr, matrix: Any) -> tuple[NDArray[np.float64], Rotation, NDArray[np.float64]]:
    placement = pxr.Gf.Transform(matrix)
    translation = _vector_of(placement.GetTranslation())
    rotation = _quat_rotation(placement.GetRotation().GetQuat())
    return translation, rotation, _vector_of(placement.GetScale())


def _rpy_of(rotation: Rotation) -> NDArray[np.float64]:
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Gimbal lock detected")
        return np.asarray(rotation.as_euler("xyz"), dtype=np.float64)


def _placement_matrix(pxr: Pxr, prim: Prim) -> Any:
    """Where a prim sits in its parent, however the layer spells it.

    One layer says it as a single matrix and the next as an ordered translate/orient/rotate
    stack -- the vendored robots use the latter throughout -- so compose the ops rather than
    insisting on one attribute spelling. The scale op is skipped, not divided back out: the
    shapes carry scale in their own sizes via _scale_of, and a scale that sits before the
    offset in xformOpOrder would otherwise multiply the offset too.
    """
    composed = pxr.Gf.Matrix4d(1.0)
    xformable = pxr.UsdGeom.Xformable(prim)
    if not xformable:
        return composed
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == pxr.UsdGeom.XformOp.TypeScale:
            continue
        composed = op.GetOpTransform(pxr.Usd.TimeCode.Default()) * composed
    return composed


def _scale_of(prim: Prim) -> NDArray[np.float64]:
    attribute = prim.GetAttribute("xformOp:scale")
    if not attribute or attribute.Get() is None:
        return np.ones(3)
    value = attribute.Get()
    return np.asarray([value[0], value[1], value[2]], dtype=np.float64)


def _float_attr(prim: Prim, name: str, default: float) -> float:
    attribute = prim.GetAttribute(name)
    if not attribute or attribute.Get() is None:
        return default
    return float(attribute.Get())


def _write_joint(root: ET.Element, joint: _Joint) -> None:
    element = ET.SubElement(root, "joint", name=joint.name, type=joint.kind)
    ET.SubElement(element, "parent", link=joint.parent)
    ET.SubElement(element, "child", link=joint.child)
    ET.SubElement(element, "origin", xyz=_triple(joint.origin_xyz), rpy=_triple(joint.origin_rpy))
    if joint.kind != "fixed":
        ET.SubElement(element, "axis", xyz=_triple(joint.axis))
    if joint.kind not in {"fixed", "continuous"} and joint.lower is not None:
        assert joint.upper is not None
        ET.SubElement(
            element,
            "limit",
            lower=_number(joint.lower),
            upper=_number(joint.upper),
            effort="0",
            velocity="0",
        )
    for fragment in joint.extras:
        _append_fragment(element, fragment)


def _append_fragment(parent: ET.Element, fragment: str) -> None:
    try:
        parent.append(ET.fromstring(fragment))
    except ET.ParseError:
        logger.warning(f"dropping an unconverted fragment that will not parse: {fragment[:60]}")


def _welds_of(
    pxr: Pxr, links: dict[str, Any], joints: list[_Joint], taken: set[str]
) -> list[_Joint]:
    claimed = {joint.child for joint in joints}
    welds = []
    for name, prim in links.items():
        if name in claimed or prim is None:
            continue
        ancestor = _link_ancestor(pxr, prim)
        if ancestor is None:
            continue
        xyz, rotation, _ = _decompose(pxr, _pose_in(pxr, prim, ancestor))
        rpy = _rpy_of(rotation)
        parent = frame_path(ancestor)
        joint_name = _unused_name(f"{prim.GetName()}_weld", taken)
        kind = "fixed"
        logger.debug(f"{name} is nested with no joint prim; reading it as welded to {parent}")
        welds.append(
            _Joint(
                name=joint_name,
                kind=kind,
                parent=parent,
                child=name,
                origin_xyz=xyz,
                origin_rpy=rpy,
                axis=np.array([0.0, 0.0, 1.0]),
                lower=None,
                upper=None,
                extras=[],
            )
        )
    return welds


def _warn_about_orphans(links: dict[str, Any], joints: list[_Joint]) -> None:
    named = {joint.parent for joint in joints} | {joint.child for joint in joints}
    missing = sorted(named - set(links))
    if missing:
        logger.warning(f"joints name links this layer has no prim for: {missing}")


def _triple(values: NDArray[np.float64] | Any) -> str:
    return " ".join(_number(float(value)) for value in values)


def _number(value: float) -> str:
    text = f"{value:.12g}"
    return "0" if text in {"-0", "-0.0"} else text
