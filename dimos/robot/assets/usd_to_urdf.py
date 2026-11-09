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

"""A USD layer back as URDF, for the things that only read URDF.

Only reason it exists: RoboPlan builds its collision scene from URDF XML and there is no USD
equivalent, so a USD-described robot could not filter itself out of its own point cloud.

The inverse of ``urdf_to_usd`` and shares its conventions: a link is a prim with
``PhysicsRigidBodyAPI``, a frame's name is its ``displayName``, a joint is authored about z
with the URDF axis in the local rotations, a revolute limit is degrees in USD and radians in
URDF, and what could not be expressed sits in ``customData["urdf"]["unconverted"]``. Joint
placement goes through single precision (localPos is point3f, localRot quatf). Not for
arbitrary USD: another tool's layer states its physics differently.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any
import warnings
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.robot.assets.urdf_to_usd import _OUT_OF_PLANE, PASSTHROUGH_KEY
from dimos.robot.assets.usd import Prim, Pxr, Stage, layer_from, load_pxr
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()

# A revolute limit is degrees in USD and radians in URDF (see urdf_to_usd._set_joint_limits).
_DEGREES = 180.0 / np.pi

# UsdPhysics joint prim type -> URDF joint type. A revolute joint with no limit authored is
# continuous, which is decided per joint below rather than by type.
_JOINT_TYPES = {
    "PhysicsRevoluteJoint": "revolute",
    "PhysicsPrismaticJoint": "prismatic",
    "PhysicsFixedJoint": "fixed",
    # The generic joint: free in all six degrees unless something is locked, which is a URDF
    # floating joint. Locking the axes out of one plane makes it planar -- see _free_kind.
    "PhysicsJoint": "floating",
}


def usd_to_urdf(usd: str, *, name: str | None = None) -> str:
    """``usd`` as URDF XML. ``name`` overrides the robot name, which otherwise comes from the
    layer's default prim.
    """
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a layer")
    stage = pxr.Usd.Stage.Open(layer)

    links = _usd_links_of(pxr, stage)
    joints = [
        joint
        for joint in (_joint_of(pxr, prim) for prim in _joint_prims(pxr, stage))
        if joint is not None
    ]

    default_prim = stage.GetDefaultPrim()
    robot_name = name or (default_prim.GetName() if default_prim else "robot")
    root = ET.Element("robot", name=robot_name)
    # Decided once for the whole layer: a model that marks collision properly is never
    # second-guessed, and one that marks none at all falls back to MuJoCo's grouping.
    by_group = not _marks_collision(pxr, stage)

    for link_name, prim in links.items():
        _write_link(pxr, root, link_name, prim, by_group=by_group)
    # USD conveys parent and child by nesting; a UsdPhysics joint only adds how it moves. URDF has
    # to be told outright that a nested link with no joint is welded, or the tree comes out
    # disconnected and will not load.
    joints = joints + _welds_of(pxr, links, joints)
    for joint in joints:
        _write_joint(root, joint)

    _warn_about_orphans(links, joints)
    return ET.tostring(root, encoding="unicode")


def usd_root_link(usd: str) -> str:
    """The link everything else hangs under. The layer's defaultPrim is the robot, not a link, so
    the root is found the way a URDF defines it: the one link no joint claims and nothing contains.
    """
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer"))
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
    """The prim every joint hangs off, when it is not itself a link.

    A scene converted from an MJCF welds each item to the worldbody, which is not a rigid body, so
    `roots` comes out empty and the fallback rooted office_555's 448 pieces of furniture at a
    cardboard box. Marking that prim as a rigid body instead is worse: MuJoCo then treats the
    world as dynamic and refuses the ground plane ("plane only allowed in static bodies").
    """
    parents = {name for prim in joints if (name := _body_name(prim, "physics:body0"))}
    assert pxr and stage  # the stage these joints came from
    return next(iter(parents)) if len(parents) == 1 else ""


def _nested_link_names(pxr: Pxr, links: dict[str, Any]) -> set[str]:
    """Every link that sits inside another link, which is a welded child."""
    nested = set()
    for name, prim in links.items():
        if _link_ancestor(pxr, prim) is not None:
            nested.add(name)
    return nested


def _link_ancestor(pxr: Pxr, prim: Prim) -> Any | None:
    """The nearest enclosing link prim, or None at the top of the tree."""
    parent = prim.GetParent()
    while parent and parent.GetPath() != pxr.Sdf.Path.absoluteRootPath:
        if parent.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            return parent
        parent = parent.GetParent()
    return None


def _usd_links_of(pxr: Pxr, stage: Stage) -> dict[str, Any]:
    """Every link prim, by the name it travels under: a prim with ``PhysicsRigidBodyAPI``, the same
    test graft_usd and the renamers use, so these names are the frame_ids the rest of dimos sees.
    """
    links: dict[str, Any] = {}
    for prim in stage.Traverse():
        if not prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            continue
        link_name = prim.GetDisplayName() or prim.GetName()
        if link_name in links:
            logger.warning(f"two prims claim the link name {link_name!r}; keeping the first")
            continue
        links[link_name] = prim
    return links


def _joint_prims(pxr: Pxr, stage: Stage) -> list[Any]:
    return [prim for prim in stage.Traverse() if prim.IsA(pxr.UsdPhysics.Joint)]


class _Joint:
    """One joint's URDF shape, recovered from its USD prim."""

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


def _joint_of(pxr: Pxr, prim: Prim) -> _Joint | None:
    """Undo urdf_to_usd's _set_joint_frames and _set_joint_limits for one joint prim."""
    joint = pxr.UsdPhysics.Joint(prim)
    parent = _body_name(prim, "physics:body0")
    child = _body_name(prim, "physics:body1")
    if parent is None or child is None:
        logger.warning(f"skipping joint {prim.GetPath()}: it does not name both bodies")
        return None

    kind = _JOINT_TYPES.get(str(prim.GetTypeName()), "fixed")
    if kind == "floating":
        kind = _free_kind(prim)
    lower, upper = _limits_of(prim, kind)
    # A revolute joint carrying no limit at all is what a continuous joint was written as.
    if kind == "revolute" and lower is None and upper is None:
        kind = "continuous"

    # localRot1 is the rotation that took +z onto the URDF axis, and localRot0 is that with
    # the joint's own origin rotation in front of it.
    rot0 = _quat_rotation(joint.GetLocalRot0Attr().Get())
    rot1 = _quat_rotation(joint.GetLocalRot1Attr().Get())
    axis = rot1.apply(np.array([0.0, 0.0, 1.0]))
    origin_rotation = rot0 * rot1.inv()

    position = joint.GetLocalPos0Attr().Get()
    return _Joint(
        name=prim.GetDisplayName() or prim.GetName(),
        kind=kind,
        parent=parent,
        child=child,
        origin_xyz=np.asarray([position[0], position[1], position[2]], dtype=np.float64),
        origin_rpy=np.asarray(origin_rotation.as_euler("xyz"), dtype=np.float64),
        axis=axis,
        lower=lower,
        upper=upper,
        extras=_passthrough_of(prim),
    )


def _free_kind(prim: Prim) -> str:
    """floating, or planar when the axes out of one plane are locked. Read off the applied schemas
    rather than metadata of ours, so a layer written by anything else says the same thing.
    """
    applied = {str(name) for name in prim.GetAppliedSchemas()}
    if all(f"PhysicsLimitAPI:{axis}" in applied for axis in _OUT_OF_PLANE):
        return "planar"
    return "floating"


def _body_name(prim: Prim, relationship: str) -> str | None:
    """The link name a joint's body relationship points at."""
    targets = prim.GetRelationship(relationship).GetTargets()
    if not targets:
        return None
    body = prim.GetStage().GetPrimAtPath(targets[0])
    if not body:
        logger.warning(f"{prim.GetPath()} {relationship} points at {targets[0]}, which is absent")
        return None
    return str(body.GetDisplayName() or body.GetName())


def _limits_of(prim: Prim, kind: str) -> tuple[float | None, float | None]:
    """The limits in URDF units, or (None, None) when the joint carries none."""
    lower_attr = prim.GetAttribute("physics:lowerLimit")
    upper_attr = prim.GetAttribute("physics:upperLimit")
    # HasAuthoredValue, not truthiness: the UsdPhysics joint schemas define these attributes
    # whether or not a layer wrote them. A continuous joint is a revolute one with nothing
    # authored, so the difference is the point.
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
    """A Gf.Quat as a scipy Rotation. Gf is (real, imaginary); scipy is (x, y, z, w)."""
    if quat is None:
        return Rotation.identity()
    imaginary = quat.GetImaginary()
    return Rotation.from_quat([imaginary[0], imaginary[1], imaginary[2], quat.GetReal()])


def _passthrough_of(prim: Prim) -> list[str]:
    """The XML fragments the conversion to USD could not express, verbatim."""
    data = prim.GetCustomData() or {}
    urdf = data.get("urdf") or {}
    return [str(fragment) for fragment in urdf.get(PASSTHROUGH_KEY.split(":", 1)[1], ())]


def _write_link(pxr: Pxr, root: ET.Element, link_name: str, prim: Prim, *, by_group: bool) -> None:
    element = ET.SubElement(root, "link", name=link_name)
    for child in prim.GetChildren():
        if child.HasAPI(pxr.UsdPhysics.RigidBodyAPI) or child.IsA(pxr.UsdPhysics.Joint):
            continue  # a nested link or a joint, not this link's own geometry
        _write_geometry(pxr, element, child, by_group=by_group)
    for fragment in _passthrough_of(prim):
        _append_fragment(element, fragment)


# MuJoCo's own convention in every menagerie model: geoms in group 3 are the collision shapes.
# Only consulted for a layer that marks no collision at all -- see _collides.
_MJC_COLLISION_GROUP = 3


def _marks_collision(pxr: Pxr, stage: Stage) -> bool:
    return any(prim.HasAPI(pxr.UsdPhysics.CollisionAPI) for prim in stage.Traverse())


def _collides(pxr: Pxr, prim: Prim, *, by_group: bool) -> bool:
    """Whether this geometry prim is a collision volume rather than something to look at.

    CollisionAPI where the layer uses it. A model converted from an MJCF that switches collision
    off wholesale (`contype="0" conaffinity="0"`, which is how the G1 is set up) carries none at
    all, so MuJoCo's own `mjc:group` is the fallback -- only a fallback, because the group number
    is per-model (the M20's vendor MJCF collides on group 1).
    """
    if prim.HasAPI(pxr.UsdPhysics.CollisionAPI):
        return True
    if not by_group:
        return False
    group = prim.GetAttribute("mjc:group")
    return bool(group) and group.Get() == _MJC_COLLISION_GROUP


def _write_geometry(pxr: Pxr, link: ET.Element, prim: Prim, *, by_group: bool) -> None:
    shape = _shape_of(pxr, prim)
    if shape is None:
        return
    # Collision geometry is the half RoboPlan needs.
    tag = "collision" if _collides(pxr, prim, by_group=by_group) else "visual"
    element = ET.SubElement(link, tag)
    xyz, rpy = _placement_of(prim)
    if np.any(xyz) or np.any(rpy):
        ET.SubElement(element, "origin", xyz=_triple(xyz), rpy=_triple(rpy))
    geometry = ET.SubElement(element, "geometry")
    geometry.append(shape)


def _shape_of(pxr: Pxr, prim: Prim) -> ET.Element | None:
    """The <box>/<cylinder>/<sphere>/<mesh> element for a geometry prim, or None."""
    type_name = str(prim.GetTypeName())
    scale = _scale_of(prim)
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
        return ET.Element(
            "cylinder",
            radius=_number(radius * float(scale[0])),
            length=_number(height * float(scale[2])),
        )
    if type_name == "Capsule":
        # URDF has no capsule. A cylinder of the same radius and length is the usual stand-in: it
        # under-covers only the two hemispherical caps, where dropping it would give a G1 with no
        # collision volume on its thighs, shins or hands at all.
        radius = _float_attr(prim, "radius", 1.0)
        height = _float_attr(prim, "height", 1.0)
        return ET.Element(
            "cylinder",
            radius=_number(radius * float(scale[0])),
            length=_number((height + 2.0 * radius) * float(scale[2])),
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
    """The asset path a mesh prim names, which is the filename the URDF named.

    assetInfo first, because that is where a mesh written by urdf_to_usd names its file: a
    reference arc composes another USD layer and an STL is not one. A reference is still read
    after it, for a mesh that genuinely does reference a layer.
    """
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


def _placement_of(prim: Prim) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """A geometry prim's <origin>, out of the transform op urdf_to_usd wrote."""
    transform = prim.GetAttribute("xformOp:transform")
    if not transform or transform.Get() is None:
        return np.zeros(3), np.zeros(3)
    # Gf matrices are row-vector convention, so the transpose is the usual 4x4.
    matrix = np.asarray(transform.Get(), dtype=np.float64).T
    rotation = Rotation.from_matrix(matrix[:3, :3])
    # A URDF states rotation as intrinsic rpy, which is not unique at gimbal lock: scipy warns and
    # picks a representative. Any of them rebuild the same matrix, so the warning is noise.
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Gimbal lock detected")
        rpy = np.asarray(rotation.as_euler("xyz"), dtype=np.float64)
    return matrix[:3, 3], rpy


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
        # effort and velocity are required by the URDF schema but say nothing about collision.
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


def _welds_of(pxr: Pxr, links: dict[str, Any], joints: list[_Joint]) -> list[_Joint]:
    """A joint for every nested link no joint prim already accounts for.

    USD conveys parent and child by nesting and a joint prim only adds how the child moves, so a
    link nested inside another with no joint is welded. Every joint this converter writes is a
    prim, so this only fires for a layer written elsewhere that states nothing but the hierarchy.
    """
    claimed = {joint.child for joint in joints}
    welds = []
    for name, prim in links.items():
        if name in claimed:
            continue
        ancestor = _link_ancestor(pxr, prim)
        if ancestor is None:
            continue
        xyz, rpy = _placement_of(prim)
        parent = str(ancestor.GetDisplayName() or ancestor.GetName())
        joint_name = f"{name}_weld"
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
    """A joint naming a link the layer does not carry would make an unloadable URDF."""
    named = {joint.parent for joint in joints} | {joint.child for joint in joints}
    missing = sorted(named - set(links))
    if missing:
        logger.warning(f"joints name links this layer has no prim for: {missing}")


def _triple(values: NDArray[np.float64] | Any) -> str:
    return " ".join(_number(float(value)) for value in values)


def _number(value: float) -> str:
    """Short enough to read, exact enough that a round trip does not drift."""
    text = f"{value:.12g}"
    return "0" if text in {"-0", "-0.0"} else text
