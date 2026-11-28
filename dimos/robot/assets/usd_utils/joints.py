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

"""A USD's kinematics read straight off its UsdPhysics joints: each joint, and the welds its nested bodies imply."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any
import warnings
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.robot.assets.model import JointDescription
from dimos.robot.assets.usd_utils.from_urdf import OUT_OF_PLANE, PASSTHROUGH_KEY
from dimos.robot.assets.usd_utils.pxr import Prim, Pxr, Stage, frame_path, layer_from, load_pxr
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()

_JOINT_TYPES = {
    "PhysicsRevoluteJoint": "revolute",
    "PhysicsPrismaticJoint": "prismatic",
    "PhysicsFixedJoint": "fixed",
    "PhysicsJoint": "floating",
}

_AXIS_TOKENS = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}


def usd_joints(usd: str, root: Path | str | None = None) -> tuple[JointDescription, ...]:
    """Every joint of the layer, then a fixed joint for each nested body no joint holds."""
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer", root))
    links = _links_of(pxr, stage)
    default_prim = stage.GetDefaultPrim()
    world = frame_path(default_prim) if default_prim else None
    joint_prims = _joint_prims(pxr, stage)
    # Every name the layer already uses: a frame or joint this reader adds must not take one.
    taken = set(links) | {str(prim.GetDisplayName() or prim.GetName()) for prim in joint_prims}
    joints = [joint for prim in joint_prims for joint in _joints_of(pxr, prim, taken, world)]
    return (*joints, *_welds_of(pxr, links, joints, taken))


def usd_root_link(usd: str, root: Path | str | None = None) -> str:
    """The body no joint or enclosing body holds; with none, the one body every joint hangs off."""
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer", root))
    links = _links_of(pxr, stage)
    joints = _joint_prims(pxr, stage)
    held = {_body_name(prim, "physics:body1") for prim in joints}
    held |= {name for name, prim in links.items() if _link_ancestor(pxr, prim) is not None}
    roots = [name for name in links if name not in held]
    if not roots:
        parents = {name for prim in joints if (name := _body_name(prim, "physics:body0"))}
        return next(iter(parents)) if len(parents) == 1 else next(iter(links), "")
    if len(roots) > 1:
        logger.warning(f"this layer has more than one root link ({roots}); taking the first")
    return roots[0]


def _links_of(pxr: Pxr, stage: Stage) -> dict[str, Prim]:
    return {
        frame_path(prim): prim
        for prim in stage.Traverse()
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI)
    }


def _joint_prims(pxr: Pxr, stage: Stage) -> list[Prim]:
    return [prim for prim in stage.Traverse() if prim.IsA(pxr.UsdPhysics.Joint)]


def _link_ancestor(pxr: Pxr, prim: Prim) -> Prim | None:
    parent = prim.GetParent()
    while parent and parent.GetPath() != pxr.Sdf.Path.absoluteRootPath:
        if parent.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            return parent
        parent = parent.GetParent()
    return None


def _unused_name(base: str, taken: set[str]) -> str:
    """``base``, or ``base_2``, ``base_3``... if the layer already uses it; claimed in ``taken``."""
    name, index = base, 1
    while name in taken:
        index += 1
        name = f"{base}_{index}"
    taken.add(name)
    return name


def _joints_of(pxr: Pxr, prim: Prim, taken: set[str], world: str | None) -> list[JointDescription]:
    """One USD joint as joints whose frame is the child's: itself, or itself and a weld off its anchor.

    UsdPhysics puts the joint at localPose0 in body0 and at localPose1 in body1, so at rest the child sits at
    parent_from_joint * inverse(child_from_joint). A moving joint anchored off the child's origin turns about the
    anchor, so it ends at a frame on the anchor (oriented like the child) and the child is welded off that.
    """
    joint = pxr.UsdPhysics.Joint(prim)
    named = _body_name(prim, "physics:body0")
    parent = named or world
    child = _body_name(prim, "physics:body1")
    if parent is None or child is None:
        logger.warning(f"skipping joint {prim.GetPath()}: it does not name both bodies")
        return []

    kind = _JOINT_TYPES.get(str(prim.GetTypeName()), "fixed")
    if kind == "floating" and all(
        f"PhysicsLimitAPI:{axis}" in {str(name) for name in prim.GetAppliedSchemas()}
        for axis in OUT_OF_PLANE
    ):
        kind = "planar"
    lower, upper = _limits_of(prim, kind)
    if kind == "revolute" and lower is None:
        kind = "continuous"

    rot0 = _rotation_of(joint.GetLocalRot0Attr().Get())
    rot1 = _rotation_of(joint.GetLocalRot1Attr().Get())
    pos0 = _vector_of(joint.GetLocalPos0Attr().Get())
    pos1 = _vector_of(joint.GetLocalPos1Attr().Get())
    child_rotation = rot0 * rot1.inv()
    child_xyz = pos0 - child_rotation.apply(pos1)
    if named is None:
        # Welded to the world: the child's own transform is where it sits.
        child_xyz, child_rotation = _decompose(
            pxr, _pose_in(pxr, _body_prim(prim, "physics:body1"), prim.GetStage().GetDefaultPrim())
        )

    name = str(prim.GetDisplayName() or prim.GetName())
    anchored = kind != "fixed" and bool(np.any(pos1))
    moving = JointDescription(
        name=name,
        type=kind,
        parent_link=parent,
        child_link=_unused_name(f"{name}_anchor", taken) if anchored else child,
        origin_xyz=_triple(child_xyz + child_rotation.apply(pos1) if anchored else child_xyz),
        origin_rpy=_rpy_of(child_rotation),
        lower=lower,
        upper=upper,
        axis=_triple(rot1.apply(_axis_of(prim))),
        mimic=_mimic_of(prim),
    )
    if not anchored:
        return [moving]
    weld = JointDescription(
        name=_unused_name(f"{name}_anchor_weld", taken),
        type="fixed",
        parent_link=moving.child_link,
        child_link=child,
        origin_xyz=_triple(-pos1),
    )
    return [moving, weld]


def _welds_of(
    pxr: Pxr, links: dict[str, Prim], joints: list[JointDescription], taken: set[str]
) -> list[JointDescription]:
    """A fixed joint for each body nested in another that no joint holds, where its prim puts it."""
    held = {joint.child_link for joint in joints}
    welds = []
    for name, prim in links.items():
        ancestor = _link_ancestor(pxr, prim)
        if name in held or ancestor is None:
            continue
        xyz, rotation = _decompose(pxr, _pose_in(pxr, prim, ancestor))
        welds.append(
            JointDescription(
                name=_unused_name(f"{prim.GetName()}_weld", taken),
                type="fixed",
                parent_link=frame_path(ancestor),
                child_link=name,
                origin_xyz=_triple(xyz),
                origin_rpy=_rpy_of(rotation),
            )
        )
    return welds


def _body_prim(prim: Prim, relationship: str) -> Prim:
    return prim.GetStage().GetPrimAtPath(prim.GetRelationship(relationship).GetTargets()[0])


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
    lower = prim.GetAttribute("physics:lowerLimit")
    upper = prim.GetAttribute("physics:upperLimit")
    if not (lower and upper and lower.HasAuthoredValue() and upper.HasAuthoredValue()):
        return None, None
    if lower.Get() is None or upper.Get() is None:
        return None, None
    # UsdPhysics states a revolute joint's limits in degrees.
    scale = np.pi / 180.0 if kind == "revolute" else 1.0
    return float(lower.Get()) * scale, float(upper.Get()) * scale


def _axis_of(prim: Prim) -> NDArray[np.float64]:
    """The axis a joint turns about in its own frame: its axis token, or z for a multi-DOF joint (whose normal localRot1 turns)."""
    attribute = prim.GetAttribute("physics:axis")
    if not attribute or not attribute.IsDefined():
        return np.asarray(_AXIS_TOKENS["Z"])
    token = str(attribute.Get())
    if token not in _AXIS_TOKENS:
        logger.warning(f"{prim.GetPath()} turns about {token!r}, which is no axis; taking X")
    return np.asarray(_AXIS_TOKENS.get(token, _AXIS_TOKENS["X"]))


def _mimic_of(prim: Prim) -> tuple[str, float, float] | None:
    """The URDF mimic a joint carries as an unconverted fragment: USD cannot say one joint follows another."""
    fragments = ((prim.GetCustomData() or {}).get("urdf") or {}).get(
        PASSTHROUGH_KEY.split(":", 1)[1], ()
    )
    for fragment in fragments:
        element = ET.fromstring(str(fragment))
        if element.tag == "mimic":
            return (
                element.attrib["joint"],
                float(element.get("multiplier", 1.0)),
                float(element.get("offset", 0.0)),
            )
    return None


def _pose_in(pxr: Pxr, prim: Prim, owner: Prim | None) -> Any:
    """Where a prim sits in ``owner`` (an ancestor), through every grouping prim between them; its own scale left out."""
    pose = pxr.Gf.Matrix4d(1.0)
    xformable = pxr.UsdGeom.Xformable(prim)
    if xformable:
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() != pxr.UsdGeom.XformOp.TypeScale:
                pose = op.GetOpTransform(pxr.Usd.TimeCode.Default()) * pose
    parent = prim.GetParent()
    while parent and parent != owner and parent.GetPath() != pxr.Sdf.Path.absoluteRootPath:
        group = pxr.UsdGeom.Xformable(parent)
        if group:
            pose = pose * group.GetLocalTransformation(pxr.Usd.TimeCode.Default())
        parent = parent.GetParent()
    return pose


def _decompose(pxr: Pxr, matrix: Any) -> tuple[NDArray[np.float64], Rotation]:
    placement = pxr.Gf.Transform(matrix)
    return _vector_of(placement.GetTranslation()), _rotation_of(placement.GetRotation().GetQuat())


def _rotation_of(quat: Any) -> Rotation:
    if quat is None:
        return Rotation.identity()
    imaginary = quat.GetImaginary()
    return Rotation.from_quat([imaginary[0], imaginary[1], imaginary[2], quat.GetReal()])


def _vector_of(value: Any) -> NDArray[np.float64]:
    if value is None:
        return np.zeros(3)
    return np.asarray([value[0], value[1], value[2]], dtype=np.float64)


def _rpy_of(rotation: Rotation) -> tuple[float, float, float]:
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Gimbal lock detected")
        return _triple(rotation.as_euler("xyz"))


def _triple(values: Any) -> tuple[float, float, float]:
    return float(values[0]), float(values[1]), float(values[2])
