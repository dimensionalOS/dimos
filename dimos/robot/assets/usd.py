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

A converted stage is a string (a ``.usda`` layer), never a file: a description already travels
as one through ``UsdManager.file_info``.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import hashlib
import itertools
from pathlib import Path
from threading import Lock
from typing import TYPE_CHECKING, Any
import uuid
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.robot.assets.model import JointDescription, LoadedRobotModel

Vector = tuple[float, float, float]

logger = setup_logger()

# No single position, and no UsdPhysics equivalent: their pose comes from odometry.
_MULTI_DOF_JOINT_TYPES = frozenset({"floating", "planar"})

# A URDF planar joint moves in its own xy and turns about z, so the other three are locked.
_OUT_OF_PLANE = ("transZ", "rotX", "rotY")

# The algorithm is inside the value, so it can change without the old ones becoming unreadable.
HASH_ALGORITHM = "sha256"

# Asset paths a scene leaves with are not paths on any disk: they are names the manager that
# assembled the scene answers for, under a root nothing can collide with.
ASSET_ROOT_PREFIX = "/dimos/assets/"

# customData holding the URDF this converter does not read. Standard field, not an attribute.
PASSTHROUGH_KEY = "urdf:unconverted"


_NO_USD = (
    "Reading or writing USD needs usd-core (pxr), which is in the 'sim' and 'scene' extras: "
    "uv sync --extra sim. On linux-aarch64 a wheel exists only from usd-core 26.11 and only for "
    "Python 3.11 and up, so a 3.10 arm robot cannot serve USD at all."
)


# The import, done once, behind a lock. A manager assembling a scene hashes every mounted
# subtree in parallel, so three robots is three threads reaching this at the same instant; a
# concurrent first `from pxr import ...` wedges on CPython's per-module import lock and never
# comes back. The module then answers nothing -- not its file requests, not even its rpcs --
# while looking perfectly healthy, because nothing raised.
_PXR_LOCK = Lock()
_PXR: _Pxr | None = None


def _pxr() -> _Pxr:
    """The pxr modules, imported once on first use.

    Deferred because usd-core is an optional extra, and on linux-aarch64 it is installable
    only on Python 3.11 and up: a robot must come up and pose itself on tf without it, and
    only asking for a USD layer needs it.
    """
    global _PXR
    if _PXR is not None:
        return _PXR
    with _PXR_LOCK:
        if _PXR is None:
            _let_mujoco_register_its_schemas_first()
            try:
                from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics, Vt
            except ImportError as exc:  # pragma: no cover - only where the extra is absent
                raise ImportError(_NO_USD) from exc
            _PXR = _Pxr(
                Gf=Gf, Sdf=Sdf, Tf=Tf, Usd=Usd, UsdGeom=UsdGeom, UsdPhysics=UsdPhysics, Vt=Vt
            )
    return _PXR


def _let_mujoco_register_its_schemas_first() -> None:
    """Import MuJoCo's USD reader before pxr, where MuJoCo is installed at all.

    Two OpenUSD runtimes end up in one process -- usd-core's, which this module uses to write a
    layer, and the one MuJoCo bundles to read one -- and whichever initialises first owns the
    plugin registry. With usd-core first, MuJoCo's own ``mjcPhysics`` schemas are never
    registered, so every ``MjcActuator`` in a layer parses as an untyped prim and is dropped.

    Nothing says so. Geometry is plain UsdGeom and UsdPhysics and loads either way, so the G1
    came up as a complete robot with **no actuators at all** and the only symptom, much later,
    was `no 'left_hip_pitch_joint' among ` -- a servo looking for a motor in a model that had
    silently lost all 29 of them.

    Importing it here is the only placement that holds, because this function is where pxr is
    first touched and a caller cannot know it is the first. It costs nothing where MuJoCo is
    absent (a robot posing itself on tf never has it) and nothing on later calls, since the
    import is cached and this runs once.
    """
    try:
        import mujoco._usd  # noqa: F401
    except ImportError:
        # No MuJoCo here, so nothing is going to read a layer back with it.
        return


@dataclass(frozen=True)
class _Pxr:
    Gf: Any
    Sdf: Any
    Tf: Any
    Usd: Any
    UsdGeom: Any
    UsdPhysics: Any
    Vt: Any


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
    """``model`` as one ``.usda`` layer; ``name`` is the defaultPrim, default ``<robot name>``.

    Links nest down the kinematic tree at the URDF zero pose, so a viewer with no physics
    engine still shows the robot assembled; joints are also written as ``UsdPhysics`` prims. A
    ``<mesh>`` becomes a reference to the path the URDF named, converted no further.
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

    link_extras, joint_extras = _urdf_extras(model.xml)

    paths: dict[str, str] = {}
    _write_link_tree(
        pxr,
        stage,
        parent_path=str(root.GetPath()),
        link=model.root_link,
        links=links,
        children=children,
        paths=paths,
        extras=link_extras,
        base_dir=Path(model.source_path).parent if model.source_path else None,
        inertials=_inertial_of(model.xml),
    )
    _write_joints(
        pxr,
        stage,
        root_path=str(root.GetPath()),
        joints=model.joints,
        paths=paths,
        extras=joint_extras,
    )
    return str(stage.GetRootLayer().ExportToString())


def graft_usd(root_usd: str, subtrees: Mapping[str, str], root: Path | str | None = None) -> str:
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
        if root_stage.GetPrimAtPath(target):
            raise ValueError(
                f"cannot attach a subtree at {frame_id!r}: {target} is taken. Two copies of one "
                f"description mount under the same prim name, because prefixing renames frames "
                f"and not prims -- so grafting both would keep only the last, silently. Give "
                f"the scene a frame per copy."
            )
        if not pxr.Sdf.CopySpec(sub_layer, source.GetPath(), root_layer, target):
            raise ValueError(f"could not copy the subtree USD for {frame_id!r} into the scene")
        # Copied specs keep their authored target paths, so a joint's body rels would still
        # point at the subtree's old root. Rebase them onto where the subtree now lives.
        _rebase_relationships(pxr, root_layer, target, str(source.GetPath()), str(target))
        _mount_joint(pxr, root_stage, attach_path, str(target), frame_id)

    return str(root_layer.ExportToString())


def usd_with_frames(
    usd: str, frames: Mapping[str, tuple[str, Vector, Vector]], root: Path | str | None = None
) -> str:
    """``usd`` with each named frame added as a link on the frame it hangs off.

    A scene needs somewhere to mount things that are not part of it -- three robots standing in
    a room need three frames, because grafting two subtrees at one frame is refused and rightly
    so -- and a converted room arrives with frames for its own furniture and none for its
    visitors.

    Each value is ``(parent frame, xyz, rpy)``. The frame becomes a prim carrying
    RigidBodyAPI, which is what makes it a link to everything that reads this layer: graft_usd
    matches a mount frame on it, and the viewer places it from tf like any other.
    """
    if not frames:
        return usd
    pxr = _pxr()
    layer = _layer_from(pxr, usd, "a scene", root)
    stage = pxr.Usd.Stage.Open(layer)
    for name, (parent, xyz, rpy) in frames.items():
        # Read afresh each time, because a frame may hang off one added a moment ago: the Go2's
        # Mid-360 sits on its front camera, which is itself a frame the rig adds. Reading the
        # layer's links once before the loop made that "the layer has no frame 'front_camera'",
        # which is true of the file and not of the stage it is being built into.
        by_link = _link_prim_paths(stage)
        parent_path = by_link.get(parent)
        if parent_path is None:
            raise ValueError(
                f"cannot add the frame {name!r}: the layer has no frame {parent!r} to hang it "
                f"off (it has {sorted(by_link)[:8]}...)"
            )
        path = pxr.Sdf.Path(parent_path).AppendChild(_identifier(pxr, name))
        if stage.GetPrimAtPath(path):
            raise ValueError(f"cannot add the frame {name!r}: {path} is taken")
        prim = pxr.UsdGeom.Xform.Define(stage, path)
        pxr.UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
        prim.GetPrim().SetDisplayName(name)
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        transform[:3, 3] = xyz
        prim.AddTransformOp().Set(_matrix(pxr, transform))
        _weld_frame(pxr, stage, parent_path, str(path), name)
    return str(layer.ExportToString())


def _weld_frame(pxr: Any, stage: Any, parent_path: str, path: str, name: str) -> None:
    """Say what holds the frame: the link it hangs off, or the world where that is not a link.

    Nesting places a frame but says nothing about how it is held, and a physics engine has to
    decide. MuJoCo's guess is the most permissive one -- a body free in all six degrees -- and a
    frame carries no mass, so the G1's lidar mount came back as "mass and inertia of moving
    bodies must be larger than mjMINVAL" and the robot would not compile at all. A weld is the
    honest statement (a mount bolted to the torso is not free to move) and it costs no degree of
    freedom: MuJoCo folds a welded massless body into its parent.

    Where the parent is *not* a link the frame is welded to the world instead, which is what a
    FixedJoint naming only body1 means. That is the case for a converted room, whose root is an
    Xform and not a rigid body: a stand frame hanging off it is a body with no mass and nothing
    holding it, and the room came back with the same mjMINVAL complaint as the lidar mount did.
    Leaving the weld out instead does not work either -- a frame is a rigid body because that is
    what makes MuJoCo give it a body at all, and a prim that is only an Xform is a mount point
    no engine can find.
    """
    parent = stage.GetPrimAtPath(pxr.Sdf.Path(parent_path))
    joint = pxr.UsdPhysics.FixedJoint.Define(
        stage, pxr.Sdf.Path(path).AppendChild(_identifier(pxr, f"{name}_joint"))
    )
    if parent and parent.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
        joint.CreateBody0Rel().SetTargets([parent.GetPath()])
    joint.CreateBody1Rel().SetTargets([pxr.Sdf.Path(path)])
    # The nesting already placed the frame, so the joint adds no offset of its own.
    joint.CreateLocalPos0Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.GetPrim().SetDisplayName(f"{name}_joint")


def usd_with_drives(
    usd: str, drives: Mapping[str, tuple[float, float, float]], root: Path | str | None = None
) -> str:
    """``usd`` with each named joint driven as a position servo, by ``(kp, kd, target)``.

    Where a robot's gains belong. They are a fact about the robot -- what holds its leg up is
    the controller it ships with -- and saying them in the layer means whatever runs the robot
    reads them from the same place, instead of each simulation being handed them separately and
    a real robot being configured somewhere else again.

    Written twice, because two kinds of reader want two different spellings of it:

    * ``UsdPhysics.DriveAPI`` on the joint (stiffness, damping, target), which is the standard
      and what Isaac reads: an ImplicitActuator takes its gains off the joint prim unless the
      config overrides them.
    * MuJoCo's own ``MjcActuator`` attributes on whichever actuator drives that joint --
      ``mjc:gainPrm[0] = kp`` and ``mjc:biasPrm = [0, -kp, -kd]`` with ``mjc:biasType =
      "affine"``, which is a position servo written out longhand. MuJoCo reads that straight out
      of the layer, so nothing has to turn a motor into a servo after loading.

    A motor's control range, where it has one, is its torque limit, so it moves to the force
    range and the control limit comes off: a servo's control signal is a position, and leaving a
    torque range on it as a control range clamps the position instead.
    """
    if not drives:
        return usd
    pxr = _pxr()
    layer = _layer_from(pxr, usd, "a robot", root)
    stage = pxr.Usd.Stage.Open(layer)
    joints = {_frame_name(prim): prim for prim in stage.Traverse() if _is_joint(pxr, prim)}
    actuators = {
        str(targets[0]): prim
        for prim in stage.Traverse()
        if prim.GetTypeName() == "MjcActuator"
        and (targets := prim.GetRelationship("mjc:target").GetTargets())
    }
    missing = sorted(set(drives) - set(joints))
    if missing:
        raise ValueError(f"no joint for {missing} in this layer; it has {sorted(joints)}")
    for name, (kp, kd, target) in drives.items():
        joint = joints[name]
        drive = pxr.UsdPhysics.DriveAPI.Apply(joint, _drive_kind(pxr, joint))
        drive.CreateStiffnessAttr(float(kp))
        drive.CreateDampingAttr(float(kd))
        drive.CreateTargetPositionAttr(float(target))
        actuator = actuators.get(str(joint.GetPath()))
        if actuator is None:
            continue
        _write_mjc_servo(pxr, actuator, kp, kd)
    return str(layer.ExportToString())


def _drive_kind(pxr: Any, joint: Any) -> str:
    """ "angular" for a hinge, "linear" for a slide: DriveAPI is applied under one of the two."""
    return "linear" if joint.IsA(pxr.UsdPhysics.PrismaticJoint) else "angular"


def _is_joint(pxr: Any, prim: Any) -> bool:
    return bool(prim.IsA(pxr.UsdPhysics.Joint))


def _write_mjc_servo(pxr: Any, actuator: Any, kp: float, kd: float) -> None:
    """MuJoCo's longhand for a position servo, on an actuator prim it will read back.

    What happens to the control range depends on what the actuator *was*. A plain motor's
    control signal is a torque, so its range is a torque limit and moves to the force range: a
    servo is commanded a position, and a torque range left in place would clamp the position
    instead. An actuator that already biases is already a servo -- the G1's layer arrives with
    kp=500 and the joint's own travel as its control range -- and moving that to the force range
    would limit each joint to a couple of newton-metres, which is not enough to hold a leg up.

    The control limit comes off either way: a servo is commanded where to go, and whatever it
    is allowed to reach is the joint's own range to enforce.
    """
    bias = actuator.GetAttribute("mjc:biasType")
    was_a_motor = not (bias and bias.HasAuthoredValue() and str(bias.Get()) != "none")
    doubles = pxr.Sdf.ValueTypeNames.DoubleArray
    actuator.CreateAttribute("mjc:gainPrm", doubles).Set([float(kp), *[0.0] * 9])
    actuator.CreateAttribute("mjc:biasPrm", doubles).Set([0.0, -float(kp), -float(kd), *[0.0] * 7])
    actuator.CreateAttribute("mjc:biasType", pxr.Sdf.ValueTypeNames.Token).Set("affine")
    limits = actuator.GetAttribute("mjc:ctrlRange:min"), actuator.GetAttribute("mjc:ctrlRange:max")
    if was_a_motor and all(limit and limit.HasAuthoredValue() for limit in limits):
        low, high = (float(limit.Get()) for limit in limits)
        if low < high:
            # MuJoCo's tri-state tokens ("auto"/"true"/"false"), not booleans.
            token = pxr.Sdf.ValueTypeNames.Token
            double = pxr.Sdf.ValueTypeNames.Double
            actuator.CreateAttribute("mjc:forceLimited", token).Set("true")
            actuator.CreateAttribute("mjc:forceRange:min", double).Set(low)
            actuator.CreateAttribute("mjc:forceRange:max", double).Set(high)
    actuator.CreateAttribute("mjc:ctrlLimited", pxr.Sdf.ValueTypeNames.Token).Set("false")


def usd_drives(usd: str, root: Path | str | None = None) -> dict[str, tuple[float, float, float]]:
    """Every driven joint in ``usd``, as ``{joint: (kp, kd, target)}``. The other half of usd_with_drives.

    Read from `UsdPhysics.DriveAPI` rather than from MuJoCo's spelling of the same thing,
    because this is the standard one and because it is the one that survives composition: a
    drive sits on the joint prim, so it is still there after a robot has been grafted into a
    room, whatever the engine then does with it.

    Which is the reason there is no keyframe here. A keyframe would be the natural place for
    "the pose this starts in", and MuJoCo reads one out of a layer -- but a keyframe is the
    whole model's and a composed world's qpos is a different vector, so a robot's own keyframe
    means nothing once it is standing in a room. A target on each joint means the same thing and
    composes.
    """
    pxr = _pxr()
    stage = pxr.Usd.Stage.Open(_layer_from(pxr, usd, "a robot", root))
    drives = {}
    for prim in stage.Traverse():
        if not prim.IsA(pxr.UsdPhysics.Joint):
            continue
        drive = pxr.UsdPhysics.DriveAPI(prim, _drive_kind(pxr, prim))
        stiffness = drive.GetStiffnessAttr()
        if not stiffness or not stiffness.HasAuthoredValue():
            continue
        drives[_frame_name(prim)] = (
            float(stiffness.Get()),
            float(drive.GetDampingAttr().Get() or 0.0),
            float(drive.GetTargetPositionAttr().Get() or 0.0),
        )
    return drives


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

    The defaultPrim moves too, the same way it does in ``usd_frame_names`` and
    ``rename_usd_frames``. A converted room is a plain Xform holding its furniture, so that root
    is a frame without being a link, and it is the one frame every copy of the layer names the
    same. Leaving it behind is what the prefix exists to prevent: two namespaced copies would
    publish "left/desk" and "right/desk" under a single shared "room", their subtrees would
    merge there, and whichever copy published its root's pose last would move both.
    """
    if not prefix:
        return usd
    pxr = _pxr()
    layer = _layer_from(pxr, usd, "a subtree")
    stage = pxr.Usd.Stage.Open(layer)
    root = stage.GetDefaultPrim()
    for prim in stage.Traverse():
        carries_a_frame = prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI) or prim.IsA(pxr.UsdPhysics.Joint)
        if carries_a_frame or (root and prim == root):
            prim.SetDisplayName(f"{prefix}/{prim.GetDisplayName() or prim.GetName()}")
    return str(layer.ExportToString())


def usd_frame_names(usd: str, root: Path | str | None = None) -> set[str]:
    """Every link and joint name ``usd`` carries, under the names they travel as.

    The defaultPrim counts, the same way it does in ``_link_prim_paths`` and for the same
    reason: a converted room is a plain Xform holding its furniture, so the frame everything
    in it hangs off is that root and not a link at all. Leaving it out means nothing can
    rename it, and a scene whose root is named something tf never publishes -- "office_555"
    where the robots stand in "world" -- cannot be placed and so is not drawn.

    Last, and only under a name no link has taken, so a real link still wins any clash.
    """
    pxr = _pxr()
    stage = pxr.Usd.Stage.Open(_layer_from(pxr, usd, "a layer", root))
    names = {
        prim.GetDisplayName() or prim.GetName()
        for prim in stage.Traverse()
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI) or prim.IsA(pxr.UsdPhysics.Joint)
    }
    root = stage.GetDefaultPrim()
    if root:
        names.add(root.GetDisplayName() or root.GetName())
    return names


def trivialize_usd(usd: str) -> str:
    """``usd`` with every link, joint and light kept, and the shapes they carry dropped.

    The same description serves a robot on real hardware and a simulated one, but only the sim
    needs the shapes: on a real robot the geometry is the world itself. So this keeps the
    skeleton -- the links, the joints between them and their poses, which is what tf and a
    mount point are -- and removes the visuals, the collision shapes and the meshes they
    reference. The frames come out identical either way, so one description can be the only
    one rather than a trivial second copy that drifts from it.

    Lights stay. A light is not a shape: it is how the scene looks, and it is the one thing a
    viewer cannot work out for itself -- told none, it has to invent some, and a room that said
    what its sky and its eighteen ceiling fittings are gets lit by a guess instead. Keeping them
    is what lets a scene hand over its lighting without handing over its geometry twice.
    """
    pxr = _pxr()
    layer = _layer_from(pxr, usd, "a layer")
    stage = pxr.Usd.Stage.Open(layer)
    # Whatever holds a frame stays, which keeps the robot prim and the Scope the joints sit in
    # without naming either: it is the prims holding no frame at all that are geometry.
    doomed = []
    stack = list(stage.GetPseudoRoot().GetChildren())
    while stack:
        prim = stack.pop()
        if _holds_a_frame(pxr, prim):
            stack.extend(prim.GetChildren())
        else:
            doomed.append(prim.GetPath())
    if not doomed:
        return usd
    for path in doomed:
        spec = layer.GetPrimAtPath(path)
        # A prim at the layer's own root has no nameParent to be removed from -- a converted
        # scene keeps its materials and its render settings up there beside the room. Those are
        # not geometry hanging off a frame, and removing one is the layer's business, not ours.
        if spec is not None and spec.nameParent is not None:
            del spec.nameParent.nameChildren[spec.name]
    return str(layer.ExportToString())


def _holds_a_frame(pxr: Any, prim: Any) -> bool:
    """True when ``prim`` or anything under it is a link, a joint, or a light."""
    if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI) or prim.IsA(pxr.UsdPhysics.Joint):
        return True
    # By type name rather than UsdLux.LightAPI, which _pxr does not carry: this is the same test
    # the viewer makes, and it covers DomeLight, DistantLight, SphereLight, DiskLight and RectLight
    # without naming each one.
    if str(prim.GetTypeName()).endswith("Light"):
        return True
    return any(_holds_a_frame(pxr, child) for child in prim.GetChildren())


def rename_usd_frames(usd: str, renames: Mapping[str, str]) -> str:
    """``usd`` with the named links and joints renamed; names it does not carry are ignored.

    A vendor description names its frames whatever it names them, which is rarely what the robot
    they are bolted to calls them. This is the hook for saying so.

    Renames are applied simultaneously, so swapping a pair of names does what it looks like
    rather than collapsing both onto one. A name this layer does not have is skipped rather
    than refused, because one mapping is applied to every subtree of a scene and most keys are
    meant for exactly one of them; whether a key matched *anywhere* is the caller's to check
    (see usd_frame_names).
    """
    if not renames:
        return usd
    pxr = _pxr()
    layer = _layer_from(pxr, usd, "a subtree")
    stage = pxr.Usd.Stage.Open(layer)
    frames = [
        prim
        for prim in stage.Traverse()
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI) or prim.IsA(pxr.UsdPhysics.Joint)
    ]
    # The defaultPrim is a frame too, the same way usd_frame_names and _link_prim_paths count
    # it: a converted room is a plain Xform holding its furniture, so the frame everything in
    # it hangs off is that root. Leaving it out here means a rename validates and then quietly
    # does nothing, which is worse than refusing it.
    root = stage.GetDefaultPrim()
    if root and root not in frames:
        frames.append(root)
    # Read every name before writing any, so a -> b alongside b -> a is a swap.
    named = [(prim, prim.GetDisplayName() or prim.GetName()) for prim in frames]
    changed = False
    for prim, name in named:
        if name in renames:
            prim.SetDisplayName(renames[name])
            changed = True
    return str(layer.ExportToString()) if changed else usd


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
        # assetInfo names the same file, so it moves with it or the two disagree.
        identifier = spec.assetInfo.get("identifier") if spec.assetInfo else None
        if identifier is not None and not Path(str(identifier.path)).is_absolute():
            # assetInfo is a proxy, not a dict: set the key on it rather than assigning a copy.
            spec.assetInfo["identifier"] = pxr.Sdf.AssetPath(str(base / str(identifier.path)))
            changed = True
        # A texture names its image in an ordinary asset attribute, which is neither a reference
        # nor assetInfo. Rebasing only those two left every texture pointing at a path relative
        # to a directory that no longer figures anywhere, so it resolved against nothing.
        for attribute, value in asset_attributes(spec):
            if not Path(str(value.path)).is_absolute():
                attribute.default = pxr.Sdf.AssetPath(str(base / str(value.path)))
                changed = True
        stack.extend(spec.nameChildren)
    return str(layer.ExportToString()) if changed else usd


def publish_usd_assets(usd: str) -> tuple[str, dict[str, str]]:
    """``usd`` with every asset path replaced by a stable public one, and the map back.

    The paths a scene is built with are absolute on whichever machine assembled it, which says
    nothing to whoever asks for the file. So before a scene leaves, every asset is renamed to a
    path under one fictional root, and the manager keeps the mapping so it can answer for them.

    A name is taken from where the asset first appears in the scene -- its prim path, which is
    the mount point it hangs from -- so it is stable across runs of the same scene rather than
    being an opaque number. The *first* appearance wins and every later reference to the same
    file reuses it: two copies of one arm are two mount points but one mesh, and the asset is
    the same asset regardless of how many times the thing carrying it was duplicated.

    Returns the rewritten layer and ``{public path: real path}``.
    """
    pxr = _pxr()
    layer = _layer_from(pxr, usd, "a scene")
    public: dict[str, str] = {}
    taken: set[str] = set()
    changed = False

    stack = [layer.pseudoRoot]
    while stack:
        spec = stack.pop(0)  # breadth-first, so "first" means nearest the root
        for real in _asset_paths_of(spec):
            if real not in public:
                public[real] = _public_asset_path(str(spec.path), real, taken)
                taken.add(public[real])
        if _rewrite_assets(pxr, spec, public):
            changed = True
        stack.extend(spec.nameChildren)

    rewritten = str(layer.ExportToString()) if changed else usd
    return rewritten, {name: real for real, name in public.items()}


def _asset_paths_of(spec: Any) -> list[str]:
    """Every asset path this prim spec names: a reference, assetInfo, or an asset attribute."""
    items = spec.referenceList.explicitItems or spec.referenceList.prependedItems
    paths = [ref.assetPath for ref in items if ref.assetPath]
    identifier = spec.assetInfo.get("identifier") if spec.assetInfo else None
    if identifier is not None and str(identifier.path):
        paths.append(str(identifier.path))
    paths += [str(value.path) for _, value in asset_attributes(spec)]
    return paths


def asset_attributes(spec: Any) -> list[tuple[Any, Any]]:
    """This prim spec's asset-valued attributes, with their paths.

    A texture is neither a reference nor a mesh: a `UsdUVTexture` names its image in
    `inputs:file`, an ordinary attribute of type `asset`. Walking only arcs and assetInfo left
    every texture unseen -- unpublished, so nothing could fetch it, and unhashed, so the check
    that exists to refuse an unidentified asset passed over it in silence.

    Anything else that names a file this way is seen for the same reason: a splat, a volume,
    whatever comes next. Seen is not hashed -- whoever authors it still has to say the hash --
    but it is refused loudly rather than shipped unidentified.
    """
    found = []
    for attribute in spec.attributes:
        value = attribute.default
        if value is not None and type(value).__name__ == "AssetPath" and str(value.path):
            found.append((attribute, value))
    return found


def _public_asset_path(prim_path: str, real: str, taken: set[str]) -> str:
    """A path under the fictional root, from where this asset first appears.

    The prim path is already unique within a scene, so the only way two assets collide is by
    being on one prim -- a link with two meshes. The suffix is for exactly that, and it counts
    rather than hashes so the names stay readable.
    """
    stem = f"{ASSET_ROOT_PREFIX}{prim_path.lstrip('/')}/{Path(real).name}"
    if stem not in taken:
        return stem
    base, dot, extension = stem.rpartition(".")
    for index in itertools.count(2):
        candidate = f"{base}_{index}{dot}{extension}" if dot else f"{stem}_{index}"
        if candidate not in taken:
            return candidate
    raise AssertionError("unreachable")


def _rewrite_assets(pxr: Any, spec: Any, public: Mapping[str, str]) -> bool:
    """Point this prim spec's asset paths at their public names."""
    changed = False
    items = spec.referenceList.explicitItems or spec.referenceList.prependedItems
    rebased = [
        pxr.Sdf.Reference(public[ref.assetPath], ref.primPath) if ref.assetPath in public else ref
        for ref in items
    ]
    if rebased and rebased != list(items):
        if spec.referenceList.explicitItems:
            spec.referenceList.explicitItems = rebased
        else:
            spec.referenceList.prependedItems = rebased
        changed = True
    identifier = spec.assetInfo.get("identifier") if spec.assetInfo else None
    if identifier is not None and str(identifier.path) in public:
        spec.assetInfo["identifier"] = pxr.Sdf.AssetPath(public[str(identifier.path)])
        changed = True
    for attribute, value in asset_attributes(spec):
        if str(value.path) in public:
            attribute.default = pxr.Sdf.AssetPath(public[str(value.path)])
            changed = True
    return changed


def _mount_joint(pxr: Any, stage: Any, mount_path: str, subtree_path: str, frame_id: str) -> None:
    """A floating joint from the mount frame to the subtree's root link.

    Nesting alone says a grafted subtree hangs off the mount frame, but says nothing about how.
    A reader is then left to guess, and the one guess it cannot make is a weld: the root of an
    articulated subtree is a frame tf already publishes, so welding it to the room as well would
    give it two parents and its pose would flip between them. The honest answer is a joint free
    in all six degrees -- "something else places this" -- which is what tf does.

    Saying it costs nothing and buys two things: a reader that has no tf yet still has somewhere
    to put the subtree (its mount), instead of a root it must either drop or place at random;
    and the root stops looking like a link nothing hangs off, which is how a scene's static
    tiles are told apart from a robot.
    """
    root_link = _first_link(pxr, stage, subtree_path)
    if root_link is None:
        return
    mount = stage.GetPrimAtPath(pxr.Sdf.Path(mount_path))
    name = _identifier(pxr, f"{frame_id}_mount")
    joint = pxr.UsdPhysics.Joint.Define(stage, f"{subtree_path}/{name}")
    joint.CreateBody0Rel().SetTargets([mount.GetPath()])
    joint.CreateBody1Rel().SetTargets([root_link.GetPath()])
    # The subtree sits where its mount is: the nesting already placed it, so the joint adds no
    # offset of its own.
    joint.CreateLocalPos0Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.GetPrim().SetDisplayName(f"{_frame_name(root_link)}_mount")


def _first_link(pxr: Any, stage: Any, path: str) -> Any:
    """The topmost prim at or below ``path`` carrying RigidBodyAPI: the subtree's root link."""
    root = stage.GetPrimAtPath(pxr.Sdf.Path(path))
    if not root:
        return None
    for prim in pxr.Usd.PrimRange(root):
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            return prim
    return None


def _frame_name(prim: Any) -> str:
    return prim.GetDisplayName() or prim.GetName()


def warm_usd() -> None:
    """Load the USD bindings, on this thread, before anything uses them from several.

    Importing `pxr` once behind a lock is not enough: pxr pulls its own submodules in lazily
    from inside C code, the first time a layer is read rather than at import. Two threads
    reaching that at the same instant wedge on CPython's per-module import lock, and the module
    they belong to stops answering anything at all without raising. Reading one tiny layer here
    walks the same path once, on one thread, so the pool that follows finds it all loaded.
    """
    pxr = _pxr()
    layer = pxr.Sdf.Layer.CreateAnonymous(".usda")
    layer.ImportFromString('#usda 1.0\n(\n    defaultPrim = "warm"\n)\n\ndef Xform "warm"\n{\n}\n')
    # The stage has to outlive the traversal: a range over prims of a stage nobody holds walks
    # into expired prims, which USD reports as an error rather than an empty range.
    stage = pxr.Usd.Stage.Open(layer)
    list(stage.Traverse())


def _layer_from(pxr: Any, usd: str, what: str, root: Path | str | None = None) -> Any:
    """``usd`` as a layer. ``root`` is the directory its relative asset paths are relative to.

    Given a root, the layer takes an identifier *in* that directory -- a name nothing is ever
    written to -- so that USD can resolve the references it carries. Without one, every mesh a
    committed robot layer names comes out as a warning per prim while the stage composes:
    "Could not open asset @meshes/base_0.usdc@ ... (instantiating stage on stage @anon:0x...@)".
    Nothing here needs the geometry, so the result is the same either way, but one start of the
    three-G1 office printed 139k lines of it and real errors went down in the noise.

    The name is unique per call because `Sdf.Layer.New` refuses an identifier another live layer
    already holds, and two frames being added to one robot is two layers off the same file.
    """
    if root is None:
        layer = pxr.Sdf.Layer.CreateAnonymous(".usda")
    else:
        name = f".dimos-in-memory-{uuid.uuid4().hex}.usda"
        layer = pxr.Sdf.Layer.New(
            pxr.Sdf.FileFormat.FindByExtension(".usda"), str(Path(root) / name)
        )
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
    """URDF link name -> prim path, for every prim carrying ``UsdPhysics.RigidBodyAPI``.

    Narrower than "any Xform": it leaves out the joints, which could shadow a link sharing a
    name. The name is displayName, or the prim name without one.

    The defaultPrim is included last and only under a name no link has taken, so a link still
    wins any clash. It has to be here because it is what a scene is rooted at: a converted room
    is a plain Xform holding 450 bodies, so without this there is nowhere to mount a robot in
    one -- and `usd_root_link` names it as the root, which would then not be attachable.
    """
    pxr = _pxr()
    paths: dict[str, str] = {}
    for prim in stage.Traverse():
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            paths[prim.GetDisplayName() or prim.GetName()] = str(prim.GetPath())
    root = stage.GetDefaultPrim()
    if root:
        paths.setdefault(root.GetDisplayName() or root.GetName(), str(root.GetPath()))
    return paths


def _inertial_of(xml: str) -> dict[str, tuple[float, np.ndarray, np.ndarray]]:
    """Each link's mass, centre of mass and inertia tensor, from its ``<inertial>``.

    A link without one is left out: in a converted MJCF that is a frame -- an IMU mount, a foot
    contact point, a lidar mount -- which is massless on purpose.
    """
    found: dict[str, tuple[float, np.ndarray, np.ndarray]] = {}
    for link in ET.fromstring(xml).iter("link"):
        name, inertial = link.get("name"), link.find("inertial")
        if name is None or inertial is None:
            continue
        mass_element, tensor = inertial.find("mass"), inertial.find("inertia")
        if mass_element is None or tensor is None:
            continue
        origin = inertial.find("origin")
        zero = (0.0, 0.0, 0.0)
        xyz = np.array(_floats(origin.get("xyz") if origin is not None else None, 3, zero))
        rpy = np.array(_floats(origin.get("rpy") if origin is not None else None, 3, zero))
        read = {
            key: float(tensor.get(key, 0.0)) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
        }
        matrix = np.array(
            [
                [read["ixx"], read["ixy"], read["ixz"]],
                [read["ixy"], read["iyy"], read["iyz"]],
                [read["ixz"], read["iyz"], read["izz"]],
            ]
        )
        # the tensor is about the centre of mass in the frame the origin's rpy names, so carry
        # that rotation into the axes rather than dropping it
        rotation = Rotation.from_euler("xyz", rpy).as_matrix()
        found[name] = (float(mass_element.get("value", 0.0)), xyz, rotation @ matrix @ rotation.T)
    return found


def _write_mass(pxr: Any, prim: Any, mass: float, centre: np.ndarray, tensor: np.ndarray) -> None:
    """Say the link's mass in the way a physics engine reading this layer looks for it.

    The URDF's tensor is a full symmetric matrix; UsdPhysics wants principal moments and the
    rotation onto them, so it is diagonalised. Without this a reader falls back to a default
    density over the visual meshes, which for a G1 means a robot whose every link weighs the wrong
    amount -- it loads, and it cannot stand up.
    """
    moments, axes = np.linalg.eigh(tensor)
    if np.linalg.det(axes) < 0:
        axes[:, 0] = -axes[:, 0]  # a reflection is not a rotation
    api = pxr.UsdPhysics.MassAPI.Apply(prim)
    api.CreateMassAttr(float(mass))
    api.CreateCenterOfMassAttr(pxr.Gf.Vec3f(*(float(value) for value in centre)))
    api.CreateDiagonalInertiaAttr(pxr.Gf.Vec3f(*(float(max(value, 0.0)) for value in moments)))
    quaternion = Rotation.from_matrix(axes).as_quat()
    api.CreatePrincipalAxesAttr(
        pxr.Gf.Quatf(
            float(quaternion[3]), float(quaternion[0]), float(quaternion[1]), float(quaternion[2])
        )
    )


def _write_link_tree(
    pxr: Any,
    stage: Any,
    *,
    parent_path: str,
    link: str,
    links: Mapping[str, Sequence[_Geometry]],
    children: Mapping[str, Sequence[JointDescription]],
    paths: dict[str, str],
    extras: Mapping[str, Sequence[str]],
    base_dir: Path | None,
    inertials: Mapping[str, tuple[float, np.ndarray, np.ndarray]] | None = None,
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
    _set_passthrough(pxr, prim, extras.get(link))
    pxr.UsdPhysics.RigidBodyAPI.Apply(prim)
    inertial = (inertials or {}).get(link)
    if inertial is not None:
        _write_mass(pxr, prim, *inertial)
    if joint is not None:
        xform.AddTransformOp().Set(_matrix(pxr, _zero_pose(joint)))

    for index, geometry in enumerate(links.get(link, ())):
        _write_geometry(
            pxr,
            stage,
            f"{prim_path}/{_geometry_name(geometry, index)}",
            geometry,
            base_dir,
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
            extras=extras,
            base_dir=base_dir,
            inertials=inertials,
            joint=child_joint,
        )


def _geometry_name(geometry: _Geometry, index: int) -> str:
    return f"{'collision' if geometry.collision else 'visual'}_{index}"


def _zero_pose(joint: JointDescription) -> NDArray[np.float64]:
    """Where this joint puts its child before anything moves.

    A floating or planar joint has no single position to be asked for, so its origin stands in
    -- which is also what parent_from_child(0) gives every other type.
    """
    if joint.type in _MULTI_DOF_JOINT_TYPES:
        return joint.origin()
    return joint.parent_from_child(0.0)


def _write_geometry(
    pxr: Any, stage: Any, path: str, geometry: _Geometry, base_dir: Path | None
) -> None:
    gprim = _define_gprim(pxr, stage, path, geometry, base_dir)
    if gprim is None:
        return
    gprim.AddTransformOp().Set(_matrix(pxr, geometry.origin))
    if geometry.color is not None:
        gprim.CreateDisplayColorAttr([pxr.Gf.Vec3f(*geometry.color)])
    if geometry.collision:
        # Collision geometry is carried for planners and self-filters, not for rendering.
        pxr.UsdGeom.Imageable(gprim).CreateVisibilityAttr(pxr.UsdGeom.Tokens.invisible)
        pxr.UsdPhysics.CollisionAPI.Apply(gprim.GetPrim())


def _define_gprim(
    pxr: Any, stage: Any, path: str, geometry: _Geometry, base_dir: Path | None
) -> Any:
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
        return _define_mesh(pxr, stage, path, geometry, base_dir)
    logger.warning(f"skipping URDF geometry of unsupported kind {geometry.kind!r}")
    return None


def _define_mesh(
    pxr: Any, stage: Any, path: str, geometry: _Geometry, base_dir: Path | None
) -> Any:
    """The file the URDF named, in assetInfo, plus its hash. Nothing is converted here.

    Named in ``assetInfo`` and **not** by a reference arc. A reference composes another USD
    layer; it is not "this geometry lives over there" for an arbitrary file. Pointing one at an
    STL makes the layer invalid -- every standard consumer reports

        Cannot determine file format for @.../arm.stl:SDF_FORMAT_ARGS:target=usd@

    and composes the mesh to nothing, which is a robot with no geometry in usdview, in Isaac and
    in MuJoCo. dimos never noticed because it reads the authored path itself and serves the file
    over its own bus rather than composing the stage.

    assetInfo is the right place for it: it is USD's own metadata for "which asset is this prim",
    it travels through grafting and publishing unchanged, and a consumer that cannot fetch the
    file sees an empty mesh rather than a broken layer.
    """
    assert geometry.filename is not None
    mesh = pxr.UsdGeom.Mesh.Define(stage, path)
    mesh.AddScaleOp().Set(pxr.Gf.Vec3f(*geometry.mesh_scale))
    # The path the URDF named, kept verbatim: those are the names UsdManager answers for.
    _set_asset_info(pxr, mesh.GetPrim(), geometry.filename, base_dir)
    return mesh


def _write_joints(
    pxr: Any,
    stage: Any,
    *,
    root_path: str,
    joints: Sequence[JointDescription],
    paths: Mapping[str, str],
    extras: Mapping[str, Sequence[str]],
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
        _set_passthrough(pxr, prim.GetPrim(), extras.get(joint.name))
        _set_joint_frames(pxr, prim, joint)
        _set_joint_limits(prim, joint)


def _define_joint(pxr: Any, stage: Any, path: str, joint: JointDescription) -> Any:
    if joint.type in {"revolute", "continuous"}:
        return pxr.UsdPhysics.RevoluteJoint.Define(stage, path)
    if joint.type == "prismatic":
        return pxr.UsdPhysics.PrismaticJoint.Define(stage, path)
    if joint.type == "fixed":
        return pxr.UsdPhysics.FixedJoint.Define(stage, path)
    if joint.type in _MULTI_DOF_JOINT_TYPES:
        # The base PhysicsJoint is the generic one: with nothing locked it is free in all six
        # degrees, which is a URDF floating joint. A planar joint is the same with the axes out
        # of its plane locked, which is what PhysicsLimitAPI is for. Both are standard, so
        # nothing downstream has to read metadata of ours to recognise them.
        prim = pxr.UsdPhysics.Joint.Define(stage, path)
        if joint.type == "planar":
            for axis in _OUT_OF_PLANE:
                locked = pxr.UsdPhysics.LimitAPI.Apply(prim.GetPrim(), axis)
                locked.CreateLowAttr(0.0)
                locked.CreateHighAttr(0.0)
        return prim
    logger.warning(f"joint {joint.name!r} of type {joint.type!r} has no USD equivalent; left out")
    return None


def _set_joint_frames(pxr: Any, prim: Any, joint: JointDescription) -> None:
    """The joint's frame in each body.

    ``physics:axis`` is only ever X, Y or Z, so a URDF axis that is none of those is folded
    into both local rotations: the joint is authored about z with the bodies turned instead.
    """
    # A joint with no single axis -- fixed, or free in several degrees -- is not turned onto z.
    single_axis = joint.type not in {"fixed", *_MULTI_DOF_JOINT_TYPES}
    to_z = _rotation_to_z(joint.axis) if single_axis else Rotation.identity()
    origin = Rotation.from_euler("xyz", joint.origin_rpy)
    prim.CreateLocalPos0Attr(pxr.Gf.Vec3f(*joint.origin_xyz))
    prim.CreateLocalRot0Attr(_quat(pxr, origin * to_z))
    prim.CreateLocalPos1Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    prim.CreateLocalRot1Attr(_quat(pxr, to_z))
    if single_axis:
        # Only the single-axis joint schemas have an axis; the generic PhysicsJoint has none.
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


# What urdf_to_usd reads for itself. Everything else a <link> or <joint> carries is passed
# through untouched rather than dropped, which is what <inertial>, <gazebo>, <transmission>,
# <sensor> and any vendor tag amount to.
_UNDERSTOOD_LINK_TAGS = frozenset({"visual", "collision"})
_UNDERSTOOD_JOINT_TAGS = frozenset({"origin", "parent", "child", "axis", "limit", "mimic"})


def _passthrough_xml(element: ET.Element, understood: frozenset[str]) -> list[str]:
    """The children of ``element`` this converter does not read, as XML text."""
    return [
        ET.tostring(child, encoding="unicode").strip()
        for child in element
        if child.tag not in understood
    ]


def _urdf_extras(xml: str) -> tuple[dict[str, list[str]], dict[str, list[str]]]:
    """Per-link and per-joint XML that urdf_to_usd does not itself understand.

    Inertia, gazebo blocks, transmissions, vendor tags: meaningless here, not a reason to lose.
    """
    root = ET.fromstring(xml)
    links = {
        name: extras
        for link in root.iter("link")
        if (name := link.get("name")) is not None
        and (extras := _passthrough_xml(link, _UNDERSTOOD_LINK_TAGS))
    }
    joints = {
        name: extras
        for joint in root.iter("joint")
        if (name := joint.get("name")) is not None
        and (extras := _passthrough_xml(joint, _UNDERSTOOD_JOINT_TAGS))
    }
    return links, joints


def asset_hash(data: bytes) -> str:
    """The hash an asset is identified by, in the form assetInfo["version"] carries."""
    return f"{HASH_ALGORITHM}:{hashlib.new(HASH_ALGORITHM, data).hexdigest()}"


def hash_of_file(path: str, base_dir: Path | None = None) -> str | None:
    """``path``'s asset hash, or None when it cannot be read from here."""
    resolved = Path(path)
    if not resolved.is_absolute() and base_dir is not None:
        resolved = base_dir / resolved
    try:
        return asset_hash(resolved.read_bytes())
    except OSError:
        return None


def _set_asset_info(pxr: Any, prim: Any, filename: str, base_dir: Path | None) -> None:
    """Identify the asset this prim references, and hash it if it can be read.

    assetInfo is USD's own metadata for this, so a consumer can check what it fetched against
    what the description meant without knowing anything about us.
    """
    prim.SetAssetInfoByKey("identifier", pxr.Sdf.AssetPath(filename))
    digest = hash_of_file(filename, base_dir)
    if digest is None:
        # The converter runs where the description lives, so this is worth saying out loud.
        logger.warning(f"cannot hash {filename!r}: not readable from here, so it gets no hash")
        return
    prim.SetAssetInfoByKey("version", digest)


def referenced_asset_paths(usd: str, root: Path | str | None = None) -> set[str]:
    """Every asset path ``usd`` names, as authored. What a renderer may ask to be served."""
    pxr = _pxr()
    layer = _layer_from(pxr, usd, "a layer", root)
    found: set[str] = set()
    stack = [layer.pseudoRoot]
    while stack:
        spec = stack.pop()
        found.update(_asset_paths_of(spec))
        stack.extend(spec.nameChildren)
    return found


def unhashed_assets(usd: str) -> dict[str, str]:
    """Every asset ``usd`` references without a hash, mapped to the prim path carrying it."""
    pxr = _pxr()
    stage = pxr.Usd.Stage.Open(_layer_from(pxr, usd, "a layer"))
    missing: dict[str, str] = {}
    for prim in stage.Traverse():
        info = prim.GetAssetInfo() or {}
        for asset in _referenced_assets(pxr, prim):
            if not info.get("version"):
                missing[asset] = str(prim.GetPath())
    return missing


def _referenced_assets(pxr: Any, prim: Any) -> list[str]:
    """The asset paths this prim names, as authored.

    Three ways a prim can name one: a reference arc, for a prim that really does compose
    another USD layer; assetInfo, which is how a mesh names the file it is; and an asset-valued
    attribute, which is how a texture names its image. Reading only arcs would make the hash
    check pass vacuously on every mesh, and reading only those two passed over every texture.
    """
    assets = []
    for spec in prim.GetPrimStack():
        items = spec.referenceList.explicitItems or spec.referenceList.prependedItems
        assets += [ref.assetPath for ref in items if ref.assetPath]
        assets += [str(value.path) for _, value in asset_attributes(spec)]
    identifier = (prim.GetAssetInfo() or {}).get("identifier")
    if identifier is not None and str(identifier.path):
        assets.append(str(identifier.path))
    assert pxr  # the pxr bundle is what opened the stage these specs came from
    return assets


def require_asset_hashes(usd: str, *, what: str) -> None:
    """Refuse a layer whose assets are not identified by content.

    The error carries the hash each one should have had, computed wherever the file can still
    be read, so the answer is the fix ready to paste.
    """
    missing = unhashed_assets(usd)
    if not missing:
        return
    lines = []
    for asset, prim_path in sorted(missing.items()):
        digest = hash_of_file(asset)
        lines.append(
            f"  {asset}  (at {prim_path})\n"
            + (
                f"    assetInfo version: {digest}"
                if digest
                else "    unreadable from here, so hash it where the file lives"
            )
        )
    listed = "\n".join(lines)
    raise ValueError(
        f"{what} references {len(missing)} asset(s) with no hash. Every asset needs one: a "
        f"path alone cannot say whether the file fetched is the file meant. Set "
        f'assetInfo["version"] on each prim below (urdf_to_usd does this for you):\n{listed}'
    )


def _set_passthrough(pxr: Any, prim: Any, xml: Sequence[str] | None) -> None:
    """Hang the URDF this converter did not read off the prim it belongs to.

    Nothing here is interpreted; it is kept so converting is not also a way to lose half a
    description.
    """
    if xml:
        # A plain python list is not a value USD can hold; an array of strings is.
        prim.SetCustomDataByKey(PASSTHROUGH_KEY, pxr.Vt.StringArray(list(xml)))


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
