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

"""USD as something to read and edit: the pxr modules, layers, frames, grafts and drives.

A stage is a string (a ``.usda`` layer), never a file. Conversion lives next door
(``urdf_to_usd``, ``usd_to_urdf``) and so does the asset bookkeeping (``usd_assets``); what is
here is the vocabulary all three share.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from types import ModuleType
from typing import TYPE_CHECKING, Any, TypeAlias
import uuid

import numpy as np
from scipy.spatial.transform import Rotation

if TYPE_CHECKING:
    from numpy.typing import NDArray


Vector = tuple[float, float, float]


_NO_USD = (
    "Reading or writing USD needs usd-core (pxr), which is in the 'sim' and 'scene' extras: "
    "uv sync --extra sim. On linux-aarch64 a wheel exists only from usd-core 26.11 and only for "
    "Python 3.11 and up, so a 3.10 arm robot cannot serve USD at all."
)
# Done once, behind a lock: a manager hashes every mounted subtree in parallel, and a
# concurrent first `from pxr import ...` wedges on CPython's per-module import lock and never
# comes back -- the module then answers nothing, not even its rpcs, and nothing has raised.
_PXR: Pxr | None = None
_PXR_LOCK = Lock()


def load_pxr() -> Pxr:
    """The pxr modules, imported once on first use. Deferred: usd-core is an optional extra
    (linux-aarch64 needs Python 3.11+) and a robot must pose itself on tf without it.
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
            _PXR = Pxr(
                Gf=Gf, Sdf=Sdf, Tf=Tf, Usd=Usd, UsdGeom=UsdGeom, UsdPhysics=UsdPhysics, Vt=Vt
            )
    return _PXR


def _let_mujoco_register_its_schemas_first() -> None:
    """Import MuJoCo's USD reader before pxr, where MuJoCo is installed at all.

    Whichever OpenUSD runtime initialises first owns the plugin registry; with usd-core first,
    MuJoCo's ``mjcPhysics`` schemas go unregistered and every ``MjcActuator`` is silently dropped
    (the G1 loaded with none of its 29 motors). This is where pxr is first touched.
    """
    try:
        import mujoco._usd  # type: ignore[import-untyped]  # noqa: F401
    except ImportError:
        # No MuJoCo here, so nothing is going to read a layer back with it.
        return


# pxr ships no stubs and these modules are imported lazily, so an alias is as far as a signature
# can go: it says what the object is without claiming anything is checked about it.
Layer: TypeAlias = Any
Matrix: TypeAlias = Any
Prim: TypeAlias = Any
Quat: TypeAlias = Any
Spec: TypeAlias = Any
Stage: TypeAlias = Any


@dataclass(frozen=True)
class Pxr:
    Gf: ModuleType
    Sdf: ModuleType
    Tf: ModuleType
    Usd: ModuleType
    UsdGeom: ModuleType
    UsdPhysics: ModuleType
    Vt: ModuleType


def graft_usd(root_usd: str, subtrees: Mapping[str, str], root: Path | str | None = None) -> str:
    """``root_usd`` with each subtree copied in under the prim standing for its frame.

    Keys are frame_ids in ``root_usd``, matched on ``displayName``. Specs are copied, not
    referenced: the scene leaves as a string with no layer the far side could resolve.
    """
    pxr = load_pxr()
    root_layer = layer_from(pxr, root_usd, "root")
    root_stage = pxr.Usd.Stage.Open(root_layer)
    by_link = link_prim_paths(root_stage)

    for frame_id, subtree_usd in subtrees.items():
        attach_path = by_link.get(frame_id)
        if attach_path is None:
            raise ValueError(
                f"cannot attach a subtree at {frame_id!r}: the root USD has no prim for that "
                f"frame (it has {sorted(by_link)})"
            )
        sub_layer = layer_from(pxr, subtree_usd, frame_id)
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
        # Copied specs keep authored targets, so a joint's body rels still name the old subtree root.
        _rebase_relationships(pxr, root_layer, target, str(source.GetPath()), str(target))
        _mount_joint(pxr, root_stage, attach_path, str(target), frame_id)

    return str(root_layer.ExportToString())


def usd_with_frames(
    usd: str, frames: Mapping[str, tuple[str, Vector, Vector]], root: Path | str | None = None
) -> str:
    """``usd`` with each named frame, ``(parent frame, xyz, rpy)``, added as a link.

    Mount points for things not part of the scene; grafting two subtrees at one frame is refused,
    so three robots in a room need three frames. Each carries RigidBodyAPI, which is what makes
    it a link to everything reading this layer.
    """
    if not frames:
        return usd
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a scene", root)
    stage = pxr.Usd.Stage.Open(layer)
    for name, (parent, xyz, rpy) in frames.items():
        # Read afresh each time: a frame may hang off one added a moment ago (the Go2's Mid-360 sits on
        # its front camera, itself a frame the rig adds), which is in the stage, not in the layer.
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
    """Weld the frame to the link it hangs off, or to the world where that is not a link.

    MuJoCo's default for a nested frame is a free body, which fails mjMINVAL because a frame is
    massless; a weld costs no degree of freedom (MuJoCo folds a welded massless body into its
    parent). A FixedJoint naming only body1 is the weld to the world, for a converted room whose
    root is an Xform. It still has to be a rigid body, or MuJoCo gives it no body at all.
    """
    parent = stage.GetPrimAtPath(pxr.Sdf.Path(parent_path))
    joint = pxr.UsdPhysics.FixedJoint.Define(
        stage, pxr.Sdf.Path(path).AppendChild(prim_identifier(pxr, f"{name}_joint"))
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

    Gains belong in the layer, so whatever runs the robot reads them from one place. Written
    twice: ``UsdPhysics.DriveAPI`` on the joint (standard, and what Isaac reads) and MuJoCo's
    ``mjc:gainPrm[0]=kp``, ``mjc:biasPrm=[0,-kp,-kd]``, ``mjc:biasType="affine"``. A motor's
    control range is a torque limit, so it moves to the force range: left in place it would clamp
    a servo's commanded position.
    """
    if not drives:
        return usd
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a robot", root)
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


def _drive_kind(pxr: Pxr, joint: Prim) -> str:
    """ "angular" for a hinge, "linear" for a slide: DriveAPI is applied under one of the two."""
    return "linear" if joint.IsA(pxr.UsdPhysics.PrismaticJoint) else "angular"


def _is_joint(pxr: Pxr, prim: Prim) -> bool:
    return bool(prim.IsA(pxr.UsdPhysics.Joint))


def _write_mjc_servo(pxr: Pxr, actuator: Prim, kp: float, kd: float) -> None:
    """MuJoCo's longhand for a position servo, on an actuator prim it will read back.

    A plain motor's control range is a torque and moves to the force range; one that already
    biases is already a servo (the G1 arrives with kp=500 and the joint's travel as its range),
    and moving that would cap each joint at a couple of newton-metres. The control limit comes
    off either way: travel is the joint's own range to enforce.
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
    """Every driven joint in ``usd``, as ``{joint: (kp, kd, target)}``.

    From `UsdPhysics.DriveAPI`, not MuJoCo's spelling: it sits on the joint prim and survives
    composition into a room. Same reason there is no keyframe -- a keyframe is the whole model's,
    and a composed world's qpos is a different vector.
    """
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a robot", root))
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

    Link names are tf frame_ids off the URDF, so two copies of one arm name the same frames; the
    namespace it is deployed under is already unique. Joint names move too -- they are what a
    JointState keys on. Only ``displayName`` changes: prim names cannot hold a "/" and prim paths
    are unique by nesting. The defaultPrim moves as well, because a converted room's root is a
    frame without being a link and the one frame every copy names the same; left behind, two
    namespaced copies would merge their subtrees there.
    """
    if not prefix:
        return usd
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a subtree")
    stage = pxr.Usd.Stage.Open(layer)
    root = stage.GetDefaultPrim()
    for prim in stage.Traverse():
        carries_a_frame = prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI) or prim.IsA(pxr.UsdPhysics.Joint)
        if carries_a_frame or (root and prim == root):
            prim.SetDisplayName(f"{prefix}/{prim.GetDisplayName() or prim.GetName()}")
    return str(layer.ExportToString())


def usd_frame_names(usd: str, root: Path | str | None = None) -> set[str]:
    """Every link and joint name ``usd`` carries, under the names they travel as.

    The defaultPrim counts: a converted room's root is a plain Xform, so the frame everything
    hangs off is not a link, and a scene rooted at a name tf never publishes is never drawn.
    Last, and only under a name no link has taken.
    """
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a layer", root))
    names = {
        prim.GetDisplayName() or prim.GetName()
        for prim in stage.Traverse()
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI) or prim.IsA(pxr.UsdPhysics.Joint)
    }
    default = stage.GetDefaultPrim()
    if default:
        names.add(default.GetDisplayName() or default.GetName())
    return names


def trivialize_usd(usd: str) -> str:
    """``usd`` with every link, joint and light kept, and the shapes they carry dropped.

    One description serves real hardware and a simulation, but only the sim needs the shapes, and
    the frames come out identical either way. Lights stay: a light is how the scene looks, and the
    one thing a viewer cannot work out for itself.
    """
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a layer")
    stage = pxr.Usd.Stage.Open(layer)
    # Whatever holds a frame stays; the prims holding no frame at all are the geometry.
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
        # A prim at the layer's own root has no nameParent -- a converted scene keeps its materials and
        # render settings up there, and removing one is the layer's business, not ours.
        if spec is not None and spec.nameParent is not None:
            del spec.nameParent.nameChildren[spec.name]
    return str(layer.ExportToString())


def _holds_a_frame(pxr: Pxr, prim: Prim) -> bool:
    if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI) or prim.IsA(pxr.UsdPhysics.Joint):
        return True
    # By type name rather than UsdLux.LightAPI, which load_pxr does not carry: the same test the viewer
    # makes, and it covers Dome, Distant, Sphere, Disk and RectLight without naming each.
    if str(prim.GetTypeName()).endswith("Light"):
        return True
    return any(_holds_a_frame(pxr, child) for child in prim.GetChildren())


def rename_usd_frames(usd: str, renames: Mapping[str, str], root: Path | str | None = None) -> str:
    """``usd`` with the named links and joints renamed; names it does not carry are ignored.

    Renames are applied simultaneously, so swapping a pair does not collapse both onto one. A
    missing name is skipped rather than refused, because one mapping is applied to every subtree
    of a scene; whether a key matched *anywhere* is the caller's to check (see usd_frame_names).
    """
    if not renames:
        return usd
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a subtree", root)
    stage = pxr.Usd.Stage.Open(layer)
    frames = [
        prim
        for prim in stage.Traverse()
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI) or prim.IsA(pxr.UsdPhysics.Joint)
    ]
    # The defaultPrim is a frame too, the way usd_frame_names and link_prim_paths count it.
    # Leaving it out means a rename validates and then quietly does nothing.
    default = stage.GetDefaultPrim()
    if default and default not in frames:
        frames.append(default)
    # Read every name before writing any, so a -> b alongside b -> a is a swap.
    named = [(prim, prim.GetDisplayName() or prim.GetName()) for prim in frames]
    changed = False
    for prim, name in named:
        if name in renames:
            prim.SetDisplayName(renames[name])
            changed = True
    return str(layer.ExportToString()) if changed else usd


def _mount_joint(pxr: Pxr, stage: Stage, mount_path: str, subtree_path: str, frame_id: str) -> None:
    """A floating joint from the mount frame to the subtree's root link.

    Nesting says a subtree hangs off the mount but not how, and the one guess a reader must not
    make is a weld: the subtree's root is a frame tf already publishes, so welding it would give
    it two parents and flip its pose between them. Free in all six degrees says "something else
    places this".
    """
    root_link = _first_link(pxr, stage, subtree_path)
    if root_link is None:
        return
    mount = stage.GetPrimAtPath(pxr.Sdf.Path(mount_path))
    name = prim_identifier(pxr, f"{frame_id}_mount")
    joint = pxr.UsdPhysics.Joint.Define(stage, f"{subtree_path}/{name}")
    joint.CreateBody0Rel().SetTargets([mount.GetPath()])
    joint.CreateBody1Rel().SetTargets([root_link.GetPath()])
    # The nesting already placed the subtree, so the joint adds no offset of its own.
    joint.CreateLocalPos0Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.GetPrim().SetDisplayName(f"{_frame_name(root_link)}_mount")


def _first_link(pxr: Pxr, stage: Stage, path: str) -> Prim:
    """The topmost prim at or below ``path`` carrying RigidBodyAPI: the subtree's root link."""
    root = stage.GetPrimAtPath(pxr.Sdf.Path(path))
    if not root:
        return None
    for prim in pxr.Usd.PrimRange(root):
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            return prim
    return None


def _frame_name(prim: Prim) -> str:
    return str(prim.GetDisplayName() or prim.GetName())


def warm_usd() -> None:
    """Load the USD bindings, on this thread, before anything uses them from several.

    Importing `pxr` once behind a lock is not enough: it pulls its own submodules in lazily from
    inside C code, the first time a layer is read, and two threads reaching that at once wedge on
    CPython's per-module import lock without raising.
    """
    pxr = load_pxr()
    layer = pxr.Sdf.Layer.CreateAnonymous(".usda")
    layer.ImportFromString('#usda 1.0\n(\n    defaultPrim = "warm"\n)\n\ndef Xform "warm"\n{\n}\n')
    # The stage has to outlive the traversal: a range over prims of a stage nobody holds walks
    # into expired prims, which USD reports as an error rather than an empty range.
    stage = pxr.Usd.Stage.Open(layer)
    list(stage.Traverse())


def layer_from(pxr: Pxr, usd: str, what: str, root: Path | str | None = None) -> Layer:
    """``usd`` as a layer. ``root`` is the directory its relative asset paths are relative to.

    With a root the layer takes an identifier *in* that directory -- a name nothing is written to
    -- so USD can resolve its references; without one every mesh warns "Could not open asset"
    once per prim while composing (139k lines on one three-G1 office start). The name is unique
    per call because `Sdf.Layer.New` refuses an identifier another live layer holds.
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
    pxr: Pxr, layer: Layer, root: Any, old_prefix: str, new_prefix: str
) -> None:
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


def link_prim_paths(stage: Stage) -> dict[str, str]:
    """URDF link name -> prim path, for every prim carrying ``UsdPhysics.RigidBodyAPI``.

    Narrower than "any Xform": it leaves out the joints, which could shadow a link of the same
    name. The name is displayName, or the prim name without one. The defaultPrim comes last and
    only under an untaken name, because it is what a scene is rooted at -- a converted room is a
    plain Xform, so without it there is nowhere to mount a robot in one.
    """
    pxr = load_pxr()
    paths: dict[str, str] = {}
    for prim in stage.Traverse():
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            paths[prim.GetDisplayName() or prim.GetName()] = str(prim.GetPath())
    root = stage.GetDefaultPrim()
    if root:
        paths.setdefault(root.GetDisplayName() or root.GetName(), str(root.GetPath()))
    return paths


def usd_matrix(pxr: Pxr, transform: NDArray[np.float64]) -> Matrix:
    """A 4x4 row-vector transform as USD's column-vector Gf.Matrix4d."""
    return pxr.Gf.Matrix4d(*np.asarray(transform, dtype=np.float64).T.reshape(-1).tolist())


def prim_identifier(pxr: Pxr, name: str) -> str:
    """``name`` as a USD prim name; the original spelling stays on the prim as its displayName."""
    return str(pxr.Tf.MakeValidIdentifier(name))
