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
_PXR: Pxr | None = None
_PXR_LOCK = Lock()


def load_pxr() -> Pxr:
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
    try:
        import mujoco._usd  # type: ignore[import-untyped]  # noqa: F401
    except ImportError:
        return


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
    pxr = load_pxr()
    root_layer = layer_from(pxr, root_usd, "root", root)
    root_stage = pxr.Usd.Stage.Open(root_layer)
    by_link = link_prim_paths(root_stage)

    for frame_id, subtree_usd in subtrees.items():
        attach_path = by_link.get(frame_id)
        if attach_path is None:
            raise ValueError(
                f"cannot attach a subtree at {frame_id!r}: the root USD has no prim for that "
                f"frame (it has {sorted(by_link)})"
            )
        sub_layer = layer_from(pxr, subtree_usd, frame_id, root)
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
        _rebase_relationships(pxr, root_layer, target, str(source.GetPath()), str(target))
        _mount_joint(pxr, root_stage, attach_path, str(target), frame_id)

    return str(root_layer.ExportToString())


def usd_with_frames(
    usd: str, frames: Mapping[str, tuple[str, Vector, Vector]], root: Path | str | None = None
) -> str:
    if not frames:
        return usd
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a scene", root)
    stage = pxr.Usd.Stage.Open(layer)
    for name, (parent, xyz, rpy) in frames.items():
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
    parent = stage.GetPrimAtPath(pxr.Sdf.Path(parent_path))
    joint = pxr.UsdPhysics.FixedJoint.Define(
        stage, pxr.Sdf.Path(path).AppendChild(prim_identifier(pxr, f"{name}_joint"))
    )
    if parent and parent.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
        joint.CreateBody0Rel().SetTargets([parent.GetPath()])
    joint.CreateBody1Rel().SetTargets([pxr.Sdf.Path(path)])
    joint.CreateLocalPos0Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.GetPrim().SetDisplayName(f"{name}_joint")


def usd_with_drives(
    usd: str, drives: Mapping[str, tuple[float, float, float]], root: Path | str | None = None
) -> str:
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
    return "linear" if joint.IsA(pxr.UsdPhysics.PrismaticJoint) else "angular"


def _is_joint(pxr: Pxr, prim: Prim) -> bool:
    return bool(prim.IsA(pxr.UsdPhysics.Joint))


def _write_mjc_servo(pxr: Pxr, actuator: Prim, kp: float, kd: float) -> None:
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
            token = pxr.Sdf.ValueTypeNames.Token
            double = pxr.Sdf.ValueTypeNames.Double
            actuator.CreateAttribute("mjc:forceLimited", token).Set("true")
            actuator.CreateAttribute("mjc:forceRange:min", double).Set(low)
            actuator.CreateAttribute("mjc:forceRange:max", double).Set(high)
    actuator.CreateAttribute("mjc:ctrlLimited", pxr.Sdf.ValueTypeNames.Token).Set("false")


def usd_drives(usd: str, root: Path | str | None = None) -> dict[str, tuple[float, float, float]]:
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


def rename_usd_frames(usd: str, renames: Mapping[str, str], root: Path | str | None = None) -> str:
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
    default = stage.GetDefaultPrim()
    if default and default not in frames:
        frames.append(default)
    named = [(prim, prim.GetDisplayName() or prim.GetName()) for prim in frames]
    changed = False
    for prim, name in named:
        if name in renames:
            prim.SetDisplayName(renames[name])
            changed = True
    return str(layer.ExportToString()) if changed else usd


def _mount_joint(pxr: Pxr, stage: Stage, mount_path: str, subtree_path: str, frame_id: str) -> None:
    root_link = _first_link(pxr, stage, subtree_path)
    if root_link is None:
        return
    mount = stage.GetPrimAtPath(pxr.Sdf.Path(mount_path))
    name = prim_identifier(pxr, f"{frame_id}_mount")
    joint = pxr.UsdPhysics.Joint.Define(stage, f"{subtree_path}/{name}")
    joint.CreateBody0Rel().SetTargets([mount.GetPath()])
    joint.CreateBody1Rel().SetTargets([root_link.GetPath()])
    joint.CreateLocalPos0Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.GetPrim().SetDisplayName(f"{_frame_name(root_link)}_mount")


def _first_link(pxr: Pxr, stage: Stage, path: str) -> Prim:
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
    pxr = load_pxr()
    layer = pxr.Sdf.Layer.CreateAnonymous(".usda")
    layer.ImportFromString('#usda 1.0\n(\n    defaultPrim = "warm"\n)\n\ndef Xform "warm"\n{\n}\n')
    stage = pxr.Usd.Stage.Open(layer)
    list(stage.Traverse())


def layer_from(pxr: Pxr, usd: str, what: str, root: Path | str | None = None) -> Layer:
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
    return pxr.Gf.Matrix4d(*np.asarray(transform, dtype=np.float64).T.reshape(-1).tolist())


def prim_identifier(pxr: Pxr, name: str) -> str:
    return str(pxr.Tf.MakeValidIdentifier(name))
