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


"""Author and read USD drives used by the MuJoCo backend; targets use SI units."""

from collections.abc import Mapping
from pathlib import Path

import numpy as np

from dimos.robot.assets.usd_utils.pxr import Prim, Pxr, frame_name, layer_from, load_pxr


def usd_with_drives(
    usd: str, drives: Mapping[str, tuple[float, float, float]], root: Path | str | None = None
) -> str:
    if not drives:
        return usd
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a robot", root)
    stage = pxr.Usd.Stage.Open(layer)
    joints = {frame_name(prim): prim for prim in stage.Traverse() if _is_joint(pxr, prim)}
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
        drive.CreateTargetPositionAttr(
            float(target if _drive_kind(pxr, joint) == "linear" else np.degrees(target))
        )
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
        drives[frame_name(prim)] = (
            float(stiffness.Get()),
            float(drive.GetDampingAttr().Get() or 0.0),
            float(drive.GetTargetPositionAttr().Get() or 0.0)
            * (1.0 if _drive_kind(pxr, prim) == "linear" else np.pi / 180.0),
        )
    return drives
