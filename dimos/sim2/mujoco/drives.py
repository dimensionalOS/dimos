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


"""The servo gains (``UsdPhysics.DriveAPI`` stiffness, damping, target) a robot's USD authored onto its joints, which hold its initial pose."""

from __future__ import annotations

from pathlib import Path

from dimos.robot.assets.usd_utils.pxr import Prim, Pxr, Stage, frame_name, layer_from, load_pxr


def usd_drives(usd: str, root: Path | str | None = None) -> dict[str, tuple[float, float, float]]:
    pxr = load_pxr()
    return _stage_drives(pxr, pxr.Usd.Stage.Open(layer_from(pxr, usd, "a robot", root)))


def _stage_drives(pxr: Pxr, stage: Stage) -> dict[str, tuple[float, float, float]]:
    drives: dict[str, tuple[float, float, float]] = {}
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
            float(drive.GetTargetPositionAttr().Get() or 0.0),
        )
    return drives


def _drive_kind(pxr: Pxr, joint: Prim) -> str:
    return "linear" if joint.IsA(pxr.UsdPhysics.PrismaticJoint) else "angular"


def usd_with_mjc_servos(usd: str, root: Path | str | None = None) -> str:
    """Spell every UsdPhysics drive as the ``mjc:`` attributes MuJoCo's USD reader reads, which ignores DriveAPI, so the asset stays standard USD."""
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a robot", root)
    # One stage for reading the drives and writing the servos: opening a room's takes most of a second.
    stage = pxr.Usd.Stage.Open(layer)
    drives = _stage_drives(pxr, stage)
    if not drives:
        return usd
    actuators = {
        str(targets[0]): prim
        for prim in stage.Traverse()
        if prim.GetTypeName() == "MjcActuator"
        and (targets := prim.GetRelationship("mjc:target").GetTargets())
    }
    changed = False
    for prim in stage.Traverse():
        if not prim.IsA(pxr.UsdPhysics.Joint) or frame_name(prim) not in drives:
            continue
        actuator = actuators.get(str(prim.GetPath()))
        if actuator is None:
            continue
        kp, kd, _target = drives[frame_name(prim)]
        _write_mjc_servo(pxr, actuator, kp, kd)
        changed = True
    return str(layer.ExportToString()) if changed else usd


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
