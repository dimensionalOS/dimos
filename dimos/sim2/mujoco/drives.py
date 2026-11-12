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


"""The servo gains a robot's USD authored onto its joints.

A drive is ``UsdPhysics.DriveAPI`` on a joint: a stiffness (kp), a damping (kd) and a target
position. A joint without one is a free hinge, so a robot whose layer says nothing would simply
fall over -- the backend reads the targets here to hold its initial pose, and MuJoCo reads the
gains off the same prims to servo it.
"""

from __future__ import annotations

from pathlib import Path

from dimos.robot.assets.usd_utils.pxr import Prim, Pxr, frame_name, layer_from, load_pxr


def usd_drives(usd: str, root: Path | str | None = None) -> dict[str, tuple[float, float, float]]:
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a robot", root))
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
