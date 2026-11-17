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

"""Author the basic demonstration USD once; never called by the blueprint."""

import math
from pathlib import Path

from pxr import Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr
from dimos.sim2.physics.drives import usd_with_drives


def prepare() -> None:
    model = (
        RobotModel.from_file("assets/sim2_basic/scene.urdf")
        .with_fixed_frame(
            "camera_mount", "tool", xyz=(-0.25, 0, 0.6), rpy=(-3 * math.pi / 4, 0, -math.pi / 2)
        )
        .with_fixed_frame("lidar_mount", "tool", xyz=(0, 0, 0.12))
        .with_fixed_frame("imu_mount", "tool")
    )
    stage = Usd.Stage.Open(layer_from(load_pxr(), model.to_usd().usd, "basic", None))
    for prim in list(stage.Traverse()):
        if prim.IsA(UsdGeom.Xformable):
            xf = UsdGeom.Xformable(prim)
            ops = xf.GetOrderedXformOps()
            xf.SetXformOpOrder(
                sorted(ops, key=lambda op: op.GetOpType() == UsdGeom.XformOp.TypeScale)
            )
        name = prim.GetDisplayName() or prim.GetName()
        if name == "assets":
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            prim.SetDisplayName("world")
        if name == "box_joint":
            stage.RemovePrim(prim.GetPath())
            continue
        if name == "box" and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdGeom.Xformable(prim).ClearXformOpOrder()
            UsdGeom.Xformable(prim).AddTranslateOp().Set((0.1, 0, 0.075))
        if prim.IsA(UsdPhysics.Joint):
            j = UsdPhysics.Joint(prim)
            targets = j.GetBody0Rel().GetTargets()
            if targets and str(targets[0]).endswith("/assets"):
                j.GetBody0Rel().ClearTargets(True)
        if name == "tool_joint":
            act = stage.DefinePrim(
                stage.GetDefaultPrim().GetPath().AppendChild("actuator"), "MjcActuator"
            )
            act.CreateRelationship("mjc:target").SetTargets([prim.GetPath()])
            act.CreateAttribute("mjc:forceLimited", Sdf.ValueTypeNames.Token).Set("true")
            act.CreateAttribute("mjc:forceRange:min", Sdf.ValueTypeNames.Double).Set(-80)
            act.CreateAttribute("mjc:forceRange:max", Sdf.ValueTypeNames.Double).Set(80)
    floor = UsdGeom.Plane.Define(stage, "/sim2_basic/floor")
    floor.CreateAxisAttr("Z")
    floor.CreateWidthAttr(8)
    floor.CreateLengthAttr(8)
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    floor.CreateDisplayColorAttr([(0.25, 0.3, 0.35)])
    light = UsdLux.DistantLight.Define(stage, "/sim2_basic/light")
    light.CreateIntensityAttr(1)
    Path("assets/sim2_basic/scene.usda").write_text(
        usd_with_drives(stage.GetRootLayer().ExportToString(), {"tool_joint": (200, 20, 0)})
    )


if __name__ == "__main__":
    prepare()
