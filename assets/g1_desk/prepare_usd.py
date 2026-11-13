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

"""Prepare the G1 desk USD bundle once, before launching Sim2.

Input: Jeff's dimos/robot/unitree/g1/assets directory at
9f18a8f335d1096b198f8ac2e4c09de3eb096c7d (fetch its Git LFS objects first).
Run from the repository root; the blueprint never calls this script.
"""

import argparse
import math
from pathlib import Path
import shutil

from pxr import Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt
import trimesh

from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import G1_GROOT_KD, G1_GROOT_KP
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.usd_utils.assets import resolve_usd_assets, write_asset_info
from dimos.robot.assets.usd_utils.frames import prefix_usd_frames, usd_with_frames
from dimos.robot.assets.usd_utils.layers import graft_usd
from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr
from dimos.robot.unitree.g1.sim2_policy import JOINTS, POLICY
from dimos.sim2.physics.drives import usd_with_drives


def prepare(g1_source: Path, out: Path) -> None:
    out = out.resolve()
    out.mkdir(exist_ok=True)
    shutil.copytree(g1_source, out / "g1", dirs_exist_ok=True)
    pxr = load_pxr()
    p = out / "g1/g1.usda"
    stage = Usd.Stage.Open(str(p))
    root = stage.GetDefaultPrim().GetPath()
    # Resolve the converter's top-level material prototypes once, so the component
    # can be grafted into another scene without references outside its default prim.
    materials = root.AppendChild("Materials")
    for material in stage.GetPrimAtPath(materials).GetChildren():
        material.SetInstanceable(False)
    flat = stage.Flatten()
    Sdf.CopySpec(flat, materials, stage.GetRootLayer(), materials)
    for prim in list(stage.GetPseudoRoot().GetChildren()):
        if str(prim.GetName()).startswith("Flattened_Prototype"):
            stage.RemovePrim(prim.GetPath())
    # Keep the head separately addressable for this sensor's optical-window exclusion.
    torso = next(
        p
        for p in stage.Traverse()
        if p.GetName() == "torso_link" and p.HasAPI(UsdPhysics.RigidBodyAPI)
    )
    head_path = torso.GetPath().AppendChild("head_body")
    head = UsdGeom.Xform.Define(stage, head_path)
    head.GetPrim().SetDisplayName("head_link")
    UsdPhysics.RigidBodyAPI.Apply(head.GetPrim())
    joint = UsdPhysics.FixedJoint.Define(stage, head_path.AppendChild("head_fixed"))
    joint.CreateBody0Rel().SetTargets([torso.GetPath()])
    joint.CreateBody1Rel().SetTargets([head_path])
    for prim in list(torso.GetChildren()):
        if prim.GetName() in ("head_link", "head_link_1"):
            Sdf.CopySpec(
                stage.GetRootLayer(),
                prim.GetPath(),
                stage.GetRootLayer(),
                head_path.AppendChild(prim.GetName()),
            )
            UsdPhysics.MassAPI.Apply(
                stage.GetPrimAtPath(head_path.AppendChild(prim.GetName()))
            ).CreateMassAttr(0)
            stage.RemovePrim(prim.GetPath())
    # The old site is replaced by the IMU provider's own rigid frame.
    for prim in list(stage.Traverse()):
        if prim.GetName() == "imu_in_pelvis":
            stage.RemovePrim(prim.GetPath())
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            prim.CreateAttribute("newton:armature", Sdf.ValueTypeNames.Float).Set(0.01)
            prim.CreateAttribute("newton:friction", Sdf.ValueTypeNames.Float).Set(0.1)
        if prim.HasAttribute("mjc:inertia"):
            prim.RemoveProperty("mjc:inertia")
    usd = usd_with_frames(
        stage.GetRootLayer().ExportToString(),
        {
            "imu_mount": ("pelvis", (0.04525, 0, -0.08339), (0, 0, 0)),
            "camera_mount": ("torso_link", (0.05, 0, 0.40), (-math.pi / 2, 0, -math.pi / 2)),
            "lidar_mount": (
                "torso_link",
                (0.0002835, 0.00003, 0.41618),
                (math.pi, 0.04014257279586953, 0),
            ),
        },
        root=out / "g1",
    )
    usd = prefix_usd_frames(usd, "g1")
    usd = usd_with_drives(
        usd,
        {
            n: (kp, kd, q)
            for n, kp, kd, q in zip(
                JOINTS, G1_GROOT_KP, G1_GROOT_KD, POLICY.default_positions_29, strict=True
            )
        },
        root=out / "g1",
    )
    stage = Usd.Stage.Open(layer_from(pxr, usd, "g1 drives", out / "g1"))
    efforts = {
        j.name: j.effort
        for j in RobotModel.from_file("dimos/robot/unitree/g1/g1.urdf").load().joints
    }
    for prim in stage.Traverse():
        if prim.GetTypeName() == "MjcActuator":
            limit = efforts[str(prim.GetName())]
            prim.CreateAttribute("mjc:forceLimited", Sdf.ValueTypeNames.Token).Set("true")
            prim.CreateAttribute("mjc:forceRange:min", Sdf.ValueTypeNames.Double).Set(-limit)
            prim.CreateAttribute("mjc:forceRange:max", Sdf.ValueTypeNames.Double).Set(limit)
    p.write_text(stage.GetRootLayer().ExportToString())
    # Native USD world: the world prim is a coordinate frame, not a floating rigid body.
    stage = Usd.Stage.CreateInMemory()
    world = UsdGeom.Xform.Define(stage, "/world")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.SetStageUpAxis(stage, "Z")
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    floor = UsdGeom.Plane.Define(stage, "/world/floor")
    floor.CreateAxisAttr("Z")
    floor.CreateWidthAttr(8)
    floor.CreateLengthAttr(8)
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    floor.CreateDisplayColorAttr([(0.25, 0.3, 0.35)])
    light = UsdLux.DistantLight.Define(stage, "/world/sun")
    light.CreateIntensityAttr(1)
    light.AddRotateXYZOp().Set((-25, -30, 0))
    usd = usd_with_frames(
        stage.GetRootLayer().ExportToString(),
        {
            "desk_mount": ("world", (2, 0, 0), (0, 0, math.pi / 2)),
            "box_mount": ("world", (1, 1, 0.05), (0, 0, 0)),
        },
    )
    (out / "world.usda").write_text(usd)
    # Desk authoring is an offline operation; the running blueprint only loads this USD.
    model = RobotModel.from_file(
        "assets/g1_desk/desk.urdf", package_paths={"g1_desk": Path("data/g1_desk").resolve()}
    )
    stage = Usd.Stage.Open(layer_from(pxr, model.to_usd().usd, "desk", None))
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Xformable):
            xf = UsdGeom.Xformable(prim)
            ops = xf.GetOrderedXformOps()
            xf.SetXformOpOrder(
                sorted(ops, key=lambda op: op.GetOpType() == UsdGeom.XformOp.TypeScale)
            )
    body = next(p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI))
    UsdPhysics.RigidBodyAPI(body).CreateKinematicEnabledAttr(True)
    for prim in list(stage.Traverse()):
        if prim.IsA(UsdGeom.Mesh):
            name = Path(str(prim.GetAssetInfo()["identifier"].path)).name
            source = Path("data/g1_desk") / name
            mesh = trimesh.load(source, force="mesh", process=False)
            geom = UsdGeom.Mesh(prim)
            geom.CreatePointsAttr(Vt.Vec3fArray(mesh.vertices.tolist()))
            geom.CreateFaceVertexCountsAttr([3] * len(mesh.faces))
            geom.CreateFaceVertexIndicesAttr(mesh.faces.ravel().tolist())
            geom.CreateSubdivisionSchemeAttr("none")
            shutil.copy(source, out / name)
            write_asset_info(pxr, prim, name, out)
            texname = {
                "desk.obj": "plane_divided_wood_floor_BaseColor.png",
                "computer.obj": "Computer_low_Desktop_BaseColor.png",
                "chair.obj": "SwivlChair_low_SwivlChair_BaseColor.png",
            }[name]
            shutil.copy(Path("data/g1_desk/textures") / texname, out / texname)
            if hasattr(mesh.visual, "uv") and mesh.visual.uv is not None:
                UsdGeom.PrimvarsAPI(prim).CreatePrimvar(
                    "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
                ).Set(Vt.Vec2fArray(mesh.visual.uv.tolist()))
            material = UsdShade.Material.Define(stage, str(prim.GetPath()) + "_material")
            shader = UsdShade.Shader.Define(stage, str(material.GetPath()) + "/surface")
            shader.CreateIdAttr("UsdPreviewSurface")
            texture = UsdShade.Shader.Define(stage, str(material.GetPath()) + "/texture")
            texture.CreateIdAttr("UsdUVTexture")
            texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texname))
            uv = UsdShade.Shader.Define(stage, str(material.GetPath()) + "/uv")
            uv.CreateIdAttr("UsdPrimvarReader_float2")
            uv.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            write_asset_info(pxr, texture.GetPrim(), texname, out)
            texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
                uv.ConnectableAPI(), "result"
            )
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
                texture.ConnectableAPI(), "rgb"
            )
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
    (out / "desk.usda").write_text(stage.GetRootLayer().ExportToString())
    box = RobotModel.from_file("assets/g1_desk/box.urdf").with_prefix("box/").to_usd().usd
    (out / "box.usda").write_text(box)
    print(out)

    # These two props belong to the authored environment, with the box free.
    world = graft_usd(
        (out / "world.usda").read_text(),
        {
            "desk_mount": (out / "desk.usda").read_text(),
            "box_mount": (out / "box.usda").read_text(),
        },
        root=out,
    )
    stage = Usd.Stage.Open(layer_from(pxr, world, "world", out))
    for prim in list(stage.Traverse()):
        if prim.GetTypeName() == "PhysicsJoint":
            if prim.GetName() == "box_mount_mount":
                stage.RemovePrim(prim.GetPath())
            else:
                prim.SetTypeName("PhysicsFixedJoint")
    (out / "world.usda").write_text(stage.GetRootLayer().ExportToString())

    world = graft_usd(
        (out / "world.usda").read_text(),
        {"world": resolve_usd_assets((out / "g1/g1.usda").read_text(), str(out / "g1"))},
        root=out,
    )
    stage = Usd.Stage.Open(layer_from(pxr, world, "g1 scene", out))
    for prim in list(stage.Traverse()):
        if prim.GetTypeName() == "PhysicsJoint" and prim.GetName() == "world_mount":
            stage.RemovePrim(prim.GetPath())
    # Store relative asset locations, never this author's workspace paths.
    (out / "world.usda").write_text(
        stage.GetRootLayer().ExportToString().replace(str(out) + "/", "")
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("g1_source", type=Path)
    parser.add_argument("--out", type=Path, default=Path("data/sim2_g1_usd"))
    args = parser.parse_args()
    prepare(args.g1_source, args.out)
