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

"""Offline native-USD authoring for an exported fixed-base PolicyIK robot."""

import argparse
from itertools import combinations
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, Vt
import trimesh

from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.usd import layer_from, load_pxr, usd_with_drives


def excluded_pairs(urdf: Path) -> list[tuple[str, str]]:
    root = ET.parse(urdf).getroot()
    groups = {link.attrib["name"]: link.attrib["name"] for link in root.findall("link")}

    def representative(name: str) -> str:
        while groups[name] != name:
            name = groups[name]
        return name

    pairs = {tuple(sorted(("torso_link4", f"{side}_arm_link1"))) for side in ("left", "right")}
    pairs.update(
        tuple(sorted((f"{side}_arm_link{a}", f"{side}_{b}")))
        for side in ("left", "right")
        for a, b in ((5, "arm_link7"), (7, "d405_link"), (7, "gmsl_link"))
    )
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        assert parent is not None and child is not None
        a, b = parent.attrib["link"], child.attrib["link"]
        pairs.add(tuple(sorted((a, b))))
        if joint.attrib["type"] == "fixed":
            groups[representative(b)] = representative(a)
    pairs.update(
        (a, b) for a, b in combinations(groups, 2) if representative(a) == representative(b)
    )
    return [(a, b) for a, b in sorted(pairs)]


def prepare(bundle: Path, output: Path, case: int = 0) -> None:
    """Write a new scene; never alter the bundle or a previous preparation."""
    if output.exists():
        raise FileExistsError(output)
    manifest = json.loads((bundle / "manifest.json").read_text())
    with np.load(bundle / "development_cases.npz") as cases:
        q_start = cases["q_start"][case]
    urdf = bundle / "robot.urdf"
    layer = RobotModel.from_file(urdf).to_usd().usd
    stage = Usd.Stage.Open(layer_from(load_pxr(), layer, "R1 PolicyIK", bundle))
    bodies = {}
    joints = {}
    for prim in stage.Traverse():
        name = prim.GetDisplayName() or prim.GetName()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            bodies[name] = prim
        if prim.IsA(UsdPhysics.Joint):
            joints[name] = prim
        if prim.IsA(UsdGeom.Xformable):
            xf = UsdGeom.Xformable(prim)
            xf.SetXformOpOrder(
                sorted(
                    xf.GetOrderedXformOps(),
                    key=lambda op: op.GetOpType() == UsdGeom.XformOp.TypeScale,
                )
            )
        if prim.IsA(UsdGeom.Mesh):
            asset = prim.GetAssetInfo()["identifier"]
            path = Path(asset.path)
            if not path.is_absolute():
                path = bundle / path
            mesh = trimesh.load_mesh(path, process=False)
            geometry = UsdGeom.Mesh(prim)
            geometry.CreatePointsAttr(Vt.Vec3fArray(mesh.vertices.tolist()))
            geometry.CreateFaceVertexCountsAttr([3] * len(mesh.faces))
            geometry.CreateFaceVertexIndicesAttr(mesh.faces.ravel().tolist())
            geometry.CreateSubdivisionSchemeAttr("none")
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
    root = stage.GetDefaultPrim().GetPath()
    fixed = UsdPhysics.FixedJoint.Define(stage, root.AppendChild("fixed_chassis"))
    fixed.CreateBody1Rel().SetTargets([bodies[manifest["frame"]].GetPath()])
    for index, name in enumerate(manifest["joints"]):
        actuator = stage.DefinePrim(root.AppendChild("drive_" + name), "MjcActuator")
        actuator.CreateRelationship("mjc:target").SetTargets([joints[name].GetPath()])
        limit = manifest["effort_limits"][index]
        actuator.CreateAttribute("mjc:forceLimited", Sdf.ValueTypeNames.Token).Set("true")
        actuator.CreateAttribute("mjc:forceRange:min", Sdf.ValueTypeNames.Double).Set(-limit)
        actuator.CreateAttribute("mjc:forceRange:max", Sdf.ValueTypeNames.Double).Set(limit)
    for a, b in excluded_pairs(urdf):
        UsdPhysics.FilteredPairsAPI.Apply(bodies[a]).CreateFilteredPairsRel().AddTarget(
            bodies[b].GetPath()
        )
    floor = UsdGeom.Plane.Define(stage, root.AppendChild("floor"))
    floor.CreateAxisAttr("Z")
    floor.CreateWidthAttr(6)
    floor.CreateLengthAttr(6)
    floor.CreateDisplayColorAttr([(0.18, 0.22, 0.28)])
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    light = UsdLux.DistantLight.Define(stage, root.AppendChild("sun"))
    light.CreateIntensityAttr(1)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        usd_with_drives(
            stage.GetRootLayer().ExportToString(),
            {
                name: (kp, kd, q)
                for name, kp, kd, q in zip(
                    manifest["joints"], manifest["kp"], manifest["kd"], q_start, strict=True
                )
            },
            root=bundle,
        )
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--case", type=int, default=0)
    args = parser.parse_args()
    prepare(args.bundle.resolve(), args.output.resolve(), args.case)


if __name__ == "__main__":
    main()
