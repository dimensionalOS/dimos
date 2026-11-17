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

"""Prepare the pinned Booster model/checkpoint for sim2; run once when updating assets."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

import mujoco
from pxr import Usd, UsdPhysics, UsdShade

from dimos.robot.booster.k1.control_config import EFFORT_LIMITS, HOME, JOINTS, KD, KP
from dimos.sim2.physics.drives import usd_with_drives

ASSETS_REV = "12a97516f57469078707afb587a5f84c1cb110d1"
DEPLOY_REV = "7bb1462e2ac742eba5c01659b815e2e99fbe617f"


def prepare(assets: Path, deploy: Path, output: Path) -> None:
    for repo, expected in ((assets, ASSETS_REV), (deploy, DEPLOY_REV)):
        actual = subprocess.check_output(
            ["git", "-C", str(repo), "rev-parse", "HEAD"], text=True
        ).strip()
        if actual != expected:
            raise ValueError(f"{repo}: expected {expected}, got {actual}")
    output.mkdir(parents=True, exist_ok=True)
    source = assets / "robots/K1/K1_22dof.xml"
    spec = mujoco.MjSpec.from_file(str(source))
    spec.meshdir = str(source.parent / "meshes")
    spec.body("trunk").pos = (0, 0, 0.6)
    # Sim2 sensors own their mounted frames and measure the shared WorldState.
    for sensor in list(spec.sensors):
        spec.delete(sensor)
    spec.delete(spec.site("imu"))
    geometries = {}
    for index, geom in enumerate(spec.geoms):
        geom.name = f"k1_geom_{index}"
        geometries[geom.name] = geom
    for actuator, limit in zip(spec.actuators, EFFORT_LIMITS, strict=True):
        actuator.forcerange = (-limit, limit)
    spec.compile()
    with tempfile.TemporaryDirectory() as temp:
        xml = Path(temp) / "k1.xml"
        spec.to_file(str(xml))
        stage = Usd.Stage.Open(str(xml))
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.IsA(UsdPhysics.Joint):
                prim.SetDisplayName("k1/" + prim.GetName())
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                name = prim.GetParent().GetName() if prim.GetName() == "Mesh" else prim.GetName()
                geom = geometries[name]
                # The native exporter also marks visual-only meshes as colliders.
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(
                    bool(geom.contype or geom.conaffinity)
                )
                material = stage.GetPrimAtPath(f"/K1/PhysicsMaterials/{name}")
                if material:
                    # The exporter puts physical and visual materials into one
                    # binding's target list. USD expects a single material.
                    targets = prim.GetRelationship("material:binding").GetTargets()
                    if len(targets) == 2:
                        visual = stage.GetPrimAtPath(targets[1])
                        for attr in visual.GetAttributes():
                            if attr.GetName().startswith("outputs:") and attr.GetConnections():
                                material.CreateAttribute(
                                    attr.GetName(), attr.GetTypeName()
                                ).SetConnections(attr.GetConnections())
                    UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(material))
        usd = usd_with_drives(
            stage.GetRootLayer().ExportToString(),
            dict(zip(JOINTS, zip(KP, KD, HOME, strict=True), strict=True)),
        )
        (output / "world.usda").write_text(usd)
    checkpoint = deploy / "tasks/locomotion/robots/k1/models/k1_walk.pt"
    shutil.copyfile(checkpoint, output / checkpoint.name)
    shutil.copyfile(assets / "LICENSE", output / "LICENSE.booster_assets")
    shutil.copyfile(deploy / "LICENSE", output / "LICENSE.booster_deploy")
    (output / "provenance.json").write_text(
        json.dumps(
            {
                "booster_assets": ASSETS_REV,
                "booster_deploy": DEPLOY_REV,
                "mujoco": mujoco.__version__,
                "sha256": {
                    p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in output.iterdir()
                    if p.is_file() and p.name != "provenance.json"
                },
            },
            indent=2,
        )
        + "\n"
    )
    archive = output.parent / ".lfs" / "booster_k1.tar.gz"
    archive.parent.mkdir(exist_ok=True)
    with tarfile.open(archive, "w:gz") as tar:
        tar.add(output, arcname="booster_k1")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("assets", type=Path)
    parser.add_argument("deploy", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    prepare(args.assets, args.deploy, args.output)
