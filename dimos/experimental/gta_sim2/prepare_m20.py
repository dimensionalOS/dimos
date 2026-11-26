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

"""Replace the G1 in an authored patch with the M20's native MJCF-to-USD model."""

import argparse
import hashlib
from importlib import import_module
import json
import math
from pathlib import Path
import shutil

import mujoco
from pxr import Sdf, Usd

from dimos.experimental.gta_sim2.m20_actor import JOINTS, KD, KP, Profile, home_pose
from dimos.robot.assets.model import UsdModel
from dimos.sim2.description import joint_paths
from dimos.sim2.physics.drives import usd_with_drives

BASE = "world/m20/base_link"


def prepare(
    patch: Path,
    model: Path,
    meshes: Path,
    output: Path,
    profile: Profile = "sdk57",
    heading_degrees: float = 0.0,
) -> Path:
    import_module("mujoco._usd")  # Registers the native MJCF Sdf reader.
    output.mkdir(parents=True, exist_ok=True)
    spec = mujoco.MjSpec.from_file(str(model))
    spec.meshdir = str(meshes.resolve())
    home_pose_values = home_pose(profile)
    native = spec.compile()
    data = mujoco.MjData(native)
    for name, home in zip(JOINTS, home_pose_values, strict=True):
        data.qpos[native.joint(name).qposadr[0]] = home
    mujoco.mj_forward(native, data)
    wheels = [
        g
        for g in range(native.ngeom)
        if native.body(native.geom_bodyid[g]).name.endswith("_wheel") and native.geom_contype[g]
    ]
    lowest = min(float(data.geom_xpos[g, 2] - native.geom_size[g, 0]) for g in wheels)
    position = list(spec.body("base_link").pos)
    position[2] -= lowest - 0.005
    spec.body("base_link").pos = position
    angle = math.radians(heading_degrees) / 2
    spec.body("base_link").quat = [math.cos(angle), 0, 0, math.sin(angle)]
    for key in list(spec.keys):
        spec.delete(key)
    # MjSpec serialization uses its compiled snapshot; refresh after changing pose.
    spec.compile()
    spec.to_file(str(output / "robot.xml"))
    robot = Usd.Stage.Open(str(output / "robot.xml"))
    robot.Export(str(output / "robot.usda"))
    stage = Usd.Stage.Open(str(patch / "world.usda"))
    # Work on an anonymous copy; never modify the active G1's scene.
    layer = Sdf.Layer.CreateAnonymous()
    layer.ImportFromString(stage.Flatten().ExportToString())
    stage = Usd.Stage.Open(layer)
    stage.RemovePrim("/world/g1_29dof_rev_1_0")
    stage.DefinePrim("/world/m20", "Xform").GetReferences().AddReference(
        str(output / "robot.usda"), "/M20"
    )
    usd = (
        UsdModel.from_layer(stage.Flatten().ExportToString())
        .with_frames(
            {
                "imu_mount": (BASE, (0, 0, 0), (0, 0, 0)),
                "camera_mount": (
                    BASE,
                    (0.35, 0, 0.1),
                    (-1.5707963267948966, 0, -1.5707963267948966),
                ),
            }
        )
        .usd
    )
    paths = joint_paths(UsdModel.from_layer(usd))
    usd = usd_with_drives(
        usd,
        {
            paths[n]: (float(kp), float(kd), float(home))
            for n, kp, kd, home in zip(JOINTS, KP, KD, home_pose_values, strict=True)
        },
    )
    driven = Sdf.Layer.CreateAnonymous()
    driven.ImportFromString(usd)
    stage = Usd.Stage.Open(driven)
    for prim in stage.Traverse():
        if prim.GetTypeName() != "MjcActuator":
            continue
        actuator = native.actuator(prim.GetName())
        # Native motor ctrlrange is a torque limit; position-servo ctrlrange is
        # not. Preserve that limit explicitly across the USD conversion.
        low, high = native.actuator_ctrlrange[actuator.id]
        if profile == "dreamwaq342" and "wheel" in prim.GetName():
            low, high = max(low, -20.0), min(high, 20.0)
        prim.CreateAttribute("mjc:forceLimited", Sdf.ValueTypeNames.Token).Set("true")
        prim.CreateAttribute("mjc:forceRange:min", Sdf.ValueTypeNames.Double).Set(float(low))
        prim.CreateAttribute("mjc:forceRange:max", Sdf.ValueTypeNames.Double).Set(float(high))
    scene = output / "world.usda"
    scene.write_text(driven.ExportToString())
    for name in ("surfaces.npz",):
        shutil.copyfile(patch / name, output / name)
    manifest = json.loads((patch / "manifest.json").read_text())
    manifest["robot"] = {
        "name": "M20",
        "mjcf_sha256": hashlib.sha256(model.read_bytes()).hexdigest(),
        "source": str(model),
        "policy_contract": profile,
        "heading_degrees": heading_degrees,
        "stairs_validated": False,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2))
    return scene


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("patch", "model", "meshes", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--profile", choices=("sdk57", "dreamwaq342"), default="sdk57")
    parser.add_argument("--heading-degrees", type=float, default=0)
    args = parser.parse_args()
    print(
        prepare(
            args.patch.resolve(),
            args.model.resolve(),
            args.meshes.resolve(),
            args.output.resolve(),
            args.profile,
            args.heading_degrees,
        )
    )


if __name__ == "__main__":
    main()
