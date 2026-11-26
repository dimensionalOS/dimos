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

"""Measure the actual M20 policy through the composed Sim2 deployment."""

import argparse
from functools import partial
import json
from pathlib import Path
import time
import uuid

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.experimental.gta_sim2.demo_interactive import physics_ready
from dimos.experimental.gta_sim2.m20 import M20PolicyModule, m20_blueprint
from dimos.experimental.gta_sim2.prepare_m20 import BASE
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.module import SimModule
from dimos.utils.testing.waiting import wait_until


def fixture(scene: Path, output: Path, terrain: str) -> Path:
    """Synthetic evaluation only; never replace the interactive map collision."""
    stage = Usd.Stage.Open(str(scene.resolve()))
    stage.RemovePrim("/world/gta_patch")
    # Evaluation fixtures run along +X regardless of the street spawn heading.
    body = UsdGeom.Xformable(stage.GetPrimAtPath("/" + BASE))
    transform = Gf.Matrix4d(1.0)
    transform.SetTranslate(body.GetLocalTransformation().ExtractTranslation())
    body.MakeMatrixXform().Set(transform)
    boxes: list[tuple[str, tuple[float, float, float], tuple[float, float, float]]] = [
        ("ground", (0, 0, -0.05), (40, 10, 0.1))
    ]
    if terrain == "stairs":
        for i in range(5):
            left = 1.5 + 0.35 * i
            length = 0.35 if i < 4 else 4.0
            height = 0.1 * (i + 1)
            boxes.append((f"step_{i}", (left + length / 2, 0, height / 2), (length, 6, height)))
    for name, center, size in boxes:
        cube = UsdGeom.Cube.Define(stage, "/world/evaluation/" + name)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    result = output.with_suffix(".usda").resolve()
    stage.Export(str(result))
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--seconds", type=float, default=4)
    parser.add_argument("--profile", choices=("sdk57", "dreamwaq342"), default="sdk57")
    parser.add_argument("--terrain", choices=("road", "flat", "stairs"), default="road")
    args = parser.parse_args()
    scene = args.scene.resolve()
    if args.terrain != "road":
        scene = fixture(scene, args.output, args.terrain)
    deployment = ModuleCoordinator.build(
        m20_blueprint(scene, args.policy.resolve(), args.profile)
        .global_config(zenoh_domain="m20-eval-" + uuid.uuid4().hex)
        .solidify()
    )
    try:
        sim = deployment.get_instance(SimModule)
        policy = deployment.get_instance(M20PolicyModule)
        wait_until(partial(physics_ready, sim), timeout=40)
        start = time.monotonic()
        rows: list[list[float]] = []
        forward_axis = np.zeros(2)
        while time.monotonic() - start < args.seconds:
            tick = time.monotonic()
            policy.move(Twist(linear=Vector3(args.speed, 0, 0)), duration=0.35)
            sample = sim.inspect()
            body = poses_by_id(sample)[BASE]
            q, p = body.orientation, body.position
            rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
            if not rows:
                forward_axis = rotation.as_matrix()[:2, 0]
                forward_axis /= np.linalg.norm(forward_axis)
            rows.append([sample.header.time_ns / 1e9, p.x, p.y, p.z, rotation.as_matrix()[2, 2]])
            time.sleep(max(0, 0.05 - (time.monotonic() - tick)))
        policy.move(Twist(), duration=0.35)
        a = np.asarray(rows)
        report = {
            "terrain": args.terrain,
            "command_mps": args.speed,
            "duration_s": float(a[-1, 0] - a[0, 0]),
            "distance_xy_m": float(np.linalg.norm(a[-1, 1:3] - a[0, 1:3])),
            "forward_axis_xy": forward_axis.tolist(),
            "forward_mps": float(
                np.dot(a[-1, 1:3] - a[0, 1:3], forward_axis) / (a[-1, 0] - a[0, 0])
            ),
            "min_upright": float(a[:, 4].min()),
            "z_range_m": [float(a[:, 3].min()), float(a[:, 3].max())],
            "sim": sim.status(),
            "policy": policy.status(),
            "trajectory": rows,
        }
        args.output.write_text(json.dumps(report, indent=2))
        print(json.dumps({k: v for k, v in report.items() if k != "trajectory"}), flush=True)
    finally:
        deployment.stop()


if __name__ == "__main__":
    main()
