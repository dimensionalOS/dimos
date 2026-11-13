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

"""Check MuJoCo collisions against the saved office's native support/door probes."""

import argparse
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from dimos.robot.assets.model import RobotModel
from dimos.sim2.geometry.mujoco import compile_scene
from dimos.sim2.scenes.urdf import Component, prepare_scene

# The V24 design's documented course origin and reversed plan axes.
COURSE_ORIGIN = (29.43, 10.65)


def validate(assets: Path) -> None:
    started = time.monotonic()
    scene = prepare_scene(
        {"office": Component(RobotModel.from_file(assets / "office.urdf"))}, ground_plane=False
    )
    model = compile_scene(scene, 2_000_000)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    model.geom_group[:] = (model.geom_contype > 0).astype(np.int32)
    groups = np.array([0, 1, 0, 0, 0, 0], dtype=np.uint8)
    hit = np.zeros(1, dtype=np.int32)
    support = []
    for row in json.loads((assets / "reference_contacts.json").read_text()):
        z = row["structural_height_m"] + 0.06
        point = np.array([COURSE_ORIGIN[0] - row["u"], COURSE_ORIGIN[1] - row["v"], z])
        distance = mujoco.mj_ray(model, data, point, np.array([0.0, 0, -1]), groups, 1, -1, hit)
        height = None if distance < 0 else float(z - distance)
        support.append(
            {
                "name": row["name"],
                "height_m": height,
                "expected_m": row["surface_height_m"],
                "passed": height is not None and abs(height - row["surface_height_m"]) < 0.002,
            }
        )
    doors = []
    for u in [0.6, 0.95, 1.3]:
        for z in [0.7, 1.35, 2.1]:
            point = np.array([COURSE_ORIGIN[0] - u, COURSE_ORIGIN[1] - 12.40, z])
            distance = mujoco.mj_ray(model, data, point, np.array([0.0, -1, 0]), groups, 1, -1, hit)
            doors.append(
                {
                    "u": u,
                    "z": z,
                    "distance": float(distance),
                    "passed": distance < 0 or distance > 0.52,
                }
            )
    report = {
        "geometries": model.ngeom,
        "meshes": model.nmesh,
        "load_seconds": time.monotonic() - started,
        "support": support,
        "doorway": doors,
        "passed": all(x["passed"] for x in support + doors),
    }
    destination = assets / "validation"
    destination.mkdir(exist_ok=True)
    (destination / "geometry.json").write_text(json.dumps(report, indent=2))
    print({k: v for k, v in report.items() if k not in ("support", "doorway")}, flush=True)
    assert report["passed"], [x for x in support + doors if not x["passed"]]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets", type=Path, required=True)
    validate(parser.parse_args().assets)
