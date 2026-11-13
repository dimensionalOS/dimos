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

"""Prepare R1's native USD scenes explicitly, after preparing the office URDFs."""

import argparse
from dataclasses import replace
import math

from dimos.robot.galaxea.r1pro.config import R1PRO_COLLISION_EXCLUSIONS
from dimos.robot.galaxea.r1pro.sim.config import JOINTS, KD, KP, MODEL, SCENE
from dimos.robot.galaxea.r1pro.sim.office import MODELS, SCENE as OFFICE_SCENE
from dimos.sim2.scenes.urdf import Component, prepare_scene


def prepare(*, office: bool = False) -> None:
    robot = Component(
        MODEL,
        xyz=(0, 0, 0.005),
        floating=True,
        drives={n: (max(kp, 1.0), kd, 0) for n, kp, kd in zip(JOINTS, KP, KD, strict=True)},
        collision_exclusions=tuple(
            (f"r1pro/{a}", f"r1pro/{b}")
            for a, b in R1PRO_COLLISION_EXCLUSIONS
            if "gmsl" not in a + b and "d405" not in a + b
        ),
    )
    components = {"r1pro": robot}
    output = SCENE
    if office:
        output = OFFICE_SCENE
        components = {
            "r1pro": replace(
                robot, model=MODELS["r1pro"], xyz=(18, 8.41, 0.005), rpy=(0, 0, math.pi)
            ),
            "office": Component(MODELS["office"]),
            "bottle_a": Component(
                MODELS["bottle_a"], xyz=(15.720000267, 8.409999847, 0.877500033), floating=True
            ),
            "bottle_b": Component(
                MODELS["bottle_b"], xyz=(17.109998703, 8.409999847, 0.871500025), floating=True
            ),
        }
    scene = prepare_scene(
        components, ground_plane=not office, mesh_dir=output.parent / "usd_meshes"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(scene.usd)
    print(output)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--office", action="store_true")
    prepare(**vars(parser.parse_args()))
