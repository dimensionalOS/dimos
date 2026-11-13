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

"""Prepare the existing G1 office demo for the shared USD scene contract."""

import math

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import G1_GROOT_KD, G1_GROOT_KP
from dimos.robot.assets.model import RobotModel
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sim2_sf555 import ASSETS, SCENE
from dimos.robot.unitree.g1.sim2_policy import JOINTS, POLICY
from dimos.sim2.scenes.urdf import Component, prepare_scene
from dimos.utils.data import LfsPath


def prepare() -> None:
    robot = (
        RobotModel.from_file(
            DIMOS_PROJECT_ROOT / "dimos/robot/unitree/g1/g1.urdf",
            package_paths={"g1_description": LfsPath("g1_urdf")},
        )
        .with_subtree_rooted_at("pelvis")
        .with_fixed_frame(
            "camera", "torso_link", xyz=(0.05, 0, 0.40), rpy=(-math.pi / 2, 0, -math.pi / 2)
        )
        .with_prefix("g1/")
    )
    scene = prepare_scene(
        {
            "g1": Component(
                robot,
                xyz=(16, 1, 0.793),
                floating=True,
                armature=0.01,
                friction=0.1,
                drives={
                    n: (kp, kd, q)
                    for n, kp, kd, q in zip(
                        JOINTS, G1_GROOT_KP, G1_GROOT_KD, POLICY.default_positions_29, strict=True
                    )
                },
            ),
            "office": Component(RobotModel.from_file(ASSETS / "office.urdf")),
        },
        ground_plane=False,
        mesh_dir=ASSETS / "g1_usd_meshes",
    )
    SCENE.write_text(scene.usd)
    print(SCENE)


if __name__ == "__main__":
    prepare()
