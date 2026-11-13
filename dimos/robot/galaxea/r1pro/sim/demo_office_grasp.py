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

"""Check prepared bottle contacts with each actual R1 gripper on a lifting fixture.

No robot is connected. The fixture fixes wrist orientation and drives a vertical
slide, while both finger joints and the bottle obey dynamics. It uses the same
finger gains, exclusions and mesh collisions as the office blueprint, without a
weld or attachment constraint. This checks contact feasibility, not arm IK.
"""

import copy
import json
import math
import xml.etree.ElementTree as ET

import mujoco

from dimos.robot.assets.model import RobotModel
from dimos.robot.galaxea.r1pro.config import R1PRO_COLLISION_EXCLUSIONS
from dimos.robot.galaxea.r1pro.sim.office import MODELS
from dimos.sim2.geometry.mujoco import build_scene
from dimos.sim2.scenes.sf555.pickables import ASSETS
from dimos.sim2.scenes.urdf import Component, prepare_scene


def check_grasp(side: str, bottle: str, height: float) -> dict[str, float | str]:
    source = ET.fromstring(MODELS["r1pro"].load().xml)
    root = ET.Element("robot", name="gripper_fixture")
    palm = f"r1pro/{side}_gripper_link"
    links = {palm, *(f"r1pro/{side}_gripper_finger_link{i}" for i in (1, 2))}
    for link in source.findall("link"):
        if link.attrib["name"] in links:
            root.append(copy.deepcopy(link))
    for joint in source.findall("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        assert parent is not None and child is not None
        if parent.attrib["link"] in links and child.attrib["link"] in links:
            root.append(copy.deepcopy(joint))
    model = RobotModel.from_urdf_string(ET.tostring(root, encoding="unicode"))
    fingers = tuple(f"r1pro/{side}_gripper_finger_joint{i}" for i in (1, 2))
    exclusions = tuple(
        (f"r1pro/{a}", f"r1pro/{b}")
        for a, b in R1PRO_COLLISION_EXCLUSIONS
        if f"r1pro/{a}" in links and f"r1pro/{b}" in links
    )
    scene = prepare_scene(
        {
            "gripper": Component(
                model,
                xyz=(0.065, 0, 0.30),
                rpy=(0, math.pi / 2, 0),
                drives={n: (1000, 10, 0) for n in fingers},
                collision_exclusions=exclusions,
            ),
            bottle: Component(MODELS[bottle], xyz=(0, 0, 0.30), floating=True),
        },
        ground_plane=False,
    )
    spec = build_scene(scene)
    # Palm local -X points world +Z in this explicitly staged fixture.
    spec.body(palm).add_joint(name="lift", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[-1, 0, 0])
    spec.add_actuator(
        name="lift",
        target="lift",
        trntype=mujoco.mjtTrn.mjTRN_JOINT,
        gainprm=[10000] + [0] * 9,
        biastype=mujoco.mjtBias.mjBIAS_AFFINE,
        biasprm=[0, -10000, -200] + [0] * 7,
    )
    half_table = (0.30 - height / 2) / 2
    spec.worldbody.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[0.3, 0.3, half_table],
        pos=[0, 0, half_table],
    )
    physics = spec.compile()
    state = mujoco.MjData(physics)
    for name, value in zip(fingers, (0.05, -0.05), strict=True):
        state.qpos[physics.joint(name).qposadr[0]] = value
    body = physics.body(f"props/{bottle}").id
    for tick in range(1500):
        aperture = max(0, 0.05 * (1 - tick / 250))
        state.ctrl[:2] = [aperture, -aperture]
        state.ctrl[2] = min(0.20, max(0, (tick - 500) / 500 * 0.20))
        mujoco.mj_step(physics, state)
    lifted = float(state.xpos[body, 2])
    assert lifted > 0.45, f"{side}/{bottle} failed to lift: {lifted}"
    for _ in range(500):
        state.ctrl[:2] = [0.05, -0.05]
        mujoco.mj_step(physics, state)
    released = float(state.xpos[body, 2])
    assert lifted - released > 0.10, f"{side}/{bottle} did not release"
    return {
        "side": side,
        "bottle": bottle,
        "lift_m": lifted - 0.30,
        "release_drop_m": lifted - released,
    }


if __name__ == "__main__":
    bottles = json.loads((ASSETS / "pickables.json").read_text())["bottles"]
    print(
        json.dumps(
            [
                check_grasp(side, bottle["id"], bottle["height_m"])
                for side in ("left", "right")
                for bottle in bottles
            ],
            indent=2,
        )
    )
