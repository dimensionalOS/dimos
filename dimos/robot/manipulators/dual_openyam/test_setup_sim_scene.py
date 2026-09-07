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

import xml.etree.ElementTree as ET

import pytest

from dimos.robot.manipulators.dual_openyam.setup_sim_scene import write_demo_scene


def test_demo_scene_preserves_robot_and_contacts_and_is_reproducible(tmp_path):
    source = tmp_path / "source.xml"
    source.write_text("""<mujoco><asset>
      <mesh name="bin" file="garbage_can/mesh.obj" scale="0.01 0.02 0.03"/>
      <mesh name="arm" file="arm.obj" scale="1 1 1"/>
    </asset><worldbody>
      <body name="robot"><joint name="arm_joint" range="-1 2"/>
        <geom friction="4 0.1 0.01"/></body>
      <body name="bottle_1" pos="0.4 -0.3 0.75"><freejoint name="free_1"/>
        <geom name="bottle_geom" mesh="bottle" friction="0.6"/></body>
      <body name="bottle_2"/><body name="bottle_3"/>
      <body name="bottle_4" pos="0.4 0.3 0.75"><geom mesh="other"/></body>
      <body name="bottle_5"/><body name="bottle_6" pos="0.5 0 0.75"/>
    </worldbody><keyframe><key qpos="1"/></keyframe></mujoco>""")
    outputs = [tmp_path / "first.xml", tmp_path / "second.xml"]
    for output in outputs:
        write_demo_scene(source, output)
    original = ET.parse(source).getroot()
    scene = ET.parse(outputs[0]).getroot()

    assert outputs[0].read_bytes() == outputs[1].read_bytes()
    assert scene.find("keyframe") is None
    assert ET.tostring(scene.find("./worldbody/body[@name='robot']")) == ET.tostring(
        original.find("./worldbody/body[@name='robot']")
    )
    assert {body.get("name") for body in scene.findall("./worldbody/body")} == {
        "robot",
        "bottle_1",
        "bottle_4",
        "bottle_6",
    }
    assert scene.find("./asset/mesh[@name='bin']").get("scale") == "0.015 0.02 0.045"
    left = scene.find("./worldbody/body[@name='bottle_4']")
    assert left.get("pos") == "0.4 0.3 0.75"
    assert left.find("geom").get("mesh") == "bottle"
    assert left.find("geom").get("friction") == "0.6"
    names = [element.get("name") for element in scene.iter() if element.get("name")]
    assert len(names) == len(set(names))
    assert scene.find("./worldbody/body[@name='bottle_6']").get("pos") == "0.9 0.0 0.754"
    with pytest.raises(FileExistsError):
        write_demo_scene(source, outputs[0])
