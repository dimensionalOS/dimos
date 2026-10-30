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

"""Unreal asset bindings survive robosuite scene composition."""

from contextlib import closing
import json
import xml.etree.ElementTree as ET

import mujoco

from dimos.sim2.environment import MotorScene
from dimos.sim2.sensors.camera.unreal.assets import export_scene


def test_motor_scene_preserves_custom_native_assets_and_other_authored_metadata(tmp_path):
    root = ET.fromstring("""
        <mujoco>
          <custom>
            <text name="authored_label" data="Workshop shelf"/>
            <numeric name="authored_dimensions" data="1 2 3"/>
          </custom>
          <worldbody>
            <geom name="shelf_visual" type="sphere" size=".001" mass="0"
              contype="0" conaffinity="0" pos="1 -2 3"/>
            <geom name="shelf_collision" type="box" size=".5 .2 1" pos="1 -2 4"/>
          </worldbody>
        </mujoco>
    """)
    metadata = {
        "version": 1,
        "geoms": {"shelf_visual": {"asset": "/Game/Props/Shelf.Shelf", "scale": [1, 2, 3]}},
        "hidden_geoms": ["shelf_collision"],
    }
    ET.SubElement(
        root.find("custom"), "text", name="dimos_unreal_assets", data=json.dumps(metadata)
    )
    path = tmp_path / "scene.xml"
    path.write_text(ET.tostring(root, encoding="unicode"))

    with closing(MotorScene(path, models={})) as environment:
        model = environment.model.get_model()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        scene = export_scene(model, data)

        label = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXT, "authored_label")
        start, size = model.text_adr[label], model.text_size[label]
        assert model.text_data[start : start + size - 1] == b"Workshop shelf"
        assert model.numeric_data.tolist() == [1, 2, 3]
        assert scene == {
            "version": 1,
            "meshes": [],
            "geoms": [
                {
                    "id": 0,
                    "position": [100, 200, 300],
                    "quaternion": [0, 0, 0, 1],
                    "static_mesh": "/Game/Props/Shelf.Shelf",
                    "scale": [1, 2, 3],
                }
            ],
        }
