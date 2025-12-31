#!/usr/bin/env python3

# Copyright 2025 Dimensional Inc.
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


from pathlib import Path
import xml.etree.ElementTree as ET

from etils import epath  # type: ignore[import-untyped]
import mujoco  # type: ignore[import-untyped]
from mujoco_playground._src import mjx_env  # type: ignore[import-untyped]
import numpy as np

from dimos.core.global_config import GlobalConfig
from dimos.mapping.occupancy.extrude_occupancy import generate_mujoco_scene
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.simulation.mujoco.input_controller import InputController
from dimos.simulation.mujoco.policy import G1OnnxController, Go1OnnxController, OnnxController
from dimos.utils.data import get_data


def _get_data_dir() -> epath.Path:
    return epath.Path(str(get_data("mujoco_sim")))


def get_assets() -> dict[str, bytes]:
    data_dir = _get_data_dir()
    # Assets used from https://sketchfab.com/3d-models/mersus-office-8714be387bcd406898b2615f7dae3a47
    # Created by Ryan Cassidy and Coleman Costello
    assets: dict[str, bytes] = {}
    mjx_env.update_assets(assets, data_dir, "*.xml")
    mjx_env.update_assets(assets, data_dir / "scene_office1/textures", "*.png")
    mjx_env.update_assets(assets, data_dir / "scene_office1/office_split", "*.obj")
    mjx_env.update_assets(assets, mjx_env.MENAGERIE_PATH / "unitree_go1" / "assets")
    mjx_env.update_assets(assets, mjx_env.MENAGERIE_PATH / "unitree_g1" / "assets")
    return assets


def load_model(
    input_device: InputController, robot: str, scene_xml: str
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    mujoco.set_mjcb_control(None)

    xml_string = get_model_xml(robot, scene_xml)
    model = mujoco.MjModel.from_xml_string(xml_string, assets=get_assets())
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, 0)

    match robot:
        case "unitree_g1":
            sim_dt = 0.002
        case _:
            sim_dt = 0.005

    ctrl_dt = 0.02
    n_substeps = round(ctrl_dt / sim_dt)
    model.opt.timestep = sim_dt

    params = {
        "policy_path": (_get_data_dir() / f"{robot}_policy.onnx").as_posix(),
        "default_angles": np.array(model.keyframe("home").qpos[7:]),
        "n_substeps": n_substeps,
        "action_scale": 0.5,
        "input_controller": input_device,
        "ctrl_dt": ctrl_dt,
    }

    match robot:
        case "unitree_go1":
            policy: OnnxController = Go1OnnxController(**params)
        case "unitree_g1":
            policy = G1OnnxController(**params, drift_compensation=[-0.18, 0.0, -0.09])
        case _:
            raise ValueError(f"Unknown robot policy: {robot}")

    mujoco.set_mjcb_control(policy.get_control)

    return model, data


def get_model_xml(robot: str, scene_xml: str) -> str:
    root = ET.fromstring(scene_xml)
    root.set("model", f"{robot}_scene")
    root.insert(0, ET.Element("include", file=f"{robot}.xml"))

    # Ensure visual/map element exists with znear and zfar
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    map_elem = visual.find("map")
    if map_elem is None:
        map_elem = ET.SubElement(visual, "map")
    map_elem.set("znear", "0.01")
    map_elem.set("zfar", "10000")

    return ET.tostring(root, encoding="unicode")


def load_scene_xml(config: GlobalConfig) -> str:
    if config.mujoco_room_from_occupancy:
        path = Path(config.mujoco_room_from_occupancy)
        return generate_mujoco_scene(OccupancyGrid.from_path(path))

    mujoco_room = config.mujoco_room or "office1"
    xml_file = (_get_data_dir() / f"scene_{mujoco_room}.xml").as_posix()
    with open(xml_file) as f:
        return f.read()
