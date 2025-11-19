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


import xml.etree.ElementTree as ET

from etils import epath
import mujoco
from mujoco_playground._src import mjx_env
import numpy as np

from dimos.simulation.mujoco.policy import G1OnnxController, Go1OnnxController, OnnxController
from dimos.simulation.mujoco.types import InputController

DATA_DIR = epath.Path(__file__).parent / "../../../data/mujoco_sim"


def get_assets() -> dict[str, bytes]:
    # Assets used from https://sketchfab.com/3d-models/mersus-office-8714be387bcd406898b2615f7dae3a47
    # Created by Ryan Cassidy and Coleman Costello
    assets: dict[str, bytes] = {}
    mjx_env.update_assets(assets, DATA_DIR, "*.xml")
    mjx_env.update_assets(assets, DATA_DIR / "scene_office1/textures", "*.png")
    mjx_env.update_assets(assets, DATA_DIR / "scene_office1/office_split", "*.obj")
    mjx_env.update_assets(assets, mjx_env.MENAGERIE_PATH / "unitree_go1" / "assets")
    mjx_env.update_assets(assets, mjx_env.MENAGERIE_PATH / "unitree_g1" / "assets")
    return assets


def load_model(input_device: InputController, robot: str, scene: str):
    mujoco.set_mjcb_control(None)

    xml_string = get_model_xml(robot, scene)
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
        "policy_path": (DATA_DIR / f"{robot}_policy.onnx").as_posix(),
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


def get_model_xml(robot: str, scene: str):
    xml_file = (DATA_DIR / f"scene_{scene}.xml").as_posix()

    tree = ET.parse(xml_file)
    root = tree.getroot()
    root.set("model", f"{robot}_{scene}")
    root.insert(0, ET.Element("include", file=f"{robot}.xml"))
    return ET.tostring(root, encoding="unicode")
