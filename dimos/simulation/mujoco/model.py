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


import mujoco
import numpy as np
from etils import epath
from mujoco_playground._src import mjx_env


from dimos.simulation.mujoco.policy import OnnxController
from dimos.simulation.mujoco.types import InputController

_HERE = epath.Path(__file__).parent


def get_assets() -> dict[str, bytes]:
    # Assets used from https://sketchfab.com/3d-models/mersus-office-8714be387bcd406898b2615f7dae3a47
    # Created by Ryan Cassidy and Coleman Costello
    assets: dict[str, bytes] = {}
    assets_path = _HERE / "../../../data/mujoco_sim/go1"
    mjx_env.update_assets(assets, assets_path, "*.xml")
    mjx_env.update_assets(assets, assets_path / "assets")
    path = mjx_env.MENAGERIE_PATH / "unitree_go1"
    mjx_env.update_assets(assets, path, "*.xml")
    mjx_env.update_assets(assets, path / "assets")
    return assets


def load_model(input_device: InputController, model=None, data=None):
    mujoco.set_mjcb_control(None)

    model = mujoco.MjModel.from_xml_path(
        (_HERE / "../../../data/mujoco_sim/go1/robot.xml").as_posix(),
        assets=get_assets(),
    )
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, 0)

    ctrl_dt = 0.02
    sim_dt = 0.01
    n_substeps = int(round(ctrl_dt / sim_dt))
    model.opt.timestep = sim_dt

    policy = OnnxController(
        policy_path=(_HERE / "../../../data/mujoco_sim/go1/go1_policy.onnx").as_posix(),
        default_angles=np.array(model.keyframe("home").qpos[7:]),
        n_substeps=n_substeps,
        action_scale=0.5,
        input_controller=input_device,
    )

    mujoco.set_mjcb_control(policy.get_control)

    return model, data
