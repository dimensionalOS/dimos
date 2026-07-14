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

"""Simulation xArm perception manipulation blueprints."""

from __future__ import annotations

import math

from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    make_xarm7_model_config,
    make_xarm_hardware,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

XARM7_SIM_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, -0.7, 0.0]

_xarm7_sim_hw = make_xarm_hardware(
    "arm",
    7,
    adapter_type="sim_mujoco",
    address=str(XARM7_SIM_PATH),
    gripper=True,
    home_joints=XARM7_SIM_HOME,
)

xarm_perception_sim = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[
            make_xarm7_model_config(
                name="arm",
                add_gripper=True,
                pitch=math.radians(45),
                tf_extra_links=["link7"],
                home_joints=XARM7_SIM_HOME,
                pre_grasp_offset=0.05,
            )
        ],
        planning_timeout=10.0,
        visualization={"backend": "meshcat"},
    ),
    MujocoSimModule.blueprint(
        address=str(XARM7_SIM_PATH),
        headless=False,
        dof=7,
        camera_name="wrist_camera",
        base_frame_id="link7",
    ),
    ObjectSceneRegistrationModule.blueprint(target_frame="world"),
    coordinator(
        hardware=[_xarm7_sim_hw],
        tasks=[trajectory_task(_xarm7_sim_hw)],
    ),
    RerunBridgeModule.blueprint(),
)
