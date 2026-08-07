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

"""GraspGenX-enabled real-hardware xArm perception blueprint."""

from __future__ import annotations

import math

from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.robot.manipulators.xarm.blueprints.perception import xarm_perception
from dimos.robot.manipulators.xarm.config import make_xarm7_model_config
from dimos.robot.manipulators.xarm.grasp_config import make_xarm_graspgenx_config

_graspgenx_config = make_xarm_graspgenx_config()

xarm_graspgenx = autoconnect(
    xarm_perception,
    PickAndPlaceModule.blueprint(
        robots=[
            make_xarm7_model_config(
                name="arm",
                add_gripper=True,
                pitch=math.radians(45),
                tf_extra_links=["link7"],
            )
        ],
        planning_timeout=10.0,
        visualization={"backend": "meshcat"},
        floor_z=-0.02,
        heuristic_grasp_fallback=False,
        planning_frame="world",
        grasp_approach_vector=(0.0, 0.0, -1.0),
        grasp_verification={
            # Enable only after completing the hardware calibration recorded
            # in the grasp-pipeline OpenSpec change.
            "enabled": False,
            "open_position": 0.85,
            "closed_position": 0.0,
            "held_threshold": 0.02,
            "timeout": 2.0,
            "poll_interval": 0.05,
        },
    ),
    GraspGenXModule.blueprint(
        **_graspgenx_config.model_dump(exclude={"rpc_transport", "tf_transport", "g"})
    ),
).global_config(n_workers=5)
