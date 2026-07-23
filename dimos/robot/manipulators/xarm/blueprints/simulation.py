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

from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

_xarm7_sim_hw = make_xarm7_sim_hardware(XARM7_SIM_PATH)

xarm_perception_sim = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[make_xarm7_sim_robot_config()],
        planning_timeout=10.0,
        # Sim stack runs on the RoboPlan (pinocchio) backend with its native
        # RRT planner; Drake stays the default for hardware blueprints.
        world_backend="roboplan",
        planner_name="roboplan",
        # Viser replaces the retired DrakeWorld meshcat; reachable over an SSH
        # port-forward to 127.0.0.1:8095 (logged as "Visualization:" at startup).
        # "Scan from here" drives the PickAndPlace scan_objects skill; the
        # ground-truth overlay mirrors the MJCF object positions.
        visualization={
            "backend": "viser",
            "scan_tool": "scan_objects",
            "ground_truth_objects": [
                {"name": "apple", "x": 0.40, "y": 0.08, "z": 0.17},
                {"name": "orange", "x": 0.45, "y": -0.08, "z": 0.175},
                {"name": "cup", "x": 0.50, "y": 0.0, "z": 0.19},
            ],
        },
    ),
    MujocoSimModule.blueprint(**make_xarm7_sim_module_kwargs(XARM7_SIM_PATH)),
    ObjectSceneRegistrationModule.blueprint(target_frame="world"),
    coordinator(
        hardware=[_xarm7_sim_hw],
        tasks=[trajectory_task(_xarm7_sim_hw)],
    ),
    RerunBridgeModule.blueprint(),
)
