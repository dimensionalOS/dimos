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

"""RoboLab's DROID robot, simulated by NVIDIA RoboLab in Isaac Sim.

RoboLab runs the scene and the robot; start its runner first
(dimos/simulation/robolab/runner/run.py), then:

    dimos --simulation run droid-robolab-sim

The runner's zenoh endpoint defaults to tcp/127.0.0.1:7448; set both the adapter's
address and ``--robolabsimmodule.address`` when it runs elsewhere.
"""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXModule
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_localize import ManipulationLocalize
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.perception.localize.types import LocalizeParams
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.droid.config import (
    DROID_GRASP_FRAME_TO_TCP,
    DROID_GRIPPER_JOINT,
    DROID_GRIPPER_SWEEP_VOLUME,
    DROID_GRIPPER_TASK,
    droid_robolab_hardware,
    make_droid_model_config,
)
from dimos.simulation.robolab.robolab_sim_module import RoboLabSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

_droid_hw = droid_robolab_hardware()

droid_robolab_sim = autoconnect(
    ManipulationModule.blueprint(
        gripper_tasks={"manipulator": DROID_GRIPPER_TASK},
        model=make_droid_model_config(),
        planning_timeout=10.0,
        visualization={"backend": "viser"},
        # Isaac runs slower than the wall clock: execute returns once the arm is there.
        arrival_tolerance=0.01,
        arrival_timeout=60.0,
    ),
    ManipulationSkills.blueprint(),
    # Grasps from an object's point cloud, e.g. ManipulationLocalize.get_pointcloud.
    HeuristicGraspModule.blueprint(),
    RoboLabSimModule.blueprint(),
    # RoboLab's camera is fixed, so every sighting comes from one view.
    ManipulationLocalize.blueprint(world_frame="world", params=LocalizeParams(min_views=1)),
    coordinator(
        hardware=[_droid_hw],
        tasks=[
            trajectory_task(_droid_hw),
            TaskConfig(
                name=DROID_GRIPPER_TASK,
                type="gripper",
                joint_names=[DROID_GRIPPER_JOINT],
                priority=20,
                # finger_joint is 0 open, pi/4 closed.
                params={"open_at": "lower"},
            ),
        ],
    ),
    RerunBridgeModule.blueprint(),
)

# Learned grasps for the Robotiq 2F-85, beside the heuristic ones. Needs a GPU.
droid_robolab_graspgenx = GraspGenXModule.blueprint(
    gripper=DROID_GRIPPER_SWEEP_VOLUME,
    grasp_frame_to_tcp=DROID_GRASP_FRAME_TO_TCP,
    # Return every sample: the best-scored ones often all come in from one side.
    max_candidates=500,
)
