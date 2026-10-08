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

"""Robosuite stacking through the raw robot bridge: wrist and overview images, twist commands.

MUJOCOSIMMODULE__HEADLESS=false dimos evals run dimos.evals.suites.robosuite_twist \
    --agent dimos.evals.agents.decisions

The coordinator assumes the default 0.12 m base height while these scenes mount it at
0.912 m, so ee_pose on the bridge is offset; twists and images are unaffected.
"""

from dimos.evals.constants import RAW_ARM_README
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.scorers import stacked_on
from dimos.evals.types import EvalCase, Suite
from dimos.utils.data import LfsPath

SUITE: Suite = [
    EvalCase(
        id="robosuite_stack_cubes_twist",
        inputs=(
            "Stack the small red cube (4 cm wide) on top of the larger green cube (5 cm wide), "
            "then release it."
        ),
        environment=MujocoEnvironment(
            blueprint=["xarm-sim", "mcp-server"],
            raw_bridge=True,
            raw_guide=RAW_ARM_README,
            module_env={
                "RAWROBOTBRIDGE__CAMERA_FRAME": "wrist_camera_color_optical_frame",
                "RAWROBOTBRIDGE__OVERVIEW_FRAME": "env_camera_color_optical_frame",
                "RAWROBOTBRIDGE__EE_FRAME": "link_tcp",
                "RAWROBOTBRIDGE__GRIPPER_JOINT": "arm/gripper",
                "RAWROBOTBRIDGE__GRIPPER_RANGE": "[0.0, 0.85]",
            },
            ready_streams=("color_image", "overview_image", "coordinator_joint_state"),
            scene=LfsPath("robosuite/stack/scene.xml"),
            tracked_bodies=("cubeA_main", "cubeB_main"),
        ),
        grade=stacked_on("cubeA_main", "cubeB_main", rise_m=(0.041, 0.049), band_m=0.03),
        threshold=0.5,
        timeout_s=900.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "stack", "raw"}),
    ),
]
