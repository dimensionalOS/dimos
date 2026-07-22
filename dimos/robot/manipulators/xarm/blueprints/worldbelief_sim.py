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

"""xArm6 WorldBelief perception stack against the MuJoCo sim.

Mirrors :mod:`worldbelief` (xarm6_worldbelief) with the RealSense wrist camera
replaced by :class:`MujocoSimModule`, which renders the MJCF ``wrist_camera``
and publishes the same color/depth/camera-info topics plus the
``link6 -> wrist_camera_*_optical_frame`` TF from simulation ground truth.
The ``world -> link6`` edge comes from ManipulationModule FK, which is
consistent with the sim because ``data/xarm6/scene.xml`` spawns ``link_base``
at the MuJoCo world origin (invariant; see grasp-sim-main history).
"""

from __future__ import annotations

from functools import partial

from dimos.agents.mcp.mcp_server import McpServer
from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.perception.worldbelief_module import WorldBeliefModule
from dimos.perception.worldbelief_recorder import WorldBeliefRecorder
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.blueprints.worldbelief import (
    _camera_info_to_rerun,
    _rerun_blueprint,
    _topic_to_entity,
)
from dimos.robot.manipulators.xarm.config import (
    XARM6_SIM_HOME,
    XARM6_SIM_PATH,
    make_xarm6_model_config,
    make_xarm6_sim_hardware,
    make_xarm6_sim_module_kwargs,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

_hw = make_xarm6_sim_hardware(XARM6_SIM_PATH)

xarm6_worldbelief_sim = autoconnect(
    # Provides world->link6 FK/TF (the sim camera's TF parent) and the gripper
    # skill path (set_gripper -> coordinator set_gripper_position RPC).
    ManipulationModule.blueprint(
        robots=[
            make_xarm6_model_config(
                name="arm",
                add_gripper=True,
                tf_extra_links=["link6", "link_base"],
                home_joints=XARM6_SIM_HOME,
            ),
        ],
    ),
    MujocoSimModule.blueprint(**make_xarm6_sim_module_kwargs(XARM6_SIM_PATH)),
    RerunBridgeModule.blueprint(
        blueprint=_rerun_blueprint,
        topic_to_entity=_topic_to_entity,
        visual_override={
            "world/color_camera": partial(
                _camera_info_to_rerun, image_topic="world/color_camera/color_image"
            ),
            "world/depth_camera": partial(
                _camera_info_to_rerun, image_topic="world/depth_camera/depth_image"
            ),
        },
        max_hz={
            "world/color_camera/color_image": 10.0,
            "world/depth_camera/depth_image": 5.0,
            "world/detections_3d": 10.0,
            "world/pointcloud": 5.0,
        },
    ),
    WorldBeliefRecorder.blueprint(
        db_path=STATE_DIR / "worldbelief" / "xarm6_sim" / "recordings" / "xarm6_worldbelief.db",
    ),
    WorldBeliefModule.blueprint(
        db_path=STATE_DIR / "worldbelief" / "xarm6_sim" / "recordings" / "xarm6_worldbelief.db",
        history_path=STATE_DIR / "worldbelief" / "xarm6_sim" / "worldbelief_history.db",
        scan_prompts=[],
        depth_tolerance_s=0.1,
        stationary_hz=4.0,
        yoloe_model_name="yoloe-11l-seg.pt",
        dino_model_name="facebook/dinov2-base",
        clip_model_name="openai/clip-vit-base-patch32",
    ),
    McpServer.blueprint(),
    coordinator(
        hardware=[_hw],
        tasks=[trajectory_task(_hw)],
    ),
).global_config(n_workers=8)
