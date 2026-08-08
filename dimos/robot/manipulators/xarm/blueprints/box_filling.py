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

"""Real xArm6 box-filling product blueprint."""

import math

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.box_filling_pick_and_place_module import (
    BoxFillingPickAndPlaceModule,
)
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.visualization.rerun import picknplace_rerun_config
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import make_xarm6_model_config, xarm6_hardware
from dimos.robot.manipulators.xarm.grasp_config import make_xarm_graspgenx_config
from dimos.visualization.vis_module import vis_module

_CAMERA_TRANSFORM = Transform(
    translation=Vector3(0.06693724, -0.0309563, 0.00691482),
    rotation=Quaternion(0.70513398, 0.00535696, 0.70897578, -0.01052180),
)

BOX_FILLING_SYSTEM_PROMPT = """You operate an xArm that fills an open box.

Use scan_objects with simple object names. Select the destination using
select_destination_container, then select and pick each source object with
select_object and pick_selected. After a verified pick, use
place_in_destination. Treat tool results as authoritative and rescan after a
stale-selection or perception failure.
"""

_hardware = xarm6_hardware("arm", gripper=True)
_model = make_xarm6_model_config(
    name="arm",
    add_gripper=True,
    tf_extra_links=["link_base", "link6"],
    home_joints=[0.0, math.radians(-40.0), math.radians(-50.0), 0.0, math.radians(90.0), 0.0],
)
_model.max_velocity = 0.25
_model.max_acceleration = 0.5
_graspgenx = make_xarm_graspgenx_config()

xarm_box_filling = autoconnect(
    coordinator(hardware=[_hardware], tasks=[trajectory_task(_hardware)]),
    BoxFillingPickAndPlaceModule.blueprint(
        robots=[_model],
        visualization=ViserVisualizationConfig(port=8095),
        planning_timeout=10.0,
        planning_frame="world",
        grasp_verification={
            "open_position": 0.85,
            "closed_position": 0.0,
            "held_threshold": 0.02,
        },
    ),
    RealSenseCamera.blueprint(
        width=848,
        height=480,
        fps=15,
        camera_name="camera",
        base_frame_id="link6",
        base_transform=_CAMERA_TRANSFORM,
        enable_depth=True,
        align_depth_to_color=True,
        enable_pointcloud=False,
    ),
    ObjectSceneRegistrationModule.blueprint(
        instance_name="osr",
        target_frame="world",
        register_objects=False,
        detect_on_request=True,
        detector_confidence=0.4,
        object_voxel_downsample=0.001,
    ),
    GraspGenXModule.blueprint(
        instance_name="ggx",
        load_on_start=False,
        **_graspgenx.model_dump(
            exclude={"rpc_transport", "tf_transport", "g", "instance_name", "load_on_start"}
        ),
    ),
    vis_module(global_config.viewer, rerun_config=picknplace_rerun_config()),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=BOX_FILLING_SYSTEM_PROMPT),
).global_config(rerun_open="web")
