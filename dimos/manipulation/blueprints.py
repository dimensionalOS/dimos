# Copyright 2025-2026 Dimensional Inc.
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

"""Compatibility exports for manipulation blueprints.

Robot-owned manipulation blueprints now live under ``dimos.robot.manipulators``.
"""

import math

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.picknplace import PickNPlaceModule
from dimos.manipulation.visualization.rerun import picknplace_rerun_config
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.blueprints.agentic import (
    xarm7_planner_coordinator_agent as xarm7_planner_coordinator_agent,
    xarm_perception_agent as xarm_perception_agent,
    xarm_perception_sim_agent as xarm_perception_sim_agent,
)
from dimos.robot.manipulators.xarm.blueprints.basic import (
    xarm7_planner_coordinator as xarm7_planner_coordinator,
)
from dimos.robot.manipulators.xarm.blueprints.perception import xarm_perception as xarm_perception
from dimos.robot.manipulators.xarm.blueprints.simulation import (
    xarm_perception_sim as xarm_perception_sim,
)
from dimos.robot.manipulators.xarm.config import make_xarm6_model_config, xarm6_hardware
from dimos.robot.manipulators.xarm.grasp_config import make_xarm_graspgenx_config
from dimos.visualization.vis_module import vis_module

PICKNPLACE_CAMERA_TRANSFORM = Transform(
    translation=Vector3(0.06693724, -0.0309563, 0.00691482),
    rotation=Quaternion(0.70513398, 0.00535696, 0.70897578, -0.01052180),
)

BOX_FILLING_SYSTEM_PROMPT = """You are operating an xArm box-filling workspace with RGB-D perception.

Your recurring task is to collect requested blocks from the table and drop them into the measured white box. The available tools are the live interface to the robot, planner, gripper, and scene. Use their results as authoritative, make multiple calls when needed, and only report physical actions after a tool confirms success.

For a collection task: go home to observe, use ``scan_objects`` with separate simple noun phrases such as ``["colored wooden block", "white box"]``, estimate and install the table collision with no added margin, and measure the white box with ``install_open_box``. Use ``get_object_geometry`` to identify blocks whose centers are inside the measured box opening; those blocks are complete and must be ignored. Select only outside blocks, then call ``pick_selected``. If it succeeds, call ``place_selected`` to drop it into the remembered box. Repeat for other outside blocks. If pickup verification fails, rescan and select before another attempt.

When the user says put, place, or drop an object in the box, use ``place_selected``. It is a depth-derived drop: it computes the box-rim and held-object clearance itself, releases above the rim, and does not lower the end effector into the box. Do not substitute manually chosen poses or individual gripper commands for pick or drop sequences.
"""

_picknplace_xarm6_hardware = xarm6_hardware("arm", gripper=True)
_picknplace_xarm6_model = make_xarm6_model_config(
    name="arm",
    add_gripper=True,
    tf_extra_links=["link_base", "link6"],
    home_joints=[0.0, math.radians(-40.0), math.radians(-50.0), 0.0, math.radians(90.0), 0.0],
)
_picknplace_xarm6_model.max_velocity = 0.25
_picknplace_xarm6_model.max_acceleration = 0.5
_xarm_graspgenx = make_xarm_graspgenx_config()


picknplace = autoconnect(
    coordinator(
        hardware=[_picknplace_xarm6_hardware],
        tasks=[trajectory_task(_picknplace_xarm6_hardware)],
    ),
    ManipulationModule.blueprint(
        robots=[_picknplace_xarm6_model],
        visualization=ViserVisualizationConfig(port=8095),
        planning_timeout=10.0,
    ),
    RealSenseCamera.blueprint(
        width=848,
        height=480,
        fps=15,
        camera_name="camera",
        base_frame_id="link6",
        base_transform=PICKNPLACE_CAMERA_TRANSFORM,
        enable_depth=True,
        align_depth_to_color=True,
        enable_pointcloud=False,
    ),
    ObjectSceneRegistrationModule.blueprint(
        instance_name="osr",
        target_frame="link_base",
        register_objects=False,
        detect_on_request=True,
        detector_confidence=0.4,
        object_voxel_downsample=0.001,
    ),
    PickNPlaceModule.blueprint(instance_name="pnp", align_grasp_yaw=True),
    GraspGenXModule.blueprint(
        instance_name="ggx",
        load_on_start=False,
        **_xarm_graspgenx.model_dump(
            exclude={"rpc_transport", "tf_transport", "g", "instance_name", "load_on_start"}
        ),
    ),
    vis_module(
        global_config.viewer,
        rerun_config=picknplace_rerun_config(),
    ),
).global_config(rerun_open="web")

picknplace_agent = autoconnect(
    picknplace,
    McpServer.blueprint(
        allowed_skills=[
            "describe_scene",
            "scan_objects",
            "estimate_table",
            "select_object",
            "pick_selected",
            "place_selected",
            "get_object_geometry",
            "install_open_box",
            "set_table_collision",
            "get_robot_state",
            "reset",
            "move_to_pose",
            "close_gripper",
            "open_gripper",
            "go_home",
        ]
    ),
    McpClient.blueprint(system_prompt=BOX_FILLING_SYSTEM_PROMPT),
)
