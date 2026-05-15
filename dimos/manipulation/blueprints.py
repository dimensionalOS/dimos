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

"""
Manipulation blueprints.

Quick start:
    # 1. Verify manipulation deps load correctly (standalone, no hardware):
    dimos run xarm6-planner-only

    # 2. Keyboard teleop with mock arm:
    dimos run keyboard-teleop-xarm7

    # 3. Interactive RPC client (plan, preview, execute from Python):
    dimos run xarm7-planner-coordinator
    python -i -m dimos.manipulation.planning.examples.manipulation_client
"""

from typing import Any
import math

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.memory2 import LazyPerceptionModule, RGBDCameraRecorder
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.catalog.ufactory import xarm6 as _catalog_xarm6, xarm7 as _catalog_xarm7
from dimos.visualization.vis_module import vis_module

# Single XArm6 planner (standalone, no coordinator)
_xarm6_planner_cfg = _catalog_xarm6(
    name="arm",
    adapter_type="xarm" if global_config.xarm6_ip else "mock",
    address=global_config.xarm6_ip,
)

xarm6_planner_only = ManipulationModule.blueprint(
    robots=[_xarm6_planner_cfg.to_robot_model_config()],
    planning_timeout=10.0,
    enable_viz=True,
).transports(
    {
        ("joint_state", JointState): LCMTransport("/xarm/joint_states", JointState),
    }
)


# Dual XArm6 planner with coordinator integration
# Usage: Start with coordinator_dual_mock, then plan/execute via RPC
_left_arm_cfg = _catalog_xarm6(
    name="left_arm",
    adapter_type="xarm" if global_config.xarm6_ip else "mock",
    address=global_config.xarm6_ip,
    y_offset=0.5,
)
_right_arm_cfg = _catalog_xarm6(
    name="right_arm",
    adapter_type="xarm" if global_config.xarm6_ip else "mock",
    address=global_config.xarm6_ip,
    y_offset=-0.5,
)

dual_xarm6_planner = ManipulationModule.blueprint(
    robots=[
        _left_arm_cfg.to_robot_model_config(),
        _right_arm_cfg.to_robot_model_config(),
    ],
    planning_timeout=10.0,
    enable_viz=True,
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


# Single XArm7 planner + coordinator (uses real hardware when XARM7_IP is set)
# Usage: XARM7_IP=<ip> dimos run xarm7-planner-coordinator
_xarm7_cfg = _catalog_xarm7(
    name="arm",
    adapter_type="xarm" if global_config.xarm7_ip else "mock",
    address=global_config.xarm7_ip,
)

xarm7_planner_coordinator = autoconnect(
    ManipulationModule.blueprint(
        robots=[_xarm7_cfg.to_robot_model_config()],
        planning_timeout=10.0,
        enable_viz=True,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm7_cfg.to_hardware_component()],
        tasks=[_xarm7_cfg.to_task_config()],
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


# XArm7 planner + LLM agent for testing base ManipulationModule skills
# No perception — uses the base module's planning + gripper skills only.
# Usage: dimos run coordinator-mock, then dimos run xarm7-planner-coordinator-agent
_BASE_MANIPULATION_AGENT_SYSTEM_PROMPT = """\
You are a robotic manipulation assistant controlling an xArm7 robot arm.

Available skills:
- get_robot_state: Get current joint positions, end-effector pose, and gripper state.
- move_to_pose: Move end-effector to ABSOLUTE x, y, z (meters) with optional roll, pitch, yaw (radians).
- move_to_joints: Move to a joint configuration (comma-separated radians).
- open_gripper / close_gripper / set_gripper: Control the gripper.
- go_home: Move to the home/observe position.
- go_init: Return to the startup position.
- reset: Clear a FAULT state and return to IDLE. Use this when a motion fails.

COORDINATE SYSTEM (world frame, meters):
- X axis = forward (away from the robot base)
- Y axis = left
- Z axis = up
- Z=0 is the robot base level; typical working height is Z = 0.2-0.5

CRITICAL WORKFLOW for relative movement requests (e.g. "move 20cm forward"):
1. Call get_robot_state to get the current EE pose.
2. Add the requested offset to the CURRENT position. Example: if EE is at \
(0.3, 0.0, 0.4) and user says "move 20cm forward", target is (0.5, 0.0, 0.4).
3. Call move_to_pose with the computed ABSOLUTE target.
NEVER pass only the offset as coordinates — that would send the robot to near-origin.

ERROR RECOVERY: If a motion fails or the state becomes FAULT, call reset before retrying.
"""

xarm7_planner_coordinator_agent = autoconnect(
    xarm7_planner_coordinator,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=_BASE_MANIPULATION_AGENT_SYSTEM_PROMPT),
)


# XArm7 with eye-in-hand RealSense camera for perception-based manipulation
# TF chain: world → link7 (ManipulationModule) → camera_link (RealSense)
# Usage: dimos run coordinator-mock, then dimos run xarm-perception
_XARM_PERCEPTION_CAMERA_TRANSFORM = Transform(
    translation=Vector3(x=0.06693724, y=-0.0309563, z=0.00691482),
    rotation=Quaternion(0.70513398, 0.00535696, 0.70897578, -0.01052180),  # xyzw
)

_xarm7_perception_cfg = _catalog_xarm7(
    name="arm",
    adapter_type="xarm" if global_config.xarm7_ip else "mock",
    address=global_config.xarm7_ip,
    pitch=math.radians(45),
    add_gripper=True,
    tf_extra_links=["link7"],
)

xarm_perception = (
    autoconnect(
        PickAndPlaceModule.blueprint(
            robots=[_xarm7_perception_cfg.to_robot_model_config()],
            planning_timeout=10.0,
            enable_viz=True,
            floor_z=-0.02,
        ),
        RealSenseCamera.blueprint(
            base_frame_id="link7",
            base_transform=_XARM_PERCEPTION_CAMERA_TRANSFORM,
        ),
        ObjectSceneRegistrationModule.blueprint(
            target_frame="world",
            distance_threshold=0.08,
            min_detections_for_permanent=3,
            max_distance=1.0,
            use_aabb=True,
            max_obstacle_width=0.06,
        ),
        vis_module("foxglove"),
    )
    .transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        }
    )
    .global_config(n_workers=4)
)


# XArm7 perception + LLM agent for agentic manipulation.
# Skills (pick, place, move_to_pose, etc.) auto-register with the agent's SkillCoordinator.
# Usage: XARM7_IP=<ip> dimos run coordinator-xarm7 xarm-perception-agent
_MANIPULATION_AGENT_SYSTEM_PROMPT = """\
You are a robotic manipulation assistant controlling an xArm7 robot arm with an \
eye-in-hand RealSense camera and a gripper.

# Skills

## Perception
- **look**: Quick snapshot of objects visible from the current camera pose. Does NOT \
move the arm. Example: "what do you see?", "what's on the table?"
- **scan_objects**: Full scan — moves the arm to the init position for a clear view, \
then refreshes detections. Use before pick/place, after a failed grasp, or when the \
user explicitly asks to scan. Example: "scan the table", "what objects are there?"

## Pick & Place
- **pick <object_name>**: Pick up a detected object by name. Use the EXACT name from \
look/scan_objects output. When duplicates exist, pass the object_id shown in brackets \
(e.g. [id=abc12345]). Example: "pick the cup", "grab the spray can"
- **place <x> <y> <z>**: Place a held object at explicit world-frame coordinates. \
Example: "place it at 0.4, 0.3, 0.1"
- **drop_on <object_name>**: Drop a held object onto another detected object. \
Automatically compensates for camera occlusion. Example: "drop it in the bowl", \
"put it on the box"
- **place_back**: Return a held object to its original pick position.
- **pick_and_place <object_name> <x> <y> <z>**: Pick then place in one command.

## Motion
- **move_to_pose <x> <y> <z> [roll pitch yaw]**: Move end-effector to an absolute \
world-frame pose (meters / radians).
- **move_to_joints <j1, j2, ..., j7>**: Move to a joint configuration (radians).
- **go_home**: Move to the home/observe position.
- **go_init**: Return to the startup position. Use after pick/place as a safe resting pose.

## Gripper
- **open_gripper / close_gripper / set_gripper**: Direct gripper control.

## Status & Recovery
- **get_robot_state**: Current joint positions, end-effector pose, and gripper state.
- **get_scene_info**: Full robot state, detected objects, and scene overview.
- **reset**: Clear a FAULT state and return to IDLE. Available as both a skill and RPC.
- **clear_perception_obstacles**: Remove detected obstacles from the planning world. \
Use when planning fails with COLLISION_AT_START.

# Choosing look vs scan_objects
- "what can you see?" / "what's there?" → **look** (instant, no movement)
- "scan the scene" / before pick-and-place → **scan_objects** (thorough, moves arm)
- If objects were ALREADY detected by a previous look, do NOT scan again — just proceed.

# Rules
- Use the EXACT object name from detection output. Do NOT substitute similar names \
(e.g. if detection says "spray can", do not use "grinder").
- "drop it in/on [object]" → use **drop_on**. "place it at [coords]" → use **place**.
- "bring it back" → pick, then **go_init**. Do NOT place randomly.
- "bring it to me" / "hand it over" → pick, then move toward user (≈ X=0, Y=0.5).
- NEVER open the gripper while holding an object unless the user asks or you are \
executing place/drop_on. The gripper stays closed during movement.
- After pick or place, return to init with **go_init** unless another action follows.

# Coordinate System
World frame (meters): X = forward, Y = left, Z = up. Z = 0 is robot base.
Typical working area: X 0.3-0.7, Y -0.5 to 0.5, Z 0.05-0.5.

# Error Recovery
If planning fails with COLLISION_AT_START: call **clear_perception_obstacles**, then \
**reset**, then retry.
"""

xarm_perception_agent = autoconnect(
    xarm_perception,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=_MANIPULATION_AGENT_SYSTEM_PROMPT),
)


# Sim perception: MujocoSimModule owns the MujocoEngine and publishes both
# camera streams and joint state via shared memory.
# ShmMujocoAdapter attaches to the same SHM buffers by MJCF path.

from dimos.robot.catalog.ufactory import XARM7_SIM_PATH
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

_xarm7_sim_cfg = _catalog_xarm7(
    name="arm",
    adapter_type="sim_mujoco",
    address=str(XARM7_SIM_PATH),
    add_gripper=True,
    pitch=math.radians(45),
    tf_extra_links=["link7"],
    home_joints=[0.0, 0.0, 0.0, 0.0, 0.0, -0.7, 0.0],
    pre_grasp_offset=0.05,
)

xarm_perception_sim = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[_xarm7_sim_cfg.to_robot_model_config()],
        planning_timeout=10.0,
        enable_viz=True,
    ),
    MujocoSimModule.blueprint(
        address=str(XARM7_SIM_PATH),
        headless=False,
        dof=7,
        camera_name="wrist_camera",
        base_frame_id="link7",
    ),
    ObjectSceneRegistrationModule.blueprint(target_frame="world"),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm7_sim_cfg.to_hardware_component()],
        tasks=[_xarm7_sim_cfg.to_task_config()],
    ),
    RerunBridgeModule.blueprint(),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


xarm_perception_sim_agent = autoconnect(
    xarm_perception_sim,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=_MANIPULATION_AGENT_SYSTEM_PROMPT),
)


# ============================================================================
# XArm6 perception — memory2-native pipeline (open-vocab VLM, no tracker)
# ============================================================================
# Mirrors the xArm7 xarm_perception blueprint but uses the new memory2-native
# perception modules (RGBDCameraRecorder + LazyPerceptionModule) in place of
# ObjectSceneRegistrationModule. RGBDCameraRecorder also runs the continuous
# CLIP embed pipeline (no separate SemanticSearch module — see spec.py for the
# "why one module" reasoning). The PickAndPlaceModule API is unchanged — its
# `objects: In[list[Object]]` port is now fed by LazyPerceptionModule.objects.
#
# TF chain: world → link6 (ManipulationModule) → camera_link (RealSense)
# Usage: chain coordinator + agent in ONE dimos invocation. dimos's CLI takes
# multiple blueprint names and autoconnect()s them into one process — separate
# invocations conflict on dimos's internal port reservation (check_port_conflicts).
#   XARM6_IP=<ip> OPENAI_API_KEY=<key> dimos run coordinator-xarm6 xarm6-perception-agent
# Without the coordinator the arm won't actually move (joint_state stream has no
# producer otherwise).
#
# Camera transform: PLACEHOLDER reusing xArm7 values. Replace with measured
# hand-eye calibration for the xArm6 mount on first hardware run if positions
# returned by find_objects are off by a consistent offset.
_XARM6_PERCEPTION_CAMERA_TRANSFORM = Transform(
    translation=Vector3(x=0.06693724, y=-0.0309563, z=0.00691482),
    rotation=Quaternion(0.70513398, 0.00535696, 0.70897578, -0.01052180),  # xyzw
)

_xarm6_perception_cfg = _catalog_xarm6(
    name="arm",
    adapter_type="xarm" if global_config.xarm6_ip else "mock",
    address=global_config.xarm6_ip,
    pitch=math.radians(0),
    add_gripper=True,
    tf_extra_links=["link6"],
)

def _convert_color_camera_info(ci: CameraInfo):
    return ci.to_rerun(
        image_topic="world/color_image",
        optical_frame="camera_color_optical_frame",
    )

def _convert_depth_camera_info(ci: CameraInfo):
    return ci.to_rerun(
        image_topic="world/depth_image",
        optical_frame="camera_depth_optical_frame",
    )

xarm6_perception = (
    autoconnect(
        PickAndPlaceModule.blueprint(
            robots=[_xarm6_perception_cfg.to_robot_model_config()],
            planning_timeout=10.0,
            enable_viz=True,
            floor_z=-0.02,
        ),
        RealSenseCamera.blueprint(
            base_frame_id="link6",
            base_transform=_XARM6_PERCEPTION_CAMERA_TRANSFORM,
        ),
        # Memory2-native perception pipeline — replaces ObjectSceneRegistrationModule.
        # RGBDCameraRecorder records color/depth/intrinsics AND continuously
        # CLIP-embeds color frames into color_image_embedded.
        # db_path is xArm6-specific to silo from other memory2 users and to avoid
        # picking up stale embeddings from a different camera mount.
        # Resume-if-exists: delete recording_xarm6.db manually for a clean slate.
        RGBDCameraRecorder.blueprint(db_path="recording_xarm6.db"),
        LazyPerceptionModule.blueprint(db_path="recording_xarm6.db"),
        vis_module(
            global_config.viewer,
            rerun_config={
                "visual_override": {
                    "world/camera_info": _convert_color_camera_info,
                    "world/depth_camera_info": _convert_depth_camera_info,
                }
            },
        ),
    )
    .transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        }
    )
    .global_config(n_workers=4)
)


# XArm6 perception + LLM agent. Three perception skills (find_objects,
# find_objects_near, recall) replace the previous look / scan_objects from
# the tracker-based pipeline. The agent reads the (seen Ns ago) timestamp
# in each response and decides whether to re-query before acting.
# Usage: chain coordinator + agent in ONE dimos invocation. dimos's CLI takes
# multiple blueprint names and autoconnect()s them into one process — separate
# invocations conflict on dimos's internal port reservation (check_port_conflicts).
#   XARM6_IP=<ip> OPENAI_API_KEY=<key> dimos run coordinator-xarm6 xarm6-perception-agent
# Without the coordinator the arm won't actually move (joint_state stream has no
# producer otherwise).
_MANIPULATION_AGENT_SYSTEM_PROMPT_XARM6 = """\
You are a robotic manipulation assistant controlling an xArm6 robot arm with an \
eye-in-hand RealSense camera and a gripper.

# Skills

## Perception
- **find_objects <prompt>**: Find objects matching the prompt in the current scene. \
Returns 3D positions and a "(seen Ns ago)" timestamp so you can judge freshness. \
Prompt is any natural language — use spatial qualifiers ("the cup on the right") \
or attributes ("the red mug") to disambiguate when multiple instances exist. \
Example: "what cups are there?" → find_objects("cup"); "find the red mug" → \
find_objects("red mug").

- **find_objects_near <prompt> <x> <y> <z> [radius]**: Same as find_objects but \
narrows to frames captured when the camera was within `radius` meters of (x, y, z). \
Use when you know which workspace area to scan — cheaper than full-scene query. \
Example: "find a screwdriver near the workbench at 1.0, 0.5, 0.8" → \
find_objects_near("screwdriver", 1.0, 0.5, 0.8, 0.5).

- **recall <name>**: Where did I last see something matching <name>? Cheaper than \
find_objects (no VLM, returns camera pose at the matching frame plus timestamp), \
across full session history including previous process runs. Use for "where was the \
cup earlier?" / "have I ever seen a wrench?" type questions.

## Pick & Place
- **pick <object_name>**: Pick up an object that find_objects most recently \
detected. Use the EXACT name from find_objects output. When multiple instances \
match, add a spatial or attribute qualifier in your find_objects prompt ("cup on \
the right") so VLM returns just one — then pick operates unambiguously.
- **place <x> <y> <z>**: Place a held object at explicit world-frame coordinates. \
Example: "place it at 0.4, 0.3, 0.1"
- **drop_on <object_name>**: Drop a held object onto another detected object. \
Automatically compensates for camera occlusion. Example: "drop it in the bowl".
- **place_back**: Return a held object to its original pick position.
- **pick_and_place <object_name> <x> <y> <z>**: Pick then place in one command.

## Motion
- **move_to_pose <x> <y> <z> [roll pitch yaw]**: Move end-effector to an absolute \
world-frame pose (meters / radians).
- **move_to_joints <j1, j2, ..., j6>**: Move to a 6-DOF joint configuration (radians).
- **go_home**: Move to the home/observe position.
- **go_init**: Return to startup position. Use after pick/place as a safe rest pose.

## Gripper
- **open_gripper / close_gripper / set_gripper**: Direct gripper control.

## Status & Recovery
- **get_robot_state**: Current joint positions, end-effector pose, gripper state.
- **get_scene_info**: Latest detection snapshot (from the most recent find_objects call).
- **reset**: Clear a FAULT state and return to IDLE.
- **clear_perception_obstacles**: Remove detected obstacles from the planning world. \
Use when planning fails with COLLISION_AT_START.

# Freshness and re-querying
- Every find_objects / find_objects_near response includes "(seen Ns ago)".
- A few seconds old → fresh, safe to act on.
- Minutes old → scene may have changed; re-query before acting.
- If you just picked or placed an object, the scene changed — re-query before the \
next action.

# Disambiguation (multiple instances)
- If find_objects returns multiple positions, either:
  1. Re-query with a spatial/attribute qualifier ("cup on the right", "red cup"), or
  2. Read the positions yourself and choose by reasoning (e.g., "right" = larger Y; \
"closest" = smallest distance from gripper).
- Don't rely on identity across calls — find_objects always reflects the most recent \
detection. The same name in two calls may be the same physical object or different ones.

# Rules
- Use the EXACT object name from find_objects output. Do NOT substitute similar names.
- "drop it in/on [object]" → use **drop_on**. "place it at [coords]" → use **place**.
- "bring it back" → pick, then **go_init**.
- "bring it to me" / "hand it over" → pick, then move toward user (≈ X=0, Y=0.5).
- NEVER open the gripper while holding an object unless executing place / drop_on / \
when the user explicitly asks.
- After pick or place, return to init with **go_init** unless another action follows.

# Coordinate System
World frame (meters): X = forward, Y = left, Z = up. Z = 0 is robot base.
Typical working area: X 0.3–0.7, Y −0.5 to 0.5, Z 0.05–0.5.

# Error Recovery
If planning fails with COLLISION_AT_START: call **clear_perception_obstacles**, then \
**reset**, then retry.
"""

xarm6_perception_agent = autoconnect(
    xarm6_perception,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=_MANIPULATION_AGENT_SYSTEM_PROMPT_XARM6),
)


__all__ = [
    "dual_xarm6_planner",
    "xarm6_perception",
    "xarm6_perception_agent",
    "xarm6_planner_only",
    "xarm7_planner_coordinator",
    "xarm7_planner_coordinator_agent",
    "xarm_perception",
    "xarm_perception_agent",
    "xarm_perception_sim",
    "xarm_perception_sim_agent",
]
