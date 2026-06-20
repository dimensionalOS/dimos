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

import math
from pathlib import Path
from typing import Any

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.control.blueprints._hardware import XARM7_SIM_PATH, manipulator
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.data import LfsPath
from dimos.visualization.rerun.bridge import RerunBridgeModule

XARM_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("right_inner_knuckle", "right_outer_knuckle"),
    ("left_inner_knuckle", "left_outer_knuckle"),
    ("right_inner_knuckle", "right_finger"),
    ("left_inner_knuckle", "left_finger"),
    ("left_finger", "right_finger"),
    ("left_outer_knuckle", "right_outer_knuckle"),
    ("left_inner_knuckle", "right_inner_knuckle"),
    ("left_outer_knuckle", "right_finger"),
    ("right_outer_knuckle", "left_finger"),
    ("xarm_gripper_base_link", "left_inner_knuckle"),
    ("xarm_gripper_base_link", "right_inner_knuckle"),
    ("xarm_gripper_base_link", "left_finger"),
    ("xarm_gripper_base_link", "right_finger"),
    ("link6", "xarm_gripper_base_link"),
    ("link6", "left_outer_knuckle"),
    ("link6", "right_outer_knuckle"),
]

_XARM_MODEL_PATH = LfsPath("xarm_description") / "urdf/xarm_device.urdf.xacro"
_XARM_PACKAGE_PATHS: dict[str, Path] = {"xarm_description": LfsPath("xarm_description")}


def _base_pose(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x=x, y=y, z=z),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


def _coordinator_joint_mapping(
    name: str,
    dof: int,
    *,
    joint_prefix: str | None = None,
) -> dict[str, str]:
    prefix = f"{name}/" if joint_prefix is None else joint_prefix
    if not prefix:
        return {}
    return {f"{prefix}joint{i}": f"joint{i}" for i in range(1, dof + 1)}


def _make_xarm_model_config(
    name: str,
    dof: int,
    *,
    add_gripper: bool = True,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
    z_offset: float = 0.0,
    pitch: float = 0.0,
    joint_prefix: str | None = None,
    coordinator_task_name: str | None = None,
    tf_extra_links: list[str] | None = None,
    home_joints: list[float] | None = None,
    pre_grasp_offset: float = 0.10,
) -> RobotModelConfig:
    xacro_args = {
        "dof": str(dof),
        "limited": "true",
        "attach_xyz": f"{x_offset} {y_offset} {z_offset}",
        "attach_rpy": f"0 {pitch} 0",
    }
    if add_gripper:
        xacro_args["add_gripper"] = "true"

    return RobotModelConfig(
        name=name,
        model_path=_XARM_MODEL_PATH,
        base_pose=_base_pose(x_offset, y_offset, z_offset),
        joint_names=[f"joint{i}" for i in range(1, dof + 1)],
        end_effector_link="link_tcp" if add_gripper else f"link{dof}",
        base_link="link_base",
        package_paths=_XARM_PACKAGE_PATHS,
        xacro_args=xacro_args,
        auto_convert_meshes=True,
        collision_exclusion_pairs=(XARM_GRIPPER_COLLISION_EXCLUSIONS if add_gripper else []),
        joint_name_mapping=_coordinator_joint_mapping(
            name,
            dof,
            joint_prefix=joint_prefix,
        ),
        coordinator_task_name=coordinator_task_name or f"traj_{name}",
        gripper_hardware_id=name if add_gripper else None,
        tf_extra_links=tf_extra_links or [],
        home_joints=home_joints or [0.0] * dof,
        pre_grasp_offset=pre_grasp_offset,
    )


def _make_xarm6_model_config(
    name: str = "arm",
    **kwargs: Any,
) -> RobotModelConfig:
    return _make_xarm_model_config(name, 6, **kwargs)


def _make_xarm7_model_config(
    name: str = "arm",
    **kwargs: Any,
) -> RobotModelConfig:
    return _make_xarm_model_config(name, 7, **kwargs)


xarm6_planner_only = ManipulationModule.blueprint(
    robots=[_make_xarm6_model_config(name="arm")],
    planning_timeout=10.0,
    visualization={"backend": "meshcat"},
)


dual_xarm6_planner = ManipulationModule.blueprint(
    robots=[
        _make_xarm6_model_config(name="left_arm", y_offset=0.5),
        _make_xarm6_model_config(name="right_arm", y_offset=-0.5),
    ],
    planning_timeout=10.0,
    visualization={"backend": "meshcat"},
)


# Single XArm7 planner + coordinator (uses real hardware when XARM7_IP is set)
# Usage: XARM7_IP=<ip> dimos run xarm7-planner-coordinator
_xarm7_hw = manipulator(
    "arm",
    7,
    adapter_type="xarm" if global_config.xarm7_ip else "mock",
    address=global_config.xarm7_ip,
    gripper=True,
)

xarm7_planner_coordinator = autoconnect(
    ManipulationModule.blueprint(
        robots=[_make_xarm7_model_config(name="arm", add_gripper=True)],
        planning_timeout=10.0,
        visualization={"backend": "meshcat"},
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm7_hw],
        tasks=[
            TaskConfig(
                name="traj_arm",
                type="trajectory",
                joint_names=_xarm7_hw.joints,
                priority=10,
            ),
        ],
    ),
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

xarm_perception = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[
            _make_xarm7_model_config(
                name="arm",
                add_gripper=True,
                pitch=math.radians(45),
                tf_extra_links=["link7"],
            )
        ],
        planning_timeout=10.0,
        visualization={"backend": "meshcat"},
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
).global_config(n_workers=4)


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
_xarm7_sim_home = [0.0, 0.0, 0.0, 0.0, 0.0, -0.7, 0.0]
_xarm7_sim_hw = manipulator(
    "arm",
    7,
    adapter_type="sim_mujoco",
    address=str(XARM7_SIM_PATH),
    gripper=True,
    home_joints=_xarm7_sim_home,
)

xarm_perception_sim = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[
            _make_xarm7_model_config(
                name="arm",
                add_gripper=True,
                pitch=math.radians(45),
                tf_extra_links=["link7"],
                home_joints=_xarm7_sim_home,
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
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm7_sim_hw],
        tasks=[
            TaskConfig(
                name="traj_arm",
                type="trajectory",
                joint_names=_xarm7_sim_hw.joints,
                priority=10,
            ),
        ],
    ),
    RerunBridgeModule.blueprint(),
)


xarm_perception_sim_agent = autoconnect(
    xarm_perception_sim,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=_MANIPULATION_AGENT_SYSTEM_PROMPT),
)


__all__ = [
    "dual_xarm6_planner",
    "xarm6_planner_only",
    "xarm7_planner_coordinator",
    "xarm7_planner_coordinator_agent",
    "xarm_perception",
    "xarm_perception_agent",
    "xarm_perception_sim",
    "xarm_perception_sim_agent",
]
