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

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.foxglove_bridge import FoxgloveBridge  # TODO: migrate to rerun
from dimos.utils.data import get_data


def _make_base_pose(
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
) -> PoseStamped:
    """Create a base pose with optional xyz offset and rpy orientation.

    Args:
        x, y, z: Position offset in meters
        roll, pitch, yaw: Orientation in radians (Euler angles)
    """
    return PoseStamped(
        position=Vector3(x=x, y=y, z=z),
        orientation=Quaternion.from_euler(Vector3(x=roll, y=pitch, z=yaw)),
    )


def _get_xarm_urdf_path() -> Path:
    """Get path to xarm URDF."""
    return get_data("xarm_description") / "urdf/xarm_device.urdf.xacro"


def _get_xarm_package_paths() -> dict[str, Path]:
    """Get package paths for xarm xacro resolution."""
    return {"xarm_description": get_data("xarm_description")}


def _get_piper_urdf_path() -> Path:
    """Get path to piper URDF."""
    return get_data("piper_description") / "urdf/piper_description.xacro"


def _get_piper_package_paths() -> dict[str, Path]:
    """Get package paths for piper xacro resolution."""
    return {"piper_description": get_data("piper_description")}


# Piper gripper collision exclusions (parallel jaw gripper)
# The gripper fingers (link7, link8) can touch each other and gripper_base
PIPER_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("gripper_base", "link7"),
    ("gripper_base", "link8"),
    ("link7", "link8"),
    ("link6", "gripper_base"),
]


# XArm gripper collision exclusions (parallel linkage mechanism)
# The gripper uses mimic joints where non-adjacent links can overlap legitimately
XARM_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    # Inner knuckle <-> outer knuckle (parallel linkage)
    ("right_inner_knuckle", "right_outer_knuckle"),
    ("left_inner_knuckle", "left_outer_knuckle"),
    # Inner knuckle <-> finger (parallel linkage)
    ("right_inner_knuckle", "right_finger"),
    ("left_inner_knuckle", "left_finger"),
    # Cross-finger pairs (mimic joint symmetry)
    ("left_finger", "right_finger"),
    ("left_outer_knuckle", "right_outer_knuckle"),
    ("left_inner_knuckle", "right_inner_knuckle"),
    # Outer knuckle <-> opposite finger
    ("left_outer_knuckle", "right_finger"),
    ("right_outer_knuckle", "left_finger"),
    # Gripper base <-> all moving parts (can touch at limits)
    ("xarm_gripper_base_link", "left_inner_knuckle"),
    ("xarm_gripper_base_link", "right_inner_knuckle"),
    ("xarm_gripper_base_link", "left_finger"),
    ("xarm_gripper_base_link", "right_finger"),
    # Arm link6 <-> gripper (attached via fixed joint, can touch)
    ("link6", "xarm_gripper_base_link"),
    ("link6", "left_outer_knuckle"),
    ("link6", "right_outer_knuckle"),
]


def _make_xarm6_config(
    name: str = "arm",
    y_offset: float = 0.0,
    joint_prefix: str = "",
    coordinator_task: str | None = None,
    add_gripper: bool = True,
) -> RobotModelConfig:
    """Create XArm6 robot config.

    Args:
        name: Robot name in Drake world
        y_offset: Y-axis offset for base pose (for multi-arm setups)
        joint_prefix: Prefix for joint name mapping (e.g., "left_" or "right_")
        coordinator_task: Task name for coordinator RPC execution
        add_gripper: Whether to add the xarm gripper
    """
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    joint_mapping = {f"{joint_prefix}{j}": j for j in joint_names} if joint_prefix else {}

    xacro_args: dict[str, str] = {
        "dof": "6",
        "limited": "true",
        "attach_xyz": f"0 {y_offset} 0",
    }
    if add_gripper:
        xacro_args["add_gripper"] = "true"

    return RobotModelConfig(
        name=name,
        urdf_path=_get_xarm_urdf_path(),
        base_pose=_make_base_pose(y=y_offset),
        joint_names=joint_names,
        end_effector_link="link_tcp" if add_gripper else "link6",
        base_link="link_base",
        package_paths=_get_xarm_package_paths(),
        xacro_args=xacro_args,
        collision_exclusion_pairs=XARM_GRIPPER_COLLISION_EXCLUSIONS if add_gripper else [],
        auto_convert_meshes=True,
        max_velocity=1.0,
        max_acceleration=2.0,
        joint_name_mapping=joint_mapping,
        coordinator_task_name=coordinator_task,
        home_joints=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )


def _make_xarm7_config(
    name: str = "arm",
    y_offset: float = 0.0,
    z_offset: float = 0.0,
    pitch: float = 0.0,
    joint_prefix: str = "",
    coordinator_task: str | None = None,
    add_gripper: bool = False,
    gripper_hardware_id: str | None = None,
    tf_extra_links: list[str] | None = None,
    scan_joints: list[list[float]] | None = None,
) -> RobotModelConfig:
    """Create XArm7 robot config.

    Args:
        name: Robot name in Drake world
        y_offset: Y-axis offset for base pose (for multi-arm setups)
        z_offset: Z-axis offset for base pose (e.g., table height)
        pitch: Base pitch angle in radians (e.g., tilted mount)
        joint_prefix: Prefix for joint name mapping (e.g., "left_" or "right_")
        coordinator_task: Task name for coordinator RPC execution
        add_gripper: Whether to add the xarm gripper
        gripper_hardware_id: Coordinator hardware ID for gripper control
        tf_extra_links: Additional links to publish TF for (e.g., ["link7"] for camera mount)
        scan_joints: List of joint configurations for multi-view scanning
    """
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    joint_mapping = {f"{joint_prefix}{j}": j for j in joint_names} if joint_prefix else {}

    xacro_args: dict[str, str] = {
        "dof": "7",
        "limited": "true",
        "attach_xyz": f"0 {y_offset} {z_offset}",
        "attach_rpy": f"0 {pitch} 0",
    }
    if add_gripper:
        xacro_args["add_gripper"] = "true"

    return RobotModelConfig(
        name=name,
        urdf_path=_get_xarm_urdf_path(),
        base_pose=_make_base_pose(y=y_offset, z=z_offset, pitch=pitch),
        joint_names=joint_names,
        end_effector_link="link_tcp" if add_gripper else "link7",
        base_link="link_base",
        package_paths=_get_xarm_package_paths(),
        xacro_args=xacro_args,
        collision_exclusion_pairs=XARM_GRIPPER_COLLISION_EXCLUSIONS if add_gripper else [],
        auto_convert_meshes=True,
        max_velocity=1.0,
        max_acceleration=2.0,
        joint_name_mapping=joint_mapping,
        coordinator_task_name=coordinator_task,
        gripper_hardware_id=gripper_hardware_id,
        tf_extra_links=tf_extra_links or [],
        # Home configuration: arm extended forward, elbow up (safe observe pose)
        home_joints=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        scan_joints=scan_joints,
    )


def _make_piper_config(
    name: str = "piper",
    y_offset: float = 0.0,
    joint_prefix: str = "",
    coordinator_task: str | None = None,
) -> RobotModelConfig:
    """Create Piper robot config.

    Args:
        name: Robot name in Drake world
        y_offset: Y-axis offset for base pose (for multi-arm setups)
        joint_prefix: Prefix for joint name mapping (e.g., "piper_")
        coordinator_task: Task name for coordinator RPC execution

    Note:
        Piper has 6 revolute joints (joint1-joint6) for the arm and 2 prismatic
        joints (joint7, joint8) for the parallel jaw gripper.
    """
    # Piper arm joints (6-DOF)
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    joint_mapping = {f"{joint_prefix}{j}": j for j in joint_names} if joint_prefix else {}

    return RobotModelConfig(
        name=name,
        urdf_path=_get_piper_urdf_path(),
        base_pose=_make_base_pose(y=y_offset),
        joint_names=joint_names,
        end_effector_link="gripper_base",  # End of arm, before gripper fingers
        base_link="arm_base",
        package_paths=_get_piper_package_paths(),
        xacro_args={},  # Piper xacro doesn't need special args
        collision_exclusion_pairs=PIPER_GRIPPER_COLLISION_EXCLUSIONS,
        auto_convert_meshes=True,
        max_velocity=1.0,
        max_acceleration=2.0,
        joint_name_mapping=joint_mapping,
        coordinator_task_name=coordinator_task,
        home_joints=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )


# Single XArm6 planner (standalone, no coordinator)
xarm6_planner_only = ManipulationModule.blueprint(
    robots=[_make_xarm6_config()],
    planning_timeout=10.0,
    enable_viz=True,
).transports(
    {
        ("joint_state", JointState): LCMTransport("/xarm/joint_states", JointState),
    }
)


# Dual XArm6 planner with coordinator integration
# Usage: Start with coordinator_dual_mock, then plan/execute via RPC
dual_xarm6_planner = ManipulationModule.blueprint(
    robots=[
        _make_xarm6_config(
            "left_arm", y_offset=0.5, joint_prefix="left_", coordinator_task="traj_left"
        ),
        _make_xarm6_config(
            "right_arm", y_offset=-0.5, joint_prefix="right_", coordinator_task="traj_right"
        ),
    ],
    planning_timeout=10.0,
    enable_viz=True,
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


# Single XArm7 planner + mock coordinator (standalone, no external coordinator needed)
# Usage: dimos run xarm7-planner-coordinator
xarm7_planner_coordinator = autoconnect(
    ManipulationModule.blueprint(
        robots=[_make_xarm7_config("arm", joint_prefix="arm_", coordinator_task="traj_arm")],
        planning_timeout=10.0,
        enable_viz=True,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[
            HardwareComponent(
                hardware_id="arm",
                hardware_type=HardwareType.MANIPULATOR,
                joints=make_joints("arm", 7),
                adapter_type="mock",
            ),
        ],
        tasks=[
            TaskConfig(
                name="traj_arm",
                type="trajectory",
                joint_names=[f"arm_joint{i + 1}" for i in range(7)],
                priority=10,
            ),
        ],
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

# Scan viewpoints for multi-view object scanning (xarm7 with 45° base pitch).
# Each entry is [j1, j2, j3, j4, j5, j6, j7] in radians.
# j1 = base yaw, j2 = shoulder lift, j5 = wrist pitch — these vary to sweep the workspace.
_XARM7_SCAN_JOINTS: list[list[float]] = [
    # Center view — looking straight ahead and down
    [-0.000, -1.2217, 0.000, 1.1345, 0.000, 1.0472, 0.000],
    # Left view
    [-1.3963, -1.2217, 0.7854, 2.0944, 0.3491, 1.5708, 1.5708],
    # Center high — looking from above
    [0.0, -1.7453, 0.0, 0.1745, 0.0, 0.4363, 0.0],
    # Right view
    [1.4835, -0.1745, -2.618, 1.5708, -1.7453, -1.6581, 0.5236],
    # Center high — looking from above
    [0.0, -1.7453, 0.0, 0.1745, 0.0, 0.4363, 0.0],
    # Center low — closer to the table
    [-0.000, -0.428, 0.000, 0.000, 0.000, -1.475, 0.000],
]

xarm_perception = (
    autoconnect(
        PickAndPlaceModule.blueprint(
            robots=[
                _make_xarm7_config(
                    "arm",
                    pitch=math.radians(45),
                    joint_prefix="arm_",
                    coordinator_task="traj_arm",
                    add_gripper=True,
                    gripper_hardware_id="arm",
                    tf_extra_links=["link7"],
                    scan_joints=_XARM7_SCAN_JOINTS,
                ),
            ],
            planning_timeout=10.0,
            enable_viz=True,
            safe_waypoint=[0.0, -1.7453, 0.0, 0.1745, 0.0, 0.4363, 0.0],
        ),
        RealSenseCamera.blueprint(
            base_frame_id="link7",
            base_transform=_XARM_PERCEPTION_CAMERA_TRANSFORM,
        ),
        ObjectSceneRegistrationModule.blueprint(target_frame="world"),
        FoxgloveBridge.blueprint(),  # TODO: migrate to rerun
    )
    .transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        }
    )
    .global_config(viewer="foxglove", n_workers=4)
)


# XArm7 perception + LLM agent for agentic manipulation
# Includes coordinator_xarm7 so everything runs in a single blueprint.
# Skills (pick, place, move_to_pose, etc.) auto-register with the agent's SkillCoordinator.
# Usage: dimos run xarm-perception-agent
_MANIPULATION_AGENT_SYSTEM_PROMPT = """\
You are a robotic manipulation assistant controlling an xArm7 robot arm.
ws://localhost:8765 LCM
-Foxglove Bridge
Available skills:
- get_robot_state: Get current joint positions, end-effector pose, and gripper state.
- look: Quick check of what objects are visible from the current camera position. \
Does NOT move the arm. Use when the user asks "what can you see?" or "what's in front of you?".
- scan_objects: Scan for objects — goes to init position, then refreshes detections. \
Use before pick-and-place, after a failed attempt, or when the user asks to scan.
- pick: Pick up an object by name. Use the EXACT object name from look/scan_objects output. \
When multiple objects share the same name, pass the object_id (shown in brackets like [id=abc12345]) \
to pick the specific one the user wants.
- place: Place a held object at EXPLICIT x, y, z coordinates.
- drop_on: Drop a held object on top of a detected object by name (e.g. "drop_on cup"). \
Use this when the user says "drop it in/on the [object]" — it auto-corrects for camera occlusion.
- place_back: Place a held object back at its original pick position.
- pick_and_place: Pick an object and place it at a target location.
- move_to_pose: Move end-effector to ABSOLUTE x, y, z (meters) with optional roll, pitch, yaw (radians).
- move_to_joints: Move to a joint configuration (comma-separated radians).
- open_gripper / close_gripper / set_gripper: Control the gripper.
- go_home: Move to the home/observe position.
- go_init: Return to the startup position.
- get_scene_info: Get full robot state, detected objects, and scene info.
- reset: Clear a FAULT state and return to IDLE.
- clear_perception_obstacles: Clear detected obstacles from the planning world. \
Use when planning fails with COLLISION_AT_START.

CHOOSING BETWEEN look AND scan_objects:
- "what can you see?" / "what's there?" → look (quick, no movement)
- "scan everything" / "what all objects are there?" / user explicitly asks to scan → scan_objects (thorough, moves arm)

CRITICAL RULES:
- If the user asks to pick/place an object that was ALREADY detected by a previous look, \
do NOT scan again. Just proceed with the pick/place using the already-detected object name.
- If the user explicitly asks you to scan, ALWAYS run scan_objects. Never refuse a direct request.
- When calling pick, use the EXACT name from the detection output. Do NOT substitute similar names \
(e.g. if detection says "spray can", use "spray can", not "grinder").
- When user says "drop it in/on [object]", use drop_on with the target object name. \
When user says "drop" at a specific location, use place with z_offset +0.1m.
- "bring it back" = pick the object, then go_init. Do NOT place it somewhere random.
- "bring it to me", "hand it over", "give it to me" = pick the object, then move toward the user \
(roughly X=0, Y=0.5). Only use place/place_back if the user specifies an explicit location.
COORDINATE SYSTEM (world frame, meters): X=forward, Y=left, Z=up. Z=0 is robot base.
User is sitting to the left roughly at X=0 Y=0.5, Z=0.05. Typical working area is X=0.3-0.7, Y=-0.5 to 0.5, Z=0.2-0.5.


ERROR RECOVERY: If planning fails with COLLISION_AT_START, call clear_perception_obstacles \
then reset, then retry. Detected objects may overlap the robot's current position.

After pick or place, return to init with go_init unless another action follows immediately.
Do NOT use the 'detect' or 'select' skills — use look or scan_objects instead.
"""

xarm_perception_agent = autoconnect(
    coordinator_xarm7,
    xarm_perception,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=_MANIPULATION_AGENT_SYSTEM_PROMPT),
)


__all__ = [
    "PIPER_GRIPPER_COLLISION_EXCLUSIONS",
    "XARM_GRIPPER_COLLISION_EXCLUSIONS",
    "dual_xarm6_planner",
    "xarm6_planner_only",
    "xarm7_planner_coordinator",
    "xarm7_planner_coordinator_agent",
    "xarm_perception",
    "xarm_perception_agent",
]
