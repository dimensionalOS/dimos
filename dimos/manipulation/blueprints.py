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

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.sim_presets import xarm7_mujoco_scene_preset
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.catalog.galaxea import (
    r1pro_arm as _catalog_r1pro_arm,
    r1pro_bimanual as _catalog_r1pro_bimanual,
)
from dimos.robot.catalog.ufactory import xarm6 as _catalog_xarm6, xarm7 as _catalog_xarm7
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

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
    add_gripper=True,
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


# ---------------------------------------------------------------------------
# Galaxea R1Pro (left arm) — planner + mock coordinator + Viser visualization.
# Visualization-only demo: open http://127.0.0.1:8095 to render the R1Pro, drag
# the end-effector gizmo, and Plan / Preview / Execute via the Viser panel.
# Usage: dimos run galaxea-viser-planner-coordinator
# ---------------------------------------------------------------------------
_r1pro_cfg = _catalog_r1pro_arm(
    "left",
    name="arm",
    adapter_type="mock",
    add_gripper=True,
)

_R1PRO_VISER_VIZ = {
    "backend": "viser",
    "host": "0.0.0.0",
    "port": 8095,
    "panel_enabled": True,
    "allow_plan_execute": True,
}

galaxea_viser_planner_coordinator = autoconnect(
    ManipulationModule.blueprint(
        robots=[_r1pro_cfg.to_robot_model_config()],
        planning_timeout=10.0,
        kinematics_name="pink",  # Pink differential IK: fast, smooth, deterministic
        visualization=_R1PRO_VISER_VIZ,
    ),
    ControlCoordinator.blueprint(
        # 50Hz (not 100) so the state monitor writes the shared Drake context half as
        # often, freeing the GIL/world-lock for the interactive gizmo IK eval. Still
        # smooth for trajectory execution.
        tick_rate=50.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_r1pro_cfg.to_hardware_component()],
        tasks=[_r1pro_cfg.to_task_config()],
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


# Galaxea R1Pro left arm + LLM agent (gpt-4o) driving base manipulation skills.
# No perception yet — move_to_pose / gripper / go_home over /human_input.
# Usage: dimos run galaxea-viser-planner-coordinator-agent
_R1PRO_AGENT_SYSTEM_PROMPT = """\
You are a robotic manipulation assistant controlling the LEFT arm of a Galaxea
R1Pro (a 7-DOF arm with a parallel-jaw gripper mounted on a torso).

Available skills:
- get_robot_state: Get current joint positions, end-effector pose, and gripper state.
- move_to_pose: Move end-effector to ABSOLUTE x, y, z (meters) with optional roll, pitch, yaw (radians).
- move_to_joints: Move to a joint configuration (comma-separated radians).
- open_gripper / close_gripper / set_gripper: Control the gripper (open ~0.04 m, closed 0.0).
- go_home / go_init: Return to the home / startup position.
- reset: Clear a FAULT state and return to IDLE. Use this when a motion fails.

COORDINATE SYSTEM (world frame, meters):
- X axis = forward (away from the robot base), Y axis = left, Z axis = up.
- The left arm mounts on the torso at roughly Z = 0.85 m, on the +Y side; its
  natural working volume is about X = 0.1-0.5, Y = 0.1-0.5, Z = 0.6-1.0.

CRITICAL WORKFLOW for relative movement (e.g. "move 20cm forward"):
1. Call get_robot_state to read the CURRENT end-effector pose.
2. Add the requested offset to the current position and call move_to_pose with the
   ABSOLUTE target. Never pass only the offset.

ERROR RECOVERY: if a motion fails or the state becomes FAULT, call reset before retrying.
"""

galaxea_viser_planner_coordinator_agent = autoconnect(
    galaxea_viser_planner_coordinator,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=_R1PRO_AGENT_SYSTEM_PROMPT),
)


# Galaxea R1Pro BIMANUAL: both arms (14 DOF) as one planning unit + mock coordinator
# + viser. Multi-target IK (Pink) drives both arm tips at once; one synchronized
# 14-joint path. Usage: dimos run galaxea-viser-bimanual
_r1pro_dual_cfg = _catalog_r1pro_bimanual(name="arm", adapter_type="mock")

# One gizmo per arm tip; both are solved together via multi-target IK.
_R1PRO_BIMANUAL_VIZ = {
    **_R1PRO_VISER_VIZ,
    "pose_target_links": ["left_arm_link7", "right_arm_link7"],
}

galaxea_viser_bimanual = autoconnect(
    ManipulationModule.blueprint(
        robots=[_r1pro_dual_cfg.to_robot_model_config()],
        planning_timeout=10.0,
        # RoboPlan native planner + world: pinocchio/HPP-FCL collision (no Drake context
        # clone), far faster than rrt_connect for the 14-joint bimanual problem.
        # Requires: uv pip install "roboplan @ git+https://github.com/TomCC7/roboplan.git"
        world_backend="roboplan",
        planner_name="roboplan",
        kinematics_name="pink",
        visualization=_R1PRO_BIMANUAL_VIZ,
    ),
    ControlCoordinator.blueprint(
        tick_rate=50.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_r1pro_dual_cfg.to_hardware_component()],
        tasks=[_r1pro_dual_cfg.to_task_config()],
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


# Light-robot (xArm7) control for the viser gizmo responsiveness comparison vs the
# heavy R1Pro — identical viser/pink/coordinator wiring, just a 7-link arm with tiny
# meshes. If this gizmo tracks live and R1Pro doesn't, the bottleneck is robot weight
# (render/update of the 28-joint/46-link body), not the viser code or the IK.
# Usage: dimos run xarm7-viser-planner-coordinator
_xarm7_viser_cfg = _catalog_xarm7(name="arm", adapter_type="mock", add_gripper=True)

xarm7_viser_planner_coordinator = autoconnect(
    ManipulationModule.blueprint(
        robots=[_xarm7_viser_cfg.to_robot_model_config()],
        planning_timeout=10.0,
        kinematics_name="pink",
        visualization=_R1PRO_VISER_VIZ,
    ),
    ControlCoordinator.blueprint(
        tick_rate=50.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm7_viser_cfg.to_hardware_component()],
        tasks=[_xarm7_viser_cfg.to_task_config()],
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
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

_xarm7_sim_preset = xarm7_mujoco_scene_preset()
_xarm7_sim_cfg = _catalog_xarm7(
    name="arm",
    adapter_type="sim_mujoco",
    add_gripper=True,
    pitch=math.radians(45),
    tf_extra_links=["link7"],
    pre_grasp_offset=0.05,
    **_xarm7_sim_preset.robot_config_kwargs,
)


def _xarm_perception_rerun_blueprint():
    """Split layout: wrist-camera feeds beside the 3D scene.

    Left column = what the eye-in-hand camera actually sees (color + depth) so you
    can check whether an object is in frame; right = the 3D world (arm, TF frames,
    detections, camera). Replaces the default 3D-only layout which hides the camera.
    """
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image", name="Wrist cam (color)"),
                rrb.Spatial2DView(origin="world/depth_image", name="Wrist cam (depth)"),
            ),
            rrb.Spatial3DView(
                origin="world",
                name="Scene",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.0)),
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


xarm_perception_sim = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[_xarm7_sim_cfg.to_robot_model_config()],
        planning_timeout=10.0,
        # Meshcat in-process viz is redundant with the RerunBridge below and runs a
        # second 10Hz scene server per episode; off for the benchmark to cut CPU/mem.
        enable_viz=False,
    ),
    MujocoSimModule.blueprint(
        **_xarm7_sim_preset.mujoco_module_kwargs,
        headless=False,
        dof=7,
        camera_name="wrist_camera",
        base_frame_id="link7",
        # The perception module builds its own clouds from depth; nothing subscribes
        # to the sim's pointcloud port (GraspGen is unused — pick uses heuristics).
        # Disabling drops a 5Hz voxelized full-office-scene cloud (the main mem/CPU sink).
        enable_pointcloud=False,
        fps=5,
    ),
    # Match the real-hardware xarm_perception thresholds: promotion reachable in a
    # short warm-up (3 vs 6 frames), tighter dedup so neighbours don't merge, and a
    # 1m range cap to drop far office-background detections.
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        distance_threshold=0.08,
        min_detections_for_permanent=3,
        max_distance=1.0,
        use_aabb=True,
        max_obstacle_width=0.06,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm7_sim_cfg.to_hardware_component()],
        tasks=[_xarm7_sim_cfg.to_task_config()],
    ),
    RerunBridgeModule.blueprint(
        blueprint=_xarm_perception_rerun_blueprint,
        memory_limit="5%",
        max_hz={"world/color_image": 2.0, "world/depth_image": 2.0, "world/pointcloud": 2.0},
    ),
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
