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

"""Unitree G1 GR00T whole-body-control blueprint — MuJoCo sim.

Sim counterpart to ``unitree_g1_groot_wbc``.  Same coordinator + tasks,
swap the real DDS adapter for the MuJoCo sim adapter, drop the
real-hw safety ritual (no operator → auto-arm, no ramp, no dry-run),
add the viser viewer + splat-rendered head camera so perception /
memory / agents can consume the same wire format real cameras produce.

Architecture:
    dashboard WASD ──▶ WebsocketVisModule ──▶ LCM /g1/cmd_vel
                                                       │
                              coordinator twist_command ──▶ GrootWBCTask
                                                       │
    ControlCoordinator ──joint_state, odom──▶ LCM
                              │
                              ▼
                  ViserRenderModule (browser at :8082)
                              │
                              ▼
                  SplatCameraModule ──▶ /splat/color_image
                                        /splat/camera_info

Splat + alignment YAML are pulled via the standard Git-LFS data flow:
``get_data("dimos_office")`` triggers a one-time pull of
``data/.lfs/dimos_office.tar.gz`` and decompresses to
``data/dimos_office/``.  YAML schema: ``SplatAlignment`` in
``dimos/visualization/viser/splat.py``.  If the pull fails (no
Git-LFS, offline, etc.) the viser modules are skipped and the rest
of the blueprint runs unchanged.

Usage:
    dimos run unitree-g1-groot-wbc-sim
"""

from __future__ import annotations

import os
from pathlib import Path

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.agents.skills.sim_g1_locomotion import G1SimLocomotion
from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.core.transport import LCMTransport
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Bool import Bool as DimosBool
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.patrolling.module import PatrollingModule
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.perception.experimental.temporal_memory.temporal_memory import TemporalMemory
from dimos.perception.object_tracker import ObjectTracking
from dimos.perception.perceive_loop_skill import PerceiveLoopSkill
from dimos.perception.spatial_perception import SpatialMemory
from dimos.robot.catalog.g1 import g1_left_arm
from dimos.robot.unitree.g1.blueprints.basic._groot_wbc_common import (
    ARM_DEFAULT_POSE,
    G1_GROOT_KD,
    G1_GROOT_KP,
    g1_arms,
    g1_joints,
    g1_legs_waist,
)
from dimos.robot.unitree.g1.g1_manipulation import G1ManipulationModule
from dimos.robot.unitree.g1.system_prompt import G1_SYSTEM_PROMPT
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger
from dimos.visualization.viser import SplatCameraModule, ViserRenderModule
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

logger = setup_logger()


class G1MemoryConfig(RecorderConfig):
    db_path: str | Path = "recording_g1.db"


class G1Memory(Recorder):
    """G1 ``Recorder`` subclass — records the visual + spatial streams.

    Mirrors ``Go2Memory`` shape so memory2's existing playback / search
    tooling works on G1 recordings without special-casing.
    """

    color_image: In[Image]
    lidar: In[PointCloud2]
    config: G1MemoryConfig


# MJCF the GR00T policies were trained against — torque actuators, the
# subprocess (and now the in-process MujocoEngine) computes PD itself.
_MJCF_PATH = "data/mujoco_sim/g1_gear_wbc.xml"

# Manipulation: the G1's left arm as a 7-DOF stationary manipulator
# rooted at torso_link.  Drake-driven IK + RRT plans trajectories;
# the coordinator's "trajectory" task on the same joint subset
# executes them.  See dimos/robot/catalog/g1.py for the details.
_g1_left_arm_cfg = g1_left_arm()

_g1_coordinator = (
    ControlCoordinator.blueprint(
        tick_rate=500.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[
            HardwareComponent(
                hardware_id="g1",
                hardware_type=HardwareType.WHOLE_BODY,
                joints=g1_joints,
                # In-process engine via MujocoSimModule — adapter and
                # engine share state through SHM keyed on the MJCF path.
                adapter_type="sim_mujoco_g1",
                address=_MJCF_PATH,
                domain_id=0,
                auto_enable=True,
                wb_config=WholeBodyConfig(kp=tuple(G1_GROOT_KP), kd=tuple(G1_GROOT_KD)),
            ),
        ],
        tasks=[
            TaskConfig(
                name="groot_wbc",
                type="groot_wbc",
                joint_names=g1_legs_waist,
                priority=50,
                model_path=os.getenv("GROOT_MODEL_DIR", str(get_data("groot"))),
                hardware_id="g1",
                auto_start=True,
                # Sim convenience: the MuJoCo subprocess holds the MJCF
                # init pose until the first command arrives, so no
                # operator-arm ritual or ramp is needed.  Dry-run off.
                auto_arm=True,
                auto_dry_run=False,
                default_ramp_seconds=0.0,
            ),
            # Right arm: servo-hold at the default pose (manipulation
            # only drives the left arm in this blueprint).
            TaskConfig(
                name="servo_right_arm",
                type="servo",
                joint_names=g1_arms[7:],  # right arm joints (7)
                priority=10,
                default_positions=ARM_DEFAULT_POSE[7:],
                auto_start=True,
            ),
            # Left arm: trajectory follower driven by ManipulationModule.
            # Higher priority than servo so when manipulation is idle the
            # arm just dangles under WBC kp/kd damping; when active, the
            # planned trajectory wins.
            _g1_left_arm_cfg.task_config,
        ],
    )
    .transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
            ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
            ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
            ("activate", DimosBool): LCMTransport("/g1/activate", DimosBool),
            ("dry_run", DimosBool): LCMTransport("/g1/dry_run", DimosBool),
        }
    )
    .global_config(
        # global_config.simulation gates real-vs-sim adapter selection
        # in some upstream blueprints; harmless to set even though the
        # in-process engine + adapter pair don't read robot_model anymore.
        robot_model="unitree_g1",
    )
)

# In-process MuJoCo engine.  Owns the MujocoEngine (single thread, no
# subprocess) and publishes joint state + IMU into SHM for the WB
# adapter, plus camera/lidar/pointcloud streams for downstream consumers.
# The G1 GR00T MJCF has no head_camera so we point camera_name at the
# torso lidar instead — a separate splat camera handles RGB perception.
_g1_engine = MujocoSimModule.blueprint(
    address=_MJCF_PATH,
    headless=False,
    dof=29,
    # SplatCameraModule is the canonical RGB source for this sim; suppress
    # MujocoSimModule's own RGB to keep /splat/color_image single-publisher
    # (autoconnect merges any module's `color_image` Out into one shared
    # channel, so per-module transports can't separate them — must gate
    # the publish itself).
    enable_color=False,
    enable_depth=True,
    enable_pointcloud=True,
    pointcloud_fps=2.0,
    camera_name="lidar_front_camera",
    # G1 GR00T MJCF references meshes by bare filename (menagerie convention);
    # without the legacy asset injection MjModel.from_xml_path can't find them.
    inject_legacy_assets=True,
).transports(
    {
        # ShmMujocoG1WholeBodyAdapter.read_odom returns None (no SHM
        # base-pose channel); MujocoSimModule publishes the floating
        # base pose directly so the viser viewer + nav stack see it.
        ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
        # Bridge pointcloud → /lidar topic so downstream consumers
        # (VoxelGridMapper, G1Memory) with ``lidar`` In ports can
        # subscribe by topic regardless of port-name mismatch.
        ("pointcloud", PointCloud2): LCMTransport("/lidar", PointCloud2),
    }
)

# Embedded shell at http://localhost:7779/ — WASD teleop, viser iframe,
# camera MJPEG.  TODO(perf): move /splat/color_image to JpegShmTransport
# across the splat → ws-vis → memory chain to drop LCM JSON overhead.
_g1_ws_vis = WebsocketVisModule.blueprint().transports(
    {
        ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        ("activate", DimosBool): LCMTransport("/g1/activate", DimosBool),
        ("dry_run", DimosBool): LCMTransport("/g1/dry_run", DimosBool),
        ("color_image", Image): LCMTransport("/splat/color_image", Image),
    },
)

# Splat viewer + splat-rendered head camera, gated on the splat asset
# being pullable.  When the LFS pull fails the modules are silently
# skipped and the rest of the sim runs as a flat-floor MuJoCo viewer.
_viser_modules: tuple = ()
try:
    _splat_dir = get_data("dimos_office")
    _splat_path = _splat_dir / "dimos_office.ply"
    _alignment_yaml = _splat_dir / "dimos_office.yaml"
except Exception as e:
    logger.warning(f"Splat asset unavailable: {e}; viser viewer + splat camera disabled")
    _splat_path = None
if _splat_path is not None and _splat_path.exists():
    _g1_viser = ViserRenderModule.blueprint(
        splat_path=str(_splat_path),
        mjcf_path=_MJCF_PATH,
        alignment_yaml=str(_alignment_yaml) if _alignment_yaml.exists() else None,
        port=8082,
    ).transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
        },
    )
    _g1_splat_cam = SplatCameraModule.blueprint(
        splat_path=str(_splat_path),
        mjcf_path=_MJCF_PATH,
        alignment_yaml=str(_alignment_yaml) if _alignment_yaml.exists() else None,
        render_hz=10.0,
    ).transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
            ("color_image", Image): LCMTransport("/splat/color_image", Image),
            ("camera_info", CameraInfo): LCMTransport("/splat/camera_info", CameraInfo),
        },
    )
    _viser_modules = (_g1_viser, _g1_splat_cam)

# Mapping + planning + memory + telemetry layered on top of the base
# sim.  The base sim publishes pointcloud → /lidar (see the engine
# transports above) and color_image → /splat/color_image; downstream
# subscribers bind to those topics by name.
_g1_perception_stack = (
    # Mapping + planning
    VoxelGridMapper.blueprint().transports(
        {
            ("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2),
        }
    ),
    CostMapper.blueprint(),
    ReplanningAStarPlanner.blueprint(),
    # Visual perception (object detection + tracking, semantic spatial memory)
    SpatialMemory.blueprint(),
    ObjectTracking.blueprint(frame_id="camera_link"),
    # Episode recording (memory2)
    G1Memory.blueprint().transports(
        {
            ("color_image", Image): LCMTransport("/splat/color_image", Image),
            ("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2),
        }
    ),
)

# Agentic stack — Go2 parity minus xArm and minus PersonFollow.
# UnitreeG1SkillContainer is still skipped (its move()/arm-gesture/mode
# skills need G1ConnectionSpec which our in-process engine doesn't
# provide); G1SimLocomotion gives the agent move() via /cmd_vel instead.
# Vision is via PerceiveLoopSkill (Qwen API), memory introspection via
# TemporalMemory.query().  Requires OPENAI_API_KEY (LLM + TTS) and
# ALIBABA_API_KEY (Qwen-VL for navigate_with_text + look_out_for).
_g1_agentic_stack = (
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=G1_SYSTEM_PROMPT),
    G1SimLocomotion.blueprint().transports(
        {
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        }
    ),
    NavigationSkillContainer.blueprint(),
    SpeakSkill.blueprint(),
    # Manipulation — Drake IK + RRT planner driving the G1 left arm via
    # the coordinator's trajectory task.  Subscribes to coordinator
    # joint_state for live state sync.  Meshcat viz off in this
    # composed sim (we already have viser as the live 3D view).
    G1ManipulationModule.blueprint(
        robots=[_g1_left_arm_cfg.robot_model_config],
        planning_timeout=10.0,
        enable_viz=False,
    ).transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
        }
    ),
    PerceiveLoopSkill.blueprint().transports(
        {
            ("color_image", Image): LCMTransport("/splat/color_image", Image),
        }
    ),
    TemporalMemory.blueprint(
        new_memory=global_config.new_memory,
        # CLIP filter is ~350MB on GPU; gsplat already lives there, and
        # Qwen-VL is API-based so we don't need a local image encoder.
        # Disable to keep VRAM headroom.
        use_clip_filtering=False,
    ).transports(
        {
            ("color_image", Image): LCMTransport("/splat/color_image", Image),
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
        }
    ),
    PatrollingModule.blueprint(),
    WavefrontFrontierExplorer.blueprint(),
)

unitree_g1_groot_wbc_sim = autoconnect(
    _g1_coordinator,
    _g1_engine,
    _g1_ws_vis,
    *_viser_modules,
    *_g1_perception_stack,
    *_g1_agentic_stack,
).global_config(n_workers=18, detection_model="qwen")

__all__ = ["unitree_g1_groot_wbc_sim"]
