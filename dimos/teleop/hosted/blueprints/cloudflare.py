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

"""Hosted teleop blueprints — Cloudflare broker (module-based).

Composes the split hosted-teleop modules — driver (GO2Connection, as-is),
Go2CommandModule, CameraMuxModule, HostedStatsModule, MapCompressModule — plus
mapping/planning. GO2Connection runs in its own worker (``dedicated_worker``);
all broker-bound modules share the other worker, so the Cloudflare transports
resolve to a single session. GO2Connection binds no broker transport (it's
RPC/LCM only), so the split doesn't fragment the CF session.

  * Operator-facing planes (video, map, telemetry, acks, inbound state/cmd) →
    TRANSPORT (each broker-bound Out binds to a ``Cloudflare*`` transport).
  * Robot-internal driver commands → RPC (Go2CommandModule holds a ``go2:
    GO2Connection`` ref and calls its @rpc methods).

Drive routing (kept off RPC): broker cmd_unreliable → Go2CommandModule
``cmd_vel_in`` → guard → ``tele_cmd_vel`` → MovementManager (arbitrates manual vs
nav) → GO2Connection ``cmd_vel``. ``state_reliable`` is fanned to BOTH
HostedStatsModule and Go2CommandModule.
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import Blueprint, TransportSpec, autoconnect
from dimos.core.global_config import global_config
from dimos.core.module import ModuleBase
from dimos.core.stream import Transport
from dimos.core.transport import (
    CloudflareTransport,
    CloudflareVideoTransport,
    LCMTransport,
)
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.std_msgs.Bool import Bool
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.manipulators.xarm.blueprints.teleop import (
    coordinator_teleop_xarm6,
    coordinator_teleop_xarm7,
)
from dimos.robot.manipulators.xarm.config import XARM6_SIM_PATH, XARM7_SIM_PATH
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.spec.utils import Spec
from dimos.teleop.hosted.arm_command import ArmCommandModule
from dimos.teleop.hosted.camera_mux import CameraMuxModule
from dimos.teleop.hosted.go2_command import Go2CommandModule
from dimos.teleop.hosted.hosted_stats import HostedStatsModule
from dimos.teleop.hosted.map_compress import MapCompressModule
from dimos.teleop.quest.quest_types import Buttons

# Single camera: only the Go2's front camera feeds the video track.
teleop_hosted_go2_transport = (
    autoconnect(
        GO2Connection.blueprint(),  # driver AS-IS (+ @rpc command methods); no vis
        Go2CommandModule.blueprint(),  # command/E-STOP dispatch + drive guard
        CameraMuxModule.blueprint(cameras=["cam1"]),  # go2 cam → mux_image
        HostedStatsModule.blueprint(),  # state stats dispatch + telemetry + acks
        MapCompressModule.blueprint(),  # costmap (+odom) → map_out
        VoxelGridMapper.blueprint(emit_every=5),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
        MovementManager.blueprint(),  # arbitrates manual vs nav → owns cmd_vel
    )
    # MovementManager is the SOLE cmd_vel producer. It combines guarded manual
    # drive (Go2CommandModule.tele_cmd_vel) with the planner (nav_cmd_vel);
    # manual input auto-cancels the active plan (tele_cooldown). Its cmd_vel
    # output feeds the driver.
    .remappings(
        [
            (GO2Connection, "color_image", "cam1"),
        ]
    )
    .transports(
        {
            # inbound operator planes
            ("cmd_vel_in", Twist): CloudflareTransport.spec("cmd_unreliable", TwistStamped),
            ("state_json", bytes): CloudflareTransport.spec("state_reliable"),  # → stats + command
            ("camera_select", bytes): CloudflareTransport.spec("state_reliable"),  # → mux
            ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),  # stats tap
            # outbound operator planes
            ("mux_image", Image): CloudflareVideoTransport.spec(),
            ("map_out", bytes): CloudflareTransport.spec("map_unreliable"),
            ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
            ("cmd_ack", bytes): CloudflareTransport.spec("state_reliable_back"),
            # robot-internal drive chain — namespaced LCM topics so the bare
            # global /cmd_vel (used by other robots/tools on the machine) can't
            # cross-decode into these Twist subscribers.
            ("tele_cmd_vel", Twist): LCMTransport.spec("/hosted/tele_cmd_vel", Twist),
            ("nav_cmd_vel", Twist): LCMTransport.spec("/hosted/nav_cmd_vel", Twist),
            ("cmd_vel", Twist): LCMTransport.spec("/hosted/cmd_vel", Twist),
        }
    )
    .global_config(viewer="none", n_workers=2)  # go2 driver | broker+nav modules
)


# ─── XArm hosted manipulation (coordinator-driven, WebXR + browser operator) ──
#
# The arm analog of the Go2 hosted blueprints. ArmCommandModule (the operator
# command/E-STOP plane + engage/delta-pose control loop) replaces Go2CommandModule;
# there's no robot driver — actuation runs through the ControlCoordinator, fed
# over LCM. CameraMuxModule + HostedStatsModule are reused as-is (the arm has no
# battery, so HostedStatsModule.go2 is left unbound → soc omitted).
#
# Two cameras: front/overview = cam1, wrist = cam2, operator-selectable via the
# mux. Real hardware uses two RealSense units; --simulation renders both from the
# MuJoCo scene. Run with -o transports.broker.api_key=dtk_live_...


# Distinct classes so two RealSense units coexist in one blueprint: module
# identity is the class throughout the stack. Configure serials with
# -o frontcamera.serial_number=... -o wristcamera.serial_number=...
class FrontCamera(RealSenseCamera):
    pass


class WristCamera(RealSenseCamera):
    pass


# The LCM topics that feed the coordinator (cartesian_command, ee_twist, gripper)
# MUST carry a leading slash to match its default "/"-prefixed inputs — without
# it the arm engages but never moves.
_Remaps = list[tuple[type[ModuleBase], str, str | type[ModuleBase] | type[Spec]]]

_ARM_TRANSPORTS: dict[tuple[str, type], TransportSpec | Transport[Any]] = {
    # inbound operator planes
    ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),
    ("state_json", bytes): CloudflareTransport.spec("state_reliable"),  # → command + stats
    ("camera_select", bytes): CloudflareTransport.spec("state_reliable"),  # → mux
    # outbound operator planes
    ("mux_image", Image): CloudflareVideoTransport.spec(),
    ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
    ("cmd_ack", bytes): CloudflareTransport.spec("state_reliable_back"),
    # command → coordinator over LCM (leading slash — see note above)
    ("right_controller_output", PoseStamped): LCMTransport.spec(
        "/coordinator_cartesian_command", PoseStamped
    ),
    ("teleop_buttons", Buttons): LCMTransport.spec("teleop_buttons", Buttons),
    ("coordinator_ee_twist_command", TwistStamped): LCMTransport.spec(
        "/coordinator_ee_twist_command", TwistStamped
    ),
    ("gripper_command", Bool): LCMTransport.spec("/gripper_command", Bool),
    # robot_state (command → stats), robot_telemetry + cmd_vel_stamped (stats →
    # recorder) are bytes/intra-process streams: with n_workers=1 autoconnect
    # wires them in-process, so no transport binding is needed here (a bytes
    # stream can't be bound to a typed LCM topic anyway).
}


_xarm6_cameras: tuple[Blueprint, ...]
_xarm6_remaps: _Remaps
if global_config.simulation:
    _xarm6_cameras = (
        MujocoSimModule.blueprint(
            address=str(XARM6_SIM_PATH),
            headless=False,
            dof=6,
            camera_name="wrist_camera",
            camera2_name="env_camera",
            width=848,
            height=480,
        ),
    )
    _xarm6_remaps = [
        (MujocoSimModule, "color_image2", "cam1"),  # env overview → cam1
        (MujocoSimModule, "color_image", "cam2"),  # wrist → cam2
    ]
else:
    _xarm6_cameras = (
        FrontCamera.blueprint(camera_name="front", enable_depth=False, enable_pointcloud=False),
        WristCamera.blueprint(camera_name="wrist", enable_depth=False, enable_pointcloud=False),
    )
    _xarm6_remaps = [
        (FrontCamera, "color_image", "cam1"),
        (WristCamera, "color_image", "cam2"),
    ]

teleop_hosted_xarm6 = (
    autoconnect(
        ArmCommandModule.blueprint(task_names={"right": "teleop_xarm"}),
        HostedStatsModule.blueprint(),
        CameraMuxModule.blueprint(cameras=["cam1", "cam2"]),
        coordinator_teleop_xarm6,
        *_xarm6_cameras,
    )
    .remappings(_xarm6_remaps)
    .transports(_ARM_TRANSPORTS)
    .global_config(viewer="none", n_workers=1)  # one process → one CF session
)


_xarm7_cameras: tuple[Blueprint, ...]
_xarm7_remaps: _Remaps
if global_config.simulation:
    _xarm7_cameras = (
        MujocoSimModule.blueprint(
            address=str(XARM7_SIM_PATH),
            headless=False,
            dof=7,
            camera_name="wrist_camera",
            camera2_name="env_camera",
            width=848,
            height=480,
        ),
    )
    _xarm7_remaps = [
        (MujocoSimModule, "color_image2", "cam1"),  # env overview → cam1
        (MujocoSimModule, "color_image", "cam2"),  # wrist → cam2
    ]
else:
    _xarm7_cameras = (
        FrontCamera.blueprint(camera_name="front", enable_depth=False, enable_pointcloud=False),
        WristCamera.blueprint(camera_name="wrist", enable_depth=False, enable_pointcloud=False),
    )
    _xarm7_remaps = [
        (FrontCamera, "color_image", "cam1"),
        (WristCamera, "color_image", "cam2"),
    ]

teleop_hosted_xarm7 = (
    autoconnect(
        ArmCommandModule.blueprint(task_names={"right": "teleop_xarm"}),
        HostedStatsModule.blueprint(),
        CameraMuxModule.blueprint(cameras=["cam1", "cam2"]),
        coordinator_teleop_xarm7,
        *_xarm7_cameras,
    )
    .remappings(_xarm7_remaps)
    .transports(_ARM_TRANSPORTS)
    .global_config(viewer="none", n_workers=1)  # one process → one CF session
)


# Multicam: adds a RealSense as cam2 (operator-selectable in the mux). Needs the
# RealSense wired in; use teleop-hosted-go2-transport otherwise.
teleop_hosted_go2_multicam = (
    autoconnect(
        GO2Connection.blueprint(),  # driver AS-IS (+ @rpc command methods); no vis
        Go2CommandModule.blueprint(),  # command/E-STOP dispatch + drive guard
        CameraMuxModule.blueprint(cameras=["cam1", "cam2"]),  # go2 + realsense → mux_image
        HostedStatsModule.blueprint(),  # state stats dispatch + telemetry + acks
        MapCompressModule.blueprint(),  # costmap (+odom) → map_out
        RealSenseCamera.blueprint(enable_depth=False, enable_pointcloud=False),
        VoxelGridMapper.blueprint(emit_every=5),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
        MovementManager.blueprint(),  # arbitrates manual vs nav → owns cmd_vel
    )
    .remappings(
        [
            (GO2Connection, "color_image", "cam1"),
            (RealSenseCamera, "color_image", "cam2"),
        ]
    )
    .transports(
        {
            # inbound operator planes
            ("cmd_vel_in", Twist): CloudflareTransport.spec("cmd_unreliable", TwistStamped),
            ("state_json", bytes): CloudflareTransport.spec("state_reliable"),  # → stats + command
            ("camera_select", bytes): CloudflareTransport.spec("state_reliable"),  # → mux
            ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),  # stats tap
            ("cam2", Image): LCMTransport.spec("cam2", Image),  # realsense over LCM
            # outbound operator planes
            ("mux_image", Image): CloudflareVideoTransport.spec(),
            ("map_out", bytes): CloudflareTransport.spec("map_unreliable"),
            ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
            ("cmd_ack", bytes): CloudflareTransport.spec("state_reliable_back"),
            # robot-internal drive chain — namespaced LCM topics (see above).
            ("tele_cmd_vel", Twist): LCMTransport.spec("/hosted/tele_cmd_vel", Twist),
            ("nav_cmd_vel", Twist): LCMTransport.spec("/hosted/nav_cmd_vel", Twist),
            ("cmd_vel", Twist): LCMTransport.spec("/hosted/cmd_vel", Twist),
        }
    )
    .global_config(viewer="none", n_workers=2)  # go2 driver | broker+nav modules
)
