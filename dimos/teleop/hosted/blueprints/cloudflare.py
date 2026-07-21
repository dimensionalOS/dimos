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

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import (
    CloudflareTransport,
    CloudflareVideoTransport,
    LCMTransport,
)
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.manipulators.xarm.blueprints.teleop import (
    coordinator_teleop_xarm6,
    coordinator_teleop_xarm7,
)
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.teleop.hosted.arm_command import ArmCommandModule
from dimos.teleop.hosted.camera_mux import CameraMuxModule
from dimos.teleop.hosted.go2_command import Go2CommandModule
from dimos.teleop.hosted.hosted_stats import HostedStatsModule
from dimos.teleop.hosted.map_compress import MapCompressModule

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


# ─── XArm hosted manipulation (coordinator-driven, WebXR + browser operator) ──
#
# ArmCommandModule is the operator command plane; actuation runs through the
# ControlCoordinator over LCM. Two RealSense cameras (front = cam1, wrist =
# cam2), operator-selectable via the mux.


# Distinct classes so two RealSense units coexist in one blueprint. Serials:
# -o frontcamera.serial_number=... -o wristcamera.serial_number=...


# These subclasses exist only until blueprints support running multiple
# instances of the same module.
class FrontCamera(RealSenseCamera):
    pass


class WristCamera(RealSenseCamera):
    pass


teleop_hosted_xarm6 = (
    autoconnect(
        ArmCommandModule.blueprint(task_names={"right": "teleop_xarm"}),
        HostedStatsModule.blueprint(),
        CameraMuxModule.blueprint(cameras=["cam1", "cam2"]),
        coordinator_teleop_xarm6,
        FrontCamera.blueprint(camera_name="front", enable_depth=False, enable_pointcloud=False),
        WristCamera.blueprint(camera_name="wrist", enable_depth=False, enable_pointcloud=False),
    )
    .remappings(
        [
            (FrontCamera, "color_image", "cam1"),
            (WristCamera, "color_image", "cam2"),
            (ArmCommandModule, "right_controller_output", "coordinator_cartesian_command"),
        ]
    )
    .transports(
        {
            ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),
            ("state_json", bytes): CloudflareTransport.spec("state_reliable"),
            ("camera_select", bytes): CloudflareTransport.spec("state_reliable"),
            ("mux_image", Image): CloudflareVideoTransport.spec(),
            ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
            ("cmd_ack", bytes): CloudflareTransport.spec("state_reliable_back"),
        }
    )
    .global_config(viewer="none", n_workers=1)  # one process → one CF session
)


teleop_hosted_xarm7 = (
    autoconnect(
        ArmCommandModule.blueprint(task_names={"right": "teleop_xarm"}),
        HostedStatsModule.blueprint(),
        CameraMuxModule.blueprint(cameras=["cam1", "cam2"]),
        coordinator_teleop_xarm7,
        FrontCamera.blueprint(camera_name="front", enable_depth=False, enable_pointcloud=False),
        WristCamera.blueprint(camera_name="wrist", enable_depth=False, enable_pointcloud=False),
    )
    .remappings(
        [
            (FrontCamera, "color_image", "cam1"),
            (WristCamera, "color_image", "cam2"),
            (ArmCommandModule, "right_controller_output", "coordinator_cartesian_command"),
        ]
    )
    .transports(
        {
            ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),
            ("state_json", bytes): CloudflareTransport.spec("state_reliable"),
            ("camera_select", bytes): CloudflareTransport.spec("state_reliable"),
            ("mux_image", Image): CloudflareVideoTransport.spec(),
            ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
            ("cmd_ack", bytes): CloudflareTransport.spec("state_reliable_back"),
        }
    )
    .global_config(viewer="none", n_workers=1)  # one process → one CF session
)
