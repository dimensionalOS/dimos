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

Operator-facing planes ride Cloudflare transports; robot-internal driver
commands go over RPC (``go2: GO2Connection`` ref). All broker-bound modules
share one worker so the Cloudflare transports resolve to a single session
(GO2Connection is RPC/LCM only in its own worker — no session fragmentation).

Drive routing: broker cmd_unreliable → Go2CommandModule ``cmd_vel_in`` → guard
→ ``tele_cmd_vel`` → MovementManager (arbitrates manual vs nav, sole cmd_vel
producer) → GO2Connection. ``state_reliable`` fans to stats AND command modules.
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
from dimos.mapping.voxels.module import VoxelGridMapper
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
from dimos.teleop.hosted.robot_type import RobotType

# Single camera: only the Go2's front camera feeds the video track.
teleop_hosted_go2_transport = (
    autoconnect(
        GO2Connection.blueprint(),
        Go2CommandModule.blueprint(allow_acrobatics=True),
        CameraMuxModule.blueprint(cameras=["cam1"]),
        HostedStatsModule.blueprint(),
        MapCompressModule.blueprint(),
        VoxelGridMapper.blueprint(emit_every=5),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
        MovementManager.blueprint(),
    )
    .remappings(
        [
            (GO2Connection, "color_image", "cam1"),
        ]
    )
    .transports(
        {
            # inbound operator planes
            ("cmd_vel_in", TwistStamped): CloudflareTransport.spec("cmd_unreliable", TwistStamped),
            ("state_json", bytes): CloudflareTransport.spec(
                "state_reliable", robot_type=RobotType.GO2
            ),
            ("camera_select", bytes): CloudflareTransport.spec("state_reliable"),
            ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),
            # outbound operator planes
            ("mux_image", Image): CloudflareVideoTransport.spec(),
            ("map_out", bytes): CloudflareTransport.spec("map_unreliable"),
            ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
            ("cmd_ack", bytes): CloudflareTransport.spec("state_reliable_back"),
            # drive chain on namespaced LCM topics — the bare global /cmd_vel
            # (other robots/tools on the machine) must not cross-decode here.
            ("tele_cmd_vel", Twist): LCMTransport.spec("/hosted/tele_cmd_vel", Twist),
            ("nav_cmd_vel", Twist): LCMTransport.spec("/hosted/nav_cmd_vel", Twist),
            ("cmd_vel", Twist): LCMTransport.spec("/hosted/cmd_vel", Twist),
        }
    )
    .global_config(viewer="none", n_workers=2)  # go2 driver | broker+nav modules
)


# Multicam: adds a RealSense as cam2 (operator-selectable in the mux).
teleop_hosted_go2_multicam = (
    autoconnect(
        GO2Connection.blueprint(),
        Go2CommandModule.blueprint(allow_acrobatics=True),
        CameraMuxModule.blueprint(cameras=["cam1", "cam2"]),
        HostedStatsModule.blueprint(),
        MapCompressModule.blueprint(),
        RealSenseCamera.blueprint(enable_depth=False, enable_pointcloud=False),
        VoxelGridMapper.blueprint(emit_every=5),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
        MovementManager.blueprint(),
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
            ("cmd_vel_in", TwistStamped): CloudflareTransport.spec("cmd_unreliable", TwistStamped),
            ("state_json", bytes): CloudflareTransport.spec(
                "state_reliable", robot_type=RobotType.GO2
            ),
            ("camera_select", bytes): CloudflareTransport.spec("state_reliable"),
            ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),
            ("cam2", Image): LCMTransport.spec("cam2", Image),
            # outbound operator planes
            ("mux_image", Image): CloudflareVideoTransport.spec(),
            ("map_out", bytes): CloudflareTransport.spec("map_unreliable"),
            ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
            ("cmd_ack", bytes): CloudflareTransport.spec("state_reliable_back"),
            # drive chain on namespaced LCM topics (see teleop_hosted_go2_transport).
            ("tele_cmd_vel", Twist): LCMTransport.spec("/hosted/tele_cmd_vel", Twist),
            ("nav_cmd_vel", Twist): LCMTransport.spec("/hosted/nav_cmd_vel", Twist),
            ("cmd_vel", Twist): LCMTransport.spec("/hosted/cmd_vel", Twist),
        }
    )
    .global_config(viewer="none", n_workers=2)  # go2 driver | broker+nav modules
)


# ─── XArm hosted manipulation (coordinator-driven, WebXR + browser operator) ──


# Distinct classes only because blueprints can't yet run two instances of one
# module. Serials: -o frontcamera.serial_number=... -o wristcamera.serial_number=...
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
            ("state_json", bytes): CloudflareTransport.spec(
                "state_reliable", robot_type=RobotType.ARM
            ),
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
            ("state_json", bytes): CloudflareTransport.spec(
                "state_reliable", robot_type=RobotType.ARM
            ),
            ("camera_select", bytes): CloudflareTransport.spec("state_reliable"),
            ("mux_image", Image): CloudflareVideoTransport.spec(),
            ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
            ("cmd_ack", bytes): CloudflareTransport.spec("state_reliable_back"),
        }
    )
    .global_config(viewer="none", n_workers=1)  # one process → one CF session
)
