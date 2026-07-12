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

"""Hosted teleop blueprints — LiveKit broker (module-based).

Drop-in LiveKit variant of ``cloudflare.py``: identical module composition and
drive routing, only the operator-facing transports swap ``Cloudflare*`` →
``LiveKit*`` (both resolve their broker config from ``transports.broker.*``).
All modules run in ONE process (``n_workers=1``) so the broker transports share
a single LiveKit session.

See ``cloudflare.py`` for the full architecture notes (RPC vs transport split,
drive arbitration).

NOTE: operator→speaker audio is NOT available on LiveKit — the LiveKit provider
has no inbound-audio track handling (no ``set_audio_frame_callback``), so
``go2connection.audio_in`` is a no-op here. Use the Cloudflare blueprint if you
need operator audio on the dog's speaker.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import (
    LCMTransport,
    LiveKitTransport,
    LiveKitVideoTransport,
)
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.teleop.hosted.camera_mux import CameraMuxModule
from dimos.teleop.hosted.go2_command import Go2CommandModule
from dimos.teleop.hosted.hosted_stats import HostedStatsModule
from dimos.teleop.hosted.map_compress import MapCompressModule

# Single camera: only the Go2's front camera feeds the video track.
teleop_hosted_go2_livekit = (
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
            (MovementManager, "cmd_vel", "cmd_vel"),  # → GO2Connection.cmd_vel
            (GO2Connection, "color_image", "cam1"),
        ]
    )
    .transports(
        {
            # inbound operator planes
            ("cmd_vel_in", Twist): LiveKitTransport.spec("cmd_unreliable", TwistStamped),
            ("state_json", bytes): LiveKitTransport.spec("state_reliable"),  # → stats + command
            ("camera_select", bytes): LiveKitTransport.spec("state_reliable"),  # → mux
            ("cmd_raw", bytes): LiveKitTransport.spec("cmd_unreliable"),  # stats tap
            # outbound operator planes
            ("mux_image", Image): LiveKitVideoTransport.spec(),
            ("map_out", bytes): LiveKitTransport.spec("map_unreliable"),
            ("telemetry_out", bytes): LiveKitTransport.spec("state_reliable_back"),
            ("cmd_ack", bytes): LiveKitTransport.spec("state_reliable_back"),
            # robot-internal drive chain — namespaced LCM topics so the bare
            # global /cmd_vel (used by other robots/tools on the machine) can't
            # cross-decode into these Twist subscribers.
            ("tele_cmd_vel", Twist): LCMTransport.spec("/hosted/tele_cmd_vel", Twist),
            ("nav_cmd_vel", Twist): LCMTransport.spec("/hosted/nav_cmd_vel", Twist),
            ("cmd_vel", Twist): LCMTransport.spec("/hosted/cmd_vel", Twist),
            # robot-internal / recorder over LCM
            ("cmd_vel_stamped", TwistStamped): LCMTransport.spec("cmd_vel_stamped", TwistStamped),
            ("lidar", PointCloud2): LCMTransport.spec("lidar", PointCloud2),
            ("global_map", PointCloud2): LCMTransport.spec("global_map", PointCloud2),
            ("global_costmap", OccupancyGrid): LCMTransport.spec("global_costmap", OccupancyGrid),
            ("goal_request", PoseStamped): LCMTransport.spec("goal_request", PoseStamped),
        }
    )
    .global_config(viewer="none", n_workers=1)  # one process → one LiveKit session
)


# Multicam: adds a RealSense as cam2 (operator-selectable in the mux). Needs the
# RealSense wired in; use teleop-hosted-go2-livekit otherwise.
teleop_hosted_go2_livekit_multicam = (
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
            (MovementManager, "cmd_vel", "cmd_vel"),  # → GO2Connection.cmd_vel
            (GO2Connection, "color_image", "cam1"),
            (RealSenseCamera, "color_image", "cam2"),
        ]
    )
    .transports(
        {
            # inbound operator planes
            ("cmd_vel_in", Twist): LiveKitTransport.spec("cmd_unreliable", TwistStamped),
            ("state_json", bytes): LiveKitTransport.spec("state_reliable"),  # → stats + command
            ("camera_select", bytes): LiveKitTransport.spec("state_reliable"),  # → mux
            ("cmd_raw", bytes): LiveKitTransport.spec("cmd_unreliable"),  # stats tap
            ("cam2", Image): LCMTransport.spec("cam2", Image),  # realsense over LCM
            # outbound operator planes
            ("mux_image", Image): LiveKitVideoTransport.spec(),
            ("map_out", bytes): LiveKitTransport.spec("map_unreliable"),
            ("telemetry_out", bytes): LiveKitTransport.spec("state_reliable_back"),
            ("cmd_ack", bytes): LiveKitTransport.spec("state_reliable_back"),
            # robot-internal drive chain — namespaced LCM topics (see above).
            ("tele_cmd_vel", Twist): LCMTransport.spec("/hosted/tele_cmd_vel", Twist),
            ("nav_cmd_vel", Twist): LCMTransport.spec("/hosted/nav_cmd_vel", Twist),
            ("cmd_vel", Twist): LCMTransport.spec("/hosted/cmd_vel", Twist),
            # robot-internal / recorder over LCM
            ("cmd_vel_stamped", TwistStamped): LCMTransport.spec("cmd_vel_stamped", TwistStamped),
            ("lidar", PointCloud2): LCMTransport.spec("lidar", PointCloud2),
            ("global_map", PointCloud2): LCMTransport.spec("global_map", PointCloud2),
            ("global_costmap", OccupancyGrid): LCMTransport.spec("global_costmap", OccupancyGrid),
            ("goal_request", PoseStamped): LCMTransport.spec("goal_request", PoseStamped),
        }
    )
    .global_config(viewer="none", n_workers=1)  # one process → one LiveKit session
)
