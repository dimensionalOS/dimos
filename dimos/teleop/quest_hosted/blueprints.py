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

"""Hosted teleop blueprints (WebRTC transport)."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import (
    CloudflareTransport,
    CloudflareVideoTransport,
    LCMTransport,
    LiveKitTransport,
    LiveKitVideoTransport,
)
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.teleop.quest_hosted.go2_hosted_connection import Go2HostedConnection
from dimos.teleop.quest_hosted.hosted_extensions import HostedTwistTeleopModule

# Hosted teleop via the legacy module wrapper (the transport swap is preferred).
teleop_hosted_go2 = autoconnect(
    HostedTwistTeleopModule.blueprint(),
    unitree_go2_basic,
).global_config(n_workers=8, viewer="none")

# Hosted teleop over CF Realtime. Run with -o transports.broker.api_key=dtk_live_...
# VoxelGridMapper → CostMapper build the OccupancyGrid; Go2HostedConnection
# encodes it (+ odom) onto state_reliable_back for the operator minimap.
teleop_hosted_go2_transport = (
    autoconnect(
        unitree_go2_basic.disabled_modules(GO2Connection),
        Go2HostedConnection.blueprint(),
        VoxelGridMapper.blueprint(emit_every=5),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
    )
    .transports(
        {
            ("cmd_vel", Twist): CloudflareTransport.spec("cmd_unreliable", TwistStamped),
            ("mux_image", Image): CloudflareVideoTransport.spec(),
            ("state_json", bytes): CloudflareTransport.spec("state_reliable"),
            ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
            ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),  # stats tap
            ("cmd_vel_stamped", TwistStamped): LCMTransport.spec("cmd_vel_stamped", TwistStamped),
            ("lidar", PointCloud2): LCMTransport.spec("lidar", PointCloud2),
            ("global_map", PointCloud2): LCMTransport.spec("global_map", PointCloud2),
            ("global_costmap", OccupancyGrid): LCMTransport.spec("global_costmap", OccupancyGrid),
            ("map_out", bytes): CloudflareTransport.spec("map_unreliable"),
        }
    )
    .global_config(viewer="none")
)


# LiveKit backend: drive + camera + state/telemetry. No minimap or
# click-to-nav yet (the CF transport blueprint wires VoxelGridMapper/CostMapper/
# ReplanningAStarPlanner + map_unreliable; LiveKit doesn't).
teleop_hosted_go2_livekit = (
    autoconnect(
        unitree_go2_basic.disabled_modules(GO2Connection),
        Go2HostedConnection.blueprint(),
    )
    .transports(
        {
            ("cmd_vel", Twist): LiveKitTransport.spec("cmd_unreliable", TwistStamped),
            ("mux_image", Image): LiveKitVideoTransport.spec(),
            ("state_json", bytes): LiveKitTransport.spec("state_reliable"),
            ("telemetry_out", bytes): LiveKitTransport.spec("state_reliable_back"),
            ("cmd_raw", bytes): LiveKitTransport.spec("cmd_unreliable"),  # stats tap
            ("cmd_vel_stamped", TwistStamped): LCMTransport.spec("cmd_vel_stamped", TwistStamped),
        }
    )
    .global_config(viewer="none")
)


# Adds a RealSense as cam2 (mux'd into the video track by Go2HostedConnection).
# Needs the RealSense wired in; use teleop-hosted-go2-transport otherwise.
# Same map overlay as teleop_hosted_go2_transport.
teleop_hosted_go2_multicam = (
    autoconnect(
        unitree_go2_basic.disabled_modules(GO2Connection),
        Go2HostedConnection.blueprint(),
        RealSenseCamera.blueprint(enable_depth=False, enable_pointcloud=False),
        VoxelGridMapper.blueprint(emit_every=5),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
    )
    .remappings([(RealSenseCamera, "color_image", "cam2_in")])
    .transports(
        {
            ("cmd_vel", Twist): CloudflareTransport.spec("cmd_unreliable", TwistStamped),
            ("mux_image", Image): CloudflareVideoTransport.spec(),
            ("cam2_in", Image): LCMTransport.spec("cam2_in", Image),
            ("state_json", bytes): CloudflareTransport.spec("state_reliable"),
            ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
            ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),
            ("cmd_vel_stamped", TwistStamped): LCMTransport.spec("cmd_vel_stamped", TwistStamped),
            ("lidar", PointCloud2): LCMTransport.spec("lidar", PointCloud2),
            ("global_map", PointCloud2): LCMTransport.spec("global_map", PointCloud2),
            ("global_costmap", OccupancyGrid): LCMTransport.spec("global_costmap", OccupancyGrid),
            ("map_out", bytes): CloudflareTransport.spec("map_unreliable"),
        }
    )
    .global_config(viewer="none")
)
