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

"""Hosted teleoperation blueprints backed by a LiveKit edge module."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.msgs.geometry_msgs.Twist import Twist
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
from dimos.teleop.hosted.livekit import LiveKitTeleopModule
from dimos.teleop.hosted.map_compress import MapCompressModule
from dimos.teleop.hosted.robot_type import RobotType


class LiveKitFrontCamera(RealSenseCamera):
    pass


class LiveKitWristCamera(RealSenseCamera):
    pass


teleop_hosted_go2_livekit = (
    autoconnect(
        GO2Connection.blueprint(),
        Go2CommandModule.blueprint(allow_acrobatics=True),
        CameraMuxModule.blueprint(cameras=["cam1"]),
        HostedStatsModule.blueprint(),
        MapCompressModule.blueprint(),
        LiveKitTeleopModule.blueprint(robot_type=RobotType.GO2),
        VoxelGridMapper.blueprint(emit_every=5),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
        MovementManager.blueprint(),
    )
    .remappings([(GO2Connection, "color_image", "cam1")])
    .transports(
        {
            ("tele_cmd_vel", Twist): LCMTransport.spec("/hosted/tele_cmd_vel", Twist),
            ("nav_cmd_vel", Twist): LCMTransport.spec("/hosted/nav_cmd_vel", Twist),
            ("cmd_vel", Twist): LCMTransport.spec("/hosted/cmd_vel", Twist),
        }
    )
    .global_config(viewer="none", n_workers=2)
)


teleop_hosted_xarm6_livekit = (
    autoconnect(
        ArmCommandModule.blueprint(task_names={"right": "teleop_xarm"}),
        HostedStatsModule.blueprint(),
        CameraMuxModule.blueprint(cameras=["cam1", "cam2"]),
        LiveKitTeleopModule.blueprint(robot_type=RobotType.ARM),
        coordinator_teleop_xarm6,
        LiveKitFrontCamera.blueprint(
            camera_name="front", enable_depth=False, enable_pointcloud=False
        ),
        LiveKitWristCamera.blueprint(
            camera_name="wrist", enable_depth=False, enable_pointcloud=False
        ),
    )
    .remappings(
        [
            (LiveKitFrontCamera, "color_image", "cam1"),
            (LiveKitWristCamera, "color_image", "cam2"),
            (ArmCommandModule, "right_controller_output", "coordinator_cartesian_command"),
        ]
    )
    .global_config(viewer="none", n_workers=1)
)


teleop_hosted_xarm7_livekit = (
    autoconnect(
        ArmCommandModule.blueprint(task_names={"right": "teleop_xarm"}),
        HostedStatsModule.blueprint(),
        CameraMuxModule.blueprint(cameras=["cam1", "cam2"]),
        LiveKitTeleopModule.blueprint(robot_type=RobotType.ARM),
        coordinator_teleop_xarm7,
        LiveKitFrontCamera.blueprint(
            camera_name="front", enable_depth=False, enable_pointcloud=False
        ),
        LiveKitWristCamera.blueprint(
            camera_name="wrist", enable_depth=False, enable_pointcloud=False
        ),
    )
    .remappings(
        [
            (LiveKitFrontCamera, "color_image", "cam1"),
            (LiveKitWristCamera, "color_image", "cam2"),
            (ArmCommandModule, "right_controller_output", "coordinator_cartesian_command"),
        ]
    )
    .global_config(viewer="none", n_workers=1)
)
