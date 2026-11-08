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

"""Real R1 Pro: the shared Desktop/XR station, vendor actuation and calibrated RGB-D."""

import math

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.livox.ports import (
    SDK_HOST_CMD_DATA_PORT,
    SDK_HOST_IMU_DATA_PORT,
    SDK_HOST_POINT_DATA_PORT,
)
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.pointlio_blueprints import mid360_for_pointlio
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_vr_teleop import vr_tasks
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_CHASSIS_LIDAR_HOST_IP,
    R1PRO_CHASSIS_LIDAR_IP,
    R1PRO_COORDINATOR_MODEL,
    R1PRO_MODEL,
    make_r1pro_model_config,
    make_r1pro_upper_teleop_model,
)
from dimos.robot.galaxea.r1pro.connection import R1PRO_GRIPPER_JOINTS, R1ProConnection
from dimos.robot.galaxea.r1pro.lio import R1ProLio
from dimos.robot.galaxea.r1pro.spatial import R1ProSpatialTeleop
from dimos.robot.galaxea.r1pro.spatial_sensors import R1SpatialCloud, R1SpatialSensors
from dimos.robot.galaxea.r1pro.teleop import R1HardwareTeleopCoordinator
from dimos.robot.urdf_manager import UrdfManager
from dimos.teleop.spatial.model import SpatialGroup


def spatial_tasks() -> list[TaskConfig]:
    tasks = vr_tasks()
    for task in tasks:
        if task.type == "teleop_ik":
            task.params = {
                **task.params,
                "absolute_targets": True,
                "robot_model": make_r1pro_upper_teleop_model(),
                "max_joint_velocity_rad_s": 0.25,
                "feedback_limit_tolerance": 0.01,
                "command_limit_margin": 0.03,
            }
            task.params["pink"] = task.params["pink"].model_copy(
                update={"solver": "quadprog", "dt": 0.02}
            )
    return tasks


r1pro_spatial = (
    autoconnect(
        UrdfManager.blueprint(model=R1PRO_MODEL, unchanged_tf_hz=15),
        R1ProConnection.blueprint(
            tracking_speed=0.25,
            enable_wrist_color=False,
            enable_wrist_depth=False,
            boot_vendor_stack=False,
            publish_odom_tf=False,
        ),
        R1HardwareTeleopCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=R1PRO_COORDINATOR_MODEL,
            command_only_joints=list(R1PRO_GRIPPER_JOINTS),
            tasks=spatial_tasks(),
            tick_rate=50,
            publish_joint_state=False,
        ),
        R1SpatialSensors.blueprint(),
        mid360_for_pointlio(
            frame_id="lidar_pointlio_link",
            lidar_ip=R1PRO_CHASSIS_LIDAR_IP,
            host_ip=R1PRO_CHASSIS_LIDAR_HOST_IP,
            # Keep the vendor listener ports available; this stack owns its own sockets.
            host_cmd_data_port=SDK_HOST_CMD_DATA_PORT + 1,
            host_point_data_port=SDK_HOST_POINT_DATA_PORT + 1,
            host_imu_data_port=SDK_HOST_IMU_DATA_PORT + 1,
        ),
        PointLio.blueprint(frame_id="world", sensor_frame_id="lidar_pointlio_link").remappings(
            [(PointLio, "odometry", "pointlio_odometry")]
        ),
        R1ProLio.blueprint(publish_hz=15).remappings([(R1ProLio, "odometry", "pointlio_odometry")]),
        R1SpatialCloud.blueprint(),
        VoxelGridMapper.blueprint(
            device="CPU:0", voxel_size=0.05, carve_columns=False, block_count=100_000, emit_every=5
        ).remappings([(VoxelGridMapper, "lidar", "pointcloud")]),
        R1ProSpatialTeleop.blueprint(
            title="R1 Pro · Real robot",
            environment="Hardware",
            joint_jog_available=False,
            absolute_body_targets=True,
            ik_base_frame="torso_link2",
            robot_model=make_r1pro_model_config(),
            groups={
                side: SpatialGroup(
                    label=f"{side.title()} arm",
                    joint_names=tuple(f"r1pro/{side}_arm_joint{i}" for i in range(1, 8)),
                    tip_frame=f"{side}_gripper_link",
                    base_frame=f"{side}_arm_base_link",
                    body_shoulder_frame=f"{side}_arm_link2",
                    body_anchor_frame="torso_link3",
                    # Neutral URDF arms point down; a forward hand is Ry(-pi/2).
                    body_forward_orientation=(0, -math.sqrt(0.5), 0, math.sqrt(0.5)),
                    gripper_joint=f"r1pro/{side}_gripper",
                    gripper_open_position=0.1,
                )
                for side in ("left", "right")
            },
            cameras={
                "workspace": "Head · stereo depth",
                "left": "Left wrist · RGB-D",
                "right": "Right wrist · RGB-D",
            },
            camera_depth_ranges={side: (0.07, 0.5) for side in ("left", "right")},
            camera_max_edge_m={side: 0.035 for side in ("left", "right")},
            base_drive=True,
            linear_speed=0.15,
            angular_speed=0.25,
            map_voxel_size_m=0.05,
            max_depth_m=8,
            max_edge_m=0.25,
        ),
    )
    .remappings(
        [
            (R1ProConnection, "cmd_vel", "r1pro_cmd_vel"),
            (R1ProConnection, "position_command", "r1pro_position_command"),
            (R1ProSpatialTeleop, "cmd_vel", "twist_command"),
            (R1HardwareTeleopCoordinator, "r1pro_joint_state", "joint_state"),
            (R1ProSpatialTeleop, "coordinator_joint_state", "joint_state"),
            (UrdfManager, "coordinator_joint_state", "joint_state"),
            (R1ProSpatialTeleop, "left_controller_output", "left_cartesian_command"),
            (R1ProSpatialTeleop, "right_controller_output", "right_cartesian_command"),
        ]
    )
    .global_config(viewer="none", transport="zenoh", n_workers=9, simulation="")
)
