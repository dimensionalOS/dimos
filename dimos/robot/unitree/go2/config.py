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

"""The Unitree Go2's numbers: its body, its camera, its URDF, and the settings its navigation runs with."""

from __future__ import annotations

from pathlib import Path

from scipy.spatial.transform import Rotation

from dimos.mapping.ray_tracing.module import RayTracingVoxelMapConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNativeConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource

ROBOT_LENGTH = 0.6858
ROBOT_WIDTH = 0.3175
ROBOT_HEIGHT = 0.60
BASE_LINK_HEIGHT = 0.287
CMD_VEL_TIMEOUT = 0.2

# base_link -> camera_link (m)
CAMERA_OFFSET = (0.3, 0.0, 0.0)

# The head L1, which the WebRTC connection's lidar is relative to: base_link -> l1_link, fitted on recordings
# (gyro Kabsch + map fit; the URDF's radar_joint is ~120 deg off), fixed-axis roll/pitch/yaw in degrees.
L1_FRAME = "l1_link"
L1_XYZ = (0.226, 0.025, 0.089)
L1_RPY_DEG = (-167.877, -7.907, -59.588)


def l1_mount() -> tuple[float, ...]:
    """base_link -> l1_link as x, y, z then a quaternion x, y, z, w."""
    quaternion = Rotation.from_euler("xyz", L1_RPY_DEG, degrees=True).as_quat()
    return (*L1_XYZ, *(float(q) for q in quaternion))


ASSETS = Path(__file__).parent / "assets"


def front_camera_info() -> CameraInfo:
    return CameraInfo.from_yaml(str(ASSETS / "front_camera_720.yaml"))


# base_link -> camera_link -> camera_optical, the chain the connections publish under the odometry
BASE_TO_OPTICAL: Transform = Transform(
    translation=Vector3(*CAMERA_OFFSET),
    frame_id="base_link",
    child_frame_id="camera_link",
) + Transform(
    rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
    frame_id="camera_link",
    child_frame_id="camera_optical",
)


_UNITREE_ROS = RobotDescriptionSource(
    url="https://github.com/unitreerobotics/unitree_ros",
    ref="25746e8c73f097c3b32bed7b6c16d4e195dfacd8",
)


def go2_model() -> RobotModel:
    """The Go2's URDF (assets/go2.urdf), its meshes from unitree_ros (cloned into the asset cache on first load)."""
    return RobotModel.from_file(
        ASSETS / "go2.urdf",
        package_paths={"go2_description": _UNITREE_ROS / "robots" / "go2_description"},
    )


# The lowstate's 12 motors, in motor order, as the URDF names them.
JOINT_NAMES = tuple(
    f"{leg}_{part}_joint" for leg in ("FR", "FL", "RR", "RL") for part in ("hip", "thigh", "calf")
)

# The fixed frame the Go2's odometry, mapping and planning share.
WORLD_FRAME = "world"

voxel_size = 0.08
wall_clearance_m = 0.1
planner_viz_hz = 2.0
viz_sweep_cells = 1

ray_tracing_config = RayTracingVoxelMapConfig(
    world_frame=WORLD_FRAME,
    voxel_size=voxel_size,
    global_emit_every=50,
    viz_emit_every=5,
    viz_sweep_cells=viz_sweep_cells,
)

mls_planner_config = MLSPlannerNativeConfig(
    world_frame=WORLD_FRAME,
    voxel_size=voxel_size,
    robot_height=ROBOT_HEIGHT,
    start_z_offset_m=BASE_LINK_HEIGHT,
    wall_clearance_m=wall_clearance_m,
    viz_publish_hz=planner_viz_hz,
    viz_sweep_cells=viz_sweep_cells,
)
