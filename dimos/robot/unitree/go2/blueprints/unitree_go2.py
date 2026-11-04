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

"""The Go2 navigating: its URDF, a connection, the ray-traced voxel map, MLS planning, Ivan's local planner,
premap relocalization, and the viewer.

    ROBOT_IP=<ip> dimos run unitree-go2                 # the real robot over WebRTC
    dimos run unitree-go2 --simulation mujoco           # dim-go2-sim's MuJoCo Go2 behind the same ports
    dimos run unitree-go2 --replay.db <x.db>            # a recording (.db or dimos .mcap) in the robot's place

Relocalization stays off until it has a premap: ``--localmaprelocalization.map-file <map>``.
"""

from __future__ import annotations

import numpy as np

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.relocalization.lidar.module import LocalMapRelocalization
from dimos.mapping.relocalization.lidar.relocalize import MID360_NAV
from dimos.memory.replay_native.native import ReplayNative
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.embodiment.go2 import GO2
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.local_planner.native import LocalPlannerNative
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.trajectory_follower.fancy.native import TrajectoryFollowerNative
from dimos.robot.unitree.go2.config import (
    CAMERA_OFFSET,
    go2_model,
    mls_planner_config,
    ray_tracing_config,
)
from dimos.robot.unitree.go2.connections.webrtc import Go2WebRtc
from dimos.robot.unitree.go2.rerun import rerun_config
from dimos.robot.urdf_manager_native.native import UrdfManagerNative
from dimos.visualization.vis_module import vis_module

WORLD_FRAME = "world"

_navigation = autoconnect(
    UrdfManagerNative.blueprint(model=go2_model()).remappings(
        [(UrdfManagerNative, "coordinator_joint_state", "joint_state")]
    ),
    RayTracingVoxelMap.blueprint(
        **ray_tracing_config.model_copy(update={"world_frame": WORLD_FRAME}).model_dump(
            exclude_unset=True
        )
    ),
    MLSPlannerNative.blueprint(
        **mls_planner_config.model_copy(update={"world_frame": WORLD_FRAME}).model_dump(
            exclude_unset=True
        )
    ).remappings(
        [
            (MLSPlannerNative, "global_map", "global_map_unused"),
            (MLSPlannerNative, "path", "planner_path"),
        ]
    ),
    LocalPlannerNative.blueprint(embodiment=GO2, world_frame=WORLD_FRAME),
    TrajectoryFollowerNative.blueprint(embodiment=GO2),
    MovementManager.blueprint(),
    LocalMapRelocalization.blueprint(world_frame=WORLD_FRAME, relocalize=MID360_NAV),
    vis_module(global_config.viewer, rerun_config=rerun_config()),
)


def odom_transforms(pose: PoseStamped) -> TFMessage:
    """The odometry's base_link edge and the camera chain under it, as webrtc.py's Rust module publishes them."""
    camera_link = Transform(
        translation=Vector3(*CAMERA_OFFSET),
        frame_id="base_link",
        child_frame_id="camera_link",
        ts=pose.ts,
    )
    camera_optical = Transform(
        rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
        frame_id="camera_link",
        child_frame_id="camera_optical",
        ts=pose.ts,
    )
    return TFMessage(Transform.from_pose("base_link", pose), camera_link, camera_optical)


def cloud_in_base_link(cloud: PointCloud2, pose: PoseStamped) -> PointCloud2:
    """A world-frame cloud as seen from base_link at *pose*."""
    world_from_base = Transform.from_pose("base_link", pose).to_matrix()
    rotation, translation = world_from_base[:3, :3], world_from_base[:3, 3]
    body = (cloud.points_f32() - translation) @ rotation
    return PointCloud2.from_numpy(body.astype(np.float32), frame_id="base_link", timestamp=cloud.ts)


class Replay(ReplayNative):
    """A Go2 recording in the connection's place: idle until ``--replay.db <x.db|x.mcap>``."""

    lidar: Out[PointCloud2]
    odom: Out[PoseStamped]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    # only these: the recording's other streams would be ports nothing in the blueprint wired
    topics = "lidar,odom,color_image,camera_info"


class Go2RecordingTf(Module):
    """What a Go2 recording lacks to stand in for the connection: the odometry's tf, and the L1 map in base_link."""

    recorded_odom: In[PoseStamped]
    recorded_lidar: In[PointCloud2]
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]
    tf: Out[TFMessage]

    _pose: PoseStamped | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(self.recorded_odom.observable().subscribe(self._on_odom))  # type: ignore[no-untyped-call]
        self.register_disposable(self.recorded_lidar.observable().subscribe(self._on_lidar))  # type: ignore[no-untyped-call]

    def _on_odom(self, pose: PoseStamped) -> None:
        pose.frame_id = WORLD_FRAME
        self._pose = pose
        self.tf.publish(odom_transforms(pose))
        self.odom.publish(pose)

    def _on_lidar(self, cloud: PointCloud2) -> None:
        if self._pose is not None:
            self.lidar.publish(cloud_in_base_link(cloud, self._pose))


def _connection() -> Blueprint:
    """dim-go2-sim's Go2 with --simulation, else the real robot (idle without a robot IP)."""
    if global_config.simulation:
        try:
            from dimos.robot.unitree.go2.connections.webrtc_sim import Go2WebRtcSim
        except ImportError as error:
            reason = f"--simulation needs dim-go2-sim ({error}): uv sync --extra sim"
            return autoconnect().requirements(lambda: reason)
        sim: Blueprint = Go2WebRtcSim.blueprint()
        return sim
    # the L1 map cast from base_link, not world's origin, so the ray tracer carves from the robot
    return Go2WebRtc.blueprint(
        odom_frame_id=WORLD_FRAME, lidar_in_base_link=True, obstacle_avoidance=False
    )


unitree_go2 = autoconnect(
    _navigation,
    _connection(),
    Replay.blueprint().remappings(
        [(Replay, "lidar", "recorded_lidar"), (Replay, "odom", "recorded_odom")]
    ),
    Go2RecordingTf.blueprint(),
).global_config(n_workers=12, robot_model="unitree_go2")
