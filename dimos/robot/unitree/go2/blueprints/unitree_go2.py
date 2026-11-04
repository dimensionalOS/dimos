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
premap relocalization, the viewer, and with ``--agentic`` the agent skills (modules/agentic.py).

    ROBOT_IP=<ip> dimos run unitree-go2                 # the real robot over WebRTC
    dimos run unitree-go2 --simulation mujoco           # dim-go2-sim's MuJoCo Go2 behind the same ports
    dimos run unitree-go2 --replay.db <x.db>            # a recording (.db or dimos .mcap) in the robot's place

Relocalization stays off until it has a premap: ``--localmaprelocalization.map-file <map>``.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import Out
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.relocalization.lidar.module import LocalMapRelocalization
from dimos.mapping.relocalization.lidar.relocalize import MID360_NAV
from dimos.memory.replay_native.native import ReplayNative
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
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
    WORLD_FRAME,
    go2_model,
    mls_planner_config,
    ray_tracing_config,
)
from dimos.robot.unitree.go2.connections.webrtc.module import Go2WebRtc
from dimos.robot.unitree.go2.modules.agentic import agentic
from dimos.robot.unitree.go2.rerun import rerun_config
from dimos.robot.urdf_manager_native.native import UrdfManagerNative
from dimos.visualization.vis_module import vis_module

_navigation = autoconnect(
    UrdfManagerNative.blueprint(model=go2_model()).remappings(
        [(UrdfManagerNative, "coordinator_joint_state", "joint_state")]
    ),
    RayTracingVoxelMap.blueprint(**ray_tracing_config.model_dump(exclude_unset=True)),
    MLSPlannerNative.blueprint(**mls_planner_config.model_dump(exclude_unset=True)).remappings(
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


class Replay(ReplayNative):
    """A Go2 recording in the connection's place: idle until ``--replay.db <x.db|x.mcap>``. The recording carries its
    own tf, and its lidar relative to the sensor, as both connections publish them."""

    lidar: Out[PointCloud2]
    odom: Out[PoseStamped]
    tf: Out[TFMessage]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    # only these: the recording's other streams would be ports nothing in the blueprint wired
    topics = ("lidar", "odom", "tf", "color_image", "camera_info")


def _connection() -> Blueprint:
    """The sim (DimSim's or dim-go2-sim's, see webrtc_sim.py) with --simulation, else the real robot (idle without a robot IP)."""
    if global_config.simulation:
        try:
            from dimos.robot.unitree.go2.connections.webrtc_sim import Go2WebRtcSim
        except ImportError as error:
            reason = f"--simulation {global_config.simulation} can't load its sim ({error}): uv sync --extra sim"
            return autoconnect().requirements(lambda: reason)
        sim: Blueprint = Go2WebRtcSim.blueprint()
        return sim
    return Go2WebRtc.blueprint(obstacle_avoidance=False)


unitree_go2 = autoconnect(
    _navigation,
    _connection(),
    Replay.blueprint(),
    *([agentic()] if global_config.agentic else []),
).global_config(n_workers=16, robot_model="unitree_go2")
