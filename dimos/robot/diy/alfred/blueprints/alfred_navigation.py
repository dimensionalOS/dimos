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

"""Alfred on the move: Mid-360 navigation and teleop on the coordinator, the lift parked, no arms.

    dimos run alfred-navigation

The other half of the Orin NX's pair (``alfred-manipulation`` is the bench half): the lidar,
Point-LIO, the voxel map, both planners and the holonomic follower, with the Controller's
"Go here" and teleop. The pillar is connected so it can be homed and parked, but there is no
planner and the arms' CAN is left alone, so the box has its cores for navigation.
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.pointlio_blueprints import mid360_for_pointlio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.relocalization.lidar.module import LocalMapRelocalization
from dimos.mapping.relocalization.lidar.relocalize import MID360_NAV
from dimos.navigation.embodiment.alfred import ALFRED
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.global_planner.mls_planner.start_relay import StartRelay
from dimos.navigation.local_planner.native import LocalPlannerNative
from dimos.robot.diy.alfred.alfred_model import ALFRED_V1_MODEL
from dimos.robot.diy.alfred.blueprints._nav import (
    PLANNER_VIZ_HZ,
    STEP_THRESHOLD_M,
    VOXEL_SIZE_METERS,
    WALL_CLEARANCE_M,
)
from dimos.robot.diy.alfred.blueprints.alfred_base import (
    BASE_FRAME,
    ODOM_FRAME,
    alfred_connections,
    alfred_coordinator,
)
from dimos.robot.diy.alfred.blueprints.alfred_hardware import _alfred_lidar_hardware
from dimos.robot.diy.alfred.blueprints.alfred_nav import MID360_MAX_RANGE_METERS
from dimos.robot.diy.alfred.config import ALFRED_HARDWARE, LIO_BLIND_M
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.robot.diy.alfred.pillar_connection import PillarConnection
from dimos.robot.urdf_manager import UrdfManager


def alfred_nav_modules() -> tuple[Any, ...]:
    """Navigation on Point-LIO: the lidar, odometry, the voxel map, both planners, the base pose
    relay for the coordinator's base adapter, and relocalization."""
    return (
        mid360_for_pointlio(lidar_ip=ALFRED_HARDWARE.mid360_ip),
        PointLio.blueprint(blind=LIO_BLIND_M),
        RayTracingVoxelMap.blueprint(
            voxel_size=VOXEL_SIZE_METERS,
            max_range=MID360_MAX_RANGE_METERS,
            global_emit_every=50,
        ),
        MLSPlannerNative.blueprint(
            voxel_size=VOXEL_SIZE_METERS,
            robot_height=ALFRED.height,
            wall_clearance_m=WALL_CLEARANCE_M,
            step_threshold_m=STEP_THRESHOLD_M,
            step_penalty_weight=1.0,
            viz_publish_hz=PLANNER_VIZ_HZ,
        ).remappings(
            [
                (MLSPlannerNative, "global_map", "global_map_unused"),
                (MLSPlannerNative, "path", "planner_path"),
                # A click is the goal: the Controller's "Go here" publishes clicked_point, and
                # there is no MovementManager here to relay it.
                (MLSPlannerNative, "goal", "clicked_point"),
            ]
        ),
        # Its `path` is what the follower task binds to on the coordinator.
        LocalPlannerNative.blueprint(embodiment=ALFRED, step_speed=0.2),
        # The coordinator reads the base pose off flowbase_odom; the relay puts tf's
        # odom -> base_link there.
        StartRelay.blueprint(world_frame=ODOM_FRAME, base_frame=BASE_FRAME),
        LocalMapRelocalization.blueprint(world_frame=ODOM_FRAME, relocalize=MID360_NAV),
    )


alfred_navigation = (
    autoconnect(
        _alfred_lidar_hardware,
        *alfred_nav_modules(),
        # No Rerun bridge (see alfred_base): the Desktop reads zenoh itself.
        PillarConnection.blueprint(),
        # The XML and meshes for the Controller. Joint tf stays off: Point-LIO owns the lidar's
        # parent edge, and AlfredMountTf hangs the robot under it.
        UrdfManager.blueprint(model=ALFRED_V1_MODEL, publish_joint_tf=False),
        alfred_coordinator(arms=False, follower=True),
    )
    .remappings(alfred_connections(AlfredHighLevel, StartRelay))
    .global_config(n_workers=10, robot_model="alfred")
)
