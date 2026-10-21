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

"""Alfred navigating off the Mid-360 alone, with Point-LIO for odometry and premap relocalization.

dimos run alfred-nav
"""

from __future__ import annotations

from functools import partial
from pathlib import Path
from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.pointlio_blueprints import mid360_for_pointlio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.relocalization.lidar.module import LocalMapRelocalization
from dimos.mapping.relocalization.lidar.relocalize import MID360_NAV
from dimos.navigation.embodiment.alfred import ALFRED
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.global_planner.mls_planner.viz import planner_visual_override
from dimos.navigation.local_planner.native import LocalPlannerNative
from dimos.navigation.local_planner.viz import motion_visual_override
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.trajectory_follower.fancy.native import TrajectoryFollowerNative
from dimos.robot.diy.alfred.blueprints.alfred_hardware import _alfred_lidar_hardware
from dimos.robot.diy.alfred.config import ALFRED as ALFRED_CONFIG, LIO_BLIND_M, alfred_model
from dimos.visualization.rerun.urdf_robot import UrdfRobotStaticRerunFactory
from dimos.visualization.vis_module import vis_module

ALFRED_RERUN_ROOT = "world/alfred"

VOXEL_SIZE_METERS = 0.08
WALL_CLEARANCE_M = 0.20
STEP_THRESHOLD_M = 0.08
BODY_DILATE_M = 0.0
PLANNER_VIZ_HZ = 2.0
MID360_MAX_RANGE_METERS = 15.0
"""The Mid-360 reaches far past this, but the voxel map raytraces every point from the
sensor origin, so the far returns cost the most and carve the least reliable space."""


def _path_colored(msg: Any, color: tuple[int, int, int]) -> Any:
    return msg.to_rerun(color=color)


def _resolved_urdf() -> Path:
    """The model written out with package:// resolved, which the rerun urdf loader cannot do."""
    loaded = alfred_model().load()
    path = loaded.source_path.parent / ".resolved" / loaded.source_path.name
    path.parent.mkdir(exist_ok=True)
    path.write_text(loaded.xml)
    return path


def _alfred_urdf_static(rr: Any) -> list[tuple[str, Any]]:
    factory = UrdfRobotStaticRerunFactory(urdf_path=_resolved_urdf(), root_path=ALFRED_RERUN_ROOT)
    return [
        *factory(rr),
        (ALFRED_RERUN_ROOT, rr.Transform3D(parent_frame="tf#/base_link")),
    ]


def _rerun_blueprint() -> Any:
    """The default 3D view with the premap hidden until ticked on."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
            overrides={"world/loaded_map": rrb.EntityBehavior(visible=False)},
        ),
    )


def _alfred_rerun_config() -> dict[str, Any]:
    """The rerun bridge config every Alfred navigation blueprint shares."""
    return {
        "blueprint": _rerun_blueprint,
        "static": {ALFRED_RERUN_ROOT: _alfred_urdf_static},
        # Keyed by the topic's entity path, before any visual_override renames it.
        # Everything is held to 1 Hz: the viewer rides a wifi link, and the
        # uncapped clouds put it seconds behind live. tf is capped too, so the
        # whole scene ticks once a second rather than re-posing per odom update.
        "max_hz": {
            "world/tf": 1.0,
            "world/lidar": 1.0,
            "world/global_map": 1.0,
            "world/local_map": 1.0,
            "world/nodes": 1.0,
        },
        "visual_override": {
            "world/planner_path": partial(_path_colored, color=(170, 60, 220)),
            **motion_visual_override(embodiment=ALFRED, body_dilate_m=BODY_DILATE_M),
            # The planner publishes its surface and edges by cell.
            **planner_visual_override(PLANNER_VIZ_HZ, VOXEL_SIZE_METERS, WALL_CLEARANCE_M),
        },
    }


def _alfred_nav(*, map_cloud_topic: str, map_max_range: float) -> Blueprint:
    """Mapping, planning and following. map_cloud_topic is what the voxel map raytraces."""
    return autoconnect(
        RayTracingVoxelMap.blueprint(
            voxel_size=VOXEL_SIZE_METERS,
            max_range=map_max_range,
        ).remappings([(RayTracingVoxelMap, "lidar", map_cloud_topic)]),
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
            ]
        ),
        LocalPlannerNative.blueprint(embodiment=ALFRED, body_dilate_m=BODY_DILATE_M),
        TrajectoryFollowerNative.blueprint(embodiment=ALFRED),
        MovementManager.blueprint(),
    )


alfred_nav = autoconnect(
    _alfred_lidar_hardware,
    mid360_for_pointlio(lidar_ip=ALFRED_CONFIG.mid360_ip),
    PointLio.blueprint(blind=LIO_BLIND_M),
    _alfred_nav(map_cloud_topic="lidar", map_max_range=MID360_MAX_RANGE_METERS),
    LocalMapRelocalization.blueprint(world_frame="odom", relocalize=MID360_NAV),
    vis_module(global_config.viewer, rerun_config=_alfred_rerun_config()),
).global_config(n_workers=12, robot_model="alfred")
