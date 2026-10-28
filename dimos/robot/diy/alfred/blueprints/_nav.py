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

"""Alfred's navigation fragment: voxel map, planners and follower, plus its rerun config."""

from __future__ import annotations

from functools import partial
from pathlib import Path
from typing import TYPE_CHECKING, Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.navigation.embodiment.alfred import ALFRED
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.global_planner.mls_planner.viz import planner_visual_override
from dimos.navigation.local_planner.native import LocalPlannerNative
from dimos.navigation.local_planner.viz import motion_visual_override
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.trajectory_follower.fancy.native import TrajectoryFollowerNative
from dimos.robot.diy.alfred.config import alfred_model
from dimos.visualization.rerun.urdf_robot import UrdfRobotStaticRerunFactory

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype
    import rerun.blueprint as rrb

ALFRED_RERUN_ROOT = "world/alfred"

VOXEL_SIZE_METERS = 0.08
WALL_CLEARANCE_M = 0.20
STEP_THRESHOLD_M = 0.08
PLANNER_VIZ_HZ = 2.0


def _path_colored(msg: NavPath, color: tuple[int, int, int]) -> Archetype:
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


def _rerun_blueprint() -> rrb.Blueprint:
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
        # Keyed by topic entity path, before visual_override renames it. Capped at 1 Hz
        # so the viewer keeps up over wifi.
        "max_hz": {
            "world/tf": 1.0,
            "world/lidar": 1.0,
            "world/global_map": 1.0,
            "world/local_map": 1.0,
            "world/nodes": 1.0,
        },
        "visual_override": {
            "world/planner_path": partial(_path_colored, color=(170, 60, 220)),
            **motion_visual_override(embodiment=ALFRED),
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
            global_emit_every=50,
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
        LocalPlannerNative.blueprint(embodiment=ALFRED, step_speed=0.2),
        TrajectoryFollowerNative.blueprint(embodiment=ALFRED),
        MovementManager.blueprint(),
    )
