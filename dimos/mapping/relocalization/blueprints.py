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

"""Relocalization against a premap, driven by a recording instead of a robot.

    dimos run relocalize-mid360

Replays a mid360 walk's lidar and tf at wall-clock rate into the Go2 mapper and
relocalizer. In Rerun, `loaded_map` appears once a fix lands, on top of
`global_map` when the fix is right. Same dataset as ``lidar/tune.py``.
"""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import Out
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.relocalization.module import PremapModule
from dimos.memory.replay_module import ReplayModule, ReplayModuleConfig
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.visualization.vis_module import vis_module

WORLD = "odom"


class RecordingPlayerConfig(ReplayModuleConfig):
    dataset: str = "go2_mid360_sf_office_outdoors_2026-05-29"  # recording stem or path; `.db`, LFS-fetched on miss


class RecordingPlayer(ReplayModule):
    """Replay a recording's lidar and tf exactly as the sensors published them."""

    config: RecordingPlayerConfig
    pointlio_lidar: Out[PointCloud2]
    tf: Out[TFMessage]


def _fine_points(cloud: Any) -> Any:
    """The premap is millimetre-scale; draw it at that size, not the 5 cm default."""
    return cloud.to_rerun(voxel_size=0.0015)


# Off until asked for.
HIDDEN = tuple(f"world/{name}" for name in ("global_map", "lidar", "region_bounds"))


def _view() -> Any:
    """The default 3D view, with the noisy entities off."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
            overrides={path: rrb.EntityBehavior(visible=False) for path in HIDDEN},
        ),
    )


relocalize_mid360 = autoconnect(
    RecordingPlayer.blueprint().remappings([(RecordingPlayer, "pointlio_lidar", "lidar")]),
    RayTracingVoxelMap.blueprint(voxel_size=0.1, world_frame=WORLD, global_emit_every=5).remappings(
        [(RayTracingVoxelMap, "loaded_map", "loaded_map_unused")]
    ),
    # Scans are recorded in the sensor frame, so match the mapper's registered local map.
    PremapModule.blueprint(world_frame=WORLD, live_cloud="local_map"),
    vis_module(
        "rerun",
        {"visual_override": {"world/loaded_map": _fine_points}, "blueprint": _view},
    ),
).global_config(n_workers=5, robot_model="relocalize_mid360")
