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

Replays a mid360 walk's lidar and tf at wall-clock rate into the same mapper
the Go2 stack runs, so the relocalizer sees exactly the streams it would on
hardware, and has to find the walk inside a premap built from it. Watch it in
Rerun: `loaded_map` appears only once a fix lands, and lands on top of
`global_map` when the fix is right.

The recording and the premap are the eval's dataset (``lidar/tune.py``,
``lidar/tune.md``), so a demo that looks wrong and an eval that scores
badly are the same bug.
"""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.relocalization.lidar.module import LocalMapRelocalization
from dimos.memory.replay import ReplayStream
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger
from dimos.visualization.vis_module import vis_module

logger = setup_logger()

# The walk, and a premap built from the same walk's second half. Overlapping
# but not identical: the relocalizer has to place the live scans, not
WORLD = "odom"


class RecordingPlayerConfig(ModuleConfig):
    dataset: str = "go2_mid360_sf_office_outdoors_2026-05-29"  # recording stem or path; `.db`, LFS-fetched on miss
    stream: str = "pointlio_lidar"
    speed: float = 1.0
    seek: float | None = None
    duration: float | None = None


class RecordingPlayer(Module):
    """Replay a recording's lidar and tf exactly as the sensors published them."""

    config: RecordingPlayerConfig
    lidar: Out[PointCloud2]
    tf: Out[TFMessage]

    @rpc
    def start(self) -> None:
        super().start()
        path = resolve_named_path(self.config.dataset, ".db")
        store = self.register_disposable(SqliteStore(path=str(path), must_exist=True))
        store.start()
        replay = store.replay(
            speed=self.config.speed,
            seek=self.config.seek,
            duration=self.config.duration,
        )
        lidar: ReplayStream[PointCloud2] = replay.stream(self.config.stream)
        logger.info(
            f"Replaying {path.name}:{self.config.stream} "
            f"({lidar.count()} frames at {self.config.speed}x)"
        )
        self.register_disposable(lidar.observable().subscribe(self.lidar.publish))

        tf: ReplayStream[TFMessage] = replay.stream("tf")
        self.register_disposable(tf.observable().subscribe(self.tf.publish))


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
    RecordingPlayer.blueprint(),
    RayTracingVoxelMap.blueprint(voxel_size=0.1, world_frame=WORLD, global_emit_every=5),
    # LocalMapRelocalization and not LidarWindowRelocalization: these recordings
    # carry the scans in the sensor frame, as the sensor published them, and the
    # window one wants them registered. The mapper is what registers them here.
    LocalMapRelocalization.blueprint(world_frame=WORLD),
    vis_module(
        "rerun",
        {"visual_override": {"world/loaded_map": _fine_points}, "blueprint": _view},
    ),
).global_config(n_workers=5, robot_model="relocalize_mid360")
