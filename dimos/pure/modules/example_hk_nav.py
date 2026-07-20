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

"""End-to-end pure nav pipeline over the go2 hongkong-office recording.

    hk lidar ─▶ VoxelMapper ─▶ PureCostMapper ─▶ Planner  (goal: origin)
       └▶ odom ─▶ OdomTf ──────────────────────▶ (planner position)
                          all stages ─▶ NavRerunSink  (one recording)

Every module is a plain :class:`dimos.pure.PureModule`, so this wiring runs live
(ports bound to topics) or over a recording — here, over the stored streams. The
fan-out to the sink (each stage feeds both the next module and the sink) is
``itertools.tee`` — lazy: the sink pulls one frame, which pulls one map / one
costmap / one plan through the chain. Nothing is materialized; peak memory is one
message per stage in flight, never a list of clouds.

Run::

    python -m dimos.pure.modules.example_hk_nav --sink spawn      # launch a viewer
    python -m dimos.pure.modules.example_hk_nav --sink grpc       # connect a running viewer
    python -m dimos.pure.modules.example_hk_nav --sink save --out /tmp/nav.rrd
"""

from __future__ import annotations

import argparse
from itertools import tee

from dimos import pure as pm
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.pure.modules.costmapper import PureCostMapper
from dimos.pure.modules.odom_tf import OdomTf
from dimos.pure.modules.planner import Planner
from dimos.pure.modules.rerun_tap import _Sink, open_sink, render_fields
from dimos.pure.modules.voxel_mapper import VoxelMapper
from dimos.utils.data import get_data

WORLD = "world"


class NavRerunSink(pm.PureModule):
    """Render a nav scene: log every declared input under its port name, one recording."""

    entity_path: str = WORLD
    app_id: str = "dimos"
    recording_id: str | None = "hk_nav"
    timeline: str = "time"
    sink: str = "grpc"
    url: str | None = None
    save_path: str | None = None

    class In(pm.In):
        map: PointCloud2 = pm.tick(expect_hz=0.5)  # the driving stage
        costmap: OccupancyGrid | None = pm.latest(default=None)
        path: Path | None = pm.latest(default=None)
        robot: Transform | None = pm.latest(default=None)

    class Out(pm.Out):
        n: int  # render receipt (terminal sink); left unbound live, so nothing re-encodes

    class State(pm.State):
        n: int = 0

    @pm.resource
    def rec(self) -> _Sink:
        """Open the recording; all ports log into this one stream (one .rrd / one recording)."""
        return open_sink(
            app_id=self.app_id,
            recording_id=self.recording_id,
            sink=self.sink,
            url=self.url,
            save_path=self.save_path,
            timeline=self.timeline,
        )

    def step(self, s: State, i: In) -> tuple[State, Out]:
        """Log every present input under entity_path/<port>; count the frame."""
        render_fields(self.rec, self.entity_path, i)
        return s.replace(n=s.n + 1), NavRerunSink.Out(n=s.n + 1)


def run(scans: int, sink: str, out: str | None) -> int:
    """Wire hk lidar/odom through the pure nav stack into one rerun recording. Returns frames."""
    path = get_data("go2_hongkong_office.db")
    with SqliteStore(path=str(path)) as store:
        lidar = store.stream("lidar", PointCloud2).range_seek(0, scans)
        odom = store.stream("odom", PoseStamped).range_seek(0, scans * 4)
        tf_items = [r.tf for r in OdomTf().over(odom=odom)]  # small; planner tf + sink robot

        goal = PoseStamped(frame_id=WORLD, position=[0.0, 0.0, 0.0])
        goal.ts = 1.0  # below every recording ts, so the planner's latest() sees it from tick 1

        # Lazy DAG: tee fans each stage to both the next module and the sink. One frame in
        # flight — pulling the sink pulls one map -> one costmap -> one plan through the chain.
        maps = (r.global_map for r in VoxelMapper(voxel_size=0.1, emit_every=20).over(scan=lidar))
        map_cost, map_sink = tee(maps)
        costmaps = (r.global_costmap for r in PureCostMapper().over(global_map=map_cost))
        cost_plan, cost_sink = tee(costmaps)
        paths = (r.path for r in Planner().over(costmap=cost_plan, goal=[goal], tf=tf_items))

        rows = NavRerunSink(sink=sink, save_path=out).over(
            map=map_sink, costmap=cost_sink, path=paths, robot=tf_items
        )
        n = sum(1 for _ in rows)  # drive the terminal — pulls the whole lazy chain

    print(f"rendered {n} frames")
    return n


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scans", type=int, default=120, help="lidar scans to replay")
    parser.add_argument("--sink", choices=("grpc", "spawn", "save"), default="spawn")
    parser.add_argument("--out", default=None, help=".rrd path for --sink save")
    args = parser.parse_args()
    run(args.scans, args.sink, args.out)


if __name__ == "__main__":
    main()
