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

"""NavRerunSink: render a pure nav scene into one rerun recording.

A terminal pure-module sink — every declared In port with a ``to_rerun`` payload
logs under ``entity_path/<port>`` at the row's ts. Feed it by name: wire ports
directly (``over(map=..., path=...)``) or fan a graph's exports into it with
``run.save(NavRerunSink(...))``, which matches exported output names to In ports.
"""

from __future__ import annotations

from dimos import pure as pm
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.GraphNodes3D import GraphNodes3D
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.pure.modules.rerun_tap import _Sink, open_sink, render_fields

__all__ = ["NavRerunSink"]


class NavRerunSink(pm.PureModule):
    """Render a nav scene: log every declared input under its port name, one recording."""

    entity_path: str = "world"
    app_id: str = "dimos"
    recording_id: str | None = "hk_nav"
    timeline: str = "time"
    sink: str = "grpc"
    url: str | None = None
    save_path: str | None = None

    class In(pm.In):
        # raw scans drive the render: lidar-rate frames, everything else sampled.
        # (save() feeds `lidar` straight from the graph's rim input — spec §0.5.)
        lidar: PointCloud2 = pm.tick(expect_hz=5.0)
        map: PointCloud2 | None = pm.latest(default=None)
        costmap: OccupancyGrid | None = pm.latest(default=None)
        path: Path | None = pm.latest(default=None)
        robot: Transform | None = pm.latest(default=None)
        keyframes: GraphNodes3D | None = pm.latest(default=None)  # pgo debug viz
        loop_closures: LineSegments3D | None = pm.latest(default=None)

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
