#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

"""Demo sensor publisher — a minimal hand-written dimos Module.

Subscribes to /odom (PoseStamped) and publishes /map (nav_msgs/OccupancyGrid)
of a fake 6m room with two obstacles. Pairs with examples/simplerobot for a
richer two-topic demo (e.g. for the dimoscope web viewer in dimos/web/dimoscope).

Modelled on examples/simplerobot/simplerobot.py: declare typed In/Out ports,
then run STANDALONE by assigning LCMTransport (no Blueprint needed).

Run (in the dimos venv):
    python examples/simplerobot/simplerobot.py --headless    # publishes /odom
    python examples/fakesensors.py                           # publishes /map

Note: dimos has no Python LaserScan wrapper, so this only does /map. A /scan
would use dimos_lcm.sensor_msgs.LaserScan directly (left as a stretch).
"""

import time

import numpy as np
import reactivex as rx

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

ROOM = 3.0  # half-width of the room, meters
RES = 0.1  # meters per cell
OBSTACLES = [  # (x0, y0, x1, y1) world-coord boxes
    (0.5, -0.6, 1.6, 0.6),
    (-2.2, 0.8, -1.0, 2.0),
]


def build_grid() -> np.ndarray:
    """A static occupancy grid (height x width) of int8: 100=occupied, 0=free."""
    n = int(ROOM * 2 / RES)  # 60 x 60
    g = np.zeros((n, n), dtype=np.int8)
    for row in range(n):
        for col in range(n):
            wx = -ROOM + (col + 0.5) * RES
            wy = -ROOM + (row + 0.5) * RES
            border = row in (0, n - 1) or col in (0, n - 1)
            occ = border or any(x0 <= wx <= x1 and y0 <= wy <= y1 for (x0, y0, x1, y1) in OBSTACLES)
            g[row, col] = 100 if occ else 0
    return g


class FakeSensorsConfig(ModuleConfig):
    frame_id: str = "world"
    rate: float = 1.0  # Hz to republish the map


class FakeSensors(Module):
    """Publishes a static map; subscribes to odom (just to show an In port)."""

    config: FakeSensorsConfig
    odom: In[PoseStamped]
    map: Out[OccupancyGrid]
    _grid: np.ndarray | None = None

    @rpc
    def start(self) -> None:
        self._grid = build_grid()
        # In port: react to incoming poses (here we only log occasionally).
        self.register_disposable(self.odom.observable().subscribe(self._on_odom))
        # Out port: republish the map at `rate` Hz so late joiners get it.
        self.register_disposable(
            rx.interval(1.0 / self.config.rate).subscribe(lambda _: self._publish_map())
        )

    def _on_odom(self, ps: PoseStamped) -> None:
        # Demonstrates the In[PoseStamped] port is wired; a real module would use it.
        pass

    def _publish_map(self) -> None:
        grid = self._grid if self._grid is not None else build_grid()
        self.map.publish(
            OccupancyGrid(
                grid=grid,
                resolution=RES,
                origin=Pose(-ROOM, -ROOM, 0.0),  # world coord of cell (0,0)
                frame_id=self.config.frame_id,
            )
        )


if __name__ == "__main__":
    from dimos.core.transport import LCMTransport

    mod = FakeSensors()
    # Same standalone pattern as simplerobot.py: assign transports by hand.
    # LCMTransport appends "#<msg_name>", so these become the exact channels:
    # /map#nav_msgs.OccupancyGrid and /odom#geometry_msgs.PoseStamped
    mod.map.transport = LCMTransport("/map", OccupancyGrid)
    mod.odom.transport = LCMTransport("/odom", PoseStamped)
    mod.start()

    print("FakeSensors running.")
    print("  Publishing: /map (OccupancyGrid)")
    print("  Subscribing: /odom (PoseStamped)")
    print("  Pair with: python examples/simplerobot/simplerobot.py --headless")
    print("  Ctrl+C to exit")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        mod.stop()
