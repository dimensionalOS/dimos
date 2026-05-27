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

"""Native DimOS blueprint for the PACK MIND A/B coverage race.

    dimos run pack-mind-sim --viewer rerun

Runs PACK (shared coverage memory) and INDEPENDENT (private) side by side and
publishes each one's live coverage as an OccupancyGrid. The Rerun bridge (added
automatically by ``--viewer rerun``) subscribes to the bus and renders both:
walls = OCCUPIED (dark), searched = FREE (light), unsearched = UNKNOWN (gray).
Watch the PACK map fill cleanly while INDEPENDENT re-walks the same corridors.
"""

from __future__ import annotations

import threading

import numpy as np

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.experimental.pack_mind.sim import PackSim, build_sim
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid

_TICK_DT = 0.005  # seconds between sim ticks (animation pace in the viewer)
_PUB_EVERY = 8  # publish a coverage grid every N ticks
_HOLD_S = 4.0  # hold the final frame before replaying the race
_SEED = 0


def _coverage_grid(sim: PackSim, frame_id: str) -> OccupancyGrid:
    """Encode coverage into OccupancyGrid semantics for the native colormap:
    OCCUPIED=wall (dark), FREE=searched (light), UNKNOWN=unsearched (gray)."""
    g = sim.world.grid
    free = g == CostValues.FREE
    out = np.full(g.shape, CostValues.UNKNOWN, dtype=np.int8)
    out[g == CostValues.OCCUPIED] = CostValues.OCCUPIED
    union: np.ndarray | None = None
    for rb in sim.robots:
        v = rb.router._visited
        if v is None:
            continue
        union = v.copy() if union is None else (union | v)
    if union is not None:
        out[union & free] = CostValues.FREE
    return OccupancyGrid(grid=out, resolution=sim.world.info.resolution, frame_id=frame_id)


class PackMindSimModule(Module):
    """Steps both A/B sims in a background thread and publishes their coverage."""

    pack_coverage: Out[OccupancyGrid]
    indep_coverage: Out[OccupancyGrid]

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_evt.set()
        if getattr(self, "_thread", None) is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        super().stop()

    def _publish(self, pack: PackSim, indep: PackSim) -> None:
        self.pack_coverage.publish(_coverage_grid(pack, "pack"))
        self.indep_coverage.publish(_coverage_grid(indep, "independent"))

    def _loop(self) -> None:
        while not self._stop_evt.is_set():
            pack = build_sim(shared=True, seed=_SEED)
            indep = build_sim(shared=False, seed=_SEED)
            self._publish(pack, indep)
            while not self._stop_evt.is_set():
                done_p = all(rb.done for rb in pack.robots)
                done_i = all(rb.done for rb in indep.robots)
                if not done_p:
                    pack.step()
                if not done_i:
                    indep.step()
                if pack.tick_n % _PUB_EVERY == 0 or indep.tick_n % _PUB_EVERY == 0:
                    self._publish(pack, indep)
                if done_p and done_i:
                    break
                self._stop_evt.wait(_TICK_DT)
            self._publish(pack, indep)
            self._stop_evt.wait(_HOLD_S)  # hold final frame, then replay


pack_mind_sim = autoconnect(PackMindSimModule.blueprint())
