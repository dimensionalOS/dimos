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

"""PACK MIND sim — PackSim orchestrator + SimResult (G1/G2).

Honest A/B experiment: PACK and INDEPENDENT use the SAME robots, SAME start
point, SAME map and survivors. The ONLY difference is whether the robots share
one VisitationHistory (the coverage memory) or each keep a private one. So any
performance gap is attributable to shared memory alone — not to start positions.

PACK (shared=True): one shared VisitationHistory. Each robot's CoveragePatrolRouter
    sees teammates' coverage and steers toward still-uncovered area → the pack
    fans out from the single entrance and divides the map with little overlap.

INDEPENDENT (shared=False): private VisitationHistory per robot. Each robot
    believes the whole map is uncovered → they redundantly re-walk the same
    corridors → far more ticks to full coverage.

CLI:
    python -m dimos.experimental.pack_mind.sim --mode pack --seed 0
    python -m dimos.experimental.pack_mind.sim --mode independent --seed 0
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field

import numpy as np

from dimos.experimental.pack_mind.sim_robot import SimRobot
from dimos.experimental.pack_mind.world import free_cell_count, make_maze_world, plant_survivors
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.navigation.patrolling.routers.coverage_patrol_router import CoveragePatrolRouter
from dimos.navigation.patrolling.routers.visitation_history import VisitationHistory

CLEARANCE_M = 0.1
DETECT_R = 0.4
_STALL_TICKS = 200  # stop a run once coverage has not improved for this many ticks
_STALL_EPS = 0.001


@dataclass
class Finding:
    survivor_xy: tuple[float, float]
    by: str
    tick: int


@dataclass
class SimResult:
    mode: str
    ticks: int
    coverage: float
    found: int
    total_survivors: int
    ticks_to_target: int | None  # ticks to reach coverage_target
    ticks_to_all: int | None  # ticks to find every survivor
    coverage_target: float = 0.75
    findings: list[Finding] = field(default_factory=list)

    def __str__(self) -> str:
        tt = self.ticks_to_target if self.ticks_to_target is not None else "never"
        tall = self.ticks_to_all if self.ticks_to_all is not None else "never"
        return (
            f"SimResult(mode={self.mode}, ticks={self.ticks}, "
            f"coverage={self.coverage:.1%}, found={self.found}/{self.total_survivors}, "
            f"ticks_to_{self.coverage_target:.0%}={tt}, ticks_to_all={tall})"
        )


class PackSim:
    """Run N robots on a shared (pack) or private (independent) coverage mission."""

    def __init__(
        self,
        world: OccupancyGrid,
        survivors: list[tuple[float, float]],
        n_robots: int,
        shared: bool,
        starts: list[tuple[float, float]],
        seed: int = 0,
        kill_at_tick: int | None = None,
    ) -> None:
        np.random.seed(seed)  # CoveragePatrolRouter samples candidates via np.random
        self.world = world
        self.survivors = survivors
        self.shared = shared
        self.kill_at_tick = kill_at_tick
        self._free_total = free_cell_count(world)
        self.findings: list[Finding] = []
        self._found: set[int] = set()
        self.tick_n = 0

        routers = [CoveragePatrolRouter(CLEARANCE_M) for _ in range(n_robots)]
        if shared:
            # Shared coverage memory: set the SAME VisitationHistory on every router
            # BEFORE SimRobot.__init__ calls handle_occupancy_grid (which initialises
            # the mask). Robot B then immediately sees robot A's footprint.
            shared_vis = VisitationHistory(CLEARANCE_M)
            for r in routers:
                r._visitation = shared_vis
        # VisitationHistory is built for *patrolling*: at 50% saturation it prunes the
        # oldest half of visited points (bounded memory), capping coverage at ~50%.
        # A one-shot SAR search must remember everything — disable pruning. Also widen
        # candidate sampling so the router targets the last uncovered pockets.
        for r in routers:
            r._visitation._saturation_threshold = 10.0
            r._candidates_to_consider = 24

        self.robots = [
            SimRobot(f"r{i}", routers[i], world, starts[i]) for i in range(n_robots)
        ]

    def _coverage(self) -> float:
        """Fraction of free cells covered (union of robots' visited masks)."""
        union: np.ndarray | None = None
        free = self.world.grid == CostValues.FREE
        for rb in self.robots:
            v = rb.router._visited
            if v is None:
                continue
            union = v.copy() if union is None else (union | v)
        if union is None:
            return 0.0
        return float(np.count_nonzero(union & free)) / max(1, self._free_total)

    def _detect(self) -> None:
        for si, s in enumerate(self.survivors):
            if si in self._found:
                continue
            for rb in self.robots:
                if rb.done:
                    continue
                if (rb.x - s[0]) ** 2 + (rb.y - s[1]) ** 2 <= DETECT_R**2:
                    self._found.add(si)
                    self.findings.append(Finding(s, rb.name, self.tick_n))
                    break

    def step(self) -> None:
        self.tick_n += 1
        # Death/persistence beat: kill the first robot at the given tick. Its
        # coverage + findings remain (shared mask / findings pool outlive it).
        if self.kill_at_tick is not None and self.tick_n == self.kill_at_tick:
            self.robots[0].done = True
        for rb in self.robots:
            rb.tick()
        self._detect()

    def run(self, max_ticks: int = 4000, coverage_target: float = 0.75) -> SimResult:
        t_target: int | None = None
        tall: int | None = None
        best_cov = 0.0
        stall = 0
        for _ in range(max_ticks):
            self.step()
            cov = self._coverage()
            if t_target is None and cov >= coverage_target:
                t_target = self.tick_n
            if tall is None and len(self._found) == len(self.survivors):
                tall = self.tick_n
            if cov > best_cov + _STALL_EPS:
                best_cov = cov
                stall = 0
            else:
                stall += 1
            if all(rb.done for rb in self.robots):
                break
            if stall >= _STALL_TICKS:
                break
        return SimResult(
            "pack" if self.shared else "independent",
            self.tick_n,
            self._coverage(),
            len(self._found),
            len(self.survivors),
            t_target,
            tall,
            coverage_target,
            list(self.findings),
        )


def build_sim(
    shared: bool,
    seed: int = 0,
    n_robots: int = 3,
    n_survivors: int = 4,
    kill_at_tick: int | None = None,
) -> PackSim:
    """Build a sim. Both modes share IDENTICAL starts (single SAR entry point);
    the only variable is shared vs private memory."""
    world = make_maze_world()
    survivors = plant_survivors(world, n_survivors, seed=seed)
    # Single deploy point at the bottom-left entrance, tiny offsets to avoid overlap.
    starts = [(0.5 + 0.2 * i, 0.4) for i in range(n_robots)]
    return PackSim(world, survivors, n_robots, shared, starts, seed, kill_at_tick)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="PACK MIND sim CLI")
    p.add_argument("--mode", choices=["pack", "independent"], default="pack")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--render", metavar="PATH", default=None)
    p.add_argument("--max-ticks", type=int, default=4000)
    return p.parse_args()


if __name__ == "__main__":
    import time

    args = _parse_args()
    t0 = time.perf_counter()
    sim = build_sim(shared=args.mode == "pack", seed=args.seed)
    result = sim.run(max_ticks=args.max_ticks)
    wall = time.perf_counter() - t0
    print(result)
    print(f"wall time: {wall:.1f}s")

    if args.render:
        from dimos.experimental.pack_mind.render import render_ab

        render_ab(seed=args.seed, out_path=args.render)
        print(f"rendered -> {args.render}")
