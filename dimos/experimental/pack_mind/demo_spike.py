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

"""PACK MIND de-risk spike (PLAN §6).

Answers the only question that decides feasibility of the video-only sim:

  Can we drive FrontierPatrolRouter + min_cost_astar on a hand-fabricated
  OccupancyGrid in a plain loop, no robot stack, no CUDA, no ROS?

Plus it proves the A/B knob: two routers sharing ONE VisitationHistory vs two
private ones. Run:

    uv run python dimos/experimental/pack_mind/demo_spike.py
"""

from __future__ import annotations

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.navigation.patrolling.routers.frontier_patrol_router import FrontierPatrolRouter
from dimos.navigation.patrolling.routers.visitation_history import VisitationHistory
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar

RES = 0.1  # m/cell
CLEARANCE_M = 0.3


def make_world(w: int = 40, h: int = 40) -> OccupancyGrid:
    """Free interior, OCCUPIED border + one interior wall. Fully known (no fog)."""
    grid = np.full((h, w), CostValues.FREE, dtype=np.int8)
    grid[0, :] = grid[-1, :] = CostValues.OCCUPIED
    grid[:, 0] = grid[:, -1] = CostValues.OCCUPIED
    grid[10:30, 20] = CostValues.OCCUPIED  # vertical divider with a gap below row 30
    return OccupancyGrid(grid=grid, resolution=RES)


def pose_at(x: float, y: float) -> PoseStamped:
    p = PoseStamped()
    p.position.x = x
    p.position.y = y
    return p


def main() -> None:
    world = make_world()
    print(f"[spike] world: {world.width}x{world.height} @ {RES}m  ({world})")

    # --- 1. router accepts a fabricated grid + odom, returns a goal ---
    r = FrontierPatrolRouter(CLEARANCE_M)
    r.handle_occupancy_grid(world)
    r.handle_odom(pose_at(0.5, 0.5))
    goal = r.next_goal()
    assert goal is not None, "FAIL: next_goal returned None on a fresh map"
    print(f"[spike] PASS next_goal -> ({goal.position.x:.2f}, {goal.position.y:.2f})")

    # --- 2. A* plans start->goal with python fallback (use_cpp=False) ---
    path = min_cost_astar(
        world, (goal.position.x, goal.position.y), (0.5, 0.5), use_cpp=False
    )
    assert path is not None and len(path.poses) > 0, "FAIL: A* found no path"
    print(f"[spike] PASS min_cost_astar -> {len(path.poses)} waypoints")

    # --- 3. THE A/B KNOB: shared vs private VisitationHistory ---
    # Private: two routers, independent visited masks.
    a_priv, b_priv = FrontierPatrolRouter(CLEARANCE_M), FrontierPatrolRouter(CLEARANCE_M)
    for rr in (a_priv, b_priv):
        rr.handle_occupancy_grid(world)
    a_priv.handle_odom(pose_at(0.5, 0.5))
    # A walks a bit; B should NOT see A's coverage.
    for x in np.arange(0.5, 2.5, 0.1):
        a_priv.handle_odom(pose_at(float(x), 0.5))
    b_sees_via_private = b_priv._visited.sum() if b_priv._visited is not None else 0

    # Shared: B reuses A's VisitationHistory instance.
    a_sh, b_sh = FrontierPatrolRouter(CLEARANCE_M), FrontierPatrolRouter(CLEARANCE_M)
    shared = VisitationHistory(CLEARANCE_M)
    a_sh._visitation = b_sh._visitation = shared
    for rr in (a_sh, b_sh):
        rr.handle_occupancy_grid(world)
    for x in np.arange(0.5, 2.5, 0.1):
        a_sh.handle_odom(pose_at(float(x), 0.5))
    b_sees_via_shared = b_sh._visited.sum() if b_sh._visited is not None else 0

    print(
        f"[spike] A/B knob: B sees {b_sees_via_private} visited cells (private) "
        f"vs {b_sees_via_shared} (shared)"
    )
    assert b_sees_via_private == 0, "FAIL: private leaked coverage"
    assert b_sees_via_shared > 0, "FAIL: shared did not propagate A's coverage to B"
    print("[spike] PASS shared VisitationHistory propagates; private does not")

    print("\n[spike] ALL GREEN — PLAN is buildable as written.")


if __name__ == "__main__":
    main()
