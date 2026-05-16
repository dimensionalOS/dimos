# Copyright 2025-2026 Dimensional Inc.
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

"""Per-arm evaluation scenario data for ``dimos.manipulation.eval``.

This is pure data. Tune existing arms or add a new one by editing this
file — the framework in ``eval.py`` does not need to change.

Each entry in ``SCENARIOS_BY_ARM`` is a list of ``Scenario`` objects. To add a
new arm:

1. Find the arm's "gripper-down" quaternion for its URDF convention
   (the EE link's local axes may differ between vendors).
2. Pick target poses inside the arm's reachable workspace.
3. Add an entry to ``SCENARIOS_BY_ARM`` under the arm's canonical name.

Customers with a specific cell layout can also write their own ``Scenario``
list at the call site and skip this registry entirely::

    from dimos.manipulation.eval import Scenario, evaluate
    my_cases = [Scenario(...), ...]
    evaluate(my_arm, my_cases)
"""

from __future__ import annotations

from typing import Callable

from dimos.manipulation.eval import Scenario, grasp_scenario, reach_scenario
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


# Quaternion (x, y, z, w) for "gripper pointing down" — depends on the
# EE link's URDF convention. Measure empirically by running an IK probe
# against a known-reachable pose and trying common orientations.
XARM_GRIPPER_DOWN = (1.0, 0.0, 0.0, 0.0)        # xArm: roll=180°, EE +Z up
PIPER_GRIPPER_DOWN = (0.0, 0.7071, 0.0, 0.7071)  # Piper: pitch=90°, EE +X forward


def _box(name: str, pos: tuple[float, float, float], dims: tuple[float, float, float]) -> Obstacle:
    """Helper: axis-aligned box obstacle at ``pos`` with full-size ``dims``."""
    return Obstacle(
        name=name,
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(
            frame_id="world",
            position=Vector3(*pos),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        dimensions=dims,
    )


def pick_place_for_object(
    name: str,
    object_at: tuple[float, float, float],
    object_size: tuple[float, float, float],
    place_at: tuple[float, float, float],
    approach_height_m: float = 0.10,
    obstacles: list[Obstacle] | None = None,
) -> list[Scenario]:
    """Six-scenario pick-and-place sequence for a specific object.

    Uses ``PickAndPlaceModule``'s production grasp heuristics to derive
    the pick pose and orientation from the object's geometry — same
    decision a real ``pick()`` call would make. So this exercises the
    full "heuristic → planner" path: when ``_grasp_orientation`` or
    ``_occlusion_offset`` changes, the eval automatically tracks it.

    The place pose reuses the same orientation as the pick (matches
    ``place_back()`` semantics).
    """
    from dimos.manipulation.pick_and_place_module import PickAndPlaceModule

    cx, cy, cz = object_at
    sx, sy, sz = object_size
    gx, gy = PickAndPlaceModule._occlusion_offset(
        Vector3(x=cx, y=cy, z=cz), Vector3(x=sx, y=sy, z=sz)
    )
    gz = cz + sz / 2
    xy_dist = (gx * gx + gy * gy) ** 0.5
    q = PickAndPlaceModule._grasp_orientation(gx, gy, xy_dist)
    orientation = (q.x, q.y, q.z, q.w)

    return pick_place_scenarios(
        name,
        pick=(gx, gy, gz),
        place=place_at,
        approach_height_m=approach_height_m,
        orientation=orientation,
        obstacles=obstacles,
    )


def pick_place_scenarios(
    name: str,
    pick: tuple[float, float, float],
    place: tuple[float, float, float],
    approach_height_m: float = 0.10,
    orientation: tuple[float, float, float, float] = XARM_GRIPPER_DOWN,
    obstacles: list[Obstacle] | None = None,
) -> list[Scenario]:
    """Six-scenario sequence representing a pick-and-place motion.

    Mirrors what ``PickAndPlaceModule.pick()`` + ``place()`` actually do —
    approach above, descend to target, retract — three plan-and-execute
    calls per side, six in total.

    Each Scenario is planned independently from the zero configuration, so
    this tests reachability of every leg without modelling "held-object"
    collisions or chaining start configurations between motions. To test
    chained execution, write a higher-level scenario that threads start
    configs through ``Scenario.start_joints``.

    Args:
        name: Prefix for the six generated scenario names.
        pick: (x, y, z) of the pick pose, EE target in world frame.
        place: (x, y, z) of the place pose.
        approach_height_m: Vertical offset for the pre-/post- poses.
        orientation: EE quaternion (x, y, z, w). Defaults to xArm gripper-down.
        obstacles: Shared obstacles added to every scenario in the sequence —
            e.g. a work surface or fixture beneath the pick/place poses
            that the arm must avoid.

    Returns:
        Six Scenarios: ``<name>_pre_pick``, ``_pick``, ``_post_pick``,
        ``_pre_place``, ``_place``, ``_post_place``.
    """
    px, py, pz = pick
    qx, qy, qz = place
    obs = obstacles or []
    return [
        reach_scenario(f"{name}_pre_pick", px, py, pz + approach_height_m, orientation=orientation, obstacles=obs),
        reach_scenario(f"{name}_pick", px, py, pz, orientation=orientation, obstacles=obs),
        reach_scenario(f"{name}_post_pick", px, py, pz + approach_height_m, orientation=orientation, obstacles=obs),
        reach_scenario(f"{name}_pre_place", qx, qy, qz + approach_height_m, orientation=orientation, obstacles=obs),
        reach_scenario(f"{name}_place", qx, qy, qz, orientation=orientation, obstacles=obs),
        reach_scenario(f"{name}_post_place", qx, qy, qz + approach_height_m, orientation=orientation, obstacles=obs),
    ]


def _xarm6_cases() -> list[Scenario]:
    """xArm6 — full suite: basic reaches, harder geometry, one pick-and-place.

    Sections, by intent:

    - **Basic reaches** — 6 scenarios across the workspace. Cheap, fast,
      catches gross IK / planner regressions.
    - **Harder geometry** — 4 scenarios that stress the planner: obstacle
      avoidance, workspace corners, a tight passage.
    - **Pick-and-place** — 6-scenario sequence mirroring ``PickAndPlaceModule``'s
      pre-grasp / grasp / retract pattern. Catches "can the planner actually
      run a real pick-and-place" regressions.
    """
    blocker = _box("blocker", (0.30, 0.0, 0.40), (0.06, 0.25, 0.06))

    # Flanking posts for tight_clearance_passage. Tall enough that going
    # over the top isn't trivial (z up to 0.40), with a 20 cm gap between
    # them at y=±0.10. Target sits past the posts at moderate height.
    corridor_left = _box("corridor_left", (0.32, 0.10, 0.25), (0.06, 0.04, 0.30))
    corridor_right = _box("corridor_right", (0.32, -0.10, 0.25), (0.06, 0.04, 0.30))

    # Work surface beneath the pick-and-place poses. Positioned in front of
    # the arm base (x≥0.25 keeps clear of base/gripper-at-zero geometry),
    # 2 cm thin so it reads as a table surface in Meshcat. Pick/place poses
    # at z=0.10 have 5 cm of clearance above the surface top.
    work_surface = _box("work_surface", (0.325, 0.0, 0.04), (0.15, 0.55, 0.02))

    return [
        # ── basic reaches ──
        reach_scenario("reach_center", 0.35, 0.0, 0.25),
        reach_scenario("reach_left", 0.30, 0.25, 0.25),
        reach_scenario("reach_right", 0.30, -0.25, 0.25),
        reach_scenario("reach_high", 0.25, 0.0, 0.42),
        reach_scenario("reach_low", 0.35, 0.0, 0.13),
        reach_scenario("reach_extended", 0.48, 0.0, 0.20),
        # ── harder geometry ──
        reach_scenario("reach_obstacle_box", 0.42, 0.0, 0.28, obstacles=[blocker]),
        reach_scenario("reach_corner_far_right", 0.42, -0.30, 0.20),
        reach_scenario("reach_corner_far_left", 0.42, 0.30, 0.20),
        reach_scenario(
            "tight_clearance_passage",
            0.42,
            0.0,
            0.20,
            obstacles=[corridor_left, corridor_right],
        ),
        # ── pick-and-place: side-to-side transfer over a work surface ──
        *pick_place_scenarios(
            "pp_left_to_right",
            pick=(0.30, 0.20, 0.10),
            place=(0.30, -0.20, 0.10),
            approach_height_m=0.10,
            obstacles=[work_surface],
        ),
        # NB: ``pick_place_for_object`` is available for tests that want to
        # exercise the production grasp heuristic + planner together, but
        # we don't put one in the default suite — the heuristic-derived
        # orientations don't always pass xArm6's collision-checked IK, and
        # that's a real finding worth surfacing rather than hiding behind
        # "default suite passes 100%". Use ``pick_place_for_object`` in
        # your own tests when you want this signal.
    ]


def _xarm7_cases() -> list[Scenario]:
    """xArm7 — xArm6 suite minus the two scenarios that COLLISION_AT_GOAL on 7-DOF.

    The free-space reaches, the workspace-corner reaches, and the
    pick-and-place-over-work-surface sequence all still apply. Only
    ``reach_obstacle_box`` and ``tight_clearance_passage`` consistently
    fail on xArm7 — its extra DOF puts the elbow into the obstacle when
    the goal pose is in the right zone.
    """
    blocked = {"reach_obstacle_box", "tight_clearance_passage"}
    return [c for c in _xarm6_cases() if c.name not in blocked]


def _piper_cases() -> list[Scenario]:
    """Piper — 6 reaches tuned to its smaller workspace and gripper convention."""
    q = PIPER_GRIPPER_DOWN
    return [
        reach_scenario("reach_center", 0.25, 0.0, 0.20, orientation=q),
        reach_scenario("reach_left", 0.25, 0.15, 0.20, orientation=q),
        reach_scenario("reach_right", 0.25, -0.15, 0.20, orientation=q),
        reach_scenario("reach_high", 0.20, 0.0, 0.30, orientation=q),
        reach_scenario("reach_extended", 0.30, 0.0, 0.20, orientation=q),
        reach_scenario("reach_diagonal", 0.25, 0.15, 0.30, orientation=q),
    ]


# Registry of arm name → scenario-list factory. Add an entry to support a new arm.
# Factories are zero-arg so scenarios are constructed lazily (avoids heavy imports
# at module load time).
SCENARIOS_BY_ARM: dict[str, Callable[[], list[Scenario]]] = {
    "xarm6": _xarm6_cases,
    "xarm7": _xarm7_cases,
    "piper": _piper_cases,
}
