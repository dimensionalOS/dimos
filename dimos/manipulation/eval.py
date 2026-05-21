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

"""Planner evaluation — score motion-planning behaviour on a fixed list of scenarios.

Runs the manipulation planning stack (IK + RRT) against a list of target
poses for one arm and reports per-scenario results: did it plan, how long it
took, how close the planner got, and several path-quality and feasibility
metrics.

The eval is built around three pieces:

- ``Scenario``: what the robot should do (pure data)
- ``evaluate``: how to run a Scenario against an arm (function)
- ``Score``: what came back, with a ``metrics: dict[str, float]`` for
  named measurements. Adding a new metric is one dict-key change.

Usage::

    from dimos.manipulation.eval import evaluate, default_scenarios
    from dimos.robot.catalog.ufactory import xarm6

    scores = evaluate(xarm6(adapter_type="mock"), default_scenarios())
    for s in scores:
        print(s.name, s.passed, s.metrics)

CLI::

    python -m dimos.manipulation.eval --arm xarm6
    python -m dimos.manipulation.eval --arm xarm6 --json out.json
    python -m dimos.manipulation.eval --arm xarm6 --viz
"""

from __future__ import annotations

import argparse
import importlib
import json
import pathlib
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec.models import Obstacle


logger = setup_logger()


# ── data types ────────────────────────────────────────────────────────────────


@dataclass
class Scenario:
    """One planning scenario.

    Attributes:
        name: Short identifier shown in reports.
        target: Goal pose for the end-effector in world frame.
        obstacles: Collision objects added before planning, removed after.
        start_joints: Starting joint angles. ``None`` uses zero configuration.
        position_tolerance_m: Max IK position error to count as passed.
        orientation_tolerance_rad: Max IK orientation error to count as passed.
    """

    name: str
    target: PoseStamped
    obstacles: list[Obstacle] = field(default_factory=list)
    start_joints: list[float] | None = None
    position_tolerance_m: float = 0.01
    orientation_tolerance_rad: float = 0.1


@dataclass
class Score:
    """Result of running one Scenario.

    ``metrics`` is a free-form dict so new measurements can be added without
    changing the type. CI and dashboards read by key.

    Standard keys (all set when a plan was found; subset on failure):

    - ``planning_time_s``                 wall-clock IK + planner time
    - ``position_error_m``                IK Cartesian error at the goal
    - ``orientation_error_rad``           IK orientation error at the goal
    - ``path_length_rad``                 joint-space path length
    - ``n_waypoints``                     waypoints in the planned path
    - ``path_length_cartesian_m``         3D EE travel distance
    - ``joint_limit_margin_rad``          min joint-limit clearance along path
    - ``max_joint_velocity_ratio``        ≤ 1 is feasible (after time-param)
    - ``max_joint_acceleration_ratio``    same for acceleration
    """

    name: str
    passed: bool
    reason: str = ""
    metrics: dict[str, float] = field(default_factory=dict)


# ── scenario helpers ──────────────────────────────────────────────────────────────


def reach_scenario(
    name: str,
    x: float,
    y: float,
    z: float,
    orientation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    obstacles: list[Obstacle] | None = None,
    position_tolerance_m: float = 0.01,
    orientation_tolerance_rad: float = 0.1,
) -> Scenario:
    """Convenience constructor for a single-reach Scenario.

    Saves the ``PoseStamped(Vector3(...), Quaternion(...))`` boilerplate.

    Default ``orientation`` is xArm's "gripper-down" (roll=180°). For arms
    with a different EE link convention (e.g. Piper points its gripper
    forward at zero config), pass an explicit quaternion tuple.

    Common quaternions you may want to pass:

    - ``(1.0, 0.0, 0.0, 0.0)``         xArm gripper-down (default)
    - ``(0.0, 0.7071, 0.0, 0.7071)``   Piper gripper-down
    - ``(0.0, 0.0, 0.0, 1.0)``         identity (no rotation from world)
    """
    qx, qy, qz, qw = orientation
    return Scenario(
        name=name,
        target=PoseStamped(
            frame_id="world",
            position=Vector3(x, y, z),
            orientation=Quaternion(qx, qy, qz, qw),
        ),
        obstacles=obstacles or [],
        position_tolerance_m=position_tolerance_m,
        orientation_tolerance_rad=orientation_tolerance_rad,
    )


def grasp_scenario(
    name: str,
    object_at: tuple[float, float, float],
    object_size: tuple[float, float, float],
    obstacles: list[Obstacle] | None = None,
) -> Scenario:
    """Build a Scenario to grasp an object at the given position and size.

    Uses ``PickAndPlaceModule``'s production grasp heuristics
    (``_occlusion_offset`` and ``_grasp_orientation``) to derive the pick
    pose — so when the heuristics change, the eval automatically reflects
    the new behavior. Catches the failure mode "heuristic produces a pose
    the planner can't reach."

    The grasp z is the top of the object (``center.z + size.z / 2``).
    """
    # Lazy import: pulling in the full pick-and-place module is expensive,
    # and most evals don't need it.
    from dimos.manipulation.pick_and_place_module import PickAndPlaceModule

    cx, cy, cz = object_at
    sx, sy, sz = object_size
    center = Vector3(x=cx, y=cy, z=cz)
    size = Vector3(x=sx, y=sy, z=sz)

    gx, gy = PickAndPlaceModule._occlusion_offset(center, size)
    gz = cz + sz / 2
    xy_dist = (gx * gx + gy * gy) ** 0.5
    q = PickAndPlaceModule._grasp_orientation(gx, gy, xy_dist)

    return reach_scenario(
        name,
        gx,
        gy,
        gz,
        orientation=(q.x, q.y, q.z, q.w),
        obstacles=obstacles,
    )


# ── runner ────────────────────────────────────────────────────────────────────


def evaluate(
    arm: Any,
    scenarios: list[Scenario],
    planning_timeout_s: float = 10.0,
    kinematics_name: str = "jacobian",
    planner_name: str = "rrt_connect",
    viz: bool = False,
) -> list[Score]:
    """Run a list of planning scenarios against an arm. One Score per Scenario.

    Args:
        arm: A ``RobotConfig`` from the dimOS catalog (e.g. ``xarm6()``).
        scenarios: Scenarios to run, in order.
        planning_timeout_s: Per-scenario RRT timeout.
        kinematics_name: IK solver name (see ``create_kinematics``).
        planner_name: Motion planner name (see ``create_planner``).
        viz: When ``True``, opens a Meshcat viewer and animates each
            planned path. The Meshcat URL is printed at startup.

    Returns:
        One Score per Scenario, in the same order. Failures are returned as
        Scores with ``passed=False`` and a human-readable ``reason``;
        exceptions never propagate out.
    """
    from dimos.manipulation.planning.factory import create_kinematics, create_planner
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor

    config = arm.to_robot_model_config()
    monitor = WorldMonitor(enable_viz=viz)
    robot_id = monitor.add_robot(config)
    monitor.finalize()
    kinematics = create_kinematics(name=kinematics_name)
    planner = create_planner(name=planner_name)
    dof = len(config.joint_names)

    if viz:
        url = monitor.world.get_visualization_url()
        if url:
            print(f"\n  Meshcat: {url}\n", flush=True)

    scores: list[Score] = []
    for scenario in scenarios:
        scores.append(
            _run_one(
                scenario, monitor, kinematics, planner, robot_id, config, dof, planning_timeout_s, viz
            )
        )
    return scores


def _run_one(
    scenario: Scenario,
    monitor: Any,
    kinematics: Any,
    planner: Any,
    robot_id: str,
    config: Any,
    dof: int,
    timeout_s: float,
    viz: bool = False,
) -> Score:
    """Run a single scenario end-to-end. Catches exceptions to keep the suite running."""
    start_joints = scenario.start_joints or [0.0] * dof
    initial_js = JointState(name=config.joint_names, position=start_joints)
    monitor.world.sync_from_joint_state(robot_id, initial_js)

    added_ids: list[str] = []
    try:
        for obs in scenario.obstacles:
            added_ids.append(monitor.add_obstacle(obs))
        if viz:
            print(f"  ▶ {scenario.name}", flush=True)
            time.sleep(0.5)
        return _plan_and_score(
            scenario, monitor, kinematics, planner, robot_id, initial_js, timeout_s, config, viz
        )
    except Exception as exc:
        logger.warning(f"{scenario.name}: eval raised exception: {exc}")
        return Score(name=scenario.name, passed=False, reason=f"exception: {exc}")
    finally:
        if viz:
            time.sleep(0.5)
        for obs_id in added_ids:
            monitor.remove_obstacle(obs_id)


def _plan_and_score(
    scenario: Scenario,
    monitor: Any,
    kinematics: Any,
    planner: Any,
    robot_id: str,
    initial_js: JointState,
    timeout_s: float,
    config: Any,
    viz: bool = False,
) -> Score:
    """Solve IK, plan, compute metrics, decide pass/fail."""
    t_start = time.perf_counter()

    # check_collision=True: IK must return a goal joint config that is
    # collision-free. Otherwise RRT can't plan a path TO it and we get
    # spurious COLLISION_AT_GOAL failures whenever obstacles exist.
    ik = kinematics.solve(
        world=monitor.world,
        robot_id=robot_id,
        target_pose=scenario.target,
        seed=initial_js,
        check_collision=True,
    )

    if not ik.is_success() or ik.joint_state is None:
        return Score(
            name=scenario.name,
            passed=False,
            reason=f"ik failed: {ik.status.name}",
            metrics={"planning_time_s": time.perf_counter() - t_start},
        )

    plan = planner.plan_joint_path(
        world=monitor.world,
        robot_id=robot_id,
        start=initial_js,
        goal=ik.joint_state,
        timeout=timeout_s,
    )
    elapsed = time.perf_counter() - t_start

    metrics: dict[str, float] = {
        "planning_time_s": elapsed,
        "position_error_m": ik.position_error,
        "orientation_error_rad": ik.orientation_error,
    }

    if not plan.is_success():
        return Score(
            name=scenario.name,
            passed=False,
            reason=f"plan failed: {plan.status.name}",
            metrics=metrics,
        )

    metrics["path_length_rad"] = plan.path_length
    metrics["n_waypoints"] = float(len(plan.path))

    # Path-quality + feasibility metrics. Non-fatal: scenario can still pass
    # on IK accuracy alone if these computations raise.
    try:
        metrics.update(_path_metrics(plan.path, monitor, robot_id, config))
    except Exception as exc:
        logger.warning(f"{scenario.name}: extra metrics failed: {exc}")

    if viz:
        try:
            monitor.world.animate_path(robot_id, plan.path, duration=2.0)
        except Exception as exc:
            logger.warning(f"animate_path failed: {exc}")

    if ik.position_error > scenario.position_tolerance_m:
        return Score(
            name=scenario.name,
            passed=False,
            reason=(
                f"position error {ik.position_error * 1000:.1f}mm "
                f"> {scenario.position_tolerance_m * 1000:.1f}mm tolerance"
            ),
            metrics=metrics,
        )
    if ik.orientation_error > scenario.orientation_tolerance_rad:
        return Score(
            name=scenario.name,
            passed=False,
            reason=(
                f"orientation error {ik.orientation_error:.3f}rad "
                f"> {scenario.orientation_tolerance_rad:.3f}rad tolerance"
            ),
            metrics=metrics,
        )

    return Score(name=scenario.name, passed=True, metrics=metrics)


def _path_metrics(
    path: list[JointState], monitor: Any, robot_id: str, config: Any
) -> dict[str, float]:
    """Compute path-quality and feasibility metrics for a planned path.

    Any metric whose computation raises is silently skipped (a warning is
    logged) — the rest still get returned.

    Returns keys:

    - ``joint_limit_margin_rad``: minimum distance from any joint to its
      hard limit along the path. Negative → joint limit violated.
    - ``path_length_cartesian_m``: total 3D distance the EE travels.
    - ``max_joint_velocity_ratio``: max ``|v_i|/v_limit_i`` after
      time-parameterization with the arm's configured limits. > 1 means
      infeasible.
    - ``max_joint_acceleration_ratio``: same for acceleration (discrete
      derivative of velocity, sensitive to time-step granularity).

    Follows MoveIt's ``moveit_ros_benchmarks`` and OMPL's ``PathGeometric``
    conventions (path length, clearance) plus TOPP-RA-style feasibility
    ratios (velocity/acceleration after time parameterization).
    """
    import numpy as np

    out: dict[str, float] = {}
    dof = len(config.joint_names)

    # 1. joint limit margin — pure path inspection
    try:
        lower, upper = monitor.world.get_joint_limits(robot_id)
        lower = np.asarray(lower)
        upper = np.asarray(upper)
        min_margin = float("inf")
        for js in path:
            q = np.asarray(js.position[: len(lower)])
            margin = float(np.min(np.minimum(q - lower, upper - q)))
            if margin < min_margin:
                min_margin = margin
        if min_margin != float("inf"):
            out["joint_limit_margin_rad"] = min_margin
    except Exception as exc:
        logger.warning(f"joint_limit_margin failed: {exc}")

    # 2. cartesian path length — FK at each waypoint, sum 3D deltas
    try:
        positions: list[tuple[float, float, float]] = []
        with monitor.world.scratch_context() as ctx:
            for js in path:
                monitor.world.set_joint_state(ctx, robot_id, js)
                pose = monitor.world.get_ee_pose(ctx, robot_id)
                positions.append((pose.position.x, pose.position.y, pose.position.z))
        total = 0.0
        for a, b in zip(positions, positions[1:]):
            total += ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5
        out["path_length_cartesian_m"] = total
    except Exception as exc:
        logger.warning(f"path_length_cartesian failed: {exc}")

    # 3 + 4. trajectory feasibility — time-parameterize via JointTrajectoryGenerator,
    # then compare per-joint velocities (and discrete-differentiated accelerations)
    # against the arm's configured limits.
    try:
        from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
            JointTrajectoryGenerator,
        )

        waypoints = [list(js.position[:dof]) for js in path]
        gen = JointTrajectoryGenerator(
            num_joints=dof,
            max_velocity=config.max_velocity,
            max_acceleration=config.max_acceleration,
        )
        traj = gen.generate(waypoints)
        v_lim = np.asarray(gen.max_velocity)
        a_lim = np.asarray(gen.max_acceleration)

        max_v_ratio = 0.0
        for pt in traj.points:
            v = np.asarray(pt.velocities)
            r = float(np.max(np.abs(v) / v_lim))
            if r > max_v_ratio:
                max_v_ratio = r
        out["max_joint_velocity_ratio"] = max_v_ratio

        max_a_ratio = 0.0
        for i in range(1, len(traj.points)):
            dt = traj.points[i].time_from_start - traj.points[i - 1].time_from_start
            if dt > 1e-9:
                v0 = np.asarray(traj.points[i - 1].velocities)
                v1 = np.asarray(traj.points[i].velocities)
                a = (v1 - v0) / dt
                r = float(np.max(np.abs(a) / a_lim))
                if r > max_a_ratio:
                    max_a_ratio = r
        out["max_joint_acceleration_ratio"] = max_a_ratio
    except Exception as exc:
        logger.warning(f"trajectory feasibility failed: {exc}")

    return out


# ── scenario registry ─────────────────────────────────────────────────────────────


def scenarios_for(arm_name: str) -> list[Scenario]:
    """Tuned scenario set for the given arm.

    Looks up ``SCENARIOS_BY_ARM`` in ``eval_scenarios.py`` — that's the file to edit
    to tune existing arms or add new ones, not this one.

    Raises:
        ValueError: if no scenario set is registered for ``arm_name``.
    """
    # Local import avoids an import cycle (eval_scenarios imports Scenario from here).
    from dimos.manipulation.eval_scenarios import SCENARIOS_BY_ARM

    factory = SCENARIOS_BY_ARM.get(arm_name)
    if factory is None:
        raise ValueError(
            f"no registered scenarios for arm: {arm_name!r}. "
            f"Registered: {sorted(SCENARIOS_BY_ARM)}. "
            f"Add an entry to dimos/manipulation/eval_scenarios.py or pass a "
            f"custom list to evaluate()."
        )
    return factory()


def default_scenarios() -> list[Scenario]:
    """Canonical suite — xArm6, 7 scenarios. Used in docs and examples."""
    return scenarios_for("xarm6")


# ── reporting ─────────────────────────────────────────────────────────────────


def print_scores(scores: list[Score]) -> None:
    """Render scores as a terminal table."""
    line = "─" * 60
    print(line)
    print(f"  Planner Eval · {len(scores)} scenarios")
    print(line)
    n_passed = sum(1 for s in scores if s.passed)
    for s in scores:
        icon = "✓" if s.passed else "✗"
        parts: list[str] = []
        t = s.metrics.get("planning_time_s")
        e = s.metrics.get("position_error_m")
        v = s.metrics.get("max_joint_velocity_ratio")
        if t is not None:
            parts.append(f"{t * 1000:.0f}ms")
        if e is not None:
            parts.append(f"err={e * 1000:.1f}mm")
        if v is not None:
            parts.append(f"v={v * 100:.0f}%")
        if not s.passed and s.reason:
            parts.append(s.reason)
        detail = "  " + "  ".join(parts) if parts else ""
        print(f"  {icon}  {s.name:<24}{detail}")
    print(line)
    print(f"  {n_passed}/{len(scores)} passed")
    print(line)


def to_json(scores: list[Score], path: str) -> None:
    """Dump scores to JSON. Suitable as a CI artifact."""
    payload = [
        {"name": s.name, "passed": s.passed, "reason": s.reason, "metrics": s.metrics}
        for s in scores
    ]
    pathlib.Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)


# ── CLI ───────────────────────────────────────────────────────────────────────


# Arm name → "<module>:<factory>". Add an entry to expose a new arm to the CLI.
# Panda intentionally absent: its URDF tarball is not in default LFS pulls.
# Construct programmatically if you have it available locally.
_ARM_FACTORIES: dict[str, str] = {
    "xarm6": "dimos.robot.catalog.ufactory:xarm6",
    "xarm7": "dimos.robot.catalog.ufactory:xarm7",
    "piper": "dimos.robot.catalog.piper:piper",
}


def _load_arm(name: str) -> Any:
    """Import and call the catalog factory for ``name`` with ``adapter_type='mock'``."""
    if name not in _ARM_FACTORIES:
        raise ValueError(f"unknown arm: {name!r}. Available: {sorted(_ARM_FACTORIES)}")
    module_path, fn_name = _ARM_FACTORIES[name].split(":")
    factory = getattr(importlib.import_module(module_path), fn_name)
    return factory(adapter_type="mock")


def main() -> int:
    """CLI entry point. Returns shell exit code (0 = all passed, 1 = any failed)."""
    parser = argparse.ArgumentParser(description="Planner evaluation for dimOS arms.")
    parser.add_argument(
        "--arm",
        default="xarm6",
        choices=sorted(_ARM_FACTORIES.keys()),
        help="Which arm to evaluate.",
    )
    parser.add_argument("--json", default=None, help="Write results to this JSON file.")
    parser.add_argument(
        "--viz",
        action="store_true",
        help="Open a Meshcat viewer and animate each planned path.",
    )
    args = parser.parse_args()

    arm = _load_arm(args.arm)
    scores = evaluate(arm, scenarios_for(args.arm), viz=args.viz)
    print_scores(scores)
    if args.json:
        to_json(scores, args.json)
    return 0 if all(s.passed for s in scores) else 1


if __name__ == "__main__":
    # Drake's Meshcat destructor runs from one of its background threads during
    # process teardown and asserts main-thread, printing a noisy abort trace
    # despite a clean eval. By the time main() returns all output has flushed,
    # so redirect stderr to /dev/null and hard-exit before that fires.
    import os as _os
    import sys as _sys

    _rc = main()
    _sys.stdout.flush()
    try:
        _os.dup2(_os.open(_os.devnull, _os.O_WRONLY), 2)
    except Exception:
        pass
    _os._exit(_rc)
