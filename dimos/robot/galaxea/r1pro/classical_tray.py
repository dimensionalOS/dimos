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

"""Two-handed tray handling for the classical R1Pro demos."""

from __future__ import annotations

from collections.abc import Callable
from itertools import groupby, pairwise
import time
from typing import TYPE_CHECKING, Any, Protocol

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.galaxea.r1pro.classical_motion import CLASSICAL_MOTION_SPEED_SCALE
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.tray_task import tray_state

if TYPE_CHECKING:
    from dimos.robot.galaxea.r1pro.home_spec import HomeControlSpec
    from dimos.robot.galaxea.r1pro.object_primitive_state import PrimitiveSceneState
    from dimos.robot.galaxea.r1pro.placement_regions import PlacementRegion

# Tray origin relative to the docked base, as built on the worktable.
TRAY_DOCK_OFFSET = np.array([0.34, -0.04])
# Rim plus handles, so a footprint stays clear of neighbouring props.
TRAY_HALF_FOOTPRINT = np.array([0.16, 0.235])
TRAY_TASK = "joint_trajectory"
GRASP_PHASES = frozenset({"lift_tray", "extend_over_table", "lower_onto_table"})


class TraySimSpec(Protocol):
    def primitive_state(self) -> dict[str, Any]: ...
    def tray_state(self) -> dict[str, Any]: ...


def tray_report(scene: PrimitiveSceneState, regions: dict[str, PlacementRegion]) -> dict[str, Any]:
    """Measure tray contacts and cargo; name the platform supporting it."""
    rows = scene.inventory()
    cargo = tuple(row["object"] for row in rows if row["inside"])
    state = tray_state(scene.model, scene.data, cargo_bodies=cargo)
    supports = set(state["support_geoms"])
    state["cargo"] = [f"object_{i + 1}" for i, row in enumerate(rows) if row["inside"]]
    state["station"] = next(
        (name for name, region in regions.items() if supports & set(region.support_geoms)),
        None,
    )
    state["held"] = bool(state["bimanual_grasp"] and not supports)
    return state


def tray_footprint(
    scene: PrimitiveSceneState, region: PlacementRegion, yaw: float
) -> NDArray[np.float64]:
    """Nearest clear tray footprint along the platform's near edge, centre first."""
    geom = scene.model.geom(region.support_geoms[0])
    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    if int(geom.type[0]) == mujoco.mjtGeom.mjGEOM_BOX:
        centre = np.asarray(scene.data.geom_xpos[geom.id], dtype=float)
        top = float(centre[2] + geom.size[2])
        half = np.abs(rotation.T @ np.asarray(geom.size[:2], dtype=float))
    else:
        centre = np.asarray(region.center, dtype=float)
        top = float(region.center[2])
        half = np.abs(rotation.T @ np.asarray(region.half_size, dtype=float))
    along = -(half[0] - TRAY_HALF_FOOTPRINT[0] - 0.03)
    lateral_limit = half[1] - TRAY_HALF_FOOTPRINT[1] - 0.02
    if along > 0 or lateral_limit < 0:
        raise RuntimeError(f"{region.name} is too small for the tray")
    blockers = []
    for i, row in enumerate(scene.inventory()):
        position = np.asarray(row["position"], dtype=float)
        if row["inside"] or row["held_by"] is not None or abs(position[2] - top) > 0.35:
            continue
        radius = float(scene.layout.objects[i].radius)
        blockers.append((rotation.T @ (position[:2] - centre[:2]), np.array([radius, radius])))
    # Fixtures resting on the platform, such as a laptop, are not layout props.
    model = scene.model
    skip = {model.body("task_bin").id, int(geom.bodyid[0])}
    skip.update(model.body(obj.name).id for obj in scene.layout.objects)
    root = model.body("base_link").id
    for body in range(model.nbody):
        if body == root or int(model.body_rootid[body]) == root:
            skip.add(body)
    for gid in range(model.ngeom):
        body = int(model.geom_bodyid[gid])
        if body in skip or not (model.geom_contype[gid] or model.geom_conaffinity[gid]):
            continue
        frame = np.asarray(scene.data.geom_xmat[gid], dtype=float).reshape(3, 3)
        aabb = np.asarray(model.geom_aabb[gid], dtype=float)
        box_centre = np.asarray(scene.data.geom_xpos[gid], dtype=float) + frame @ aabb[:3]
        box_half = np.abs(frame) @ aabb[3:]
        bottom = box_centre[2] - box_half[2]
        if not top - 0.05 <= bottom < top + 0.4:
            continue
        local = rotation.T @ (box_centre[:2] - centre[:2])
        local_half = np.abs(rotation.T @ box_half[:2])
        if np.all(np.abs(local) < half + local_half):
            blockers.append((local, local_half))
    for lateral in sorted(np.arange(-lateral_limit, lateral_limit + 1e-9, 0.02), key=abs):
        local = np.array([along, lateral])
        if all(
            np.any(np.abs(local - offset) > TRAY_HALF_FOOTPRINT + extent + 0.02)
            for offset, extent in blockers
        ):
            return np.array([*(centre[:2] + rotation @ local), top + 0.001])
    raise RuntimeError(f"No clear tray footprint on {region.name}")


def run_tray_motion(
    control: HomeControlSpec,
    sim: TraySimSpec,
    waypoints: list[dict[str, Any]],
    report: dict[str, Any],
    pause: Callable[[float], None],
    phase: Callable[[str], None],
    *,
    allowed_support: set[str],
    destination_support: set[str] | None = None,
) -> dict[str, Any]:
    """Execute checked tray waypoints phase by phase, verifying contacts throughout."""
    joints = list(R1PRO_PICK_PLACE_JOINTS)
    tray = sim.tray_state()
    cargo = set(tray["cargo"])
    for name, group in groupby(waypoints, key=lambda p: str(p["phase"])):
        if name == "release_tray" and not (destination_support or set()) & set(
            tray["support_geoms"]
        ):
            raise RuntimeError(
                "Tray is not supported by the destination; keeping both hands closed"
            )
        phase(name)
        commands = sim.primitive_state()["joint_commands"]
        points = [
            TrajectoryPoint(
                positions=[commands[n] for n in joints],
                velocities=[0.0] * len(joints),
                time_from_start=0.0,
            )
        ]
        duration = 0.0
        for waypoint in group:
            target = list(waypoint["positions"])
            displacement = np.abs(np.asarray(target) - points[-1].positions)
            transit = max(
                waypoint["seconds"] / CLASSICAL_MOTION_SPEED_SCALE,
                float(np.max(displacement[:18])) / 2.0,
                float(np.max(displacement[18:])) / 0.25,
            )
            for hold in (0.0, 1.0):
                # These commands contain no base joints. Only transit is
                # accelerated; every contact/settling hold keeps its duration.
                duration += transit if hold == 0.0 else hold
                points.append(
                    TrajectoryPoint(
                        positions=target,
                        velocities=[0.0] * len(joints),
                        time_from_start=duration,
                    )
                )
        tray = _execute(
            control,
            sim,
            joints,
            points,
            name,
            report,
            pause,
            allowed_support=allowed_support,
            grasp=name in GRASP_PHASES,
            cargo=cargo,
        )
        if name in ("grasp_handles", "lift_tray") and not tray["bimanual_grasp"]:
            raise RuntimeError("Both hands must contact the tray handles")
        if name == "lift_tray" and tray["support_geoms"]:
            raise RuntimeError("Tray still contacts its support after lifting")
    return tray


def _execute(
    control: HomeControlSpec,
    sim: TraySimSpec,
    joints: list[str],
    points: list[TrajectoryPoint],
    name: str,
    report: dict[str, Any],
    pause: Callable[[float], None],
    *,
    allowed_support: set[str],
    grasp: bool,
    cargo: set[str],
) -> dict[str, Any]:
    pause(0)
    initial = sim.primitive_state()
    if initial["error"]:
        raise RuntimeError(initial["error"])
    accepted = control.execute_trajectory(
        JointTrajectory(joint_names=joints, points=points), TRAY_TASK
    )
    if accepted.status is not TrajectoryExecutionStatus.ACCEPTED:
        raise RuntimeError(f"{name} trajectory rejected: {accepted}")
    report["motion_started"] = True
    stage: dict[str, Any] = dict(phase=name, samples=0)
    report.setdefault("tray_stages", []).append(stage)
    start = float(initial["sim_time"])
    stamp = start
    wall_start = fresh = progress = time.monotonic()
    previous = np.array(
        [initial["joint_positions"][joint] for joint in joints]
        + [initial["joint_commands"][joint] for joint in joints],
        dtype=float,
    )
    previous_speed = (
        max(abs(value) for value in initial.get("joint_velocities", {}).values())
        if initial.get("joint_velocities")
        else 0.0
    )
    progress_threshold = np.array(([1e-4] * 18 + [1e-5] * 2) * 2)
    duration = points[-1].time_from_start
    stage["command_duration_s"] = duration
    quiet_since: float | None = None
    try:
        while True:
            state = sim.primitive_state()
            if state["error"]:
                raise RuntimeError(state["error"])
            now = time.monotonic()
            sim_time = float(state["sim_time"])
            elapsed = sim_time - start
            commanded_elapsed = now - wall_start
            if sim_time < stamp:
                raise RuntimeError("Simulation clock moved backwards during tray execution")
            if sim_time > stamp:
                fresh, stamp = now, sim_time
            if now - fresh > 10.0:
                raise RuntimeError("Simulation stopped updating during tray execution")
            if elapsed > duration + 25.0:
                raise RuntimeError(f"{name} failed to reach and settle at its target")
            measured = np.array(
                [state["joint_positions"][joint] for joint in joints]
                + [state["joint_commands"][joint] for joint in joints],
                dtype=float,
            )
            holding = any(
                first.time_from_start <= commanded_elapsed < last.time_from_start
                and first.positions == last.positions
                for first, last in pairwise(points)
            )
            speed = (
                max(abs(value) for value in state.get("joint_velocities", {}).values())
                if state.get("joint_velocities")
                else 0.0
            )
            if (
                holding
                or np.any(np.abs(measured - previous) > progress_threshold)
                or speed < previous_speed - 1e-4
            ):
                progress, previous = now, measured
                previous_speed = speed
            elif now - progress > 30.0:
                raise RuntimeError("Tray trajectory stopped making measured or commanded progress")
            tray = sim.tray_state()
            stage["samples"] += 1
            if tray["tilt_radians"] > 0.25:
                raise RuntimeError("Tray tilted beyond the carrying limit")
            if grasp and not tray["bimanual_grasp"]:
                raise RuntimeError("Lost two-handed tray contact")
            if cargo - set(tray["cargo"]):
                raise RuntimeError("Cargo left the tray")
            unexpected = set(tray["support_geoms"]) - allowed_support
            if unexpected:
                raise RuntimeError(f"Tray contacted an unexpected surface: {sorted(unexpected)}")
            if commanded_elapsed >= duration:
                error = max(
                    abs(state["joint_positions"][n] - target)
                    for n, target in zip(joints[:18], points[-1].positions[:18], strict=True)
                )
                if error < 0.05 and tray["velocity_norm"] < 0.03:
                    quiet_since = sim_time if quiet_since is None else quiet_since
                    if sim_time - quiet_since > 0.5:
                        stage["final"] = tray
                        return tray
                else:
                    quiet_since = None
            pause(0.05)
    finally:
        control.cancel_trajectory(TRAY_TASK)
