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

"""Execute and verify physical tray delivery through coordinator trajectories."""

from __future__ import annotations

from itertools import groupby, pairwise
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.galaxea.r1pro.grasping_sim import VIRTUAL_BASE_JOINTS
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS

if TYPE_CHECKING:
    from dimos.core.rpc_client import ModuleProxy


def _check_cargo(state: dict[str, Any], *, grasp: bool) -> None:
    if state.get("robot_obstacles"):
        raise RuntimeError(f"Robot contacted an obstacle: {state['robot_obstacles']}")
    if not state["inside_bin"]:
        raise RuntimeError("Cargo left the tray during delivery")
    if not state.get("upright", True):
        raise RuntimeError("A bottle tipped over during delivery")
    if state["tray"]["tilt_radians"] > 0.25:
        raise RuntimeError("Tray tilted beyond the carrying limit")
    if grasp and not state["tray"]["bimanual_grasp"]:
        raise RuntimeError("Lost two-handed tray contact; stopping the delivery")


def _execute(
    control: ModuleProxy,
    sim: ModuleProxy,
    joints: list[str],
    points: list[TrajectoryPoint],
    task: str,
    phase: str,
    report: dict[str, Any],
    *,
    grasp: bool,
    check_obstacles: bool = False,
) -> dict[str, Any]:
    accepted = control.execute_trajectory(JointTrajectory(joint_names=joints, points=points), task)
    if accepted.status is not TrajectoryExecutionStatus.ACCEPTED:
        raise RuntimeError(f"{phase} trajectory rejected: {accepted}")
    print(f"Tray delivery: {phase}", flush=True)
    stage: dict[str, Any] = {"phase": phase, "history": []}
    report["stages"].append(stage)
    source_phases = {
        "raise_hands",
        "approach_tray",
        "lower_to_handles",
        "grasp_handles",
        "lift_tray",
    }
    destination_phases = {
        "extend_over_table",
        "lower_onto_table",
        "release_tray",
        "retreat_from_tray",
    }
    allowed_support = (
        set(report["initial"]["tray"]["support_geoms"])
        if phase in source_phases
        else ({report["destination"]["support_geom"]} if phase in destination_phases else set())
    )
    start = time.monotonic()
    duration = points[-1].time_from_start
    quiet_since: float | None = None
    try:
        while time.monotonic() - start < 2 * duration + 10:
            state: dict[str, Any] = sim.task_state()
            stage["history"].append(state)
            _check_cargo(state, grasp=grasp)
            unexpected = set(state["tray"]["support_geoms"]) - allowed_support
            if unexpected:
                raise RuntimeError(f"Tray contacted an unexpected object: {sorted(unexpected)}")
            if check_obstacles and state["obstacles"]:
                raise RuntimeError(
                    f"Carried tray or robot contacted an obstacle: {state['obstacles']}"
                )
            if time.monotonic() - start >= duration:
                measured = control.get_joint_positions()
                error = max(
                    abs(measured[name] - target)
                    for name, target in zip(joints, points[-1].positions, strict=True)
                )
                quiet = state["tray"]["velocity_norm"] < 0.03
                tolerance = 0.015 if task == "base_transport" else 0.05
                if task == "base_transport":
                    quiet = quiet and np.linalg.norm(state["base_velocity"]) < 0.02
                if error < tolerance and quiet:
                    quiet_since = time.monotonic() if quiet_since is None else quiet_since
                    if time.monotonic() - quiet_since > 0.5:
                        stage["final"] = state
                        return state
                else:
                    quiet_since = None
            time.sleep(0.05)
        raise RuntimeError(f"{phase} failed to reach and settle at its target")
    finally:
        control.cancel_trajectory(task)


def _arm_motion(
    control: ModuleProxy,
    sim: ModuleProxy,
    waypoints: list[dict[str, Any]],
    report: dict[str, Any],
) -> dict[str, Any]:
    state: dict[str, Any] = sim.task_state()
    for phase, group in groupby(
        waypoints, key=lambda p: "lift_tray" if p["phase"].startswith("lift_tray") else p["phase"]
    ):
        if (
            phase == "release_tray"
            and report["destination"]["support_geom"] not in state["tray"]["support_geoms"]
        ):
            raise RuntimeError(
                "Tray is not supported by the destination table; keeping both hands closed"
            )
        joints = list(R1PRO_PICK_PLACE_JOINTS)
        measured = control.get_joint_positions()
        points = [
            TrajectoryPoint(
                positions=list(report.get("last_arm_command", [measured[name] for name in joints])),
                velocities=[0.0] * len(joints),
                time_from_start=0.0,
            )
        ]
        duration = 0.0
        for waypoint in group:
            duration += waypoint["seconds"]
            points.append(
                TrajectoryPoint(
                    positions=waypoint["positions"],
                    velocities=[0.0] * len(joints),
                    time_from_start=duration,
                )
            )
            duration += 1.0
            points.append(
                TrajectoryPoint(
                    positions=waypoint["positions"],
                    velocities=[0.0] * len(joints),
                    time_from_start=duration,
                )
            )
        grasp = phase in (
            "lift_tray",
            "extend_over_table",
            "lower_onto_table",
        )
        state = _execute(
            control, sim, joints, points, "tray_manipulation", phase, report, grasp=grasp
        )
        # Preserve grip preload across tasks: measured finger positions are
        # wider than the commanded opening while squeezing a handle.
        report["last_arm_command"] = points[-1].positions
        if phase in ("grasp_handles", "lift_tray"):
            _check_cargo(state, grasp=True)
        if phase == "lift_tray" and state["tray"]["support_geoms"]:
            raise RuntimeError("Tray still contacts the worktop after lifting")
    return state


def run_tray_delivery(control: ModuleProxy, sim: ModuleProxy, report: dict[str, Any]) -> None:
    """Run after ACT has stopped; every moving joint is owned by the coordinator."""
    report.update(success=False, stages=[], destination=sim.tray_destination())
    destination = report["destination"]
    initial = sim.task_state()
    if not initial["tray"]["support_geoms"] or not initial["tray"]["released"]:
        raise RuntimeError("Expected the loaded tray to rest on the starting worktop")
    report["initial"] = initial
    sim.prepare_tray_holding()
    report["pickup"] = _arm_motion(control, sim, sim.plan_tray_motion("pickup"), report)
    report["pickup_snapshot"] = sim.simulation_snapshot()
    sim.set_tray_delivery_view()
    path = sim.plan_transport(*destination["base_position"])
    report["path"] = path
    for index, (first, second) in enumerate(pairwise(path)):
        duration = max(
            0.5,
            1.5 * float(np.linalg.norm(np.array(second[:2]) - first[:2])) / 0.08,
            1.5 * abs(second[2] - first[2]) / 0.12,
        )
        # Finish each collision-checked segment in measured state before
        # dispatching the next. A slow simulator must not cut route corners.
        points = []
        for fraction in np.linspace(0, 1, max(2, int(np.ceil(duration * 20))) + 1):
            smooth = 3 * fraction**2 - 2 * fraction**3
            target = np.array(first) + smooth * (np.array(second) - first)
            points.append(
                TrajectoryPoint(
                    positions=target.tolist(),
                    velocities=[0.0] * 3,
                    time_from_start=float(fraction * duration),
                )
            )
        report["arrival"] = _execute(
            control,
            sim,
            list(VIRTUAL_BASE_JOINTS),
            points,
            "base_transport",
            f"carry_segment_{index + 1}",
            report,
            grasp=True,
            check_obstacles=True,
        )
    report["arrival_snapshot"] = sim.simulation_snapshot()
    final = _arm_motion(
        control, sim, sim.plan_tray_motion("place", destination["tray_position"]), report
    )
    report["final"] = final
    report["final_snapshot"] = sim.simulation_snapshot()
    tray = final["tray"]
    report["success"] = bool(
        tray["released"]
        and destination["support_geom"] in tray["support_geoms"]
        and tray["velocity_norm"] < 0.03
        and tray["tilt_radians"] < 0.08
        and final["settled"]
        and np.linalg.norm(np.array(tray["position"]) - destination["tray_position"]) < 0.05
        and final["inside_bin"]
        and final["released"]
        and final.get("upright", True)
    )
    if not report["success"]:
        raise RuntimeError(
            "Tray delivery did not meet physical release/support/containment criteria"
        )
