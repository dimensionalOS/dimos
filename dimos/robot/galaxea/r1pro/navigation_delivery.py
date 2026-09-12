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

"""Execute native KronkNav routes with the coordinator's holonomic task."""

from __future__ import annotations

from collections.abc import Callable
from itertools import pairwise
from pathlib import Path
import time
from typing import Any

import numpy as np

from dimos.msgs.nav_msgs.Path import Path as NavigationPath
from dimos.robot.galaxea.r1pro.home_spec import HomeControlSpec, HomeSimSpec
from dimos.robot.galaxea.r1pro.navigation_sim import NAV_TASK, pose_message


def follow_navigation_path(
    control: HomeControlSpec,
    sim: HomeSimSpec,
    path: list[list[float]],
    phase: str,
    report: dict[str, Any],
    pause: Callable[[float], None] = time.sleep,
    speed: float | None = None,
) -> dict[str, Any]:
    """Validate a full-pose path, then monitor physical cargo while the task drives."""
    pause(0)
    sim.validate_navigation_path(path)
    pause(0)
    control.task_invoke(NAV_TASK, "reset", {})
    if speed is not None:
        control.task_invoke(NAV_TASK, "set_speed", {"speed": speed})
    state: dict[str, Any] = sim.task_state()
    message = NavigationPath(poses=[pose_message(p) for p in path], frame_id="world")
    accepted = control.task_invoke(
        NAV_TASK,
        "start_path",
        {
            "path": message,
            "current_odom": pose_message(state["base_pose"]),
        },
    )
    if not accepted:
        raise RuntimeError(f"Holonomic task rejected {phase}")
    print(f"Tray navigation: {phase} ({len(path)} poses)", flush=True)
    stage: dict[str, Any] = {"phase": phase, "path": path, "history": []}
    report["stages"].append(stage)
    poses = np.asarray(path)
    length = float(np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1).sum())
    turns = float(np.abs(np.diff(poses[:, 2])).sum())
    deadline = time.monotonic() + 60 + length / 0.02 + turns / 0.04
    last_sim_time, last_update = state["sim_time"], time.monotonic()
    progress_pose = np.asarray(state["base_pose"])
    progress_time = time.monotonic()
    try:
        while time.monotonic() < deadline:
            state = sim.task_state()
            stage["history"].append(state)
            if state["sim_time"] > last_sim_time:
                last_sim_time, last_update = state["sim_time"], time.monotonic()
            if time.monotonic() - last_update > 1.0:
                raise RuntimeError("Navigation odometry stopped updating")
            if state["obstacles"] or state["tray"]["support_geoms"]:
                raise RuntimeError(f"Loaded robot contacted an obstacle: {state['obstacles']}")
            if not (
                state["inside_bin"]
                and state.get("upright", True)
                and state["tray"]["bimanual_grasp"]
            ):
                raise RuntimeError("Navigation lost an upright bottle or a two-handed tray grasp")
            if state["tray"]["tilt_radians"] > 0.25:
                raise RuntimeError("Navigation exceeded the carrying tilt limit")
            status = control.task_invoke(NAV_TASK, "get_state", {})
            measured_pose = np.asarray(state["base_pose"])
            if np.linalg.norm(measured_pose - progress_pose) > 0.002:
                progress_pose, progress_time = measured_pose, time.monotonic()
            if status == "tracking" and time.monotonic() - progress_time > 5.0:
                raise RuntimeError(
                    f"Base made no progress for 5 seconds during {phase}; "
                    "check the base command and odometry transport connections"
                )
            if status == "aborted":
                raise RuntimeError("Holonomic task was aborted")
            if status == "arrived":
                error = np.asarray(state["base_pose"]) - poses[-1]
                error[2] = (error[2] + np.pi) % (2 * np.pi) - np.pi
                if np.linalg.norm(error[:2]) > 0.012 or abs(error[2]) > 0.012:
                    raise RuntimeError("Holonomic task reported arrival outside measured tolerance")
                if (
                    np.linalg.norm(state["base_velocity"]) < 0.01
                    and state["tray"]["velocity_norm"] < 0.03
                ):
                    stage["final"] = state
                    return state
            pause(0.05)
        raise RuntimeError(f"Navigation timed out during {phase}")
    finally:
        try:
            control.task_invoke(NAV_TASK, "cancel", {})
        finally:
            sim.stop_navigation_base()


def prepare_navigation_map(
    sim: HomeSimSpec, cloud: Path, pause: Callable[[float], None] = time.sleep
) -> None:
    """Require a native map acknowledgement before starting manipulation."""
    if sim.navigation_status()["surface_points"] > 0:
        return
    sim.publish_navigation_map(str(cloud.resolve()))
    deadline = time.monotonic() + 30
    while sim.navigation_status()["surface_points"] == 0:
        if time.monotonic() > deadline:
            raise RuntimeError("KronkNav did not acknowledge the complete environment map")
        pause(0.1)


def run_navigation_transport(
    control: HomeControlSpec,
    sim: HomeSimSpec,
    report: dict[str, Any],
    cloud: Path,
    pause: Callable[[float], None] = time.sleep,
) -> None:
    """Use a simulated whole-house lidar map; ACT is stopped throughout driving."""
    report["navigation"] = {
        "planner": "MLSPlannerNative (KronkNav)",
        "controller": "HolonomicPoseFollowerTask",
        "cloud": str(cloud.resolve()),
    }
    prepare_navigation_map(sim, cloud, pause=pause)
    destination = report["destination"]
    target = destination.get("approach_position", destination["base_position"])
    source_approach = report.get("source", {}).get("approach_position")
    if source_approach is not None:
        # Leave the furniture's overhang before asking the 2D planner to travel.
        # The whole robot/cargo sweep is checked against the actual 3D scene.
        heading = sim.task_state()["base_pose"][2]
        departure_path = sim.plan_transport(source_approach[0], source_approach[1], heading)
        report["surface_departure_path"] = departure_path
        follow_navigation_path(
            control, sim, departure_path, "surface_departure", report, pause=pause, speed=0.10
        )
    departure = sim.plan_departure(target[2])
    for index, (a, b) in enumerate(pairwise(departure)):
        follow_navigation_path(
            control, sim, [a, b], f"departure_{index + 1}", report, pause=pause, speed=0.055
        )
    sim.request_navigation_path(target)
    deadline = time.monotonic() + 30
    while True:
        status = sim.navigation_status()
        if status.get("error"):
            raise RuntimeError(status["error"])
        if status["path"] is not None:
            break
        if time.monotonic() > deadline:
            raise RuntimeError("KronkNav did not return a complete route")
        pause(0.1)
    report["navigation"].update(status)
    report["path"] = sim.refine_navigation_path(status["path"])
    report["arrival"] = follow_navigation_path(
        control, sim, report["path"], "kronknav_carry", report, pause=pause, speed=0.6
    )
    if "approach_position" in destination:
        docking = sim.plan_transport(*destination["base_position"])
        report["docking_path"] = docking
        report["arrival"] = follow_navigation_path(
            control, sim, docking, "surface_docking", report, pause=pause, speed=0.10
        )
