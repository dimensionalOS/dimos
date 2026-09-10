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

"""Run the trained R1Pro task through DimOS with the full MuJoCo display."""

from __future__ import annotations

import argparse
from itertools import pairwise
import json
from pathlib import Path
import time
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.imitation.policy.module import POLICY_ROLLOUT_INSTANCE_NAME
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.galaxea.r1pro.grasping_blueprint import R1ProGraspingSim, build_r1pro_pick_place
from dimos.robot.galaxea.r1pro.grasping_sim import (
    BOTTLE_XY,
    VIRTUAL_BASE_JOINTS,
    prepare_grasping_scene,
)
from dimos.robot.galaxea.r1pro.sim_session import DemoSessionInUseError, reserve_demo_session
from dimos.robot.galaxea.r1pro.tray_delivery import run_tray_delivery
from dimos.robot.galaxea.r1pro.tray_sim import prepare_tray_delivery_scene


def run(args: argparse.Namespace) -> dict[str, Any]:
    with reserve_demo_session(args.zenoh_scout_addr, args.output):
        return _run(args)


def _run(args: argparse.Namespace) -> dict[str, Any]:
    args.output.mkdir(parents=True, exist_ok=True)
    scene = (
        prepare_tray_delivery_scene(args.output / "scene.xml", scene_package=args.scene_package)
        if args.deliver_to_laptop
        else prepare_grasping_scene(
            args.output / "scene.xml",
            scene_package=args.scene_package,
            mobile=args.mobile,
        )
    )
    # Episode randomization belongs to scene initialization, before physics starts.
    root = ET.parse(scene)
    bottle = root.find('.//body[@name="task_bottle"]')
    assert bottle is not None
    xyz = list(map(float, bottle.attrib["pos"].split()))
    xyz[:2] = (
        np.array(BOTTLE_XY) + np.random.default_rng(args.seed).uniform(-0.012, 0.012, 2)
    ).tolist()
    bottle.set("pos", " ".join(map(str, xyz)))
    root.write(scene, encoding="unicode")
    blueprint = build_r1pro_pick_place(
        scene_path=scene, artifact=str(args.artifact), headless=args.no_viewer
    ).global_config(
        viewer="none",
        n_workers=3,
        zenoh_scout_addr=args.zenoh_scout_addr,
    )
    coordinator: ModuleCoordinator | None = None
    report: dict[str, Any] = {
        "artifact": str(args.artifact.resolve()),
        "seed": args.seed,
        "success": False,
    }
    try:
        coordinator = ModuleCoordinator.build(blueprint)
        policy = coordinator.get_instance(POLICY_ROLLOUT_INSTANCE_NAME)
        sim = coordinator.get_instance(R1ProGraspingSim)
        deadline = time.monotonic() + 45
        while True:
            status = policy.preflight_rollout()
            if status["policy_ready"] and status["observations_ready"]:
                break
            if time.monotonic() > deadline:
                raise RuntimeError(f"Preflight failed: {status}")
            time.sleep(0.1)
        report["initial"] = sim.task_state()
        status = policy.start_rollout()
        if not status["active"]:
            raise RuntimeError(f"Policy did not start: {status}")
        history = []
        stable_since: float | None = None
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            status = policy.rollout_status()
            if status["last_error"] or not status["active"]:
                raise RuntimeError(f"Rollout failed: {status}")
            state = sim.task_state()
            history.append(state)
            if state["success"]:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= 1:
                    report["success"] = True
                    break
            else:
                stable_since = None
            time.sleep(0.05)
        started = time.monotonic()
        report["stopped"] = policy.stop_rollout()
        report["stop_seconds"] = time.monotonic() - started
        if report["stopped"]["active"] or report["stopped"]["last_error"]:
            raise RuntimeError(f"Policy did not stop cleanly: {report['stopped']}")
        report["final"] = sim.task_state()
        report["history"] = history
        report["manipulation_success"] = report["success"]
        if report["success"] and args.deliver_to_laptop:
            report["success"] = False
            report["delivery"] = {}
            run_tray_delivery(coordinator.get_instance(ControlCoordinator), sim, report["delivery"])
            report["success"] = report["delivery"]["success"]
            report["final"] = sim.task_state()
        if report["success"] and args.transport_x is not None:
            report["success"] = False
            control = coordinator.get_instance(ControlCoordinator)
            path = sim.plan_transport(args.transport_x, args.transport_y)
            points = [TrajectoryPoint(positions=path[0], velocities=[0.0] * 3, time_from_start=0.0)]
            duration = 0.0
            for first, second in pairwise(path):
                duration += max(
                    0.5, 1.5 * float(np.linalg.norm(np.array(second[:2]) - first[:2])) / 0.1
                )
                points.append(
                    TrajectoryPoint(
                        positions=second, velocities=[0.0] * 3, time_from_start=duration
                    )
                )
            accepted = control.execute_trajectory(
                JointTrajectory(joint_names=list(VIRTUAL_BASE_JOINTS), points=points),
                "base_transport",
            )
            if accepted.status is not TrajectoryExecutionStatus.ACCEPTED:
                raise RuntimeError(f"Transport trajectory rejected: {accepted}")
            transport_history = []
            deadline = time.monotonic() + duration + 5
            stable_since = None
            try:
                while time.monotonic() < deadline:
                    state = sim.task_state()
                    transport_history.append(state)
                    if state["obstacles"]:
                        raise RuntimeError(f"Robot contacted an obstacle: {state['obstacles']}")
                    if not state["inside_bin"]:
                        raise RuntimeError("Bottle left the onboard tray during transport")
                    reached = (
                        np.linalg.norm(
                            np.array(state["base_pose"][:2]) - [args.transport_x, args.transport_y]
                        )
                        < 0.015
                    )
                    quiet = np.linalg.norm(state["base_velocity"]) < 0.02
                    if reached and quiet and state["settled"]:
                        stable_since = time.monotonic() if stable_since is None else stable_since
                        if time.monotonic() - stable_since >= 1:
                            break
                    else:
                        stable_since = None
                    time.sleep(0.05)
                else:
                    raise RuntimeError("Transport did not reach and settle at the destination")
            finally:
                control.cancel_trajectory("base_transport")
                report["transport"] = {
                    "path": path,
                    "history": transport_history,
                    "final": sim.task_state(),
                }
            report["transport"]["success"] = True
            report["success"] = True
        print(
            json.dumps(
                {
                    key: value
                    for key, value in report.items()
                    if key not in ("history", "transport", "delivery")
                },
                indent=2,
            ),
            flush=True,
        )
        (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        if args.stay_open:
            print(
                "Policy stopped. Close the MuJoCo window or press Ctrl-C here to close this stack.",
                flush=True,
            )
            while sim.is_simulation_running():
                time.sleep(0.25)
        return report
    except Exception as error:
        report.update(success=False, error=str(error))
        raise
    finally:
        try:
            if coordinator is not None:
                coordinator.stop()
        finally:
            (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--zenoh-scout-addr", required=True, help="Separate multicast address:port")
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--seconds", type=float, default=25)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--stay-open", action="store_true")
    parser.add_argument("--scene-package", type=Path)
    parser.add_argument("--mobile", action="store_true")
    parser.add_argument(
        "--deliver-to-laptop",
        action="store_true",
        help="ACT loads a free tray; both hands carry it to the house laptop table",
    )
    parser.add_argument("--transport-x", type=float)
    parser.add_argument("--transport-y", type=float)
    args = parser.parse_args()
    if args.deliver_to_laptop:
        if (
            args.scene_package is None
            or args.transport_x is not None
            or args.transport_y is not None
        ):
            parser.error(
                "--deliver-to-laptop requires --scene-package and chooses its own destination"
            )
        args.mobile = True
    if not 0 < args.seconds <= 120:
        parser.error("Duration must be between 0 and 120 seconds")
    if (args.transport_x is None) != (args.transport_y is None):
        parser.error("Specify both transport coordinates")
    if args.transport_x is not None and not args.mobile:
        parser.error("Transport requires --mobile")
    try:
        report = run(args)
    except DemoSessionInUseError as error:
        parser.exit(2, f"{error}\n")
    if not report["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
