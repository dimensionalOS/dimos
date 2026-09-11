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

"""Run five-bottle ACT packing through DimOS with a native MuJoCo display."""

import argparse
import json
from pathlib import Path
import time
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.imitation.policy.module import POLICY_ROLLOUT_INSTANCE_NAME
from dimos.robot.galaxea.r1pro.navigation_blueprint import build_r1pro_packing_navigation
from dimos.robot.galaxea.r1pro.navigation_cloud import save_environment_cloud
from dimos.robot.galaxea.r1pro.navigation_delivery import prepare_navigation_map
from dimos.robot.galaxea.r1pro.packing_blueprint import R1ProPackingSim, build_r1pro_packing
from dimos.robot.galaxea.r1pro.packing_sim import (
    PACKING_BODIES,
    PACKING_SOURCES,
    prepare_packing_scene,
)
from dimos.robot.galaxea.r1pro.sim_session import reserve_demo_session
from dimos.robot.galaxea.r1pro.tray_delivery import run_tray_delivery


def run(args: argparse.Namespace) -> dict[str, Any]:
    with reserve_demo_session(args.zenoh_scout_addr, args.output):
        return _run(args)


def _run(args: argparse.Namespace) -> dict[str, Any]:
    args.output.mkdir(parents=True, exist_ok=True)
    scene = prepare_packing_scene(args.output / "scene.xml", scene_package=args.scene_package)
    tree = ET.parse(scene)
    rng = np.random.default_rng(args.seed)
    for name, xy in zip(PACKING_BODIES, PACKING_SOURCES, strict=True):
        body = tree.find(f'.//body[@name="{name}"]')
        assert body is not None
        x, y = np.asarray(xy) + rng.uniform(-args.jitter, args.jitter, 2)
        body.set("pos", f"{x} {y} 0.771")
    tree.write(scene, encoding="unicode")
    navigation_cloud = None
    if args.kronknav:
        navigation_cloud = args.output / "navigation-cloud.npy"
        save_environment_cloud(scene, navigation_cloud)
    builder = build_r1pro_packing_navigation if args.kronknav else build_r1pro_packing
    blueprint = builder(
        scene_path=scene, artifact=str(args.artifact), headless=args.no_viewer
    ).global_config(
        viewer="none",
        n_workers=4 if args.kronknav else 3,
        zenoh_scout_addr=args.zenoh_scout_addr,
    )
    coordinator: ModuleCoordinator | None = None
    report: dict[str, Any] = {
        "artifact": str(args.artifact.resolve()),
        "seed": args.seed,
        "order_mode": "random" if args.random_order else "left_to_right",
        "success": False,
        "delivery_requested": args.deliver_to_laptop,
        "picks": [],
    }
    try:
        coordinator = ModuleCoordinator.build(blueprint)
        policy = coordinator.get_instance(POLICY_ROLLOUT_INSTANCE_NAME)
        sim = coordinator.get_instance(R1ProPackingSim)
        if navigation_cloud is not None:
            prepare_navigation_map(sim, navigation_cloud)
        deadline = time.monotonic() + 30
        while not sim.packing_state()["ready_for_pick"]:
            if time.monotonic() >= deadline:
                raise RuntimeError("Simulation did not settle during startup")
            time.sleep(0.05)
        report["order"] = sim.packing_order(args.seed if args.random_order else None)
        report["initial"] = sim.packing_state()
        for index in report["order"]:
            pick: dict[str, Any] = {"bottle": index + 1, "history": [], "success": False}
            report["picks"].append(pick)
            selected = sim.select_bottle(index)
            pick["selection"] = selected
            if not selected["selected"]:
                report["completion_reason"] = selected["reason"]
                break
            policy.clear_rollout_observations()
            deadline = time.monotonic() + 45
            while True:
                status = policy.preflight_rollout()
                if (
                    status["policy_ready"]
                    and status["observations_ready"]
                    and not status["last_error"]
                ):
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError(f"Packing preflight failed: {status}")
                time.sleep(0.1)
            status = policy.start_rollout()
            if not status["active"]:
                raise RuntimeError(f"Packing policy did not start: {status}")
            deadline = time.monotonic() + args.seconds
            stable_since = None
            try:
                while time.monotonic() < deadline:
                    status = policy.rollout_status()
                    if status["last_error"] or not status["active"]:
                        raise RuntimeError(f"Packing rollout failed: {status}")
                    state = sim.packing_state()
                    pick["history"].append(state)
                    if state["selected"]["pick_complete"]:
                        stable_since = time.monotonic() if stable_since is None else stable_since
                        if time.monotonic() - stable_since >= 0.1:
                            pick["success"] = True
                            break
                    else:
                        stable_since = None
                    time.sleep(0.05)
            finally:
                pick["stopped"] = policy.stop_rollout()
                # Cancel at arrival, then verify while the coordinator holds.
                # Keeping ACT active here can start another approach to the old goal.
                time.sleep(0.5)
                pick["final"] = sim.packing_state()
                pick["success"] = bool(
                    pick["success"] and pick["final"]["selected"]["pick_complete"]
                )
                (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
            if pick["stopped"]["active"] or pick["stopped"]["last_error"]:
                raise RuntimeError(f"Packing policy did not stop cleanly: {pick['stopped']}")
            print(
                json.dumps({key: value for key, value in pick.items() if key != "history"}),
                flush=True,
            )
            if not pick["success"]:
                report["completion_reason"] = "pick_failed"
                break
        else:
            report["completion_reason"] = "completed"
        report["final"] = sim.packing_state()
        report["success"] = (
            report["final"]["success"] and report["completion_reason"] == "completed"
        )
        report["packing_success"] = report["success"]
        if report["packing_success"] and args.deliver_to_laptop:
            report["packing_final"] = report["final"]
            report["success"] = False
            report["completion_reason"] = "delivery_in_progress"
            report["delivery"] = {}
            (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
            run_tray_delivery(
                coordinator.get_instance(ControlCoordinator),
                sim,
                report["delivery"],
                navigation_cloud=navigation_cloud,
            )
            report["final"] = sim.task_state()
            report["success"] = bool(report["delivery"]["success"] and report["final"]["success"])
            report["completion_reason"] = "delivered" if report["success"] else "delivery_failed"
        print(
            json.dumps(
                {key: value for key, value in report.items() if key not in ("picks", "delivery")},
                indent=2,
            ),
            flush=True,
        )
        (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        if args.stay_open:
            print("Packing stopped. Close the MuJoCo window or press Ctrl-C.", flush=True)
            while sim.is_simulation_running():
                time.sleep(0.25)
        return report
    except Exception as error:
        report.update(success=False, error=str(error))
        if report.get("completion_reason") == "delivery_in_progress":
            report["completion_reason"] = "delivery_failed"
        raise
    finally:
        # Persist diagnostics before teardown: Ctrl-C must not discard pick history.
        (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        if coordinator is not None:
            coordinator.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--zenoh-scout-addr", required=True)
    parser.add_argument("--scene-package", type=Path)
    parser.add_argument(
        "--deliver-to-laptop",
        action="store_true",
        help="After ACT packs five bottles, carry and place the tray using planned trajectories",
    )
    parser.add_argument(
        "--kronknav",
        action="store_true",
        help="Plan loaded-tray travel from the whole house cloud and execute with the holonomic task",
    )
    parser.add_argument("--seed", type=int, default=9000)
    parser.add_argument(
        "--random-order",
        action="store_true",
        help="Randomize accessible source choices for robustness testing",
    )
    parser.add_argument("--jitter", type=float, default=0.003)
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--stay-open", action="store_true")
    args = parser.parse_args()
    if args.kronknav and not args.deliver_to_laptop:
        parser.error("--kronknav requires --deliver-to-laptop")
    if args.deliver_to_laptop and args.scene_package is None:
        parser.error("--deliver-to-laptop requires --scene-package")
    if not 0 <= args.jitter <= 0.01 or not 0 < args.seconds <= 120:
        parser.error("Use jitter up to one centimetre and positive pick duration up to 120 seconds")
    try:
        report = run(args)
    except KeyboardInterrupt:
        raise SystemExit(130) from None
    if not report["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
