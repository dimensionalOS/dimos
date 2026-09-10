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

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.imitation.policy.module import POLICY_ROLLOUT_INSTANCE_NAME
from dimos.robot.galaxea.r1pro.packing_blueprint import R1ProPackingSim, build_r1pro_packing
from dimos.robot.galaxea.r1pro.packing_sim import (
    PACKING_BODIES,
    PACKING_SOURCES,
    prepare_packing_scene,
)
from dimos.robot.galaxea.r1pro.sim_session import reserve_demo_session


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
    blueprint = build_r1pro_packing(
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
        "picks": [],
    }
    try:
        coordinator = ModuleCoordinator.build(blueprint)
        policy = coordinator.get_instance(POLICY_ROLLOUT_INSTANCE_NAME)
        sim = coordinator.get_instance(R1ProPackingSim)
        deadline = time.monotonic() + 30
        while not sim.packing_state()["ready_for_pick"]:
            if time.monotonic() >= deadline:
                raise RuntimeError("Simulation did not settle during startup")
            time.sleep(0.05)
        report["order"] = sim.packing_order(args.seed)
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
                        if time.monotonic() - stable_since >= 1:
                            pick["success"] = True
                            break
                    else:
                        stable_since = None
                    time.sleep(0.05)
            finally:
                pick["stopped"] = policy.stop_rollout()
                pick["final"] = sim.packing_state()
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
        print(
            json.dumps({key: value for key, value in report.items() if key != "picks"}, indent=2),
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
    parser.add_argument("--seed", type=int, default=9000)
    parser.add_argument("--jitter", type=float, default=0.003)
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--stay-open", action="store_true")
    args = parser.parse_args()
    if not 0 <= args.jitter <= 0.01 or not 0 < args.seconds <= 120:
        parser.error("Use jitter up to one centimetre and positive pick duration up to 120 seconds")
    run(args)


if __name__ == "__main__":
    main()
