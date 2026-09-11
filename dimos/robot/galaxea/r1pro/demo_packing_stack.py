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
from dimos.robot.galaxea.r1pro.packing_blueprint import R1ProPackingSim, build_r1pro_packing
from dimos.robot.galaxea.r1pro.packing_run import PackingRunConfig, run_packing_sequence
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
        run_packing_sequence(
            coordinator.get_instance(ControlCoordinator),
            policy,
            sim,
            PackingRunConfig(
                artifact=args.artifact,
                output=args.output,
                seed=args.seed,
                random_order=args.random_order,
                seconds=args.seconds,
                deliver_to_laptop=args.deliver_to_laptop,
            ),
            report,
            navigation_cloud=navigation_cloud,
        )
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
