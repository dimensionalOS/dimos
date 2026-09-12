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

"""Validate selected-object ACT through native DimOS modules and trajectory tasks."""

import argparse
import json
from pathlib import Path
import time
from typing import Any

import numpy as np

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.imitation.policy.lerobot.module import R1ProObjectPackingPolicy
from dimos.imitation.policy.module import POLICY_ROLLOUT_INSTANCE_NAME
from dimos.robot.galaxea.r1pro.grasping_blueprint import build_r1pro_manipulation
from dimos.robot.galaxea.r1pro.object_packing import OBJECT_PACKING_TASK
from dimos.robot.galaxea.r1pro.object_packing_run import run_object_pick
from dimos.robot.galaxea.r1pro.object_packing_scene import prepare_object_scene, sample_layout
from dimos.robot.galaxea.r1pro.object_packing_sim import R1ProObjectPackingSim
from dimos.robot.galaxea.r1pro.object_packing_task import ObjectPackingTask
from dimos.robot.galaxea.r1pro.sim_session import reserve_demo_session


def run(args: argparse.Namespace) -> dict[str, Any]:
    with reserve_demo_session(args.zenoh_scout_addr, args.output):
        args.output.mkdir(parents=True, exist_ok=True)
        layout = sample_layout(args.seed)
        scene = prepare_object_scene(
            args.output / "scene.xml", layout, scene_package=args.scene_package
        )
        scene.with_suffix(".objects.json").write_text(json.dumps(layout.to_dict()))
        with ObjectPackingTask(scene, layout, images=False) as task:
            home = task.home.tolist()
        blueprint = build_r1pro_manipulation(
            scene_path=scene,
            artifact=str(args.artifact),
            device="cuda",
            headless=args.no_viewer,
            simulator=R1ProObjectPackingSim,
            policy_module=R1ProObjectPackingPolicy,
            task_description=OBJECT_PACKING_TASK,
            initial_joint_positions=home,
            background_camera_rendering=True,
            viewer_lookat=(0.3, -0.2, 0.8),
            viewer_distance=2.0,
            viewer_azimuth=130,
            viewer_elevation=-35,
        ).global_config(
            transport="zenoh",
            viewer="none",
            simulation="mujoco",
            n_workers=3,
            zenoh_scout_addr=args.zenoh_scout_addr,
        )
        report: dict[str, Any] = dict(
            seed=args.seed, artifact=str(args.artifact), success=False, picks=[]
        )
        coordinator = None
        try:
            coordinator = ModuleCoordinator.build(blueprint)
            sim = coordinator.get_instance(R1ProObjectPackingSim)
            policy = coordinator.get_instance(POLICY_ROLLOUT_INSTANCE_NAME)
            deadline = time.monotonic() + 30
            while not sim.object_state()["at_home"] or sim.object_state()["sim_time"] < 0.6:
                if time.monotonic() > deadline:
                    raise RuntimeError("Object simulation did not settle at home")
                time.sleep(0.05)
            order = list(
                map(int, np.random.default_rng(args.seed + 127).permutation(len(layout.objects)))
            )
            if args.index is not None:
                order = [args.index]
            for index in order:
                pick: dict[str, Any] = {}
                report["picks"].append(pick)
                run_object_pick(policy, sim, index, pick, seconds=args.seconds)
                print(json.dumps({k: v for k, v in pick.items() if k != "history"}), flush=True)
                if not pick["success"]:
                    break
            report["success"] = len(report["picks"]) == len(order) and all(
                p["success"] for p in report["picks"]
            )
            if args.stay_open:
                print("ACT stopped. Close the viewer or press Ctrl-C.", flush=True)
                while sim.is_simulation_running():
                    time.sleep(0.25)
        except Exception as exc:
            report.update(success=False, error=str(exc))
            raise
        finally:
            (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
            if coordinator is not None:
                coordinator.stop()
        return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-package", type=Path)
    parser.add_argument("--zenoh-scout-addr", required=True)
    parser.add_argument("--seed", type=int, default=210000)
    parser.add_argument("--index", type=int)
    parser.add_argument("--seconds", type=float, default=50)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--stay-open", action="store_true")
    args = parser.parse_args()
    if args.seconds <= 0 or args.seed < 0:
        parser.error("Use a positive timeout and a nonnegative seed")
    if not run(args)["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
