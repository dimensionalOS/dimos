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

"""Evaluate ACT on held-out random objects; no demonstration or grasp fallback."""

import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import time
from typing import Any

from dimos_lerobot.runtime import LeRobotBackend
import mujoco.viewer
import numpy as np
from numpy.typing import NDArray
import torch

from dimos.imitation.policy.lerobot.module import LeRobotPolicyConfig
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_FPS as FPS
from dimos.robot.galaxea.r1pro.object_packing import OBJECT_PACKING_IO, OBJECT_PACKING_TASK
from dimos.robot.galaxea.r1pro.object_packing_scene import prepare_object_scene, sample_layout
from dimos.robot.galaxea.r1pro.object_packing_task import ObjectPackingTask
from dimos.robot.galaxea.r1pro.tray_sim import prepare_tray_delivery_scene


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    args.output.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(4)
    backend = LeRobotBackend(
        LeRobotPolicyConfig(
            artifact=str(args.artifact), device=args.device, task=OBJECT_PACKING_TASK
        )
    )
    info = backend.load(OBJECT_PACKING_IO)
    steps = args.action_steps or info.preferred_execution_steps
    if not 1 <= steps <= info.chunk_length:
        raise ValueError("Execution steps must fit the trained chunk")
    template = prepare_tray_delivery_scene(
        args.output / "object-template.xml", scene_package=args.scene_package
    )
    results = []
    for number, seed in enumerate(range(args.start_seed, args.start_seed + args.layouts)):
        layout = sample_layout(seed, occupied=number % (args.occupied_max + 1))
        scene = prepare_object_scene(args.output / f"scene-{seed}.xml", layout, template=template)
        with ExitStack() as resources:
            task = resources.enter_context(ObjectPackingTask(scene, layout))
            viewer = None
            if not args.no_viewer:
                viewer = resources.enter_context(
                    mujoco.viewer.launch_passive(task.model, task.data)
                )
                viewer.cam.lookat[:] = (0.3, -0.2, 0.8)
                viewer.cam.distance = 2.0
                viewer.cam.azimuth = 130
                viewer.cam.elevation = -35
            candidates = [i for i, o in enumerate(layout.objects) if not o.in_tray]
            order = list(map(int, np.random.default_rng(seed + 127).permutation(candidates)))
            if args.single_pick:
                order = order[:1]
            picks = []
            reason = "completed"
            for index in order:
                if not task.select_object(index):
                    reason = "tray_full"
                    break
                initial = task.inventory()
                backend.reset()
                actions: NDArray[np.float32] = np.empty((0, len(task.home)), dtype=np.float32)
                error = None
                stable = 0
                history = []
                try:
                    for frame in range(round(args.seconds * FPS)):
                        tick = time.monotonic()
                        if viewer is not None and not viewer.is_running():
                            raise KeyboardInterrupt("MuJoCo viewer closed")
                        if len(actions) == 0:
                            actions = backend.predict(task.observation(), OBJECT_PACKING_TASK)[
                                :steps
                            ]
                            if info.action_lower is not None and info.action_upper is not None:
                                actions = np.clip(actions, info.action_lower, info.action_upper)
                        task.step(actions[0])
                        actions = actions[1:]
                        task.validate(initial)
                        stable = stable + 1 if task.pick_complete() else 0
                        history.append(
                            dict(
                                frame=frame,
                                tcp=task.data.site("right_tcp").xpos.tolist(),
                                **task.result().to_dict(),
                            )
                        )
                        if viewer is not None:
                            viewer.sync()
                            time.sleep(max(0, 1 / FPS - (time.monotonic() - tick)))
                        if stable >= 3:
                            break
                    # Cancel exactly as hardware does: hold the last ACT command.
                    hold = task.data.ctrl[task.aids].copy()
                    for _ in range(FPS // 2):
                        task.step(hold)
                        task.validate(initial)
                except RuntimeError as exc:
                    error = str(exc)
                success = error is None and stable >= 3 and task.pick_complete()
                failure_reason = error
                if not success and failure_reason is None:
                    measured = task.result()
                    failure_reason = (
                        "return_home_timeout"
                        if measured.success
                        else "no_bilateral_grasp"
                        if not measured.bilateral_grasp
                        else "insufficient_lift"
                        if measured.peak_lift_m <= 0.06
                        else "placement_failed"
                        if not measured.inside_bin
                        else "release_or_settling_failed"
                    )
                row = dict(
                    selected=index,
                    object=layout.objects[index].name,
                    shape=layout.objects[index].shape,
                    success=success,
                    error=error,
                    failure_reason=failure_reason,
                    result=task.result().to_dict(),
                )
                picks.append(row)
                (args.output / f"rollout-{seed}-{index}.json").write_text(
                    json.dumps(history) + "\n"
                )
                np.savez(
                    args.output / f"state-{seed}-{index}.npz",
                    qpos=task.data.qpos,
                    qvel=task.data.qvel,
                    ctrl=task.data.ctrl,
                )
                print(json.dumps(dict(seed=seed, **row)), flush=True)
                if not success:
                    reason = "pick_failed"
                    break
            results.append(
                dict(
                    seed=seed,
                    layout=layout.to_dict(),
                    order=order,
                    picks=picks,
                    completion_reason=reason,
                    success=reason == "completed"
                    and len(picks) == len(order)
                    and all(p["success"] for p in picks),
                )
            )
            summary = dict(
                artifact=str(args.artifact),
                profile=OBJECT_PACKING_IO.name,
                action_steps=steps,
                layouts=results,
                successes=sum(r["success"] for r in results),
                total=len(results),
                pick_successes=sum(p["success"] for r in results for p in r["picks"]),
                pick_total=sum(len(r["picks"]) for r in results),
                interventions=0,
            )
            (args.output / "result.json").write_text(json.dumps(summary, indent=2) + "\n")
            if args.stay_open and viewer is not None:
                print("ACT stopped. Close the viewer or press Ctrl-C.", flush=True)
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(0.05)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-package", type=Path)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--layouts", type=int, default=8)
    parser.add_argument("--start-seed", type=int, default=200000)
    parser.add_argument("--occupied-max", type=int, choices=range(4), default=0)
    parser.add_argument("--action-steps", type=int)
    parser.add_argument("--seconds", type=float, default=40)
    parser.add_argument("--single-pick", action="store_true")
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--stay-open", action="store_true")
    args = parser.parse_args()
    if args.layouts < 1 or args.seconds <= 0 or args.start_seed < 0:
        parser.error("Use positive layout counts/time and a nonnegative seed")
    result = evaluate(args)
    print(json.dumps({k: v for k, v in result.items() if k != "layouts"}), flush=True)
    if result["successes"] != result["total"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
