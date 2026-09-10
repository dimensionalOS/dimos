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

"""Run a goal-conditioned ACT for five sequential physical bottle placements.

The native MuJoCo display is the default. Geometry only selects the object and
empty slot. Every arm action during a pick comes from ACT; no teacher fallback.
"""

import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import time
from typing import Any

from dimos_lerobot.runtime import LeRobotBackend
import mujoco
import mujoco.viewer
import numpy as np
from numpy.typing import NDArray
import torch

from dimos.imitation.policy.lerobot.module import LeRobotPolicyConfig
from dimos.robot.galaxea.r1pro.learning import (
    R1PRO_PACKING_IO,
    R1PRO_PACKING_TASK,
    R1PRO_PICK_PLACE_FPS as FPS,
)
from dimos.robot.galaxea.r1pro.packing_sim import prepare_packing_scene
from dimos.robot.galaxea.r1pro.packing_task import PackingTask


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    args.output.mkdir(parents=True, exist_ok=True)
    scene = prepare_packing_scene(args.output / "scene.xml", scene_package=args.scene_package)
    torch.set_num_threads(4)
    backend = LeRobotBackend(
        LeRobotPolicyConfig(
            artifact=str(args.artifact),
            device=args.device,
            task=R1PRO_PACKING_TASK,
        )
    )
    info = backend.load(R1PRO_PACKING_IO)
    steps = args.action_steps or info.preferred_execution_steps
    if not 1 <= steps <= info.chunk_length:
        raise ValueError("Execution steps must fit the ACT action chunk")
    results = []
    with ExitStack() as resources:
        task = resources.enter_context(PackingTask(scene))
        viewer = None
        if not args.no_viewer:
            viewer = resources.enter_context(mujoco.viewer.launch_passive(task.model, task.data))
            viewer.cam.lookat[:] = (0.3, -0.2, 0.8)
            viewer.cam.distance = 2.0
            viewer.cam.azimuth = 130
            viewer.cam.elevation = -35
        for seed in range(args.start_seed, args.start_seed + args.episodes):
            task.reset_packing(seed, args.jitter)
            order = task.pick_order(seed)
            picks = []
            reason = "completed"
            started = time.monotonic()
            for index in order:
                if not task.select_bottle(index):
                    reason = "tray_full"
                    break
                backend.reset()
                actions: NDArray[np.float32] = np.empty((0, len(task.home)), dtype=np.float32)
                stable = 0
                history = []
                for frame in range(round(args.seconds * FPS)):
                    tick = time.monotonic()
                    if viewer is not None and not viewer.is_running():
                        raise KeyboardInterrupt("MuJoCo viewer closed")
                    if len(actions) == 0:
                        actions = backend.predict(task.observation(), R1PRO_PACKING_TASK)[:steps]
                        if info.action_lower is not None and info.action_upper is not None:
                            actions = np.clip(actions, info.action_lower, info.action_upper)
                    task.step(actions[0])
                    actions = actions[1:]
                    stable = stable + 1 if task.pick_complete() else 0
                    history.append(
                        {
                            "frame": frame,
                            "tcp": task.data.site_xpos[task.tcp_id].tolist(),
                            **task.result().to_dict(),
                        }
                    )
                    if viewer is not None:
                        viewer.sync()
                        time.sleep(max(0, 1 / FPS - (time.monotonic() - tick)))
                    if stable >= FPS:
                        break
                task.remember_result()
                row = {
                    "bottle": index + 1,
                    "goal": task.goal.tolist(),
                    **task.result().to_dict(),
                    "pick_complete": stable >= FPS,
                    "frames": len(history),
                }
                picks.append(row)
                print(json.dumps({"seed": seed, **row}), flush=True)
                (args.output / f"seed_{seed}_bottle_{index + 1}.json").write_text(
                    json.dumps(history, indent=2) + "\n"
                )
                np.savez(
                    args.output / f"seed_{seed}_bottle_{index + 1}.npz",
                    qpos=task.data.qpos,
                    qvel=task.data.qvel,
                    ctrl=task.data.ctrl,
                )
                if stable < FPS:
                    reason = "pick_failed"
                    break
            report = task.report()
            results.append(
                {
                    "seed": seed,
                    "order": order,
                    "picks": picks,
                    **report,
                    "success": report["success"] and reason == "completed",
                    "completion_reason": reason,
                    "elapsed_s": time.monotonic() - started,
                }
            )
            summary = {
                "artifact": str(args.artifact.resolve()),
                "action_steps": steps,
                "jitter_m": args.jitter,
                "episodes": results,
                "successes": sum(row["success"] for row in results),
                "total": len(results),
            }
            (args.output / "result.json").write_text(json.dumps(summary, indent=2) + "\n")
            print(json.dumps(results[-1]), flush=True)
        if args.stay_open and viewer is not None:
            print("Packing stopped. Close the MuJoCo window or press Ctrl-C.", flush=True)
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
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--start-seed", type=int, default=9000)
    parser.add_argument("--jitter", type=float, default=0.003)
    parser.add_argument("--seconds", type=float, default=22)
    parser.add_argument("--action-steps", type=int)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--stay-open", action="store_true")
    args = parser.parse_args()
    if args.episodes < 1 or not 0 < args.seconds <= 120 or not 0 <= args.jitter <= 0.01:
        parser.error("Invalid episode count, pick duration or position jitter")
    evaluate(args)


if __name__ == "__main__":
    main()
