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

"""Run trained R1Pro ACT in contact physics, with a native MuJoCo display.

Run inside the isolated LeRobot environment with MuJoCo installed. The native
viewer is the default; --no-viewer supports faster batch evaluation. This runner
uses the same LeRobot backend, IO profile, and scene as the DimOS blueprint.
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import time
from typing import Any

import cv2
from dimos_lerobot.runtime import LeRobotBackend
import mujoco
import mujoco.viewer
import numpy as np
from numpy.typing import NDArray
import torch

from dimos.imitation.policy.lerobot.module import LeRobotPolicyConfig
from dimos.robot.galaxea.r1pro.grasping_sim import prepare_grasping_scene
from dimos.robot.galaxea.r1pro.grasping_task import GraspingTask
from dimos.robot.galaxea.r1pro.learning import (
    R1PRO_PICK_PLACE_FPS,
    R1PRO_PICK_PLACE_IO,
    R1PRO_PICK_PLACE_TASK,
)
from dimos.robot.galaxea.r1pro.tray_sim import prepare_tray_delivery_scene


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    args.output.mkdir(parents=True, exist_ok=True)
    scene = (
        prepare_tray_delivery_scene(args.output / "scene.xml", scene_package=args.scene_package)
        if args.free_tray
        else prepare_grasping_scene(
            args.output / "scene.xml",
            scene_package=args.scene_package,
            mobile=args.mobile,
        )
    )
    torch.set_num_threads(4)
    backend = LeRobotBackend(
        LeRobotPolicyConfig(
            artifact=str(args.artifact),
            device=args.device,
            task=R1PRO_PICK_PLACE_TASK,
        )
    )
    info = backend.load(R1PRO_PICK_PLACE_IO)
    execution_steps = args.action_steps or info.preferred_execution_steps
    if not 1 <= execution_steps <= info.chunk_length:
        raise ValueError("Execution steps must fit the checkpoint action chunk")
    results: list[dict[str, Any]] = []
    with ExitStack() as resources:
        task = resources.enter_context(GraspingTask(scene))
        viewer = None
        if not args.no_viewer:
            viewer = resources.enter_context(mujoco.viewer.launch_passive(task.model, task.data))
            viewer.cam.lookat[:] = (0.3, -0.1, 0.85)
            viewer.cam.distance = 2.5
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25
        for seed in range(args.start_seed, args.start_seed + args.episodes):
            task.reset(seed, args.jitter)
            backend.reset()
            history = []
            episode_resources = resources.enter_context(ExitStack())
            video = None
            renderer = None
            if args.video:
                video = cv2.VideoWriter(
                    str(args.output / f"seed_{seed}.mp4"),
                    cv2.VideoWriter.fourcc(*"mp4v"),
                    R1PRO_PICK_PLACE_FPS,
                    (640, 480),
                )
                if not video.isOpened():
                    raise RuntimeError("Could not create evaluation video")
                episode_resources.callback(video.release)
                renderer = mujoco.Renderer(task.model, 480, 640)
                episode_resources.callback(renderer.close)
            actions: NDArray[np.float32] = np.empty((0, len(task.home)), dtype=np.float32)
            stable_frames = 0
            started = time.monotonic()
            for frame in range(round(args.seconds * R1PRO_PICK_PLACE_FPS)):
                if viewer is not None and not viewer.is_running():
                    raise KeyboardInterrupt("MuJoCo viewer closed")
                frame_start = time.monotonic()
                if len(actions) == 0:
                    actions = backend.predict(task.observation(), R1PRO_PICK_PLACE_TASK)[
                        :execution_steps
                    ]
                    if info.action_lower is not None and info.action_upper is not None:
                        actions = np.clip(actions, info.action_lower, info.action_upper)
                task.step(actions[0])
                actions = actions[1:]
                result = task.result()
                stable_frames = stable_frames + 1 if result.success else 0
                history.append(
                    {
                        "frame": frame,
                        "sim_time": float(task.data.time),
                        "tcp": task.data.site_xpos[task.tcp_id].tolist(),
                        "gripper": float(task.data.qpos[task.qids[-1]]),
                        **result.to_dict(),
                    }
                )
                if renderer is not None and video is not None:
                    renderer.update_scene(task.data, camera="overview")
                    video.write(cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))
                if viewer is not None:
                    viewer.sync()
                    time.sleep(max(0, 1 / R1PRO_PICK_PLACE_FPS - (time.monotonic() - frame_start)))
                # Success is sustained for a second, not a single fortunate frame.
                if stable_frames >= R1PRO_PICK_PLACE_FPS:
                    break
            row = {
                "seed": seed,
                **task.result().to_dict(),
                "success": stable_frames >= R1PRO_PICK_PLACE_FPS,
                "frames": len(history),
                "elapsed_s": time.monotonic() - started,
            }
            results.append(row)
            (args.output / f"seed_{seed}.json").write_text(json.dumps(history, indent=2) + "\n")
            episode_resources.close()
            print(json.dumps(row), flush=True)
            report = {
                "artifact": str(args.artifact.resolve()),
                "action_steps": execution_steps,
                "jitter_m": args.jitter,
                "episodes": results,
                "successes": sum(item["success"] for item in results),
                "total": len(results),
            }
            (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--start-seed", type=int, default=1000)
    parser.add_argument("--seconds", type=float, default=20)
    parser.add_argument("--jitter", type=float, default=0.012)
    parser.add_argument("--action-steps", type=int)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--scene-package", type=Path)
    parser.add_argument("--mobile", action="store_true")
    parser.add_argument(
        "--free-tray",
        action="store_true",
        help="Evaluate ACT bottle loading with the physically supported delivery tray",
    )
    args = parser.parse_args()
    if args.episodes < 1 or not 0 < args.seconds <= 120 or not 0 <= args.jitter <= 0.02:
        parser.error("Invalid episode count, duration, or position jitter")
    report = evaluate(args)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
