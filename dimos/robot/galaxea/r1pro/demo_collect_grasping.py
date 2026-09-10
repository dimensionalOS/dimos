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

"""Collect successful contact-grasping episodes without an LLM or teleoperation.

Images and measured state are captured immediately before the recorded command
is applied. Raw episodes are local, resumable NPZ files; the isolated LeRobot
converter creates the standard dataset. Failed demonstrations are never saved.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any

import numpy as np

from dimos.robot.galaxea.r1pro.grasping_sim import prepare_grasping_scene
from dimos.robot.galaxea.r1pro.grasping_task import GraspingTask
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_FPS, R1PRO_PICK_PLACE_JOINTS


def collect(
    output: Path,
    episodes: int,
    start_seed: int,
    jitter: float,
    scene_package: Path | None = None,
    mobile: bool = False,
) -> None:
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = output / "manifest.json"
    manifest: dict[str, Any] = {
        "fps": R1PRO_PICK_PLACE_FPS,
        "joints": list(R1PRO_PICK_PLACE_JOINTS),
        "jitter_m": jitter,
        "scene_package": str(scene_package.resolve()) if scene_package else None,
        "mobile": mobile,
        "episodes": [],
        "rejected": [],
    }
    if manifest_path.exists():
        previous = json.loads(manifest_path.read_text())
        for key in ("fps", "joints", "jitter_m", "scene_package", "mobile"):
            if previous.get(key) != manifest[key]:
                raise ValueError(f"Cannot resume a collection with different {key}")
        manifest = previous
    completed = {entry["seed"] for entry in manifest["episodes"] + manifest["rejected"]}
    scene = prepare_grasping_scene(output / "scene.xml", scene_package=scene_package, mobile=mobile)
    with GraspingTask(scene) as task:
        seed = start_seed
        while len(manifest["episodes"]) < episodes:
            if seed in completed:
                seed += 1
                continue
            if seed >= start_seed + episodes * 3:
                raise RuntimeError("Too many rejected demonstrations; inspect the teacher")
            started = time.monotonic()
            task.reset(seed, jitter)
            frames: dict[str, list[Any]] = {}
            phases = []
            try:
                for phase, action in task.teacher_actions():
                    for key, value in {**task.observation(), "action": action}.items():
                        frames.setdefault(key, []).append(value)
                    phases.append(phase)
                    task.step(action)
                result = {"seed": seed, **task.result().to_dict()}
            except RuntimeError as error:
                result = {"seed": seed, "success": False, "error": str(error)}
            if result["success"]:
                name = f"episode_{seed:05d}.npz"
                arrays: dict[str, Any] = {key: np.stack(values) for key, values in frames.items()}
                np.savez_compressed(
                    output / name,
                    **arrays,
                    phase=np.asarray(phases),
                )
                result.update(file=name, frames=len(phases))
                manifest["episodes"].append(result)
            else:
                manifest["rejected"].append(result)
            # Each episode remains inspectable if collection is interrupted.
            temporary = manifest_path.with_suffix(".tmp")
            temporary.write_text(json.dumps(manifest, indent=2) + "\n")
            temporary.replace(manifest_path)
            print(
                json.dumps({**result, "elapsed_s": round(time.monotonic() - started, 2)}),
                flush=True,
            )
            seed += 1


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--episodes", type=int, default=60)
    parser.add_argument("--start-seed", type=int, default=0)
    parser.add_argument("--jitter", type=float, default=0.012)
    parser.add_argument("--scene-package", type=Path)
    parser.add_argument("--mobile", action="store_true")
    args = parser.parse_args()
    if args.episodes < 1 or not 0 <= args.jitter <= 0.02:
        parser.error("Use positive episodes and jitter between 0 and 0.02 metres")
    collect(
        args.output, args.episodes, args.start_seed, args.jitter, args.scene_package, args.mobile
    )


if __name__ == "__main__":
    main()
