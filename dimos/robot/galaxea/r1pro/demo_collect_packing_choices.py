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

"""Collect paired initial choices: identical scene, different selected bottle.

These are successful single-pick demonstrations, supplementary to the complete
five-bottle sequences. Pairing prevents initial scene appearance from identifying
the requested bottle without using the policy's explicit goal input.
"""

import argparse
import json
from pathlib import Path
import time
from typing import Any

import numpy as np

from dimos.robot.galaxea.r1pro.flexible_packing_task import FlexiblePackingTask
from dimos.robot.galaxea.r1pro.learning import (
    R1PRO_PACKING_IO,
    R1PRO_PICK_PLACE_FPS,
    R1PRO_PICK_PLACE_JOINTS,
)
from dimos.robot.galaxea.r1pro.packing_checks import validate_other_bottles
from dimos.robot.galaxea.r1pro.packing_sim import prepare_packing_scene
from dimos.robot.galaxea.r1pro.packing_task import PackingTask


def collect(args: argparse.Namespace) -> None:
    args.output.mkdir(parents=True, exist_ok=True)
    path = args.output / "manifest.json"
    contract = {
        "profile": R1PRO_PACKING_IO.name,
        "version": 2,
        "arbitrary_order": args.arbitrary_order,
        "image_stride": args.image_stride,
        "images": True,
        "fps": R1PRO_PICK_PLACE_FPS,
        "joints": list(R1PRO_PICK_PLACE_JOINTS),
        "jitter_m": args.jitter,
        "scene_package": str(args.scene_package.resolve()),
        "purpose": "paired_initial_choices",
        "choices": list(range(5)) if args.arbitrary_order else [0, 1, 3],
    }
    manifest: dict[str, Any] = {**contract, "episodes": [], "choice_groups": [], "rejected": []}
    if path.exists():
        manifest = json.loads(path.read_text())
        if any(manifest.get(key) != value for key, value in contract.items()):
            raise ValueError("Cannot resume a different choice collection")
    completed = {row["seed"] for row in manifest["choice_groups"] + manifest["rejected"]}
    scene = prepare_packing_scene(args.output / "scene.xml", scene_package=args.scene_package)
    task_type = FlexiblePackingTask if args.arbitrary_order else PackingTask
    with task_type(scene) as task:
        for seed in range(args.start_seed, args.start_seed + args.layouts * 3):
            if len(manifest["choice_groups"]) >= args.layouts:
                break
            if seed in completed:
                continue
            started = time.monotonic()
            records = []
            error = None
            for index in contract["choices"]:
                # Identical initial physics, RGB and joints for all three goals.
                task.reset_packing(seed, args.jitter)
                if not task.select_bottle(index):
                    raise RuntimeError("Initial tray must have an empty slot")
                initial = task.report()["bottles"]
                frames: dict[str, list[Any]] = {}
                try:
                    cameras: dict[str, Any] = {}
                    for frame, (_, action) in enumerate(task.teacher_actions()):
                        observation = task.observation(render_images=frame % args.image_stride == 0)
                        cameras.update(
                            {
                                k: v
                                for k, v in observation.items()
                                if k.startswith("observation.images.")
                            }
                        )
                        for key, value in {**cameras, **observation, "action": action}.items():
                            frames.setdefault(key, []).append(value)
                        task.step(action)
                        validate_other_bottles(initial, task.report()["bottles"], index)
                    if not task.pick_complete():
                        raise RuntimeError(str(task.result().to_dict()))
                except RuntimeError as exc:
                    error = str(exc)
                    break
                name = f"choice_{seed}_{index}.npz"
                arrays: dict[str, Any] = {key: np.stack(values) for key, values in frames.items()}
                np.savez_compressed(args.output / name, **arrays)
                records.append(
                    {
                        "seed": seed,
                        "bottle": index + 1,
                        "file": name,
                        "frames": len(frames["action"]),
                        "success": True,
                        "kind": "initial_choice",
                    }
                )
                print(json.dumps(records[-1]), flush=True)
            row = {
                "seed": seed,
                "success": error is None,
                "error": error,
                "elapsed_s": time.monotonic() - started,
            }
            if error is None:
                manifest["episodes"].extend(records)
                manifest["choice_groups"].append(row)
            else:
                for record in records:
                    (args.output / record["file"]).unlink()
                manifest["rejected"].append(row)
            temporary = path.with_suffix(".tmp")
            temporary.write_text(json.dumps(manifest, indent=2) + "\n")
            temporary.replace(path)
            print(json.dumps(row), flush=True)
        if len(manifest["choice_groups"]) < args.layouts:
            raise RuntimeError("Too many rejected paired-choice groups")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-package", type=Path, required=True)
    parser.add_argument("--layouts", type=int, default=10)
    parser.add_argument("--start-seed", type=int, default=8300)
    parser.add_argument("--jitter", type=float, default=0.003)
    parser.add_argument("--arbitrary-order", action="store_true")
    parser.add_argument("--image-stride", type=int, default=1, choices=range(1, 5))
    args = parser.parse_args()
    if args.layouts < 1 or not 0 <= args.jitter <= 0.01:
        parser.error("Use positive layouts and jitter up to one centimetre")
    collect(args)


if __name__ == "__main__":
    main()
