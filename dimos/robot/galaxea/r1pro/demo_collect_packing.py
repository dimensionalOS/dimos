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

"""Collect complete five-bottle sequences, retaining each pick as an ACT episode.

All five picks in a scene share a sequence seed. Only fully successful scenes
enter the dataset, ordered by sequence so evaluation can hold out whole scenes.
Use --no-images for a quick physical teacher check before expensive collection.
"""

import argparse
import json
from pathlib import Path
import time
from typing import Any

import numpy as np

from dimos.robot.galaxea.r1pro.learning import (
    R1PRO_PACKING_IO,
    R1PRO_PICK_PLACE_FPS,
    R1PRO_PICK_PLACE_JOINTS,
)
from dimos.robot.galaxea.r1pro.packing_sim import prepare_packing_scene
from dimos.robot.galaxea.r1pro.packing_task import PackingTask


def collect(args: argparse.Namespace) -> None:
    args.output.mkdir(parents=True, exist_ok=True)
    manifest_path = args.output / "manifest.json"
    contract = {
        "profile": R1PRO_PACKING_IO.name,
        "version": 2,
        "fps": R1PRO_PICK_PLACE_FPS,
        "joints": list(R1PRO_PICK_PLACE_JOINTS),
        "jitter_m": args.jitter,
        "images": not args.no_images,
        "scene_package": str(args.scene_package.resolve()) if args.scene_package else None,
    }
    manifest: dict[str, Any] = {**contract, "episodes": [], "sequences": [], "rejected": []}
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        if any(manifest.get(key) != value for key, value in contract.items()):
            raise ValueError("Cannot resume with a different collection contract")
    completed = {row["seed"] for row in manifest["sequences"] + manifest["rejected"]}
    scene = prepare_packing_scene(args.output / "scene.xml", scene_package=args.scene_package)
    with PackingTask(scene, images=not args.no_images) as task:
        for seed in range(args.start_seed, args.start_seed + args.sequences * 3):
            if len(manifest["sequences"]) >= args.sequences:
                break
            if seed in completed:
                continue
            started = time.monotonic()
            task.reset_packing(seed, args.jitter)
            order = task.pick_order(seed)
            records = []
            error = None
            try:
                for pick, index in enumerate(order):
                    if not task.select_bottle(index):
                        raise RuntimeError("No feasible empty slot remains")
                    frames: dict[str, list[Any]] = {}
                    phases = []
                    for phase, action in task.teacher_actions():
                        for key, value in {**task.observation(), "action": action}.items():
                            frames.setdefault(key, []).append(value)
                        phases.append(phase)
                        task.step(action)
                    task.remember_result()
                    if not task.pick_complete():
                        raise RuntimeError(f"Pick {pick + 1} failed: {task.result().to_dict()}")
                    name = f"sequence_{seed:05d}_pick_{pick}.npz"
                    arrays: dict[str, Any] = {
                        key: np.stack(values) for key, values in frames.items()
                    }
                    np.savez_compressed(args.output / name, **arrays, phase=np.asarray(phases))
                    records.append(
                        {
                            "seed": seed,
                            "pick": pick,
                            "bottle": index + 1,
                            "file": name,
                            "frames": len(phases),
                            "success": True,
                        }
                    )
                    print(json.dumps({"seed": seed, "pick": pick + 1, "success": True}), flush=True)
            except RuntimeError as exc:
                error = str(exc)
            report = task.report()
            row = {
                "seed": seed,
                "order": order,
                **report,
                "error": error,
                "elapsed_s": round(time.monotonic() - started, 2),
            }
            if error is None and report["success"]:
                manifest["sequences"].append(row)
                manifest["episodes"].extend(records)
            else:
                manifest["rejected"].append(row)
                # Partial sequences are not demonstrations of successful packing.
                for record in records:
                    (args.output / record["file"]).unlink()
            temporary = manifest_path.with_suffix(".tmp")
            temporary.write_text(json.dumps(manifest, indent=2) + "\n")
            temporary.replace(manifest_path)
            print(json.dumps(row), flush=True)
        if len(manifest["sequences"]) < args.sequences:
            raise RuntimeError("Too many rejected full sequences; inspect the teacher")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-package", type=Path)
    parser.add_argument("--sequences", type=int, default=20)
    parser.add_argument("--start-seed", type=int, default=8000)
    parser.add_argument("--jitter", type=float, default=0.006)
    parser.add_argument("--no-images", action="store_true")
    args = parser.parse_args()
    if args.sequences < 1 or not 0 <= args.jitter <= 0.01:
        parser.error("Use positive sequences and jitter up to one centimetre")
    collect(args)


if __name__ == "__main__":
    main()
