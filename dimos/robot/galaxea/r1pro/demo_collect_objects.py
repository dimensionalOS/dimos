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

"""Collect independent, physically verified object picks across random layouts."""

import argparse
import json
from pathlib import Path
import time
from typing import Any

import numpy as np

from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_FPS, R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.object_packing import OBJECT_PACKING_IO
from dimos.robot.galaxea.r1pro.object_packing_scene import prepare_object_scene, sample_layout
from dimos.robot.galaxea.r1pro.object_packing_task import ObjectPackingTask
from dimos.robot.galaxea.r1pro.tray_sim import prepare_tray_delivery_scene


def save_manifest(path: Path, manifest: dict[str, Any]) -> None:
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(manifest, indent=2) + "\n")
    temporary.replace(path)


def collect(args: argparse.Namespace) -> dict[str, Any]:
    args.output.mkdir(parents=True, exist_ok=True)
    path = args.output / "manifest.json"
    contract = dict(
        profile=OBJECT_PACKING_IO.name,
        version=1,
        images=not args.no_images,
        fps=R1PRO_PICK_PLACE_FPS,
        joints=list(R1PRO_PICK_PLACE_JOINTS),
        start_seed=args.start_seed,
        layouts=args.layouts,
        choices=args.choices,
        image_stride=args.image_stride,
        occupied_max=args.occupied_max,
        scene_package=str(args.scene_package.resolve()) if args.scene_package else None,
    )
    manifest = (
        json.loads(path.read_text())
        if path.exists()
        else dict(**contract, episodes=[], rejected=[])
    )
    if any(manifest.get(k) != v for k, v in contract.items()):
        raise ValueError("Cannot resume with a different collection contract")
    done = {(row["seed"], row["selected"]) for row in manifest["episodes"] + manifest["rejected"]}
    template = args.output / "object-template.xml"
    if not template.exists():
        prepare_tray_delivery_scene(template, scene_package=args.scene_package)
    for number, seed in enumerate(range(args.start_seed, args.start_seed + args.layouts)):
        try:
            layout = sample_layout(seed, occupied=number % (args.occupied_max + 1))
        except RuntimeError as exc:
            if (seed, -1) not in done:
                manifest["rejected"].append(
                    dict(seed=seed, selected=-1, error=str(exc), phase="generation")
                )
                save_manifest(path, manifest)
            continue
        scene = prepare_object_scene(args.output / f"scene-{seed}.xml", layout, template=template)
        candidates = [i for i, o in enumerate(layout.objects) if not o.in_tray]
        order = np.random.default_rng(seed + 91).permutation(candidates)[: args.choices]
        with ObjectPackingTask(scene, layout, images=not args.no_images) as task:
            for index in map(int, order):
                if (seed, index) in done:
                    continue
                task.reset(seed)
                started = time.monotonic()
                frames: dict[str, list[Any]] = {}
                cameras: dict[str, Any] = {}
                phases = []
                phase = "select"
                try:
                    if not task.select_object(index):
                        raise RuntimeError("No feasible free tray space")
                    initial = task.inventory()
                    task.validate(initial)
                    for frame, (phase, action) in enumerate(task.teacher_actions()):
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
                        phases.append(phase)
                        task.step(action)
                        task.validate(initial)
                    if not task.pick_complete():
                        raise RuntimeError(f"Teacher did not complete: {task.result().to_dict()}")
                    name = f"layout-{seed}-object-{index}.npz"
                    arrays: dict[str, Any] = {k: np.stack(v) for k, v in frames.items()}
                    np.savez_compressed(
                        args.output / name,
                        **arrays,
                        phase=np.asarray(phases),
                    )
                    row = dict(
                        seed=seed,
                        selected=index,
                        shape=layout.objects[index].shape,
                        file=name,
                        frames=len(phases),
                        success=True,
                        layout=layout.to_dict(),
                        seconds=round(time.monotonic() - started, 2),
                    )
                    manifest["episodes"].append(row)
                    print(json.dumps({k: v for k, v in row.items() if k != "layout"}), flush=True)
                except RuntimeError as exc:
                    row = dict(
                        seed=seed,
                        selected=index,
                        shape=layout.objects[index].shape,
                        error=str(exc),
                        phase=phase,
                        layout=layout.to_dict(),
                        seconds=round(time.monotonic() - started, 2),
                        result=task.result().to_dict(),
                    )
                    manifest["rejected"].append(row)
                    np.savez(
                        args.output / f"failure-{seed}-{index}.npz",
                        qpos=task.data.qpos,
                        qvel=task.data.qvel,
                        ctrl=task.data.ctrl,
                    )
                    print(json.dumps({k: v for k, v in row.items() if k != "layout"}), flush=True)
                save_manifest(path, manifest)
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-package", type=Path)
    parser.add_argument("--start-seed", type=int, default=100000)
    parser.add_argument("--layouts", type=int, default=8)
    parser.add_argument("--choices", type=int, choices=range(1, 6), default=2)
    parser.add_argument("--occupied-max", type=int, choices=range(4), default=3)
    parser.add_argument("--image-stride", type=int, choices=range(1, 5), default=2)
    parser.add_argument("--no-images", action="store_true")
    args = parser.parse_args()
    if args.layouts < 1 or args.start_seed < 0:
        parser.error("Use positive layouts and a nonnegative seed")
    result = collect(args)
    attempts = len(result["episodes"]) + len(result["rejected"])
    print(json.dumps(dict(accepted=len(result["episodes"]), attempts=attempts)), flush=True)
    if not result["episodes"]:
        raise SystemExit("No successful demonstrations; inspect rejected attempts")


if __name__ == "__main__":
    main()
