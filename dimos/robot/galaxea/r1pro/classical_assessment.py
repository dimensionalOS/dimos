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

"""Rank GraspGenX proposals in a separate process so planning cannot starve physics.

The simulator keeps one worker alive and sends it `warm <model>` and
`rank <request dir>` lines on stdin; the worker caches the compiled model and
the kinematics world, which cost tens of seconds to build, and writes
result.json into each request directory. A runaway search is killed, not
waited on.
"""

from __future__ import annotations

from dataclasses import asdict
import json
import os
from pathlib import Path
import sys
import time
import traceback
from typing import Any

import mujoco
import numpy as np
from pydantic import TypeAdapter

from dimos.robot.galaxea.r1pro.classical_planning import ClassicalGraspPlanner
from dimos.robot.galaxea.r1pro.home_kinematics import HomeKinematics
from dimos.robot.galaxea.r1pro.object_packing_scene import ObjectLayout
from dimos.robot.galaxea.r1pro.object_primitive_state import PrimitiveSceneState

Prepared = tuple[mujoco.MjModel, HomeKinematics]


def prepare(models: dict[str, Prepared], model_path: str) -> Prepared:
    """Load the model and build its kinematics world once, at the base's origin pose."""
    if model_path not in models:
        started = time.monotonic()
        model = mujoco.MjModel.from_binary_path(model_path)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        models[model_path] = (model, HomeKinematics(model, data, natural_posture=True))
        print(f"prepared {model_path} in {time.monotonic() - started:.1f}s", flush=True)
    return models[model_path]


def rank_request(request: Path, models: dict[str, Prepared]) -> list[dict[str, Any]]:
    spec = json.loads((request / "request.json").read_text())
    arrays = np.load(request / "request.npz")
    model, kinematics = prepare(models, spec["model"])
    data = mujoco.MjData(model)
    data.qpos[:] = arrays["qpos"]
    data.qvel[:] = arrays["qvel"]
    data.ctrl[:] = arrays["ctrl"]
    mujoco.mj_forward(model, data)
    layout = TypeAdapter(ObjectLayout).validate_python(spec["layout"])
    scene = PrimitiveSceneState(model, data, layout, np.asarray(spec["home"], dtype=float))
    started = time.monotonic()
    planner = ClassicalGraspPlanner(scene, kinematics=kinematics)
    plans = planner.rank(
        int(spec["index"]), arrays["poses"], arrays["scores"], arm=str(spec["arm"])
    )
    print(f"ranked {len(plans)} plans in {time.monotonic() - started:.1f}s", flush=True)
    return [asdict(plan) for plan in plans]


def handle(request: Path, models: dict[str, Prepared]) -> None:
    try:
        result: dict[str, Any] = dict(plans=rank_request(request, models))
    except Exception as exc:
        result = dict(error=f"{type(exc).__name__}: {exc}", traceback=traceback.format_exc())
    partial = request / "result.json.partial"
    partial.write_text(json.dumps(result) + "\n")
    os.replace(partial, request / "result.json")


def serve() -> None:
    models: dict[str, Prepared] = {}
    for line in sys.stdin:
        command, _, argument = line.strip().partition(" ")
        if command == "warm":
            prepare(models, argument)
        elif command == "rank":
            handle(Path(argument), models)
        print(f"done {command} {argument}", flush=True)


def main() -> None:
    if sys.argv[1] == "--serve":
        serve()
    else:
        handle(Path(sys.argv[1]), {})


if __name__ == "__main__":
    main()
