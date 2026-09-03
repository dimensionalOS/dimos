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

"""SPACE egocentric-navigation environment server. Not a suite, not dimos code.

habitat-sim ships conda builds for Python 3.9 only (through 0.3.3), while dimos
requires >=3.10 — the two can never share an interpreter. So the environment
and the benchmark's own metrics run here, under the habitat Python, and the
dimos side (`SpaceNav._ego_episode`) drives this file over line-delimited JSON
on stdio. The model loop, and with it the execution path, stays in dimos.

This file is executed by path (never imported), uses only the stdlib, numpy
and the SPACE checkout, and stays Python-3.9 compatible.

Protocol, one JSON object per line:
    {"cmd": "init", "scene": ..., "repo": ..., "habitat": {...},
     "image_downscaling": ...}                    -> {"task_info": {...}}
    {"cmd": "reset"}                              -> {"obs": <b64 png>}
    {"cmd": "step", "action": "move_forward"}     -> {"obs": <b64 png>}
    {"cmd": "finish", "stop_issued": true}        -> {"metrics": {...}}
Positions are tracked server-side; metrics come from the benchmark's own
DistanceToGoal / Success / SPL classes (space/utils/habitat.py), unmodified.
"""

from __future__ import annotations

import base64
import io
import json
import os
import sys
from typing import Any


def main() -> None:
    # habitat and its C++ renderer write banners to stdout, which is the
    # protocol channel. Keep a private copy of the original stdout for protocol
    # replies and point fd 1 at stderr for everything else, C prints included.
    protocol = os.fdopen(os.dup(1), "w")
    os.dup2(2, 1)
    sys.stdout = sys.stderr

    env: Any = None
    positions: list[Any] = []
    metric_makers: tuple[Any, Any, Any] | None = None

    def send(payload: dict[str, Any]) -> None:
        protocol.write(json.dumps(payload) + "\n")
        protocol.flush()

    def encode_obs(obs: Any) -> str:
        import numpy as np

        buffer = io.BytesIO()
        np.save(buffer, np.asarray(obs), allow_pickle=False)
        return base64.b64encode(buffer.getvalue()).decode("ascii")

    for line in sys.stdin:
        request = json.loads(line)
        command = request["cmd"]
        if command == "init":
            sys.path.insert(0, request["repo"])
            from space.envs.nav_ego import NavEgoEnv
            from space.utils.habitat import SPL, DistanceToGoal, Success

            env = NavEgoEnv(
                request["scene"],
                habitat_kwargs=request["habitat"],
                image_downscaling=request["image_downscaling"],
            )
            task_info = env.get_task_info()
            metric_makers = (
                DistanceToGoal(env.sim, task_info["goal_position"]),
                Success(env.sim, task_info["goal_position"]),
                SPL(env.sim, task_info["start_position"], task_info["goal_position"]),
            )
            send({"task_info": {"goal_desc": task_info["goal_desc"]}})
        elif command == "reset":
            obs = env.reset()
            positions = [env.get_sim_state()[0]]
            send({"obs": encode_obs(obs)})
        elif command == "step":
            obs = env.step(request["action"])
            positions.append(env.get_sim_state()[0])
            send({"obs": encode_obs(obs)})
        elif command == "finish":
            assert metric_makers is not None
            d2g, success, spl = metric_makers
            send(
                {
                    "metrics": {
                        "distance_to_goal": float(d2g(positions)),
                        "success": float(success(request["stop_issued"], positions)),
                        "spl": float(spl(request["stop_issued"], positions)),
                    }
                }
            )
        elif command == "close":
            if env is not None:
                env.close()
            return


if __name__ == "__main__":
    main()
