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

"""Factorial control: independently vary link coverage and velocity permutation.

These four experimental captures use the SAME dict schema, CBOR codec and
consumer. Neither the extra fields in Pim's Frame wrapper nor Jeff's wire format
are involved. This isolates representation from the np.roll constant factor.
"""

from __future__ import annotations

import argparse
import json
import random
import time
from typing import Any

import mujoco
import numpy as np

from dimos.protocol.encode import cbor
from dimos.sim2.experiments.demo_state_scaling import (
    OUTPUT,
    WorldPair,
    max_error,
    provenance,
    reference,
    summary,
)


def capture(pair: WorldPair, all_links: bool, roll: bool) -> dict[str, Any]:
    model, data = pair.pim.model, pair.pim.data
    ids = np.arange(1, model.nbody) if all_links else pair.root_ids
    poses = np.column_stack((data.xpos[ids], data.xquat[ids][:, [1, 2, 3, 0]]))
    twists = np.empty((len(ids), 6))
    velocity = np.empty(6)
    for i, body in enumerate(ids):
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY, int(body), velocity, 0)
        if roll:
            twists[i] = np.roll(velocity, 3)
        else:
            twists[i, :3], twists[i, 3:] = velocity[3:], velocity[:3]
    return {
        "epoch": "experiment",
        "tick": round(data.time / pair.pim.dt),
        "sim_time": float(data.time),
        "wall_time": time.time(),
        "revision": pair.pim.layout.revision,
        "poses": poses,
        "twists": twists,
        "position": data.qpos[pair.pim._q].copy(),
        "velocity": data.qvel[pair.pim._v].copy(),
        "effort": data.qfrc_actuator[pair.pim._v].copy(),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="representation")
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--frames", type=int, default=40)
    args = parser.parse_args()
    directory = OUTPUT / args.name
    directory.mkdir(parents=True, exist_ok=False)
    record: dict[str, Any] = {
        "provenance": provenance(),
        "protocol": vars(args),
        "representation": [],
    }
    rng = random.Random(20261102)
    for robots, props in [(1, 32), (32, 32), (1, 2048)]:
        pair = WorldPair.build(directory / f"scene-{robots}-{props}", props, robots)
        for repeat in range(args.repeats):
            cases = [(a, b) for a in (False, True) for b in (False, True)]
            rng.shuffle(cases)
            for all_links, roll in cases:
                samples = []
                error = 0.0
                pair.reset()
                rows = pair.root_rows if all_links else np.arange(len(pair.root_ids))
                for frame in range(args.frames + 12):
                    pair.pose_sequence(frame, 0.01)
                    start = time.perf_counter_ns()
                    state = capture(pair, all_links, roll)
                    a = time.perf_counter_ns()
                    payload = cbor.encode(state)
                    b = time.perf_counter_ns()
                    decoded = cbor.decode(payload)
                    usable = (
                        decoded["poses"][rows],
                        np.column_stack([decoded[n] for n in ("position", "velocity", "effort")]),
                        decoded["twists"][rows],
                    )
                    end = time.perf_counter_ns()
                    error = max(error, max_error(usable, reference(pair)))
                    if frame >= 12:
                        samples.append(
                            [(a - start) / 1000, (b - a) / 1000, (end - b) / 1000, len(payload)]
                        )
                if error > 1e-12:
                    raise AssertionError(f"Factorial control state mismatch: {error}")
                result = {
                    "robots": robots,
                    "props": props,
                    "all_links": all_links,
                    "roll": roll,
                    "repeat": repeat,
                    "error": error,
                    "samples": samples,
                    "total_us": summary([sum(s[:3]) for s in samples]),
                    "bytes": summary([s[3] for s in samples]),
                }
                record["representation"].append(result)
                (directory / "results.json").write_text(json.dumps(record, indent=2))
                print(json.dumps({k: v for k, v in result.items() if k != "samples"}), flush=True)


if __name__ == "__main__":
    main()
