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

"""Reference frames and targets from the Python runners (onnxruntime) for the Rust controller's tests.

    python -m dimos.simulation.m20_policy_sim.golden   # writes $M20_POLICY_DIR/golden.json

Derived from the vendor policies, so it lives next to them and is not committed.
"""

from __future__ import annotations

import json

import numpy as np

from dimos.simulation.m20_policy_sim.config import load_config, policy_dir
from dimos.simulation.m20_policy_sim.runner import RobotState, Runner

STEPS = 4


def main() -> None:
    cfg = load_config()
    rng = np.random.default_rng(7)
    out = {}
    for name, spec in cfg["runners"].items():
        runner = Runner(name, spec, cfg["robot"], policy_dir())
        runner.policy_dt = cfg["timing"]["policy_decimation"] / cfg["timing"]["control_hz"]
        hm = next((b for b in spec["obs"] if b["kind"] == "height_map"), None)
        steps = []
        for _ in range(STEPS):
            quat = np.array([1.0, *rng.normal(0, 0.05, 3)])
            quat /= np.linalg.norm(quat)
            s = RobotState(
                quat=quat,
                gyro=rng.normal(0, 0.3, 3),
                accel=np.array([0, 0, 9.81]) + rng.normal(0, 0.3, 3),
                q=np.array(spec["default_pose"]) + rng.normal(0, 0.1, 16),
                dq=rng.normal(0, 1.0, 16),
                height_map=None if hm is None else rng.uniform(-0.2, 0.2, hm["rows"] * hm["cols"]),
            )
            axes = rng.uniform(-0.5, 0.5, 3)
            t = runner.step(s, axes)
            frame = runner.last_frame
            steps.append(
                dict(
                    quat=quat.tolist(),
                    gyro=s.gyro.tolist(),
                    accel=s.accel.tolist(),
                    q=s.q.tolist(),
                    dq=s.dq.tolist(),
                    height_map=None if s.height_map is None else s.height_map.tolist(),
                    axes=axes.tolist(),
                    frame=frame.tolist(),
                    target_q=t.q.tolist(),
                    wheel_velocity=t.wheel_velocity.tolist(),
                    action=t.action.tolist(),
                )
            )
        out[name] = steps
    path = policy_dir() / "golden.json"
    path.write_text(json.dumps(out))
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
