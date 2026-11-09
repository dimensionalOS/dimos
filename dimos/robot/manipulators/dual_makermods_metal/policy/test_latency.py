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

"""CPU latency budget of the exported Metal policy runtime.

The coordinator ticks at 20 Hz, so one whole runtime tick - observation build
(including station FK), ONNX actor call and action decode - must fit inside
50 ms with room for transport and plant jitter. These are wall-clock
measurements on the deployment host's CPU with the CPUExecutionProvider; they
are evidence for the operator docs, not a portable guarantee.
"""

import os
import time

import numpy as np
import pytest

from dimos.robot.manipulators.dual_makermods_metal.policy.contract import PolicyManifest
from dimos.robot.manipulators.dual_makermods_metal.policy.runtime import (
    MetalArmPolicyRuntime,
)
from dimos.robot.manipulators.dual_makermods_metal.policy.station import (
    MetalStationKinematics,
)

MODEL_DIR = os.environ.get("DIMOS_METAL_POLICY_MODEL_DIR")
EVIDENCE_DIR = os.environ.get("DIMOS_METAL_POLICY_EVIDENCE_DIR") or MODEL_DIR
pytestmark = pytest.mark.skipif(
    not MODEL_DIR,
    reason="set DIMOS_METAL_POLICY_MODEL_DIR (and DIMOS_METAL_POLICY_EVIDENCE_DIR for states)",
)

ITERATIONS = 300
WARMUP = 20
#: One policy tick budget at 20 Hz is 50 ms; leave half for the rest of the loop.
FULL_TICK_P95_BUDGET_MS = 20.0
INFERENCE_P95_BUDGET_MS = 5.0


def _runtime() -> MetalArmPolicyRuntime:
    assert MODEL_DIR is not None
    manifest = PolicyManifest.load(os.path.join(MODEL_DIR, "metal_arm_policy.manifest.json"))
    kinematics = MetalStationKinematics(manifest.station)
    return MetalArmPolicyRuntime(
        manifest, MODEL_DIR, providers=["CPUExecutionProvider"], kinematics=kinematics
    )


def _states() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    assert EVIDENCE_DIR is not None
    rollouts = np.load(os.path.join(EVIDENCE_DIR, "rollouts.npz"))
    return (
        np.asarray(rollouts["q"][:, 0], dtype=np.float64),
        np.asarray(rollouts["dq"][:, 0], dtype=np.float64),
        np.asarray(rollouts["target"][:, 0], dtype=np.float64),
    )


def _percentile(values: list[float], percentile: float) -> float:
    return float(np.percentile(np.asarray(values), percentile))


def test_cpu_runtime_tick_fits_the_20hz_budget():
    runtime = _runtime()
    q, dq, goals = _states()
    steps = min(len(q), ITERATIONS + WARMUP)
    infer_ms: list[float] = []
    full_ms: list[float] = []
    for step in range(steps):
        start = time.perf_counter_ns()
        runtime.observe(q[step], dq[step], goals[step])
        observation = runtime.observation()
        infer_start = time.perf_counter_ns()
        action = runtime.infer(observation)
        infer_end = time.perf_counter_ns()
        runtime.decode_action(action)
        end = time.perf_counter_ns()
        if step >= WARMUP:
            infer_ms.append((infer_end - infer_start) / 1e6)
            full_ms.append((end - start) / 1e6)

    infer_p50 = _percentile(infer_ms, 50.0)
    infer_p95 = _percentile(infer_ms, 95.0)
    full_p50 = _percentile(full_ms, 50.0)
    full_p95 = _percentile(full_ms, 95.0)
    print(
        f"CPU latency over {len(full_ms)} ticks: infer p50={infer_p50:.3f} ms "
        f"p95={infer_p95:.3f} ms; full tick p50={full_p50:.3f} ms p95={full_p95:.3f} ms"
    )
    assert infer_p95 < INFERENCE_P95_BUDGET_MS, f"actor too slow: {infer_p95:.3f} ms p95"
    assert full_p95 < FULL_TICK_P95_BUDGET_MS, f"runtime tick too slow: {full_p95:.3f} ms p95"
