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

"""Artifact-backed evidence for the accepted Metal policy.

Both checks need the exported artifact and the recorded rollout archive; these
are build evidence, not repository content. Set:

    DIMOS_METAL_POLICY_MODEL_DIR     manifest + .onnx + .normalizer.npz
    DIMOS_METAL_POLICY_EVIDENCE_DIR  rollouts.npz + action_trace.npz

The parity test compares the runtime step by step with the training
controller's own evaluation rollouts. The latency test measures whole runtime
ticks on this host's CPU: one 20 Hz policy tick must fit inside 50 ms with room
for transport and plant jitter.
"""

import os
import time

import numpy as np
import pytest

from dimos.robot.manipulators.dual_makermods_metal.policy.contract import PolicyManifest
from dimos.robot.manipulators.dual_makermods_metal.policy.runtime import MetalArmPolicyRuntime
from dimos.robot.manipulators.dual_makermods_metal.policy.station import MetalStationKinematics

MODEL_DIR = os.environ.get("DIMOS_METAL_POLICY_MODEL_DIR") or ""
EVIDENCE_DIR = os.environ.get("DIMOS_METAL_POLICY_EVIDENCE_DIR") or MODEL_DIR
pytestmark = pytest.mark.skipif(
    not MODEL_DIR or not EVIDENCE_DIR,
    reason="set DIMOS_METAL_POLICY_MODEL_DIR and DIMOS_METAL_POLICY_EVIDENCE_DIR",
)

ITERATIONS = 300
WARMUP = 20
#: One policy tick budget at 20 Hz is 50 ms; leave half for the rest of the loop.
FULL_TICK_P95_BUDGET_MS = 20.0
INFERENCE_P95_BUDGET_MS = 5.0


@pytest.fixture(scope="module")
def runtime() -> MetalArmPolicyRuntime:
    manifest = PolicyManifest.load(os.path.join(MODEL_DIR, "metal_arm_policy.manifest.json"))
    assert manifest.checkpoint.iteration == 38000
    return MetalArmPolicyRuntime(
        manifest, MODEL_DIR, kinematics=MetalStationKinematics(manifest.station)
    )


def test_manifest_pins_the_accepted_checkpoint(runtime):
    assert (
        runtime.manifest.checkpoint.sha256
        == "5bc5a75fa9e2762567958b5b4b110bdf864fea5988e2419746f5c7730aebb23f"
    )
    assert runtime.manifest.checkpoint.recipe_variant == "cmdlp005-ori10-rawex001-v1"


def test_runtime_reproduces_every_recorded_action_and_command(runtime):
    roll = np.load(os.path.join(EVIDENCE_DIR, "rollouts.npz"))
    trace = np.load(os.path.join(EVIDENCE_DIR, "action_trace.npz"))
    q, dq, issued = roll["q"], roll["dq"], roll["issued"]
    target, actual = roll["target"], roll["actual"]
    previous = trace["clipped_request"]
    home = np.asarray(runtime.manifest.action.home, dtype=np.float32)
    zero = np.zeros(12, np.float32)
    steps = q.shape[0]
    action_error = 0.0
    issued_error = 0.0
    pose_error = 0.0
    for case in range(q.shape[1]):
        runtime.reset(home)
        runtime.decoder.seed(home, zero)
        runtime.observe(home, zero, target[0, case])
        for step in range(steps):
            action = runtime.infer(runtime.observation())
            action_error = max(
                action_error, float(np.abs(action - trace["actor_preclamp"][step, case]).max())
            )
            command = runtime.act()
            issued_error = max(issued_error, float(np.abs(command - issued[step, case]).max()))
            pose_error = max(
                pose_error,
                float(np.abs(runtime._kinematics.pose24(q[step, case]) - actual[step, case]).max()),
            )
            runtime.decoder.seed(
                issued[step, case].astype(np.float32), previous[step, case].astype(np.float32)
            )
            # The env appends the frame after incrementing its episode counter, so the
            # frame's goal slot is the next bank pose.
            runtime.observe(q[step, case], dq[step, case], target[min(step + 1, steps - 1), case])
    assert pose_error < 1e-5
    assert action_error < 2e-4, f"actor parity broke: {action_error}"
    assert issued_error < 1e-5, f"command decode parity broke: {issued_error}"


def test_cpu_runtime_tick_fits_the_20hz_budget(runtime):
    rollouts = np.load(os.path.join(EVIDENCE_DIR, "rollouts.npz"))
    q = np.asarray(rollouts["q"][:, 0], dtype=np.float64)
    dq = np.asarray(rollouts["dq"][:, 0], dtype=np.float64)
    goals = np.asarray(rollouts["target"][:, 0], dtype=np.float64)
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

    infer_p95 = float(np.percentile(np.asarray(infer_ms), 95.0))
    full_p95 = float(np.percentile(np.asarray(full_ms), 95.0))
    print(
        f"CPU latency over {len(full_ms)} ticks: infer p95={infer_p95:.3f} ms; "
        f"full tick p95={full_p95:.3f} ms"
    )
    assert infer_p95 < INFERENCE_P95_BUDGET_MS, f"actor too slow: {infer_p95:.3f} ms p95"
    assert full_p95 < FULL_TICK_P95_BUDGET_MS, f"runtime tick too slow: {full_p95:.3f} ms p95"
