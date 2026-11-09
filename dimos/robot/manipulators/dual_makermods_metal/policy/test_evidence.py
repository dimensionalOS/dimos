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

"""End-to-end parity against the accepted checkpoint's own evaluation rollouts.

These tests need the exported artifact and the recorded rollout archive; both
are build evidence, not repository content. Set:

    DIMOS_METAL_POLICY_MODEL_DIR     manifest + .onnx + .normalizer.npz
    DIMOS_METAL_POLICY_EVIDENCE_DIR  rollouts.npz + action_trace.npz

and the runtime is compared step by step with what the training controller did.
"""

import os

import numpy as np
import pytest

from dimos.robot.manipulators.dual_makermods_metal.policy.contract import PolicyManifest
from dimos.robot.manipulators.dual_makermods_metal.policy.runtime import MetalArmPolicyRuntime
from dimos.robot.manipulators.dual_makermods_metal.policy.station import MetalStationKinematics

MODEL_DIR = os.environ.get("DIMOS_METAL_POLICY_MODEL_DIR")
EVIDENCE_DIR = os.environ.get("DIMOS_METAL_POLICY_EVIDENCE_DIR")
pytestmark = pytest.mark.skipif(
    not MODEL_DIR or not EVIDENCE_DIR,
    reason="set DIMOS_METAL_POLICY_MODEL_DIR and DIMOS_METAL_POLICY_EVIDENCE_DIR",
)


@pytest.fixture(scope="module")
def runtime() -> MetalArmPolicyRuntime:
    manifest = PolicyManifest.load(os.path.join(MODEL_DIR, "metal_arm_policy.manifest.json"))
    assert manifest.checkpoint_iteration == 38000
    return MetalArmPolicyRuntime(
        manifest, MODEL_DIR, kinematics=MetalStationKinematics(manifest.station)
    )


def test_manifest_pins_the_accepted_checkpoint(runtime):
    assert (
        runtime.manifest.checkpoint_sha256
        == "5bc5a75fa9e2762567958b5b4b110bdf864fea5988e2419746f5c7730aebb23f"
    )
    assert runtime.manifest.recipe_variant == "cmdlp005-ori10-rawex001-v1"


def test_runtime_reproduces_every_recorded_action_and_command(runtime):
    roll = np.load(os.path.join(EVIDENCE_DIR, "rollouts.npz"))
    trace = np.load(os.path.join(EVIDENCE_DIR, "action_trace.npz"))
    q, dq, issued = roll["q"], roll["dq"], roll["issued"]
    target, actual = roll["target"], roll["actual"]
    previous = trace["clipped_request"]
    home = np.asarray(runtime.manifest.home, dtype=np.float32)
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
