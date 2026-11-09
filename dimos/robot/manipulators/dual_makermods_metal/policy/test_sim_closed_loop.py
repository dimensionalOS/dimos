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

"""CPU MuJoCo closed-loop evidence for the exported dual Metal policy.

This is not a trace replay. Two identical instances of the *trained* station
plant (``station.xml`` with the training controller's gains, gravity
compensation and 20 Hz decimation) start from the same pose and are driven in
lockstep: one by the exported ONNX runtime, one by the original checkpoint
actor rebuilt from the ``.pt`` file. Both controllers consume only the current
measured state and the current causal goal; the recorded goal sequence is fed
one tick at a time, never ahead, so the test exercises the deployment path
rather than reconstructing training observations offline.

Asset gates (build evidence, not repository content):

    DIMOS_METAL_POLICY_SIM_DIR    station.xml + meshes for the trained plant
    DIMOS_METAL_POLICY_MODEL_DIR  manifest, .onnx, normalizer, checkpoint .pt
    DIMOS_METAL_POLICY_EVIDENCE_DIR  optional; goal sequence (defaults to MODEL_DIR)
"""

import os
from typing import Any

import numpy as np
import pytest

from dimos.robot.manipulators.dual_makermods_metal.policy.contract import (
    JOINT_ORDER,
    PolicyManifest,
    sha256_file,
)
from dimos.robot.manipulators.dual_makermods_metal.policy.runtime import (
    MetalArmPolicyRuntime,
)
from dimos.robot.manipulators.dual_makermods_metal.policy.station import (
    MetalStationKinematics,
)

mujoco = pytest.importorskip("mujoco")

SIM_DIR = os.environ.get("DIMOS_METAL_POLICY_SIM_DIR")
MODEL_DIR = os.environ.get("DIMOS_METAL_POLICY_MODEL_DIR")
EVIDENCE_DIR = os.environ.get("DIMOS_METAL_POLICY_EVIDENCE_DIR") or MODEL_DIR
pytestmark = pytest.mark.skipif(
    not SIM_DIR or not MODEL_DIR,
    reason="set DIMOS_METAL_POLICY_SIM_DIR and DIMOS_METAL_POLICY_MODEL_DIR",
)

CHECKPOINT_FILE = "ori10raw001_038000.pt"

#: Training controller, copied from the candidate's experiment/config: per-joint
#: PD gains, effort limits and the gravity-compensation scale factors.
HOME = np.array([0.0, -1.9, 1.6, -1.25, 0.0, 0.0] * 2, dtype=np.float64)
KP = np.array([100.0, 150.0, 90.0, 45.0, 30.0, 20.0] * 2)
KD = np.array([8.0, 10.0, 6.0, 3.0, 2.0, 1.5] * 2)
EFFORT = np.array([30.0, 100.0, 30.0, 10.0, 10.0, 10.0] * 2)
GRAVITY_SCALE = np.array([1.2, 1.15, 1.1, 1.15, 1.0, 1.0] * 2)
SUBSTEPS = 10
POLICY_TICKS = 60
CASE = 0


class _Plant:
    """The trained station plant: position commands, PD + gravity feedforward."""

    def __init__(self, sim_dir: str) -> None:
        self.model = mujoco.MjModel.from_xml_path(os.path.join(sim_dir, "station.xml"))
        self.data = mujoco.MjData(self.model)
        self.qadr = [int(self.model.joint(name).qposadr[0]) for name in JOINT_ORDER]
        self.dadr = [int(self.model.joint(name).dofadr[0]) for name in JOINT_ORDER]
        self.subtrees: list[tuple[int, np.ndarray]] = []
        for name in JOINT_ORDER:
            joint_id = int(self.model.joint(name).id)
            body_id = int(self.model.jnt_bodyid[joint_id])
            bodies = []
            for candidate in range(self.model.nbody):
                parent = candidate
                while parent > 0 and parent != body_id:
                    parent = int(self.model.body_parentid[parent])
                if parent == body_id:
                    bodies.append(candidate)
            self.subtrees.append((joint_id, np.asarray(bodies, dtype=np.int64)))
        self.tip_sites = [int(self.model.site(f"{side}_tip").id) for side in ("left", "right")]
        self.reset(HOME)

    def reset(self, q: np.ndarray) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qadr] = q
        mujoco.mj_forward(self.model, self.data)

    def q(self) -> np.ndarray:
        return np.array(self.data.qpos[self.qadr], dtype=np.float64)

    def dq(self) -> np.ndarray:
        return np.array(self.data.qvel[self.dadr], dtype=np.float64)

    def tcp(self) -> np.ndarray:
        return np.concatenate(
            [np.asarray(self.data.site_xpos[site], dtype=np.float64) for site in self.tip_sites]
        )

    def _gravity(self) -> np.ndarray:
        masses = self.model.body_mass
        out = np.zeros(len(JOINT_ORDER))
        for index, (joint_id, bodies) in enumerate(self.subtrees):
            anchor = np.asarray(self.data.xanchor[joint_id], dtype=np.float64)
            axis = np.asarray(self.data.xaxis[joint_id], dtype=np.float64)
            total = 0.0
            for body in bodies:
                offset = np.asarray(self.data.xipos[body], dtype=np.float64) - anchor
                weight = float(masses[body]) * 9.81
                total += (offset[1] * weight) * axis[0] + (-offset[0] * weight) * axis[1]
            out[index] = total * GRAVITY_SCALE[index]
        return out

    def step(self, issued: np.ndarray) -> None:
        for _ in range(SUBSTEPS):
            q = self.q()
            dq = self.dq()
            ctrl = np.clip(KP * (issued - q) - KD * dq + self._gravity(), -EFFORT, EFFORT)
            self.data.ctrl[:] = ctrl
            mujoco.mj_step(self.model, self.data)


class _Rollout:
    def __init__(self) -> None:
        self.actor_gap = 0.0
        self.trajectory_gap = 0.0
        self.onnx_track: list[float] = []
        self.reference_track: list[float] = []
        self.goal_travel = 0.0
        self.final_goal = np.zeros(6)


def _manifest() -> PolicyManifest:
    assert MODEL_DIR is not None
    return PolicyManifest.load(os.path.join(MODEL_DIR, "metal_arm_policy.manifest.json"))


def _reference_actor() -> Any:
    """Rebuild the checkpoint actor with torch, exactly as the exporter does."""
    torch = pytest.importorskip("torch")
    assert MODEL_DIR is not None
    checkpoint_path = os.path.join(MODEL_DIR, CHECKPOINT_FILE)
    assert os.path.exists(checkpoint_path), checkpoint_path
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    from dimos.robot.manipulators.dual_makermods_metal.policy.export_onnx import (
        _load_reference_actor,
    )

    actor, _state = _load_reference_actor(checkpoint)
    return actor


def _goals() -> np.ndarray:
    assert EVIDENCE_DIR is not None
    rollouts = np.load(os.path.join(EVIDENCE_DIR, "rollouts.npz"))
    return np.asarray(rollouts["target"][:, CASE], dtype=np.float64)


def test_plant_matches_the_station_forward_kinematics():
    """The MuJoCo plant and the runtime FK must describe the same TCP sites."""
    assert SIM_DIR is not None
    manifest = _manifest()
    kinematics = MetalStationKinematics(manifest.station)
    plant = _Plant(SIM_DIR)
    plant_tcp = plant.tcp()
    station_tcp = np.asarray(kinematics.pose24(HOME)[:6], dtype=np.float64)
    assert float(np.abs(plant_tcp - station_tcp).max()) < 1e-6


def test_checkpoint_actor_and_onnx_runtime_follow_the_same_causal_goal():
    """Closed loop on the trained plant: actor parity plus actual goal arrival."""
    assert SIM_DIR is not None and MODEL_DIR is not None
    torch = pytest.importorskip("torch")
    manifest = _manifest()
    checkpoint_path = os.path.join(MODEL_DIR, CHECKPOINT_FILE)
    assert sha256_file(checkpoint_path) == manifest.checkpoint_sha256
    kinematics = MetalStationKinematics(manifest.station)
    actor = _reference_actor()
    goals = _goals()

    onnx_plant = _Plant(SIM_DIR)
    reference_plant = _Plant(SIM_DIR)
    onnx_runtime = MetalArmPolicyRuntime(manifest, MODEL_DIR, kinematics=kinematics)
    reference_runtime = MetalArmPolicyRuntime(manifest, MODEL_DIR, kinematics=kinematics)
    onnx_runtime.reset(onnx_plant.q())
    reference_runtime.reset(reference_plant.q())

    result = _Rollout()
    for tick in range(POLICY_TICKS):
        goal = goals[tick]
        if not np.isfinite(goal).all() or goal.shape != (24,):
            raise AssertionError(f"invalid goal at tick {tick}")
        onnx_runtime.observe(onnx_plant.q(), onnx_plant.dq(), goal)
        observation = onnx_runtime.observation()
        onnx_action = onnx_runtime.infer(observation)
        with torch.inference_mode():
            same_state_reference = (
                actor(torch.from_numpy(np.ascontiguousarray(observation, dtype=np.float32)))
                .numpy()
                .reshape(-1)
            )
        result.actor_gap = max(
            result.actor_gap, float(np.abs(onnx_action - same_state_reference).max())
        )

        reference_runtime.observe(reference_plant.q(), reference_plant.dq(), goal)
        with torch.inference_mode():
            reference_action = (
                actor(
                    torch.from_numpy(
                        np.ascontiguousarray(reference_runtime.observation(), dtype=np.float32)
                    )
                )
                .numpy()
                .reshape(-1)
            )
        onnx_plant.step(onnx_runtime.decode_action(onnx_action))
        reference_plant.step(reference_runtime.decode_action(reference_action))

        result.trajectory_gap = max(
            result.trajectory_gap, float(np.abs(onnx_plant.q() - reference_plant.q()).max())
        )
        result.onnx_track.append(float(np.linalg.norm(goal[:6] - onnx_plant.tcp())))
        result.reference_track.append(float(np.linalg.norm(goal[:6] - reference_plant.tcp())))
        result.goal_travel = float(np.abs(goals[tick, :6] - goals[0, :6]).max())
        result.final_goal = np.asarray(goals[tick, :6], dtype=np.float64)

    # The actor export is bit-comparable on every closed-loop observation.
    assert result.actor_gap < 1e-4, f"actor parity broke: {result.actor_gap}"
    # Identical plants diverge only through that float noise, never structurally.
    assert result.trajectory_gap < 2e-2, f"closed loop diverged: {result.trajectory_gap}"
    # The goal actually moves across the episode and the plant arrives at it.
    assert result.goal_travel > 0.05, f"goal did not travel: {result.goal_travel}"
    onnx_tail = np.asarray(result.onnx_track[-10:])
    reference_tail = np.asarray(result.reference_track[-10:])
    assert float(onnx_tail.mean()) < 1e-2, f"ONNX runtime did not track: {onnx_tail.mean()}"
    assert float(reference_tail.mean()) < 1e-2, (
        f"reference actor did not track: {reference_tail.mean()}"
    )
    assert float(onnx_tail.max()) < 2e-2, f"ONNX tracking spike: {onnx_tail.max()}"
    print(
        f"closed loop: actor_gap={result.actor_gap:.2e} "
        f"trajectory_gap={result.trajectory_gap:.2e} "
        f"goal_travel={result.goal_travel:.3f} "
        f"track_mean={float(onnx_tail.mean()):.4f} track_max={float(onnx_tail.max()):.4f}"
    )


def test_observation_consumes_only_the_current_goal():
    """The runtime's goal slots are the current goal, with no lookahead."""
    assert MODEL_DIR is not None
    manifest = _manifest()
    kinematics = MetalStationKinematics(manifest.station)
    runtime = MetalArmPolicyRuntime(manifest, MODEL_DIR, kinematics=kinematics)
    runtime.reset(HOME)
    goals = _goals()
    current = np.asarray(kinematics.pose24(HOME), dtype=np.float64)
    zero = np.zeros(len(JOINT_ORDER))
    # The recorded goal travels for the first two seconds, then holds.
    early = runtime.build_frame(HOME, zero, goals[0])
    moving = runtime.build_frame(HOME, zero, goals[10])
    held = runtime.build_frame(HOME, zero, goals[POLICY_TICKS - 2])
    assert np.allclose(early[-24:], goals[0] - current, atol=1e-5)
    assert np.allclose(moving[-24:], goals[10] - current, atol=1e-5)
    assert np.allclose(held[-24:], goals[POLICY_TICKS - 2] - current, atol=1e-5)
    assert not np.allclose(moving[-24:], early[-24:])
    assert not np.allclose(held[-24:], early[-24:])
