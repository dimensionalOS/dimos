# Copyright 2025-2026 Dimensional Inc.
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

"""IK-verified accuracy report for a capability map.

Ground truth must match the map's heading-free semantics. The clean
trick (design.md layer 3): canonicalize each evaluation pose to approach
azimuth ψ = 0 by rotating it about the pelvis vertical axis, and test
*that* representative with the pelvis fixed — map and oracle then
quotient heading identically and no pelvis-yaw search is needed.

Oracle: mink solve-to-convergence (QP, joint limits native) with random
restarts; a candidate counts as reachable when the converged pose is
within tolerance *and* the configuration passes the same self-collision
check construction used. The wrist violation (gamma collapsed in the 4D
marginal vs explicit in 5D) shows up as the FPR gap between the two maps.

CLI::

    python -m dimos.manipulation.reachability.evaluate \\
        --map data/reachability/g1_left_capability.npz --poses 2000
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import time

import numpy as np

from dimos.manipulation.planning.factory import create_world
from dimos.manipulation.planning.kinematics.config import MinkKinematicsConfig
from dimos.manipulation.planning.kinematics.mink_ik import MinkIK
from dimos.manipulation.reachability.capability_map import (
    CapabilityMap,
    MapParams,
    canonical_values,
)
from dimos.manipulation.reachability.construct import (
    ConstructionSpec,
    _robot_config_from_spec,
    arm_spec,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Paper acceptance threshold: combined translational/rotational distance
# with 1 mm ≡ 1°, threshold 25.
_ACCEPT_COMBINED = 25.0


@dataclass
class EvalReport:
    n_poses: int
    gt_reachable: int
    metrics_5d: dict[str, float]
    metrics_4d: dict[str, float]
    fpr_by_theta_5d: list[float]
    fpr_by_theta_4d: list[float]
    oracle_restarts: int
    elapsed_s: float

    def to_json(self) -> str:
        return json.dumps(self.__dict__, indent=2)


def _sample_eval_poses(
    params: MapParams, n: int, rng: np.random.Generator
) -> tuple[np.ndarray, np.ndarray]:
    """Uniform positions in the workspace cylinder x uniform orientations,
    canonicalized to ψ = 0 (rotate about pelvis z so the approach azimuth
    vanishes)."""
    from scipy.spatial.transform import Rotation

    radius = params.r_xy * np.sqrt(rng.uniform(0.0, 1.0, n))
    angle = rng.uniform(-np.pi, np.pi, n)
    positions = np.stack(
        [
            radius * np.cos(angle),
            radius * np.sin(angle),
            rng.uniform(params.z_min, params.z_max, n),
        ],
        axis=1,
    )
    rotations = Rotation.random(n, random_state=rng).as_matrix()

    # Canonicalize: rotate pose i about world z by -ψ_i.
    r_z = rotations[:, :, 2]
    psi = np.arctan2(r_z[:, 1], r_z[:, 0])
    c, s = np.cos(-psi), np.sin(-psi)
    rot_z = np.zeros((n, 3, 3))
    rot_z[:, 0, 0] = c
    rot_z[:, 0, 1] = -s
    rot_z[:, 1, 0] = s
    rot_z[:, 1, 1] = c
    rot_z[:, 2, 2] = 1.0
    positions = np.einsum("nij,nj->ni", rot_z, positions)
    rotations = np.einsum("nij,njk->nik", rot_z, rotations)
    return positions, rotations


class _MinkOracle:
    """Collision-checked solve-to-convergence IK via the planning Mink backend."""

    def __init__(self, spec: ConstructionSpec, restarts: int, seed: int) -> None:
        self._world = create_world(backend=spec.world_backend)
        self._robot_id = self._world.add_robot(_robot_config_from_spec(spec))
        self._world.finalize()
        self._ik = MinkIK(
            MinkKinematicsConfig(
                solver="daqp",
                max_iterations=300,
                dt=0.05,
                position_cost=1.0,
                orientation_cost=1.0,
                lm_damping=1.0,
            )
        )
        self._rng = np.random.default_rng(seed)
        self._restarts = restarts
        self._joint_names = list(spec.joint_names)
        self._lower, self._upper = self._world.get_joint_limits(self._robot_id)
        self._grasp_offset = np.asarray(spec.grasp_offset, dtype=np.float64)

    def reachable(self, position: np.ndarray, rotation: np.ndarray) -> bool:
        target = _target_body_pose(position, rotation, self._grasp_offset)

        for _ in range(self._restarts):
            seed = JointState(
                name=self._joint_names,
                position=self._rng.uniform(self._lower, self._upper).tolist(),
            )
            result = self._ik.solve(
                self._world,
                self._robot_id,
                target,
                seed=seed,
                position_tolerance=_ACCEPT_COMBINED / 1000.0,
                orientation_tolerance=np.deg2rad(_ACCEPT_COMBINED),
                check_collision=True,
                max_attempts=1,
            )
            if result.is_success() and _accept_combined(
                result.position_error, result.orientation_error
            ):
                return True
        return False


def _target_body_pose(
    grasp_position: np.ndarray,
    rotation: np.ndarray,
    grasp_offset: np.ndarray,
) -> PoseStamped:
    """Convert a grasp-center target into the EE body-origin target Mink solves."""
    body_position = grasp_position - rotation @ grasp_offset
    return PoseStamped(
        frame_id="world",
        position=Vector3(*body_position.tolist()),
        orientation=Quaternion.from_rotation_matrix(rotation),
    )


def _accept_combined(position_error_m: float, orientation_error_rad: float) -> bool:
    pos_err_mm = position_error_m * 1000.0
    ori_err_deg = float(np.degrees(orientation_error_rad))
    return pos_err_mm + ori_err_deg <= _ACCEPT_COMBINED


def _confusion(gt: np.ndarray, pred: np.ndarray) -> dict[str, float]:
    tp = int(np.sum(gt & pred))
    tn = int(np.sum(~gt & ~pred))
    fp = int(np.sum(~gt & pred))
    fn = int(np.sum(gt & ~pred))
    n = len(gt)
    return {
        "accuracy": (tp + tn) / n,
        "tpr": tp / max(tp + fn, 1),
        "fpr": fp / max(fp + tn, 1),
        "precision": tp / max(tp + fp, 1),
        "tp": tp,
        "tn": tn,
        "fp": fp,
        "fn": fn,
    }


def evaluate(
    cap: CapabilityMap,
    spec: ConstructionSpec,
    n_poses: int = 2000,
    restarts: int = 20,
    seed: int = 1,
) -> EvalReport:
    t0 = time.time()
    rng = np.random.default_rng(seed)
    positions, rotations = _sample_eval_poses(cap.params, n_poses, rng)

    oracle = _MinkOracle(spec, restarts=restarts, seed=seed)
    gt = np.array(
        [oracle.reachable(positions[i], rotations[i]) for i in range(n_poses)], dtype=bool
    )

    pred_5d = cap.scores(positions, rotations) > 0
    pred_4d = cap.scores_4d(positions, rotations) > 0

    # FPR per θ bin — the wrist violation shows up as structured FPR.
    theta = canonical_values(positions, rotations)[1]
    t_bins = np.minimum((theta / np.pi * cap.params.n_theta).astype(int), cap.params.n_theta - 1)
    fpr_theta_5d, fpr_theta_4d = [], []
    for t in range(cap.params.n_theta):
        mask = (t_bins == t) & ~gt
        if mask.sum() == 0:
            fpr_theta_5d.append(0.0)
            fpr_theta_4d.append(0.0)
            continue
        fpr_theta_5d.append(float(pred_5d[mask].mean()))
        fpr_theta_4d.append(float(pred_4d[mask].mean()))

    return EvalReport(
        n_poses=n_poses,
        gt_reachable=int(gt.sum()),
        metrics_5d=_confusion(gt, pred_5d),
        metrics_4d=_confusion(gt, pred_4d),
        fpr_by_theta_5d=fpr_theta_5d,
        fpr_by_theta_4d=fpr_theta_4d,
        oracle_restarts=restarts,
        elapsed_s=time.time() - t0,
    )


def cli_main() -> None:
    parser = argparse.ArgumentParser(description="IK-verified capability map accuracy.")
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--poses", type=int, default=2000)
    parser.add_argument("--restarts", type=int, default=20)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    cap = CapabilityMap.load(args.map)
    spec = arm_spec(cap.robot, cap.params)
    report = evaluate(cap, spec, n_poses=args.poses, restarts=args.restarts, seed=args.seed)
    print(report.to_json())
    if args.out:
        args.out.write_text(report.to_json() + "\n")


if __name__ == "__main__":
    cli_main()


__all__ = ["EvalReport", "evaluate"]
