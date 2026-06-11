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

"""Capability-map construction: FK-sample the arm on the same MJCF the sim uses.

Sampling protocol (design.md layer 3): pelvis pinned level at the WBC
height, waist at its model default (the WBC owns it — conservative), the
other arm at servo default; draw uniform configurations of the 7 arm
joints, reject self-colliding ones (any contact involving the arm's
moving subtree), record the TCP pose (grasp center, from the catalog
config). Saturation is tracked as new-cells-per-chunk, the paper's
stopping criterion.

CLI::

    python -m dimos.manipulation.reachability.construct \\
        --side left --samples 5000000 --workers 8 \\
        --out data/reachability/g1_left_capability.npz
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from multiprocessing import get_context
from pathlib import Path
import time

import numpy as np

from dimos.manipulation.reachability.capability_map import (
    CapabilityMap,
    MapParams,
    model_id_for,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_CHUNK = 50_000


@dataclass(frozen=True)
class ConstructionSpec:
    """Everything a (worker) process needs to sample one arm."""

    model_path: str
    model_meshdir: str | None
    joint_names: list[str]  # model joint names, e.g. left_shoulder_pitch_joint
    ee_body: str
    grasp_offset: tuple[float, float, float]
    params: MapParams
    side: str


def g1_spec(side: str = "left", params: MapParams | None = None) -> ConstructionSpec:
    """Construction spec from the G1 catalog's mujoco-backend config."""
    from dimos.robot.catalog.g1 import g1_left_arm, g1_right_arm

    entry = (g1_left_arm if side == "left" else g1_right_arm)(backend="mujoco")
    cfg = entry.robot_model_config
    return ConstructionSpec(
        model_path=str(cfg.model_path),
        model_meshdir=str(cfg.model_meshdir) if cfg.model_meshdir else None,
        joint_names=list(cfg.joint_names),
        ee_body=cfg.end_effector_link,
        grasp_offset=tuple(cfg.grasp_offset_xyz),
        params=params or MapParams(),
        side=side,
    )


class _ArmSampler:
    """One compiled model + the index tables needed for fast FK sampling."""

    def __init__(self, spec: ConstructionSpec) -> None:
        import mujoco

        self._mujoco = mujoco
        mjspec = mujoco.MjSpec.from_file(spec.model_path)
        if spec.model_meshdir:
            mjspec.meshdir = spec.model_meshdir
        else:
            mjspec.meshdir = str((Path(spec.model_path).parent / (mjspec.meshdir or "")).resolve())
        self.model = mjspec.compile()
        self.data = mujoco.MjData(self.model)
        self.spec = spec

        # Pin the floating base level at the map's pelvis height: the model
        # world becomes the gravity-aligned ground-level pelvis frame.
        self._q_base = self.model.qpos0.copy()
        for jid in range(self.model.njnt):
            if self.model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_FREE:
                adr = self.model.jnt_qposadr[jid]
                self._q_base[adr : adr + 3] = (0.0, 0.0, spec.params.pelvis_height)
                self._q_base[adr + 3 : adr + 7] = (1.0, 0.0, 0.0, 0.0)

        joint_ids = []
        for name in spec.joint_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"joint '{name}' not in model")
            joint_ids.append(jid)
        self.qpos_adr = np.array([self.model.jnt_qposadr[j] for j in joint_ids], dtype=int)
        self.lower = np.array([self.model.jnt_range[j][0] for j in joint_ids])
        self.upper = np.array([self.model.jnt_range[j][1] for j in joint_ids])

        self.ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, spec.ee_body)
        if self.ee_body_id < 0:
            raise ValueError(f"ee body '{spec.ee_body}' not in model")
        self.grasp_offset = np.asarray(spec.grasp_offset, dtype=np.float64)

        # Moving-subtree geom mask (same scoping as MujocoWorld): contacts
        # involving these geoms are self-collisions to reject.
        chain_bodies = {int(self.model.jnt_bodyid[j]) for j in joint_ids}
        mask = np.zeros(self.model.ngeom, dtype=bool)
        for body_id in range(self.model.nbody):
            b = body_id
            while b != 0:
                if b in chain_bodies:
                    adr, num = self.model.body_geomadr[body_id], self.model.body_geomnum[body_id]
                    mask[adr : adr + num] = True
                    break
                b = int(self.model.body_parentid[b])
        self.check_geom_mask = mask

    def sample_chunk(self, n: int, rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray, int]:
        """FK-sample n configs; returns (positions, rotations, n_rejected)."""
        mujoco = self._mujoco
        qs = rng.uniform(self.lower, self.upper, size=(n, len(self.qpos_adr)))
        positions = np.empty((n, 3))
        rotations = np.empty((n, 3, 3))
        kept = 0
        rejected = 0
        data, model = self.data, self.model
        for q in qs:
            data.qpos[:] = self._q_base
            data.qpos[self.qpos_adr] = q
            mujoco.mj_kinematics(model, data)
            mujoco.mj_collision(model, data)
            if data.ncon:
                geom = data.contact.geom[: data.ncon]
                dist = data.contact.dist[: data.ncon]
                involved = self.check_geom_mask[geom[:, 0]] | self.check_geom_mask[geom[:, 1]]
                if np.any(involved & (dist < 0.0)):
                    rejected += 1
                    continue
            xmat = data.xmat[self.ee_body_id].reshape(3, 3)
            positions[kept] = data.xpos[self.ee_body_id] + xmat @ self.grasp_offset
            rotations[kept] = xmat
            kept += 1
        return positions[:kept], rotations[:kept], rejected


def _worker(args: tuple[ConstructionSpec, int, int]) -> tuple[CapabilityMap, int]:
    spec, n_samples, seed = args
    sampler = _ArmSampler(spec)
    # Per-worker uint8 saturation is exact under the merge's final clip-255:
    # a cell only loses information when its per-worker count would exceed
    # 255, and the merged value is clipped there anyway.
    cap = CapabilityMap(spec.params, side=spec.side)
    rng = np.random.default_rng(seed)
    rejected = 0
    done = 0
    while done < n_samples:
        n = min(_CHUNK, n_samples - done)
        positions, rotations, rej = sampler.sample_chunk(n, rng)
        rejected += rej
        done += n
        if len(positions):
            cap.record_batch(positions, rotations)
    return cap, rejected


def construct(
    spec: ConstructionSpec,
    n_samples: int = 5_000_000,
    workers: int = 1,
    seed: int = 0,
) -> CapabilityMap:
    """Build a capability map by parallel FK sampling."""
    t0 = time.time()
    per_worker = int(np.ceil(n_samples / max(workers, 1)))
    jobs = [(spec, per_worker, seed + i) for i in range(max(workers, 1))]

    if workers <= 1:
        results = [_worker(jobs[0])]
    else:
        with get_context("spawn").Pool(workers) as pool:
            results = pool.map(_worker, jobs)

    first = results[0][0]
    total_counts = np.zeros(first.counts.shape, dtype=np.uint32)
    total_body = np.zeros(first.body_counts.shape, dtype=np.uint32)
    total_hint = np.zeros(first.heading_hint.shape, dtype=np.uint8)
    total_theta_mask = np.zeros(first.body_theta_mask.shape, dtype=np.uint64)
    rejected = 0
    for worker_cap, rej in results:
        total_counts += worker_cap.counts
        total_body += worker_cap.body_counts
        total_hint |= worker_cap.heading_hint
        total_theta_mask |= worker_cap.body_theta_mask
        rejected += rej

    cap = CapabilityMap(
        spec.params,
        side=spec.side,
        model_id=model_id_for(spec.model_path),
        counts=np.minimum(total_counts, 255).astype(np.uint8),
        heading_hint=total_hint,
        body_counts=np.minimum(total_body, 255).astype(np.uint8),
        body_theta_mask=total_theta_mask,
    )
    elapsed = time.time() - t0
    n_total = per_worker * max(workers, 1)
    stats = cap.summary()
    logger.info(
        f"constructed {spec.side} map: {n_total} samples in {elapsed:.0f}s "
        f"({n_total / max(elapsed, 1e-9):.0f}/s), {rejected} self-colliding "
        f"({rejected / n_total:.1%}), {stats['marked']} cells marked "
        f"({stats['fill_ratio']:.2%} of grid)"
    )
    return cap


def saturation_curve(
    spec: ConstructionSpec, n_samples: int, checkpoints: int = 10, seed: int = 0
) -> list[tuple[int, int]]:
    """(samples, cumulative marked cells) at regular checkpoints — the
    paper's new-cells-per-chunk stopping diagnostic."""
    sampler = _ArmSampler(spec)
    cap = CapabilityMap(spec.params, side=spec.side)
    rng = np.random.default_rng(seed)
    curve: list[tuple[int, int]] = []
    per = n_samples // checkpoints
    done = 0
    for _ in range(checkpoints):
        remaining = per
        while remaining > 0:
            n = min(_CHUNK, remaining)
            positions, rotations, _ = sampler.sample_chunk(n, rng)
            if len(positions):
                cap.record_batch(positions, rotations)
            remaining -= n
        done += per
        curve.append((done, cap.n_marked))
    return curve


def cli_main() -> None:
    parser = argparse.ArgumentParser(description="Build a G1 arm capability map.")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--samples", type=int, default=5_000_000)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--cell", type=float, default=0.05)
    parser.add_argument("--n-theta", type=int, default=36)
    parser.add_argument("--n-inplane", type=int, default=12)
    parser.add_argument("--pelvis-height", type=float, default=0.74)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    params = MapParams(
        cell=args.cell,
        n_theta=args.n_theta,
        n_inplane=args.n_inplane,
        pelvis_height=args.pelvis_height,
    )
    spec = g1_spec(args.side, params)
    cap = construct(spec, n_samples=args.samples, workers=args.workers, seed=args.seed)
    out = args.out or Path("data/reachability") / f"g1_{args.side}_capability.npz"
    cap.save(out)
    print(out)


if __name__ == "__main__":
    cli_main()


__all__ = ["ConstructionSpec", "construct", "g1_spec", "saturation_curve"]
