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
        --robot g1-left --samples 5000000 --workers 8 \\
        --out data/reachability/g1-left_capability.npz

``--robot`` is any key from ``robots.list_robots()`` (g1-left/right,
xarm7, piper, a750, openarm). MJCF and URDF models are both supported.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field, replace
from multiprocessing import get_context
from pathlib import Path
import time
from typing import Any

import numpy as np

from dimos.manipulation.planning.factory import create_world
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.reachability.capability_map import (
    CapabilityMap,
    MapParams,
    model_id_for,
)
from dimos.manipulation.reachability.robots import arm_model, compile_model, list_robots
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_CHUNK = 50_000
_UINT8_MAX = np.iinfo(np.uint8).max


@dataclass(frozen=True)
class ConstructionSpec:
    """Everything a (worker) process needs to sample one arm."""

    model_path: str
    model_meshdir: str | None
    joint_names: list[str]  # model joint names, e.g. left_shoulder_pitch_joint
    ee_body: str
    base_link: str
    grasp_offset: tuple[float, float, float]
    params: MapParams
    robot: str  # registry key, e.g. "g1-left", "xarm7"
    world_backend: str = "mujoco"
    is_urdf: bool = False
    package_roots: dict[str, str] = field(default_factory=dict)
    collision_exclude: tuple[tuple[str, str], ...] = ()


def arm_spec(
    robot: str = "g1-left",
    params: MapParams | None = None,
    *,
    world_backend: str = "mujoco",
) -> ConstructionSpec:
    """Construction spec for any registered arm (see ``robots.py``).

    With no ``params`` the grid is taken from the registry entry, or
    auto-sized from the arm's sampled workspace when the entry leaves it
    open (the default for fixed-base arms, whose reach we don't hardcode).
    """
    am = arm_model(robot)
    spec = ConstructionSpec(
        model_path=str(am.model_path),
        model_meshdir=str(am.model_meshdir) if am.model_meshdir else None,
        joint_names=list(am.joint_names),
        ee_body=am.ee_body,
        base_link=am.base_link,
        grasp_offset=am.grasp_offset,
        params=params or am.params or MapParams.at_base_height(am.base_height),
        robot=robot,
        world_backend=world_backend,
        is_urdf=am.is_urdf,
        package_roots=dict(am.package_roots),
        collision_exclude=tuple(am.collision_exclude),
    )
    if params is None and am.params is None:
        spec = replace(spec, params=_autosize(spec))
    return spec


def _autosize(spec: ConstructionSpec, probe: int = 30_000, margin: float = 0.1) -> MapParams:
    """Size the grid from the arm's collision-free workspace bounding box.

    Cheap FK probe (no grid allocated): bound the reachable TCP positions,
    then snap r_xy / z to the cell. Keeps the map tight around each arm
    instead of assuming the G1's pelvis-rooted dimensions.
    """
    sampler = _WorldSpecArmSampler(spec)
    rng = np.random.default_rng(0)
    chunks: list[np.ndarray] = []
    done = 0
    while done < probe:
        n = min(_CHUNK, probe - done)
        positions, _, _ = sampler.sample_chunk(n, rng)
        if len(positions):
            chunks.append(positions)
        done += n
    pts = np.concatenate(chunks) if chunks else np.zeros((1, 3))
    cell = spec.params.cell
    r_xy = float(np.ceil((np.abs(pts[:, :2]).max() + margin) / cell) * cell)
    z_min = float(np.floor((pts[:, 2].min() - margin) / cell) * cell)
    z_max = float(np.ceil((pts[:, 2].max() + margin) / cell) * cell)
    # Don't map reach below the mounting plane: a bench/floor-mounted arm
    # can't pass its tool under its own base. The base-link pose supplies that
    # plane in the map frame, so clamp the floor of the grid there.
    z_min = max(z_min, spec.params.base_link_pose.position.z)
    return replace(spec.params, r_xy=r_xy, z_min=z_min, z_max=z_max)


class _WorldSpecArmSampler:
    """FK sampler that uses the planning ``WorldSpec`` protocol only."""

    def __init__(self, spec: ConstructionSpec) -> None:
        self.spec = spec
        self.world = create_world(backend=spec.world_backend)
        self.robot_id = self.world.add_robot(_robot_config_from_spec(spec))
        self.world.finalize()
        self.lower, self.upper = self.world.get_joint_limits(self.robot_id)
        self.joint_names = list(spec.joint_names)
        self.grasp_offset = np.asarray(spec.grasp_offset, dtype=np.float64)

    def sample_chunk(self, n: int, rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray, int]:
        """FK-sample n configs through ``WorldSpec``."""
        qs = rng.uniform(self.lower, self.upper, size=(n, len(self.joint_names)))
        positions = np.empty((n, 3))
        rotations = np.empty((n, 3, 3))
        kept = 0
        rejected = 0
        with self.world.scratch_context() as ctx:
            for q in qs:
                self.world.set_joint_state(
                    ctx,
                    self.robot_id,
                    JointState(name=self.joint_names, position=q.tolist()),
                )
                if not self.world.is_collision_free(ctx, self.robot_id):
                    rejected += 1
                    continue
                transform = self.world.get_link_pose(ctx, self.robot_id, self.spec.ee_body)
                rotation = transform[:3, :3]
                positions[kept] = transform[:3, 3] + rotation @ self.grasp_offset
                rotations[kept] = rotation
                kept += 1
        return positions[:kept], rotations[:kept], rejected


class _DirectMujocoArmSampler:
    """One compiled model + the index tables needed for fast FK sampling."""

    def __init__(self, spec: ConstructionSpec) -> None:
        import mujoco

        self._mujoco = mujoco
        self.model = compile_model(
            spec.model_path,
            is_urdf=spec.is_urdf,
            model_meshdir=spec.model_meshdir,
            package_roots=spec.package_roots,
        )
        self.data = mujoco.MjData(self.model)
        self.spec = spec

        # Pin the floating base level at the map's pelvis height: the model
        # world becomes the gravity-aligned ground-level pelvis frame.
        self._q_base = self.model.qpos0.copy()
        for jid in range(self.model.njnt):
            if self.model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_FREE:
                adr = self.model.jnt_qposadr[jid]
                pose = spec.params.base_link_pose
                self._q_base[adr : adr + 3] = (
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                )
                self._q_base[adr + 3 : adr + 7] = (
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                )

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

        # Body→body exclusion matrix for known structural mesh overlaps
        # (links seated in mounts, fat simplified collision meshes): their
        # contacts are constant and not real self-collisions.
        self.geom_bodyid = np.asarray(self.model.geom_bodyid, dtype=int)
        self._excluded = np.zeros((self.model.nbody, self.model.nbody), dtype=bool)
        for name_a, name_b in spec.collision_exclude:
            ia = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name_a)
            ib = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name_b)
            if ia >= 0 and ib >= 0:
                self._excluded[ia, ib] = self._excluded[ib, ia] = True

    def is_self_collision(self, d: Any) -> bool:
        """Any penetrating contact that involves the arm's moving subtree and
        isn't an excluded structural overlap (``d`` is a ``mujoco.MjData``
        with up-to-date contacts). Shared by sampling and IK so both judge
        collisions identically."""
        if not d.ncon:
            return False
        geom = d.contact.geom[: d.ncon]
        dist = d.contact.dist[: d.ncon]
        involved = self.check_geom_mask[geom[:, 0]] | self.check_geom_mask[geom[:, 1]]
        excluded = self._excluded[self.geom_bodyid[geom[:, 0]], self.geom_bodyid[geom[:, 1]]]
        return bool(np.any(involved & (dist < 0.0) & ~excluded))

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
            if self.is_self_collision(data):
                rejected += 1
                continue
            xmat = data.xmat[self.ee_body_id].reshape(3, 3)
            positions[kept] = data.xpos[self.ee_body_id] + xmat @ self.grasp_offset
            rotations[kept] = xmat
            kept += 1
        return positions[:kept], rotations[:kept], rejected


def _robot_config_from_spec(spec: ConstructionSpec) -> RobotModelConfig:
    """Build a planning robot config from a reachability construction spec."""
    return RobotModelConfig(
        name=spec.robot,
        model_path=_model_path_for_spec(spec),
        base_pose=spec.params.base_link_pose,
        joint_names=list(spec.joint_names),
        end_effector_link=spec.ee_body,
        base_link=spec.base_link,
        package_paths={pkg: Path(root) for pkg, root in spec.package_roots.items()},
        collision_exclusion_pairs=list(spec.collision_exclude),
        auto_convert_meshes=spec.world_backend == "drake",
    )


def _model_path_for_spec(spec: ConstructionSpec) -> Path:
    model_path = Path(spec.model_path)
    if spec.world_backend == "drake":
        arm = arm_model(spec.robot)
        if arm.viewer_urdf is not None:
            return Path(str(arm.viewer_urdf))
    return model_path


# Backward-compatible alias for tests/viewer internals that need the original
# MuJoCo model handle. New construction code uses _WorldSpecArmSampler.
_ArmSampler = _DirectMujocoArmSampler


def _worker(args: tuple[ConstructionSpec, int, int]) -> tuple[CapabilityMap, int]:
    spec, n_samples, seed = args
    sampler = _WorldSpecArmSampler(spec)
    # Per-worker uint8 saturation is exact under the merge's final uint8 max clip:
    # a cell only loses information when its per-worker count would exceed
    # uint8 max, and the merged value is clipped there anyway.
    cap = CapabilityMap(spec.params, robot=spec.robot)
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
        robot=spec.robot,
        model_id=model_id_for(_model_path_for_spec(spec)),
        counts=np.minimum(total_counts, _UINT8_MAX).astype(np.uint8),
        heading_hint=total_hint,
        body_counts=np.minimum(total_body, _UINT8_MAX).astype(np.uint8),
        body_theta_mask=total_theta_mask,
    )
    elapsed = time.time() - t0
    n_total = per_worker * max(workers, 1)
    stats = cap.summary()
    logger.info(
        f"constructed {spec.robot} map: {n_total} samples in {elapsed:.0f}s "
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
    sampler = _WorldSpecArmSampler(spec)
    cap = CapabilityMap(spec.params, robot=spec.robot)
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
    parser = argparse.ArgumentParser(description="Build an arm capability map.")
    parser.add_argument("--robot", choices=list_robots(), default="g1-left")
    parser.add_argument("--samples", type=int, default=5_000_000)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--world-backend", choices=("mujoco", "drake"), default="mujoco")
    parser.add_argument("--cell", type=float, default=0.05)
    parser.add_argument("--n-theta", type=int, default=36)
    parser.add_argument("--n-inplane", type=int, default=12)
    parser.add_argument(
        "--out", type=Path, default=None, help="default: data/reachability/<robot>_capability.npz"
    )
    args = parser.parse_args()

    # Grid resolution comes from the CLI; extent (r_xy / z) is taken from the
    # registry or auto-sized per arm, so override only resolution here and let
    # arm_spec fill the rest.
    base = arm_spec(args.robot, world_backend=args.world_backend)
    params = replace(base.params, cell=args.cell, n_theta=args.n_theta, n_inplane=args.n_inplane)
    spec = arm_spec(args.robot, params, world_backend=args.world_backend)
    cap = construct(spec, n_samples=args.samples, workers=args.workers, seed=args.seed)
    out = args.out or Path("data/reachability") / f"{args.robot}_capability.npz"
    cap.save(out)
    print(out)


if __name__ == "__main__":
    cli_main()


__all__ = ["ConstructionSpec", "arm_spec", "construct", "saturation_curve"]
