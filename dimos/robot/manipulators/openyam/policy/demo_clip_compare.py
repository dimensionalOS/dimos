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

"""Compare the integrated DimOS policy chain with the original training rollout.

Two checks, both from the frozen artifacts this card carries:

1. **Plant provenance (open loop).** Replay the *recorded* commands
   (``actual_policy.npz`` -> ``sent``) into the exported 6-joint plant from the
   recorded reset ``qpos``/``qvel`` at the trained 480 Hz / 30 Hz ratio and
   compare the resulting joint trajectory with the training rollout's ``q``.
   This tests the plant's dynamics, not just its forward kinematics: the
   training plant had two extra finger hinge joints which the exported plant
   welds shut, so the finger columns are checked to have been parked.
2. **Integrated closed loop.** Run the same sold blueprint against the trained
   plant, driven by the same frozen reference clip from the resolved start
   index and the same initial state, then compare joint and TCP trajectories
   with the training rollout. The clip start index is resolved from the arrays
   (measured reset pose -> closest reference row), not from a note.

The command is ``demo_``-prefixed so pytest never collects it.

    MUJOCO_GL=egl PYTHONPATH=ws/overlay:ws/l2 /path/to/python \
        -m dimos.robot.manipulators.openyam.policy.demo_clip_compare \
        --initial-state frozen/t0_reset.npz --training frozen/actual_policy.npz \
        --clip artifacts/openyam_policy/reference_clip.npz \
        --output-dir ws/evidence/run227/clip_compare
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any

import mujoco
import numpy as np

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.robot.manipulators.openyam.blueprints.policy import (
    ACTOR_EVERY_N_TICKS,
    COORDINATOR_TICK_RATE,
    POLICY_TASK_NAME,
    openyam_policy_sim_clip,
    resolve_manifest_path,
    resolve_plant_path,
)
from dimos.robot.manipulators.openyam.policy.reference import ReferenceClip
from dimos.robot.manipulators.openyam.policy.tcp import (
    MjcfTcpFk,
    pose_error_summary,
    quat_angle_deg,
)
from dimos.simulation.engines.mujoco_shm import ManipShmReader, shm_key_from_path

PLANT_DT = 1.0 / 480.0
POLICY_HZ = 30.0
FINGER_TOLERANCE = 1e-3


def sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_anchor(
    clip: ReferenceClip, q_measured: np.ndarray, tolerance: float
) -> tuple[int, float]:
    """Resolve the policy tick whose reference pose matches the measured reset pose."""
    index = clip.anchor(q_measured[:6], tolerance=tolerance)
    residual = float(np.abs(clip.row(index).q - np.asarray(q_measured[:6], dtype=np.float64)).max())
    return index, residual


def open_loop_provenance(
    plant: Path,
    training: dict[str, np.ndarray],
    qpos0: np.ndarray,
    qvel0: np.ndarray,
    env: int = 0,
) -> dict[str, Any]:
    """Replay the recorded commands into the exported plant and compare to training."""
    model = mujoco.MjModel.from_xml_path(str(plant))
    data = mujoco.MjData(model)
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(model.njnt)]
    if (
        model.nq != 6
        or model.nv != 6
        or tuple(names) != tuple(f"arm_joint_{i}" for i in range(1, 7))
    ):
        raise SystemExit(f"unexpected plant joints: nq={model.nq} nv={model.nv} names={names}")
    substeps = round((1.0 / POLICY_HZ) / model.opt.timestep)
    data.qpos[:] = qpos0[:6]
    data.qvel[:] = qvel0[:6]
    mujoco.mj_forward(model, data)

    sent = np.asarray(training["sent"], dtype=np.float64)[:, env, :]
    reference_q = np.asarray(training["q"], dtype=np.float64)[:, env, :]
    steps = min(sent.shape[0], reference_q.shape[0] - 1)
    replay = np.zeros((steps + 1, 6), dtype=np.float64)
    replay[0] = data.qpos.copy()
    for tick in range(steps):
        data.ctrl[:] = sent[tick]
        for _ in range(substeps):
            mujoco.mj_step(model, data)
        replay[tick + 1] = data.qpos.copy()

    error = replay - reference_q[: steps + 1]
    return {
        "plant_joint_names": names,
        "plant_nq": int(model.nq),
        "plant_timestep_s": float(model.opt.timestep),
        "control_substeps": substeps,
        "ticks_compared": steps,
        "commands_replayed": "actual_policy.npz:sent (recorded training commands)",
        "joint_error_rad": {
            "mean_abs": float(np.abs(error).mean()),
            "max_abs": float(np.abs(error).max()),
            "rmse": float(np.sqrt((error**2).mean())),
            "final_max_abs": float(np.abs(error[-1]).max()),
        },
        "per_joint_max_abs": np.abs(error).max(axis=0).round(6).tolist(),
        "trajectory_tail": replay[-1].round(6).tolist(),
    }


def closed_loop(
    *,
    plant: Path,
    manifest: Path,
    clip_path: Path,
    anchor_index: int,
    qpos0: np.ndarray,
    qvel0: np.ndarray,
    max_ticks: int,
    timeout_s: float,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    blueprint = openyam_policy_sim_clip(
        clip_path=clip_path,
        clip_anchor_index=anchor_index,
        model_dir=manifest.parent,
        manifest_path=manifest,
        headless=True,
        initial_positions=qpos0[:6].tolist(),
        initial_velocities=qvel0[:6].tolist(),
    )
    coordinator = ModuleCoordinator.build(blueprint)
    control = coordinator.get_instance("ControlCoordinator")
    reader = ManipShmReader.wait_for_live(
        shm_key_from_path(str(plant)), attach_timeout_s=60.0, ready_timeout_s=60.0, poll_s=0.1
    )
    if reader is None:
        coordinator.stop()
        raise RuntimeError("the MuJoCo sim never published joint state")
    kinematics = MjcfTcpFk(plant)
    engaged = control.task_invoke(task_name=POLICY_TASK_NAME, method="engage", kwargs={})
    if engaged is not True:
        coordinator.stop()
        raise RuntimeError(f"task refused to engage: {engaged!r}")

    samples: dict[str, list[float]] = {
        key: []
        for key in (
            "t",
            "q0",
            "q1",
            "q2",
            "q3",
            "q4",
            "q5",
            "dq0",
            "dq1",
            "dq2",
            "dq3",
            "dq4",
            "dq5",
            "cmd0",
            "cmd1",
            "cmd2",
            "cmd3",
            "cmd4",
            "cmd5",
            "goal_x",
            "goal_y",
            "goal_z",
            "exec_x",
            "exec_y",
            "exec_z",
            "exec_qw",
            "exec_qx",
            "exec_qy",
            "exec_qz",
            "tick",
            "active",
            "shm_seq",
        )
    }
    fault_log: list[str] = []
    started = time.monotonic()
    was_active = False
    end_reason = "timeout"
    while time.monotonic() - started < timeout_s:
        q = np.asarray(reader.read_positions(6), dtype=np.float64)
        dq = np.asarray(reader.read_velocities(6), dtype=np.float64)
        snapshot = (
            control.task_invoke(task_name=POLICY_TASK_NAME, method="state_snapshot", kwargs={})
            or {}
        )
        executed, orientation = kinematics(q)
        command = snapshot.get("last_command") or [np.nan] * 6
        goal = snapshot.get("goal_position") or [np.nan] * 3
        active = 1.0 if snapshot.get("session") == "active" else 0.0
        values = {
            "t": [time.monotonic() - started],
            **{f"q{i}": [q[i]] for i in range(6)},
            **{f"dq{i}": [dq[i]] for i in range(6)},
            **{f"cmd{i}": [float(command[i])] for i in range(6)},
            "goal_x": [float(goal[0])],
            "goal_y": [float(goal[1])],
            "goal_z": [float(goal[2])],
            "exec_x": [executed[0]],
            "exec_y": [executed[1]],
            "exec_z": [executed[2]],
            "exec_qw": [float(orientation[0])],
            "exec_qx": [float(orientation[1])],
            "exec_qy": [float(orientation[2])],
            "exec_qz": [float(orientation[3])],
            "tick": [float(snapshot.get("policy_ticks", np.nan))],
            "active": [active],
            "shm_seq": [float(reader.position_sequence())],
        }
        for key, value in values.items():
            samples[key].extend(value)
        fault_log.append(str(snapshot.get("fault")))
        was_active = was_active or bool(active)
        if was_active and not active:
            end_reason = f"session closed: {snapshot.get('fault')}"
            break
        if snapshot.get("policy_ticks", 0) >= max_ticks:
            end_reason = "max ticks"
            break
        time.sleep(1.0 / 120.0)
    coordinator.stop()
    arrays = {key: np.asarray(values, dtype=np.float64) for key, values in samples.items()}
    active = arrays["active"] > 0.5
    info = {
        "end_reason": end_reason,
        "engaged": engaged,
        "active_samples": int(active.sum()),
        "actor_ticks": int(np.nanmax(arrays["tick"])) + 1 if arrays["tick"].size else 0,
        "faults": sorted(
            {fault for fault in fault_log if fault not in ("None", "nan", "none", "")}
        ),
        "cadence": {
            "coordinator_hz": COORDINATOR_TICK_RATE,
            "every_n_ticks": ACTOR_EVERY_N_TICKS,
            "expected_actor_hz": COORDINATOR_TICK_RATE / ACTOR_EVERY_N_TICKS,
        },
    }
    if active.any():
        elapsed = float(arrays["t"][active][-1] - arrays["t"][active][0])
        ticks = float(np.nanmax(arrays["tick"][active]) - np.nanmin(arrays["tick"][active]))
        info["actor_cadence_hz_measured"] = (ticks + 1.0) / elapsed if elapsed > 0 else math.inf
        sequence = arrays["shm_seq"]
        info["physics"] = {
            "shm_state_sequence_delta": float(sequence[-1] - sequence[0]),
            "implied_hz": float((sequence[-1] - sequence[0]) / elapsed)
            if elapsed > 0
            else math.inf,
        }
    return arrays, info


def compare(
    arrays: dict[str, np.ndarray],
    training: dict[str, np.ndarray],
    kinematics: Callable[[np.ndarray], tuple[np.ndarray, np.ndarray]],
    env: int,
) -> dict[str, Any]:
    """Compare the integrated rollout with the recorded training rollout.

    The training side stores joint states (``full_q``) and the goal TCP the
    actor was given (``goal`` per tick); its executed TCP is the plant FK of
    ``full_q``, the same FK the integrated run is measured with. Both sides use
    the same reference clip rows, so a per-policy-tick index is the alignment.
    """
    q_track = np.stack([arrays[f"q{i}"] for i in range(6)], axis=1)
    exec_track = np.stack([arrays["exec_x"], arrays["exec_y"], arrays["exec_z"]], axis=1)
    exec_quat = np.stack(
        [arrays["exec_qw"], arrays["exec_qx"], arrays["exec_qy"], arrays["exec_qz"]], axis=1
    )
    goal_track = np.stack([arrays["goal_x"], arrays["goal_y"], arrays["goal_z"]], axis=1)
    active = (arrays["active"] > 0.5) & np.isfinite(arrays["tick"])
    ticks = arrays["tick"].copy()
    first: dict[int, int] = {}
    for index in np.flatnonzero(active):
        first.setdefault(int(ticks[index]), index)
    order = sorted(first)

    train_q = np.asarray(training["q"], dtype=np.float64)[:, env, :]
    train_full = np.asarray(training["full_q"], dtype=np.float64)[:, env, :]
    train_goal = np.asarray(training["goal"], dtype=np.float64)[:, env, :]
    train_exec, train_exec_quat = [], []
    for row in train_full:
        position, quaternion = kinematics(row[:6])
        train_exec.append(np.asarray(position, dtype=np.float64))
        train_exec_quat.append(np.asarray(quaternion, dtype=np.float64))
    train_exec = np.stack(train_exec)
    train_exec_quat = np.stack(train_exec_quat)

    def at(array: np.ndarray, index: int) -> np.ndarray:
        return array[min(max(index, 0), array.shape[0] - 1)]

    result: dict[str, Any] = {"ticks_compared": len(order), "tick_span": [order[0], order[-1]]}
    for label, offset in (("aligned_tick", 0), ("aligned_tick_plus_one", 1)):
        joint_errors, position_errors, orientation_errors, goal_errors, tracking = (
            [],
            [],
            [],
            [],
            [],
        )
        for tick in order:
            sample = first[tick]
            index = tick + offset
            joint_errors.append(q_track[sample] - at(train_q, index))
            position_errors.append(np.linalg.norm(exec_track[sample] - at(train_exec, index)))
            orientation_errors.append(quat_angle_deg(exec_quat[sample], at(train_exec_quat, index)))
            goal_errors.append(np.linalg.norm(goal_track[sample] - at(train_goal, index)[:3]))
            tracking.append(np.linalg.norm(exec_track[sample] - goal_track[sample]))
        joint = np.asarray(joint_errors)
        tcp = pose_error_summary(position_errors, orientation_errors)
        result[label] = {
            "joint_mean_abs_rad": float(np.abs(joint).mean()),
            "joint_max_abs_rad": float(np.abs(joint).max()),
            "joint_rmse_rad": float(np.sqrt((joint**2).mean())),
            "tcp_position_mean_mm": tcp["position_mm"]["mean"],
            "tcp_position_p95_mm": tcp["position_mm"]["p95"],
            "tcp_position_max_mm": tcp["position_mm"]["max"],
            "tcp_orientation_mean_deg": tcp["orientation_deg"]["mean"],
            "tcp_orientation_p95_deg": tcp["orientation_deg"]["p95"],
            "tcp_orientation_max_deg": tcp["orientation_deg"]["max"],
            "dim_goal_vs_training_goal_mean_mm": float(np.mean(goal_errors) * 1000.0),
            "dim_exec_to_dim_goal_mean_mm": float(np.mean(tracking) * 1000.0),
        }
    result["training_rows"] = {
        "q": int(train_q.shape[0]),
        "full_q": int(train_full.shape[0]),
        "goal": int(train_goal.shape[0]),
    }
    result["note"] = (
        "aligned_tick is the primary alignment (same 30 Hz policy tick); "
        "aligned_tick_plus_one bounds a one-tick phase ambiguity and is reported, not hidden"
    )
    return result


def run(args: argparse.Namespace) -> dict[str, Any]:
    manifest = resolve_manifest_path(args.model_dir, args.manifest_path)
    plant = resolve_plant_path(manifest.parent, manifest)
    with np.load(args.initial_state) as data:
        qpos = np.asarray(data["qpos"], dtype=np.float64)
        qvel = np.asarray(data["qvel"], dtype=np.float64)
        stored_start = data["start_tick"].tolist() if "start_tick" in data.files else None
        stored_ref = data["ref_id"].tolist() if "ref_id" in data.files else None
    env = int(args.env)
    qpos0, qvel0 = qpos[env].copy(), qvel[env].copy()

    with np.load(args.training) as data:
        training = {key: np.asarray(data[key]) for key in data.files}
    clip = ReferenceClip(args.clip, policy_hz=POLICY_HZ)
    anchor_index, anchor_residual = resolve_anchor(clip, qpos0[:6], args.anchor_tolerance)
    with np.load(args.training) as data:
        finger = np.asarray(data["finger_q"], dtype=np.float64)[:, env, :]
    fingers_parked = bool(np.abs(finger - finger[0]).max() <= FINGER_TOLERANCE)
    initial_q_error = float(np.abs(qpos0[:6] - np.asarray(training["full_q"])[0, env, :6]).max())

    summary: dict[str, Any] = {
        "env": env,
        "clip_path": str(args.clip),
        "clip_sha256": sha256(args.clip),
        "training_path": str(args.training),
        "training_sha256": sha256(args.training),
        "initial_state_path": str(args.initial_state),
        "initial_state_sha256": sha256(args.initial_state),
        "plant_model": str(plant),
        "plant_sha256": sha256(plant),
        "manifest": str(manifest),
        "clip_rate": {
            "file_hz": clip.file_hz,
            "policy_hz": clip.policy_hz,
            "stride": clip.stride,
            "file_rows": clip.rows,
            "policy_ticks": clip.length,
        },
        "anchor": {
            "resolved_index_from_measured_pose": anchor_index,
            "residual_max_abs_rad": anchor_residual,
            "stored_start_tick": None if stored_start is None else int(stored_start[env]),
            "stored_ref_id": None if stored_ref is None else int(stored_ref[env]),
            "note": "index resolved from the initial-state arrays, not from notes",
        },
        "initial_state": {
            "max_abs_qpos_minus_training_full_q0_rad": initial_q_error,
            "qvel0_rad_s": qvel0[:6].round(6).tolist(),
        },
        "welded_fingers": {
            "training_finger_q_min": finger.min(axis=0).round(6).tolist(),
            "training_finger_q_max": finger.max(axis=0).round(6).tolist(),
            "training_fingers_parked": fingers_parked,
            "note": "the exported plant has six hinge joints and welds both finger "
            "bodies; this is only equivalent while the trained fingers stay parked",
        },
    }
    if stored_start is not None and int(stored_start[env]) != anchor_index:
        summary["anchor"]["mismatch"] = (
            f"stored start_tick {int(stored_start[env])} != resolved {anchor_index}; "
            "the resolved index was used because the initial-state arrays are authoritative"
        )
    summary["anchor"]["note"] = (
        "index is a 30 Hz policy tick; the 60 Hz clip is decimated by stride, so a policy "
        "tick is that many recorded rows"
    )
    if initial_q_error > 1e-5:
        summary["comparable"] = False
        summary["noncomparable_reason"] = (
            f"initial qpos differs from the training rollout by {initial_q_error:.6f} rad"
        )
        return summary

    summary["open_loop_provenance"] = open_loop_provenance(plant, training, qpos0, qvel0)
    max_ticks = int(min(180, clip.length - anchor_index - 1, np.asarray(training["q"]).shape[0]))
    arrays, loop = closed_loop(
        plant=plant,
        manifest=manifest,
        clip_path=Path(args.clip),
        anchor_index=anchor_index,
        qpos0=qpos0,
        qvel0=qvel0,
        max_ticks=max_ticks,
        timeout_s=args.timeout,
    )
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output / "clip_compare.npz", **arrays)
    summary["closed_loop"] = loop
    if int(loop["active_samples"]) > 0:
        summary["comparison"] = compare(arrays, training, MjcfTcpFk(plant), env)
        summary["comparable"] = True
    else:
        summary["comparable"] = False
        summary["noncomparable_reason"] = "the integrated session never became active"
    (output / "clip_compare.json").write_text(json.dumps(summary, indent=2, default=float) + "\n")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", default=None)
    parser.add_argument("--manifest-path", default=None)
    parser.add_argument("--clip", required=True)
    parser.add_argument("--initial-state", required=True)
    parser.add_argument("--training", required=True)
    parser.add_argument("--env", type=int, default=0)
    parser.add_argument("--anchor-tolerance", type=float, default=1e-3)
    parser.add_argument("--timeout", type=float, default=9.0)
    parser.add_argument("--output-dir", default="artifacts/openyam_policy/evidence/clip_compare")
    args = parser.parse_args()
    print(json.dumps(run(args), indent=2, default=float))


if __name__ == "__main__":
    main()
