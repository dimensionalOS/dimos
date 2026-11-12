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

"""Matched actor-11000 vs actor-6000 comparison from the preserved DimOS closed-loop traces.

No simulation, no model, no network: everything here is recomputed from the two
``closed_loop.npz`` traces that the actual ``openyam_policy_sim_replay`` blueprint
run wrote. The evidence root defaults to ``$OPENYAM_EVIDENCE_ROOT`` or the current
directory, so the evidence folder is self-contained after the scratch workspace is
gone::

    PYTHONPATH=<dimos checkout>
    python -m dimos.robot.manipulators.openyam.policy.demo_compare_actors --evidence-root .
    python -m dimos.robot.manipulators.openyam.policy.demo_compare_actors --no-urdf-fk

Metric conventions (all mean / P95 / max over the *complete* active window):

* ``policy_goal_tracking`` — policy goal vs executed joint state, both in the
  trained plant's ``arm_tcp`` frame, sampled in the same 60 Hz sampler iteration.
* ``requested_vs_achievable`` — operator request vs its IK projection, both in
  the deployed URDF ``gripper_tip`` frame.
* ``request_tracking_urdf_tool`` — operator request vs the FK of the *measured*
  joints in that same URDF tool frame: genuine controller-request tracking.
* ``cross_frame_*`` — URDF tool-frame poses against plant-frame poses. Reported
  only as a measured offset, marked ``valid: false``: subtracting them is not a
  tracking metric.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any

import numpy as np

from dimos.robot.manipulators.openyam.policy.tcp import pose_error_summary, quat_angle_deg

RUNS = ("11000", "6000")


# --------------------------------------------------------------------------- io
def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def load(evidence_root: Path, name: str) -> dict[str, np.ndarray]:
    with np.load(evidence_root / name / "closed_loop.npz", allow_pickle=False) as data:
        return {key: np.asarray(data[key], dtype=np.float64) for key in data.files}


def load_summary(evidence_root: Path, name: str) -> dict[str, Any]:
    path = evidence_root / name / "closed_loop.json"
    return json.loads(path.read_text()) if path.is_file() else {}


# ------------------------------------------------------------------------ masks
def active_mask(arrays: dict[str, np.ndarray]) -> np.ndarray:
    """Every recorded sample whose session was active (the full active window)."""
    return arrays["active"] > 0.5


def ticks_first(arrays: dict[str, np.ndarray]) -> np.ndarray:
    """First sample of each active policy tick (secondary, per-tick diagnostic)."""
    rows = active_mask(arrays)
    order: dict[int, int] = {}
    for index in np.flatnonzero(rows):
        order.setdefault(int(arrays["policy_ticks"][index]), index)
    return np.array([order[tick] for tick in sorted(order)], dtype=int)


def anchor_rows(arrays: dict[str, np.ndarray], rows: np.ndarray) -> np.ndarray:
    """Samples still carrying the session-anchor reference diagnostics (tick == min)."""
    ticks = arrays["policy_ticks"][rows]
    return rows[ticks == ticks.min()]


def correct_anchor_achievable(arrays: dict[str, np.ndarray], rows: np.ndarray) -> np.ndarray:
    """Achievable pose at the anchor in the *requested* (URDF tool) frame.

    The run-229 traces were recorded before the generator's anchor diagnostic was
    fixed: at the anchor ``achievable`` came from the plant goal FK while
    ``requested`` was the deployed URDF tool frame, so the frame offset appeared
    as IK projection error. At the anchor the IK target is the measured pose, so
    the fixed generator records ``achievable == requested``; this reproduces that.
    Returns the corrected copy of the achievable arrays.
    """
    reachable = {key: arrays[key].copy() for key in arrays if key.startswith("ach_")}
    for row in anchor_rows(arrays, rows):
        for axis in ("x", "y", "z", "qw", "qx", "qy", "qz"):
            reachable[f"ach_{axis}"][row] = arrays[f"req_{axis}"][row]
    return reachable


# --------------------------------------------------------------------- geometry
def vector(arrays: dict[str, np.ndarray], prefix: str) -> np.ndarray:
    return np.stack([arrays[f"{prefix}_x"], arrays[f"{prefix}_y"], arrays[f"{prefix}_z"]], axis=1)


def quaternion(arrays: dict[str, np.ndarray], prefix: str) -> np.ndarray:
    return np.stack(
        [
            arrays[f"{prefix}_qw"],
            arrays[f"{prefix}_qx"],
            arrays[f"{prefix}_qy"],
            arrays[f"{prefix}_qz"],
        ],
        axis=1,
    )


def pose_errors(
    position_a: np.ndarray,
    quaternion_a: np.ndarray,
    position_b: np.ndarray,
    quaternion_b: np.ndarray,
) -> dict[str, dict[str, float]]:
    return pose_error_summary(
        np.linalg.norm(position_a - position_b, axis=1).tolist(),
        [
            quat_angle_deg(quaternion_a[index], quaternion_b[index])
            for index in range(len(position_a))
        ],
    )


def tool_frame_fk(q: np.ndarray) -> tuple[np.ndarray, np.ndarray] | None:
    """FK of the deployed URDF ``gripper_tip`` frame at each joint row.

    Returns ``None`` when the IK stack (pink + pinocchio) is not importable, so the
    script still runs and the caller marks request tracking unavailable.
    """
    try:
        from dimos.control.tasks.pose_target_ik import PoseTargetIKTaskConfig
        from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
        from dimos.msgs.sensor_msgs.JointState import JointState
        from dimos.robot.manipulators.openyam.config import make_openyam_model_config
        from dimos.robot.manipulators.openyam.policy.quest_reference import TOOL_FRAME
        from dimos.robot.manipulators.openyam.teleop_ik import OpenYamPinkPoseTargetSolver
        from dimos.utils.transform_utils import pose_to_matrix
    except Exception:
        return None

    model = make_openyam_model_config(robot="arm")
    config = PoseTargetIKTaskConfig(
        joint_names=tuple(model.joint_names),
        robot_model=model,
        target_frames=(TOOL_FRAME,),
        pink=PinkKinematicsConfig(
            dt=1.0 / 30.0,
            position_cost=8.0,
            orientation_cost=2.0,
            posture_cost=0.01,
            joint_limit_posture_margin=0.3,
            lm_damping=0.01,
            gain=1.0,
        ),
        max_joint_velocity_rad_s=2.0,
        max_command_tracking_error_deg=10.0,
        joint_command_filter_cutoff_hz=30.0,
    )
    names = list(config.joint_names)
    solver = OpenYamPinkPoseTargetSolver(config)
    positions: list[np.ndarray] = []
    rotations: list[np.ndarray] = []
    for row in q:
        pose = solver.frame_poses(
            JointState(name=names, position=[float(value) for value in row]), (TOOL_FRAME,)
        )[TOOL_FRAME]
        matrix = pose_to_matrix(pose)
        positions.append(matrix[:3, 3])
        rotation = matrix[:3, :3]
        # wxyz, matching the trace layout.
        w = np.sqrt(max(0.0, 1.0 + rotation[0, 0] + rotation[1, 1] + rotation[2, 2])) / 2.0
        x = np.sqrt(max(0.0, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) / 2.0
        y = np.sqrt(max(0.0, 1.0 - rotation[0, 0] + rotation[1, 1] - rotation[2, 2])) / 2.0
        z = np.sqrt(max(0.0, 1.0 - rotation[0, 0] - rotation[1, 1] + rotation[2, 2])) / 2.0
        x = np.copysign(x, rotation[2, 1] - rotation[1, 2])
        y = np.copysign(y, rotation[0, 2] - rotation[2, 0])
        z = np.copysign(z, rotation[1, 0] - rotation[0, 1])
        rotations.append(np.array([w, x, y, z], dtype=np.float64))
    return np.array(positions), np.array(rotations)


# ---------------------------------------------------------------------- windows
def channel_report(
    arrays: dict[str, np.ndarray],
    rows: np.ndarray,
    *,
    urdf_fk: tuple[np.ndarray, np.ndarray] | None,
    urdf_rows: np.ndarray | None = None,
) -> dict[str, Any]:
    """Every pose channel for one run over one row selection."""
    reachable = correct_anchor_achievable(arrays, rows)
    merged = {**arrays, **reachable}
    goal_p, goal_q = vector(merged, "goal")[rows], quaternion(merged, "goal")[rows]
    exec_p, exec_q = vector(merged, "exec")[rows], quaternion(merged, "exec")[rows]
    req_p, req_q = vector(merged, "req")[rows], quaternion(merged, "req")[rows]
    ach_p, ach_q = vector(merged, "ach")[rows], quaternion(merged, "ach")[rows]

    report: dict[str, Any] = {
        "samples": int(rows.size),
        "policy_tick_ids": [int(value) for value in np.unique(arrays["policy_ticks"][rows])],
        "time_s": [float(arrays["t"][rows].min()), float(arrays["t"][rows].max())],
        "policy_goal_tracking": pose_errors(goal_p, goal_q, exec_p, exec_q),
        "requested_vs_achievable": pose_errors(req_p, req_q, ach_p, ach_q),
        "cross_frame_requested_vs_executed": {
            **pose_errors(req_p, req_q, exec_p, exec_q),
            "valid": False,
            "reason": (
                "requested/achievable are deployed URDF gripper_tip poses; executed is the trained "
                "plant arm_tcp body. The two frames are ~80 mm / ~100 deg apart by construction, so "
                "this subtraction is a frame offset, not a tracking error."
            ),
        },
        "cross_frame_achievable_vs_executed": {
            **pose_errors(ach_p, ach_q, exec_p, exec_q),
            "valid": False,
            "reason": "same frame mismatch as cross_frame_requested_vs_executed",
        },
    }
    if urdf_fk is not None and urdf_rows is not None:
        tool_p, tool_q = urdf_fk[0][urdf_rows], urdf_fk[1][urdf_rows]
        report["request_tracking_urdf_tool"] = {
            **pose_errors(req_p, req_q, tool_p, tool_q),
            "valid": True,
            "note": (
                "operator request (URDF gripper_tip) vs FK of the measured joints in the same URDF "
                "frame, i.e. genuine controller-request tracking. The request comes from the policy "
                "tick's state_snapshot RPC and the joints from the 60 Hz SHM read, so the pair is not "
                "atomic: the executed side lags by up to one physics step (~2 ms) plus the RPC round trip."
            ),
        }
    else:
        report["request_tracking_urdf_tool"] = {
            "valid": False,
            "reason": "URDF FK unavailable in this environment (pink/pinocchio not importable)",
        }
    return report


def finite_coverage(arrays: dict[str, np.ndarray], rows: np.ndarray) -> dict[str, float]:
    def coverage(prefix: str) -> float:
        values = np.stack([arrays[f"{prefix}_{axis}"][rows] for axis in ("x", "y", "z")], axis=1)
        return float(np.isfinite(values).all(axis=1).mean())

    command = np.stack([arrays[f"cmd{index}"][rows] for index in range(6)], axis=1)
    return {
        "goal": coverage("goal"),
        "exec": coverage("exec"),
        "req": coverage("req"),
        "ach": coverage("ach"),
        "command": float(np.isfinite(command).all(axis=1).mean()),
        "reference_dt": float(np.isfinite(arrays["dt_s"][rows]).mean()),
    }


def run_report(arrays: dict[str, np.ndarray], summary: dict[str, Any]) -> dict[str, Any]:
    rows = np.flatnonzero(active_mask(arrays))
    ticks = arrays["policy_ticks"][rows].astype(int)
    unique, counts = np.unique(ticks, return_counts=True)
    return {
        "npz_samples_total": int(arrays["t"].size),
        "active_samples": int(rows.size),
        "active_mask": "active > 0.5 (session == 'active' at that sampler iteration)",
        "unique_policy_ticks": int(unique.size),
        "policy_tick_ids": [int(unique.min()), int(unique.max())],
        "samples_per_tick": {
            str(int(tick)): int(count) for tick, count in zip(unique, counts, strict=True)
        },
        "time_s": [float(arrays["t"][rows].min()), float(arrays["t"][rows].max())],
        "finite_coverage": finite_coverage(arrays, rows),
        "faults": summary.get("faults", []),
        "events": summary.get("events", []),
        "actor_cadence": summary.get("actor_cadence"),
        "physics": summary.get("physics"),
        "reference_dt": summary.get("reference_dt"),
    }


# ------------------------------------------------------------------------ entry
def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--evidence-root",
        type=Path,
        default=Path(os.environ.get("OPENYAM_EVIDENCE_ROOT", ".")),
        help="directory holding 11000/ and 6000/ traces (default: $OPENYAM_EVIDENCE_ROOT or .)",
    )
    parser.add_argument(
        "--no-urdf-fk",
        action="store_true",
        help="skip the deployed-URDF FK (request tracking is then reported as unavailable)",
    )
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    root: Path = args.evidence_root

    runs = {name: load(root, name) for name in RUNS}
    summaries = {name: load_summary(root, name) for name in RUNS}

    report: dict[str, Any] = {
        "evidence_root": str(root),
        "inputs": {
            name: {
                "closed_loop.npz": sha256(root / name / "closed_loop.npz"),
                "closed_loop.json": sha256(root / name / "closed_loop.json"),
            }
            for name in RUNS
        },
        "channel_semantics": {
            "goal": "policy goal = FK of q_ref on the trained plant body arm_tcp",
            "exec": "executed = FK of the measured joints on the same arm_tcp body",
            "req": "operator/input target = anchored WebXR controller pose in the deployed URDF gripper_tip frame",
            "ach": "IK projection of req, deployed URDF gripper_tip frame",
        },
        "metric_convention": (
            "mean / P95 / max over every recorded sample whose session was active "
            "(the complete active window), not a first-sample-per-tick subset"
        ),
        "runs": {name: run_report(runs[name], summaries[name]) for name in RUNS},
        "anchor_diagnostic_correction": {},
        "active_window_channels": {},
        "steady_first_sample_per_tick": {},
        "matched_comparison_11000_vs_6000": {},
        "limitations": [
            (
                "The two runs are separate executions of the same blueprint/plant/controller/scripted "
                "input replay, not one synchronised session: the shared 30 Hz clock is the coordinator's."
            ),
            (
                "state_snapshot (RPC) and the SHM joint read are not atomic; per-sample pose pairs carry "
                "the RPC round trip and up to one physics step of skew."
            ),
            (
                "requested/achievable live in the deployed URDF tool frame; goal/executed in the trained "
                "plant arm_tcp frame. Only same-frame channels are tracking metrics."
            ),
        ],
    }

    # --- anchor diagnostic: how large was the run-229 frame-convention artifact?
    anchor_report: dict[str, Any] = {}
    for name, arrays in runs.items():
        rows = np.flatnonzero(active_mask(arrays))
        anchors = anchor_rows(arrays, rows)
        before = pose_errors(
            vector(arrays, "req")[anchors],
            quaternion(arrays, "req")[anchors],
            vector(arrays, "ach")[anchors],
            quaternion(arrays, "ach")[anchors],
        )
        corrected = correct_anchor_achievable(arrays, anchors)
        merged = {**arrays, **corrected}
        after = pose_errors(
            vector(merged, "req")[anchors],
            quaternion(merged, "req")[anchors],
            vector(merged, "ach")[anchors],
            quaternion(merged, "ach")[anchors],
        )
        anchor_report[name] = {
            "rows": [int(row) for row in anchors],
            "policy_tick_ids": [int(value) for value in np.unique(arrays["policy_ticks"][anchors])],
            "as_recorded": before,
            "corrected_same_urdf_frame": after,
        }
    report["anchor_diagnostic_correction"] = {
        "defect": (
            "quest_reference.anchor() recorded achievable from the plant goal FK while requested was "
            "the deployed URDF tool frame, so the anchor rows carried the ~80 mm / ~100 deg frame "
            "offset as IK projection error. Fixed in the generator; recomputed here from the saved q."
        ),
        "runs": anchor_report,
    }

    # --- primary: the complete active window, per run
    fk_cache: dict[str, tuple[np.ndarray, np.ndarray] | None] = {}
    for name, arrays in runs.items():
        rows = np.flatnonzero(active_mask(arrays))
        fk = None
        if not args.no_urdf_fk:
            q = np.stack([arrays[f"q{index}"] for index in range(6)], axis=1)
            fk = tool_frame_fk(q[rows])
        fk_cache[name] = fk
        report["active_window_channels"][name] = channel_report(
            arrays, rows, urdf_fk=fk, urdf_rows=np.arange(rows.size) if fk is not None else None
        )
        tick_rows = ticks_first(arrays)
        report["steady_first_sample_per_tick"][name] = channel_report(
            arrays,
            tick_rows,
            urdf_fk=fk,
            urdf_rows=np.searchsorted(rows, tick_rows),
        )
        # The previously published headline (`rows[1:]`) dropped the anchor *tick*:
        # keep it as an explicitly labelled subset so the old figures stay checkable.
        post_anchor = tick_rows[1:]
        report.setdefault("steady_first_sample_per_tick_post_anchor", {})[name] = channel_report(
            arrays,
            post_anchor,
            urdf_fk=fk,
            urdf_rows=np.searchsorted(rows, post_anchor),
        )

    # --- matched comparison: ordinal policy-tick alignment after the anchor
    steady = {
        name: ticks_first(arrays)[1:] for name, arrays in runs.items()
    }  # drop the anchor tick in both runs
    tick_ids = {name: runs[name]["policy_ticks"][rows].astype(int) for name, rows in steady.items()}
    common_ticks = [tick for tick in tick_ids["11000"] if tick in set(tick_ids["6000"].tolist())]
    index_a = {tick: position for position, tick in enumerate(tick_ids["11000"])}
    index_b = {tick: position for position, tick in enumerate(tick_ids["6000"])}
    rows_a = steady["11000"][[index_a[tick] for tick in common_ticks]]
    rows_b = steady["6000"][[index_b[tick] for tick in common_ticks]]

    runs_a, runs_b = runs["11000"], runs["6000"]
    q_a = np.stack([runs_a[f"q{index}"][rows_a] for index in range(6)], axis=1)
    q_b = np.stack([runs_b[f"q{index}"][rows_b] for index in range(6)], axis=1)
    cmd_a = np.stack([runs_a[f"cmd{index}"][rows_a] for index in range(6)], axis=1)
    cmd_b = np.stack([runs_b[f"cmd{index}"][rows_b] for index in range(6)], axis=1)
    exec_a, exec_b = vector(runs_a, "exec")[rows_a], vector(runs_b, "exec")[rows_b]
    quat_a, quat_b = quaternion(runs_a, "exec")[rows_a], quaternion(runs_b, "exec")[rows_b]
    goal_a, goal_b = vector(runs_a, "goal")[rows_a], vector(runs_b, "goal")[rows_b]
    time_offset = np.abs(runs_a["t"][rows_a] - runs_b["t"][rows_b])

    matched_orientation = np.array(
        [quat_angle_deg(quat_a[index], quat_b[index]) for index in range(len(common_ticks))]
    )
    report["matched_comparison_11000_vs_6000"] = {
        "ticks": len(common_ticks),
        "tick_ids": {
            "first": int(min(common_ticks)),
            "last": int(max(common_ticks)),
            "contiguous": bool(len(common_ticks) == max(common_ticks) - min(common_ticks) + 1),
        },
        "alignment": (
            "ordinal policy-tick alignment of the first sample of each tick, anchor tick excluded in "
            "both runs. This is not the same input timestamp: the actor ticks are driven by each run's "
            "own coordinator clock and the two sessions started 0.017 s apart."
        ),
        "sample_time_offset_s": {
            "mean": float(time_offset.mean()),
            "max": float(time_offset.max()),
        },
        "joint_mrad_mean": float(np.abs(q_a - q_b).mean() * 1000.0),
        "joint_mrad_max": float(np.abs(q_a - q_b).max() * 1000.0),
        "command_mrad_mean": float(np.abs(cmd_a - cmd_b).mean() * 1000.0),
        "command_mrad_max": float(np.abs(cmd_a - cmd_b).max() * 1000.0),
        "exec_tcp": pose_error_summary(
            np.linalg.norm(exec_a - exec_b, axis=1).tolist(), matched_orientation.tolist()
        ),
        "goal_position_delta_mm": {
            "mean": float(np.linalg.norm(goal_a - goal_b, axis=1).mean() * 1000.0),
            "max": float(np.linalg.norm(goal_a - goal_b, axis=1).max() * 1000.0),
        },
        "note": (
            "same scripted WebXR input replay, same frozen plant, reset pose and duration; both runs "
            "measured ~150 actor ticks at 30.1-30.2 Hz with 465.6-465.9 Hz measured physics"
        ),
    }

    output = args.output or (root / "comparison_11000_vs_6000.json")
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
