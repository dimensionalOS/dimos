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

"""Bounded CPU comparison on common generated SEW/hand inputs; no robot IO."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from time import perf_counter
from typing import Any

import numpy as np

from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver, PoseTargetIKTaskConfig
from dimos.manipulation.planning.kinematics.sew_retargeting import (
    SewArmTarget,
    rotation_error,
    unit,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.galaxea.r1pro.joints import PASSIVE_JOINTS
from dimos.robot.galaxea.r1pro.sew_model import R1SewModel
from dimos.utils.transform_utils import matrix_to_pose


def compare(path: Path, frames: int) -> dict[str, Any]:
    model = R1SewModel(path)
    names = [f"r1pro/{n}" for n in model.names]
    robot = (
        RobotModel.from_file(path)
        .with_fixed_joints(*PASSIVE_JOINTS)
        .with_default_joint_acceleration_limit(2.0)
        .with_renamed_joints(dict(zip(model.names, names, strict=True)))
    )
    pink = PinkPoseTargetSolver(
        PoseTargetIKTaskConfig(
            joint_names=tuple(names[4:]),
            robot_model=RobotModelConfig(model=robot, joint_names=names, base_link="base_link"),
            target_frames=("left_arm_link7", "right_arm_link7"),
        )
    )
    initial = np.array(
        [0.0] * 4 + [0.0, 0.3, 0.0, -0.6, 0.0, 0.0, 0.0] + [0.0, -0.3, 0.0, -0.6, 0.0, 0.0, 0.0]
    )
    states = {key: initial.copy() for key in ("sew", "pink")}
    records: list[dict[str, Any]] = []
    for frame in range(frames):
        reference = initial.copy()
        reference[4:] += 0.08 * np.sin(frame * 0.06 + np.arange(14) * 0.2)
        fk = model.fk(reference)
        targets = {}
        poses = {}
        for side in ("left", "right"):
            mount = fk[f"{side}_arm_base_link"]
            r = mount[:3, :3]
            p = mount[:3, 3]
            s, e, w = [fk[f"{side}_arm_link{i}"][:3, 3] for i in (2, 4, 7)]
            h = fk[f"{side}_arm_link7"][:3, :3]
            targets[side] = SewArmTarget(r.T @ (s - p), r.T @ (e - p), r.T @ (w - p), r.T @ h)
            pose = matrix_to_pose(fk[f"{side}_arm_link7"])
            poses[f"{side}_arm_link7"] = PoseStamped(
                frame_id="base_link", position=pose.position, orientation=pose.orientation
            )
        for backend, current in list(states.items()):
            old = current.copy()
            now = old.copy()
            error = None
            started = perf_counter()
            try:
                if backend == "sew":
                    for i, side in enumerate(("left", "right")):
                        sl = slice(4 + 7 * i, 11 + 7 * i)
                        now[sl] = model.solvers[side].solve(targets[side], old[sl])
                else:
                    result = pink.step(
                        poses, JointState(name=names[4:], position=old[4:].tolist()), 0.01
                    )
                    if result is None:
                        raise ValueError("Pink returned no result")
                    now[4:] = result.position
            except ValueError as exc:
                error = str(exc)
            duration = perf_counter() - started
            states[backend] = now
            actual = model.fk(now)
            metrics = {}
            for i, side in enumerate(("left", "right")):
                r = actual[f"{side}_arm_base_link"][:3, :3]
                s, e, w = [actual[f"{side}_arm_link{k}"][:3, 3] for k in (2, 4, 7)]
                target = targets[side]
                u, l = target.directions()
                pu, pl, h = model.solvers[side].features(now[4 + 7 * i : 11 + 7 * i])

                def angle(a: np.ndarray, b: np.ndarray) -> float:
                    return float(np.arccos(np.clip(unit(a) @ unit(b), -1, 1)))

                metrics[side] = {
                    "proxy_orientation_rad": [angle(pu, u), angle(pl, l)],
                    "bone_orientation_rad": [angle(r.T @ (e - s), u), angle(r.T @ (w - e), l)],
                    "hand_rad": rotation_error(h, target.hand_rotation),
                    "wrist_actual_m": w.tolist(),
                    "wrist_position_error_m": float(
                        np.linalg.norm(w - fk[f"{side}_arm_link7"][:3, 3])
                    ),
                }
            records.append(
                {
                    "frame": frame,
                    "backend": backend,
                    "seconds": duration,
                    "failure": error,
                    "max_joint_step_rad": float(np.max(np.abs(now - old))),
                    "limit_violation": bool(np.any(now < model.lower) | np.any(now > model.upper)),
                    "metrics": metrics,
                }
            )
    summary = {
        b: {
            "failures": sum(r["failure"] is not None for r in records if r["backend"] == b),
            "median_ms": float(
                np.median([r["seconds"] * 1000 for r in records if r["backend"] == b])
            ),
            "p95_ms": float(
                np.percentile([r["seconds"] * 1000 for r in records if r["backend"] == b], 95)
            ),
        }
        for b in states
    }
    return {
        "note": "Same generated physical SEW/hand targets. SEW raw analytic output vs existing bounded Pink streaming step; timing includes different work. Not a dynamics or safety comparison. Torso held fixed in this comparison.",
        "summary": summary,
        "records": records,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.frames <= 1000:
        parser.error("frames must be 1..1000")
    result = compare(args.urdf.resolve(), args.frames)
    args.output.write_text(json.dumps(result, indent=2))
    print(json.dumps(result["summary"], indent=2))


if __name__ == "__main__":
    main()
