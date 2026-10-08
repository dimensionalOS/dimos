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

"""Fake-hardware-only SEW group controller with human alignment and release latch."""

from __future__ import annotations

from pathlib import Path
import threading
from typing import Any

import numpy as np

from dimos.control.joint_command_envelope import bound_joint_command
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.control.tasks.sew_teleop_task.body import adapt_body
from dimos.control.tasks.sew_teleop_task.waist import WaistSolver
from dimos.manipulation.planning.kinematics.sew_retargeting import SewArmTarget, rotation_error
from dimos.robot.galaxea.r1pro.sew_model import R1SewModel
from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot
from dimos.teleop.webxr.controller_types import Buttons


class SewTeleopTask(BaseControlTask):
    def __init__(self, name: str, model: R1SewModel) -> None:
        self._name = name
        self.model = model
        self.waist = WaistSolver(model)
        self.names = [f"r1pro/{n}" for n in model.names]
        self.lock = threading.RLock()
        self.state = "WAIT_RELEASE"
        self.reason = "Release both grips, then align"
        self.snapshot: BodyTrackingSnapshot | None = None
        self.received = -np.inf
        self.buttons_at = -np.inf
        self.capture = -np.inf
        self.pressed = False
        self.previous: np.ndarray | None = None
        self.aligned_since: float | None = None
        self.reference: tuple[float, float, float] | None = None
        self.status: dict[str, Any] = {}
        self.estopped = False
        # Explicit experimental fake-hardware thresholds, not hardware-validated.
        self.angle_tolerance = np.deg2rad(8.0)
        self.joint_tolerance = np.deg2rad(10.0)
        self.height_tolerance = 0.025
        self.stable_time = 0.3
        self.timeout = 0.5

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=frozenset(self.names), priority=20, mode=ControlMode.SERVO_POSITION
        )

    def is_active(self) -> bool:
        # Compute while disarmed for alignment UI, but emit no joint command.
        return True

    def disarm(self, reason: str) -> None:
        self.state = "WAIT_RELEASE"
        self.reason = reason
        self.previous = None
        self.aligned_since = None

    def on_body_tracking(self, snapshot: BodyTrackingSnapshot, t_now: float) -> bool:
        with self.lock:
            if snapshot.capture_time_s <= self.capture:
                self.disarm("Non-monotonic body timestamp")
                return False
            if self.snapshot is not None and snapshot.frame_id != self.snapshot.frame_id:
                self.reference = None
                self.disarm("Reference space changed")
            self.capture = snapshot.capture_time_s
            self.snapshot = snapshot
            self.received = t_now
        return True

    def on_teleop_buttons(self, msg: Buttons, t_now: float) -> bool:
        with self.lock:
            old = self.pressed
            self.pressed = bool(msg.left_grip and msg.right_grip)
            self.buttons_at = t_now
            if not msg.left_grip and not msg.right_grip and not self.estopped:
                self.state = "DISARMED"
                self.previous = None
                self.aligned_since = None
            elif self.state == "DISARMED" and self.pressed and not old and not self.estopped:
                self.state = "ALIGNING"
            elif not self.pressed and self.state in ("ALIGNING", "ACTIVE"):
                self.disarm("Grip released")
        return True

    def set_estop(self, estopped: bool) -> None:
        with self.lock:
            self.estopped = estopped
            self.disarm("E-stop" if estopped else "Release to rearm")

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        with self.lock:
            self.disarm("Preempted")

    def start(self) -> None:
        pass

    def stop(self) -> None:
        with self.lock:
            self.disarm("Stopped")

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        with self.lock:
            try:
                return self._compute(state)
            except Exception as exc:
                self.disarm(str(exc))
                self.status = {"state": self.state, "reason": self.reason}
                return None

    def _compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if not np.isfinite(state.dt) or not 0 < state.dt <= 0.1:
            raise ValueError("Invalid or stalled control timestep")
        if self.estopped:
            raise ValueError("E-stop")
        if self.snapshot is None or state.t_now - self.received > self.timeout:
            raise ValueError("Body tracking stale")
        if state.t_now - self.buttons_at > self.timeout:
            raise ValueError("Buttons stale")
        body = adapt_body(self.snapshot)
        measured = np.array([state.joints.get_position(n) for n in self.names], dtype=float)
        if (
            not np.isfinite(measured).all()
            or np.any(measured < self.model.lower - 1e-3)
            or np.any(measured > self.model.upper + 1e-3)
        ):
            raise ValueError("Invalid feedback")
        frames = self.model.fk(measured)
        if self.reference is None:
            self.reference = (
                float(np.arctan2(body.chest_rotation[1, 0], body.chest_rotation[0, 0])),
                body.height,
                float(frames["torso_link4"][2, 3]),
            )
        heading, h0, z0 = self.reference
        yaw = float(np.arctan2(body.chest_rotation[1, 0], body.chest_rotation[0, 0]) - heading)
        yaw = float(np.arctan2(np.sin(yaw), np.cos(yaw)))
        pitch = float(
            np.arctan2(
                -body.chest_rotation[2, 0],
                np.hypot(body.chest_rotation[0, 0], body.chest_rotation[1, 0]),
            )
        )
        waist_target = self.waist.target(pitch, yaw, z0 + body.height - h0)
        candidate = measured.copy()
        errors = {}
        chest = frames["torso_link4"]
        for i, side in enumerate(("left", "right")):
            mount = chest[:3, :3].T @ frames[f"{side}_arm_base_link"][:3, :3]
            a = body.arms[side]
            target = SewArmTarget(
                mount.T @ a.shoulder,
                mount.T @ a.elbow,
                mount.T @ a.wrist,
                mount.T @ a.hand_rotation,
            )
            sl = slice(4 + 7 * i, 11 + 7 * i)
            solver = self.model.solvers[side]
            candidate[sl] = solver.solve(target, measured[sl])
            u, l, h = solver.features(measured[sl])
            tu, tl = target.directions()
            errors[side] = [
                float(np.arccos(np.clip(u @ tu, -1, 1))),
                float(np.arccos(np.clip(l @ tl, -1, 1))),
                rotation_error(h, target.hand_rotation),
            ]
        chest_angle = rotation_error(chest[:3, :3], waist_target.rotation)
        height_error = abs(float(chest[2, 3] - waist_target.translation[2]))
        joint_error = float(np.max(np.abs(candidate[4:] - measured[4:])))
        ready = (
            max(*errors["left"], *errors["right"], chest_angle) < self.angle_tolerance
            and height_error < self.height_tolerance
            and joint_error < self.joint_tolerance
        )
        self.status = {
            "state": self.state,
            "reason": self.reason,
            "ready": bool(ready),
            "arm_error_deg": {s: np.rad2deg(v).tolist() for s, v in errors.items()},
            "chest_error_deg": float(np.rad2deg(chest_angle)),
            "height_error_m": height_error,
            "max_arm_joint_error_deg": float(np.rad2deg(joint_error)),
            "waist_policy": "pitch/yaw + height; roll ignored",
            "experimental_fake_only": True,
            "alignment_thresholds": {
                "orientation_deg": float(np.rad2deg(self.angle_tolerance)),
                "joint_deg": float(np.rad2deg(self.joint_tolerance)),
                "height_m": self.height_tolerance,
                "stable_seconds": self.stable_time,
            },
            "target_joints": candidate.tolist(),
            "measured_joints": measured.tolist(),
        }
        desired_frames = self.model.fk(candidate)
        self.status["skeleton_front"] = {
            label: {
                side: [
                    [
                        float(source[f"{side}_arm_link{k}"][1, 3]),
                        float(source[f"{side}_arm_link{k}"][2, 3]),
                    ]
                    for k in (2, 4, 7)
                ]
                for side in ("left", "right")
            }
            for label, source in (("measured", frames), ("target", desired_frames))
        }
        if self.state == "ALIGNING":
            self.reason = (
                "Hold pose within all alignment bounds" if not ready else "Alignment stable window"
            )
            if not ready:
                self.aligned_since = None
            elif self.aligned_since is None:
                self.aligned_since = state.t_now
            elif state.t_now - self.aligned_since >= self.stable_time:
                self.state = "ACTIVE"
                self.previous = measured.copy()
                self.reason = "Tracking"
        self.status.update(state=self.state, reason=self.reason)
        if self.state != "ACTIVE":
            return None
        assert self.previous is not None
        candidate[:4] = self.waist.step(self.previous[:4], waist_target, state.dt)
        self.previous = bound_joint_command(
            candidate,
            self.previous,
            measured,
            self.model.lower + 1e-4,
            self.model.upper - 1e-4,
            np.minimum(self.model.velocity, 0.5),
            state.dt,
            np.deg2rad(10.0),
        )
        self.status["state"] = self.state
        self.status["command_joints"] = self.previous.tolist()
        return JointCommandOutput(
            joint_names=self.names,
            positions=self.previous.tolist(),
            mode=ControlMode.SERVO_POSITION,
        )


def create_task(cfg: Any, hardware: Any) -> SewTeleopTask:
    del hardware
    model = R1SewModel(Path(cfg.params["urdf_path"]))
    task = SewTeleopTask(cfg.name, model)
    if cfg.joint_names != task.names:
        raise ValueError("SEW requires the complete ordered R1 upper-body group")
    return task
