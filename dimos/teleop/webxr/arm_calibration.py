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

"""Shoulder-relative arm retargeting in gravity-aligned operator coordinates.

One positive scale per arm preserves movement directions and the fraction of
available reach. Neutral calibrates grip axes, never a preset hand position.
Robot wrist geometry and tool offsets are supplied by the robot adapter.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray
from pydantic import BaseModel, ConfigDict, Field, field_validator
from scipy.spatial.transform import Rotation

Side = Literal["left", "right"]
InputKind = Literal["hands", "controllers"]
SIDES: tuple[Side, ...] = ("left", "right")
CAPTURE_WINDOW_S = 0.6
INPUT_MAX_AGE_S = 0.3
MAX_POSITION_SPREAD_M = 0.02
MAX_ROTATION_SPREAD_RAD = math.radians(8)


class CalibrationPose(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, frozen=True)
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]

    @field_validator("orientation")
    @classmethod
    def unit_quaternion(cls, value: tuple[float, ...]) -> tuple[float, ...]:
        if abs(float(np.linalg.norm(value)) - 1) > 0.01:
            raise ValueError("orientation must be a unit quaternion")
        return value

    @classmethod
    def from_matrix(cls, matrix: NDArray[np.float64]) -> CalibrationPose:
        return cls(
            position=tuple(matrix[:3, 3]),
            orientation=tuple(Rotation.from_matrix(matrix[:3, :3]).as_quat()),
        )


class ArmAnchors(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    neutral: CalibrationPose
    reach: CalibrationPose
    shoulder: tuple[float, float, float] | None = None


class RobotArmGeometry(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, frozen=True)
    shoulder: tuple[float, float, float]
    reach_m: float = Field(gt=0)
    neutral_orientation: tuple[float, float, float, float]
    # Vector from tool origin to wrist origin, expressed in tool coordinates.
    tool_to_wrist: tuple[float, float, float]


class CalibrationProfile(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, frozen=True)
    schema_version: Literal[1, 2] = 2
    name: str = Field(min_length=1, max_length=80)
    input_kind: InputKind
    device: str = Field(min_length=1, max_length=300)
    robot: str
    saved_at: float
    arms: dict[Side, ArmAnchors]


def head_frame(position: tuple[float, float, float], yaw: float) -> NDArray[np.float64]:
    frame = np.eye(4)
    frame[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    frame[:3, 3] = position
    return frame


def relative_pose(pose: CalibrationPose, frame: NDArray[np.float64]) -> CalibrationPose:
    turn = Rotation.from_matrix(frame[:3, :3]).inv()
    return CalibrationPose(
        position=tuple(turn.apply(np.asarray(pose.position) - frame[:3, 3])),
        orientation=tuple((turn * Rotation.from_quat(pose.orientation)).as_quat()),
    )


def reach_scale(human: ArmAnchors, robot: RobotArmGeometry) -> float:
    if human.shoulder is None:
        raise ValueError("This profile needs shoulder calibration; capture a new profile")
    reach = np.subtract(human.reach.position, human.shoulder)
    length = float(np.linalg.norm(reach))
    neutral_length = float(np.linalg.norm(np.subtract(human.neutral.position, human.shoulder)))
    if not 0.25 <= length <= 1.2:
        raise ValueError(
            "Shoulder-to-hand reach must be 25 cm to 1.2 m; recapture shoulders and reach"
        )
    if reach[0] / length < 0.5:
        raise ValueError("Extend the arm forward comfortably from its shoulder")
    if not 0.08 <= neutral_length <= length * 0.95:
        raise ValueError("Neutral should have a bent elbow, with the hand away from the shoulder")
    scale = robot.reach_m / length
    if not 0.2 <= scale <= 3:
        raise ValueError("Arm length scale is outside 0.2-3.0; recapture shoulders and reach")
    return scale


def arm_targets(
    pose: CalibrationPose, human: ArmAnchors, robot: RobotArmGeometry
) -> tuple[CalibrationPose, CalibrationPose, bool]:
    """Return requested and reach-bounded tool poses, plus workspace saturation.

    This sphere is only an outer wrist reach envelope. Joint limits, orientation
    feasibility and posture selection remain the IK solver's responsibility.
    """
    scale = reach_scale(human, robot)
    assert human.shoulder is not None  # Validated by reach_scale.
    vector = scale * np.subtract(pose.position, human.shoulder)
    length = float(np.linalg.norm(vector))
    limited = length > robot.reach_m + 1e-8
    bounded = vector * (robot.reach_m / length) if limited else vector
    orientation = (
        Rotation.from_quat(pose.orientation)
        * Rotation.from_quat(human.neutral.orientation).inv()
        * Rotation.from_quat(robot.neutral_orientation)
    )
    offset = orientation.apply(robot.tool_to_wrist)

    def tool(v: NDArray[np.float64]) -> CalibrationPose:
        return CalibrationPose(
            position=tuple(np.asarray(robot.shoulder) + v - offset),
            orientation=tuple(orientation.as_quat()),
        )

    return tool(vector), tool(bounded), limited


def map_arm(pose: CalibrationPose, human: ArmAnchors, robot: RobotArmGeometry) -> CalibrationPose:
    return arm_targets(pose, human, robot)[1]


@dataclass(frozen=True)
class PoseSample:
    time: float
    pose: CalibrationPose


class ArmCalibration:
    """Capture/review/save is transactional; only explicit activation enables mapping."""

    def __init__(self, geometry: dict[Side, RobotArmGeometry], robot: str, directory: Path) -> None:
        if set(geometry) != set(SIDES):
            raise ValueError("Declare shoulder, wrist and tool geometry for both arms")
        self.geometry, self.robot, self.directory = geometry, robot, directory / robot
        self.history: dict[Side, deque[PoseSample]] = {side: deque(maxlen=180) for side in SIDES}
        self.profile: CalibrationProfile | None = None
        self.draft: dict[str, Any] | None = None
        self.active = False
        self.session = ""
        self.last_stamps: dict[Side, float] = dict.fromkeys(SIDES, -math.inf)

    def observe(
        self, side: Side, pose: CalibrationPose, head: NDArray[np.float64], now: float, stamp: float
    ) -> None:
        if not math.isfinite(stamp) or stamp <= self.last_stamps[side]:
            return
        self.last_stamps[side] = stamp
        self.history[side].append(PoseSample(now, relative_pose(pose, head)))

    def quality(self, side: Side, now: float) -> dict[str, Any]:
        samples = [s for s in self.history[side] if now - s.time <= CAPTURE_WINDOW_S]
        if len(samples) < 10 or samples[-1].time - samples[0].time < CAPTURE_WINDOW_S * 0.75:
            return {
                "ready": False,
                "reason": "Hold still with tracking visible",
                "samples": len(samples),
            }
        if now - samples[-1].time > INPUT_MAX_AGE_S:
            return {"ready": False, "reason": "Tracking is stale", "samples": len(samples)}
        positions = np.array([s.pose.position for s in samples])
        rotations = Rotation.from_quat([s.pose.orientation for s in samples])
        mean_rotation = rotations.mean()
        spread = float(np.max(np.linalg.norm(positions - positions.mean(axis=0), axis=1)))
        angular = float(np.max((rotations * mean_rotation.inv()).magnitude()))
        ready = spread <= MAX_POSITION_SPREAD_M and angular <= MAX_ROTATION_SPREAD_RAD
        return {
            "ready": ready,
            "reason": "Stable" if ready else "Hold the hand and head still",
            "samples": len(samples),
            "spread_m": spread,
            "rotation_spread_deg": math.degrees(angular),
        }

    def sample(self, side: Side, now: float) -> CalibrationPose:
        quality = self.quality(side, now)
        if not quality["ready"]:
            raise ValueError(f"{side}: {quality['reason']}")
        samples = [s.pose for s in self.history[side] if now - s.time <= CAPTURE_WINDOW_S]
        return CalibrationPose(
            position=tuple(np.mean([s.position for s in samples], axis=0)),
            orientation=tuple(
                Rotation.from_quat([s.orientation for s in samples]).mean().as_quat()
            ),
        )

    def begin(self, name: str, input_kind: InputKind, device: str, session: str) -> None:
        # Validate names and source before changing a running session.
        CalibrationProfile(
            name=name.strip(),
            input_kind=input_kind,
            device=device,
            robot=self.robot,
            saved_at=0,
            arms={},
        )
        if not session:
            raise ValueError("A headset session identifier is required")
        self.active, self.session = False, session
        self.draft = {
            "name": name.strip(),
            "input_kind": input_kind,
            "device": device,
            "captures": {},
        }
        self.clear_samples()

    def clear_samples(self) -> None:
        for samples in self.history.values():
            samples.clear()
        self.last_stamps = dict.fromkeys(SIDES, -math.inf)

    def capture(
        self,
        stage: Literal["shoulders", "neutral", "left_reach", "right_reach"],
        now: float,
        session: str,
    ) -> None:
        self.check_session(session)
        if self.draft is None:
            raise ValueError("Start calibration first")
        captures = self.draft["captures"]
        if stage == "shoulders":
            poses = {side: self.sample(side, now) for side in SIDES}
            left, right = (np.asarray(poses[side].position) for side in SIDES)
            if not 0.2 <= left[1] - right[1] <= 0.7 or abs(left[2] - right[2]) > 0.12:
                raise ValueError("Mark each shoulder with its own hand; keep both shoulders level")
            if any(not -0.6 <= p[2] <= -0.03 or abs(p[0]) > 0.4 for p in (left, right)):
                raise ValueError("Mark the shoulder below each side of your head")
            captures.clear()
            captures[stage] = {side: list(p.position) for side, p in poses.items()}
        elif stage == "neutral":
            if "shoulders" not in captures:
                raise ValueError("Capture shoulders first")
            neutral = {side: self.sample(side, now).model_dump() for side in SIDES}
            for dependent in ("left_reach", "right_reach"):
                captures.pop(dependent, None)
            captures[stage] = neutral
        else:
            if "neutral" not in captures:
                raise ValueError("Capture neutral first")
            if stage not in ("left_reach", "right_reach"):
                raise ValueError("Unknown capture stage")
            side: Side = "left" if stage == "left_reach" else "right"
            pose = self.sample(side, now)
            anchors = ArmAnchors(
                neutral=CalibrationPose.model_validate(captures["neutral"][side]),
                reach=pose,
                shoulder=captures["shoulders"][side],
            )
            reach_scale(anchors, self.geometry[side])
            captures[stage] = pose.model_dump()

    def candidate(self, saved_at: float) -> CalibrationProfile:
        if self.draft is None:
            raise ValueError("Start calibration first")
        d, captures = self.draft, self.draft["captures"]
        if not all(s in captures for s in ("shoulders", "neutral", "left_reach", "right_reach")):
            raise ValueError("Capture shoulders, neutral and both reaches before saving")
        return CalibrationProfile(
            name=d["name"],
            input_kind=d["input_kind"],
            device=d["device"],
            robot=self.robot,
            saved_at=saved_at,
            arms={
                side: ArmAnchors(
                    neutral=captures["neutral"][side],
                    reach=captures[f"{side}_reach"],
                    shoulder=captures["shoulders"][side],
                )
                for side in SIDES
            },
        )

    def key(self, profile: CalibrationProfile) -> str:
        return hashlib.sha256(
            json.dumps([profile.name, profile.input_kind, profile.device]).encode()
        ).hexdigest()[:24]

    def save(self, now: float, session: str) -> CalibrationProfile:
        self.check_session(session)
        profile = self.candidate(now)
        for side in SIDES:
            reach_scale(profile.arms[side], self.geometry[side])
        self.directory.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode="w", dir=self.directory, delete=False) as f:
            temporary = Path(f.name)
            f.write(profile.model_dump_json(indent=2))
        try:
            os.replace(temporary, self.directory / f"{self.key(profile)}.json")
        finally:
            temporary.unlink(missing_ok=True)
        self.profile, self.draft, self.active = profile, None, False
        return profile

    def profiles(self) -> list[CalibrationProfile]:
        return [
            CalibrationProfile.model_validate_json(p.read_text())
            for p in sorted(self.directory.glob("*.json"))
        ]

    def activate(self, key: str, input_kind: InputKind, device: str, session: str) -> None:
        profile = next((p for p in self.profiles() if self.key(p) == key), None)
        if profile is None:
            raise ValueError("Choose a saved calibration, or calibrate first")
        if (
            profile.input_kind != input_kind
            or profile.device != device
            or profile.robot != self.robot
        ):
            raise ValueError(
                "Calibration does not match this input method, device profile and robot"
            )
        if not session:
            raise ValueError("A headset session identifier is required")
        for side in SIDES:
            reach_scale(profile.arms[side], self.geometry[side])
        self.profile, self.draft, self.session, self.active = (
            profile,
            None,
            session,
            True,
        )
        self.clear_samples()

    def check_session(self, session: str) -> None:
        if not session or session != self.session:
            raise ValueError("Calibration belongs to another headset session")

    def cancel(self, session: str) -> None:
        self.check_session(session)
        self.draft, self.active = None, False

    def delete(self, key: str) -> None:
        profile = next((p for p in self.profiles() if self.key(p) == key), None)
        if profile is None:
            raise ValueError("Unknown calibration profile")
        (self.directory / f"{self.key(profile)}.json").unlink()
        self.active = False
        if self.profile is not None and self.key(self.profile) == key:
            self.profile = None

    def map(self, side: Side) -> CalibrationPose | None:
        """Map the latest hand expressed in its captured, live operator frame.

        Head and hand turning/translating together cancel before retargeting.
        Each hand keeps its paired frame, so a new right-hand sample cannot
        rotate an older left-hand sample into a different reference.
        """
        if (
            not self.active
            or self.profile is None
            or self.draft is not None
            or not self.history[side]
        ):
            return None
        return map_arm(self.history[side][-1].pose, self.profile.arms[side], self.geometry[side])

    def mapping_status(
        self, now: float, profile: CalibrationProfile | None = None
    ) -> dict[str, Any]:
        profile = profile or self.profile
        if self.draft is not None:
            captures = self.draft["captures"]
            profile = (
                self.candidate(0)
                if all(s in captures for s in ("shoulders", "neutral", "left_reach", "right_reach"))
                else None
            )
        preview: dict[str, Any] = {}
        requested: dict[str, Any] = {}
        limited: dict[str, bool] = {}
        targets: dict[str, Any] = {}
        review: dict[str, Any] = {}
        if profile is not None and profile.schema_version == 2:
            for side in SIDES:
                human, robot = profile.arms[side], self.geometry[side]
                scale = reach_scale(human, robot)
                targets[side] = {
                    name: map_arm(getattr(human, name), human, robot).model_dump()
                    for name in ("neutral", "reach")
                }
                review[side] = {
                    "scale": scale,
                    "human_reach_m": robot.reach_m / scale,
                    "robot_reach_m": robot.reach_m,
                }
                samples = self.history[side]
                if not samples or now - samples[-1].time > INPUT_MAX_AGE_S:
                    continue
                if self.active and self.profile != profile:
                    continue
                pose = samples[-1].pose
                wanted, bounded, clipped = arm_targets(pose, human, robot)
                requested[side], preview[side], limited[side] = (
                    wanted.model_dump(),
                    bounded.model_dump(),
                    clipped,
                )
        return {
            "preview": preview,
            "requested": requested,
            "limited": limited,
            "targets": targets,
            "review": review,
        }

    def status(self, now: float, profile_id: str | None = None) -> dict[str, Any]:
        profile = self.profile
        if profile_id and self.draft is None:
            profile = next((p for p in self.profiles() if self.key(p) == profile_id), profile)
        return {
            "version": 2,
            "operator_reference": "live_head_yaw",
            "robot": self.robot,
            "active": self.active,
            "session": self.session,
            "profile": self.profile.model_dump() if self.profile else None,
            "profile_id": self.key(self.profile) if self.profile else None,
            "profiles": [{"id": self.key(p), **p.model_dump()} for p in self.profiles()],
            "draft": self.draft,
            "tracking": {side: self.quality(side, now) for side in SIDES},
            "geometry": {s: g.model_dump() for s, g in self.geometry.items()},
            **self.mapping_status(now, profile),
        }
