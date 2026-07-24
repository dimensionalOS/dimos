"""Pure evidence and safety policy for observation-first door missions."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from math import hypot
from typing import Literal


@dataclass(frozen=True)
class DoorObservation:
    """One model observation associated with one camera frame."""

    candidate_id: str
    frame_id: str
    confidence: float
    x_m: float
    y_m: float
    observed_at: datetime

    def __post_init__(self) -> None:
        if not self.candidate_id.strip():
            raise ValueError("candidate_id cannot be empty")
        if not self.frame_id.strip():
            raise ValueError("frame_id cannot be empty")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be between 0 and 1")


@dataclass(frozen=True)
class DoorEvidenceDecision:
    """Current evidence summary for one door candidate."""

    candidate_id: str
    evidence_count: int
    mean_confidence: float
    verified: bool
    action: Literal["observe", "approach"]
    reason: str


@dataclass(frozen=True)
class ApproachGoal:
    """Goal point that preserves a stand-off distance from the door."""

    x_m: float
    y_m: float
    standoff_m: float
    should_move: bool


@dataclass(frozen=True)
class SafeActionDecision:
    """Safety-filtered interpretation of an Agent recommendation."""

    action: Literal["observe", "approach", "pause", "stop"]
    movement_allowed: bool
    reason: str


class DoorMissionPolicy:
    """Accumulates independent evidence and filters Agent recommendations."""

    def __init__(
        self,
        *,
        required_observations: int = 3,
        minimum_observation_confidence: float = 0.65,
        minimum_mean_confidence: float = 0.75,
        maximum_candidate_spread_m: float = 0.75,
        door_standoff_m: float = 1.0,
        person_safety_radius_m: float = 1.5,
        maximum_perception_age_s: float = 1.0,
    ) -> None:
        self.required_observations = required_observations
        self.minimum_observation_confidence = minimum_observation_confidence
        self.minimum_mean_confidence = minimum_mean_confidence
        self.maximum_candidate_spread_m = maximum_candidate_spread_m
        self.door_standoff_m = door_standoff_m
        self.person_safety_radius_m = person_safety_radius_m
        self.maximum_perception_age_s = maximum_perception_age_s
        self._evidence: dict[str, dict[str, DoorObservation]] = {}

    def record(self, observation: DoorObservation) -> DoorEvidenceDecision:
        frames = self._evidence.setdefault(observation.candidate_id, {})
        if observation.confidence >= self.minimum_observation_confidence:
            frames.setdefault(observation.frame_id, observation)

        evidence = list(frames.values())
        count = len(evidence)
        mean_confidence = (
            sum(item.confidence for item in evidence) / count if count else 0.0
        )
        spatially_consistent = self._is_spatially_consistent(evidence)
        verified = (
            count >= self.required_observations
            and mean_confidence >= self.minimum_mean_confidence
            and spatially_consistent
        )
        if verified:
            reason = (
                f"Door verified from {count} independent frames "
                f"with mean confidence {mean_confidence:.2f}"
            )
        elif not spatially_consistent:
            reason = "Door observations disagree on position; continue observing"
        else:
            reason = (
                f"Need {self.required_observations} independent high-confidence frames; "
                f"have {count}"
            )
        return DoorEvidenceDecision(
            candidate_id=observation.candidate_id,
            evidence_count=count,
            mean_confidence=mean_confidence,
            verified=verified,
            action="approach" if verified else "observe",
            reason=reason,
        )

    def approach_goal(
        self,
        *,
        robot_x_m: float,
        robot_y_m: float,
        door_x_m: float,
        door_y_m: float,
    ) -> ApproachGoal:
        dx = door_x_m - robot_x_m
        dy = door_y_m - robot_y_m
        distance = hypot(dx, dy)
        if distance <= self.door_standoff_m or distance == 0:
            return ApproachGoal(
                x_m=robot_x_m,
                y_m=robot_y_m,
                standoff_m=self.door_standoff_m,
                should_move=False,
            )
        scale = (distance - self.door_standoff_m) / distance
        return ApproachGoal(
            x_m=robot_x_m + dx * scale,
            y_m=robot_y_m + dy * scale,
            standoff_m=self.door_standoff_m,
            should_move=True,
        )

    def next_safe_action(
        self,
        *,
        agent_action: str,
        movement_locked: bool,
        person_distance_m: float | None,
        perception_age_s: float,
    ) -> SafeActionDecision:
        """Filter model advice without ever allowing it to change the lock."""

        if movement_locked:
            return SafeActionDecision("stop", False, "运动锁开启; Agent 无权解锁")
        if perception_age_s > self.maximum_perception_age_s:
            return SafeActionDecision(
                "stop",
                False,
                f"感知数据已过期 {perception_age_s:.1f} 秒",
            )
        if (
            person_distance_m is not None
            and person_distance_m < self.person_safety_radius_m
        ):
            return SafeActionDecision(
                "pause",
                False,
                (
                    f"人员位于 {person_distance_m:.2f} m; "
                    f"安全半径为 {self.person_safety_radius_m:.2f} m"
                ),
            )
        if agent_action == "approach":
            return SafeActionDecision("approach", True, "安全闸门允许接近")
        if agent_action == "observe":
            return SafeActionDecision("observe", False, "继续收集门的独立证据")
        return SafeActionDecision("stop", False, "未知 Agent 动作; 拒绝执行")

    def _is_spatially_consistent(self, evidence: list[DoorObservation]) -> bool:
        if len(evidence) < 2:
            return True
        mean_x = sum(item.x_m for item in evidence) / len(evidence)
        mean_y = sum(item.y_m for item in evidence) / len(evidence)
        return all(
            hypot(item.x_m - mean_x, item.y_m - mean_y)
            <= self.maximum_candidate_spread_m
            for item in evidence
        )
