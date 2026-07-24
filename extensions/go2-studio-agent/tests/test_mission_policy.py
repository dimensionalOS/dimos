"""Tests for evidence-based, effect-free door mission policy helpers."""

from datetime import datetime, timedelta, timezone

from dimos_go2_studio.mission_policy import (
    DoorMissionPolicy,
    DoorObservation,
)
import pytest


def observation(
    frame_id: str,
    confidence: float,
    *,
    seconds: float,
    x_m: float = 4.0,
    y_m: float = 0.0,
) -> DoorObservation:
    return DoorObservation(
        candidate_id="door-a",
        frame_id=frame_id,
        confidence=confidence,
        x_m=x_m,
        y_m=y_m,
        observed_at=datetime(2026, 7, 24, tzinfo=timezone.utc)
        + timedelta(seconds=seconds),
    )


def test_door_requires_three_independent_observations() -> None:
    policy = DoorMissionPolicy()

    first = policy.record(observation("frame-1", 0.9, seconds=0))
    duplicate = policy.record(observation("frame-1", 0.95, seconds=1))
    second = policy.record(observation("frame-2", 0.85, seconds=2))

    assert first.verified is False
    assert duplicate.evidence_count == 1
    assert second.verified is False

    third = policy.record(observation("frame-3", 0.88, seconds=4))

    assert third.verified is True
    assert third.action == "approach"


def test_low_confidence_candidate_is_rejected() -> None:
    policy = DoorMissionPolicy()

    for index, confidence in enumerate((0.9, 0.45, 0.88)):
        decision = policy.record(
            observation(f"frame-{index}", confidence, seconds=index * 2)
        )

    assert decision.verified is False
    assert decision.action == "observe"
    assert decision.evidence_count == 2


def test_approach_goal_stops_one_meter_from_door() -> None:
    policy = DoorMissionPolicy()

    goal = policy.approach_goal(
        robot_x_m=0.0,
        robot_y_m=0.0,
        door_x_m=4.0,
        door_y_m=0.0,
    )

    assert goal.should_move is True
    assert goal.x_m == pytest.approx(3.0)
    assert goal.y_m == pytest.approx(0.0)
    assert goal.standoff_m == 1.0


def test_person_presence_pauses_and_stale_perception_stops() -> None:
    policy = DoorMissionPolicy()

    person = policy.next_safe_action(
        agent_action="approach",
        movement_locked=False,
        person_distance_m=1.2,
        perception_age_s=0.1,
    )
    stale = policy.next_safe_action(
        agent_action="approach",
        movement_locked=False,
        person_distance_m=None,
        perception_age_s=1.2,
    )

    assert person.action == "pause"
    assert "1.50 m" in person.reason
    assert stale.action == "stop"
    assert "过期" in stale.reason


def test_agent_advice_cannot_unlock_movement() -> None:
    policy = DoorMissionPolicy()

    decision = policy.next_safe_action(
        agent_action="approach",
        movement_locked=True,
        person_distance_m=None,
        perception_age_s=0.0,
    )

    assert decision.action == "stop"
    assert decision.movement_allowed is False
    assert "运动锁" in decision.reason
