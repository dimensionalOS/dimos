# ruff: noqa: RUF001
"""Tests for the deterministic Go2 mission and safety state machine."""

from datetime import datetime, timedelta, timezone

import pytest

from dimos.web.studio.mission import (
    MissionController,
    MissionState,
    SafetySnapshot,
)
from dimos.web.studio.models import (
    MissionActionRequest,
    MissionCreateRequest,
)


def test_new_mission_starts_as_draft() -> None:
    controller = MissionController()

    mission = controller.create("探索会场，找到门并在门前一米停下")

    assert mission.state is MissionState.DRAFT
    assert mission.phase == "等待安全确认"
    assert mission.policy.max_speed_mps == 0.1
    assert mission.policy.door_standoff_m == 1.0


def test_movement_locked_mission_cannot_start() -> None:
    controller = MissionController()
    controller.create("找到门")

    with pytest.raises(ValueError, match="运动锁"):
        controller.start(runtime_running=True, movement_locked=True)


def test_mission_requires_running_runtime() -> None:
    controller = MissionController()
    controller.create("找到门")

    with pytest.raises(ValueError, match="DimOS"):
        controller.start(runtime_running=False, movement_locked=False)


def test_person_inside_safety_radius_pauses_mission() -> None:
    controller = MissionController()
    controller.create("找到门")
    controller.start(runtime_running=True, movement_locked=False)

    mission = controller.observe(
        SafetySnapshot(person_distance_m=1.2),
        observed_at=datetime(2026, 7, 24, 12, 0, tzinfo=timezone.utc),
    )

    assert mission.state is MissionState.PAUSED
    assert mission.safety_reason == "检测到人员距离 1.20 m，小于安全半径 1.50 m"


def test_mission_resumes_only_after_three_clear_seconds() -> None:
    controller = MissionController()
    controller.create("找到门")
    controller.start(runtime_running=True, movement_locked=False)
    started = datetime(2026, 7, 24, 12, 0, tzinfo=timezone.utc)
    controller.observe(SafetySnapshot(person_distance_m=1.0), observed_at=started)

    mission = controller.observe(
        SafetySnapshot(person_distance_m=None),
        observed_at=started + timedelta(seconds=2.9),
    )
    assert mission.state is MissionState.PAUSED

    mission = controller.observe(
        SafetySnapshot(person_distance_m=None),
        observed_at=started + timedelta(seconds=5.9),
    )
    assert mission.state is MissionState.RUNNING
    assert mission.safety_reason == ""


def test_emergency_stop_always_stops() -> None:
    controller = MissionController()
    controller.create("找到门")

    mission = controller.emergency_stop("operator pressed E-STOP")

    assert mission.state is MissionState.STOPPED
    assert mission.phase == "急停"
    assert mission.safety_reason == "operator pressed E-STOP"


def test_three_navigation_failures_fail_mission() -> None:
    controller = MissionController()
    controller.create("找到门")
    controller.start(runtime_running=True, movement_locked=False)

    controller.record_navigation_failure("path blocked")
    controller.record_navigation_failure("path blocked")
    mission = controller.record_navigation_failure("path blocked")

    assert mission.state is MissionState.FAILED
    assert mission.navigation_failures == 3
    assert mission.safety_reason == "连续导航失败 3 次"


def test_mission_request_dto_trims_operator_text() -> None:
    request = MissionCreateRequest(objective="  去门口  ")
    action = MissionActionRequest(reason="  操作员取消  ")

    assert request.objective == "去门口"
    assert action.reason == "操作员取消"


def test_mission_request_dto_rejects_blank_or_unknown_fields() -> None:
    with pytest.raises(ValueError):
        MissionCreateRequest(objective="   ")

    with pytest.raises(ValueError):
        MissionCreateRequest(objective="去门口", movement_speed=1.0)
