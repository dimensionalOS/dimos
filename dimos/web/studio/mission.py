# ruff: noqa: RUF001
"""Deterministic mission state and safety policy for the local Go2 Studio.

This module deliberately contains no robot I/O. It decides whether a requested
mission may progress; existing DimOS modules remain responsible for perception,
planning, and movement.
"""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
from enum import StrEnum
from uuid import uuid4

from pydantic import BaseModel, Field


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


class MissionState(StrEnum):
    """Operator-visible lifecycle for one autonomous mission."""

    IDLE = "idle"
    DRAFT = "draft"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    STOPPED = "stopped"
    FAILED = "failed"


class MissionPolicy(BaseModel):
    """Conservative limits for the first real-world Go2 mission release."""

    max_speed_mps: float = Field(default=0.1, gt=0, le=0.1)
    person_safety_radius_m: float = Field(default=1.5, ge=1.5)
    minimum_passage_width_m: float = Field(default=1.5, ge=1.5)
    clear_before_resume_s: float = Field(default=3.0, ge=3.0)
    door_standoff_m: float = Field(default=1.0, ge=1.0)
    exploration_radius_m: float = Field(default=5.0, gt=0, le=5.0)
    mission_timeout_s: int = Field(default=300, gt=0, le=300)
    minimum_battery_percent: float = Field(default=30.0, ge=30.0)
    maximum_perception_age_s: float = Field(default=1.0, gt=0, le=1.0)
    maximum_navigation_failures: int = Field(default=3, ge=1, le=3)
    allow_reverse: bool = False


class SafetySnapshot(BaseModel):
    """Latest facts supplied by perception/runtime monitoring."""

    link_ok: bool = True
    battery_percent: float | None = None
    person_distance_m: float | None = None
    perception_age_s: float = 0.0
    manual_takeover: bool = False


class MissionEvent(BaseModel):
    """One concise mission transition for the operator event log."""

    at: datetime = Field(default_factory=_utc_now)
    state: MissionState
    message: str


class MissionRecord(BaseModel):
    """Serializable current mission state."""

    id: str = Field(default_factory=lambda: uuid4().hex)
    objective: str
    state: MissionState = MissionState.DRAFT
    phase: str = "等待安全确认"
    safety_reason: str = ""
    policy: MissionPolicy = Field(default_factory=MissionPolicy)
    created_at: datetime = Field(default_factory=_utc_now)
    updated_at: datetime = Field(default_factory=_utc_now)
    started_at: datetime | None = None
    clear_since: datetime | None = None
    navigation_failures: int = 0
    events: list[MissionEvent] = Field(default_factory=list)


class MissionController:
    """Applies mission transitions and rejects unsafe state changes."""

    def __init__(
        self,
        mission: MissionRecord | None = None,
        *,
        policy: MissionPolicy | None = None,
    ) -> None:
        self.mission = mission
        self.policy = policy or (mission.policy if mission else MissionPolicy())

    def create(self, objective: str, *, now: datetime | None = None) -> MissionRecord:
        text = objective.strip()
        if not text:
            raise ValueError("任务目标不能为空")
        timestamp = now or _utc_now()
        self.mission = MissionRecord(
            objective=text,
            policy=self.policy,
            created_at=timestamp,
            updated_at=timestamp,
            events=[
                MissionEvent(
                    at=timestamp,
                    state=MissionState.DRAFT,
                    message="任务已创建，等待安全确认",
                )
            ],
        )
        return self.mission

    def start(
        self,
        *,
        runtime_running: bool,
        movement_locked: bool,
        now: datetime | None = None,
    ) -> MissionRecord:
        mission = self._require_mission()
        if movement_locked:
            raise ValueError("运动锁仍然开启，不能启动自主任务")
        if not runtime_running:
            raise ValueError("DimOS 尚未运行，不能启动任务")
        if mission.state not in {MissionState.DRAFT, MissionState.PAUSED}:
            raise ValueError(f"当前任务状态 {mission.state.value} 不能启动")
        timestamp = now or _utc_now()
        mission.state = MissionState.RUNNING
        mission.phase = "观察环境并探索"
        mission.safety_reason = ""
        mission.started_at = mission.started_at or timestamp
        mission.clear_since = None
        self._event("安全闸门通过，任务开始", timestamp)
        return mission

    def observe(
        self,
        snapshot: SafetySnapshot,
        *,
        observed_at: datetime | None = None,
    ) -> MissionRecord:
        mission = self._require_mission()
        timestamp = observed_at or _utc_now()
        reason = self._unsafe_reason(snapshot)
        if reason:
            mission.clear_since = None
            if mission.state in {MissionState.RUNNING, MissionState.PAUSED}:
                mission.state = MissionState.PAUSED
                mission.phase = "安全暂停"
                if mission.safety_reason != reason:
                    mission.safety_reason = reason
                    self._event(reason, timestamp)
            return mission

        if mission.state is MissionState.PAUSED:
            if mission.clear_since is None:
                mission.clear_since = timestamp
                mission.updated_at = timestamp
                return mission
            required_clear = timedelta(seconds=mission.policy.clear_before_resume_s)
            if timestamp - mission.clear_since >= required_clear:
                mission.state = MissionState.RUNNING
                mission.phase = "观察环境并探索"
                mission.safety_reason = ""
                mission.clear_since = None
                self._event("环境持续安全，任务自动恢复", timestamp)
        return mission

    def pause(self, reason: str = "操作员暂停", *, now: datetime | None = None) -> MissionRecord:
        mission = self._require_mission()
        if mission.state is not MissionState.RUNNING:
            raise ValueError("只有运行中的任务可以暂停")
        mission.state = MissionState.PAUSED
        mission.phase = "操作员暂停"
        mission.safety_reason = reason
        mission.clear_since = None
        self._event(reason, now or _utc_now())
        return mission

    def resume(
        self,
        *,
        runtime_running: bool,
        movement_locked: bool,
        safety: SafetySnapshot | None = None,
        now: datetime | None = None,
    ) -> MissionRecord:
        mission = self._require_mission()
        if mission.state is not MissionState.PAUSED:
            raise ValueError("只有暂停中的任务可以恢复")
        reason = self._unsafe_reason(safety or SafetySnapshot())
        if reason:
            raise ValueError(reason)
        return self.start(
            runtime_running=runtime_running,
            movement_locked=movement_locked,
            now=now,
        )

    def complete(self, summary: str, *, now: datetime | None = None) -> MissionRecord:
        mission = self._require_mission()
        if mission.state not in {MissionState.RUNNING, MissionState.PAUSED}:
            raise ValueError("当前任务不能标记为完成")
        mission.state = MissionState.COMPLETED
        mission.phase = "任务完成"
        mission.safety_reason = ""
        self._event(summary or "任务完成", now or _utc_now())
        return mission

    def stop(self, reason: str = "操作员停止", *, now: datetime | None = None) -> MissionRecord:
        mission = self._require_mission()
        mission.state = MissionState.STOPPED
        mission.phase = "已停止"
        mission.safety_reason = reason
        mission.clear_since = None
        self._event(reason, now or _utc_now())
        return mission

    def emergency_stop(
        self,
        reason: str = "操作员触发急停",
        *,
        now: datetime | None = None,
    ) -> MissionRecord:
        mission = self._require_mission()
        mission.state = MissionState.STOPPED
        mission.phase = "急停"
        mission.safety_reason = reason
        mission.clear_since = None
        self._event(f"急停：{reason}", now or _utc_now())
        return mission

    def record_navigation_failure(
        self,
        reason: str,
        *,
        now: datetime | None = None,
    ) -> MissionRecord:
        mission = self._require_mission()
        mission.navigation_failures += 1
        timestamp = now or _utc_now()
        if mission.navigation_failures >= mission.policy.maximum_navigation_failures:
            mission.state = MissionState.FAILED
            mission.phase = "任务失败"
            mission.safety_reason = f"连续导航失败 {mission.navigation_failures} 次"
            self._event(f"{mission.safety_reason}：{reason}", timestamp)
        else:
            self._event(
                f"导航失败 {mission.navigation_failures} 次：{reason}",
                timestamp,
            )
        return mission

    def _unsafe_reason(self, snapshot: SafetySnapshot) -> str:
        if snapshot.manual_takeover:
            return "检测到人工接管"
        if not snapshot.link_ok:
            return "机器人连接中断"
        if (
            snapshot.battery_percent is not None
            and snapshot.battery_percent < self.policy.minimum_battery_percent
        ):
            return (
                f"电量 {snapshot.battery_percent:.0f}% 低于"
                f" {self.policy.minimum_battery_percent:.0f}%"
            )
        if snapshot.perception_age_s > self.policy.maximum_perception_age_s:
            return f"感知数据已过期 {snapshot.perception_age_s:.1f} 秒"
        if (
            snapshot.person_distance_m is not None
            and snapshot.person_distance_m < self.policy.person_safety_radius_m
        ):
            return (
                f"检测到人员距离 {snapshot.person_distance_m:.2f} m，"
                f"小于安全半径 {self.policy.person_safety_radius_m:.2f} m"
            )
        return ""

    def _event(self, message: str, timestamp: datetime) -> None:
        mission = self._require_mission()
        mission.updated_at = timestamp
        mission.events.append(
            MissionEvent(at=timestamp, state=mission.state, message=message)
        )

    def _require_mission(self) -> MissionRecord:
        if self.mission is None:
            raise ValueError("尚未创建任务")
        return self.mission
