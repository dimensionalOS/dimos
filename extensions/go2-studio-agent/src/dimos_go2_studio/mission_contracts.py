"""Strict, provider-neutral contracts for deterministic Go2 missions.

This module deliberately has no DimOS module, network, model, storage, or robot
I/O dependencies.  It defines what a mission means before P1-T2 adds an
executor that can cause physical behavior.
"""

from __future__ import annotations

from datetime import datetime, timezone
from enum import StrEnum
from typing import Annotated, Any
from uuid import uuid4

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    field_validator,
    model_validator,
)

TaskId = Annotated[
    str,
    Field(
        min_length=8,
        max_length=128,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9_.:-]*$",
    ),
]
EvidenceId = Annotated[
    str,
    Field(
        min_length=1,
        max_length=128,
        pattern=r"^[A-Za-z0-9][A-Za-z0-9_.:-]*$",
    ),
]


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _normalize_aware_datetime(value: datetime) -> datetime:
    if value.tzinfo is None or value.utcoffset() is None:
        raise ValueError("datetime must include timezone information")
    return value.astimezone(timezone.utc)


class _ContractModel(BaseModel):
    model_config = ConfigDict(
        extra="forbid",
        frozen=True,
        str_strip_whitespace=True,
        validate_default=True,
    )


class MissionKind(StrEnum):
    """Supported high-level product missions."""

    GO_TO_PLACE = "go_to_place"
    INSPECT_PLACE = "inspect_place"
    FIND_TARGET = "find_target"
    COME_TO_USER = "come_to_user"
    FOLLOW_PERSON = "follow_person"


class TaskPriority(StrEnum):
    """Initial mission priority vocabulary."""

    NORMAL = "normal"
    URGENT = "urgent"


class TaskState(StrEnum):
    """Observable lifecycle of one deterministic mission."""

    QUEUED = "queued"
    RESOLVING = "resolving"
    EXPLORING = "exploring"
    NAVIGATING = "navigating"
    RECOVERING = "recovering"
    VERIFYING = "verifying"
    FOLLOWING = "following"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"

    @property
    def is_terminal(self) -> bool:
        return self in {
            TaskState.COMPLETED,
            TaskState.FAILED,
            TaskState.CANCELLED,
        }

    @property
    def is_resumable(self) -> bool:
        return self in {
            TaskState.RESOLVING,
            TaskState.EXPLORING,
            TaskState.NAVIGATING,
            TaskState.RECOVERING,
            TaskState.VERIFYING,
            TaskState.FOLLOWING,
        }


class TaskSpec(_ContractModel):
    """Validated intent accepted by the future deterministic executor."""

    task_id: TaskId = Field(default_factory=lambda: f"task-{uuid4().hex}")
    kind: MissionKind
    destination: str | None = Field(default=None, max_length=200)
    target_description: str | None = Field(default=None, max_length=500)
    question: str | None = Field(default=None, max_length=1000)
    priority: TaskPriority = TaskPriority.NORMAL
    created_at: datetime = Field(default_factory=_utc_now)

    @field_validator("destination", "target_description", "question", mode="before")
    @classmethod
    def _blank_optional_text_is_none(cls, value: Any) -> Any:
        if isinstance(value, str):
            value = value.strip()
            return value or None
        return value

    @field_validator("created_at")
    @classmethod
    def _created_at_is_aware_utc(cls, value: datetime) -> datetime:
        return _normalize_aware_datetime(value)

    @model_validator(mode="before")
    @classmethod
    def _default_user_destination(cls, value: Any) -> Any:
        if not isinstance(value, dict):
            return value
        kind = value.get("kind")
        if kind in {MissionKind.COME_TO_USER, MissionKind.COME_TO_USER.value}:
            destination = value.get("destination")
            if destination is None or (
                isinstance(destination, str) and not destination.strip()
            ):
                return {**value, "destination": "用户身边"}
        return value

    @model_validator(mode="after")
    def _validate_kind_fields(self) -> TaskSpec:
        if self.priority is TaskPriority.URGENT and self.kind is not MissionKind.COME_TO_USER:
            raise ValueError("urgent priority is only valid for come_to_user")

        if self.kind is MissionKind.GO_TO_PLACE:
            self._require("destination", self.destination)
            self._forbid("target_description", self.target_description)
            self._forbid("question", self.question)
        elif self.kind is MissionKind.INSPECT_PLACE:
            self._require("destination", self.destination)
            self._require("question", self.question)
        elif self.kind is MissionKind.FIND_TARGET:
            self._require("target_description", self.target_description)
        elif self.kind is MissionKind.COME_TO_USER:
            if self.destination != "用户身边":
                raise ValueError("come_to_user destination must be 用户身边")
            self._forbid("target_description", self.target_description)
            self._forbid("question", self.question)
        elif self.kind is MissionKind.FOLLOW_PERSON:
            self._forbid("destination", self.destination)
            self._forbid("question", self.question)
        return self

    @staticmethod
    def _require(name: str, value: str | None) -> None:
        if value is None:
            raise ValueError(f"{name} is required for this mission kind")

    @staticmethod
    def _forbid(name: str, value: str | None) -> None:
        if value is not None:
            raise ValueError(f"{name} is not valid for this mission kind")


class TaskResult(_ContractModel):
    """Evidence-backed successful result."""

    summary: str = Field(min_length=1, max_length=2000)
    evidence_ids: tuple[EvidenceId, ...] = Field(min_length=1)

    @field_validator("evidence_ids")
    @classmethod
    def _evidence_ids_are_unique(
        cls,
        value: tuple[str, ...],
    ) -> tuple[str, ...]:
        if len(value) != len(set(value)):
            raise ValueError("evidence_ids must be unique")
        return value


class TaskSnapshot(_ContractModel):
    """One internally consistent mission lifecycle snapshot."""

    task: TaskSpec
    state: TaskState = TaskState.QUEUED
    result: TaskResult | None = None
    terminal_reason: str | None = Field(default=None, max_length=1000)
    resume_state: TaskState | None = None
    updated_at: datetime = Field(default_factory=_utc_now)

    @field_validator("terminal_reason", mode="before")
    @classmethod
    def _blank_reason_is_none(cls, value: Any) -> Any:
        if isinstance(value, str):
            value = value.strip()
            return value or None
        return value

    @field_validator("updated_at")
    @classmethod
    def _updated_at_is_aware_utc(cls, value: datetime) -> datetime:
        return _normalize_aware_datetime(value)

    @model_validator(mode="after")
    def _validate_lifecycle_payload(self) -> TaskSnapshot:
        if self.state is TaskState.PAUSED:
            if self.resume_state is None:
                raise ValueError("paused state requires resume_state")
            if not self.resume_state.is_resumable:
                raise ValueError("resume_state must be a resumable active state")
        elif self.resume_state is not None:
            raise ValueError("resume_state is only valid while paused")

        if self.state is TaskState.COMPLETED:
            if self.result is None:
                raise ValueError("completed state requires result and evidence")
            if self.terminal_reason is not None:
                raise ValueError("completed state cannot contain terminal_reason")
        elif self.state in {TaskState.FAILED, TaskState.CANCELLED}:
            if self.terminal_reason is None:
                raise ValueError("failed or cancelled state requires terminal_reason")
            if self.result is not None:
                raise ValueError("failed or cancelled state cannot contain result")
        else:
            if self.result is not None:
                raise ValueError("non-completed state cannot contain result")
            if self.terminal_reason is not None:
                raise ValueError("non-terminal state cannot contain terminal_reason")
        return self


ALLOWED_TASK_TRANSITIONS: dict[TaskState, frozenset[TaskState]] = {
    TaskState.QUEUED: frozenset(
        {
            TaskState.RESOLVING,
            TaskState.FAILED,
            TaskState.CANCELLED,
        }
    ),
    TaskState.RESOLVING: frozenset(
        {
            TaskState.EXPLORING,
            TaskState.NAVIGATING,
            TaskState.VERIFYING,
            TaskState.FOLLOWING,
            TaskState.PAUSED,
            TaskState.FAILED,
            TaskState.CANCELLED,
        }
    ),
    TaskState.EXPLORING: frozenset(
        {
            TaskState.NAVIGATING,
            TaskState.RECOVERING,
            TaskState.VERIFYING,
            TaskState.PAUSED,
            TaskState.COMPLETED,
            TaskState.FAILED,
            TaskState.CANCELLED,
        }
    ),
    TaskState.NAVIGATING: frozenset(
        {
            TaskState.RECOVERING,
            TaskState.VERIFYING,
            TaskState.PAUSED,
            TaskState.COMPLETED,
            TaskState.FAILED,
            TaskState.CANCELLED,
        }
    ),
    TaskState.RECOVERING: frozenset(
        {
            TaskState.EXPLORING,
            TaskState.NAVIGATING,
            TaskState.FOLLOWING,
            TaskState.PAUSED,
            TaskState.FAILED,
            TaskState.CANCELLED,
        }
    ),
    TaskState.VERIFYING: frozenset(
        {
            TaskState.EXPLORING,
            TaskState.NAVIGATING,
            TaskState.PAUSED,
            TaskState.COMPLETED,
            TaskState.FAILED,
            TaskState.CANCELLED,
        }
    ),
    TaskState.FOLLOWING: frozenset(
        {
            TaskState.RECOVERING,
            TaskState.PAUSED,
            TaskState.COMPLETED,
            TaskState.FAILED,
            TaskState.CANCELLED,
        }
    ),
    TaskState.PAUSED: frozenset(),
    TaskState.COMPLETED: frozenset(),
    TaskState.FAILED: frozenset(),
    TaskState.CANCELLED: frozenset(),
}


def validate_task_transition(
    current: TaskState,
    target: TaskState,
    *,
    resume_state: TaskState | None = None,
) -> TaskState:
    """Validate one lifecycle transition and return the normalized target."""

    current = TaskState(current)
    target = TaskState(target)
    normalized_resume = TaskState(resume_state) if resume_state is not None else None

    if current.is_terminal:
        raise ValueError(f"{current.value} is terminal and cannot transition")

    if current is TaskState.PAUSED:
        if target in {TaskState.FAILED, TaskState.CANCELLED}:
            return target
        if normalized_resume is None:
            raise ValueError("resume_state is required to resume a paused task")
        if not normalized_resume.is_resumable:
            raise ValueError("resume_state must be resumable")
        if target is not normalized_resume:
            raise ValueError(
                f"resume target must match resume_state {normalized_resume.value}"
            )
        return target

    if normalized_resume is not None:
        raise ValueError("resume_state is only valid when current state is paused")

    if target not in ALLOWED_TASK_TRANSITIONS[current]:
        raise ValueError(
            f"Invalid task transition: {current.value} -> {target.value}"
        )
    return target
