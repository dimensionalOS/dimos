"""Contract tests for deterministic Go2 missions."""

from datetime import datetime, timedelta, timezone

from dimos_go2_studio.mission_contracts import (
    MissionKind,
    TaskPriority,
    TaskResult,
    TaskSnapshot,
    TaskSpec,
    TaskState,
    validate_task_transition,
)
from pydantic import ValidationError
import pytest


def _go_to_place(**overrides: object) -> TaskSpec:
    values: dict[str, object] = {
        "kind": MissionKind.GO_TO_PLACE,
        "destination": "门口",
    }
    values.update(overrides)
    return TaskSpec(**values)


def test_task_spec_is_strict_normalized_and_serializable() -> None:
    created_at = datetime(
        2026,
        7,
        25,
        14,
        30,
        tzinfo=timezone(timedelta(hours=8)),
    )

    task = _go_to_place(
        task_id=" task-0001 ",
        destination=" 门口 ",
        created_at=created_at,
    )

    assert task.task_id == "task-0001"
    assert task.destination == "门口"
    assert task.created_at == datetime(2026, 7, 25, 6, 30, tzinfo=timezone.utc)
    assert TaskSpec.model_validate(task.model_dump()).task_id == task.task_id

    with pytest.raises(ValidationError, match="extra_forbidden"):
        _go_to_place(unexpected=True)


def test_task_spec_rejects_naive_timestamp_and_invalid_id() -> None:
    with pytest.raises(ValidationError, match="timezone"):
        _go_to_place(created_at=datetime(2026, 7, 25, 6, 30))

    with pytest.raises(ValidationError):
        _go_to_place(task_id="bad id")


@pytest.mark.parametrize(
    ("values", "expected_kind"),
    [
        (
            {"kind": "go_to_place", "destination": "门口"},
            MissionKind.GO_TO_PLACE,
        ),
        (
            {
                "kind": "inspect_place",
                "destination": "厨房",
                "question": "有没有水瓶?",
            },
            MissionKind.INSPECT_PLACE,
        ),
        (
            {
                "kind": "find_target",
                "destination": "客厅",
                "target_description": "红色水瓶",
            },
            MissionKind.FIND_TARGET,
        ),
        (
            {"kind": "come_to_user", "priority": "urgent"},
            MissionKind.COME_TO_USER,
        ),
        (
            {"kind": "follow_person", "target_description": "当前用户"},
            MissionKind.FOLLOW_PERSON,
        ),
    ],
)
def test_supported_mission_kinds_are_valid(
    values: dict[str, object],
    expected_kind: MissionKind,
) -> None:
    task = TaskSpec(**values)

    assert task.kind is expected_kind
    if expected_kind is MissionKind.COME_TO_USER:
        assert task.destination == "用户身边"


@pytest.mark.parametrize(
    "values",
    [
        {"kind": "go_to_place"},
        {
            "kind": "go_to_place",
            "destination": "门口",
            "question": "这是门吗?",
        },
        {"kind": "inspect_place", "destination": "厨房"},
        {"kind": "find_target", "destination": "客厅"},
        {"kind": "come_to_user", "destination": "厨房"},
        {"kind": "follow_person", "destination": "门口"},
        {
            "kind": "go_to_place",
            "destination": "门口",
            "priority": "urgent",
        },
    ],
)
def test_invalid_mission_field_combinations_are_rejected(
    values: dict[str, object],
) -> None:
    with pytest.raises(ValidationError):
        TaskSpec(**values)


def test_task_state_transition_contract_is_bounded() -> None:
    assert (
        validate_task_transition(TaskState.QUEUED, TaskState.RESOLVING)
        is TaskState.RESOLVING
    )
    assert (
        validate_task_transition(TaskState.NAVIGATING, TaskState.PAUSED)
        is TaskState.PAUSED
    )
    assert (
        validate_task_transition(
            TaskState.PAUSED,
            TaskState.NAVIGATING,
            resume_state=TaskState.NAVIGATING,
        )
        is TaskState.NAVIGATING
    )
    assert (
        validate_task_transition(TaskState.PAUSED, TaskState.CANCELLED)
        is TaskState.CANCELLED
    )

    with pytest.raises(ValueError, match="terminal"):
        validate_task_transition(TaskState.COMPLETED, TaskState.RESOLVING)

    with pytest.raises(ValueError, match="resume"):
        validate_task_transition(
            TaskState.PAUSED,
            TaskState.VERIFYING,
            resume_state=TaskState.NAVIGATING,
        )

    with pytest.raises(ValueError, match="Invalid task transition"):
        validate_task_transition(TaskState.QUEUED, TaskState.COMPLETED)


def test_paused_snapshot_requires_resumable_previous_state() -> None:
    task = _go_to_place()

    paused = TaskSnapshot(
        task=task,
        state=TaskState.PAUSED,
        resume_state=TaskState.NAVIGATING,
    )
    assert paused.resume_state is TaskState.NAVIGATING

    with pytest.raises(ValidationError, match="resume_state"):
        TaskSnapshot(task=task, state=TaskState.PAUSED)

    with pytest.raises(ValidationError, match="resumable"):
        TaskSnapshot(
            task=task,
            state=TaskState.PAUSED,
            resume_state=TaskState.COMPLETED,
        )


def test_completed_snapshot_requires_result_and_evidence() -> None:
    task = _go_to_place()

    with pytest.raises(ValidationError, match="result"):
        TaskSnapshot(task=task, state=TaskState.COMPLETED)

    with pytest.raises(ValidationError):
        TaskResult(summary="到达门口", evidence_ids=())

    snapshot = TaskSnapshot(
        task=task,
        state=TaskState.COMPLETED,
        result=TaskResult(
            summary="已到达门口",
            evidence_ids=("arrival-pose-001",),
        ),
    )
    assert snapshot.result is not None
    assert snapshot.result.evidence_ids == ("arrival-pose-001",)


@pytest.mark.parametrize("state", [TaskState.FAILED, TaskState.CANCELLED])
def test_failed_and_cancelled_snapshots_require_reason(state: TaskState) -> None:
    task = _go_to_place()

    with pytest.raises(ValidationError, match="terminal_reason"):
        TaskSnapshot(task=task, state=state)

    snapshot = TaskSnapshot(
        task=task,
        state=state,
        terminal_reason="操作员取消" if state is TaskState.CANCELLED else "导航超时",
    )
    assert snapshot.terminal_reason


def test_non_terminal_snapshot_rejects_terminal_payload() -> None:
    task = _go_to_place()

    with pytest.raises(ValidationError, match="terminal"):
        TaskSnapshot(
            task=task,
            state=TaskState.NAVIGATING,
            terminal_reason="不应存在",
        )

    with pytest.raises(ValidationError, match="result"):
        TaskSnapshot(
            task=task,
            state=TaskState.NAVIGATING,
            result=TaskResult(
                summary="不应提前完成",
                evidence_ids=("frame-1",),
            ),
        )


def test_contract_enums_are_stable_strings() -> None:
    assert MissionKind.FIND_TARGET.value == "find_target"
    assert TaskPriority.URGENT.value == "urgent"
    assert TaskState.RECOVERING.value == "recovering"
    assert TaskState.PAUSED.value == "paused"
