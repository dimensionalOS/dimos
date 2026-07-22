import pytest

from dimos.manipulation.execution_models import ActionMethod, Outcome, TaskActivity
from dimos.manipulation.execution_policy import (
    reconcile_cancel_completion,
    reconcile_execute_completion,
    reconcile_reset_completion,
    reconcile_status_completion,
)


@pytest.mark.parametrize(
    ("helper", "outcome", "activity"),
    [
        (reconcile_execute_completion, Outcome.ACCEPTED, TaskActivity.ACTIVE),
        (reconcile_cancel_completion, Outcome.CANCELLED, TaskActivity.CANCELLED),
        (reconcile_status_completion, Outcome.COMPLETED, TaskActivity.COMPLETED),
    ],
)
def test_completion_policy_normalizes_public_outcomes(
    helper: object, outcome: Outcome, activity: TaskActivity
) -> None:
    decision = helper(outcome)  # type: ignore[operator]
    assert decision.activity == activity
    assert not decision.fault


@pytest.mark.parametrize(
    (
        "method",
        "outcome",
        "activity",
        "expected_activity",
        "cancel_required",
        "request_cancel",
        "reset_success",
    ),
    [
        (
            ActionMethod.STATUS,
            Outcome.INACTIVE,
            TaskActivity.ACTIVE,
            TaskActivity.INACTIVE,
            False,
            False,
            None,
        ),
        (
            ActionMethod.STATUS,
            Outcome.CANCELLED,
            TaskActivity.ACTIVE,
            TaskActivity.CANCELLED,
            False,
            False,
            None,
        ),
        (
            ActionMethod.STATUS,
            Outcome.COMPLETED,
            TaskActivity.ACTIVE,
            TaskActivity.COMPLETED,
            False,
            False,
            None,
        ),
        (
            ActionMethod.STATUS,
            Outcome.RUNNING,
            TaskActivity.COMPLETED,
            TaskActivity.ACTIVE,
            True,
            True,
            None,
        ),
        (
            ActionMethod.STATUS,
            Outcome.ACCEPTED,
            TaskActivity.INACTIVE,
            TaskActivity.ACTIVE,
            True,
            True,
            None,
        ),
        (
            ActionMethod.STATUS,
            Outcome.UNKNOWN,
            TaskActivity.ACTIVE,
            TaskActivity.UNKNOWN,
            True,
            True,
            False,
        ),
        (ActionMethod.CANCEL, Outcome.UNKNOWN, TaskActivity.ACTIVE, None, False, False, False),
        (ActionMethod.RESET, Outcome.FAILED, TaskActivity.ACTIVE, None, None, False, False),
    ],
)
def test_reset_completion_policy_is_pure_and_classifies_safety(
    method: ActionMethod,
    outcome: Outcome,
    activity: TaskActivity,
    expected_activity: TaskActivity | None,
    cancel_required: bool | None,
    request_cancel: bool,
    reset_success: bool | None,
) -> None:
    decision = reconcile_reset_completion(
        method=method,
        outcome=outcome,
        activity=activity,
        gate_sealed=True,
        clock_failed=True,
    )
    assert decision.cancel_required == cancel_required
    assert decision.request_cancel is request_cancel
    assert decision.activity == expected_activity
    assert decision.reset_success == reset_success
