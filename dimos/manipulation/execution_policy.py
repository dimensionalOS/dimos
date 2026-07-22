"""Pure policy decisions for execution-runtime reconciliation."""

from dataclasses import dataclass

from dimos.manipulation.execution_models import (
    ActionMethod,
    Outcome,
    TaskActivity,
)


@dataclass(frozen=True)
class ResetCompletionDecision:
    """Passive result of reconciling one reset-related coordinator call."""

    activity: TaskActivity | None = None
    cancel_required: bool | None = None
    reset_required: bool | None = None
    prove_inactive: bool = False
    complete_reset: bool = False
    advance_reset: bool = False
    reset_success: bool | None = None
    request_cancel: bool = False
    emergency_cancel: bool = False
    diagnostic: str | None = None


@dataclass(frozen=True)
class CompletionDecision:
    """Pure normalized decision for a non-reset coordinator completion."""

    activity: TaskActivity | None = None
    fault: bool = False
    rejected: bool = False
    accepted: bool = False
    diagnostic: str | None = None


_SAFE = frozenset((TaskActivity.INACTIVE, TaskActivity.CANCELLED, TaskActivity.COMPLETED))


def reconcile_execute_completion(outcome: Outcome) -> CompletionDecision:
    if outcome == Outcome.ACCEPTED:
        return CompletionDecision(activity=TaskActivity.ACTIVE, accepted=True)
    if outcome == Outcome.REJECTED:
        return CompletionDecision(activity=TaskActivity.INACTIVE, rejected=True)
    return CompletionDecision(fault=True, diagnostic=f"execute outcome={outcome.value}")


def reconcile_cancel_completion(outcome: Outcome) -> CompletionDecision:
    if outcome in (Outcome.CANCELLED, Outcome.INACTIVE, Outcome.ACCEPTED):
        return CompletionDecision(activity=TaskActivity.CANCELLED)
    return CompletionDecision(
        fault=True,
        diagnostic=(
            "cancellation is uncertain"
            if outcome == Outcome.UNKNOWN
            else "cancellation failed"
            if outcome == Outcome.FAILED
            else "malformed cancel outcome"
        ),
    )


def reconcile_status_completion(outcome: Outcome) -> CompletionDecision:
    activity = {
        Outcome.COMPLETED: TaskActivity.COMPLETED,
        Outcome.INACTIVE: TaskActivity.INACTIVE,
        Outcome.CANCELLED: TaskActivity.CANCELLED,
        Outcome.ACCEPTED: TaskActivity.ACTIVE,
        Outcome.RUNNING: TaskActivity.ACTIVE,
    }.get(outcome)
    if activity is not None:
        return CompletionDecision(activity=activity)
    return CompletionDecision(fault=True, diagnostic=f"status outcome={outcome.value}")


def reconcile_reset_completion(
    *,
    method: ActionMethod,
    outcome: Outcome,
    activity: TaskActivity,
    gate_sealed: bool,
    clock_failed: bool,
) -> ResetCompletionDecision:
    """Classify a reset completion without mutating runtime state."""
    if method == ActionMethod.STATUS:
        if outcome in (Outcome.RUNNING, Outcome.ACCEPTED):
            return ResetCompletionDecision(
                activity=TaskActivity.ACTIVE,
                cancel_required=True,
                request_cancel=True,
                emergency_cancel=gate_sealed or clock_failed,
            )
        if outcome in (Outcome.INACTIVE, Outcome.CANCELLED, Outcome.COMPLETED):
            resolved = {
                Outcome.INACTIVE: TaskActivity.INACTIVE,
                Outcome.CANCELLED: TaskActivity.CANCELLED,
                Outcome.COMPLETED: TaskActivity.COMPLETED,
            }[outcome]
            return ResetCompletionDecision(
                activity=resolved,
                cancel_required=False,
                prove_inactive=True,
                advance_reset=True,
            )
        safe = activity in _SAFE
        return ResetCompletionDecision(
            activity=activity if safe else TaskActivity.UNKNOWN,
            cancel_required=not safe,
            request_cancel=not safe,
            emergency_cancel=not safe and (gate_sealed or clock_failed),
            reset_success=False,
            diagnostic="status reconciliation is unsafe",
        )
    if method == ActionMethod.CANCEL:
        if outcome in (Outcome.CANCELLED, Outcome.INACTIVE, Outcome.ACCEPTED):
            return ResetCompletionDecision(
                activity=TaskActivity.ACTIVE,
                cancel_required=False,
                advance_reset=True,
            )
        return ResetCompletionDecision(
            cancel_required=False,
            reset_success=False,
            diagnostic=None,
        )
    if method == ActionMethod.RESET and outcome == Outcome.INACTIVE:
        return ResetCompletionDecision(
            reset_required=False,
            complete_reset=True,
            advance_reset=True,
        )
    return ResetCompletionDecision(reset_success=False)
