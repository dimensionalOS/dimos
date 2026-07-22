"""Public model and snapshot projection tests."""

from dataclasses import FrozenInstanceError

import pytest

from dimos.manipulation.execution_models import LifecycleState, RuntimeContext, RuntimeSnapshot


def test_runtime_snapshot_is_an_immutable_public_projection() -> None:
    context = RuntimeContext(state=LifecycleState.READY, diagnostic="ready")
    snapshot = RuntimeSnapshot(
        context.state,
        context.ready_plan,
        context.ready_plan_id,
        context.planning_token,
        context.active,
        context.fault,
        context.diagnostic,
        context.shutdown,
        context.shutdown_result,
        context.revision,
    )
    assert snapshot.state == LifecycleState.READY
    assert snapshot.diagnostic == "ready"
    with pytest.raises(FrozenInstanceError):
        snapshot.diagnostic = "mutated"  # type: ignore[misc]
