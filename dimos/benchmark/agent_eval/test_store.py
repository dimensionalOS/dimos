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

from datetime import UTC, datetime
import json
from pathlib import Path

import pytest

from dimos.benchmark.agent_eval.models import NormalizedOutcome
from dimos.benchmark.agent_eval.store import (
    AttemptAlreadyActiveError,
    AttemptStore,
    new_operation_id,
)


def _outcome(
    attempt_id: str,
    *,
    attempt_status: str = "completed",
    task_result: str = "passed",
    complete: bool = True,
    reason: str = "native evaluator terminal result",
) -> NormalizedOutcome:
    return NormalizedOutcome(
        attempt_id=attempt_id,
        attempt_status=attempt_status,
        task_result=task_result,
        terminal_stage="terminal",
        reason=reason,
        required_artifacts_complete=complete,
        finished_at=datetime.now(UTC),
        duration_s=1.0,
    )


@pytest.mark.parametrize("task_result", ["passed", "failed"])
def test_completed_pass_and_fail_write_atomic_outcome(tmp_path: Path, task_result: str) -> None:
    with AttemptStore(tmp_path) as store:
        reference = store.write_outcome(_outcome(store.attempt_id, task_result=task_result))

        assert reference.path == "outcome.v1.json"
        assert store.verify_artifacts((reference,))
        payload = json.loads((store.path / reference.path).read_text())
        assert payload["attempt_status"] == "completed"
        assert payload["task_result"] == task_result


def test_infrastructure_failure_is_not_evaluated_and_retains_partial_evidence(
    tmp_path: Path,
) -> None:
    with AttemptStore(tmp_path) as store:
        event = store.append_event(
            "reset-failed",
            operation_id=new_operation_id(),
            payload={"stage": "reset"},
        )
        diagnostic = store.write_artifact("diagnostics/reset.txt", "reset timed out")
        outcome = store.write_outcome(
            _outcome(
                store.attempt_id,
                attempt_status="failed",
                task_result="not_evaluated",
                complete=False,
                reason="reset timed out",
            )
        )

        assert event.sequence == 1
        assert store.verify_artifacts((diagnostic, outcome))


def test_interrupted_store_retains_events_and_releases_target_lock(
    tmp_path: Path,
) -> None:
    first = AttemptStore(tmp_path)
    first.append_event("interrupted")
    first_path = first.path
    first.close()

    with AttemptStore(tmp_path) as second:
        assert second.path != first_path
    assert (first_path / "events.jsonl").read_text()


def test_missing_or_changed_artifact_is_detected(tmp_path: Path) -> None:
    with AttemptStore(tmp_path) as store:
        missing = store.write_artifact("partial.json", {"retained": True})
        (store.path / missing.path).unlink()

        assert not store.verify_artifacts((missing,))


def test_existing_attempt_directory_is_never_reused(tmp_path: Path) -> None:
    attempt_id = "attempt_" + "1" * 32
    (tmp_path / attempt_id).mkdir(parents=True)

    with pytest.raises(FileExistsError):
        AttemptStore(tmp_path, attempt_id)


def test_concurrent_attempt_against_same_target_is_rejected(tmp_path: Path) -> None:
    first = AttemptStore(tmp_path)
    try:
        with pytest.raises(AttemptAlreadyActiveError):
            AttemptStore(tmp_path)
    finally:
        first.close()


def test_events_are_append_only_correlated_and_monotonic(tmp_path: Path) -> None:
    with AttemptStore(tmp_path) as store:
        operation_id = new_operation_id()
        first = store.append_event("reset-started", operation_id=operation_id)
        second = store.append_event("reset-finished", operation_id=operation_id)

    records = [json.loads(line) for line in (store.path / "events.jsonl").read_text().splitlines()]
    assert [record["sequence"] for record in records] == [1, 2]
    assert first.operation_id == second.operation_id == operation_id
    assert first.monotonic_offset_s <= second.monotonic_offset_s
    assert all(record["attempt_id"] == store.attempt_id for record in records)


def test_outcome_is_non_overwriting(tmp_path: Path) -> None:
    with AttemptStore(tmp_path) as store:
        store.write_outcome(_outcome(store.attempt_id))

        with pytest.raises(FileExistsError):
            store.write_outcome(
                _outcome(store.attempt_id, task_result="failed", reason="different")
            )


def test_outcome_write_failure_keeps_partial_evidence_and_no_outcome(
    tmp_path: Path, monkeypatch
) -> None:
    with AttemptStore(tmp_path) as store:
        evidence = store.write_artifact("task.v1.json", {"task": "public"})

        def fail_link(_source: Path, _destination: Path) -> None:
            raise OSError("simulated link failure")

        monkeypatch.setattr("dimos.benchmark.agent_eval.store.os.link", fail_link)
        with pytest.raises(OSError, match="simulated link failure"):
            store.write_outcome(_outcome(store.attempt_id))

        assert store.verify_artifacts((evidence,))
        assert not (store.path / "outcome.v1.json").exists()
