"""Contract tests for native-session-gated result publication.

These tests deliberately exercise the authority boundary rather than adapter,
model, container, or network behavior.  The finalization tests name the
smallest host seam required by the OpenSpec; a missing seam is a clear failure,
not a reason to fall back to the legacy manifest writer.
"""

from __future__ import annotations

import ast
from collections.abc import Callable
import os
from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast

import pytest

from dimos.benchmark.spatial.pi_baseline import cli_support, runner
from dimos.benchmark.spatial.pi_baseline.evidence import (
    EvidenceIdentityContext,
    EvidencePublishDisposition,
    load_committed_result,
    publish_evidence_manifest_noreplace,
)
from dimos.benchmark.spatial.pi_baseline.native_session import (
    CaptureState,
    FailureReason,
    validate_native_session,
)

from .test_evidence_commit import _bundle, _identity
from .test_native_session import entry, session


def _source(module: object) -> str:
    path = Path(module.__file__)  # type: ignore[attr-defined]
    return path.read_text(encoding="utf-8")


def _required_gate() -> Callable[..., Any]:
    gate = getattr(runner, "finalize_attempt", None)
    if not callable(gate):
        pytest.fail(
            "runner.finalize_attempt is the missing native-session publication seam; "
            "do not reintroduce write_evidence_manifest as a fallback"
        )
    return cast("Callable[..., Any]", gate)


def test_runner_final_publication_has_no_legacy_replace_writer() -> None:
    tree = ast.parse(_source(runner))
    calls = [
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    ]
    assert "write_evidence_manifest" not in calls
    assert "publish_evidence_manifest_noreplace" in _source(runner)
    source = _source(runner)
    assert "renameat2" in source
    assert "RENAME_NOREPLACE" in source
    assert "os.link(" not in source


def test_orphan_prediction_and_score_are_invisible_without_commit_marker(
    tmp_path: Path,
) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        os.rename(
            tmp_path / "private" / "prediction.v1.json",
            tmp_path / "private" / "prediction.provisional.v1.json",
        )
        os.rename(
            tmp_path / "private" / "score.v1.json",
            tmp_path / "private" / "score.provisional.v1.json",
        )
        assert load_committed_result(public, private, expected_identity=_identity()) is None
        assert not (tmp_path / "private" / "evidence-manifest.v1.json").exists()
        assert not (tmp_path / "private" / "prediction.v1.json").exists()
        assert not (tmp_path / "private" / "score.v1.json").exists()
        assert (tmp_path / "private" / "prediction.provisional.v1.json").exists()
        assert (tmp_path / "private" / "score.provisional.v1.json").exists()

        recovery = getattr(runner, "recover_publication_orphans", None)
        assert callable(recovery), (
            "runner.recover_publication_orphans is the required startup/retry seam; "
            "markerless provisional/canonical outputs must be removed or quarantined "
            "before an attempt executes"
        )
        recovery(private, public, expected_identity=_identity())
        assert not (tmp_path / "private" / "prediction.provisional.v1.json").exists()
        assert not (tmp_path / "private" / "score.provisional.v1.json").exists()
        for name in (
            "prediction.v1.json",
            "score.v1.json",
            "prediction.provisional.v1.json",
            "score.provisional.v1.json",
        ):
            path = tmp_path / "private" / name
            if path.exists():
                assert path.stat().st_nlink == 1
        assert not (tmp_path / "private" / "prediction.v1.json").exists()
        assert not (tmp_path / "private" / "score.v1.json").exists()
    finally:
        public.close()
        private.close()


def test_result_publication_is_explicitly_provisional_until_marker_commit() -> None:
    source = _source(runner)
    assert "prediction.provisional.v1.json" in source
    assert "score.provisional.v1.json" in source
    assert "promote" in source
    assert "recover_publication_orphans" in source


@pytest.mark.parametrize(
    ("failure_stage", "promotion_number"),
    [("after_prediction_promotion", 1), ("after_score_promotion", 2)],
)
def test_publication_faults_cleanup_and_retry_after_result_promotion(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    failure_stage: str,
    promotion_number: int,
) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        prediction = private.read_bytes("prediction.v1.json")
        score = private.read_bytes("score.v1.json")
        os.rename(
            tmp_path / "private" / "prediction.v1.json",
            tmp_path / "private" / "prediction.provisional.v1.json",
        )
        os.rename(
            tmp_path / "private" / "score.v1.json",
            tmp_path / "private" / "score.provisional.v1.json",
        )
        original = runner._promote_no_replace
        calls = 0

        def fail_after_promotion(root: object, provisional: str, canonical: str) -> None:
            nonlocal calls
            original(cast("Any", root), provisional, canonical)
            calls += 1
            if calls == promotion_number:
                raise RuntimeError(failure_stage)

        monkeypatch.setattr(runner, "_promote_no_replace", fail_after_promotion)
        with pytest.raises(RuntimeError, match=failure_stage):
            runner.publish_attempt_results(
                private=private,
                public=public,
                prediction="prediction.provisional.v1.json",
                score="score.provisional.v1.json",
                manifest_factory=lambda: manifest,
                expected_identity=_identity(),
            )
        runner.recover_publication_orphans(private, public, expected_identity=_identity())
        assert not (tmp_path / "private" / "prediction.v1.json").exists()
        assert not (tmp_path / "private" / "score.v1.json").exists()
        assert not (tmp_path / "private" / "prediction.provisional.v1.json").exists()
        assert not (tmp_path / "private" / "score.provisional.v1.json").exists()

        private.write_bytes("prediction.provisional.v1.json", prediction)
        private.write_bytes("score.provisional.v1.json", score)
        monkeypatch.setattr(runner, "_promote_no_replace", original)
        assert (
            runner.publish_attempt_results(
                private=private,
                public=public,
                prediction="prediction.provisional.v1.json",
                score="score.provisional.v1.json",
                manifest_factory=lambda: manifest,
                expected_identity=_identity(),
            ).value
            == "committed"
        )
    finally:
        public.close()
        private.close()


def test_publication_fault_immediately_before_marker_cleans_and_retries(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        prediction = private.read_bytes("prediction.v1.json")
        score = private.read_bytes("score.v1.json")
        os.rename(
            tmp_path / "private" / "prediction.v1.json",
            tmp_path / "private" / "prediction.provisional.v1.json",
        )
        os.rename(
            tmp_path / "private" / "score.v1.json",
            tmp_path / "private" / "score.provisional.v1.json",
        )
        with pytest.raises(RuntimeError, match="before_marker"):
            runner.publish_attempt_results(
                private=private,
                public=public,
                prediction="prediction.provisional.v1.json",
                score="score.provisional.v1.json",
                manifest_factory=lambda: (_ for _ in ()).throw(RuntimeError("before_marker")),
                expected_identity=_identity(),
            )
        runner.recover_publication_orphans(private, public, expected_identity=_identity())
        assert not (tmp_path / "private" / "prediction.v1.json").exists()
        assert not (tmp_path / "private" / "score.v1.json").exists()
        private.write_bytes("prediction.provisional.v1.json", prediction)
        private.write_bytes("score.provisional.v1.json", score)
        assert (
            runner.publish_attempt_results(
                private=private,
                public=public,
                prediction="prediction.provisional.v1.json",
                score="score.provisional.v1.json",
                manifest_factory=lambda: manifest,
                expected_identity=_identity(),
            ).value
            == "committed"
        )
    finally:
        public.close()
        private.close()


def test_recovery_preserves_only_exact_identity_commit(
    tmp_path: Path,
) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        os.rename(
            tmp_path / "private" / "prediction.v1.json",
            tmp_path / "private" / "prediction.provisional.v1.json",
        )
        os.rename(
            tmp_path / "private" / "score.v1.json",
            tmp_path / "private" / "score.provisional.v1.json",
        )
        runner.publish_attempt_results(
            private=private,
            public=public,
            prediction="prediction.provisional.v1.json",
            score="score.provisional.v1.json",
            manifest_factory=lambda: manifest,
            expected_identity=_identity(),
        )
        runner.recover_publication_orphans(private, public, expected_identity=_identity())
        assert (tmp_path / "private" / "prediction.v1.json").exists()
        assert (tmp_path / "private" / "score.v1.json").exists()
        marker_before = (tmp_path / "private" / "evidence-manifest.v1.json").read_bytes()
        prediction_before = (tmp_path / "private" / "prediction.v1.json").read_bytes()
        score_before = (tmp_path / "private" / "score.v1.json").read_bytes()
        mismatched = EvidenceIdentityContext("other", "instance", "mode", "release", "revision")
        with pytest.raises(RuntimeError, match="conflicts"):
            runner.recover_publication_orphans(private, public, expected_identity=mismatched)
        assert (tmp_path / "private" / "evidence-manifest.v1.json").read_bytes() == marker_before
        assert (tmp_path / "private" / "prediction.v1.json").read_bytes() == prediction_before
        assert (tmp_path / "private" / "score.v1.json").read_bytes() == score_before
    finally:
        public.close()
        private.close()


@pytest.mark.parametrize(
    "state, reason",
    [
        (CaptureState.UNAVAILABLE, FailureReason.MISSING),
        (CaptureState.PARTIAL, FailureReason.INVALID_JSON),
    ],
)
def test_invalid_or_unavailable_session_cannot_publish_orphan_results(
    tmp_path: Path, state: CaptureState, reason: FailureReason
) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        data = session(
            entry("message", "m", message={"role": "user", "content": "x", "timestamp": 1})
        )
        invalid = data + b"{" if state is CaptureState.PARTIAL else b""
        validation = validate_native_session(invalid, state=state)
        assert validation.state is state
        assert validation.reason is reason
        assert load_committed_result(public, private, expected_identity=_identity()) is None
        assert not (tmp_path / "private" / "evidence-manifest.v1.json").exists()
        del manifest
    finally:
        public.close()
        private.close()


def test_complete_gate_requires_submission_score_session_prompts_export_and_immutability() -> None:
    gate = _required_gate()
    result = gate(
        accepted_submission=True,
        scoring_complete=True,
        answer_correct=False,
        session_state="complete",
        receipt_matches=True,
        native_tree_valid=True,
        exact_prompts=True,
        pinned_export_succeeded=True,
        source_unchanged=True,
    )
    assert result.status == "committed"


@pytest.mark.parametrize(
    "field",
    [
        "accepted_submission",
        "scoring_complete",
        "receipt_matches",
        "native_tree_valid",
        "exact_prompts",
        "pinned_export_succeeded",
        "source_unchanged",
    ],
)
def test_missing_gate_prerequisite_retains_failure_and_publishes_no_result(field: str) -> None:
    gate = _required_gate()
    values = {
        "accepted_submission": True,
        "scoring_complete": True,
        "answer_correct": False,
        "session_state": "complete",
        "receipt_matches": True,
        "native_tree_valid": True,
        "exact_prompts": True,
        "pinned_export_succeeded": True,
        "source_unchanged": True,
    }
    values[field] = False
    result = gate(**values)
    assert result.status == "failed"
    assert result.committed is False
    assert result.failure_evidence_retained is True


def test_first_infrastructure_or_session_failure_wins_over_cleanup_and_publication() -> None:
    gate = _required_gate()
    result = gate(
        accepted_submission=True,
        scoring_complete=True,
        session_state="partial",
        infrastructure_error="session_open_failed",
        cleanup_error="cleanup_failed",
        publication_error="publication_failed",
    )
    assert result.status == "failed"
    assert result.failure_reason == "session_open_failed"


def test_commit_dispositions_are_terminal_and_never_rollback(tmp_path: Path) -> None:
    public, private, manifest = _bundle(tmp_path)
    try:
        first = publish_evidence_manifest_noreplace(
            private, manifest, public_root=public, expected_identity=_identity()
        )
        same = publish_evidence_manifest_noreplace(
            private, manifest, public_root=public, expected_identity=_identity()
        )
        conflict = publish_evidence_manifest_noreplace(
            private,
            manifest.model_copy(update={"public": ()}),
            public_root=public,
            expected_identity=_identity(),
        )
        assert first is EvidencePublishDisposition.COMMITTED
        assert same is EvidencePublishDisposition.ALREADY_EXISTING
        assert conflict is EvidencePublishDisposition.CONFLICT
        assert load_committed_result(public, private, expected_identity=_identity()) is not None
    finally:
        public.close()
        private.close()


def test_report_reads_only_committed_results_and_binds_identity() -> None:
    source = _source(cli_support)
    assert "load_committed_result" in source, (
        "report consumer must use the committed-result authority loader"
    )
    assert 'rglob("score.v1.json")' not in source
    assert "expected_identity" in source


def test_adapter_receives_attempt_bound_native_path_separate_from_transcript() -> None:
    source = Path(__file__).parents[4] / "packages/pi-spatial-adapter/src/session.ts"
    text = source.read_text(encoding="utf-8")
    assert "SessionManager.inMemory" not in text
    assert "PI_SPATIAL_SESSION_DIR" in text
    assert "getSessionFile" in text
    assert "sessionEvidence" in text
    assert "adapter.transcript" not in text


def test_precursor_command_selects_one_forbidden_job_without_pairing() -> None:
    source = _source(cli_support)
    assert "precursor" in source
    assert "visualization-forbidden" in source
    assert "paired" not in source[source.find("precursor") :]
    assert '_job_ids_with_state({"pending"})' not in source[source.find("precursor") :]


def test_gate_result_is_not_authority_when_only_prediction_like_attributes_exist() -> None:
    fake = SimpleNamespace(prediction=True, score=True, committed=False)
    assert not fake.committed
    assert not hasattr(fake, "evidence_manifest")
