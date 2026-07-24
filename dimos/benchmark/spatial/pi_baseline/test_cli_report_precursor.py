from __future__ import annotations

from contextlib import contextmanager
from datetime import datetime, timezone
from hashlib import sha256
import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast

import pytest

from dimos.benchmark.spatial.models import AnswerType
from dimos.benchmark.spatial.pi_baseline import cli_support
from dimos.benchmark.spatial.pi_baseline.cli_support import execute_pi_precursor
from dimos.benchmark.spatial.pi_baseline.evidence import (
    EvidenceArtifact,
    EvidenceIdentityContext,
    EvidenceManifest,
    load_committed_result,
    publish_evidence_manifest_noreplace,
)
from dimos.benchmark.spatial.pi_baseline.records import Prediction
from dimos.benchmark.spatial.pi_baseline.scheduler_models import (
    AttemptContext,
    ExpandedCase,
    JobIdentity,
    JobSummary,
    NamedCondition,
    TerminalOutcome,
)
from dimos.benchmark.spatial.pi_baseline.scheduler_pi_executor import _condition_run_id
from dimos.benchmark.spatial.pi_baseline.scoring import PrivateScore
from dimos.benchmark.spatial.pi_baseline.topology import PinnedDirectory


def _runtime(*summaries: JobSummary) -> SimpleNamespace:
    condition = SimpleNamespace(
        name="forbidden", payload={"prompt_mode": "visualization-forbidden"}
    )
    plan = SimpleNamespace(
        jobs=tuple(
            SimpleNamespace(case_id=summary.identity.case_id, condition_name=condition.name)
            for summary in summaries
        ),
        conditions=(condition,),
    )
    store = SimpleNamespace(root=Path("experiment"))

    @contextmanager
    def lease():
        yield object()

    store.coordinator_lease = lease
    runtime = SimpleNamespace(
        store=store,
        _summaries={summary.identity.job_id: summary for summary in summaries},
        plan=plan,
        executor=None,
        manifest=None,
        _reload_reconcile_locked=lambda: None,
    )
    return runtime


def _summary(job_id: str, *, state: str = "pending") -> JobSummary:
    identity = JobIdentity(
        experiment_id="experiment",
        case_id="case",
        condition_name="forbidden",
        job_id=job_id,
    )
    return JobSummary(identity=identity, state=state)  # type: ignore[arg-type]


def _report_plan(case_id: str = "case") -> SimpleNamespace:
    condition = NamedCondition(name="forbidden", payload={"prompt_mode": "visualization-forbidden"})
    return SimpleNamespace(
        jobs=(SimpleNamespace(case_id=case_id, condition_name=condition.name),),
        conditions=(condition,),
    )


def _patch_precursor_dependencies(
    monkeypatch: pytest.MonkeyPatch, runtime: SimpleNamespace, calls: list[object]
) -> None:
    result = SimpleNamespace(
        manifest=SimpleNamespace(experiment_id="experiment", executor_fingerprint="fingerprint"),
        plan=runtime.plan,
        snapshot=SimpleNamespace(),
        manifest_digest="manifest",
        private_binding_digest="binding",
        admission_context=SimpleNamespace(),
    )
    monkeypatch.setattr(cli_support, "validate_pi_definition", lambda root, bindings: result)
    monkeypatch.setattr(
        "dimos.benchmark.spatial.pi_baseline.scheduler_pi_executor.PiSchedulerExecutor",
        lambda *args, **kwargs: object(),
    )
    monkeypatch.setattr(cli_support, "private_binding_digest", lambda *args, **kwargs: "binding")
    monkeypatch.setattr(cli_support, "bind_private_tree", lambda *args, **kwargs: None)
    runtime._execute = lambda selected, **kwargs: calls.append((selected, kwargs)) or ("done",)


def test_precursor_explicit_job_dispatches_once_to_existing_executor(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    summary = _summary("job-1")
    runtime = _runtime(summary)
    calls: list[object] = []
    _patch_precursor_dependencies(monkeypatch, runtime, calls)

    result = execute_pi_precursor(
        cast("Any", runtime),
        cast(
            "Any",
            SimpleNamespace(oracle_root=Path("oracle"), private_root=Path("private")),
        ),
        host_prerequisite=lambda: True,
        job_id_value="job-1",
    )

    assert result == ("done",)
    selected, kwargs = cast("tuple[tuple[str, ...], dict[str, object]]", calls[0])
    assert selected == ("job-1",)
    assert kwargs["allowed_states"] == {"pending"}


def test_precursor_rejects_ambiguous_or_missing_implicit_selection(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime = _runtime(_summary("job-1"), _summary("job-2"))
    calls: list[object] = []
    _patch_precursor_dependencies(monkeypatch, runtime, calls)

    with pytest.raises(ValueError, match="exactly one"):
        execute_pi_precursor(
            cast("Any", runtime),
            cast(
                "Any",
                SimpleNamespace(oracle_root=Path("oracle"), private_root=Path("private")),
            ),
            host_prerequisite=lambda: True,
        )
    assert calls == []


def test_precursor_rejects_non_forbidden_condition_without_dispatch(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    summary = _summary("job-1")
    runtime = _runtime(summary)
    runtime.plan.conditions[0].payload["prompt_mode"] = "visualization-encouraged"
    calls: list[object] = []
    _patch_precursor_dependencies(monkeypatch, runtime, calls)

    with pytest.raises(ValueError, match="visualization-forbidden"):
        execute_pi_precursor(
            cast("Any", runtime),
            cast(
                "Any",
                SimpleNamespace(oracle_root=Path("oracle"), private_root=Path("private")),
            ),
            host_prerequisite=lambda: True,
            job_id_value="job-1",
        )
    assert calls == []


def test_report_candidate_loader_calls_committed_authority_and_omits_invalid_artifacts(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    job_id = "job-1"
    attempt_id = "attempt-1"
    identity = JobIdentity(
        experiment_id="experiment",
        case_id="case",
        condition_name="forbidden",
        job_id=job_id,
    )
    context = AttemptContext(
        identity=identity, attempt_id=attempt_id, attempt_number=1, directory_name=attempt_id
    )
    run_id = _condition_run_id(context)
    public = (
        tmp_path
        / "public"
        / "experiment"
        / job_id
        / attempt_id
        / "public"
        / run_id
        / "visualization-forbidden"
        / "evidence"
    )
    private = (
        tmp_path
        / "private"
        / "experiment"
        / job_id
        / attempt_id
        / "private"
        / run_id
        / "visualization-forbidden"
    )
    public.mkdir(parents=True)
    private.mkdir(parents=True)
    provenance = b'{"release_id":"release"}\n'
    (public / "provenance.v1.json").write_bytes(provenance)
    (private / "evidence-manifest.v1.json").write_bytes(b"committed")
    attempt = SimpleNamespace(
        context=context,
        case=SimpleNamespace(case_id="case"),
        condition=NamedCondition(
            name="forbidden", payload={"prompt_mode": "visualization-forbidden"}
        ),
        outcome=SimpleNamespace(status="succeeded"),
    )
    summary = SimpleNamespace(identity=identity, state="succeeded", latest_attempt_id=attempt_id)
    selected = SimpleNamespace(case_id="case", provenance_sha256=sha256(provenance).hexdigest())
    snapshot = SimpleNamespace(selected_inputs=(selected,), scorer_revision="scorer")
    manifest = SimpleNamespace(experiment_id="experiment")
    store = SimpleNamespace(observe_attempts_read_only=lambda *args: (attempt,))
    calls: list[object] = []

    def committed(public_root: Path, private_root: Path, *, expected_identity: object) -> object:
        calls.append((public_root, private_root, expected_identity))
        return SimpleNamespace(
            prediction=SimpleNamespace(model_dump=lambda mode: {"value": True}),
            score=SimpleNamespace(model_dump=lambda mode: {"outcome": "incorrect"}),
        )

    monkeypatch.setattr(cli_support, "load_committed_result", committed)
    records = cli_support._committed_report_records(
        cast("Any", manifest),
        cast("Any", _report_plan()),
        cast("Any", snapshot),
        cast("Any", store),
        cast("Any", (summary,)),
        tmp_path / "private",
        tmp_path / "public",
    )

    assert len(records) == 1
    assert records[0]["job_id"] == job_id
    assert calls
    expected_identity = cast("Any", calls[0])[2]
    assert expected_identity.release_id == "release"
    assert expected_identity.scorer_revision == "scorer"


def _write_committed_attempt(
    public: Path,
    private: Path,
    identity: EvidenceIdentityContext,
    *,
    score_identity: EvidenceIdentityContext | None = None,
) -> None:
    public.mkdir(parents=True, exist_ok=True)
    private.mkdir(parents=True, exist_ok=True)
    public_descriptor = PinnedDirectory.open(public)
    private_descriptor = PinnedDirectory.open(private)
    try:
        public_bytes = b"case-public"
        public_descriptor.write_bytes("case.v1.json", public_bytes)
        score_identity = score_identity or identity
        prediction = Prediction.typed(score_identity.case_id, AnswerType.BOOLEAN, False)
        score = PrivateScore(
            instance_id=score_identity.case_id,
            answer_type=AnswerType.BOOLEAN,
            value=False,
            run_id=score_identity.run_id,
            case_id=score_identity.case_id,
            mode=score_identity.mode,
            release_id=score_identity.release_id,
            scorer_revision=score_identity.scorer_revision,
            outcome="incorrect",
            scored_at=datetime(2026, 1, 1, tzinfo=timezone.utc),
        )
        prediction_bytes = prediction.model_dump_json().encode()
        score_bytes = score.model_dump_json().encode()
        private_descriptor.write_bytes("prediction.v1.json", prediction_bytes)
        private_descriptor.write_bytes("score.v1.json", score_bytes)
        manifest = EvidenceManifest(
            public=(
                EvidenceArtifact(
                    path="case.v1.json",
                    size_bytes=len(public_bytes),
                    sha256=sha256(public_bytes).hexdigest(),
                ),
            ),
            private=(
                EvidenceArtifact(
                    path="prediction.v1.json",
                    size_bytes=len(prediction_bytes),
                    sha256=sha256(prediction_bytes).hexdigest(),
                ),
                EvidenceArtifact(
                    path="score.v1.json",
                    size_bytes=len(score_bytes),
                    sha256=sha256(score_bytes).hexdigest(),
                ),
            ),
        )
        assert publish_evidence_manifest_noreplace(
            private_descriptor,
            manifest,
            public_root=public_descriptor,
            expected_identity=score_identity,
        ).value in {"committed", "already-existing"}
    finally:
        public_descriptor.close()
        private_descriptor.close()


def _actual_report_fixture(
    tmp_path: Path,
    *,
    job_id: str = "job-1",
    case_id: str = "case",
) -> tuple[SimpleNamespace, EvidenceIdentityContext, Path, Path]:
    identity = JobIdentity(
        experiment_id="experiment",
        case_id=case_id,
        condition_name="forbidden",
        job_id=job_id,
    )
    context = AttemptContext(
        identity=identity,
        attempt_id="attempt-1",
        attempt_number=1,
        directory_name="attempt-1",
    )
    condition_run_id = _condition_run_id(context)
    mode = "visualization-forbidden"
    report_identity = EvidenceIdentityContext(
        run_id=condition_run_id,
        case_id=case_id,
        mode=mode,
        release_id="release",
        scorer_revision="scorer",
    )
    public = (
        tmp_path
        / "public"
        / "experiment"
        / job_id
        / "attempt-1"
        / "public"
        / condition_run_id
        / mode
        / "evidence"
    )
    private = (
        tmp_path
        / "private"
        / "experiment"
        / job_id
        / "attempt-1"
        / "private"
        / condition_run_id
        / mode
    )
    provenance = b'{"release_id":"release"}\n'
    public.mkdir(parents=True)
    (public / "provenance.v1.json").write_bytes(provenance)
    _write_committed_attempt(public, private, report_identity)
    attempt = SimpleNamespace(
        context=context,
        case=ExpandedCase(case_id=case_id, payload={}),
        condition=NamedCondition(
            name="forbidden", payload={"prompt_mode": "visualization-forbidden"}
        ),
        outcome=TerminalOutcome(status="succeeded", reason="committed"),
    )
    summary = JobSummary(
        identity=identity,
        state="succeeded",
        latest_attempt_id="attempt-1",
        outcome=TerminalOutcome(status="succeeded", reason="committed"),
    )
    selected = SimpleNamespace(case_id=case_id, provenance_sha256=sha256(provenance).hexdigest())
    snapshot = SimpleNamespace(selected_inputs=(selected,), scorer_revision="scorer")
    manifest = SimpleNamespace(experiment_id="experiment")
    store = SimpleNamespace(observe_attempts_read_only=lambda *args: (attempt,))
    return (
        SimpleNamespace(
            manifest=manifest,
            snapshot=snapshot,
            store=store,
            summary=summary,
            plan=_report_plan(case_id),
        ),
        report_identity,
        public,
        private,
    )


def test_report_discovers_real_runner_roots_once_and_excludes_private_contents(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    fixture, expected, public, private = _actual_report_fixture(tmp_path)
    calls: list[tuple[Path, Path, EvidenceIdentityContext]] = []

    def load(public_root: Path, private_root: Path, *, expected_identity: EvidenceIdentityContext):
        calls.append((public_root, private_root, expected_identity))
        return load_committed_result(public_root, private_root, expected_identity=expected_identity)

    monkeypatch.setattr(cli_support, "load_committed_result", load)
    records = cli_support._committed_report_records(
        fixture.manifest,
        fixture.plan,
        fixture.snapshot,
        fixture.store,
        (fixture.summary,),
        tmp_path / "private",
        tmp_path / "public",
    )

    assert len(records) == 1
    assert len(calls) == 1
    assert calls[0] == (public, private, expected)
    encoded = json.dumps(records, sort_keys=True)
    assert "prompt" not in encoded
    assert "private" not in encoded
    assert (
        records[0]["committed_manifest_sha256"]
        == sha256((private / "evidence-manifest.v1.json").read_bytes()).hexdigest()
    )


@pytest.mark.parametrize("failure", ["orphan", "tampered"])
def test_report_ignores_non_authoritative_actual_root_artifacts(
    tmp_path: Path, failure: str
) -> None:
    fixture, expected, public, private = _actual_report_fixture(tmp_path)
    if failure == "orphan":
        (private / "evidence-manifest.v1.json").unlink()
    elif failure == "tampered":
        (private / "score.v1.json").write_bytes(b"tampered")
    assert (
        cli_support._committed_report_records(
            fixture.manifest,
            fixture.plan,
            fixture.snapshot,
            fixture.store,
            (fixture.summary,),
            tmp_path / "private",
            tmp_path / "public",
        )
        == []
    )


@pytest.mark.parametrize("field", ["run_id", "case_id", "mode", "release_id", "scorer_revision"])
def test_report_rejects_each_wrong_committed_identity_field(tmp_path: Path, field: str) -> None:
    fixture, expected, public, private = _actual_report_fixture(tmp_path)
    values = {
        "run_id": "wrong-run",
        "case_id": "wrong-case",
        "mode": "visualization-encouraged",
        "release_id": "wrong-release",
        "scorer_revision": "wrong-scorer",
    }
    wrong = EvidenceIdentityContext(
        run_id=values["run_id"] if field == "run_id" else expected.run_id,
        case_id=values["case_id"] if field == "case_id" else expected.case_id,
        mode=values["mode"] if field == "mode" else expected.mode,
        release_id=values["release_id"] if field == "release_id" else expected.release_id,
        scorer_revision=(
            values["scorer_revision"] if field == "scorer_revision" else expected.scorer_revision
        ),
    )
    (private / "evidence-manifest.v1.json").unlink()
    _write_committed_attempt(public, private, expected, score_identity=wrong)
    assert (
        cli_support._committed_report_records(
            fixture.manifest,
            fixture.plan,
            fixture.snapshot,
            fixture.store,
            (fixture.summary,),
            tmp_path / "private",
            tmp_path / "public",
        )
        == []
    )


def test_report_ignores_wrong_attempt_job_and_old_simplified_paths(tmp_path: Path) -> None:
    fixture, expected, _, _ = _actual_report_fixture(tmp_path)
    old_public = tmp_path / "public" / "experiment" / "job-1" / "attempt-1" / "public"
    old_private = tmp_path / "private" / "experiment" / "job-1" / "attempt-1" / "private"
    _write_committed_attempt(old_public, old_private, expected)
    wrong_public = tmp_path / "public" / "wrong-run" / "visualization-forbidden" / "evidence"
    wrong_private = tmp_path / "private" / "wrong-run" / "visualization-forbidden"
    _write_committed_attempt(wrong_public, wrong_private, expected)

    records = cli_support._committed_report_records(
        fixture.manifest,
        fixture.plan,
        fixture.snapshot,
        fixture.store,
        (fixture.summary,),
        tmp_path / "private",
        tmp_path / "public",
    )
    assert len(records) == 1
    assert records[0]["attempt_id"] == "attempt-1"


def test_report_ignores_summary_pointing_at_wrong_attempt(tmp_path: Path) -> None:
    fixture, expected, _, _ = _actual_report_fixture(tmp_path)
    wrong_summary = fixture.summary.model_copy(update={"latest_attempt_id": "attempt-2"})
    assert (
        cli_support._committed_report_records(
            fixture.manifest,
            fixture.plan,
            fixture.snapshot,
            fixture.store,
            (wrong_summary,),
            tmp_path / "private",
            tmp_path / "public",
        )
        == []
    )


def test_report_rejects_outcome_root_redirection_and_namespace_symlink(tmp_path: Path) -> None:
    fixture, _, public, private = _actual_report_fixture(tmp_path)
    attempt = fixture.store.observe_attempts_read_only("experiment", "job-1")[0]
    attempt.outcome = SimpleNamespace(
        status="succeeded", public_root=tmp_path / "redirected-public"
    )
    assert (
        cli_support._committed_report_records(
            fixture.manifest,
            fixture.plan,
            fixture.snapshot,
            fixture.store,
            (fixture.summary,),
            tmp_path / "private",
            tmp_path / "public",
        )
        == []
    )

    attempt.outcome = TerminalOutcome(status="succeeded", reason="committed")
    redirected = tmp_path / "redirected"
    redirected.mkdir()
    namespace_root = tmp_path / "public" / "experiment"
    namespace_root.rename(redirected)
    namespace_root.symlink_to(redirected, target_is_directory=True)
    assert (
        cli_support._committed_report_records(
            fixture.manifest,
            fixture.plan,
            fixture.snapshot,
            fixture.store,
            (fixture.summary,),
            tmp_path / "private",
            tmp_path / "public",
        )
        == []
    )


def test_report_order_is_deterministic_across_scheduler_jobs(tmp_path: Path) -> None:
    first, _, _, _ = _actual_report_fixture(tmp_path, job_id="job-z")
    second, _, _, _ = _actual_report_fixture(tmp_path, job_id="job-a")
    attempts = {
        "job-z": first.store.observe_attempts_read_only,
        "job-a": second.store.observe_attempts_read_only,
    }
    store = SimpleNamespace(observe_attempts_read_only=lambda _, job_id: attempts[job_id](job_id))
    snapshot = SimpleNamespace(
        selected_inputs=first.snapshot.selected_inputs + second.snapshot.selected_inputs,
        scorer_revision="scorer",
    )
    records = cli_support._committed_report_records(
        first.manifest,
        first.plan,
        cast("Any", snapshot),
        cast("Any", store),
        (first.summary, second.summary),
        tmp_path / "private",
        tmp_path / "public",
    )
    assert [record["job_id"] for record in records] == ["job-a", "job-z"]
