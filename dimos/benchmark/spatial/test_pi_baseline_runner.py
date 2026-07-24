from hashlib import sha256
import json
import os
from pathlib import Path
from threading import Event, Lock, Thread
from types import SimpleNamespace

import pytest

from dimos.benchmark.spatial.pi_baseline.config import PiBaselineConfig
from dimos.benchmark.spatial.pi_baseline.controller import (
    PROTOCOL_VERSION,
    AdapterCleanupError,
    AdapterTerminalResult,
    SessionEvidence,
    SessionPromptEvidence,
)
from dimos.benchmark.spatial.pi_baseline.evidence import (
    EvidenceManifest,
    write_evidence_manifest,
)
from dimos.benchmark.spatial.pi_baseline.lifecycle_audit import (
    LifecycleAuditRecord,
    bounded_audit,
)
from dimos.benchmark.spatial.pi_baseline.native_session import (
    SESSION_FORMAT_VERSION,
    CaptureState,
    ExportVerificationRecord,
    ExportVerificationStatus,
    FailureReason,
    NativeSessionReceipt,
    PromptContextRecord,
    compare_receipt,
    make_prompt_sidecar,
    receipt_for_session,
    validate_native_session,
    verify_export_session,
    verify_prompt_context,
)
from dimos.benchmark.spatial.pi_baseline.prompts import build_prompt_pair
import dimos.benchmark.spatial.pi_baseline.runner as runner_module
from dimos.benchmark.spatial.pi_baseline.runner import (
    _start_frame,
    run_condition,
    run_paired,
)
from dimos.benchmark.spatial.pi_baseline.scheduler_runtime import SchedulerRuntime
from dimos.benchmark.spatial.pi_baseline.topology import PinnedDirectory
from dimos.benchmark.spatial.test_pi_baseline_broker import _png
from dimos.benchmark.spatial.test_pi_baseline_config import valid_payload


class FakeCase:
    def __init__(self, request: object) -> None:
        self.request = request

    def logs(self) -> SimpleNamespace:
        return SimpleNamespace(stdout="container", stderr="")


class FakePodman:
    executable = "podman"

    def __init__(self) -> None:
        self.removed: list[str] = []

    def persistent(self, request: object, cancel_requested: Event) -> "FakeContext":
        assert not cancel_requested.is_set()
        return FakeContext(self, request)

    def verify_removed(self, run_id: str) -> bool:
        self.removed.append(run_id)
        return True


class FakeContext:
    def __init__(self, podman: FakePodman, request: object) -> None:
        self.podman = podman
        self.request = request

    def __enter__(self) -> FakeCase:
        return FakeCase(self.request)

    def __exit__(self, *_: object) -> None:
        return None


class FakeAdapter:
    def __init__(self, _command: tuple[str, ...], _auth: Path, lifecycle_audit: Path) -> None:
        self.lifecycle_audit = lifecycle_audit

    def run(
        self,
        run_id: str,
        broker: object,
        start: dict[str, object],
        cancel_requested: Event,
        publication_lock: Lock,
    ) -> object:
        assert not cancel_requested.is_set()
        assert publication_lock.locked() is False
        assert start["id"] == run_id
        if start["config"]["promptMode"] == "visualization_encouraged":
            path = broker.case.request.workspace_dir / "fake.png"  # type: ignore[attr-defined]
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(_png(2, 2))
            broker.read_generated_image("fake.png")  # type: ignore[attr-defined]
            broker.commit_image_read(delivered=True)  # type: ignore[attr-defined]
        broker.submit_answer(True)  # type: ignore[attr-defined]
        return _complete_terminal_result(broker, start, self.lifecycle_audit, run_id)


def _complete_terminal_result(
    broker: object, start: dict[str, object], lifecycle_audit: Path, run_id: str
) -> AdapterTerminalResult:
    """Write the smallest real, attempt-bound Pi v3 capture used by fake adapters."""
    request = broker.case.request  # type: ignore[attr-defined]
    private = request.topology.private
    private.mkdir("pi-session")
    private.mkdir("pi-prompt")
    mode = str(start["config"]["promptMode"])
    pair = build_prompt_pair()
    prompt = (
        pair.visualization_forbidden
        if mode == "visualization_forbidden"
        else pair.visualization_encouraged
    )
    prompt_bytes = prompt.encode()
    session_bytes = (
        json.dumps(
            {
                "type": "session",
                "version": SESSION_FORMAT_VERSION,
                "id": "header",
                "timestamp": "t",
                "cwd": "/",
            }
        )
        + "\n"
        + json.dumps(
            {
                "type": "message",
                "id": "user",
                "parentId": None,
                "timestamp": "t",
                "message": {"role": "user", "content": prompt, "timestamp": 1},
            }
        )
        + "\n"
    ).encode()
    initial_prompt_bytes = (prompt + "\n\nCase question:\nanswer").encode()
    session_dir = private.open_relative("pi-session")
    prompt_dir = private.open_relative("pi-prompt")
    try:
        session_dir.write_bytes("session.jsonl", session_bytes)
        prompt_dir.write_bytes("system.txt", prompt_bytes)
        prompt_dir.write_bytes("initial.txt", initial_prompt_bytes)
    finally:
        session_dir.close()
        prompt_dir.close()
    private.write_bytes(
        "adapter-lifecycle-audit.v1.jsonl",
        bounded_audit(
            [
                LifecycleAuditRecord("1.0", 0, "in", "run_start"),
                LifecycleAuditRecord("1.0", 1, "out", "terminal"),
            ],
            limit=16,
        ),
    )
    return AdapterTerminalResult(
        run_id,
        True,
        (),
        lifecycle_audit,
        SessionEvidence(
            state="complete",
            persisted=True,
            relative_path="pi-session/session.jsonl",
            system_prompt=SessionPromptEvidence(
                relative_path="pi-prompt/system.txt",
                byte_count=len(prompt_bytes),
                sha256=sha256(prompt_bytes).hexdigest(),
            ),
            initial_prompt=SessionPromptEvidence(
                relative_path="pi-prompt/initial.txt",
                byte_count=len(initial_prompt_bytes),
                sha256=sha256(initial_prompt_bytes).hexdigest(),
            ),
        ),
    )


def _config(tmp_path: Path) -> PiBaselineConfig:
    auth = tmp_path / "oauth.json"
    auth.write_text("{}", encoding="utf-8")
    node, adapter, _ = _verified_pi_package(tmp_path)
    payload = valid_payload(auth)
    payload.update(
        {
            "node_adapter_command": [str(node), str(adapter)],
            "output_root": str(tmp_path / "out"),
            "private_root": str(tmp_path / "private"),
            "ledger_path": str(tmp_path / "ledger.jsonl"),
        }
    )
    return PiBaselineConfig.model_validate(payload)


def _operation_pair() -> tuple[Event, Lock]:
    return Event(), Lock()


def _score(*_: object, **kwargs: object) -> SimpleNamespace:
    mode = str(kwargs["mode"])
    payload = {
        "record_type": "pi-score",
        "schema_version": "1.0",
        "instance_id": "instance-1",
        "answer_type": "boolean",
        "value": True,
        "run_id": str(kwargs["run_id"]),
        "case_id": "instance-1",
        "mode": mode,
        "release_id": str(kwargs["release_id"]),
        "scorer_revision": str(kwargs["scorer_revision"]),
        "outcome": "correct",
        "scored_at": "2026-01-01T00:00:00Z",
    }
    return SimpleNamespace(outcome="correct", model_dump_json=lambda: json.dumps(payload))


def _stage(corpus: Path, parent: Path, **_: str) -> Path:
    staged = parent / "staged"
    (staged / "cases").mkdir(parents=True)
    (staged / "maps").mkdir()
    (staged / "cases" / "case.v1.json").write_text(
        json.dumps(
            {"schema_version": "1.0", "question": {"text": "answer", "answer_type": "boolean"}}
        ),
        encoding="utf-8",
    )
    (staged / "maps" / "map.lcm").write_bytes(b"map")
    (staged / "provenance.v1.json").write_text("{}", encoding="utf-8")
    (staged / "staging-manifest.v1.json").write_text(
        json.dumps({"release": {"release_id": "release", "release_version": "v1.0.0"}}),
        encoding="utf-8",
    )
    return staged


def _verified_pi_package(tmp_path: Path) -> tuple[Path, Path, Path]:
    """Create a deterministic adapter install with its pinned Pi dependency."""
    install = tmp_path / "pi-adapter-install"
    install.mkdir(exist_ok=True)
    adapter = install / "adapter.js"
    adapter.write_text("// deterministic test adapter entrypoint\n", encoding="utf-8")
    adapter.chmod(0o600)
    package = install / "node_modules" / "@earendil-works" / "pi-coding-agent"
    (package / "dist").mkdir(parents=True, exist_ok=True)
    (package / "package.json").write_text(
        json.dumps({"version": "0.80.10", "bin": {"pi": "dist/cli.js"}}),
        encoding="utf-8",
    )
    (package / "dist" / "cli.js").write_text("#!/bin/sh\n", encoding="utf-8")
    node = install / "node"
    node.write_text(
        '#!/bin/sh\ntest "$2" = "--export" || exit 2\ncp -- "$3" "$4"\n',
        encoding="utf-8",
    )
    node.chmod(0o700)
    (package / "dist" / "cli.js").chmod(0o600)
    return node, adapter, package


def test_runner_exports_admitted_complete_session_before_authority(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """The runner must export only after complete, pinned native admission."""
    monkeypatch.setattr(runner_module, "stage_public_instance", _stage)
    monkeypatch.setattr(runner_module, "score_case", _score)
    node, adapter, package = _verified_pi_package(tmp_path)
    session_path = tmp_path / "session.jsonl"
    session_path.write_bytes(
        b'{"type":"session","version":3,"id":"header","timestamp":"t","cwd":"/"}\n'
        b'{"type":"message","id":"user","parentId":null,"timestamp":"t",'
        b'"message":{"role":"user","content":"prompt","timestamp":1}}\n'
    )
    assert validate_native_session(session_path.read_bytes()).state is CaptureState.COMPLETE
    exported: list[tuple[Path, int, str | None, Path | None]] = []
    real_export = verify_export_session

    def observe_export(
        executable: Path,
        source: Path | PinnedDirectory,
        *,
        source_relative_path: str | None = None,
        package_root: Path | None = None,
        timeout: float = 30.0,
    ) -> ExportVerificationRecord:
        assert isinstance(source, PinnedDirectory)
        exported.append(
            (executable, os.fstat(source.fd).st_ino, source_relative_path, package_root)
        )
        return real_export(
            executable,
            source,
            source_relative_path=source_relative_path,
            package_root=package_root,
            timeout=timeout,
        )

    # This is the supported imported seam; absence of the seam is a blocker.
    monkeypatch.setattr(runner_module, "verify_export_session", observe_export, raising=False)
    result = run_condition(
        _config(tmp_path).model_copy(update={"node_adapter_command": (str(node), str(adapter))}),
        mode="visualization-forbidden",
        podman=FakePodman(),
        controller_factory=FakeAdapter,
        cancel_requested=Event(),
        publication_lock=Lock(),
    )  # type: ignore[arg-type]
    assert result.evidence.review_bundle.path == "evidence-manifest.v1.json"
    assert len(exported) == 1
    executable, root_inode, relative, package_root = exported[0]
    assert executable == node
    assert root_inode == os.stat(tmp_path / "private" / "run-1" / "visualization-forbidden").st_ino
    assert relative == "pi-session/session.jsonl"
    assert package_root == package
    assert not list((tmp_path / "private").rglob("*.html"))
    private = tmp_path / "private" / result.run_id / "visualization-forbidden"
    verification = json.loads(
        (private / "native-export-verification.v1.json").read_text(encoding="utf-8")
    )
    assert verification["status"] == ExportVerificationStatus.SUCCEEDED.value
    assert verification["package_version"] == "0.80.10"
    assert verification["session_format_version"] == 3
    assert verification["relative_path"] == "pi-session/session.jsonl"
    assert verification["source_byte_count_before"] == verification["source_byte_count_after"]
    assert verification["source_sha256_before"] == verification["source_sha256_after"]
    assert verification["cli_identity_verified"] is True
    assert verification["disposable_html_output_produced"] is True
    assert verification["disposable_html_output_disposed"] is True
    manifest = json.loads((private / "evidence-manifest.v1.json").read_text(encoding="utf-8"))
    assert "native-export-verification.v1.json" in {item["path"] for item in manifest["private"]}
    assert not (private / "adapter.transcript.ndjson").exists()

    def failed_export(*args: object, **kwargs: object) -> ExportVerificationRecord:
        del args, kwargs
        return ExportVerificationRecord(
            status=ExportVerificationStatus.FAILED,
            relative_path="pi-session/session.jsonl",
            failure_code=FailureReason.EXPORT_FAILED,
            failure_reason="pinned export command failed",
        )

    monkeypatch.setattr(runner_module, "verify_export_session", failed_export)
    failed_root = tmp_path / "failed"
    with pytest.raises(ValueError, match="export_failed"):
        run_condition(
            _config(tmp_path).model_copy(
                update={
                    "node_adapter_command": (str(node), str(adapter)),
                    "output_root": str(failed_root / "out"),
                    "private_root": str(failed_root / "private"),
                }
            ),
            mode="visualization-forbidden",
            podman=FakePodman(),
            controller_factory=FakeAdapter,
            cancel_requested=Event(),
            publication_lock=Lock(),
        )  # type: ignore[arg-type]
    failed_private = failed_root / "private" / "run-1" / "visualization-forbidden"
    record = json.loads(
        (failed_private / "native-export-verification.v1.json").read_text(encoding="utf-8")
    )
    assert record["status"] == ExportVerificationStatus.FAILED.value
    assert record["failure_code"] == FailureReason.EXPORT_FAILED.value
    assert not (failed_private / "prediction.v1.json").exists()
    assert not (failed_private / "score.v1.json").exists()
    assert not (failed_private / "evidence-manifest.v1.json").exists()
    assert not list(failed_private.rglob("*.html"))
    assert not (failed_private / "adapter.transcript.ndjson").exists()

    resolution_root = tmp_path / "resolution-failed"

    def fail_resolution(_entry: Path) -> Path:
        raise RuntimeError("SECRET /absolute/private/path")

    monkeypatch.setattr(runner_module, "_resolve_pi_package_root", fail_resolution)
    with pytest.raises(ValueError, match="executable_unsafe"):
        run_condition(
            _config(tmp_path).model_copy(
                update={
                    "node_adapter_command": (str(node), str(adapter)),
                    "output_root": str(resolution_root / "out"),
                    "private_root": str(resolution_root / "private"),
                }
            ),
            mode="visualization-forbidden",
            podman=FakePodman(),
            controller_factory=FakeAdapter,
            cancel_requested=Event(),
            publication_lock=Lock(),
        )  # type: ignore[arg-type]
    resolution_private = resolution_root / "private" / "run-1" / "visualization-forbidden"
    resolution_record = json.loads(
        (resolution_private / "native-export-verification.v1.json").read_text(encoding="utf-8")
    )
    assert resolution_record == {
        "failure_code": FailureReason.EXECUTABLE_UNSAFE.value,
        "failure_reason": "pinned executable identity was not verified",
        "producer": "@earendil-works/pi-coding-agent",
        "relative_path": "pi-session/session.jsonl",
        "record_type": "pi-export-verification",
        "schema_version": "1.0",
        "session_format_version": 3,
        "package_version": "0.80.10",
        "status": ExportVerificationStatus.FAILED.value,
        "cli_identity_verified": False,
        "disposable_html_output_produced": False,
        "disposable_html_output_disposed": False,
    }
    assert "SECRET" not in (resolution_private / "native-export-verification.v1.json").read_text()
    assert not (resolution_private / "prediction.v1.json").exists()
    assert not (resolution_private / "score.v1.json").exists()
    assert not (resolution_private / "evidence-manifest.v1.json").exists()


def test_partial_and_unavailable_adapter_evidence_are_retained_without_commit(
    tmp_path: Path,
) -> None:
    partial = b'{"type":"session","version":3,"id":"header","timestamp":"t","cwd":"/"}\n'
    partial += b'{"type":"message","id":"user","parentId":null,"timestamp":"t",'
    partial += b'"message":{"role":"user","content":"x","timestamp":1}}\n{"'
    assert (
        validate_native_session(partial, state=CaptureState.PARTIAL).state is CaptureState.PARTIAL
    )
    private_path = tmp_path / "private"
    private_path.mkdir()
    private = PinnedDirectory.open(private_path)
    private.mkdir("pi-session")
    private.mkdir("pi-prompt")
    session_dir = private.open_relative("pi-session")
    session_dir.write_bytes("session.jsonl", partial)
    session_dir.close()
    prompt_dir = private.open_relative("pi-prompt")
    prompt_dir.write_bytes("system.txt", b"system")
    prompt_dir.write_bytes("initial.txt", b"initial")
    prompt_dir.close()
    partial_receipt = receipt_for_session(
        private,
        "pi-session/session.jsonl",
        state=CaptureState.PARTIAL,
        reason=FailureReason.INVALID_JSON,
    )
    assert partial_receipt.relative_path == "pi-session/session.jsonl"
    assert compare_receipt(private, partial_receipt)
    unavailable = NativeSessionReceipt(state=CaptureState.UNAVAILABLE, reason=FailureReason.MISSING)
    assert partial_receipt.state is CaptureState.PARTIAL
    assert unavailable.relative_path is None and unavailable.sha256 is None
    assert not unavailable.session_id
    assert not (private_path / "evidence-manifest.v1.json").exists()
    private.close()


def test_cleanup_failure_is_supplemental_to_adapter_failure(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage)

    class FailingAdapter(FakeAdapter):
        def run(self, *args: object, **kwargs: object) -> object:
            raise RuntimeError("adapter primary failure")

    monkeypatch.setattr(
        runner_module,
        "_verify_removed",
        lambda *_: (_ for _ in ()).throw(RuntimeError("cleanup supplemental failure")),
    )
    with pytest.raises(RuntimeError, match="adapter primary failure"):
        run_condition(
            _config(tmp_path),
            mode="visualization-forbidden",
            podman=FakePodman(),
            controller_factory=FailingAdapter,
            cancel_requested=Event(),
            publication_lock=Lock(),
        )  # type: ignore[arg-type]


def test_swapped_or_tampered_prompt_sidecars_fail_closed(tmp_path: Path) -> None:
    private = tmp_path / "private"
    private.mkdir()
    system = b"system prompt"
    initial = b"full initial prompt"
    system_record = PromptContextRecord(
        kind="system",
        relative_path="pi-prompt/system.txt",
        byte_count=len(system),
        sha256=sha256(system).hexdigest(),
        session_id="header",
    )
    initial_record = PromptContextRecord(
        kind="initial",
        relative_path="pi-prompt/initial.txt",
        byte_count=len(initial),
        sha256=sha256(initial).hexdigest(),
        session_id="header",
    )
    (private / "pi-prompt").mkdir()
    (private / "pi-prompt" / "system.txt").write_bytes(system)
    (private / "pi-prompt" / "initial.txt").write_bytes(initial)
    (private / "pi-prompt" / "system.txt").chmod(0o600)
    (private / "pi-prompt" / "initial.txt").chmod(0o600)
    pinned = PinnedDirectory.open(private)
    try:
        assert verify_prompt_context(pinned, system_record)
        assert verify_prompt_context(pinned, initial_record)
        assert not verify_prompt_context(
            pinned,
            system_record.model_copy(
                update={"sha256": sha256(initial).hexdigest(), "byte_count": len(initial)}
            ),
        )
        assert not verify_prompt_context(
            pinned, initial_record.model_copy(update={"relative_path": "pi-prompt/missing.bin"})
        )
        assert make_prompt_sidecar(system).sha256 != make_prompt_sidecar(initial).sha256
    finally:
        pinned.close()


def test_paired_run_seals_evidence_and_leaves_human_gate(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage)
    monkeypatch.setattr(
        "dimos.benchmark.spatial.pi_baseline.runner.score_case",
        _score,
    )
    cancel_requested, publication_lock = _operation_pair()
    result = run_paired(
        _config(tmp_path),
        podman=FakePodman(),
        controller_factory=FakeAdapter,
        cancel_requested=cancel_requested,
        publication_lock=publication_lock,
    )  # type: ignore[arg-type]
    gate = json.loads(result.gate_path.read_text(encoding="utf-8"))
    assert gate["decision"] is None
    assert len(gate["smoke_runs"]) == 2
    assert all((root / "evidence").is_dir() for root in result.mode_roots)
    manifest = json.loads(
        (
            tmp_path
            / "private"
            / result.run_id
            / "visualization-forbidden"
            / "run-manifest.v1.json"
        ).read_text(encoding="utf-8")
    )
    assert manifest["model_id"] == "gpt-5.6-luna"
    assert manifest["thinking_level"] == "medium"
    assert manifest["implementation_digests"]["protocol"] == "protocol@sha256:" + "d" * 64
    private = tmp_path / "private" / result.run_id / "visualization-forbidden"
    audit = private / "adapter-lifecycle-audit.v1.jsonl"
    assert audit.is_file()
    assert not (private / "adapter.transcript.ndjson").exists()
    committed_manifest = json.loads(
        (private / "evidence-manifest.v1.json").read_text(encoding="utf-8")
    )
    private_paths = {item["path"] for item in committed_manifest["private"]}
    assert "adapter-lifecycle-audit.v1.jsonl" in private_paths
    assert "adapter.transcript.ndjson" not in private_paths
    assert "pi-session/session.jsonl" in private_paths


def test_single_condition_run_scores_without_pairing_or_gate(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage)
    monkeypatch.setattr(
        "dimos.benchmark.spatial.pi_baseline.runner.score_case",
        _score,
    )
    result = run_condition(
        _config(tmp_path),
        mode="visualization-encouraged",
        podman=FakePodman(),
        controller_factory=FakeAdapter,  # type: ignore[arg-type]
        cancel_requested=Event(),
        publication_lock=Lock(),
    )

    assert result.mode == "visualization-encouraged"
    assert result.mode_root.name == "visualization-encouraged"
    assert result.evidence.mode == result.mode
    assert result.evidence.transcript is not None
    assert result.evidence.transcript.path == "pi-session/session.jsonl"
    assert not (tmp_path / "private" / result.run_id / "pending-human-gate.json").exists()


def test_score_cancellation_wins_before_locked_score_persistence(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage)
    cancel_requested, publication_lock = _operation_pair()
    monkeypatch.setattr(
        "dimos.benchmark.spatial.pi_baseline.runner.score_case",
        lambda *args, **kwargs: (cancel_requested.set() or _score(*args, **kwargs)),
    )
    with pytest.raises(runner_module.ExecutionInterrupted):
        run_condition(
            _config(tmp_path),
            mode="visualization-forbidden",
            podman=FakePodman(),
            controller_factory=FakeAdapter,
            cancel_requested=cancel_requested,
            publication_lock=publication_lock,
        )
    assert not list((tmp_path / "private").rglob("score.v1.json"))
    assert not (tmp_path / "ledger.jsonl").exists()


def test_score_publication_wins_before_later_cancellation(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage)
    cancel_requested, publication_lock = _operation_pair()
    runtime = object.__new__(SchedulerRuntime)
    runtime._cancel_requested = cancel_requested
    runtime._publication_lock = publication_lock
    monkeypatch.setattr(
        "dimos.benchmark.spatial.pi_baseline.runner.score_case",
        _score,
    )
    original_write = runner_module._write_private_exclusive
    cancellers: list[Thread] = []

    def write_bytes(directory: PinnedDirectory, name: str, data: bytes) -> None:
        if name == "score.provisional.v1.json":
            cancellation_started = Event()

            def cancel() -> None:
                cancellation_started.set()
                runtime.cancel()

            canceller = Thread(target=cancel)
            canceller.start()
            assert cancellation_started.wait(1)
            assert canceller.is_alive()
            original_write(directory, name, data)
            cancellers.append(canceller)
            return
        original_write(directory, name, data)

    monkeypatch.setattr(runner_module, "_write_private_exclusive", write_bytes)
    with pytest.raises(runner_module.ExecutionInterrupted):
        run_condition(
            _config(tmp_path),
            mode="visualization-forbidden",
            podman=FakePodman(),
            controller_factory=FakeAdapter,
            cancel_requested=cancel_requested,
            publication_lock=publication_lock,
        )
    for canceller in cancellers:
        canceller.join(1)
        assert not canceller.is_alive()
    assert not list((tmp_path / "private").rglob("score.v1.json"))
    assert not list((tmp_path / "private").rglob("score.provisional.v1.json"))


def test_end_to_end_operation_pair_identity_reaches_runner_boundaries(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage)
    monkeypatch.setattr(
        "dimos.benchmark.spatial.pi_baseline.runner.score_case",
        _score,
    )
    cancel_requested, publication_lock = _operation_pair()
    seen: dict[str, object] = {}

    class CapturingPodman(FakePodman):
        def persistent(self, request: object, event: Event) -> "FakeContext":
            seen["podman_event"] = event
            return super().persistent(request, event)

    class CapturingAdapter(FakeAdapter):
        def run(self, run_id, broker, start, event, lock):
            seen["controller_event"] = event
            seen["controller_lock"] = lock
            return super().run(run_id, broker, start, event, lock)

    run_condition(
        _config(tmp_path),
        mode="visualization-forbidden",
        podman=CapturingPodman(),
        controller_factory=CapturingAdapter,
        cancel_requested=cancel_requested,
        publication_lock=publication_lock,
    )
    assert seen["podman_event"] is cancel_requested
    assert seen["controller_event"] is cancel_requested
    assert seen["controller_lock"] is publication_lock


def test_stubborn_reader_cleanup_fails_and_container_cleanup_still_runs(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage)
    markers: list[str] = []

    class CleanupPodman(FakePodman):
        def persistent(self, request: object, cancel_requested: Event) -> "FakeContext":
            markers.append("container-created")
            return super().persistent(request, cancel_requested)

        def verify_removed(self, run_id: str) -> bool:
            markers.append("absence-verified")
            return super().verify_removed(run_id)

    class StubbornAdapter(FakeAdapter):
        def run(self, run_id, broker, start, cancel_requested, publication_lock):
            markers.append("adapter-terminated")
            raise AdapterCleanupError("reader did not quiesce")

    with pytest.raises(AdapterCleanupError):
        run_condition(
            _config(tmp_path),
            mode="visualization-forbidden",
            podman=CleanupPodman(),
            controller_factory=StubbornAdapter,
            cancel_requested=Event(),
            publication_lock=Lock(),
        )
    assert markers == ["container-created", "adapter-terminated", "absence-verified"]
    assert not list((tmp_path / "private").rglob("outcome.v1.json"))


def test_start_frame_composes_selected_prompt_with_case_question(tmp_path: Path) -> None:
    config = _config(tmp_path)
    staging = _stage(tmp_path / "corpus", tmp_path / "stage")
    pair = build_prompt_pair()

    forbidden = _start_frame(config, "visualization-forbidden", staging, "run-forbidden")
    encouraged = _start_frame(config, "visualization-encouraged", staging, "run-encouraged")

    assert forbidden["prompt"] == pair.visualization_forbidden + "\n\nCase question:\nanswer"
    assert encouraged["prompt"] == pair.visualization_encouraged + "\n\nCase question:\nanswer"
    assert forbidden["version"] == encouraged["version"] == PROTOCOL_VERSION
    assert "relative path" in str(encouraged["prompt"])
    assert "regular non-symlink PNG" in str(encouraged["prompt"])


def test_failed_adapter_still_verifies_container_removal(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    class FailingAdapter(FakeAdapter):
        def run(
            self,
            run_id: str,
            broker: object,
            start: dict[str, object],
            cancel_requested: Event,
            publication_lock: Lock,
        ) -> object:
            raise RuntimeError("adapter failed")

    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage)
    podman = FakePodman()
    with pytest.raises(RuntimeError, match="adapter failed"):
        run_paired(
            _config(tmp_path),
            podman=podman,
            controller_factory=FailingAdapter,
            cancel_requested=Event(),
            publication_lock=Lock(),
        )  # type: ignore[arg-type]
    assert podman.removed


def test_policy_failures_retain_evidence_and_never_score_or_append(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    class PolicyAdapter(FakeAdapter):
        def run(
            self,
            run_id: str,
            broker: object,
            start: dict[str, object],
            cancel_requested: Event,
            publication_lock: Lock,
        ) -> object:
            if broker.prompt_mode == "visualization-forbidden":  # type: ignore[attr-defined]
                with pytest.raises(ValueError, match="visualization_forbidden"):
                    broker.read_generated_image("missing.png")  # type: ignore[attr-defined]
                broker.submit_answer(True)  # type: ignore[attr-defined]
            else:
                with pytest.raises(ValueError, match="visualization_required_before_submission"):
                    broker.submit_answer(True)  # type: ignore[attr-defined]
            return _complete_terminal_result(broker, start, self.lifecycle_audit, run_id)

    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage)
    score_calls: list[object] = []
    monkeypatch.setattr(
        "dimos.benchmark.spatial.pi_baseline.runner.score_case",
        lambda *args, **kwargs: score_calls.append(args),
    )
    podman = FakePodman()
    with pytest.raises(ValueError, match="visualization_forbidden"):
        run_paired(
            _config(tmp_path),
            podman=podman,
            controller_factory=PolicyAdapter,
            cancel_requested=Event(),
            publication_lock=Lock(),
        )  # type: ignore[arg-type]
    private = tmp_path / "private" / "run-1" / "visualization-forbidden"
    assert (private / "compliance.v1.json").is_file()
    assert (private / "failure.v1.json").is_file()
    assert not (private / "adapter.transcript.ndjson").exists()
    assert (private / "adapter-lifecycle-audit.v1.jsonl").is_file()
    assert not score_calls
    assert not (tmp_path / "ledger.jsonl").exists()
    assert len(podman.removed) == 1


def test_encouraged_policy_failure_is_retained_without_scoring_that_mode(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    class EncouragedFailureAdapter(FakeAdapter):
        def run(
            self,
            run_id: str,
            broker: object,
            start: dict[str, object],
            cancel_requested: Event,
            publication_lock: Lock,
        ) -> object:
            terminal: AdapterTerminalResult | None = None
            if broker.prompt_mode != "visualization-forbidden":  # type: ignore[attr-defined]
                terminal = _complete_terminal_result(broker, start, self.lifecycle_audit, run_id)
            if broker.prompt_mode == "visualization-forbidden":  # type: ignore[attr-defined]
                broker.submit_answer(True)  # type: ignore[attr-defined]
            else:
                with pytest.raises(ValueError, match="visualization_required_before_submission"):
                    broker.submit_answer(True)  # type: ignore[attr-defined]
            result = terminal or _complete_terminal_result(
                broker, start, self.lifecycle_audit, run_id
            )
            return result

    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage)
    scored_modes: list[str] = []
    monkeypatch.setattr(
        "dimos.benchmark.spatial.pi_baseline.runner.score_case",
        lambda *args, **kwargs: (scored_modes.append(kwargs["mode"]) or _score(*args, **kwargs)),
    )
    with pytest.raises(ValueError, match="visualization_required_before_submission"):
        run_paired(
            _config(tmp_path),
            podman=FakePodman(),
            controller_factory=EncouragedFailureAdapter,
            cancel_requested=Event(),
            publication_lock=Lock(),
        )  # type: ignore[arg-type]
    private = tmp_path / "private" / "run-1" / "visualization-encouraged"
    assert json.loads((private / "compliance.v1.json").read_text())["scoring_eligible"] is False
    assert (private / "failure.v1.json").is_file()
    assert not (private / "adapter.transcript.ndjson").exists()
    assert "visualization-encouraged" not in scored_modes


def test_paired_outer_failure_record_keeps_a_pinned_descriptor(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage)
    retained: list[object] = []
    original = runner_module._retain_failure_record

    def record(private: object, mode: object, broker: object, error: Exception) -> None:
        retained.append(private)
        original(private, mode, broker, error)  # type: ignore[arg-type]

    monkeypatch.setattr(runner_module, "_retain_failure_record", record)

    class FailingAdapter(FakeAdapter):
        def run(
            self,
            run_id: str,
            broker: object,
            start: dict[str, object],
            cancel_requested: Event,
            publication_lock: Lock,
        ) -> object:
            raise RuntimeError("paired failure")

    with pytest.raises(RuntimeError, match="paired failure"):
        run_paired(
            _config(tmp_path),
            podman=FakePodman(),
            controller_factory=FailingAdapter,
            cancel_requested=Event(),
            publication_lock=Lock(),
        )  # type: ignore[arg-type]

    assert len(retained) == 2
    assert all(isinstance(private, runner_module.PinnedDirectory) for private in retained)


def test_workspace_symlink_is_rejected_without_following_host_content(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    outside = tmp_path / "outside-secret.txt"
    outside.write_text("private", encoding="utf-8")

    class SymlinkAdapter(FakeAdapter):
        def run(
            self,
            run_id: str,
            broker: object,
            start: dict[str, object],
            cancel_requested: Event,
            publication_lock: Lock,
        ) -> object:
            work = broker.case.request.workspace_dir  # type: ignore[attr-defined]
            os.symlink(outside, work / "leak.txt")
            broker.submit_answer(True)  # type: ignore[attr-defined]
            return _complete_terminal_result(broker, start, self.lifecycle_audit, run_id)

    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage)
    podman = FakePodman()
    with pytest.raises(ValueError, match="symbolic-link"):
        run_paired(
            _config(tmp_path),
            podman=podman,
            controller_factory=SymlinkAdapter,
            cancel_requested=Event(),
            publication_lock=Lock(),
        )  # type: ignore[arg-type]
    assert not (
        tmp_path
        / "out"
        / "run-1"
        / "visualization-forbidden"
        / "evidence"
        / "workspace"
        / "leak.txt"
    ).exists()
    assert (
        tmp_path / "private" / "run-1" / "visualization-forbidden" / "failure.v1.json"
    ).is_file()
    assert podman.removed


def test_final_evidence_write_failure_prevents_ledger_append(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage)
    monkeypatch.setattr(
        "dimos.benchmark.spatial.pi_baseline.runner.score_case",
        _score,
    )

    def fail_final_manifest(*args: object, **kwargs: object) -> object:
        raise OSError("evidence storage failure")

    monkeypatch.setattr(
        "dimos.benchmark.spatial.pi_baseline.runner.publish_evidence_manifest_noreplace",
        fail_final_manifest,
    )
    with pytest.raises(OSError, match="evidence storage failure"):
        run_paired(
            _config(tmp_path),
            podman=FakePodman(),
            controller_factory=FakeAdapter,
            cancel_requested=Event(),
            publication_lock=Lock(),
        )  # type: ignore[arg-type]
    assert not (tmp_path / "ledger.jsonl").exists()
    assert not (
        tmp_path / "out" / "run-1" / "visualization-forbidden" / "evidence-manifest.v1.json"
    ).exists()
    assert not (
        tmp_path / "private" / "run-1" / "visualization-forbidden" / "evidence-manifest.v1.json"
    ).exists()


def _assert_descriptors_close(monkeypatch: pytest.MonkeyPatch) -> list[object]:
    closed: list[object] = []
    original = runner_module.PinnedDirectory.close

    def close(directory: object) -> None:
        closed.append(directory)
        original(directory)  # type: ignore[arg-type]

    monkeypatch.setattr(runner_module.PinnedDirectory, "close", close)
    return closed


def test_successful_run_closes_every_retained_descriptor(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage)
    monkeypatch.setattr(
        "dimos.benchmark.spatial.pi_baseline.runner.score_case",
        _score,
    )
    closed = _assert_descriptors_close(monkeypatch)

    run_condition(
        _config(tmp_path),
        mode="visualization-forbidden",
        podman=FakePodman(),
        controller_factory=FakeAdapter,
        cancel_requested=Event(),
        publication_lock=Lock(),
    )  # type: ignore[arg-type]

    assert closed
    assert all(directory.fd == -1 for directory in closed)  # type: ignore[attr-defined]


@pytest.mark.parametrize("failure", ["stage", "prepare", "verify"])
def test_preexecution_and_container_failures_close_every_retained_descriptor(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, failure: str
) -> None:
    if failure == "stage":

        def fail_stage(*args: object, **kwargs: object) -> Path:
            raise RuntimeError("stage failed")

        monkeypatch.setattr(
            "dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", fail_stage
        )
    elif failure == "prepare":
        monkeypatch.setattr(
            "dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage
        )

        def fail_prepare(*args: object, **kwargs: object) -> object:
            raise RuntimeError("setup failed")

        monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner._prepare_run", fail_prepare)
    else:
        monkeypatch.setattr(
            "dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage
        )

        class FailingVerificationPodman(FakePodman):
            def verify_removed(self, run_id: str) -> bool:
                super().verify_removed(run_id)
                raise RuntimeError("container verification failed")

        podman: FakePodman = FailingVerificationPodman()
    closed = _assert_descriptors_close(monkeypatch)

    with pytest.raises(RuntimeError):
        run_condition(
            _config(tmp_path),
            mode="visualization-forbidden",
            podman=podman if failure == "verify" else FakePodman(),
            controller_factory=FakeAdapter,
            cancel_requested=Event(),
            publication_lock=Lock(),
        )  # type: ignore[arg-type]

    assert closed
    assert all(directory.fd == -1 for directory in closed)  # type: ignore[attr-defined]


class _ReplacingContext(FakeContext):
    def __enter__(self) -> FakeCase:
        topology = self.request.topology  # type: ignore[attr-defined]
        self.old_paths: dict[str, Path] = {}
        for name, directory in (
            ("work", topology.workspace),
            ("evidence", topology.output),
            ("private", topology.private),
        ):
            path = directory.path
            old = path.with_name(path.name + "-pinned")
            path.rename(old)
            replacement = path.with_name(path.name + "-replacement")
            replacement.mkdir()
            path.symlink_to(replacement, target_is_directory=True)
            self.old_paths[name] = old
        redirected = topology.output.path.parent / "redirected-final-artifact"
        redirected.write_text("must remain unchanged", encoding="utf-8")
        for replacement, name in (
            (topology.output.path, "case.v1.json"),
            (topology.private.path, "tool-audit.json"),
        ):
            (replacement / name).symlink_to(redirected)
        self.redirected = redirected
        case = FakeCase(topology.workspace.path)
        case.request = self.request
        return case


class _ReplacingPodman(FakePodman):
    def persistent(self, request: object, cancel_requested: Event) -> _ReplacingContext:
        return _ReplacingContext(self, request)


class _DescriptorWritingAdapter(FakeAdapter):
    def run(
        self,
        run_id: str,
        broker: object,
        start: dict[str, object],
        cancel_requested: Event,
        publication_lock: Lock,
    ) -> object:
        request = broker.case.request  # type: ignore[attr-defined]
        topology = request.topology
        Path(f"/proc/self/fd/{topology.workspace.fd}/generated.txt").write_text(
            "pinned", encoding="utf-8"
        )
        broker.submit_answer(True)  # type: ignore[attr-defined]
        return _complete_terminal_result(broker, start, self.lifecycle_audit, run_id)


class _FailingDescriptorAdapter(_DescriptorWritingAdapter):
    def run(
        self,
        run_id: str,
        broker: object,
        start: dict[str, object],
        cancel_requested: Event,
        publication_lock: Lock,
    ) -> object:
        raise RuntimeError("descriptor-owned failure")


def test_replaced_runtime_paths_cannot_redirect_export_or_evidence(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage)
    monkeypatch.setattr(
        "dimos.benchmark.spatial.pi_baseline.runner.score_case",
        _score,
    )
    podman = _ReplacingPodman()
    result = run_condition(
        _config(tmp_path),
        mode="visualization-forbidden",
        podman=podman,
        controller_factory=_DescriptorWritingAdapter,
        cancel_requested=Event(),
        publication_lock=Lock(),
    )  # type: ignore[arg-type]

    pinned_evidence = tmp_path / "out" / "run-1" / "visualization-forbidden" / "evidence-pinned"
    pinned_private = tmp_path / "private" / "run-1" / "visualization-forbidden-pinned"
    assert (pinned_evidence / "case.v1.json").is_file()
    assert (pinned_evidence / "workspace" / "generated.txt").read_text() == "pinned"
    assert (pinned_private / "adapter-lifecycle-audit.v1.jsonl").is_file()
    assert not (pinned_private / "adapter.transcript.ndjson").exists()
    assert (
        tmp_path / "out" / "run-1" / "visualization-forbidden" / "redirected-final-artifact"
    ).read_text() == "must remain unchanged"
    assert result.evidence.review_bundle.path == "evidence-manifest.v1.json"


def test_replaced_private_path_cannot_redirect_failure_record(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr("dimos.benchmark.spatial.pi_baseline.runner.stage_public_instance", _stage)
    with pytest.raises(RuntimeError, match="descriptor-owned failure"):
        run_condition(
            _config(tmp_path),
            mode="visualization-forbidden",
            podman=_ReplacingPodman(),
            controller_factory=_FailingDescriptorAdapter,
            cancel_requested=Event(),
            publication_lock=Lock(),
        )  # type: ignore[arg-type]

    pinned_private = tmp_path / "private" / "run-1" / "visualization-forbidden-pinned"
    assert (pinned_private / "failure.v1.json").is_file()
    assert (
        tmp_path / "out" / "run-1" / "visualization-forbidden" / "redirected-final-artifact"
    ).read_text() == "must remain unchanged"


def test_provisional_evidence_manifest_symlink_in_retained_private_leaf_fails_closed(
    tmp_path: Path,
) -> None:
    private_path = tmp_path / "private"
    private_path.mkdir()
    escaped = tmp_path / "escaped-evidence-manifest.json"
    escaped.write_bytes(b"must remain unchanged")
    provisional_name = "evidence-manifest.v1.provisional.json"
    private_path.joinpath(provisional_name).symlink_to(escaped)
    private = PinnedDirectory.open(private_path)
    try:
        with pytest.raises(OSError):
            write_evidence_manifest(
                private,
                EvidenceManifest(public=(), private=()),
                name=provisional_name,
            )
        assert escaped.read_bytes() == b"must remain unchanged"
        assert (private_path / provisional_name).is_symlink()
    finally:
        private.close()


def test_failure_record_symlink_in_retained_private_leaf_fails_closed(tmp_path: Path) -> None:
    private_path = tmp_path / "private"
    private_path.mkdir()
    escaped = tmp_path / "escaped-failure.json"
    escaped.write_bytes(b"must remain unchanged")
    private_path.joinpath("failure.v1.json").symlink_to(escaped)
    private = PinnedDirectory.open(private_path)
    try:
        runner_module._retain_failure_record(
            private, "visualization-forbidden", None, RuntimeError("boom")
        )
        assert escaped.read_bytes() == b"must remain unchanged"
        assert (private_path / "failure.v1.json").is_symlink()
    finally:
        private.close()


def test_staged_export_symlink_source_fails_closed(tmp_path: Path) -> None:
    staging_path = tmp_path / "staging"
    work_path = tmp_path / "work"
    public_path = tmp_path / "public"
    for path in (staging_path, work_path, public_path):
        path.mkdir()
    (staging_path / "cases").mkdir()
    escaped = tmp_path / "escaped-staged-source.json"
    escaped.write_bytes(b"must remain unchanged")
    (staging_path / "cases" / "case.v1.json").symlink_to(escaped)
    staging = PinnedDirectory.open(staging_path)
    work = PinnedDirectory.open(work_path)
    public = PinnedDirectory.open(public_path)
    try:
        with pytest.raises(OSError):
            runner_module._export_public_staging_and_workspace(staging, work, public, Event())
        assert escaped.read_bytes() == b"must remain unchanged"
        assert not (public_path / "case.v1.json").exists()
    finally:
        staging.close()
        work.close()
        public.close()


def test_evidence_artifact_symlink_in_pinned_evidence_leaf_fails_closed(tmp_path: Path) -> None:
    evidence_path = tmp_path / "evidence"
    private_path = tmp_path / "private"
    evidence_path.mkdir()
    private_path.mkdir()
    escaped = tmp_path / "escaped-evidence-artifact.json"
    escaped.write_bytes(b"must remain unchanged")
    (evidence_path / "case.v1.json").symlink_to(escaped)
    evidence = PinnedDirectory.open(evidence_path)
    private = PinnedDirectory.open(private_path)
    try:
        with pytest.raises(ValueError, match="required evidence artifact is missing"):
            runner_module.build_evidence_manifest(
                evidence,
                private,
                public_artifacts=("case.v1.json",),
                private_artifacts=(),
            )
        assert escaped.read_bytes() == b"must remain unchanged"
        assert (evidence_path / "case.v1.json").is_symlink()
    finally:
        evidence.close()
        private.close()
