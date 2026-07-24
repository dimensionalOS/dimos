# Copyright 2026 Dimensional Inc.
"""Minimal, fail-closed Pi baseline execution boundary."""

from __future__ import annotations

from collections.abc import Callable
import ctypes
from dataclasses import dataclass
import errno
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import threading
from typing import cast
import uuid

from dimos.benchmark.spatial.models import AnswerType
from dimos.benchmark.spatial.pi_baseline.broker import CaseBroker, PolicyViolationError
from dimos.benchmark.spatial.pi_baseline.config import PiBaselineConfig, PromptMode
from dimos.benchmark.spatial.pi_baseline.controller import (
    PROTOCOL_VERSION,
    AdapterController,
    AdapterRunError,
    AdapterTerminalResult,
)
from dimos.benchmark.spatial.pi_baseline.evidence import (
    EvidenceIdentityContext,
    EvidenceManifest,
    EvidencePublishDisposition,
    build_evidence_manifest,
    load_committed_result,
    publish_evidence_manifest_noreplace,
)
from dimos.benchmark.spatial.pi_baseline.gate import (
    ArtifactReference,
    HumanReleaseRecord,
    SmokeRunEvidence,
)
from dimos.benchmark.spatial.pi_baseline.native_session import (
    CaptureState,
    ExportVerificationRecord,
    ExportVerificationStatus,
    FailureReason,
    NativeSessionReceipt,
    PromptContextRecord,
    compare_receipt,
    read_prompt_context,
    read_session_bytes,
    receipt_for_session,
    validate_native_session,
    verify_export_session,
    verify_prompt_context,
)
from dimos.benchmark.spatial.pi_baseline.podman import (
    ContainerCleanupError,
    PersistentPodmanCase,
    PodmanLimits,
    PodmanRun,
    PodmanTimeoutError,
    RootlessPodman,
)
from dimos.benchmark.spatial.pi_baseline.projection import stage_public_instance
from dimos.benchmark.spatial.pi_baseline.prompts import build_prompt_pair
from dimos.benchmark.spatial.pi_baseline.scheduler_executor import ExecutionInterrupted
from dimos.benchmark.spatial.pi_baseline.scoring import score_case
from dimos.benchmark.spatial.pi_baseline.topology import (
    PinnedDirectory,
    PinnedRuntimeTopology,
    fresh_directory_cursor,
    pin_runtime_topology,
)
from dimos.benchmark.spatial.pi_baseline.transaction import AnswerTransaction
from dimos.benchmark.spatial.utilities import JsonValue, canonical_json


@dataclass(frozen=True)
class PairedRun:
    run_id: str
    gate_path: Path
    mode_roots: tuple[Path, ...]


@dataclass(frozen=True)
class ConditionRun:
    """Artifacts produced by one independent case/condition execution."""

    run_id: str
    mode: PromptMode
    mode_root: Path
    evidence: SmokeRunEvidence


@dataclass(frozen=True)
class FinalizeAttemptResult:
    status: str
    committed: bool
    failure_reason: str | None = None
    failure_evidence_retained: bool = False
    answer_correct: bool | None = None


@dataclass(frozen=True)
class _NativeAdmission:
    session_path: str
    prompt_path: str
    initial_prompt_path: str
    session_state: str
    receipt_matches: bool
    native_tree_valid: bool
    exact_prompts: bool
    receipt: NativeSessionReceipt
    system_prompt: PromptContextRecord
    initial_prompt: PromptContextRecord


def finalize_attempt(
    *,
    accepted_submission: bool,
    scoring_complete: bool,
    answer_correct: bool | None = None,
    session_state: str,
    receipt_matches: bool = False,
    native_tree_valid: bool = False,
    exact_prompts: bool = False,
    pinned_export_succeeded: bool = False,
    source_unchanged: bool = False,
    infrastructure_error: str | None = None,
    cleanup_error: str | None = None,
    publication_error: str | None = None,
) -> FinalizeAttemptResult:
    """Decide publication only after every non-scoring prerequisite succeeds."""
    failure = infrastructure_error or (
        "session_incomplete" if session_state != "complete" else None
    )
    prerequisites = (
        ("submission_not_accepted", accepted_submission),
        ("scoring_incomplete", scoring_complete),
        ("receipt_mismatch", receipt_matches),
        ("native_tree_invalid", native_tree_valid),
        ("prompt_mismatch", exact_prompts),
        ("export_failed", pinned_export_succeeded),
        ("source_mutated", source_unchanged),
    )
    if failure is None:
        failure = next((reason for reason, passed in prerequisites if not passed), None)
    failure = failure or cleanup_error or publication_error
    if failure is not None:
        return FinalizeAttemptResult("failed", False, failure, True, answer_correct=answer_correct)
    return FinalizeAttemptResult("committed", True, answer_correct=answer_correct)


def _unlink_owned_result(private: PinnedDirectory, name: str) -> None:
    """Remove only an owned regular result file, never a link or symlink."""
    try:
        info = os.stat(name, dir_fd=private.fd, follow_symlinks=False)
    except FileNotFoundError:
        return
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_uid != os.getuid():
        raise ValueError(f"unsafe publication artifact: {name}")
    os.unlink(name, dir_fd=private.fd)
    os.fsync(private.fd)


def _write_private_exclusive(private: PinnedDirectory, name: str, data: bytes) -> None:
    """Create one private provisional result with durable no-replace semantics."""
    fd = os.open(
        name,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
        0o600,
        dir_fd=private.fd,
    )
    try:
        view = memoryview(data)
        while view:
            view = view[os.write(fd, view) :]
        os.fsync(fd)
        os.fsync(private.fd)
    except BaseException:
        try:
            os.unlink(name, dir_fd=private.fd)
            os.fsync(private.fd)
        except OSError:
            pass
        raise
    finally:
        os.close(fd)


def _promote_no_replace(private: PinnedDirectory, provisional: str, canonical: str) -> None:
    """Atomically promote one private result without rename-overwrite semantics."""
    info = os.stat(provisional, dir_fd=private.fd, follow_symlinks=False)
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_uid != os.getuid():
        raise ValueError(f"unsafe provisional artifact: {provisional}")
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        raise OSError(errno.ENOSYS, "renameat2 is unavailable")
    renameat2.argtypes = [
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    ]
    renameat2.restype = ctypes.c_int
    result = renameat2(
        private.fd,
        os.fsencode(provisional),
        private.fd,
        os.fsencode(canonical),
        1,  # RENAME_NOREPLACE
    )
    if result != 0:
        error = ctypes.get_errno()
        if error == errno.EEXIST:
            raise ValueError(f"publication collision: {canonical}")
        raise OSError(error, os.strerror(error), canonical)
    os.fsync(private.fd)


def recover_publication_orphans(
    private: PinnedDirectory,
    public: PinnedDirectory,
    expected_identity: EvidenceIdentityContext,
) -> None:
    """Clean only markerless orphans; preserve conflicting committed evidence."""
    try:
        os.stat("evidence-manifest.v1.json", dir_fd=private.fd, follow_symlinks=False)
    except FileNotFoundError:
        _unlink_owned_result(private, "prediction.provisional.v1.json")
        _unlink_owned_result(private, "score.provisional.v1.json")
        _unlink_owned_result(private, "prediction.v1.json")
        _unlink_owned_result(private, "score.v1.json")
        return
    try:
        committed = load_committed_result(public, private, expected_identity=expected_identity)
    except (OSError, TypeError, ValueError) as error:
        raise RuntimeError(
            "evidence-manifest.v1.json conflicts with supplied identity or artifacts"
        ) from error
    if committed is None:
        raise RuntimeError("evidence-manifest.v1.json could not be authoritatively admitted")


def _persist_export_verification(
    private: PinnedDirectory, verification: ExportVerificationRecord
) -> None:
    """Persist bounded export evidence before any status decision is made."""
    private.write_bytes(
        "native-export-verification.v1.json",
        canonical_json(cast("JsonValue", verification.model_dump(mode="json", exclude_none=True)))
        + b"\n",
    )


def publish_attempt_results(
    *,
    private: PinnedDirectory,
    public: PinnedDirectory,
    prediction: str,
    score: str,
    manifest_factory: Callable[[], object],
    expected_identity: EvidenceIdentityContext,
) -> EvidencePublishDisposition:
    """Promote both results, then publish the evidence marker as the commit point."""
    promoted: list[str] = []
    marker_linearized = False
    try:
        _promote_no_replace(private, prediction, "prediction.v1.json")
        promoted.append("prediction.v1.json")
        _promote_no_replace(private, score, "score.v1.json")
        promoted.append("score.v1.json")
        manifest = manifest_factory()
        if not hasattr(manifest, "private"):
            raise TypeError("manifest factory did not return an evidence manifest")
        disposition = publish_evidence_manifest_noreplace(
            private,
            cast("EvidenceManifest", manifest),
            public_root=public,
            expected_identity=expected_identity,
        )
        if disposition in {
            EvidencePublishDisposition.COMMITTED,
            EvidencePublishDisposition.ALREADY_EXISTING,
        }:
            marker_linearized = True
        elif disposition is EvidencePublishDisposition.DURABILITY_UNKNOWN:
            marker_linearized = (
                load_committed_result(public, private, expected_identity=expected_identity)
                is not None
            )
        if not marker_linearized:
            raise RuntimeError(f"evidence publication failed: {disposition.value}")
        if load_committed_result(public, private, expected_identity=expected_identity) is None:
            raise RuntimeError("published evidence could not be re-admitted")
        return disposition
    except BaseException:
        if not marker_linearized:
            for name in reversed(promoted):
                _unlink_owned_result(private, name)
            _unlink_owned_result(private, prediction)
            _unlink_owned_result(private, score)
        raise


def run_condition(
    config: PiBaselineConfig,
    *,
    mode: PromptMode,
    podman: RootlessPodman | None = None,
    controller_factory: type[AdapterController] = AdapterController,
    cancel_requested: threading.Event,
    publication_lock: threading.Lock,
) -> ConditionRun:
    """Run one case condition; no pairing or human-gate policy is applied."""
    run_id = config.run_id or f"run-{uuid.uuid4().hex}"
    root = Path(config.output_root).expanduser() / run_id
    private_root = Path(config.private_root).expanduser() / run_id
    podman = podman or RootlessPodman()
    _check_cancel(cancel_requested)
    return _run_condition(
        config,
        mode=mode,
        run_id=run_id,
        root=root,
        private_root=private_root,
        podman=podman,
        controller_factory=controller_factory,
        cancel_requested=cancel_requested,
        publication_lock=publication_lock,
    )


def run_paired(
    config: PiBaselineConfig,
    *,
    podman: RootlessPodman | None = None,
    controller_factory: type[AdapterController] = AdapterController,
    cancel_requested: threading.Event,
    publication_lock: threading.Lock,
) -> PairedRun:
    """Compatibility wrapper for the legacy CLI; execute each condition independently."""
    run_id = config.run_id or f"run-{uuid.uuid4().hex}"
    root = Path(config.output_root).expanduser() / run_id
    private_root = Path(config.private_root).expanduser() / run_id
    podman = podman or RootlessPodman()
    runs: list[ConditionRun] = []
    current_private: PinnedDirectory | None = None
    current_mode: PromptMode | None = None
    current_broker: CaseBroker | None = None
    try:
        modes: tuple[PromptMode, ...] = ("visualization-forbidden", "visualization-encouraged")
        for mode in modes:
            current_mode = mode
            current_private = PinnedDirectory.open(private_root / mode, create=True)
            os.fchmod(current_private.fd, 0o700)
            try:
                runs.append(
                    _run_condition(
                        config,
                        mode=mode,
                        run_id=run_id,
                        root=root,
                        private_root=private_root,
                        podman=podman,
                        controller_factory=controller_factory,
                        cancel_requested=cancel_requested,
                        publication_lock=publication_lock,
                    )
                )
            except Exception:
                raise
            else:
                current_private.close()
                current_private = None
                current_broker = None
        gate = HumanReleaseRecord(
            release_id=f"pending-{run_id}",
            infrastructure=(),
            smoke_runs=tuple(run.evidence for run in runs),
            blockers=("pending human review",),
            decision=None,
        )
        gate_path = private_root / "pending-human-gate.json"
        gate_root = PinnedDirectory.open(private_root, create=False)
        try:
            gate_root.write_bytes(
                "pending-human-gate.json",
                canonical_json(gate.model_dump(mode="json")) + b"\n",
            )
        finally:
            gate_root.close()
        return PairedRun(run_id, gate_path, tuple(run.mode_root for run in runs))
    except Exception as error:
        # Keep private diagnostics, but never leave a staging/container resource behind.
        _retain_failure_record(current_private, current_mode, current_broker, error)
        if current_private is not None:
            current_private.close()
        raise


def _run_condition(
    config: PiBaselineConfig,
    *,
    mode: PromptMode,
    run_id: str,
    root: Path,
    private_root: Path,
    podman: RootlessPodman,
    controller_factory: type[AdapterController],
    cancel_requested: threading.Event,
    publication_lock: threading.Lock,
) -> ConditionRun:
    """Execute, attest, score, and ledger one condition in strict order."""
    topology: PinnedRuntimeTopology | None = None
    mode_anchor: PinnedDirectory | None = None
    private_anchor: PinnedDirectory | None = None
    request: PodmanRun | None = None
    transaction: AnswerTransaction | None = None
    broker: CaseBroker | None = None
    terminal_result: AdapterTerminalResult | None = None
    native_admission: _NativeAdmission | None = None
    export_succeeded = False
    source_unchanged = False
    cleanup_error: Exception | None = None
    execution_error: Exception | None = None
    identity: EvidenceIdentityContext | None = None
    try:
        _check_cancel(cancel_requested)
        mode_root = root / mode
        topology, mode_anchor, private_anchor, _staged_input = _prepare_topology(
            config, mode, mode_root, private_root, cancel_requested
        )
        _check_cancel(cancel_requested)
        topology.verify()
        staging_path = topology.input.path
        private = topology.private.path
        private_dir = topology.private
        evidence_dir = topology.output
        identity = EvidenceIdentityContext(
            run_id=run_id,
            case_id=config.selection.instance_id,
            mode=mode,
            release_id=_release(staging_path)[0],
            scorer_revision=config.scorer_revision,
        )
        recover_publication_orphans(private_dir, evidence_dir, expected_identity=identity)
        transaction, request, lifecycle_audit = _prepare_run(
            config,
            mode,
            run_id,
            topology,
            staging_path,
            private,
            mode_anchor,
            private_anchor,
            cancel_requested,
        )
        try:
            with _persistent(podman, request, cancel_requested) as case:
                broker = CaseBroker(config.selection.instance_id, case, transaction, mode)
                controller = controller_factory(
                    tuple(config.node_adapter_command),
                    config.auth_path,
                    topology.private
                    if controller_factory is AdapterController
                    else lifecycle_audit,
                    **(
                        {"auth_mode": config.auth_mode}
                        if controller_factory is AdapterController
                        else {}
                    ),
                )
                try:
                    terminal_result = cast(
                        "AdapterTerminalResult",
                        _controller_run(
                            f"{run_id}-{mode.split('-')[-1]}",
                            controller,
                            broker,
                            _start_frame(
                                config, mode, staging_path, f"{run_id}-{mode.split('-')[-1]}"
                            ),
                            cancel_requested,
                            publication_lock,
                        ),
                    )
                except ExecutionInterrupted:
                    raise
                except Exception as error:
                    execution_error = error
                if getattr(case, "poisoned", False) and execution_error is not None:
                    raise execution_error
                try:
                    _check_cancel(cancel_requested)
                    logs = case.logs()
                    private_dir.write_bytes("container.log", (logs.stdout + logs.stderr).encode())
                except ExecutionInterrupted:
                    raise
                except Exception as error:
                    if execution_error is None:
                        execution_error = error
                if execution_error is None:
                    if terminal_result is None:
                        raise RuntimeError("adapter completed without terminal evidence")
                    native_admission = _admit_native_evidence(
                        private_dir, terminal_result, mode, staging_path, config
                    )
                    export_verification = _export_private_native_session(
                        config, private_dir, native_admission
                    )
                    if export_verification.status is not ExportVerificationStatus.SUCCEEDED:
                        code = export_verification.failure_code
                        reason = export_verification.failure_reason or "export verification failed"
                        raise ValueError(
                            f"{code.value if code is not None else 'export_failed'}: {reason}"
                        )
                    if receipt_for_session(
                        private_dir, native_admission.session_path
                    ) != native_admission.receipt or not compare_receipt(
                        private_dir, native_admission.receipt
                    ):
                        raise ValueError(FailureReason.SOURCE_MUTATED.value)
                    export_succeeded = True
                elif isinstance(execution_error, AdapterRunError):
                    _retain_native_failure_evidence(
                        private_dir,
                        execution_error.session_evidence
                        or (terminal_result.session_evidence if terminal_result else None),
                        execution_error,
                    )
                try:
                    _check_cancel(cancel_requested)
                    _export_public_staging_and_workspace(
                        topology.input, topology.workspace, topology.output, cancel_requested
                    )
                except ExecutionInterrupted:
                    raise
                except Exception as error:
                    if execution_error is None:
                        execution_error = error
                if execution_error is not None:
                    raise execution_error
        finally:
            active_error = sys.exc_info()[1]
            try:
                _verify_removed(podman, request)
            except Exception as error:
                cleanup_error = error
                if active_error is None and execution_error is None:
                    raise
        if broker is None:
            raise RuntimeError("case broker was not initialized")
        _check_cancel(cancel_requested)
        private_dir.write_bytes(
            "tool-audit.json", canonical_json(cast("JsonValue", list(broker.audit))) + b"\n"
        )
        compliance_error: PolicyViolationError | None = None
        try:
            broker.assert_compliant()
        except PolicyViolationError as error:
            compliance_error = error
        _check_cancel(cancel_requested)
        private_dir.write_bytes(
            "compliance.v1.json",
            canonical_json(
                {
                    "record_type": "pi-visualization-compliance",
                    "schema_version": "1.0",
                    "mode": mode,
                    "compliant": compliance_error is None,
                    "scoring_eligible": compliance_error is None,
                    "failure": compliance_error.code if compliance_error else None,
                }
            )
            + b"\n",
        )
        _check_cancel(cancel_requested)
        evidence_dir.mkdir("workspace")
        evidence_dir.write_bytes(
            "workspace-manifest.v1.json",
            canonical_json(
                cast(
                    "JsonValue",
                    {
                        "record_type": "pi-workspace",
                        "files": sorted(
                            path.removeprefix("workspace/")
                            for path in _descriptor_files(evidence_dir)
                            if path.startswith("workspace/")
                        ),
                    },
                )
            )
            + b"\n",
        )
        if native_admission is None:
            raise RuntimeError("native session evidence was not admitted")
        source_unchanged = False
        score = None
        private_artifacts = [
            "tool-audit.json",
            "compliance.v1.json",
            "container.log",
            "adapter-lifecycle-audit.v1.jsonl",
            "run-manifest.v1.json",
            "native-session-receipt.v1.json",
            "native-session-verification.v1.json",
            "native-export-verification.v1.json",
            native_admission.prompt_path,
            native_admission.initial_prompt_path,
            "prompt-context.v1.json",
            native_admission.session_path,
        ]
        if compliance_error is not None:
            raise compliance_error
        if transaction.prediction is None:
            raise RuntimeError(f"{mode}: adapter completed without a durable prediction")
        _check_cancel(cancel_requested)
        score = score_case(
            staging_path / "cases" / "case.v1.json",
            transaction.prediction,
            oracle_root=Path(config.oracle_root),
            run_id=run_id,
            mode=mode,
            release_id=_release(staging_path)[0],
            scorer_revision=config.scorer_revision,
            ledger_path=None,
        )
        with publication_lock:
            _check_cancel(cancel_requested)
            _write_private_exclusive(
                private_dir, "score.provisional.v1.json", score.model_dump_json().encode() + b"\n"
            )
        source_unchanged = _recheck_native_binding(
            private_dir, native_admission, config, mode, staging_path
        )
        if not source_unchanged:
            raise ValueError(FailureReason.SOURCE_MUTATED.value)
        final_gate = finalize_attempt(
            accepted_submission=transaction.prediction is not None,
            scoring_complete=score is not None,
            answer_correct=score.outcome == "correct",
            session_state=native_admission.session_state,
            receipt_matches=native_admission.receipt_matches,
            native_tree_valid=native_admission.native_tree_valid,
            exact_prompts=native_admission.exact_prompts,
            pinned_export_succeeded=export_succeeded,
            source_unchanged=source_unchanged,
        )
        if not final_gate.committed:
            raise RuntimeError(final_gate.failure_reason or "attempt finalization failed")
        with publication_lock:
            _check_cancel(cancel_requested)
            identity = EvidenceIdentityContext(
                run_id=run_id,
                case_id=config.selection.instance_id,
                mode=mode,
                release_id=_release(staging_path)[0],
                scorer_revision=config.scorer_revision,
            )
            publish_attempt_results(
                private=private_dir,
                public=evidence_dir,
                prediction=transaction.provisional_filename,
                score="score.provisional.v1.json",
                manifest_factory=lambda: build_evidence_manifest(
                    evidence_dir,
                    private_dir,
                    public_artifacts=_approved_public_files(evidence_dir),
                    private_artifacts=tuple(
                        (*private_artifacts, "prediction.v1.json", "score.v1.json")
                    ),
                ),
                expected_identity=identity,
            )
        run_evidence = SmokeRunEvidence(
            run_id=run_id,
            mode=mode,
            case_sha256=hashlib.sha256(evidence_dir.read_relative("case.v1.json")).hexdigest(),
            manifest_sha256=hashlib.sha256(
                private_dir.read_bytes("evidence-manifest.v1.json")
            ).hexdigest(),
            review_bundle=_ref(private_dir, "evidence-manifest.v1.json"),
            private_score=_ref(private_dir, "score.v1.json"),
            transcript=_ref(private_dir, native_admission.session_path),
            tool_trace=_ref(private_dir, "tool-audit.json"),
            audit=_ref(private_dir, "container.log"),
        )
        return ConditionRun(run_id, mode, mode_root, run_evidence)
    except Exception as error:
        retained_private = topology.private if topology is not None else None
        if retained_private is not None and topology is not None and identity is not None:
            try:
                recover_publication_orphans(
                    retained_private, topology.output, expected_identity=identity
                )
            except Exception:
                pass
        _retain_native_failure_evidence(
            retained_private,
            getattr(error, "session_evidence", None)
            or (terminal_result.session_evidence if terminal_result else None),
            error,
        )
        if cleanup_error is None:
            _retain_failure_record(retained_private, mode, broker, error)
        else:
            _retain_failure_record(
                retained_private, mode, broker, error, supplemental=cleanup_error
            )
        raise
    finally:
        if transaction is not None:
            transaction.close()
        if topology is not None:
            topology.close()
        if mode_anchor is not None:
            mode_anchor.close()
        if private_anchor is not None:
            private_anchor.close()


def _prepare_topology(
    config: PiBaselineConfig,
    mode: PromptMode,
    mode_root: Path,
    private_root: Path,
    cancel_requested: threading.Event,
) -> tuple[PinnedRuntimeTopology, PinnedDirectory, PinnedDirectory, PinnedDirectory]:
    """Safely stage and pin the exact staging directory used by the container."""
    mode_anchor: PinnedDirectory | None = None
    private_anchor: PinnedDirectory | None = None
    topology: PinnedRuntimeTopology | None = None
    try:
        _check_cancel(cancel_requested)
        mode_anchor = PinnedDirectory.open(mode_root, create=True)
        private_anchor = PinnedDirectory.open(private_root, create=True)
        mode_anchor.mkdir("work")
        mode_anchor.mkdir("evidence")
        private_anchor.mkdir(mode)
        _check_cancel(cancel_requested)
        staged = stage_public_instance(
            Path(config.corpus_root), mode_anchor, **config.selection.model_dump()
        )
        staged_leaf = (
            staged
            if isinstance(staged, PinnedDirectory)
            else PinnedDirectory.open_at(mode_anchor, staged.name)
        )
        topology = pin_runtime_topology(
            input_dir=staged_leaf,
            workspace_dir=PinnedDirectory.open_at(mode_anchor, "work"),
            output_dir=PinnedDirectory.open_at(mode_anchor, "evidence"),
            private_dir=PinnedDirectory.open_at(private_anchor, mode),
        )
        assert topology is not None
        return topology, mode_anchor, private_anchor, topology.input
    except BaseException:
        if topology is not None:
            topology.close()
        if mode_anchor is not None:
            mode_anchor.close()
        if private_anchor is not None:
            private_anchor.close()
        raise


def _prepare_run(
    config: PiBaselineConfig,
    mode: PromptMode,
    run_id: str,
    topology: PinnedRuntimeTopology,
    staging: Path,
    private: Path,
    mode_anchor: PinnedDirectory,
    private_anchor: PinnedDirectory,
    cancel_requested: threading.Event,
) -> tuple[AnswerTransaction, PodmanRun, Path]:
    try:
        _check_cancel(cancel_requested)
        transaction = AnswerTransaction(
            config.selection.instance_id,
            _answer_type(staging / "cases" / "case.v1.json"),
            topology.private,
            "prediction.provisional.v1.json",
        )
        request = PodmanRun(
            config.runner_image,
            f"{run_id}-{mode.split('-')[-1]}",
            topology,
            limits=PodmanLimits(
                timeout_seconds=config.resource_limits.timeout_seconds,
                memory=f"{config.resource_limits.memory_mb}m",
                cpus=str(config.resource_limits.cpu_cores),
                pids=config.resource_limits.pids,
                agent_environment_mb=config.resource_limits.agent_environment_mb,
            ),
        )
        _check_cancel(cancel_requested)
        topology.private.write_bytes(
            "run-manifest.v1.json",
            canonical_json(
                {
                    "record_type": "pi-run-manifest",
                    "schema_version": "1.0",
                    "model_id": config.model.model_id,
                    "thinking_level": config.model.thinking_level,
                    "implementation_digests": config.implementation_digests.model_dump(),
                }
            )
            + b"\n",
        )
        return transaction, request, topology.private.path / "adapter-lifecycle-audit.v1.jsonl"
    except BaseException:
        topology.close()
        mode_anchor.close()
        private_anchor.close()
        raise


def _start_frame(
    config: PiBaselineConfig, mode: PromptMode, staging: Path, run_id: str
) -> dict[str, object]:
    case = json.loads((staging / "cases" / "case.v1.json").read_text(encoding="utf-8"))
    prompt_pair = build_prompt_pair()
    shared_prompt = (
        prompt_pair.visualization_forbidden
        if mode == "visualization-forbidden"
        else prompt_pair.visualization_encouraged
    )
    return {
        "version": PROTOCOL_VERSION,
        "type": "run_start",
        "id": run_id,
        "prompt": f"{shared_prompt}\n\nCase question:\n{case['question']['text']}",
        "budget": {
            "maxTurns": config.budgets.max_turns,
            "maxToolCalls": config.budgets.max_tool_calls,
            "timeoutMs": config.budgets.timeout_ms,
        },
        "config": {
            "promptMode": mode.replace("-", "_"),
            "answerType": case["question"]["answer_type"],
            "modelId": config.model.model_id,
            "thinkingLevel": config.model.thinking_level,
            "implementationDigests": config.implementation_digests.model_dump(),
        },
    }


def _answer_type(path: Path) -> AnswerType:
    return AnswerType(json.loads(path.read_text(encoding="utf-8"))["question"]["answer_type"])


def _release(staging: Path) -> tuple[str, str]:
    release = json.loads((staging / "staging-manifest.v1.json").read_text(encoding="utf-8"))[
        "release"
    ]
    return str(release["release_id"]), str(release["release_version"])


def _ref(directory: PinnedDirectory, name: str) -> ArtifactReference:
    """Return a durable logical reference, never a descriptor pathname."""
    return ArtifactReference(
        path=name, sha256=hashlib.sha256(directory.read_relative(name)).hexdigest()
    )


def _admit_native_evidence(
    private: PinnedDirectory,
    terminal: AdapterTerminalResult,
    mode: PromptMode,
    staging: Path,
    config: PiBaselineConfig,
) -> _NativeAdmission:
    """Admit the adapter's attempt-bound native session before public export."""
    session = terminal.session_evidence
    if session.state != CaptureState.COMPLETE.value or not session.persisted:
        raise ValueError("native session is not complete and persisted")
    if session.relative_path is None or not session.relative_path.startswith("pi-session/"):
        raise ValueError("native session path is not attempt-bound")
    data = read_session_bytes(private, session.relative_path)
    validation = validate_native_session(data, state=CaptureState.COMPLETE)
    if validation.state is not CaptureState.COMPLETE or validation.reason is not None:
        raise ValueError(f"native session validation failed: {validation.reason}")
    receipt = receipt_for_session(private, session.relative_path)
    receipt_matches = compare_receipt(private, receipt)
    if not receipt_matches:
        raise ValueError("native session receipt mismatch")
    private.write_bytes(
        "native-session-receipt.v1.json",
        canonical_json(receipt.model_dump(mode="json")) + b"\n",
    )
    private.write_bytes(
        "native-session-verification.v1.json",
        canonical_json(
            cast(
                "JsonValue",
                {
                    "record_type": "pi-native-session-verification",
                    "schema_version": "1.0",
                    "state": validation.state.value,
                    "session_id": validation.session_id,
                    "entry_count": validation.entry_count,
                },
            )
        )
        + b"\n",
    )
    prompt = session.system_prompt
    initial_prompt = session.initial_prompt
    if prompt is None or initial_prompt is None or validation.session_id is None:
        raise ValueError("native session prompt evidence is missing")
    prompt_record = PromptContextRecord(
        kind="system",
        relative_path=prompt.relative_path,
        byte_count=prompt.byte_count,
        sha256=prompt.sha256,
        session_id=validation.session_id,
    )
    exact_prompts = verify_prompt_context(private, prompt_record)
    initial_record = PromptContextRecord(
        kind="initial",
        relative_path=initial_prompt.relative_path,
        byte_count=initial_prompt.byte_count,
        sha256=initial_prompt.sha256,
        session_id=validation.session_id,
    )
    initial_bytes = read_prompt_context(private, initial_record.relative_path)
    exact_prompts = exact_prompts and verify_prompt_context(private, initial_record)
    expected_initial = str(_start_frame(config, mode, staging, "prompt-check")["prompt"]).encode()
    exact_prompts = exact_prompts and initial_bytes == expected_initial
    if not exact_prompts:
        raise ValueError("native session prompt context mismatch")
    if not initial_bytes:
        raise ValueError("native session initial prompt context is empty")
    if not exact_prompts:
        raise ValueError("native session prompt context mismatch")
    private.write_bytes(
        "prompt-context.v1.json",
        canonical_json(
            cast(
                "JsonValue",
                {
                    "system": prompt_record.model_dump(mode="json"),
                    "initial": initial_record.model_dump(mode="json"),
                },
            )
        )
        + b"\n",
    )
    del staging
    return _NativeAdmission(
        session.relative_path,
        prompt_record.relative_path,
        initial_record.relative_path,
        session.state,
        receipt_matches,
        True,
        exact_prompts,
        receipt,
        prompt_record,
        initial_record,
    )


def _export_private_native_session(
    config: PiBaselineConfig, private: PinnedDirectory, admission: _NativeAdmission
) -> ExportVerificationRecord:
    """Export admitted native bytes through the pinned, verified Pi package."""
    try:
        package_root = _resolve_pi_package_root(Path(config.node_adapter_command[1]))
    except Exception:
        verification = ExportVerificationRecord(
            status=ExportVerificationStatus.FAILED,
            relative_path=admission.session_path,
            failure_code=FailureReason.EXECUTABLE_UNSAFE,
            failure_reason="pinned executable identity was not verified",
        )
        _persist_export_verification(private, verification)
        return verification
    verification = verify_export_session(
        Path(config.node_adapter_command[0]),
        private,
        source_relative_path=admission.session_path,
        package_root=package_root,
    )
    _persist_export_verification(private, verification)
    return verification


def _recheck_native_binding(
    private: PinnedDirectory,
    admission: _NativeAdmission,
    config: PiBaselineConfig,
    mode: PromptMode,
    staging: Path,
) -> bool:
    current = receipt_for_session(private, admission.session_path)
    if current != admission.receipt or not compare_receipt(private, admission.receipt):
        return False
    # Metadata is re-read from the canonical prompt-context binding.
    payload = json.loads(private.read_bytes("prompt-context.v1.json"))
    system = PromptContextRecord.model_validate(payload["system"])
    initial = PromptContextRecord.model_validate(payload["initial"])
    if (
        system != admission.system_prompt
        or initial != admission.initial_prompt
        or system.kind != "system"
        or initial.kind != "initial"
        or system.relative_path != admission.prompt_path
        or initial.relative_path != admission.initial_prompt_path
        or system.session_id != admission.receipt.session_id
        or initial.session_id != admission.receipt.session_id
        or not verify_prompt_context(private, system)
        or not verify_prompt_context(private, initial)
    ):
        return False
    expected = str(_start_frame(config, mode, staging, "prompt-recheck")["prompt"]).encode()
    return read_prompt_context(private, initial.relative_path) == expected


def _resolve_pi_package_root(adapter_entry: Path) -> Path:
    """Resolve the immutable Pi installation beneath the adapter package."""
    entry = adapter_entry.absolute()
    for parent in (entry.parent, *entry.parents):
        nested = parent / "node_modules" / "@earendil-works" / "pi-coding-agent"
        if (nested / "package.json").is_file():
            return nested
    raise ValueError(FailureReason.EXECUTABLE_UNSAFE.value)


def _retain_native_failure_evidence(
    private: PinnedDirectory | None,
    session: object | None,
    error: Exception,
) -> None:
    """Retain native evidence without downgrading an already valid receipt."""
    if private is None:
        return

    existing: NativeSessionReceipt | None = None
    try:
        raw = private.read_bytes("native-session-receipt.v1.json")
        candidate = NativeSessionReceipt.model_validate(json.loads(raw))
        if candidate.state in {CaptureState.COMPLETE, CaptureState.PARTIAL}:
            if candidate.relative_path is not None and compare_receipt(private, candidate):
                existing = candidate
    except (OSError, ValueError, TypeError):
        existing = None

    state = getattr(session, "state", CaptureState.UNAVAILABLE.value) if session else None
    relative = getattr(session, "relative_path", None) if session else None
    receipt = existing
    reason = FailureReason.MISSING if session is None else _native_failure_reason(error)
    if existing is None and isinstance(relative, str):
        try:
            if state == CaptureState.COMPLETE.value:
                receipt = receipt_for_session(private, relative)
            elif state == CaptureState.PARTIAL.value:
                receipt = receipt_for_session(
                    private,
                    relative,
                    state=CaptureState.PARTIAL,
                    reason=FailureReason.INVALID_JSON,
                )
            assert receipt is not None
            reason = receipt.reason or reason
        except (OSError, ValueError, TypeError) as admission_error:
            reason = _native_failure_reason(admission_error)
            receipt = None

    if receipt is None:
        receipt = NativeSessionReceipt(state=CaptureState.UNAVAILABLE, reason=reason)
    try:
        if existing is None:
            private.write_bytes(
                "native-session-receipt.v1.json",
                canonical_json(receipt.model_dump(mode="json")) + b"\n",
            )
        private.write_bytes(
            "native-session-verification.v1.json",
            canonical_json(
                cast(
                    "JsonValue",
                    {
                        "record_type": "pi-native-session-verification",
                        "schema_version": "1.0",
                        "state": receipt.state.value,
                        "reason": reason.value,
                        "error": type(error).__name__,
                        **(
                            {
                                "session_id": receipt.session_id,
                                "relative_path": receipt.relative_path,
                            }
                            if receipt.state is not CaptureState.UNAVAILABLE
                            else {}
                        ),
                    },
                )
            )
            + b"\n",
        )
    except OSError:
        # Retention is best effort and must never replace the primary exception.
        return


def _native_failure_reason(error: Exception) -> FailureReason:
    """Map bounded local validation errors to a safe receipt reason."""
    text = str(error)
    for reason in FailureReason:
        if reason.value in text:
            return reason
    return FailureReason.INVALID_SCHEMA


def _source_unchanged(staging: PinnedDirectory) -> bool:
    try:
        staging.verify()
        return True
    except Exception:
        return False


def _approved_public_files(evidence: PinnedDirectory) -> tuple[str, ...]:
    approved = {
        "case.v1.json",
        "map.lcm",
        "provenance.v1.json",
        "result.v1.json",
        "verification.v1.json",
    }
    return tuple(path for path in _descriptor_files(evidence) if path in approved)


def _verify_removed(podman: RootlessPodman, request: PodmanRun) -> None:
    verifier = getattr(podman, "verify_removed", None)
    if verifier is not None:
        try:
            removed = verifier(request.run_id)
        except ContainerCleanupError:
            raise
        except Exception as error:
            raise ContainerCleanupError(ContainerCleanupError.reason) from error
        if not removed:
            raise ContainerCleanupError(ContainerCleanupError.reason)
        return
    try:
        result = subprocess.run(
            [podman.executable, "container", "exists", f"pi-baseline-{request.run_id}"],
            check=False,
            capture_output=True,
            timeout=10,
        )
    except Exception as error:
        raise ContainerCleanupError(ContainerCleanupError.reason) from error
    if result.returncode == 0:
        raise ContainerCleanupError(ContainerCleanupError.reason)


def _check_cancel(cancel_requested: threading.Event) -> None:
    if cancel_requested.is_set():
        raise ExecutionInterrupted


def _persistent(
    podman: RootlessPodman, request: PodmanRun, cancel_requested: threading.Event
) -> PersistentPodmanCase:
    return podman.persistent(request, cancel_requested)


def _controller_run(
    run_id: str,
    controller: AdapterController,
    broker: CaseBroker,
    start: dict[str, object],
    cancel_requested: threading.Event,
    publication_lock: threading.Lock,
) -> object:
    return controller.run(run_id, broker, start, cancel_requested, publication_lock)


def _export_public_staging_and_workspace(
    staging: PinnedDirectory,
    work: PinnedDirectory,
    public: PinnedDirectory,
    cancel_requested: threading.Event,
) -> None:
    """Export evidence while the case is alive without following workspace links."""
    for relative, destination in (
        ("cases/case.v1.json", "case.v1.json"),
        ("maps/map.lcm", "map.lcm"),
        ("provenance.v1.json", "provenance.v1.json"),
    ):
        _check_cancel(cancel_requested)
        public.write_bytes(destination, staging.read_relative(relative))
    public.mkdir("workspace")
    workspace = PinnedDirectory.open_at(public, "workspace")
    try:
        _export_workspace_dir(work.fd, workspace, cancel_requested)
    finally:
        workspace.close()


def _export_workspace_dir(
    source_fd: int, destination: PinnedDirectory, cancel_requested: threading.Event
) -> None:
    with fresh_directory_cursor(source_fd) as cursor_fd:
        with os.scandir(cursor_fd) as entries:
            for entry in entries:
                _check_cancel(cancel_requested)
                source_stat = os.stat(entry.name, dir_fd=cursor_fd, follow_symlinks=False)
                if stat.S_ISLNK(source_stat.st_mode) or not (
                    stat.S_ISREG(source_stat.st_mode) or stat.S_ISDIR(source_stat.st_mode)
                ):
                    raise ValueError("workspace contains an unsupported or symbolic-link entry")
                if stat.S_ISDIR(source_stat.st_mode):
                    destination.mkdir(entry.name)
                    target = PinnedDirectory.open_at(destination, entry.name)
                    child_fd = os.open(
                        entry.name,
                        os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW,
                        dir_fd=cursor_fd,
                    )
                    try:
                        _export_workspace_dir(child_fd, target, cancel_requested)
                    finally:
                        os.close(child_fd)
                        target.close()
                    continue
                if entry.name.lower().endswith((".html", ".htm")):
                    continue
                source_fd_file = os.open(entry.name, os.O_RDONLY | os.O_NOFOLLOW, dir_fd=cursor_fd)
                try:
                    if not stat.S_ISREG(os.fstat(source_fd_file).st_mode):
                        raise ValueError("workspace contains an unsupported file")
                    data = os.read(source_fd_file, os.fstat(source_fd_file).st_size)
                    destination.write_bytes(entry.name, data)
                finally:
                    os.close(source_fd_file)


def _descriptor_files(root: PinnedDirectory, prefix: str = "") -> tuple[str, ...]:
    result: list[str] = []
    with fresh_directory_cursor(root.fd) as cursor_fd:
        with os.scandir(cursor_fd) as entries:
            for entry in entries:
                info = os.stat(entry.name, dir_fd=cursor_fd, follow_symlinks=False)
                relative = f"{prefix}/{entry.name}" if prefix else entry.name
                if stat.S_ISDIR(info.st_mode):
                    child = PinnedDirectory.open_at(root, entry.name)
                    try:
                        result.extend(_descriptor_files(child, relative))
                    finally:
                        child.close()
                elif stat.S_ISREG(info.st_mode):
                    result.append(relative)
                else:
                    raise ValueError("evidence contains an unsupported or symbolic-link entry")
    return tuple(sorted(result))


def _retain_failure_record(
    private: Path | PinnedDirectory | None,
    mode: PromptMode | None,
    broker: CaseBroker | None,
    error: Exception,
    *,
    supplemental: Exception | None = None,
) -> None:
    """Best-effort host diagnostics; never let retention mask cleanup/failure."""
    if private is None:
        return
    try:
        if isinstance(private, PinnedDirectory):
            private.verify()
            target = private
        else:
            private.mkdir(parents=True, exist_ok=True)
            target = PinnedDirectory.open(private, create=False)
        owned = isinstance(private, PinnedDirectory)
        if broker is not None:
            target.write_bytes(
                "tool-audit.json", canonical_json(cast("JsonValue", list(broker.audit))) + b"\n"
            )
        adapter_reason = error.terminal_reason if isinstance(error, AdapterRunError) else None
        target.write_bytes(
            "failure.v1.json",
            canonical_json(
                cast(
                    "JsonValue",
                    {
                        "record_type": "pi-run-failure",
                        "schema_version": "1.0",
                        "mode": mode,
                        "error": type(error).__name__,
                        "scoring_eligible": False,
                        "ledger_appended": False,
                        **(
                            {"supplemental_cleanup_error": type(supplemental).__name__}
                            if supplemental is not None
                            else {}
                        ),
                        **(
                            {"adapter_reason": adapter_reason} if adapter_reason is not None else {}
                        ),
                        **(
                            {"policy_telemetry": error.policy_telemetry.model_dump(mode="json")}
                            if isinstance(error, AdapterRunError)
                            and error.policy_telemetry is not None
                            else {}
                        ),
                        **(
                            {"subprocess": _subprocess_failure_diagnostic(error)}
                            if _subprocess_failure_diagnostic(error) is not None
                            else {}
                        ),
                    },
                )
            )
            + b"\n",
        )
        if not owned:
            target.close()
    except Exception:
        return


def _subprocess_failure_diagnostic(
    error: Exception,
) -> dict[str, JsonValue] | None:
    if isinstance(error, PodmanTimeoutError):
        return {
            "timeout_operation": error.operation,
            "deadline_source": error.deadline_source,
            "stdout_truncated": error.stdout_truncated,
            "stderr_truncated": error.stderr_truncated,
            "stdout_bucket": error.stdout_bucket,
            "stderr_bucket": error.stderr_bucket,
        }
    if not isinstance(error, subprocess.CalledProcessError):
        return None
    command = error.cmd
    if isinstance(command, str):
        tokens = command.split()
    elif isinstance(command, (list, tuple)) and all(isinstance(item, str) for item in command):
        tokens = list(command)
    else:
        tokens = []
    executable = tokens[0].rsplit("/", 1)[-1] if tokens else ""
    operation = "subprocess"
    if executable == "podman":
        subcommand = tokens[1] if len(tokens) > 1 else ""
        if subcommand == "create" or (subcommand == "run" and "--detach" in tokens[2:]):
            operation = "podman-create"
        elif subcommand in {"start", "logs"}:
            operation = f"podman-{subcommand}"
    return {"operation": operation, "returncode": error.returncode}
