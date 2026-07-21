# Copyright 2026 Dimensional Inc.
"""Minimal, fail-closed Pi baseline execution boundary."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import threading
from typing import cast
import uuid

from dimos.benchmark.spatial.models import AnswerType
from dimos.benchmark.spatial.pi_baseline.broker import CaseBroker, PolicyViolationError
from dimos.benchmark.spatial.pi_baseline.config import PiBaselineConfig, PromptMode
from dimos.benchmark.spatial.pi_baseline.controller import (
    PROTOCOL_VERSION,
    AdapterCleanupError,
    AdapterController,
    AdapterRunError,
)
from dimos.benchmark.spatial.pi_baseline.evidence import (
    build_evidence_manifest,
    write_evidence_manifest,
)
from dimos.benchmark.spatial.pi_baseline.gate import (
    ArtifactReference,
    HumanReleaseRecord,
    SmokeRunEvidence,
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
    try:
        _check_cancel(cancel_requested)
        mode_root = root / mode
        topology, mode_anchor, private_anchor, staging = _prepare_topology(
            config, mode, mode_root, private_root, cancel_requested
        )
        _check_cancel(cancel_requested)
        topology.verify()
        staging = topology.input.path
        private = topology.private.path
        evidence = topology.output.path
        private_dir = topology.private
        evidence_dir = topology.output
        transaction, request, transcript = _prepare_run(
            config,
            mode,
            run_id,
            topology,
            staging,
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
                    Path(config.codex_oauth_auth_path),
                    topology.private if controller_factory is AdapterController else transcript,
                )
                execution_error: Exception | None = None
                try:
                    _controller_run(
                        f"{run_id}-{mode.split('-')[-1]}",
                        controller,
                        broker,
                        _start_frame(config, mode, staging, f"{run_id}-{mode.split('-')[-1]}"),
                        cancel_requested,
                        publication_lock,
                    )
                except ExecutionInterrupted:
                    raise
                except Exception as error:
                    execution_error = error
                if isinstance(execution_error, AdapterCleanupError):
                    raise execution_error
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
            _verify_removed(podman, request)
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
        private_artifacts = [
            "tool-audit.json",
            "compliance.v1.json",
            "container.log",
            "adapter.transcript.ndjson",
            "run-manifest.v1.json",
        ]
        if transaction.prediction is not None:
            try:
                private_dir.read_bytes("prediction.v1.json")
            except OSError:
                pass
            else:
                private_artifacts.append("prediction.v1.json")
        _check_cancel(cancel_requested)
        write_evidence_manifest(
            private_dir,
            build_evidence_manifest(
                evidence_dir,
                private_dir,
                public_artifacts=tuple(_descriptor_files(evidence_dir)),
                private_artifacts=tuple(private_artifacts),
            ),
        )
        if compliance_error is not None:
            raise compliance_error
        if transaction.prediction is None:
            raise RuntimeError(f"{mode}: adapter completed without a durable prediction")
        _check_cancel(cancel_requested)
        score = score_case(
            staging / "cases" / "case.v1.json",
            transaction.prediction,
            oracle_root=Path(config.oracle_root),
            run_id=run_id,
            mode=mode,
            release_id=_release(staging)[0],
            scorer_revision=config.scorer_revision,
            ledger_path=None,
        )
        with publication_lock:
            _check_cancel(cancel_requested)
            private_dir.write_bytes("score.v1.json", score.model_dump_json().encode() + b"\n")
        final_manifest = build_evidence_manifest(
            evidence_dir,
            private_dir,
            public_artifacts=tuple(_descriptor_files(evidence_dir)),
            private_artifacts=tuple((*private_artifacts, "score.v1.json")),
        )
        with publication_lock:
            _check_cancel(cancel_requested)
            write_evidence_manifest(private_dir, final_manifest)
        evidence = SmokeRunEvidence(
            run_id=run_id,
            mode=mode,
            case_sha256=hashlib.sha256(evidence_dir.read_relative("case.v1.json")).hexdigest(),
            manifest_sha256=hashlib.sha256(
                private_dir.read_bytes("evidence-manifest.v1.json")
            ).hexdigest(),
            review_bundle=_ref(private_dir, "evidence-manifest.v1.json"),
            private_score=_ref(private_dir, "score.v1.json"),
            transcript=_ref(private_dir, "adapter.transcript.ndjson"),
            tool_trace=_ref(private_dir, "tool-audit.json"),
            audit=_ref(private_dir, "container.log"),
        )
        return ConditionRun(run_id, mode, mode_root, evidence)
    except Exception as error:
        _retain_failure_record(
            topology.private if topology is not None else None, mode, broker, error
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
            "prediction.v1.json",
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
        return transaction, request, topology.private.path / "adapter.transcript.ndjson"
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
