"""Pi adapter for the agent-neutral scheduler executor boundary."""

from __future__ import annotations

from collections.abc import Callable
import errno
import hashlib
import os
from pathlib import Path
import subprocess
import threading
import traceback
from typing import cast

from dimos.benchmark.spatial.models import SpatialModel
from dimos.benchmark.spatial.pi_baseline.broker import PolicyViolationError
from dimos.benchmark.spatial.pi_baseline.config import (
    PiBaselineConfig,
    PromptMode,
    PublicSelection,
)
from dimos.benchmark.spatial.pi_baseline.controller import AdapterCleanupError, AdapterRunError
from dimos.benchmark.spatial.pi_baseline.podman import (
    ContainerCleanupError,
    PodmanInfrastructureError,
    PodmanSecurityError,
)
from dimos.benchmark.spatial.pi_baseline.projection import DescriptorMismatchError
from dimos.benchmark.spatial.pi_baseline.runner import ConditionRun, run_condition
from dimos.benchmark.spatial.pi_baseline.scheduler_executor import (
    EventSink,
    ExecutionInterrupted,
    Executor,
)
from dimos.benchmark.spatial.pi_baseline.scheduler_models import (
    AttemptContext,
    ExecutorArtifactEvent,
    ExecutorProgressEvent,
    ExpandedCase,
    NamedCondition,
    TerminalOutcome,
)
from dimos.benchmark.spatial.pi_baseline.scheduler_pi_binding import (
    PiAdmissionContext,
    PiExecutionSnapshot,
    PiRuntimeBindings,
    reconstruct_config,
    verify_public_inputs,
    verify_snapshot_artifacts,
)
from dimos.benchmark.spatial.pi_baseline.topology import TopologyError
from dimos.benchmark.spatial.utilities import JsonValue, canonical_json


class PiCasePayload(SpatialModel):
    """Validated scheduler payload required to execute a Pi case."""

    selection: PublicSelection


class PiConditionPayload(SpatialModel):
    """Validated Pi condition payload."""

    prompt_mode: PromptMode


ConditionRunner = Callable[
    [PiBaselineConfig, PromptMode, threading.Event, threading.Lock], ConditionRun
]


def _default_condition_runner(
    config: PiBaselineConfig,
    mode: PromptMode,
    cancel_requested: threading.Event,
    publication_lock: threading.Lock,
) -> ConditionRun:
    return run_condition(
        config,
        mode=mode,
        cancel_requested=cancel_requested,
        publication_lock=publication_lock,
    )


class PiSchedulerExecutor(Executor):
    """Adapt one scheduler job into one isolated Pi condition execution."""

    def __init__(
        self,
        snapshot: PiExecutionSnapshot,
        runtime_bindings: PiRuntimeBindings,
        *,
        manifest_executor_fingerprint: str,
        manifest_selected_inputs_digest: str | None = None,
        admission_context: PiAdmissionContext | None = None,
        condition_runner: ConditionRunner = _default_condition_runner,
    ) -> None:
        if snapshot.canonical_digest() != manifest_executor_fingerprint:
            raise ValueError("Pi execution snapshot does not match executor fingerprint")
        verify_snapshot_artifacts(snapshot)
        if manifest_selected_inputs_digest is not None:
            if snapshot.pi_selected_inputs_digest != manifest_selected_inputs_digest:
                raise ValueError("Pi selected inputs do not match Pi binding")
        self._snapshot = snapshot
        self._runtime_bindings = runtime_bindings
        self._condition_runner = condition_runner
        self._admission_context = admission_context or PiAdmissionContext.from_snapshot(snapshot)
        if self._admission_context.snapshot_digest != snapshot.canonical_digest():
            raise ValueError("Pi admission context does not match execution snapshot")

    def run(
        self,
        case: ExpandedCase,
        condition: NamedCondition,
        context: AttemptContext,
        emit: EventSink,
        cancel_requested: threading.Event,
        publication_lock: threading.Lock,
    ) -> TerminalOutcome:
        """Validate Pi payloads, execute one condition, and normalize its result."""
        _check_cancel(cancel_requested)
        parsed_case = PiCasePayload.model_validate(case.payload)
        parsed_condition = PiConditionPayload.model_validate(condition.payload)
        # All immutable material and public inputs are checked before any
        # lifecycle event or adapter process can be created.
        if self._snapshot.selected_inputs:
            _check_cancel(cancel_requested)
            recorded = self._admission_context.selected(case.case_id)
            if recorded.selection != parsed_case.selection:
                raise ValueError("Pi case is not one of the immutable selected inputs")
            verify_public_inputs(
                self._snapshot,
                self._runtime_bindings.corpus_root,
                selected_input=recorded,
            )
        _check_cancel(cancel_requested)
        emit(ExecutorProgressEvent(kind="progress", code="executor_progress"))
        condition_run_id = _condition_run_id(context)
        attempt_public_root = (
            self._runtime_bindings.public_root
            / context.identity.experiment_id
            / context.identity.job_id
            / context.attempt_id
            / "public"
        )
        attempt_private_root = (
            self._runtime_bindings.private_root
            / context.identity.experiment_id
            / context.identity.job_id
            / context.attempt_id
            / "private"
        )
        bindings = self._runtime_bindings.model_copy(
            update={
                "public_root": attempt_public_root,
                "private_root": attempt_private_root,
            }
        )
        config = reconstruct_config(
            self._snapshot,
            bindings,
            selection=parsed_case.selection,
            mode=parsed_condition.prompt_mode,
            case_id=case.case_id,
            run_id=condition_run_id,
        )
        try:
            _check_cancel(cancel_requested)
            result = _invoke_condition_runner(
                self._condition_runner,
                config,
                parsed_condition.prompt_mode,
                cancel_requested,
                publication_lock,
            )
        except ExecutionInterrupted:
            raise
        except Exception as error:
            _retain_scheduler_failure_record(
                attempt_private_root,
                parsed_condition.prompt_mode,
                error,
            )
            outcome = TerminalOutcome(
                status="failed",
                reason=_failure_reason(error),
                policy_telemetry=(
                    error.policy_telemetry if isinstance(error, AdapterRunError) else None
                ),
            )
            return outcome

        _check_cancel(cancel_requested)
        emit(
            ExecutorArtifactEvent(
                kind="artifact",
                code="artifact_recorded",
                artifact_id=f"evidence-{context.attempt_id}",
                artifact_sha256=result.evidence.manifest_sha256,
            )
        )
        return TerminalOutcome(status="succeeded", reason="Pi condition completed")


def _failure_reason(error: Exception) -> str:
    if isinstance(error, ContainerCleanupError):
        return ContainerCleanupError.reason
    if isinstance(error, AdapterCleanupError):
        # Preserve the established public cleanup code; never expose the
        # controller's process/path details.
        return ContainerCleanupError.reason
    if isinstance(error, PodmanInfrastructureError):
        return PodmanInfrastructureError.reason
    if isinstance(error, AdapterRunError):
        if error.terminal_reason in {
            "post_image_policy_violation",
            "pre_image_policy_violation",
        }:
            return error.terminal_reason
        if error.terminal_reason is not None:
            return f"adapter_run_failed:{error.terminal_reason}"
        adapter_code = str(error)
        if adapter_code in {
            "adapter_protocol_error",
            "adapter_startup_error",
            "adapter_host_deadline",
            "adapter_submitted_without_answer",
            "adapter_eof_before_complete",
            "adapter_run_failed",
        }:
            return adapter_code
        return "adapter_run_error"
    if isinstance(error, PodmanSecurityError):
        return "podman_security_error"
    if isinstance(error, TopologyError):
        return "topology_error"
    if isinstance(error, PolicyViolationError):
        if error.code in {"visualization_forbidden", "visualization_required_before_submission"}:
            return f"policy:{error.code}"
        return "policy_violation"
    return "executor_failed"


def _retain_scheduler_failure_record(
    private_root: Path,
    mode: PromptMode,
    error: Exception,
) -> None:
    """Best-effort, redacted retention for failures before runner retention."""
    directory_fd: int | None = None
    file_fd: int | None = None
    try:
        path = private_root.expanduser()
        directory_flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
        directory_fd = os.open("/" if path.is_absolute() else ".", directory_flags)
        components = path.parts[1:] if path.is_absolute() else path.parts
        for component in components:
            if component in {"", ".", ".."}:
                raise OSError("unsafe private directory component")
            try:
                child_fd = os.open(component, directory_flags, dir_fd=directory_fd)
            except FileNotFoundError:
                os.mkdir(component, 0o700, dir_fd=directory_fd)
                child_fd = os.open(component, directory_flags, dir_fd=directory_fd)
            os.close(directory_fd)
            directory_fd = child_fd

        record: dict[str, JsonValue] = {
            "record_type": "pi-run-failure",
            "schema_version": "1.0",
            "mode": mode,
            "error": type(error).__name__,
            "message": _private_failure_message(error),
            "traceback": cast("JsonValue", _structural_traceback(error)),
            "scoring_eligible": False,
            "ledger_appended": False,
        }
        if isinstance(error, OSError):
            record["errno"] = error.errno
            record["error_label"] = _os_error_label(error.errno)
        if type(error) is DescriptorMismatchError:
            record["descriptor_mismatch"] = {
                "expected_entry_count": error.expected_entry_count,
                "actual_entry_count": error.actual_entry_count,
                "actual_entries_sha256": error.actual_entries_sha256,
            }
        if isinstance(error, AdapterRunError) and error.terminal_reason is not None:
            record["adapter_reason"] = error.terminal_reason
        subprocess_diagnostic = _subprocess_failure_diagnostic(error)
        if subprocess_diagnostic is not None:
            record["subprocess"] = subprocess_diagnostic
        payload = canonical_json(cast("JsonValue", record)) + b"\n"
        file_fd = os.open(
            "failure.v1.json",
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
            0o600,
            dir_fd=directory_fd,
        )
        offset = 0
        while offset < len(payload):
            written = os.write(file_fd, payload[offset:])
            if written <= 0:
                raise OSError("failure record write made no progress")
            offset += written
        os.fsync(file_fd)
        os.fsync(directory_fd)
    except Exception:
        # Private diagnostics must never change the public execution result.
        return
    finally:
        if file_fd is not None:
            try:
                os.close(file_fd)
            except OSError:
                pass
        if directory_fd is not None:
            try:
                os.close(directory_fd)
            except OSError:
                pass


def _private_failure_message(error: Exception) -> str:
    if isinstance(error, OSError):
        return "os error"
    return "scheduler exception"


def _subprocess_failure_diagnostic(
    error: Exception,
) -> dict[str, JsonValue] | None:
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


def _os_error_label(errno_value: int | None) -> str:
    if errno_value is None:
        return "os_error"
    labels = {
        errno.ENOENT: "not_found",
        errno.EACCES: "permission_denied",
        errno.EEXIST: "already_exists",
        errno.ELOOP: "symlink_loop",
    }
    return labels.get(errno_value, "os_error")


def _structural_traceback(error: Exception) -> list[dict[str, JsonValue]]:
    frames: list[dict[str, JsonValue]] = []
    for frame, line_number in traceback.walk_tb(error.__traceback__):
        module = frame.f_globals.get("__name__")
        frames.append(
            {
                "module": module if isinstance(module, str) else "unknown",
                "function": frame.f_code.co_name,
                "line": line_number,
            }
        )
    return frames


def _check_cancel(cancel_requested: threading.Event) -> None:
    if cancel_requested.is_set():
        raise ExecutionInterrupted


def _invoke_condition_runner(
    runner: ConditionRunner,
    config: PiBaselineConfig,
    mode: PromptMode,
    cancel_requested: threading.Event,
    publication_lock: threading.Lock,
) -> ConditionRun:
    return runner(config, mode, cancel_requested, publication_lock)


def _condition_run_id(context: AttemptContext) -> str:
    """Derive an isolated, deterministic host/container identity for an attempt."""
    identity = "\x1f".join(
        (
            context.identity.experiment_id,
            context.identity.case_id,
            context.identity.condition_name,
            context.identity.job_id,
            context.attempt_id,
            str(context.attempt_number),
        )
    )
    # The runner appends a mode suffix before building the Podman request.
    # Reserve room for the longest suffix within Podman's 63-character limit.
    return f"pi-{hashlib.sha256(identity.encode('utf-8')).hexdigest()[:49]}"
