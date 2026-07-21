# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0 (the "License").

"""Small rootless Podman adapter for the PI baseline with policy-only egress."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
import subprocess
from threading import Event, Lock, Thread
import time
from typing import IO, Literal, cast

from .scheduler_executor import ExecutionInterrupted
from .topology import PinnedRuntimeTopology


class PodmanSecurityError(RuntimeError):
    """Raised when the local Podman security preconditions are not met."""


class ContainerCleanupError(RuntimeError):
    """Raised when a container cannot be removed and verified safely."""

    reason = "container_cleanup_failed"


PodmanOperation = Literal[
    "info",
    "run",
    "create",
    "start",
    "exec",
    "logs",
    "remove",
    "exists",
    "remove_cleanup",
    "exists_cleanup",
]
DeadlineSource = Literal["controller_deadline", "command_cap"]
CaptureBucket = Literal["empty", "small", "bounded", "truncated"]
_PODMAN_OPERATIONS = frozenset(
    {
        "info",
        "run",
        "create",
        "start",
        "exec",
        "logs",
        "remove",
        "exists",
        "remove_cleanup",
        "exists_cleanup",
    }
)


class PodmanTimeoutError(TimeoutError):
    """A bounded Podman operation exceeded its deadline."""

    def __init__(
        self,
        operation: PodmanOperation,
        deadline_source: DeadlineSource,
        *,
        stdout_truncated: bool = False,
        stderr_truncated: bool = False,
        stdout_bucket: CaptureBucket = "empty",
        stderr_bucket: CaptureBucket = "empty",
    ) -> None:
        self.operation = operation
        self.deadline_source = deadline_source
        self.stdout_truncated = stdout_truncated
        self.stderr_truncated = stderr_truncated
        self.stdout_bucket = stdout_bucket
        self.stderr_bucket = stderr_bucket
        super().__init__("podman operation timed out")


class PodmanCompletedProcess(subprocess.CompletedProcess[str]):
    """A bounded process result with safe capture metadata."""

    def __init__(
        self,
        args: list[str],
        returncode: int,
        stdout: str,
        stderr: str,
        *,
        stdout_truncated: bool = False,
        stderr_truncated: bool = False,
    ) -> None:
        super().__init__(args, returncode, stdout, stderr)
        self.stdout_truncated = stdout_truncated
        self.stderr_truncated = stderr_truncated


_DIGEST = re.compile(r"^[a-z0-9][a-z0-9./_-]*@sha256:[0-9a-f]{64}$")
_IDENTIFIER = re.compile(r"^[a-z0-9][a-z0-9_.-]{0,62}$")
_CLEANUP_TIMEOUT_SECONDS = 30.0
_CLEANUP_COMMAND_TIMEOUT_SECONDS = 10.0


@dataclass(frozen=True)
class PodmanLimits:
    """Hard limits applied to one baseline invocation."""

    timeout_seconds: float = 300.0
    memory: str = "4g"
    cpus: str = "2"
    pids: int = 512
    output_bytes: int = 1_048_576


@dataclass(frozen=True)
class PodmanRun:
    """Inputs needed to construct a constrained baseline container invocation."""

    image: str
    run_id: str
    topology: PinnedRuntimeTopology
    args: tuple[str, ...] = ()
    limits: PodmanLimits = PodmanLimits()

    @property
    def input_dir(self) -> Path:
        return self.topology.input.path

    @property
    def workspace_dir(self) -> Path:
        return self.topology.workspace.path


class RootlessPodman:
    """Run constrained one-shot or persistent rootless Podman cases."""

    def __init__(self, executable: str = "podman") -> None:
        self.executable = executable

    def is_rootless(self, cancel_requested: Event) -> bool:
        result = _run_command(
            [self.executable, "info", "--format", "{{.Host.Security.Rootless}}"],
            check=True,
            timeout=10.0,
            cancel_requested=cancel_requested,
            operation="info",
        )
        return result.stdout.strip().lower() == "true"

    def command(self, request: PodmanRun) -> list[str]:
        self._validate(request)
        request.topology.verify()
        name = f"pi-baseline-{request.run_id}"
        limits = request.limits
        return [
            self.executable,
            "run",
            "--rm",
            "--name",
            name,
            "--read-only",
            "--userns=keep-id",
            "--ipc=private",
            "--uts=private",
            "--pid=private",
            "--cap-drop=ALL",
            "--security-opt=no-new-privileges",
            f"--memory={limits.memory}",
            f"--cpus={limits.cpus}",
            f"--pids-limit={limits.pids}",
            "--tmpfs",
            "/tmp:rw,size=64m,mode=1777",
            "--volume",
            f"{request.topology.input.path}:/input:ro,rprivate",
            "--volume",
            f"{request.topology.workspace.path}:/work:rw,rprivate",
            request.image,
            *request.args,
        ]

    def persistent(self, request: PodmanRun, cancel_requested: Event) -> PersistentPodmanCase:
        """Return the preferred per-case persistent container lifecycle."""

        self._validate(request)
        return PersistentPodmanCase(self, request, cancel_requested)

    def run(self, request: PodmanRun, cancel_requested: Event) -> subprocess.CompletedProcess[str]:
        if not self.is_rootless(cancel_requested):
            raise PodmanSecurityError("Podman did not report rootless operation")
        command = self.command(request)
        name = command[command.index("--name") + 1]
        try:
            result = _run_command(
                command,
                check=True,
                timeout=request.limits.timeout_seconds,
                cancel_requested=cancel_requested,
                operation="run",
            )
            return self._bounded(result, request.limits.output_bytes)
        finally:
            # --rm is not sufficient for interrupted or failed runtime setup.
            _run_command(
                [self.executable, "rm", "--force", name],
                check=False,
                timeout=10.0,
                cancel_requested=Event(),
                operation="remove",
            )

    def verify_removed(
        self,
        run_id: str,
        timeout_seconds: float = 10.0,
        *,
        operation: Literal["exists", "exists_cleanup"] = "exists",
    ) -> bool:
        """Return whether the named case no longer exists.

        A non-zero exit status is Podman's documented result for a missing
        container.  Other failures are surfaced so callers do not mistake an
        unavailable Podman service for successful cleanup.
        """

        name = f"pi-baseline-{run_id}"
        result = _run_command(
            [self.executable, "container", "exists", name],
            check=False,
            timeout=timeout_seconds,
            cancel_requested=Event(),
            operation=operation,
        )
        if result.returncode == 0:
            return False
        if result.returncode == 1:
            return True
        raise RuntimeError(
            f"could not verify removal of container {name!r} "
            f"(podman container exists exited {result.returncode})"
        )

    @staticmethod
    def _bounded(
        result: PodmanCompletedProcess | subprocess.CompletedProcess[str], limit: int
    ) -> PodmanCompletedProcess:
        stdout = result.stdout[:limit]
        stderr = result.stderr[:limit]
        return PodmanCompletedProcess(
            result.args,
            result.returncode,
            stdout,
            stderr,
            stdout_truncated=getattr(result, "stdout_truncated", False)
            or len(result.stdout.encode("utf-8")) > len(stdout.encode("utf-8")),
            stderr_truncated=getattr(result, "stderr_truncated", False)
            or len(result.stderr.encode("utf-8")) > len(stderr.encode("utf-8")),
        )

    @staticmethod
    def _validate(request: PodmanRun) -> None:
        if not _DIGEST.fullmatch(request.image):
            raise PodmanSecurityError("image must be pinned to an immutable sha256 digest")
        if not _IDENTIFIER.fullmatch(request.run_id):
            raise PodmanSecurityError("run_id must be a unique safe container identifier")
        try:
            request.topology.verify()
        except Exception as error:
            raise PodmanSecurityError("pinned runtime topology is invalid") from error
        if request.limits.timeout_seconds <= 0 or request.limits.output_bytes <= 0:
            raise PodmanSecurityError("timeout and output bound must be positive")
        if request.limits.pids <= 0:
            raise PodmanSecurityError("pids limit must be positive")


class PersistentPodmanCase:
    """One container reused for bounded analysis commands for one case."""

    def __init__(
        self, adapter: RootlessPodman, request: PodmanRun, cancel_requested: Event
    ) -> None:
        self.adapter = adapter
        self.request = request
        self.name = f"pi-baseline-{request.run_id}"
        self._started = False
        self._closed = False
        self._poisoned = False
        self.cancel_requested = cancel_requested

    @property
    def poisoned(self) -> bool:
        return self._poisoned

    def __enter__(self) -> PersistentPodmanCase:
        try:
            _check_cancel(self.cancel_requested)
            if not self.adapter.is_rootless(self.cancel_requested):
                raise PodmanSecurityError("Podman did not report rootless operation")
            _check_cancel(self.cancel_requested)
            # The validated absolute paths are the only container mounts.
            self.request.topology.verify()
            _run_command(
                self.create_command(),
                check=True,
                timeout=self.request.limits.timeout_seconds,
                cancel_requested=self.cancel_requested,
                operation="create",
            )
            _check_cancel(self.cancel_requested)
            _run_command(
                [self.adapter.executable, "start", self.name],
                check=True,
                timeout=self.request.limits.timeout_seconds,
                cancel_requested=self.cancel_requested,
                operation="start",
            )
            self._started = True
            return self
        except BaseException:
            self.close()
            raise

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.close()

    def create_command(self) -> list[str]:
        """Construct the persistent container creation command."""

        request = self.request
        limits = request.limits
        return [
            self.adapter.executable,
            "create",
            "--name",
            self.name,
            "--read-only",
            "--userns=keep-id",
            "--ipc=private",
            "--uts=private",
            "--pid=private",
            "--cap-drop=ALL",
            "--security-opt=no-new-privileges",
            f"--memory={limits.memory}",
            f"--cpus={limits.cpus}",
            f"--pids-limit={limits.pids}",
            "--tmpfs",
            "/tmp:rw,size=64m,mode=1777",
            "--volume",
            f"{request.topology.input.path}:/input:ro,rprivate",
            "--volume",
            f"{request.topology.workspace.path}:/work:rw,rprivate",
            request.image,
            "sleep",
            "infinity",
        ]

    def exec(
        self, command: str, *, controller_deadline: float | None = None
    ) -> PodmanCompletedProcess:
        """Execute one shell command inside this case's existing container."""

        if not self._started or self._closed:
            raise RuntimeError("persistent Podman case is not running")
        try:
            result = _run_command(
                [
                    self.adapter.executable,
                    "exec",
                    "--workdir",
                    "/work",
                    self.name,
                    "sh",
                    "-lc",
                    command,
                ],
                check=False,
                timeout=self.request.limits.timeout_seconds,
                controller_deadline=controller_deadline,
                capture_limit=self.request.limits.output_bytes,
                cancel_requested=self.cancel_requested,
                operation="exec",
            )
            return self.adapter._bounded(result, self.request.limits.output_bytes)
        except PodmanTimeoutError:
            self._poisoned = True
            self.close()
            raise

    def logs(self) -> PodmanCompletedProcess:
        """Collect bounded container logs before the case is closed."""

        result = _run_command(
            [self.adapter.executable, "logs", self.name],
            check=True,
            timeout=self.request.limits.timeout_seconds,
            cancel_requested=self.cancel_requested,
            operation="logs",
        )
        return self.adapter._bounded(result, self.request.limits.output_bytes)

    def close(self) -> None:
        """Immediately remove the case, even when creation or execution failed."""

        if self._closed:
            return
        deadline = time.monotonic() + _CLEANUP_TIMEOUT_SECONDS
        errors: list[str] = []
        removed = False
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            command_timeout = min(_CLEANUP_COMMAND_TIMEOUT_SECONDS, remaining)
            try:
                _run_command(
                    [self.adapter.executable, "rm", "--force", "--time", "0", self.name],
                    check=False,
                    timeout=command_timeout,
                    cancel_requested=Event(),
                    operation="remove_cleanup",
                )
            except BaseException:
                errors.append("force removal failed")

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                removed = self.adapter.verify_removed(
                    self.request.run_id,
                    timeout_seconds=min(_CLEANUP_COMMAND_TIMEOUT_SECONDS, remaining),
                    operation="exists_cleanup",
                )
            except BaseException:
                errors.append("removal verification failed")
            if removed:
                self._closed = True
                break
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))

        if not removed:
            raise ContainerCleanupError(
                "failed to remove container (container_cleanup_failed)"
            ) from None


def _check_cancel(cancel_requested: Event) -> None:
    if cancel_requested.is_set():
        raise ExecutionInterrupted


def _run_command(
    command: list[str],
    *,
    check: bool,
    timeout: float,
    controller_deadline: float | None = None,
    capture_limit: int = 1_048_576,
    cancel_requested: Event,
    operation: PodmanOperation,
) -> PodmanCompletedProcess:
    """Run Podman with short polling when cooperative cancellation is enabled."""
    if operation not in _PODMAN_OPERATIONS:
        raise ValueError("invalid Podman operation label")
    _check_cancel(cancel_requested)
    if capture_limit <= 0:
        raise ValueError("capture limit must be positive")
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    stdout_capture = _Capture(capture_limit)
    stderr_capture = _Capture(capture_limit)
    stdout_stream = getattr(process, "stdout", None)
    stderr_stream = getattr(process, "stderr", None)
    stdout_thread = Thread(target=_drain_stream, args=(stdout_stream, stdout_capture), daemon=True)
    stderr_thread = Thread(target=_drain_stream, args=(stderr_stream, stderr_capture), daemon=True)
    stdout_thread.start()
    stderr_thread.start()
    started = time.monotonic()
    command_deadline = started + timeout
    deadline = command_deadline
    deadline_source: DeadlineSource = "command_cap"
    if controller_deadline is not None and controller_deadline <= deadline:
        deadline = controller_deadline
        deadline_source = "controller_deadline"
    timed_out = False
    try:
        while process.poll() is None:
            _check_cancel_process(process, cancel_requested)
            if time.monotonic() >= deadline:
                timed_out = True
                _terminate_process(process)
                break
            cancel_requested.wait(min(0.1, max(0.0, deadline - time.monotonic())))
        if process.poll() is None:
            _terminate_process(process)
        process.wait(timeout=1.0)
    finally:
        _join_drainer(stdout_thread, stdout_stream)
        _join_drainer(stderr_thread, stderr_stream)
    if stdout_stream is None or stderr_stream is None:
        communicate = getattr(process, "communicate", None)
        fallback_stdout, fallback_stderr = (
            cast("tuple[str, str]", communicate()) if callable(communicate) else ("", "")
        )
        if stdout_stream is None:
            stdout_capture.append(fallback_stdout or "")
        if stderr_stream is None:
            stderr_capture.append(fallback_stderr or "")
    stdout, stderr = stdout_capture.value(), stderr_capture.value()
    if timed_out:
        raise PodmanTimeoutError(
            operation,
            deadline_source,
            stdout_truncated=stdout_capture.truncated,
            stderr_truncated=stderr_capture.truncated,
            stdout_bucket=stdout_capture.bucket,
            stderr_bucket=stderr_capture.bucket,
        )
    result = PodmanCompletedProcess(
        command,
        process.returncode,
        stdout,
        stderr,
        stdout_truncated=stdout_capture.truncated,
        stderr_truncated=stderr_capture.truncated,
    )
    if check and result.returncode:
        raise subprocess.CalledProcessError(
            result.returncode, command, output=stdout, stderr=stderr
        )
    return result


class _Capture:
    def __init__(self, limit: int) -> None:
        self.limit = limit
        self.parts: list[str] = []
        self.size = 0
        self.truncated = False
        self.lock = Lock()

    @property
    def bucket(self) -> CaptureBucket:
        if self.truncated:
            return "truncated"
        if self.size == 0:
            return "empty"
        if self.size <= 4096:
            return "small"
        return "bounded"

    def append(self, chunk: str) -> None:
        with self.lock:
            encoded = chunk.encode("utf-8", errors="replace")
            remaining = self.limit - self.size
            if remaining <= 0:
                self.truncated = True
                return
            if len(encoded) > remaining:
                self.parts.append(encoded[:remaining].decode("utf-8", errors="ignore"))
                self.size = self.limit
                self.truncated = True
            else:
                self.parts.append(chunk)
                self.size += len(encoded)

    def value(self) -> str:
        with self.lock:
            return "".join(self.parts)


def _drain_stream(stream: IO[str] | None, capture: _Capture) -> None:
    if stream is None:
        return
    while True:
        chunk = stream.read(16 * 1024)
        if not chunk:
            return
        capture.append(chunk)


def _join_drainer(thread: Thread, stream: IO[str] | None) -> None:
    thread.join(timeout=1.0)
    if thread.is_alive() and stream is not None:
        stream.close()
        thread.join(timeout=1.0)


def _check_cancel_process(process: subprocess.Popen[str], cancel_requested: Event) -> None:
    if cancel_requested.is_set():
        _terminate_process(process)
        raise ExecutionInterrupted


def _terminate_process(process: subprocess.Popen[str]) -> None:
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)


# Short name useful to callers, while keeping the security properties explicit.
PodmanAdapter = RootlessPodman
