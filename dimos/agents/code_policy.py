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

"""Trusted agent-authored Python execution backed by a persistent Jupyter kernel."""

from __future__ import annotations

import base64
from datetime import UTC, datetime
import os
import re
import threading
import time
from typing import Annotated, Any, Literal
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MAX_EXECUTION_TIMEOUT_S = 110.0
DEFAULT_STARTUP_TIMEOUT_S = 10.0
DEFAULT_INTERRUPT_GRACE_S = 2.0
DEFAULT_OUTPUT_LIMIT = 32_000
_RECORDING_PATH_ENV = "DIMOS_CODE_POLICY_RECORDING_PATH"
_TRUNCATION_MARKER = "\n... [output truncated]"
_ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")

SessionId = Annotated[str, Field(pattern=r"^code_policy_session_[0-9a-f]{32}$")]
ExecutionId = Annotated[str, Field(pattern=r"^code_policy_call_[0-9a-f]{32}$")]
ExecutionStatus = Literal[
    "busy",
    "completed",
    "execution-failed",
    "invalid-request",
    "kernel-start-failed",
    "module-stopped",
    "python-error",
    "timed-out",
]
ObserverAvailability = Literal["ready", "replaced", "stopped", "unavailable"]


class _EvidenceModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)


class CodePolicySessionReceipt(_EvidenceModel):
    """Identity of a freshly reset persistent Python namespace."""

    session_id: SessionId
    reset_at: datetime
    previous_session_id: SessionId | None


class CodePolicyExecutionRecord(_EvidenceModel):
    """Structured evidence for one accepted or rejected ``python_exec`` call."""

    execution_id: ExecutionId
    session_id: SessionId
    source: str
    requested_timeout_s: float
    started_at: datetime
    finished_at: datetime
    monotonic_duration_s: Annotated[float, Field(ge=0)]
    status: ExecutionStatus
    jupyter_message_id: str | None
    jupyter_execution_count: int | None
    output: str
    transcript: str
    interrupt_attempted: bool
    interrupt_recovered: bool
    kernel_restarted: bool
    namespace_preserved: bool
    remote_work_may_continue: bool


class CodePolicyObserverDescriptor(_EvidenceModel):
    """Minimal credentials required to verify and read one IOPub generation."""

    transport: Literal["tcp", "ipc"]
    ip: str
    iopub_port: Annotated[int, Field(gt=0)]
    signature_scheme: str
    key_base64: str
    code_policy_session_id: SessionId
    jupyter_client_session_id: str
    kernel_generation: Annotated[int, Field(gt=0)]


class CodePolicyObserverState(_EvidenceModel):
    """Current host-side observation lifecycle without stale credentials."""

    availability: ObserverAvailability
    code_policy_session_id: SessionId
    kernel_generation: Annotated[int, Field(ge=0)]
    descriptor: CodePolicyObserverDescriptor | None


class CodePolicyObserverProbeReceipt(_EvidenceModel):
    """Identity of the fixed silent IOPub readiness probe."""

    message_id: str
    code_policy_session_id: SessionId
    kernel_generation: Annotated[int, Field(gt=0)]


class CodePolicyConfig(ModuleConfig):
    recording_path: str
    output_limit: int = DEFAULT_OUTPUT_LIMIT
    startup_timeout_s: float = DEFAULT_STARTUP_TIMEOUT_S
    interrupt_grace_s: float = DEFAULT_INTERRUPT_GRACE_S


class _BoundedTextOutput:
    """Collect bounded plain-text portions of Jupyter output messages."""

    def __init__(self, limit: int) -> None:
        self._limit = max(0, limit)
        self._parts: list[str] = []
        self._length = 0
        self._truncated = False

    def __call__(self, message: dict[str, Any]) -> None:
        message_type = message.get("header", {}).get("msg_type")
        content = message.get("content", {})
        if message_type == "stream":
            self._append(str(content.get("text", "")))
        elif message_type in {"execute_result", "display_data"}:
            text = content.get("data", {}).get("text/plain")
            if text is not None:
                self._append(str(text))
        elif message_type == "error":
            traceback = content.get("traceback")
            if isinstance(traceback, list):
                self._append("\n".join(str(line) for line in traceback))
            else:
                self._append(f"{content.get('ename', 'Error')}: {content.get('evalue', '')}")

    def text(self) -> str:
        return "".join(self._parts)

    def _append(self, value: str) -> None:
        if not value or self._truncated:
            return
        value = _ANSI_ESCAPE_RE.sub("", value)
        remaining = self._limit - self._length
        if len(value) <= remaining:
            self._parts.append(value)
            self._length += len(value)
            return

        marker = _TRUNCATION_MARKER[:remaining]
        content_limit = max(0, remaining - len(marker))
        self._parts.append(value[:content_limit] + marker)
        self._length = self._limit
        self._truncated = True


def _load_kernel_manager() -> type[Any]:
    try:
        from jupyter_client.manager import KernelManager
    except ImportError as exc:
        raise RuntimeError(
            "Code-policy execution requires the agents extra: uv sync --extra agents"
        ) from exc
    return KernelManager


def _bootstrap_source() -> str:
    return f"""
import os as _os
from dimos.memory2.store.sqlite import SqliteStore as _SqliteStore
from dimos.porcelain.dimos import Dimos as _Dimos

app = _Dimos.connect()
memory = _SqliteStore(path=_os.environ[{_RECORDING_PATH_ENV!r}], must_exist=True)
memory.start()

del _os, _SqliteStore, _Dimos
"""


class CodePolicyModule(Module):
    """Execute trusted, unsandboxed agent-authored Python against DimOS."""

    config: CodePolicyConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._execution_lock: threading.Lock | None = None
        self._records_lock = threading.Lock()
        self._kernel_lock = threading.RLock()
        self._kernel_manager: Any = None
        self._kernel_client: Any = None
        self._kernel_generation = 0
        self._session_id: str = _new_session_id()
        self._session_reset_at = _utc_now()
        self._stopped = True
        self._execution_records: list[CodePolicyExecutionRecord] = []

    @rpc
    def start(self) -> None:
        self._execution_lock = threading.Lock()
        self._stopped = False
        super().start()

    @skill
    def python_exec(self, code: str, timeout_s: float = MAX_EXECUTION_TIMEOUT_S) -> str:
        """Execute one synchronous Python program in the persistent policy session.

        The trusted, unsandboxed session preloads `app` for deployed DimOS RPCs
        and `memory` for current and historical observations. Imports, functions,
        variables, and mutations persist across calls until the host resets the
        session. Use this for observation processing, control flow, retries, and
        coordinated multi-RPC behavior.

        Args:
            code: Complete Python source for one task attempt.
            timeout_s: Execution deadline in seconds, greater than 0 and at most 110.
        """
        started_at = _utc_now()
        started_monotonic = time.monotonic()
        if self._stopped:
            transcript = "Code Policy Module stopped"
            self._record(
                code,
                timeout_s,
                started_at,
                started_monotonic,
                status="module-stopped",
                transcript=transcript,
            )
            return transcript
        if not 0 < timeout_s <= MAX_EXECUTION_TIMEOUT_S:
            transcript = (
                f"Invalid timeout_s={timeout_s!r}; expected a value in "
                f"(0, {MAX_EXECUTION_TIMEOUT_S:g}]"
            )
            self._record(
                code,
                timeout_s,
                started_at,
                started_monotonic,
                status="invalid-request",
                transcript=transcript,
            )
            return transcript

        lock = self._execution_lock
        if lock is None:
            lock = self._execution_lock = threading.Lock()
        if not lock.acquire(blocking=False):
            transcript = "Code Policy Module busy: another python_exec call is active"
            self._record(
                code,
                timeout_s,
                started_at,
                started_monotonic,
                status="busy",
                transcript=transcript,
            )
            return transcript

        logger.info("Code policy execution started", source=code, timeout_s=timeout_s)
        try:
            try:
                client = self._ensure_kernel()
            except Exception as exc:
                logger.exception("Code policy kernel failed to start")
                transcript = f"Code policy kernel failed to start: {type(exc).__name__}: {exc}"
                self._record(
                    code,
                    timeout_s,
                    started_at,
                    started_monotonic,
                    status="kernel-start-failed",
                    transcript=transcript,
                )
                return transcript

            output = _BoundedTextOutput(self.config.output_limit)
            try:
                reply = client.execute_interactive(
                    code,
                    allow_stdin=False,
                    output_hook=output,
                    store_history=True,
                    timeout=timeout_s,
                )
            except TimeoutError:
                interrupt_recovered, kernel_restarted = self._recover_from_timeout()
                if interrupt_recovered:
                    transcript = (
                        f"Execution timed out after {timeout_s:.1f}s and was interrupted. "
                        "The Python namespace was preserved. Remote RPC work may still be "
                        "running; it was not cancelled."
                    )
                else:
                    transcript = (
                        f"Execution timed out after {timeout_s:.1f}s. The kernel did not "
                        "recover from interruption and was restarted; the Python namespace "
                        "was reset. Remote RPC work may still be running; it was not cancelled."
                    )
                self._record(
                    code,
                    timeout_s,
                    started_at,
                    started_monotonic,
                    status="timed-out",
                    output=output.text(),
                    transcript=transcript,
                    interrupt_attempted=True,
                    interrupt_recovered=interrupt_recovered,
                    kernel_restarted=kernel_restarted,
                    namespace_preserved=interrupt_recovered,
                    remote_work_may_continue=True,
                )
                return transcript
            except Exception as exc:
                self._shutdown_kernel(reason=type(exc).__name__)
                logger.exception("Code policy execution failed")
                transcript = (
                    f"Code policy execution failed: {type(exc).__name__}: {exc}. "
                    "The Python namespace was reset."
                )
                self._record(
                    code,
                    timeout_s,
                    started_at,
                    started_monotonic,
                    status="execution-failed",
                    output=output.text(),
                    transcript=transcript,
                    kernel_restarted=True,
                )
                return transcript

            duration_s = time.monotonic() - started_monotonic
            transcript = _format_reply(reply, output.text(), duration_s)
            content = reply.get("content", {})
            status: ExecutionStatus = (
                "completed" if content.get("status") == "ok" else "python-error"
            )
            self._record(
                code,
                timeout_s,
                started_at,
                started_monotonic,
                status=status,
                output=output.text(),
                transcript=transcript,
                jupyter_message_id=reply.get("parent_header", {}).get("msg_id"),
                jupyter_execution_count=content.get("execution_count"),
            )
            return transcript
        finally:
            lock.release()

    @rpc
    def reset_session(self) -> CodePolicySessionReceipt:
        """Destroy the kernel namespace and issue a fresh opaque session identity."""
        lock = self._execution_lock
        if lock is None:
            lock = self._execution_lock = threading.Lock()
        if not lock.acquire(blocking=False):
            raise RuntimeError("cannot reset code policy while python_exec is active")
        try:
            previous_session_id = self._session_id
            self._shutdown_kernel(reason="session reset")
            self._session_id = _new_session_id()
            self._session_reset_at = _utc_now()
            return CodePolicySessionReceipt(
                session_id=self._session_id,
                reset_at=self._session_reset_at,
                previous_session_id=previous_session_id,
            )
        finally:
            lock.release()

    @rpc
    def get_session_receipt(self) -> CodePolicySessionReceipt:
        return CodePolicySessionReceipt(
            session_id=self._session_id,
            reset_at=self._session_reset_at,
            previous_session_id=None,
        )

    @rpc
    def get_execution_records(
        self, session_id: str | None = None
    ) -> tuple[CodePolicyExecutionRecord, ...]:
        with self._records_lock:
            records = tuple(self._execution_records)
        if session_id is None:
            return records
        return tuple(record for record in records if record.session_id == session_id)

    @rpc
    def prepare_observer(self) -> CodePolicyObserverState:
        """Prepare the policy kernel and return its minimized read-only descriptor."""
        if self._stopped:
            return self._observer_state("stopped")
        self._ensure_kernel()
        return self._observer_state("ready")

    @rpc
    def get_observer_state(self, known_generation: int | None = None) -> CodePolicyObserverState:
        """Return current observation state without creating a kernel."""
        if self._stopped:
            return self._observer_state("stopped")
        with self._kernel_lock:
            manager = self._kernel_manager
            client = self._kernel_client
            generation = self._kernel_generation
            is_ready = manager is not None and client is not None and bool(manager.is_alive())
        if not is_ready:
            return self._observer_state("unavailable")
        availability: ObserverAvailability = (
            "replaced"
            if known_generation is not None and known_generation != generation
            else "ready"
        )
        return self._observer_state(availability)

    @rpc
    def issue_observer_probe(self, kernel_generation: int) -> CodePolicyObserverProbeReceipt:
        """Emit the fixed silent, history-free readiness probe for one generation."""
        if self._stopped:
            raise RuntimeError("code policy module is stopped")
        lock = self._execution_lock
        if lock is None:
            lock = self._execution_lock = threading.Lock()
        if not lock.acquire(blocking=False):
            raise RuntimeError("cannot probe while python_exec is active")
        try:
            client = self._ensure_kernel()
            with self._kernel_lock:
                if kernel_generation != self._kernel_generation:
                    raise RuntimeError(
                        "code policy kernel generation changed before readiness probe"
                    )
                message_id = client.execute(
                    "None",
                    silent=True,
                    store_history=False,
                    allow_stdin=False,
                )
                return CodePolicyObserverProbeReceipt(
                    message_id=message_id,
                    code_policy_session_id=self._session_id,
                    kernel_generation=self._kernel_generation,
                )
        finally:
            lock.release()

    @rpc
    def interrupt_active(self) -> bool:
        """Best-effort interrupt of the currently executing policy cell."""
        lock = self._execution_lock
        if lock is None:
            return False
        if lock.acquire(blocking=False):
            lock.release()
            return False
        manager = self._kernel_manager
        if manager is None or not manager.is_alive():
            return False
        manager.interrupt_kernel()
        return True

    @rpc
    def stop(self) -> None:
        self._stopped = True
        self._shutdown_kernel(reason="module stop")
        super().stop()

    def _record(
        self,
        source: str,
        timeout_s: float,
        started_at: datetime,
        started_monotonic: float,
        *,
        status: ExecutionStatus,
        output: str = "",
        transcript: str,
        jupyter_message_id: str | None = None,
        jupyter_execution_count: int | None = None,
        interrupt_attempted: bool = False,
        interrupt_recovered: bool = False,
        kernel_restarted: bool = False,
        namespace_preserved: bool = True,
        remote_work_may_continue: bool = False,
    ) -> None:
        record = CodePolicyExecutionRecord(
            execution_id=f"code_policy_call_{uuid4().hex}",
            session_id=self._session_id,
            source=source,
            requested_timeout_s=timeout_s,
            started_at=started_at,
            finished_at=_utc_now(),
            monotonic_duration_s=max(0.0, time.monotonic() - started_monotonic),
            status=status,
            jupyter_message_id=jupyter_message_id,
            jupyter_execution_count=jupyter_execution_count,
            output=output,
            transcript=transcript,
            interrupt_attempted=interrupt_attempted,
            interrupt_recovered=interrupt_recovered,
            kernel_restarted=kernel_restarted,
            namespace_preserved=namespace_preserved,
            remote_work_may_continue=remote_work_may_continue,
        )
        with self._records_lock:
            self._execution_records.append(record)

    def _ensure_kernel(self) -> Any:
        with self._kernel_lock:
            manager = self._kernel_manager
            client = self._kernel_client
            if manager is not None and client is not None and manager.is_alive():
                return client
            self._shutdown_kernel(reason="kernel unavailable")

            manager_type = _load_kernel_manager()
            manager = manager_type(kernel_name="python3")
            client = None
            try:
                env = os.environ.copy()
                env[_RECORDING_PATH_ENV] = self.config.recording_path
                manager.start_kernel(env=env)
                client = manager.client()
                client.start_channels()
                client.wait_for_ready(timeout=self.config.startup_timeout_s)
                self._bootstrap(client)
            except Exception:
                if client is not None:
                    client.stop_channels()
                try:
                    manager.shutdown_kernel(now=True)
                except Exception:
                    logger.exception("Failed to stop an uninitialized code policy kernel")
                raise

            self._kernel_manager = manager
            self._kernel_client = client
            self._kernel_generation += 1
        logger.info("Code policy kernel started", recording_path=self.config.recording_path)
        return client

    def _bootstrap(self, client: Any) -> None:
        reply = client.execute_interactive(
            _bootstrap_source(),
            allow_stdin=False,
            output_hook=lambda _message: None,
            silent=True,
            store_history=False,
            timeout=self.config.startup_timeout_s,
        )
        content = reply.get("content", {})
        if content.get("status") != "ok":
            name = content.get("ename", "KernelBootstrapError")
            value = content.get("evalue", "unknown bootstrap failure")
            raise RuntimeError(f"{name}: {value}")

    def _recover_from_timeout(self) -> tuple[bool, bool]:
        manager = self._kernel_manager
        client = self._kernel_client
        if manager is None or client is None:
            return False, False
        try:
            manager.interrupt_kernel()
            client.wait_for_ready(timeout=self.config.interrupt_grace_s)
            logger.info("Code policy kernel recovered after interrupt")
            return True, False
        except Exception:
            logger.warning("Code policy kernel did not recover after interrupt")

        try:
            manager.restart_kernel(now=True)
            client.wait_for_ready(timeout=self.config.startup_timeout_s)
            self._bootstrap(client)
            with self._kernel_lock:
                self._kernel_generation += 1
            logger.info("Code policy kernel restarted after failed interrupt")
            return False, True
        except Exception:
            logger.exception("Code policy kernel failed to restart")
            self._shutdown_kernel(reason="restart failure")
            return False, True

    def _shutdown_kernel(self, *, reason: str) -> None:
        with self._kernel_lock:
            manager = self._kernel_manager
            client = self._kernel_client
            self._kernel_manager = None
            self._kernel_client = None
        if manager is None and client is None:
            return
        try:
            if manager is not None:
                manager.shutdown_kernel(now=True)
        except Exception:
            logger.exception("Failed to stop code policy kernel")
        finally:
            if client is not None:
                client.stop_channels()
        logger.info("Code policy kernel stopped", reason=reason)

    def _observer_state(self, availability: ObserverAvailability) -> CodePolicyObserverState:
        descriptor: CodePolicyObserverDescriptor | None = None
        with self._kernel_lock:
            generation = self._kernel_generation
            manager = self._kernel_manager
            client = self._kernel_client
            if availability in {"ready", "replaced"}:
                if manager is None or client is None or not manager.is_alive():
                    availability = "unavailable"
                else:
                    connection = manager.get_connection_info()
                    key = connection["key"]
                    if isinstance(key, str):
                        key = key.encode()
                    descriptor = CodePolicyObserverDescriptor(
                        transport=connection["transport"],
                        ip=connection["ip"],
                        iopub_port=connection["iopub_port"],
                        signature_scheme=connection["signature_scheme"],
                        key_base64=base64.b64encode(key).decode("ascii"),
                        code_policy_session_id=self._session_id,
                        jupyter_client_session_id=client.session.session,
                        kernel_generation=generation,
                    )
        return CodePolicyObserverState(
            availability=availability,
            code_policy_session_id=self._session_id,
            kernel_generation=generation,
            descriptor=descriptor,
        )


def _new_session_id() -> str:
    return f"code_policy_session_{uuid4().hex}"


def _utc_now() -> datetime:
    return datetime.now(UTC)


def _format_reply(reply: dict[str, Any], output: str, duration_s: float) -> str:
    content = reply.get("content", {})
    status = content.get("status", "unknown")
    execution_count = content.get("execution_count", "?")
    state = "completed" if status == "ok" else "failed"
    body = output.rstrip()
    if not body and status != "ok":
        body = f"{content.get('ename', 'Error')}: {content.get('evalue', '')}".rstrip()
    if not body:
        body = "(completed)"
    return f"In [{execution_count}] {state} in {duration_s:.2f}s\n\n{body}"


code_policy_module = CodePolicyModule.blueprint
