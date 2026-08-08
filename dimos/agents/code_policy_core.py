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

"""Persistent, module-independent Python session for trusted CodePolicy agents."""

from __future__ import annotations

import os
import queue
import re
import threading
import time
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field

MAX_EXECUTION_TIMEOUT_S = 110.0
DEFAULT_OUTPUT_LIMIT = 32_000
_RECORDING_PATH_ENV = "DIMOS_CODE_POLICY_RECORDING_PATH"
_DERIVED_RECORDING_PATH_ENV = "DIMOS_CODE_POLICY_DERIVED_RECORDING_PATH"
_MEMORY_CUTOFF_ENV = "DIMOS_CODE_POLICY_MEMORY_CUTOFF"
_CONNECT_APP_ENV = "DIMOS_CODE_POLICY_CONNECT_APP"
_ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
_TRUNCATION_MARKER = "\n... [output truncated]"
_CREDENTIAL_NAME_RE = re.compile(
    r"(?:API_?KEY|TOKEN|SECRET|PASSWORD|CREDENTIAL|AUTH|OPENAI|ANTHROPIC|AWS_|AZURE_)",
    re.IGNORECASE,
)


class FrozenMemoryEnvironment(BaseModel):
    """A source and derived Memory2 recording pinned at an inclusive cutoff."""

    model_config = ConfigDict(extra="forbid", frozen=True)
    kind: Literal["frozen_memory"] = "frozen_memory"
    recording_path: str = Field(min_length=1)
    derived_recording_path: str = Field(min_length=1)
    memory_cutoff_timestamp: float


class LiveDimosEnvironment(BaseModel):
    """A live DimOS environment with read-only memory and an attached app."""

    model_config = ConfigDict(extra="forbid", frozen=True)
    kind: Literal["live_dimos"] = "live_dimos"
    recording_path: str = Field(min_length=1)


CodePolicyEnvironment = FrozenMemoryEnvironment | LiveDimosEnvironment


class CodePolicySessionConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    environment: CodePolicyEnvironment
    output_limit: int = Field(default=DEFAULT_OUTPUT_LIMIT, ge=0)
    startup_timeout_s: float = Field(default=10.0, gt=0)
    interrupt_grace_s: float = Field(default=2.0, gt=0)


class _BoundedOutput:
    def __init__(self, limit: int) -> None:
        self.limit = limit
        self.parts: list[str] = []
        self.length = 0
        self.truncated = False

    def __call__(self, message: dict[str, Any]) -> None:
        message_type = message.get("header", {}).get("msg_type")
        content = message.get("content", {})
        value = ""
        if message_type == "stream":
            value = str(content.get("text", ""))
        elif message_type in {"execute_result", "display_data"}:
            value = str(content.get("data", {}).get("text/plain", ""))
        elif message_type == "error":
            traceback = content.get("traceback", [])
            value = "\n".join(str(line) for line in traceback)
        self._append(_ANSI_ESCAPE_RE.sub("", value))

    def _append(self, value: str) -> None:
        if not value or self.truncated:
            return
        remaining = self.limit - self.length
        if len(value) <= remaining:
            self.parts.append(value)
            self.length += len(value)
            return
        marker = _TRUNCATION_MARKER[:remaining]
        content_limit = max(0, remaining - len(marker))
        self.parts.append(value[:content_limit] + marker)
        self.length = self.limit
        self.truncated = True

    def text(self) -> str:
        return "".join(self.parts)


def _load_kernel_manager() -> type[Any]:
    try:
        from jupyter_client.manager import KernelManager
    except ImportError as exc:
        raise RuntimeError(
            "CodePolicy requires ipykernel and jupyter-client; install the agents extra"
        ) from exc
    return KernelManager


def _bootstrap_source() -> str:
    return f"""
import os as _os
from dimos.memory2.store.sqlite import SqliteStore as _SqliteStore

_cutoff = _os.environ.get({_MEMORY_CUTOFF_ENV!r})
if _cutoff is None:
    memory = _SqliteStore(
        path=_os.environ[{_RECORDING_PATH_ENV!r}], must_exist=True, read_only=True
    )
else:
    from dimos.memory2.store.frozen import FrozenMemoryStore as _FrozenMemoryStore
    _source = _SqliteStore(
        path=_os.environ[{_RECORDING_PATH_ENV!r}], must_exist=True, read_only=True
    )
    _derived = _SqliteStore(
        path=_os.environ[{_DERIVED_RECORDING_PATH_ENV!r}], must_exist=True, read_only=True
    )
    memory = _FrozenMemoryStore(
        source=_source, derived=_derived, through_timestamp=float(_cutoff)
    )
    del _FrozenMemoryStore, _source, _derived

memory.start()
if _os.environ.get({_CONNECT_APP_ENV!r}) == "1":
    from dimos.porcelain.dimos import Dimos as _Dimos
    app = _Dimos.connect()
    del _Dimos

del _os, _SqliteStore, _cutoff
"""


def _kernel_environment(environment: CodePolicyEnvironment) -> dict[str, str]:
    """Build a useful kernel environment without forwarding host credentials."""
    result = {
        name: value for name, value in os.environ.items() if not _CREDENTIAL_NAME_RE.search(name)
    }
    result[_RECORDING_PATH_ENV] = environment.recording_path
    if isinstance(environment, FrozenMemoryEnvironment):
        result[_CONNECT_APP_ENV] = "0"
        result[_DERIVED_RECORDING_PATH_ENV] = environment.derived_recording_path
        result[_MEMORY_CUTOFF_ENV] = str(environment.memory_cutoff_timestamp)
    else:
        result[_CONNECT_APP_ENV] = "1"
        result.pop(_DERIVED_RECORDING_PATH_ENV, None)
        result.pop(_MEMORY_CUTOFF_ENV, None)
    return result


class CodePolicySession:
    """Execute trusted Python serially in one persistent Jupyter kernel."""

    def __init__(self, config: CodePolicySessionConfig) -> None:
        self.config = config
        self.execution_count = 0
        self.execution_duration_s = 0.0
        self._execution_lock = threading.Lock()
        self._kernel_lock = threading.RLock()
        self._manager: Any = None
        self._client: Any = None
        self._stopped = True

    def start(self) -> None:
        self._stopped = False

    def python_exec(self, code: str, timeout_s: float = MAX_EXECUTION_TIMEOUT_S) -> str:
        if self._stopped:
            return "CodePolicy session is stopped"
        if not code:
            return "python_exec code must be non-empty"
        if not 0 < timeout_s <= MAX_EXECUTION_TIMEOUT_S:
            return f"timeout_s must be in (0, {MAX_EXECUTION_TIMEOUT_S:g}]"
        if not self._execution_lock.acquire(blocking=False):
            return "CodePolicy session is busy"
        started = time.monotonic()
        self.execution_count += 1
        try:
            try:
                client = self._ensure_kernel()
            except Exception as exc:
                return f"CodePolicy kernel failed to start: {type(exc).__name__}: {exc}"
            output = _BoundedOutput(self.config.output_limit)
            try:
                reply = client.execute_interactive(
                    code,
                    allow_stdin=False,
                    output_hook=output,
                    store_history=True,
                    timeout=timeout_s,
                )
            except (TimeoutError, queue.Empty):
                if self._interrupt_and_recover():
                    return f"Execution timed out after {timeout_s:.1f}s and was interrupted"
                return (
                    f"Execution timed out after {timeout_s:.1f}s; "
                    "the kernel was restarted and its namespace was reset"
                )
            except Exception as exc:
                self._shutdown_kernel()
                return f"CodePolicy execution failed: {type(exc).__name__}: {exc}"
            content = reply.get("content", {})
            body = output.text().rstrip()
            if not body and content.get("status") != "ok":
                body = f"{content.get('ename', 'Error')}: {content.get('evalue', '')}"
            if not body:
                body = "(completed)"
            state = "completed" if content.get("status") == "ok" else "failed"
            return f"In [{content.get('execution_count', '?')}] {state}\n\n{body}"
        finally:
            self.execution_duration_s += time.monotonic() - started
            self._execution_lock.release()

    def stop(self) -> None:
        self._stopped = True
        self._shutdown_kernel()

    def _ensure_kernel(self) -> Any:
        with self._kernel_lock:
            if self._manager is not None and self._client is not None and self._manager.is_alive():
                return self._client
            self._shutdown_kernel()
            manager = _load_kernel_manager()(kernel_name="python3")
            client = None
            try:
                manager.start_kernel(env=_kernel_environment(self.config.environment))
                client = manager.client()
                client.start_channels()
                client.wait_for_ready(timeout=self.config.startup_timeout_s)
                reply = client.execute_interactive(
                    _bootstrap_source(),
                    allow_stdin=False,
                    output_hook=lambda _message: None,
                    silent=True,
                    store_history=False,
                    timeout=self.config.startup_timeout_s,
                )
                if reply.get("content", {}).get("status") != "ok":
                    content = reply.get("content", {})
                    raise RuntimeError(
                        f"{content.get('ename', 'KernelBootstrapError')}: "
                        f"{content.get('evalue', 'bootstrap failed')}"
                    )
            except Exception:
                if client is not None:
                    client.stop_channels()
                try:
                    manager.shutdown_kernel(now=False)
                except Exception:
                    pass
                raise
            self._manager = manager
            self._client = client
            return client

    def _interrupt_and_recover(self) -> bool:
        manager, client = self._manager, self._client
        if manager is None or client is None:
            return False
        try:
            manager.interrupt_kernel()
            client.wait_for_ready(timeout=self.config.interrupt_grace_s)
            return True
        except Exception:
            try:
                manager.restart_kernel(now=True)
                client.wait_for_ready(timeout=self.config.startup_timeout_s)
                reply = client.execute_interactive(
                    _bootstrap_source(),
                    allow_stdin=False,
                    output_hook=lambda _message: None,
                    silent=True,
                    store_history=False,
                    timeout=self.config.startup_timeout_s,
                )
                if reply.get("content", {}).get("status") != "ok":
                    raise RuntimeError("bootstrap failed after kernel restart")
            except Exception:
                self._shutdown_kernel()
            return False

    def _shutdown_kernel(self) -> None:
        with self._kernel_lock:
            manager, client = self._manager, self._client
            self._manager = None
            self._client = None
        if manager is not None:
            try:
                manager.shutdown_kernel(now=False)
            except Exception:
                pass
        if client is not None:
            client.stop_channels()
