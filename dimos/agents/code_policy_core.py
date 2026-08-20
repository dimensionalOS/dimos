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

import base64
from dataclasses import dataclass
import inspect
import os
from pathlib import Path
import queue
import re
import threading
import time
from typing import Any, Literal, get_type_hints

import cloudpickle  # type: ignore[import-untyped]
from pydantic import BaseModel, ConfigDict, Field
import requests

from dimos.benchmark.evaluation.protocol import (
    PolicyCandidate,
    PolicySnapshot,
    TrialEvidence,
    TrialOutcome,
    TrialRun,
)
from dimos.core.global_config import global_config
from dimos.porcelain.dimos import Dimos

# The MCP transport abandons a request after roughly 300 seconds. Interrupt the
# kernel with enough headroom for the timeout response to reach the agent; otherwise
# the abandoned execution keeps the serial session lock and every later call is busy.
MAX_EXECUTION_TIMEOUT_S = 240.0
DEFAULT_OUTPUT_LIMIT = 32_000
_SUBMISSION_URL_ENV = "DIMOS_CODE_POLICY_SUBMISSION_URL"
_FREEZE_URL_ENV = "DIMOS_CODE_POLICY_FREEZE_URL"
_SUBMISSION_TOKEN_ENV = "DIMOS_CODE_POLICY_SUBMISSION_TOKEN"
_RECORDING_PATH_ENV = "DIMOS_CODE_POLICY_RECORDING_PATH"
_ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
_TRUNCATION_MARKER = "\n... [output truncated]"
_CREDENTIAL_NAME_RE = re.compile(
    r"(?:API_?KEY|TOKEN|SECRET|PASSWORD|CREDENTIAL|AUTH|OPENAI|ANTHROPIC|AWS_|AZURE_)",
    re.IGNORECASE,
)


class SubmissionEnvironment(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    kind: Literal["submission"] = "submission"
    submission_url: str = Field(min_length=1)
    freeze_url: str = Field(min_length=1)
    submission_token: str = Field(min_length=1)


class LiveDimosEnvironment(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    kind: Literal["live_dimos"] = "live_dimos"
    recording_path: str = Field(min_length=1)


class CodePolicySessionConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    environment: SubmissionEnvironment | LiveDimosEnvironment
    output_limit: int = Field(default=DEFAULT_OUTPUT_LIMIT, ge=0)
    startup_timeout_s: float = Field(default=10.0, gt=0)
    interrupt_grace_s: float = Field(default=2.0, gt=0)


@dataclass(frozen=True)
class CodePolicyImage:
    data: str
    mime_type: Literal["image/png", "image/jpeg"]


@dataclass(frozen=True)
class CodePolicyOutput:
    text: str
    images: tuple[CodePolicyImage, ...] = ()


class _BoundedOutput:
    def __init__(self, limit: int) -> None:
        self.limit = limit
        self.parts: list[str] = []
        self.length = 0
        self.truncated = False
        self.images: list[CodePolicyImage] = []

    def __call__(self, message: dict[str, Any]) -> None:
        message_type = message.get("header", {}).get("msg_type")
        content = message.get("content", {})
        value = ""
        if message_type == "stream":
            value = str(content.get("text", ""))
        elif message_type in {"execute_result", "display_data"}:
            data = content.get("data", {})
            value = str(data.get("text/plain", ""))
            for mime_type in ("image/png", "image/jpeg"):
                encoded = data.get(mime_type)
                if isinstance(encoded, str):
                    self.images.append(CodePolicyImage(data=encoded, mime_type=mime_type))
                    break
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


def _bootstrap_source(environment: SubmissionEnvironment | LiveDimosEnvironment) -> str:
    if isinstance(environment, SubmissionEnvironment):
        return """
from dimos.agents.code_policy_core import freeze_policy, submit_policy
from dimos.porcelain.dimos import Dimos
"""
    return f"""
from dimos.memory2.store.sqlite import SqliteStore
from dimos.porcelain.dimos import Dimos

app = Dimos.connect(
    memory=SqliteStore(path={environment.recording_path!r}, must_exist=True, read_only=True)
)
app.memory.start()
"""


def _kernel_environment(config: CodePolicySessionConfig) -> dict[str, str]:
    """Build an exploration environment without forwarding host credentials."""
    result = {
        name: value for name, value in os.environ.items() if not _CREDENTIAL_NAME_RE.search(name)
    }
    result["DIMOS_TRANSPORT"] = global_config.transport
    if isinstance(config.environment, SubmissionEnvironment):
        result[_SUBMISSION_URL_ENV] = config.environment.submission_url
        result[_FREEZE_URL_ENV] = config.environment.freeze_url
        result[_SUBMISSION_TOKEN_ENV] = config.environment.submission_token
    else:
        result[_RECORDING_PATH_ENV] = config.environment.recording_path
    return result


def submit_policy(policy: Any) -> PolicyCandidate:
    """Submit a typed callable from the exploration kernel for one fresh trial."""
    validate_policy_callable(policy)
    try:
        source = inspect.getsource(policy)
    except (OSError, TypeError) as exc:
        raise TypeError("policy source is unavailable; define it in the exploration REPL") from exc
    try:
        serialized = cloudpickle.dumps(policy)
    except Exception as exc:
        raise TypeError(f"policy is not serializable: {type(exc).__name__}: {exc}") from exc

    response = requests.post(
        os.environ[_SUBMISSION_URL_ENV],
        headers={"Authorization": f"Bearer {os.environ[_SUBMISSION_TOKEN_ENV]}"},
        json={"source": source, "serialized": base64.b64encode(serialized).decode("ascii")},
        timeout=None,
    )
    if response.status_code != 200:
        detail = response.json().get("error", response.text)
        raise RuntimeError(f"policy submission failed: {detail}")
    payload = response.json()
    trial = TrialRun(
        run_id=payload["run_id"],
        outcome=TrialOutcome(**payload["outcome"]),
        artifacts=Path(payload["artifacts"]),
        log_path=Path(payload["log_path"]),
        memory_path=Path(payload["memory_path"]),
        policy_output=payload["policy_output"],
    )
    policy_payload = payload["policy"]
    candidate_id = payload["candidate_id"]
    return PolicyCandidate(
        id=candidate_id,
        policy=PolicySnapshot(
            source=policy_payload["source"],
            sha256=policy_payload["sha256"],
        ),
        evidence=TrialEvidence(
            candidate_id=candidate_id,
            trial=trial,
            remaining_submissions=payload["remaining_submissions"],
        ),
    )


def freeze_policy(candidate: PolicyCandidate) -> None:
    """Irreversibly select a candidate from the current exploration."""
    if not isinstance(candidate, PolicyCandidate):
        raise TypeError("freeze_policy requires a PolicyCandidate returned by submit_policy")
    import requests

    response = requests.post(
        os.environ[_FREEZE_URL_ENV],
        headers={"Authorization": f"Bearer {os.environ[_SUBMISSION_TOKEN_ENV]}"},
        json={"candidate_id": candidate.id},
        timeout=None,
    )
    if response.status_code != 200:
        detail = response.json().get("error", response.text)
        raise RuntimeError(f"policy freeze failed: {detail}")


def validate_policy_callable(policy: Any) -> None:
    """Enforce the one canonical callable contract before a trial is launched."""
    if not inspect.isfunction(policy) or inspect.iscoroutinefunction(policy):
        raise TypeError("policy must be a synchronous Python function")
    if policy.__name__ != "policy":
        raise TypeError("submitted function must be named 'policy'")
    signature = inspect.signature(policy)
    parameters = list(signature.parameters.values())
    if len(parameters) != 1 or parameters[0].kind not in {
        inspect.Parameter.POSITIONAL_ONLY,
        inspect.Parameter.POSITIONAL_OR_KEYWORD,
    }:
        raise TypeError("policy must accept exactly one positional app parameter")
    try:
        hints = get_type_hints(policy)
    except Exception as exc:
        raise TypeError(f"policy annotations could not be resolved: {exc}") from exc
    if hints.get(parameters[0].name) is not Dimos or hints.get("return") not in {None, type(None)}:
        raise TypeError("policy signature must be policy(app: Dimos) -> None")


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

    def python_exec(
        self, code: str, timeout_s: float = MAX_EXECUTION_TIMEOUT_S
    ) -> CodePolicyOutput:
        if self._stopped:
            return CodePolicyOutput("CodePolicy session is stopped")
        if not code:
            return CodePolicyOutput("python_exec code must be non-empty")
        if not 0 < timeout_s <= MAX_EXECUTION_TIMEOUT_S:
            return CodePolicyOutput(f"timeout_s must be in (0, {MAX_EXECUTION_TIMEOUT_S:g}]")
        if not self._execution_lock.acquire(blocking=False):
            return CodePolicyOutput("CodePolicy session is busy")
        started = time.monotonic()
        self.execution_count += 1
        try:
            try:
                client = self._ensure_kernel()
            except Exception as exc:
                return CodePolicyOutput(
                    f"CodePolicy kernel failed to start: {type(exc).__name__}: {exc}"
                )
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
                    return CodePolicyOutput(
                        f"Execution timed out after {timeout_s:.1f}s and was interrupted"
                    )
                return CodePolicyOutput(
                    f"Execution timed out after {timeout_s:.1f}s; "
                    "the kernel was restarted and its namespace was reset"
                )
            except Exception as exc:
                self._shutdown_kernel()
                return CodePolicyOutput(f"CodePolicy execution failed: {type(exc).__name__}: {exc}")
            content = reply.get("content", {})
            body = output.text().rstrip()
            if not body and content.get("status") != "ok":
                body = f"{content.get('ename', 'Error')}: {content.get('evalue', '')}"
            if not body:
                body = "(completed)"
            state = "completed" if content.get("status") == "ok" else "failed"
            return CodePolicyOutput(
                text=f"In [{content.get('execution_count', '?')}] {state}\n\n{body}",
                images=tuple(output.images),
            )
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
                manager.start_kernel(env=_kernel_environment(self.config))
                client = manager.client()
                client.start_channels()
                client.wait_for_ready(timeout=self.config.startup_timeout_s)
                reply = client.execute_interactive(
                    _bootstrap_source(self.config.environment),
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
                    manager.shutdown_kernel(now=True)
                    manager.cleanup_resources()
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
                    _bootstrap_source(self.config.environment),
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
        if client is not None:
            client.stop_channels()
        if manager is not None:
            try:
                manager.shutdown_kernel(now=True)
                manager.cleanup_resources()
            except Exception:
                pass
