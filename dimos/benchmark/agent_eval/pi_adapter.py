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

"""One-tool Pi facade over an attached DimOS MCP server.

The attached server is intentionally allowed to expose the normal robot skill
inventory.  This module records that inventory but admits only the exact
``python_exec`` schema into the Pi model-facing session.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from datetime import UTC, datetime
import hashlib
import json
import os
from pathlib import Path
import threading
import time
from typing import Any, Protocol
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field, JsonValue

from dimos.agents.code_policy import MAX_EXECUTION_TIMEOUT_S
from dimos.benchmark.agent_eval.models import (
    AttemptId,
    CodePolicySessionId,
    NonEmpty,
)
from dimos.benchmark.spatial.utilities import canonical_json

PYTHON_EXEC_TOOL_NAME = "python_exec"
PI_TOOL_NAMES = (PYTHON_EXEC_TOOL_NAME,)
_EXPECTED_DESCRIPTION_PREFIX = (
    "Execute one synchronous Python program in the persistent policy session."
)


class McpBinding(Protocol):
    """Minimum MCP behavior used by the one-tool facade."""

    def wait_for_ready(self, timeout: float) -> bool: ...

    def list_tools(self) -> list[dict[str, Any]]: ...

    def call_tool(self, name: str, arguments: dict[str, Any] | None = None) -> dict[str, Any]: ...


class PiAdapterModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)
    schema_version: str = "1.0"


class McpInventoryReceipt(PiAdapterModel):
    record_type: str = "mcp-inventory-receipt"
    endpoint: NonEmpty
    observed_tools: tuple[dict[str, JsonValue], ...]
    python_exec_schema_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")


class CodePolicyCallRecord(PiAdapterModel):
    record_type: str = "code-policy-call"
    call_id: NonEmpty
    attempt_id: AttemptId
    pi_session_id: NonEmpty
    code_policy_session_id: CodePolicySessionId
    tool_name: str
    arguments: dict[str, JsonValue]
    requested_at: datetime
    completed_at: datetime
    monotonic_duration_s: float = Field(ge=0)
    ok: bool
    result: dict[str, JsonValue] | None = None
    error: NonEmpty | None = None


class ToolInventoryError(ValueError):
    """The attached MCP inventory cannot safely back the Pi facade."""


def inspect_python_exec_inventory(
    endpoint: str,
    tools: Sequence[Mapping[str, Any]],
) -> McpInventoryReceipt:
    """Validate one exact code-policy tool while retaining the full inventory."""
    observed = tuple(_json_tool(tool) for tool in tools)
    matches = [tool for tool in observed if tool.get("name") == PYTHON_EXEC_TOOL_NAME]
    if len(matches) != 1:
        raise ToolInventoryError("MCP inventory must contain exactly one python_exec tool")
    schema = matches[0].get("inputSchema")
    description = matches[0].get("description")
    if not isinstance(schema, dict) or not _is_python_exec_schema(schema):
        raise ToolInventoryError("python_exec input schema is incompatible")
    if (
        not isinstance(description, str)
        or not description.startswith(_EXPECTED_DESCRIPTION_PREFIX)
        or "trusted, unsandboxed" not in description
    ):
        raise ToolInventoryError("python_exec description is incompatible")
    return McpInventoryReceipt(
        endpoint=endpoint,
        observed_tools=observed,
        python_exec_schema_sha256=hashlib.sha256(canonical_json(schema)).hexdigest(),
    )


def wait_for_python_exec(
    endpoint: str,
    mcp: McpBinding,
    timeout_s: float,
) -> McpInventoryReceipt:
    if timeout_s <= 0:
        raise ValueError("MCP readiness timeout must be positive")
    if not mcp.wait_for_ready(timeout_s):
        raise TimeoutError(f"MCP server did not become ready at {endpoint}")
    return inspect_python_exec_inventory(endpoint, mcp.list_tools())


class CodePolicyCallLog:
    """Append-only durable evidence for calls forwarded on Pi's behalf."""

    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._descriptor = os.open(
            path,
            os.O_WRONLY | os.O_APPEND | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC,
            0o600,
        )
        self._lock = threading.Lock()
        self._closed = False

    def append(self, record: CodePolicyCallRecord) -> None:
        encoded = canonical_json(record.model_dump(mode="json")) + b"\n"
        with self._lock:
            if self._closed:
                raise RuntimeError("code-policy call log is closed")
            view = memoryview(encoded)
            while view:
                view = view[os.write(self._descriptor, view) :]
            os.fsync(self._descriptor)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            os.close(self._descriptor)

    def __enter__(self) -> CodePolicyCallLog:
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()


class PythonExecBroker:
    """Forward the sole Pi tool and bind evidence to both session identities."""

    def __init__(
        self,
        *,
        attempt_id: str,
        pi_session_id: str,
        code_policy_session_id: str,
        mcp: McpBinding,
        call_log: CodePolicyCallLog,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.attempt_id = attempt_id
        self.pi_session_id = pi_session_id
        self.code_policy_session_id = code_policy_session_id
        self.mcp = mcp
        self.call_log = call_log
        self.clock = clock
        self.call_count = 0

    def request(self, tool_name: str, arguments: Mapping[str, Any]) -> dict[str, Any]:
        if tool_name != PYTHON_EXEC_TOOL_NAME:
            raise PermissionError(f"Pi tool {tool_name!r} is not permitted")
        safe_arguments = _validate_arguments(arguments)
        requested_at = datetime.now(UTC)
        started = self.clock()
        self.call_count += 1
        result: dict[str, Any] | None = None
        error: str | None = None
        try:
            result = self.mcp.call_tool(PYTHON_EXEC_TOOL_NAME, safe_arguments)
            if not isinstance(result, dict):
                raise TypeError("MCP tool result must be an object")
            return result
        except Exception as exc:
            error = _bounded_diagnostic(exc)
            raise
        finally:
            safe_result = _json_object(result) if result is not None else None
            self.call_log.append(
                CodePolicyCallRecord(
                    call_id=f"pi_tool_call_{uuid4().hex}",
                    attempt_id=self.attempt_id,
                    pi_session_id=self.pi_session_id,
                    code_policy_session_id=self.code_policy_session_id,
                    tool_name=PYTHON_EXEC_TOOL_NAME,
                    arguments=safe_arguments,
                    requested_at=requested_at,
                    completed_at=datetime.now(UTC),
                    monotonic_duration_s=max(0.0, self.clock() - started),
                    ok=error is None,
                    result=safe_result,
                    error=error,
                )
            )


def credential_binding_sha256(
    auth_mode: str,
    binding_name: str,
    credential: str | bytes | None = None,
) -> str:
    """Return a domain-separated binding digest without retaining the secret."""
    if not auth_mode or not binding_name:
        raise ValueError("authentication mode and binding name are required")
    digest = hashlib.sha256()
    digest.update(b"dimos-agent-eval-credential-binding-v1\0")
    digest.update(auth_mode.encode())
    digest.update(b"\0")
    digest.update(binding_name.encode())
    if credential is not None:
        digest.update(b"\0")
        digest.update(
            hashlib.sha256(
                credential.encode() if isinstance(credential, str) else credential
            ).digest()
        )
    return digest.hexdigest()


def _is_python_exec_schema(schema: Mapping[str, Any]) -> bool:
    properties = schema.get("properties")
    if (
        schema.get("type") != "object"
        or schema.get("required") != ["code"]
        or not isinstance(properties, dict)
        or set(properties) != {"code", "timeout_s"}
    ):
        return False
    code = properties["code"]
    timeout = properties["timeout_s"]
    return (
        isinstance(code, dict)
        and code.get("type") == "string"
        and isinstance(timeout, dict)
        and timeout.get("type") == "number"
        and timeout.get("default") == MAX_EXECUTION_TIMEOUT_S
    )


def _validate_arguments(arguments: Mapping[str, Any]) -> dict[str, JsonValue]:
    if set(arguments) - {"code", "timeout_s"}:
        raise ValueError("python_exec arguments contain unknown fields")
    code = arguments.get("code")
    timeout = arguments.get("timeout_s", MAX_EXECUTION_TIMEOUT_S)
    if not isinstance(code, str) or not code:
        raise ValueError("python_exec code must be a non-empty string")
    if isinstance(timeout, bool) or not isinstance(timeout, (float, int)):
        raise ValueError("python_exec timeout_s must be numeric")
    if not 0 < float(timeout) <= MAX_EXECUTION_TIMEOUT_S:
        raise ValueError("python_exec timeout_s is outside the supported range")
    return {"code": code, "timeout_s": float(timeout)}


def _json_tool(tool: Mapping[str, Any]) -> dict[str, JsonValue]:
    return _json_object(dict(tool))


def _json_object(value: Mapping[str, Any]) -> dict[str, JsonValue]:
    try:
        encoded = json.dumps(value, allow_nan=False)
        decoded = json.loads(encoded)
    except (TypeError, ValueError) as exc:
        raise ValueError("value is not strict JSON") from exc
    if not isinstance(decoded, dict):
        raise ValueError("value must be a JSON object")
    return decoded


def _bounded_diagnostic(exc: Exception) -> str:
    message = f"{type(exc).__name__}: {exc}".replace("\r", " ").replace("\n", " ")
    return message[:1024] or type(exc).__name__
