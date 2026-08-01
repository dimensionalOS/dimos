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

"""Process binding for the interactive one-tool Pi SDK host."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import queue
import subprocess
import threading
import time
from typing import IO, Any
from uuid import uuid4

from dimos.benchmark.agent_eval.config import RuntimeCredential
from dimos.benchmark.agent_eval.models import ArtifactReference
from dimos.benchmark.agent_eval.pi_adapter import (
    CodePolicyCallLog,
    McpBinding,
    PythonExecBroker,
)
from dimos.benchmark.agent_eval.runner import PiTurn

_PROTOCOL_VERSION = 1
_MAX_FRAME_BYTES = 64 * 1024
_MAX_STDERR_BYTES = 64 * 1024


class NodePiSessionFactory:
    def __init__(
        self,
        *,
        command: tuple[str, ...],
        credential: RuntimeCredential,
        model: str,
        thinking_level: str,
        startup_timeout_s: float,
    ) -> None:
        if not command or startup_timeout_s <= 0:
            raise ValueError("Pi adapter command and startup timeout are required")
        if model != "gpt-5.6-luna" or thinking_level != "medium":
            raise ValueError("the pinned Pi adapter supports only gpt-5.6-luna/medium")
        self.command = command
        self.credential = credential
        self.startup_timeout_s = startup_timeout_s

    def create(
        self,
        *,
        attempt_path: Path,
        public_prompt: str,
        code_policy_session_id: str,
        call_log: CodePolicyCallLog,
        mcp: McpBinding,
    ) -> NodePiSession:
        session_id = f"pi_session_{uuid4().hex}"
        broker = PythonExecBroker(
            attempt_id=attempt_path.name,
            pi_session_id=session_id,
            code_policy_session_id=code_policy_session_id,
            mcp=mcp,
            call_log=call_log,
        )
        return NodePiSession(
            command=self.command,
            credential=self.credential,
            attempt_path=attempt_path,
            session_id=session_id,
            initial_prompt=public_prompt,
            broker=broker,
            startup_timeout_s=self.startup_timeout_s,
        )


class NodePiSession:
    def __init__(
        self,
        *,
        command: tuple[str, ...],
        credential: RuntimeCredential,
        attempt_path: Path,
        session_id: str,
        initial_prompt: str,
        broker: PythonExecBroker,
        startup_timeout_s: float,
    ) -> None:
        self.session_id = session_id
        self.policy_call_count = 0
        self._attempt_path = attempt_path
        self._broker = broker
        self._frames: queue.Queue[dict[str, Any] | BaseException] = queue.Queue()
        self._write_lock = threading.Lock()
        self._closed_evidence: dict[str, Any] | None = None
        self._disposed = False
        self._stderr_path = attempt_path / "pi-adapter.stderr.log"
        self._stderr = bytearray()
        self._process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            cwd=attempt_path,
            env=_adapter_environment(credential, attempt_path),
        )
        assert (
            self._process.stdin is not None
            and self._process.stdout is not None
            and self._process.stderr is not None
        )
        self._stdin: IO[str] = self._process.stdin
        self._reader = threading.Thread(
            target=self._read_frames,
            args=(self._process.stdout,),
            name=f"pi-adapter-reader-{session_id}",
            daemon=True,
        )
        self._stderr_reader = threading.Thread(
            target=self._read_stderr,
            args=(self._process.stderr,),
            name=f"pi-adapter-stderr-{session_id}",
            daemon=True,
        )
        self._reader.start()
        self._stderr_reader.start()
        self._send(
            {
                "version": _PROTOCOL_VERSION,
                "type": "session_start",
                "id": session_id,
                "initial_prompt": initial_prompt,
                "thinking_level": "medium",
            }
        )
        try:
            started = self._await("session_started", session_id, startup_timeout_s)
        except BaseException:
            self._terminate_process()
            raise
        if started.get("tools") != ["python_exec"]:
            self.dispose()
            raise RuntimeError("Pi adapter activated an unexpected tool inventory")

    def prompt(self, prompt: str, timeout_s: float) -> PiTurn:
        if self._disposed:
            raise RuntimeError("Pi session is disposed")
        turn_id = f"turn_{uuid4().hex}"
        self._send(
            {
                "version": _PROTOCOL_VERSION,
                "type": "prompt",
                "id": turn_id,
                "text": prompt,
            }
        )
        frame = self._await("turn_complete", turn_id, timeout_s)
        count = frame.get("policy_call_count")
        if not isinstance(count, int) or count < self.policy_call_count:
            raise RuntimeError("Pi adapter returned an invalid policy-call count")
        self.policy_call_count = count
        final_text = frame.get("final_text")
        return PiTurn(
            final_text=final_text if isinstance(final_text, str) else "",
            policy_call_count=count,
        )

    def abort(self, timeout_s: float) -> None:
        del timeout_s
        if not self._disposed and self._process.poll() is None:
            self._send({"version": _PROTOCOL_VERSION, "type": "abort"})

    def dispose(self) -> None:
        if self._disposed:
            return
        self._disposed = True
        try:
            if self._process.poll() is None:
                self._send({"version": _PROTOCOL_VERSION, "type": "dispose"})
                try:
                    frame = self._await("session_closed", self.session_id, 5.0)
                    evidence = frame.get("evidence")
                    if isinstance(evidence, dict):
                        self._closed_evidence = evidence
                except (RuntimeError, TimeoutError):
                    self._process.terminate()
            self._process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self._terminate_process()
        finally:
            self._reader.join(timeout=2.0)
            self._stderr_reader.join(timeout=2.0)
            self._stderr_path.write_bytes(bytes(self._stderr))

    def _terminate_process(self) -> None:
        self._disposed = True
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=2.0)
        self._reader.join(timeout=2.0)
        self._stderr_reader.join(timeout=2.0)
        self._stderr_path.write_bytes(bytes(self._stderr))

    def artifact_references(self) -> tuple[ArtifactReference, ...]:
        if not self._disposed:
            raise RuntimeError("Pi session evidence is available only after disposal")
        relative_paths = ["pi-adapter.stderr.log"]
        evidence = self._closed_evidence or {}
        session_path = evidence.get("relative_path")
        if evidence.get("persisted") is True and isinstance(session_path, str):
            relative_paths.append(session_path)
        for key in ("system_prompt", "initial_prompt"):
            prompt = evidence.get(key)
            if isinstance(prompt, dict) and isinstance(prompt.get("relative_path"), str):
                relative_paths.append(prompt["relative_path"])
        return tuple(_artifact(self._attempt_path, path) for path in relative_paths)

    def _read_frames(self, output: IO[str]) -> None:
        try:
            for line in output:
                if len(line.encode()) > _MAX_FRAME_BYTES:
                    raise RuntimeError("Pi adapter frame exceeds limit")
                frame = json.loads(line)
                if not isinstance(frame, dict) or frame.get("version") != _PROTOCOL_VERSION:
                    raise RuntimeError("invalid Pi adapter frame")
                if frame.get("type") == "tool_call":
                    self._handle_tool_call(frame)
                elif frame.get("type") == "transcript":
                    continue
                else:
                    self._frames.put(frame)
        except BaseException as exc:
            self._frames.put(exc)

    def _handle_tool_call(self, frame: dict[str, Any]) -> None:
        call_id = frame.get("id")
        tool = frame.get("tool")
        params = frame.get("params")
        if (
            not isinstance(call_id, str)
            or not isinstance(tool, str)
            or not isinstance(params, dict)
        ):
            raise RuntimeError("malformed Pi tool call")
        try:
            result = self._broker.request(tool, params)
            text = _mcp_text(result)
            reply = {
                "version": _PROTOCOL_VERSION,
                "type": "tool_reply",
                "id": call_id,
                "ok": True,
                "result": text,
            }
        except Exception as exc:
            reply = {
                "version": _PROTOCOL_VERSION,
                "type": "tool_reply",
                "id": call_id,
                "ok": False,
                "error": f"{type(exc).__name__}: {exc}"[:1024],
            }
        self._send(reply)

    def _read_stderr(self, stderr: IO[str]) -> None:
        for chunk in iter(lambda: stderr.read(4096), ""):
            remaining = _MAX_STDERR_BYTES - len(self._stderr)
            if remaining > 0:
                self._stderr.extend(chunk.encode()[:remaining])

    def _send(self, frame: dict[str, Any]) -> None:
        encoded = json.dumps(frame, allow_nan=False, separators=(",", ":"))
        if len(encoded.encode()) > _MAX_FRAME_BYTES:
            raise ValueError("outbound Pi adapter frame exceeds limit")
        with self._write_lock:
            self._stdin.write(encoded + "\n")
            self._stdin.flush()

    def _await(self, frame_type: str, frame_id: str, timeout_s: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        deferred: list[dict[str, Any]] = []
        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"Pi adapter timed out waiting for {frame_type}")
                try:
                    item = self._frames.get(timeout=remaining)
                except queue.Empty as exc:
                    raise TimeoutError(f"Pi adapter timed out waiting for {frame_type}") from exc
                if isinstance(item, BaseException):
                    raise RuntimeError(f"Pi adapter reader failed: {item}") from item
                if item.get("type") == "protocol_error":
                    raise RuntimeError(str(item.get("error", "Pi adapter protocol error")))
                if item.get("type") == frame_type and item.get("id") == frame_id:
                    return item
                deferred.append(item)
        finally:
            for item in deferred:
                self._frames.put(item)


def _adapter_environment(
    credential: RuntimeCredential,
    attempt_path: Path,
) -> dict[str, str]:
    env = {
        "PATH": os.environ.get("PATH", ""),
        "PI_SPATIAL_AGENT_CWD": str(attempt_path),
        "PI_SPATIAL_SESSION_DIR": "pi-session",
    }
    if credential.auth_mode == "subscription":
        env["PI_SPATIAL_AUTH_MODE"] = "codex-oauth"
        env["PI_SPATIAL_AUTH_PATH"] = credential.binding_name
    elif credential.auth_mode == "environment" and credential.value:
        env["PI_SPATIAL_AUTH_MODE"] = "openai-api-key"
        env["OPENAI_API_KEY"] = credential.value
    else:
        raise ValueError("unsupported or incomplete Pi credential binding")
    return env


def _mcp_text(result: dict[str, Any]) -> str:
    content = result.get("content")
    if not isinstance(content, list) or not content:
        return ""
    first = content[0]
    if isinstance(first, dict):
        text = first.get("text")
        if isinstance(text, str):
            return text
    return json.dumps(first, allow_nan=False, separators=(",", ":"))[:32_000]


def _artifact(root: Path, relative_path: str) -> ArtifactReference:
    if not relative_path or relative_path.startswith("/") or ".." in relative_path.split("/"):
        raise ValueError("Pi evidence path is not attempt-relative")
    data = (root / relative_path).read_bytes()
    return ArtifactReference(
        path=relative_path,
        sha256=hashlib.sha256(data).hexdigest(),
        size_bytes=len(data),
    )
