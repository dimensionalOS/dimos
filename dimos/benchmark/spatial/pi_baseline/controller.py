# Copyright 2026 Dimensional Inc.
"""Closed NDJSON control loop for the v2 Node adapter protocol."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import os
from pathlib import Path
import queue
import re
import stat
import subprocess
import sys
import threading
import time
from typing import TextIO, cast

from dimos.benchmark.spatial.pi_baseline.broker import (
    CaseBroker,
    ImageReadRetryError,
    PolicyViolationError,
    PostImagePolicyViolationError,
    PreImagePolicyViolationError,
)
from dimos.benchmark.spatial.pi_baseline.config import (
    AuthMode,
    load_openai_api_key,
    validate_node_adapter_command,
)
from dimos.benchmark.spatial.pi_baseline.lifecycle_audit import (
    AuditDirection,
    LifecycleAuditRecord,
    bounded_audit,
)
from dimos.benchmark.spatial.pi_baseline.scheduler_executor import ExecutionInterrupted
from dimos.benchmark.spatial.pi_baseline.scheduler_models import (
    PreImagePolicyTelemetry,
)
from dimos.benchmark.spatial.pi_baseline.topology import PinnedDirectory

PROTOCOL_VERSION = 2
TOOLS = ("sandbox_exec", "read_generated_image", "submit_answer")
TERMINAL_REASONS = (
    "submitted",
    "max_turns",
    "max_tool_calls",
    "timeout",
    "session_error",
    "protocol_error",
    "post_image_policy_violation",
    "pre_image_policy_violation",
)
FAILED_TERMINAL_REASONS = frozenset(TERMINAL_REASONS[1:])
SESSION_RELATIVE_PATH = re.compile(r"pi-session/[A-Za-z0-9][A-Za-z0-9._-]*\.jsonl\Z")


@dataclass(frozen=True)
class SessionPromptEvidence:
    relative_path: str
    byte_count: int
    sha256: str


@dataclass(frozen=True)
class SessionEvidence:
    state: str
    persisted: bool
    relative_path: str | None
    system_prompt: SessionPromptEvidence | None
    initial_prompt: SessionPromptEvidence | None


@dataclass(frozen=True)
class AdapterTerminalResult:
    run_id: str
    ok: bool
    tool_replies: tuple[dict[str, object], ...]
    stderr_log: Path
    session_evidence: SessionEvidence


class AdapterRunError(RuntimeError):
    """The adapter reached a terminal failure or could not be controlled."""

    def __init__(
        self,
        code: str,
        *,
        terminal_reason: str | None = None,
        policy_telemetry: PreImagePolicyTelemetry | None = None,
        session_evidence: SessionEvidence | None = None,
    ) -> None:
        super().__init__(code)
        self.terminal_reason = (
            terminal_reason if terminal_reason in FAILED_TERMINAL_REASONS else None
        )
        self.policy_telemetry = policy_telemetry
        self.session_evidence = session_evidence


class AdapterCleanupError(AdapterRunError):
    """A controller reader did not quiesce after process termination."""


class AdapterController:
    def __init__(
        self,
        command: tuple[str, ...],
        auth_path: Path,
        transcript: PinnedDirectory | Path,
        *,
        auth_mode: AuthMode = "codex-oauth",
        max_frame_bytes: int = 64 * 1024,
        max_stderr_bytes: int = 64 * 1024,
        terminate_grace_seconds: float = 1.0,
    ) -> None:
        if max_frame_bytes <= 0 or max_stderr_bytes <= 0 or terminate_grace_seconds <= 0:
            raise ValueError("adapter command and bounds are required")
        self.command = tuple(validate_node_adapter_command(command))
        self.auth_path = auth_path
        self.auth_mode = auth_mode
        if isinstance(transcript, PinnedDirectory):
            self.transcript_root = transcript
            self._owns_transcript_root = False
            self.audit_name = "adapter-lifecycle-audit.v1.jsonl"
            self.stderr_name = "adapter.transcript.stderr.log"
        else:
            self.transcript_root = PinnedDirectory.open(transcript.parent, create=False)
            self._owns_transcript_root = True
            self.audit_name = "adapter-lifecycle-audit.v1.jsonl"
            self.stderr_name = transcript.name + ".stderr.log"
        self.audit = self.transcript_root.path / self.audit_name
        self.stderr_log = self.transcript_root.path / self.stderr_name
        self.max_frame_bytes = max_frame_bytes
        self.max_stderr_bytes = max_stderr_bytes
        self.terminate_grace_seconds = terminate_grace_seconds
        self._audit_records: list[LifecycleAuditRecord] = []
        self._audit_sequence = 0
        self.max_audit_records = 2048
        self._stderr_bytes = b""
        self._ensure_private_root()

    def _ensure_private_root(self) -> None:
        self.transcript_root.verify()
        info = os.fstat(self.transcript_root.fd)
        if (
            info.st_uid != os.getuid()
            or not stat.S_ISDIR(info.st_mode)
            or stat.S_IMODE(info.st_mode) & 0o077
        ):
            raise ValueError("adapter transcript root is not private")

    def _adapter_environment(self) -> dict[str, str]:
        env = {
            "PATH": os.environ.get("PATH", ""),
            "PI_SPATIAL_AUTH_MODE": self.auth_mode,
            "PI_SPATIAL_SESSION_DIR": "pi-session",
        }
        if self.auth_mode == "codex-oauth":
            env["PI_SPATIAL_AUTH_PATH"] = str(self.auth_path)
        else:
            env["OPENAI_API_KEY"] = load_openai_api_key(self.auth_path)
        return env

    def run(
        self,
        run_id: str,
        broker: CaseBroker,
        run_start: dict[str, object],
        cancel_requested: threading.Event,
        publication_lock: threading.Lock,
    ) -> AdapterTerminalResult:
        del publication_lock
        process: subprocess.Popen[str] | None = None
        try:
            _check_cancel(cancel_requested)
            start = self._validate_run_start(run_id, run_start)
            expected_mode = str(start["config"]["promptMode"]).replace("_", "-")  # type: ignore[index]
            if broker.prompt_mode != expected_mode:
                raise ValueError("prompt mode does not match broker policy")
            deadline = time.monotonic() + start["budget"]["timeoutMs"] / 1000.0  # type: ignore[index]
            env = self._adapter_environment()
            self._ensure_private_root()
            process = subprocess.Popen(
                self.command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=env,
                # The fd is the trust boundary.  Passing it explicitly keeps
                # /proc/self/fd/N valid through the child launch even if the
                # descriptive pathname is replaced between checks.
                cwd=f"/proc/self/fd/{self.transcript_root.fd}",
                pass_fds=(self.transcript_root.fd,),
            )
            self._ensure_private_root()
            assert (
                process.stdin is not None
                and process.stdout is not None
                and process.stderr is not None
            )
        except BaseException as error:
            if process is not None:
                cleanup_error = self._cleanup_process(process, None, None)
                if cleanup_error is not None:
                    error.add_note(f"adapter cleanup supplemental failure: {cleanup_error}")
            else:
                self.close()
            raise
        assert process is not None
        stdin = cast("TextIO", process.stdin)
        frames: queue.Queue[str | None] = queue.Queue()
        stdout_thread: threading.Thread | None = None
        stderr_thread: threading.Thread | None = None
        try:
            stdout_thread = threading.Thread(
                target=_queue_lines, args=(process.stdout, frames), daemon=True
            )
            stderr_thread = threading.Thread(
                target=self._drain_stderr, args=(process.stderr,), daemon=True
            )
            stdout_thread.start()
            stderr_thread.start()
        except BaseException as error:
            cleanup_error = self._cleanup_process(process, stdout_thread, stderr_thread)
            if cleanup_error is not None:
                error.add_note(f"adapter cleanup supplemental failure: {cleanup_error}")
            raise
        responses: list[dict[str, object]] = []
        started = False
        completed = False
        session_evidence: SessionEvidence | None = None
        try:
            self._send(stdin, start)
            while True:
                _check_cancel(cancel_requested)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise AdapterRunError(
                        "adapter_host_deadline",
                        terminal_reason="timeout",
                        session_evidence=session_evidence,
                    )
                try:
                    raw = frames.get(timeout=min(remaining, 0.1))
                except queue.Empty:
                    continue
                if raw is None:
                    break
                if len(raw.encode("utf-8")) > self.max_frame_bytes:
                    raise ValueError("adapter frame exceeds configured bound")
                frame = json.loads(raw)
                self._record_inbound(frame)
                if not isinstance(frame, dict):
                    raise ValueError("adapter frame must be an object")
                frame_type = frame.get("type")
                if frame_type == "run_started":
                    self._validate_run_started(frame, run_id)
                    started = True
                elif frame_type == "transcript":
                    self._validate_transcript(frame)
                elif frame_type == "tool_call":
                    if not started or completed:
                        raise ValueError("tool_call outside active run")
                    response = self._tool_reply(frame, broker, cancel_requested, deadline)
                    _check_cancel(cancel_requested)
                    self._send(stdin, response)
                    if (
                        frame["tool"] == "read_generated_image"
                        and not response["ok"]
                        and broker.prompt_mode == "visualization-encouraged"
                    ):
                        if response.get("error") == "image_read_failed_retry_once":
                            responses.append(response)
                            continue
                        broker.commit_image_read(delivered=False)
                        raise AdapterRunError(
                            "pre_image_policy_violation",
                            terminal_reason="pre_image_policy_violation",
                            policy_telemetry=broker.policy_telemetry("image_attempt_failed"),
                        )
                    if frame["tool"] == "read_generated_image" and response["ok"]:
                        broker.commit_image_read(delivered=True)
                    responses.append(response)
                elif frame_type == "run_complete":
                    session_evidence = self._validate_run_complete(frame, run_id)
                    completed = True
                    if frame["reason"] == "submitted" and broker.transaction.prediction is None:
                        raise AdapterRunError(
                            "adapter_submitted_without_answer", session_evidence=session_evidence
                        )
                    if not frame["ok"]:
                        if (
                            broker.prompt_mode == "visualization-encouraged"
                            and frame["reason"] == "pre_image_policy_violation"
                        ):
                            raise AdapterRunError(
                                "pre_image_policy_violation",
                                terminal_reason="pre_image_policy_violation",
                                policy_telemetry=broker.policy_telemetry(
                                    "prompt_returned_without_image"
                                ),
                                session_evidence=session_evidence,
                            )
                        if (
                            broker.prompt_mode == "visualization-encouraged"
                            and not broker.successful_image_read()
                            and frame["reason"] in {"max_turns", "max_tool_calls"}
                        ):
                            raise AdapterRunError(
                                "pre_image_policy_violation",
                                terminal_reason="pre_image_policy_violation",
                                policy_telemetry=broker.policy_telemetry(
                                    "budget_exhausted_before_image"
                                ),
                                session_evidence=session_evidence,
                            )
                        if (
                            broker.prompt_mode == "visualization-encouraged"
                            and broker.successful_image_read()
                            and frame["reason"] in {"max_turns", "max_tool_calls"}
                        ):
                            raise AdapterRunError(
                                "post_image_policy_violation",
                                terminal_reason="post_image_policy_violation",
                                session_evidence=session_evidence,
                            )
                        raise AdapterRunError(
                            "adapter_run_failed",
                            terminal_reason=cast("str", frame["reason"]),
                            session_evidence=session_evidence,
                        )
                    break
                elif frame_type == "protocol_error":
                    raise AdapterRunError(
                        "adapter_protocol_error",
                        terminal_reason="protocol_error",
                        session_evidence=session_evidence,
                    )
                else:
                    raise ValueError("unknown adapter frame type")
            if not started:
                raise ValueError("missing run_started frame")
            if not completed:
                raise AdapterRunError(
                    "adapter_eof_before_complete", session_evidence=session_evidence
                )
            assert session_evidence is not None
            return AdapterTerminalResult(
                run_id, True, tuple(responses), self.stderr_log, session_evidence
            )
        except Exception as error:
            if (
                broker.prompt_mode == "visualization-encouraged"
                and not broker.successful_image_read()
                and isinstance(error, AdapterRunError)
                and error.terminal_reason in {"max_turns", "max_tool_calls"}
            ):
                raise AdapterRunError(
                    "pre_image_policy_violation",
                    terminal_reason="pre_image_policy_violation",
                    policy_telemetry=broker.policy_telemetry("budget_exhausted_before_image"),
                    session_evidence=session_evidence,
                ) from None
            if (
                broker.prompt_mode == "visualization-encouraged"
                and broker.successful_image_read()
                and isinstance(error, AdapterRunError)
                and error.terminal_reason in {"max_turns", "max_tool_calls"}
            ):
                raise AdapterRunError(
                    "post_image_policy_violation",
                    terminal_reason="post_image_policy_violation",
                    session_evidence=session_evidence,
                ) from None
            raise
        finally:
            primary_error = sys.exc_info()[1]
            close_error: BaseException | None = None
            try:
                stdin.close()
            except BaseException as error:
                close_error = error
            cleanup_error = self._cleanup_process(
                process, stdout_thread, stderr_thread, close_transcript=False
            )
            if close_error is not None:
                if cleanup_error is None:
                    cleanup_error = AdapterCleanupError("adapter stdin cleanup failed")
                    cleanup_error.__cause__ = close_error
            persistence_error: BaseException | None = None
            try:
                if self._audit_records:
                    self.transcript_root.write_bytes(
                        self.audit_name, bounded_audit(self._audit_records, self.max_audit_records)
                    )
                self.transcript_root.write_bytes(self.stderr_name, self._stderr_bytes)
            except BaseException as error:
                persistence_error = error
            finally:
                try:
                    self.close()
                except BaseException as error:
                    if persistence_error is None:
                        persistence_error = error
            supplemental = cleanup_error or persistence_error
            if supplemental is not None and primary_error is not None:
                primary_error.add_note(f"adapter cleanup supplemental failure: {supplemental}")
            elif supplemental is not None:
                raise supplemental

    def _cleanup_process(
        self,
        process: subprocess.Popen[str],
        stdout_thread: threading.Thread | None,
        stderr_thread: threading.Thread | None,
        *,
        close_transcript: bool = True,
    ) -> AdapterCleanupError | None:
        failures: list[BaseException] = []
        try:
            _terminate_then_kill(process, self.terminate_grace_seconds)
        except BaseException as error:
            failures.append(error)
        try:
            if not _quiesce_readers(
                process, stdout_thread, stderr_thread, self.terminate_grace_seconds
            ):
                failures.append(AdapterCleanupError("adapter reader did not quiesce"))
        except BaseException as error:
            failures.append(error)
        try:
            process.wait(timeout=self.terminate_grace_seconds)
        except BaseException as error:
            failures.append(error)
        if close_transcript:
            try:
                self.close()
            except BaseException as error:
                failures.append(error)
        if not failures:
            return None
        cleanup_error = AdapterCleanupError("adapter cleanup failed")
        cleanup_error.__cause__ = failures[0]
        return cleanup_error

    def close(self) -> None:
        if self._owns_transcript_root:
            self.transcript_root.close()
            self._owns_transcript_root = False

    def __enter__(self) -> AdapterController:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def __del__(self) -> None:
        if getattr(self, "_owns_transcript_root", False):
            self.close()

    def _drain_stderr(self, stream: TextIO) -> None:
        written = 0
        chunks: list[bytes] = []
        while True:
            chunk = stream.read(4096)
            if not chunk:
                break
            if written < self.max_stderr_bytes:
                text = _truncate_utf8(chunk, self.max_stderr_bytes - written)
                chunks.append(text.encode())
                written += len(chunks[-1])
        self._stderr_bytes = b"".join(chunks)

    def _tool_reply(
        self,
        frame: dict[str, object],
        broker: CaseBroker,
        cancel_requested: threading.Event,
        controller_deadline: float | None = None,
    ) -> dict[str, object]:
        if cancel_requested is not None:
            _check_cancel(cancel_requested)
        if (
            set(frame) != {"version", "type", "id", "tool", "params"}
            or frame["version"] != PROTOCOL_VERSION
        ):
            raise ValueError("invalid tool_call frame")
        call_id, tool, params = frame["id"], frame["tool"], frame["params"]
        if (
            not isinstance(call_id, str)
            or not 0 < len(call_id) <= 128
            or not isinstance(tool, str)
            or tool not in TOOLS
            or not isinstance(params, dict)
        ):
            raise ValueError("invalid tool_call frame")
        try:
            if controller_deadline is not None and time.monotonic() >= controller_deadline:
                raise AdapterRunError("adapter_host_deadline", terminal_reason="timeout")
            result = broker.dispatch(
                tool,
                params,
                **(
                    {"controller_deadline": controller_deadline}
                    if tool == "sandbox_exec" and controller_deadline is not None
                    else {}
                ),
            )
            if controller_deadline is not None and time.monotonic() >= controller_deadline:
                raise AdapterRunError("adapter_host_deadline", terminal_reason="timeout")
            if cancel_requested is not None:
                _check_cancel(cancel_requested)
            reply: dict[str, object] = {
                "version": PROTOCOL_VERSION,
                "type": "tool_reply",
                "id": call_id,
                "ok": True,
                "result": _tool_result(tool, result),
            }
        except ExecutionInterrupted:
            raise
        except PostImagePolicyViolationError:
            raise AdapterRunError(
                "post_image_policy_violation", terminal_reason="post_image_policy_violation"
            ) from None
        except ImageReadRetryError:
            reply = {
                "version": PROTOCOL_VERSION,
                "type": "tool_reply",
                "id": call_id,
                "ok": False,
                "error": "image_read_failed_retry_once",
                "result": broker.image_recovery_contract(),
            }
        except PreImagePolicyViolationError as error:
            raise AdapterRunError(
                "pre_image_policy_violation",
                terminal_reason="pre_image_policy_violation",
                policy_telemetry=broker.policy_telemetry(error.failure_kind, error.category),
            ) from None
        except PolicyViolationError as error:
            reply = {
                "version": PROTOCOL_VERSION,
                "type": "tool_reply",
                "id": call_id,
                "ok": False,
                "error": _stable_error_code(error),
            }
        except Exception as error:
            if tool == "read_generated_image" and isinstance(error, ValueError):
                try:
                    broker.note_image_failure("adapter_result_rejected")
                except PreImagePolicyViolationError as policy_error:
                    raise _pre_image_policy_error(broker, policy_error) from None
                reply = {
                    "version": PROTOCOL_VERSION,
                    "type": "tool_reply",
                    "id": call_id,
                    "ok": False,
                    "error": "image_read_failed_retry_once",
                    "result": broker.image_recovery_contract(),
                }
            else:
                raise
        if len(json.dumps(reply, separators=(",", ":")).encode("utf-8")) > self.max_frame_bytes:
            if tool == "read_generated_image":
                broker.commit_image_read(delivered=False)
                try:
                    broker.note_image_failure("reply_delivery_failed")
                except PreImagePolicyViolationError as error:
                    raise _pre_image_policy_error(broker, error) from None
            return {
                "version": PROTOCOL_VERSION,
                "type": "tool_reply",
                "id": call_id,
                "ok": False,
                "error": "image_read_failed_retry_once",
                "result": broker.image_recovery_contract(),
            }
        return reply

    def _send(self, stream: TextIO, frame: dict[str, object]) -> None:
        encoded = json.dumps(frame, separators=(",", ":"))
        if len(encoded.encode("utf-8")) > self.max_frame_bytes:
            raise ValueError("control frame exceeds configured bound")
        self._record_outbound(frame)
        stream.write(encoded + "\n")
        stream.flush()

    def _record_outbound(self, frame: dict[str, object]) -> None:
        self._append_audit("out", frame)

    def _record_inbound(self, frame: object) -> None:
        self._append_audit("in", frame)

    def _append_audit(self, direction: AuditDirection, frame: object) -> None:
        """Project control frames into the bounded lifecycle-audit vocabulary.

        Transcript deltas are validated by the protocol loop and retained in the
        native Pi session. Recording each streamed delta here duplicates that
        evidence and lets ordinary model output exhaust the lifecycle bound
        before a terminal control frame can be recorded.
        """
        if isinstance(frame, dict) and frame.get("type") == "transcript":
            return
        self._audit_sequence += 1
        if not isinstance(frame, dict):
            frame_type = "invalid"
            record = LifecycleAuditRecord("1.0", self._audit_sequence, direction, frame_type)
        else:
            frame_data = cast("dict[str, object]", frame)
            raw_frame_type = frame_data.get("type")
            frame_type = raw_frame_type if isinstance(raw_frame_type, str) else "invalid"
            raw_id = frame_data.get("id")
            raw_tool = frame_data.get("tool")
            raw_ok = frame_data.get("ok")
            raw_reason = frame_data.get("reason")
            run_id = (
                raw_id
                if frame_type in {"run_start", "run_started", "run_complete"}
                and isinstance(raw_id, str)
                else None
            )
            tool = raw_tool if frame_type == "tool_call" and isinstance(raw_tool, str) else None
            call_id = (
                raw_id
                if frame_type in {"tool_call", "tool_reply"} and isinstance(raw_id, str)
                else None
            )
            reply_ok = raw_ok if frame_type == "tool_reply" and isinstance(raw_ok, bool) else None
            terminal = (
                raw_reason if frame_type == "run_complete" and isinstance(raw_reason, str) else None
            )
            session: dict[str, object] = (
                cast("dict[str, object]", frame_data.get("session_evidence"))
                if frame_type == "run_complete"
                and isinstance(frame_data.get("session_evidence"), dict)
                else {}
            )
            record = LifecycleAuditRecord(
                "1.0",
                self._audit_sequence,
                direction,
                frame_type,
                run_id=run_id,
                tool=tool,
                call_id=call_id,
                reply_ok=reply_ok,
                terminal_category=terminal,
                reason_code=terminal,
                session_state=cast("str", session["state"])
                if isinstance(session.get("state"), str)
                else None,
                session_persisted=cast("bool", session["persisted"])
                if isinstance(session.get("persisted"), bool)
                else None,
                session_byte_count=_session_total(session),
                system_prompt_byte_count=_session_count(session, "system_prompt"),
                initial_prompt_byte_count=_session_count(session, "initial_prompt"),
                system_prompt_sha256=_session_hash(session, "system_prompt"),
                initial_prompt_sha256=_session_hash(session, "initial_prompt"),
            )
        if len(self._audit_records) >= self.max_audit_records:
            raise ValueError("adapter lifecycle audit exceeds record bound")
        self._audit_records.append(record)

    @staticmethod
    def _validate_run_start(run_id: str, frame: dict[str, object]) -> dict[str, object]:
        required = {"version", "type", "id", "prompt", "budget", "config"}
        if (
            set(frame) != required
            or frame.get("version") != PROTOCOL_VERSION
            or frame.get("type") != "run_start"
            or frame.get("id") != run_id
        ):
            raise ValueError("invalid v2 run_start frame")
        budget, config = frame["budget"], frame["config"]
        if (
            not isinstance(frame["prompt"], str)
            or len(frame["prompt"].encode("utf-8")) > 32 * 1024
            or not isinstance(budget, dict)
            or not isinstance(config, dict)
        ):
            raise ValueError("invalid v2 run_start frame")
        if set(budget) != {"maxTurns", "maxToolCalls", "timeoutMs"} or set(config) != {
            "promptMode",
            "answerType",
            "modelId",
            "thinkingLevel",
            "implementationDigests",
        }:
            raise ValueError("invalid v2 run_start frame")
        digests = config["implementationDigests"]
        if (
            config["promptMode"] not in {"visualization_forbidden", "visualization_encouraged"}
            or config["answerType"] not in {"boolean", "integer"}
            or config["modelId"] != "gpt-5.6-luna"
            or config["thinkingLevel"] != "medium"
            or not isinstance(digests, dict)
            or set(digests) != {"adapter", "scorer", "protocol"}
            or any(
                not isinstance(value, str) or not _is_digest(value) for value in digests.values()
            )
        ):
            raise ValueError("invalid v2 run_start frame")
        if (
            not isinstance(frame["id"], str)
            or not 0 < len(frame["id"]) <= 128
            or any(
                type(budget[key]) is not int for key in ("maxTurns", "maxToolCalls", "timeoutMs")
            )
        ):
            raise ValueError("invalid v2 run_start frame")
        if (
            not 1 <= budget["maxTurns"] <= 100
            or not 1 <= budget["maxToolCalls"] <= 100
            or not 1_000 <= budget["timeoutMs"] <= 900_000
        ):
            raise ValueError("invalid v2 run_start frame")
        return frame

    @staticmethod
    def _validate_run_started(frame: dict[str, object], run_id: str) -> None:
        if (
            set(frame) != {"version", "type", "id", "tools"}
            or frame.get("version") != PROTOCOL_VERSION
            or frame.get("id") != run_id
            or frame.get("tools") != list(TOOLS)
        ):
            raise ValueError("invalid run_started frame or tool inventory")

    @staticmethod
    def _validate_transcript(frame: dict[str, object]) -> None:
        if (
            set(frame) - {"version", "type", "event", "delta"}
            or frame.get("version") != PROTOCOL_VERSION
            or not isinstance(frame.get("event"), str)
            or ("delta" in frame and not isinstance(frame["delta"], str))
        ):
            raise ValueError("invalid transcript frame")

    @staticmethod
    def _validate_run_complete(frame: dict[str, object], run_id: str) -> SessionEvidence:
        required = {"version", "type", "id", "ok", "reason", "session_evidence"}
        if (
            (set(frame) != required and set(frame) != required | {"error"})
            or frame.get("version") != PROTOCOL_VERSION
            or frame.get("type") != "run_complete"
            or frame.get("id") != run_id
            or not isinstance(frame.get("ok"), bool)
            or frame.get("reason") not in TERMINAL_REASONS
            or ("error" in frame and not isinstance(frame["error"], str))
            or frame.get("ok") is not (frame.get("reason") == "submitted")
        ):
            raise ValueError("invalid run_complete frame")
        raw = frame["session_evidence"]
        if (
            not isinstance(raw, dict)
            or not set(raw)
            <= {"state", "persisted", "relative_path", "system_prompt", "initial_prompt"}
            or not {"state", "persisted"} <= set(raw)
        ):
            raise ValueError("invalid session evidence")
        state, persisted = raw["state"], raw["persisted"]
        relative_path, prompt = raw.get("relative_path"), raw.get("system_prompt")
        initial = raw.get("initial_prompt")
        if (
            ("relative_path" in raw and relative_path is None)
            or ("system_prompt" in raw and prompt is None)
            or ("initial_prompt" in raw and initial is None)
        ):
            raise ValueError("invalid session evidence")
        if state not in {"complete", "partial", "unavailable"} or not isinstance(persisted, bool):
            raise ValueError("invalid session evidence")
        if relative_path is not None and (
            not isinstance(relative_path, str)
            or SESSION_RELATIVE_PATH.fullmatch(relative_path) is None
        ):
            raise ValueError("invalid session evidence")
        parsed_prompt: SessionPromptEvidence | None = None
        if prompt is not None:
            if not isinstance(prompt, dict) or set(prompt) != {
                "relative_path",
                "byte_count",
                "sha256",
            }:
                raise ValueError("invalid session evidence")
            prompt_path = prompt["relative_path"]
            if prompt_path != "pi-prompt/system.txt":
                raise ValueError("invalid session evidence")
            if (
                not isinstance(prompt["byte_count"], int)
                or isinstance(prompt["byte_count"], bool)
                or prompt["byte_count"] < 0
                or not isinstance(prompt["sha256"], str)
                or not _is_sha256(prompt["sha256"])
            ):
                raise ValueError("invalid session evidence")
            parsed_prompt = SessionPromptEvidence(
                prompt_path, prompt["byte_count"], prompt["sha256"]
            )
        parsed_initial: SessionPromptEvidence | None = None
        if initial is not None:
            if (
                not isinstance(initial, dict)
                or set(initial) != {"relative_path", "byte_count", "sha256"}
                or initial["relative_path"] != "pi-prompt/initial.txt"
            ):
                raise ValueError("invalid session evidence")
            if (
                not isinstance(initial["byte_count"], int)
                or isinstance(initial["byte_count"], bool)
                or initial["byte_count"] < 0
                or not isinstance(initial["sha256"], str)
                or not _is_sha256(initial["sha256"])
            ):
                raise ValueError("invalid session evidence")
            parsed_initial = SessionPromptEvidence(
                initial["relative_path"], initial["byte_count"], initial["sha256"]
            )
        if state in {"complete", "partial"}:
            if (
                not persisted
                or relative_path is None
                or parsed_prompt is None
                or parsed_initial is None
            ):
                raise ValueError("invalid session evidence")
        elif (
            persisted
            or relative_path is not None
            or parsed_prompt is not None
            or parsed_initial is not None
        ):
            raise ValueError("invalid session evidence")
        return SessionEvidence(state, persisted, relative_path, parsed_prompt, parsed_initial)


def _queue_lines(stream: TextIO, output: queue.Queue[str | None]) -> None:
    try:
        for line in stream:
            output.put(line)
    finally:
        output.put(None)


def _check_cancel(cancel_requested: threading.Event) -> None:
    if cancel_requested.is_set():
        raise ExecutionInterrupted


def _terminate_then_kill(process: object, grace: float) -> None:
    poll = getattr(process, "poll", lambda: None)
    if poll() is None:
        terminate = getattr(process, "terminate", None)
        if terminate is not None:
            terminate()
        try:
            process.wait(timeout=grace)  # type: ignore[attr-defined]
        except subprocess.TimeoutExpired:
            kill = getattr(process, "kill", None)
            if kill is not None:
                kill()
            process.wait(timeout=grace)  # type: ignore[attr-defined]


def _quiesce_readers(
    process: object,
    stdout_thread: threading.Thread | None,
    stderr_thread: threading.Thread | None,
    grace: float,
) -> bool:
    """Join readers, closing blocked pipes once before the final join attempt."""
    threads = (stdout_thread, stderr_thread)
    for thread in threads:
        if thread is not None and thread.ident is not None:
            thread.join(timeout=grace)
    alive = [
        (thread, getattr(process, stream_name, None))
        for thread, stream_name in zip(threads, ("stdout", "stderr"), strict=True)
        if thread is not None and thread.is_alive()
    ]
    for _, stream in alive:
        close = getattr(stream, "close", None)
        if callable(close):
            close()
    for thread, _ in alive:
        thread.join(timeout=grace)
    return not any(thread.is_alive() for thread, _ in alive)


def _tool_result(tool: str, result: object) -> object:
    if tool == "read_generated_image":
        return {"mime": "image/png", "data": result["data"]}  # type: ignore[index]
    return _truncate_utf8(json.dumps(_jsonable(result), separators=(",", ":")), 16_384)


def _stable_error_code(error: Exception) -> str:
    if isinstance(error, PolicyViolationError):
        return error.code
    if isinstance(error, ValueError):
        return "tool_invalid_arguments"
    return "tool_execution_failed"


def _pre_image_policy_error(
    broker: CaseBroker, error: PreImagePolicyViolationError
) -> AdapterRunError:
    return AdapterRunError(
        "pre_image_policy_violation",
        terminal_reason="pre_image_policy_violation",
        policy_telemetry=broker.policy_telemetry(error.failure_kind, error.category),
    )


def _jsonable(value: object) -> object:
    if isinstance(value, Enum):
        return value.value
    if hasattr(value, "__dict__"):
        return {key: _jsonable(child) for key, child in value.__dict__.items()}
    if isinstance(value, tuple):
        return [_jsonable(child) for child in value]
    return value


def _truncate_utf8(value: str, limit: int) -> str:
    return value.encode("utf-8", errors="replace")[:limit].decode("utf-8", errors="ignore")


def _is_digest(value: str) -> bool:
    name, separator, digest = value.partition("@sha256:")
    return (
        bool(name)
        and len(digest) == 64
        and all(character in "0123456789abcdef" for character in digest)
    )


def _is_sha256(value: str) -> bool:
    return len(value) == 64 and all(character in "0123456789abcdef" for character in value)


def _session_count(session: dict[str, object], name: str) -> int | None:
    value = session.get(name)
    if not isinstance(value, dict):
        return None
    count = value.get("byte_count")
    return count if type(count) is int and 0 <= count <= 16 * 1024 * 1024 else None


def _session_total(session: dict[str, object]) -> int | None:
    counts = [_session_count(session, name) for name in ("system_prompt", "initial_prompt")]
    if any(count is None for count in counts):
        return None
    total = sum(cast("int", count) for count in counts)
    return total if total <= 32 * 1024 * 1024 else None


def _session_hash(session: dict[str, object], name: str) -> str | None:
    value = session.get(name)
    if not isinstance(value, dict):
        return None
    digest = value.get("sha256")
    return digest if isinstance(digest, str) and _is_sha256(digest) else None
