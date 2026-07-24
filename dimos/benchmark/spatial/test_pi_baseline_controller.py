# Copyright 2026 Dimensional Inc.
import io
import json
import os
from pathlib import Path
import queue
import subprocess
import threading
from threading import Event
import time

import pytest

from dimos.benchmark.spatial.models import AnswerType
from dimos.benchmark.spatial.pi_baseline.broker import CaseBroker, PostImagePolicyViolationError
import dimos.benchmark.spatial.pi_baseline.controller as controller_module
from dimos.benchmark.spatial.pi_baseline.controller import (
    TOOLS,
    AdapterController,
    AdapterRunError,
)
from dimos.benchmark.spatial.pi_baseline.lifecycle_audit import LifecycleAuditRecord
from dimos.benchmark.spatial.pi_baseline.prompts import (
    build_prompt_pair,
    make_parity_manifest,
    validate_parity,
)
from dimos.benchmark.spatial.pi_baseline.topology import PinnedDirectory
from dimos.benchmark.spatial.pi_baseline.transaction import AnswerTransaction

from .test_pi_baseline_broker import _Case, _png


def _operation_pair() -> tuple[threading.Event, threading.Lock]:
    return threading.Event(), threading.Lock()


def _session_evidence() -> dict[str, object]:
    return {
        "state": "complete",
        "persisted": True,
        "relative_path": "pi-session/session.jsonl",
        "system_prompt": {
            "relative_path": "pi-prompt/system.txt",
            "byte_count": 1,
            "sha256": "a" * 64,
        },
        "initial_prompt": {
            "relative_path": "pi-prompt/initial.txt",
            "byte_count": 2,
            "sha256": "b" * 64,
        },
    }


_AUDIT_FIELDS = frozenset(LifecycleAuditRecord.__dataclass_fields__)


def _audit(tmp_path: Path) -> tuple[bytes, list[dict[str, object]]]:
    path = tmp_path / "adapter-lifecycle-audit.v1.jsonl"
    assert path.is_file()
    assert path.stat().st_mode & 0o077 == 0
    payload = path.read_bytes()
    assert len(payload) <= 2048 * 2048
    records = [json.loads(line) for line in payload.splitlines()]
    assert records
    assert all(set(record) <= _AUDIT_FIELDS for record in records)
    assert [record["sequence"] for record in records] == list(range(1, len(records) + 1))
    assert all(record["schema_version"] == "1.0" for record in records)
    return payload, records


def _adapter_command(tmp_path: Path) -> tuple[str, str]:
    adapter = tmp_path / "adapter.js"
    adapter.touch()
    return ("/usr/bin/node", str(adapter))


def test_api_key_environment_excludes_oauth_and_ambient_credentials(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    dotenv = tmp_path / ".env"
    dotenv.write_text("OPENAI_API_KEY=file-key\nUNRELATED=private\n", encoding="utf-8")
    dotenv.chmod(0o600)
    monkeypatch.setenv("OPENAI_API_KEY", "ambient-key")
    controller = AdapterController(
        _adapter_command(tmp_path),
        dotenv,
        tmp_path / "transcript",
        auth_mode="openai-api-key",
    )

    env = controller._adapter_environment()

    assert env["OPENAI_API_KEY"] == "file-key"
    assert env["PI_SPATIAL_AUTH_MODE"] == "openai-api-key"
    assert "PI_SPATIAL_AUTH_PATH" not in env
    assert "UNRELATED" not in env
    controller.close()


class _Process:
    def __init__(self, frames: list[dict[str, object]]) -> None:
        self.stdin = io.StringIO()
        self.stdout = io.StringIO("".join(json.dumps(frame) + "\n" for frame in frames))
        self.stderr = io.StringIO()
        self.wait_calls = 0

    def wait(self, timeout: float) -> int:
        self.wait_calls += 1
        return 0


def test_prompt_pair_has_only_visualization_delta() -> None:
    pair = build_prompt_pair()
    assert "online information or services" in pair.visualization_forbidden
    assert "under /input" in pair.visualization_forbidden
    assert "only under /work" in pair.visualization_forbidden
    assert "sandbox_exec, read_generated_image, and submit_answer" in pair.visualization_forbidden
    assert "submit_answer exactly once" in pair.visualization_forbidden
    assert "package installation is allowed" in pair.visualization_forbidden
    assert "relative path" in pair.visualization_encouraged
    assert "regular non-symlink PNG" in pair.visualization_encouraged
    assert "byte, width, height, and pixel limits" in pair.visualization_encouraged
    assert "Visualization is forbidden" in pair.visualization_forbidden
    left = make_parity_manifest(
        model={"m": 1},
        tools={"t": 1},
        runtime={"r": 1},
        dependencies={"d": 1},
        mode="visualization_forbidden",
        prompt=pair.visualization_forbidden,
    )
    right = make_parity_manifest(
        model={"m": 1},
        tools={"t": 1},
        runtime={"r": 1},
        dependencies={"d": 1},
        mode="visualization_encouraged",
        prompt=pair.visualization_encouraged,
    )
    validate_parity(left, right)


def test_post_image_policy_is_terminal_not_a_failed_tool_reply(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
    )
    monkeypatch.setattr(
        broker,
        "dispatch",
        lambda tool, params: (_ for _ in ()).throw(
            PostImagePolicyViolationError("post_image_policy_violation")
        ),
    )
    controller = AdapterController(
        _adapter_command(tmp_path), tmp_path / "auth", tmp_path / "transcript"
    )
    with pytest.raises(AdapterRunError, match="post_image_policy_violation"):
        controller._tool_reply(
            {
                "version": 2,
                "type": "tool_call",
                "id": "tool-1",
                "tool": "sandbox_exec",
                "params": {"command": "secret"},
            },
            broker,
            Event(),
        )
    with pytest.raises(ValueError, match="transcript"):
        AdapterController._validate_transcript(
            {"version": 2, "type": "transcript", "event": "turn_end", "thinkingLevel": "medium"}
        )
    with pytest.raises(ValueError, match="run_complete"):
        AdapterController._validate_run_complete(
            {
                "version": 2,
                "type": "run_complete",
                "id": "run",
                "ok": True,
                "thinkingLevel": "medium",
            },
            "run",
        )
    controller.close()


def test_controller_deadline_normalizes_a_blocking_sandbox_tool(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-forbidden",
    )

    def blocking_dispatch(tool: str, params: dict[str, object], **kwargs: object) -> object:
        del tool, params, kwargs
        time.sleep(0.01)
        return {"stdout": "", "stderr": "", "exit_code": 0}

    monkeypatch.setattr(broker, "dispatch", blocking_dispatch)
    controller = AdapterController(
        _adapter_command(tmp_path), tmp_path / "auth", tmp_path / "transcript"
    )
    with pytest.raises(AdapterRunError, match="adapter_host_deadline"):
        controller._tool_reply(
            {
                "version": 2,
                "type": "tool_call",
                "id": "tool-1",
                "tool": "sandbox_exec",
                "params": {"command": "true"},
            },
            broker,
            Event(),
            time.monotonic() + 0.001,
        )
    controller.close()


def test_oversized_image_result_first_failure_returns_recovery_response(
    tmp_path: Path,
) -> None:
    (tmp_path / "image.png").write_bytes(_png())
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
    )
    controller = AdapterController(
        _adapter_command(tmp_path),
        tmp_path / "auth",
        tmp_path / "transcript",
        max_frame_bytes=128,
    )
    reply = controller._tool_reply(
        {
            "version": 2,
            "type": "tool_call",
            "id": "tool-1",
            "tool": "read_generated_image",
            "params": {"path": "image.png"},
        },
        broker,
        Event(),
    )
    assert reply["ok"] is False
    assert reply["error"] == "image_read_failed_retry_once"
    assert reply["result"] == {
        "category": "reply_delivery_failed",
        "sandbox_attempts": 0,
        "image_attempts": 1,
    }
    controller.close()


def test_oversized_image_result_second_failure_is_safe_terminal_error(
    tmp_path: Path,
) -> None:
    (tmp_path / "image.png").write_bytes(_png())
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
    )
    controller = AdapterController(
        _adapter_command(tmp_path),
        tmp_path / "auth",
        tmp_path / "transcript",
        max_frame_bytes=128,
    )
    tool_call = {
        "version": 2,
        "type": "tool_call",
        "id": "tool-1",
        "tool": "read_generated_image",
        "params": {"path": "image.png"},
    }
    first = controller._tool_reply(tool_call, broker, Event())
    assert first["ok"] is False
    with pytest.raises(AdapterRunError, match="pre_image_policy_violation") as raised:
        controller._tool_reply(tool_call, broker, Event())
    assert raised.value.policy_telemetry is not None
    assert raised.value.policy_telemetry.failure_kind == "image_attempts_exhausted"
    assert raised.value.policy_telemetry.image_failure_category == "reply_delivery_failed"
    assert raised.value.terminal_reason == "pre_image_policy_violation"
    assert raised.value.policy_telemetry.image_attempts == 2
    assert raised.value.policy_telemetry.image_delivered is False
    assert raised.value.policy_telemetry.submission_attempted is False
    assert "image.png" not in str(raised.value)
    controller.close()


def test_rejected_image_result_second_failure_is_safe_terminal_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    (tmp_path / "image.png").write_bytes(_png())
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
    )
    controller = AdapterController(
        _adapter_command(tmp_path), tmp_path / "auth", tmp_path / "transcript"
    )
    monkeypatch.setattr(
        controller_module,
        "_tool_result",
        lambda tool, result: (_ for _ in ()).throw(ValueError("raw image content")),
    )
    tool_call = {
        "version": 2,
        "type": "tool_call",
        "id": "tool-1",
        "tool": "read_generated_image",
        "params": {"path": "image.png"},
    }
    first = controller._tool_reply(tool_call, broker, Event())
    assert first["error"] == "image_read_failed_retry_once"
    with pytest.raises(AdapterRunError, match="pre_image_policy_violation") as raised:
        controller._tool_reply(tool_call, broker, Event())
    assert raised.value.policy_telemetry is not None
    assert raised.value.policy_telemetry.failure_kind == "image_attempts_exhausted"
    assert raised.value.policy_telemetry.image_failure_category == "adapter_result_rejected"
    assert "raw image content" not in str(raised.value)
    controller.close()


def test_controller_runs_closed_dialogue_and_rejects_case_mismatch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    frames = [
        {
            "version": 2,
            "type": "run_started",
            "id": "run",
            "tools": ["sandbox_exec", "read_generated_image", "submit_answer"],
        },
        {"version": 2, "type": "transcript", "event": "turn_end", "delta": "DELTA_SENTINEL"},
        {
            "version": 2,
            "type": "tool_call",
            "id": "tool-1",
            "tool": "submit_answer",
            "params": {"answer": True},
        },
        {
            "version": 2,
            "type": "run_complete",
            "id": "run",
            "ok": True,
            "reason": "submitted",
            "session_evidence": _session_evidence(),
        },
    ]
    process = _Process(frames)
    captured: dict[str, object] = {}

    def launch(command: tuple[str, ...], **kwargs: object) -> _Process:
        captured.update(kwargs)
        return process

    monkeypatch.setattr(subprocess, "Popen", launch)
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-forbidden",
    )
    transcript = tmp_path / "adapter.transcript.ndjson"
    run_start = {
        "version": 2,
        "type": "run_start",
        "id": "run",
        "prompt": "PROMPT_SENTINEL",
        "budget": {"maxTurns": 2, "maxToolCalls": 2, "timeoutMs": 1000},
        "config": {
            "promptMode": "visualization_forbidden",
            "answerType": "boolean",
            "modelId": "gpt-5.6-luna",
            "thinkingLevel": "medium",
            "implementationDigests": {
                "adapter": "adapter@sha256:" + "a" * 64,
                "scorer": "scorer@sha256:" + "b" * 64,
                "protocol": "protocol@sha256:" + "c" * 64,
            },
        },
    }
    terminal = AdapterController(
        _adapter_command(tmp_path), tmp_path / "private-auth", transcript
    ).run("run", broker, run_start, *_operation_pair())
    assert terminal.ok
    assert terminal.tool_replies[0] == {
        "version": 2,
        "type": "tool_reply",
        "id": "tool-1",
        "ok": True,
        "result": '{"accepted":true,"instance_id":"instance","answer_type":"boolean"}',
    }
    assert captured["env"]["PI_SPATIAL_AUTH_PATH"] == str(tmp_path / "private-auth")
    assert captured["env"]["PI_SPATIAL_AUTH_MODE"] == "codex-oauth"
    assert "OPENAI_API_KEY" not in captured["env"]
    assert captured["env"]["PI_SPATIAL_SESSION_DIR"] == "pi-session"
    assert captured["cwd"] == f"/proc/self/fd/{captured['pass_fds'][0]}"
    assert terminal.session_evidence.relative_path == "pi-session/session.jsonl"
    audit_bytes, audit_records = _audit(tmp_path)
    assert not transcript.exists()
    assert b"PROMPT_SENTINEL" not in audit_bytes
    assert b"DELTA_SENTINEL" not in audit_bytes
    assert b"ARGS_SENTINEL" not in audit_bytes
    assert b"private-auth" not in audit_bytes
    assert any(
        record.get("frame_type") == "tool_call"
        and record.get("tool") == "submit_answer"
        and record.get("call_id") == "tool-1"
        for record in audit_records
    )
    assert any(
        record.get("frame_type") == "tool_reply" and record.get("reply_ok") is True
        for record in audit_records
    )
    assert any(
        record.get("frame_type") == "run_complete"
        and record.get("terminal_category") == "submitted"
        and record.get("session_state") == "complete"
        for record in audit_records
    )
    assert broker.transaction.prediction is not None

    mismatch = _Process(
        [
            {
                "version": 2,
                "type": "run_started",
                "id": "other",
                "tools": ["sandbox_exec", "read_generated_image", "submit_answer"],
            }
        ]
    )
    monkeypatch.setattr(subprocess, "Popen", lambda *args, **kwargs: mismatch)
    with pytest.raises(ValueError, match="run_started"):
        AdapterController(
            _adapter_command(tmp_path), tmp_path / "auth", tmp_path / "bad.ndjson"
        ).run("run", broker, run_start, *_operation_pair())


@pytest.mark.parametrize(
    "reason", ["max_turns", "max_tool_calls", "timeout", "session_error", "protocol_error"]
)
def test_terminal_failure_retains_only_allowlisted_reason(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, reason: str
) -> None:
    process = _Process(
        [
            {
                "version": 2,
                "type": "run_started",
                "id": "run",
                "tools": ["sandbox_exec", "read_generated_image", "submit_answer"],
            },
            {
                "version": 2,
                "type": "run_complete",
                "id": "run",
                "ok": False,
                "reason": reason,
                "error": "secret/path",
                "session_evidence": _session_evidence(),
            },
        ]
    )
    process.stderr = io.StringIO("x" * 100_000)
    monkeypatch.setattr(subprocess, "Popen", lambda *args, **kwargs: process)
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-forbidden",
    )
    start = {
        "version": 2,
        "type": "run_start",
        "id": "run",
        "prompt": "offline",
        "budget": {"maxTurns": 1, "maxToolCalls": 1, "timeoutMs": 1000},
        "config": {
            "promptMode": "visualization_forbidden",
            "answerType": "boolean",
            "modelId": "gpt-5.6-luna",
            "thinkingLevel": "medium",
            "implementationDigests": {
                "adapter": "adapter@sha256:" + "a" * 64,
                "scorer": "scorer@sha256:" + "b" * 64,
                "protocol": "protocol@sha256:" + "c" * 64,
            },
        },
    }
    with pytest.raises(AdapterRunError, match="adapter_run_failed") as raised:
        AdapterController(
            _adapter_command(tmp_path),
            tmp_path / "auth",
            tmp_path / "transcript",
            max_stderr_bytes=128,
        ).run("run", broker, start, *_operation_pair())
    assert raised.value.terminal_reason == reason
    assert (tmp_path / "transcript.stderr.log").stat().st_size <= 128
    audit_bytes, audit_records = _audit(tmp_path)
    assert not (tmp_path / "adapter.transcript.ndjson").exists()
    assert b"secret/path" not in audit_bytes
    assert any(record.get("terminal_category") == reason for record in audit_records)


def test_transcript_symlink_in_pinned_leaf_fails_closed(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    escaped = tmp_path / "escaped-transcript.ndjson"
    escaped.write_bytes(b"must remain unchanged")
    transcript_root = tmp_path / "private"
    transcript_root.mkdir()
    transcript_root.chmod(0o700)
    (transcript_root / "adapter-lifecycle-audit.v1.jsonl").symlink_to(escaped)
    process = _Process([{"version": 2, "type": "run_started", "id": "run", "tools": list(TOOLS)}])
    monkeypatch.setattr(subprocess, "Popen", lambda *args, **kwargs: process)
    pinned = PinnedDirectory.open(transcript_root)
    try:
        broker = CaseBroker(
            "case",
            _Case(tmp_path),
            AnswerTransaction("instance", AnswerType.BOOLEAN),
            "visualization-forbidden",
        )
        with pytest.raises(AdapterRunError, match="adapter_eof"):
            AdapterController(_adapter_command(tmp_path), tmp_path / "auth", pinned).run(
                "run", broker, _minimal_start(), *_operation_pair()
            )
        assert escaped.read_bytes() == b"must remain unchanged"
        assert (transcript_root / "adapter-lifecycle-audit.v1.jsonl").is_symlink()
    finally:
        pinned.close()


def test_thread_start_failure_cleans_process_and_owned_transcript_fd(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    class Process(_Process):
        def __init__(self) -> None:
            super().__init__([])
            self.terminated = False
            self.killed = False

        def poll(self) -> None:
            return None

        def terminate(self) -> None:
            self.terminated = True

        def kill(self) -> None:
            self.killed = True

    process = Process()
    monkeypatch.setattr(subprocess, "Popen", lambda *args, **kwargs: process)
    original_start = threading.Thread.start

    def fail_start(thread: threading.Thread) -> None:
        raise RuntimeError("forced thread-start failure")

    monkeypatch.setattr(threading.Thread, "start", fail_start)
    controller = AdapterController(
        _adapter_command(tmp_path), tmp_path / "auth", tmp_path / "transcript"
    )
    owned_fd = controller.transcript_root.fd
    with pytest.raises(RuntimeError, match="forced thread-start failure"):
        controller.run(
            "run",
            CaseBroker(
                "case",
                _Case(tmp_path),
                AnswerTransaction("instance", AnswerType.BOOLEAN),
                "visualization-forbidden",
            ),
            _minimal_start(),
            *_operation_pair(),
        )
    assert process.terminated or process.killed
    with pytest.raises(OSError):
        os.fstat(owned_fd)
    monkeypatch.setattr(threading.Thread, "start", original_start)


def test_thread_constructor_failure_waits_and_closes_owned_transcript_fd(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    process = _Process([])
    monkeypatch.setattr(subprocess, "Popen", lambda *args, **kwargs: process)

    def fail_constructor(*args: object, **kwargs: object) -> threading.Thread:
        raise RuntimeError("forced thread construction failure")

    monkeypatch.setattr(threading, "Thread", fail_constructor)
    controller = AdapterController(
        _adapter_command(tmp_path), tmp_path / "auth", tmp_path / "transcript"
    )
    owned_fd = controller.transcript_root.fd
    with pytest.raises(RuntimeError, match="forced thread construction failure"):
        controller.run(
            "run",
            CaseBroker(
                "case",
                _Case(tmp_path),
                AnswerTransaction("instance", AnswerType.BOOLEAN),
                "visualization-forbidden",
            ),
            _minimal_start(),
            *_operation_pair(),
        )
    assert process.wait_calls
    with pytest.raises(OSError):
        os.fstat(owned_fd)


def test_second_reader_start_failure_waits_and_closes_owned_transcript_fd(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    process = _Process([])
    monkeypatch.setattr(subprocess, "Popen", lambda *args, **kwargs: process)
    original_start = threading.Thread.start
    start_calls = 0

    def fail_second_start(thread: threading.Thread) -> None:
        nonlocal start_calls
        start_calls += 1
        if start_calls == 1:
            original_start(thread)
            return
        raise RuntimeError("forced second reader start failure")

    monkeypatch.setattr(threading.Thread, "start", fail_second_start)
    controller = AdapterController(
        _adapter_command(tmp_path), tmp_path / "auth", tmp_path / "transcript"
    )
    owned_fd = controller.transcript_root.fd
    with pytest.raises(RuntimeError, match="forced second reader start failure"):
        controller.run(
            "run",
            CaseBroker(
                "case",
                _Case(tmp_path),
                AnswerTransaction("instance", AnswerType.BOOLEAN),
                "visualization-forbidden",
            ),
            _minimal_start(),
            *_operation_pair(),
        )
    assert process.wait_calls
    with pytest.raises(OSError):
        os.fstat(owned_fd)


def test_cleanup_failure_still_waits_and_closes_owned_transcript_fd(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    process = _Process([])
    monkeypatch.setattr(subprocess, "Popen", lambda *args, **kwargs: process)
    monkeypatch.setattr(
        controller_module,
        "_terminate_then_kill",
        lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("forced cleanup failure")),
    )
    controller = AdapterController(
        _adapter_command(tmp_path), tmp_path / "auth", tmp_path / "transcript"
    )
    owned_fd = controller.transcript_root.fd
    with pytest.raises(ValueError, match="missing run_started"):
        controller.run(
            "run",
            CaseBroker(
                "case",
                _Case(tmp_path),
                AnswerTransaction("instance", AnswerType.BOOLEAN),
                "visualization-forbidden",
            ),
            _minimal_start(),
            *_operation_pair(),
        )
    assert process.wait_calls
    with pytest.raises(OSError):
        os.fstat(owned_fd)


def test_cleanup_preserves_caller_owned_transcript_fd(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    process = _Process([])
    monkeypatch.setattr(subprocess, "Popen", lambda *args, **kwargs: process)
    monkeypatch.setattr(
        controller_module,
        "_terminate_then_kill",
        lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("forced cleanup failure")),
    )
    private = tmp_path / "private"
    private.mkdir()
    private.chmod(0o700)
    pinned = PinnedDirectory.open(private)
    try:
        with pytest.raises(ValueError, match="missing run_started"):
            AdapterController(_adapter_command(tmp_path), tmp_path / "auth", pinned).run(
                "run",
                CaseBroker(
                    "case",
                    _Case(tmp_path),
                    AnswerTransaction("instance", AnswerType.BOOLEAN),
                    "visualization-forbidden",
                ),
                _minimal_start(),
                *_operation_pair(),
            )
        os.fstat(pinned.fd)
        assert process.wait_calls
    finally:
        pinned.close()


def test_reader_quiescence_closes_a_blocked_real_reader_before_reporting_failure() -> None:
    entered = Event()
    release = Event()
    closed = Event()

    class BlockingReader:
        def __iter__(self) -> "BlockingReader":
            entered.set()
            release.wait(1)
            return self

        def __next__(self) -> str:
            raise StopIteration

        def close(self) -> None:
            closed.set()

    stream = BlockingReader()
    reader = threading.Thread(target=controller_module._queue_lines, args=(stream, queue.Queue()))
    reader.start()
    assert entered.wait(1)
    process = type("Process", (), {"stdout": stream, "stderr": io.StringIO()})()
    assert not controller_module._quiesce_readers(process, reader, None, 0.01)
    assert closed.is_set()
    release.set()
    reader.join(1)
    assert not reader.is_alive()


def test_peer_frames_reject_adapter_only_metadata() -> None:
    with pytest.raises(ValueError, match="run_started"):
        AdapterController._validate_run_started(
            {
                "version": 2,
                "type": "run_started",
                "id": "run",
                "tools": list(("sandbox_exec", "read_generated_image", "submit_answer")),
                "model": "gpt-5.6-luna",
            },
            "run",
        )


def test_session_evidence_states_and_paths_are_strict() -> None:
    frame = {
        "version": 2,
        "type": "run_complete",
        "id": "run",
        "ok": False,
        "reason": "session_error",
        "session_evidence": {"state": "unavailable", "persisted": False},
    }
    assert AdapterController._validate_run_complete(frame, "run").state == "unavailable"
    frame["session_evidence"] = {**_session_evidence(), "relative_path": "../escape"}
    with pytest.raises(ValueError, match="session evidence"):
        AdapterController._validate_run_complete(frame, "run")


def test_integer_answer_type_matches_corpus_wire_value() -> None:
    frame = {
        "version": 2,
        "type": "run_start",
        "id": "run",
        "prompt": "offline",
        "budget": {"maxTurns": 1, "maxToolCalls": 1, "timeoutMs": 1000},
        "config": {
            "promptMode": "visualization_forbidden",
            "answerType": "integer",
            "modelId": "gpt-5.6-luna",
            "thinkingLevel": "medium",
            "implementationDigests": {
                "adapter": "adapter@sha256:" + "a" * 64,
                "scorer": "scorer@sha256:" + "b" * 64,
                "protocol": "protocol@sha256:" + "c" * 64,
            },
        },
    }
    assert AdapterController._validate_run_start("run", frame)["config"]["answerType"] == "integer"  # type: ignore[index]


def test_controller_returns_stable_policy_errors(tmp_path: Path) -> None:
    controller = AdapterController(
        _adapter_command(tmp_path), tmp_path / "auth", tmp_path / "transcript"
    )
    encouraged = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-encouraged",
    )
    forbidden = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-forbidden",
    )
    submit = {
        "version": 2,
        "type": "tool_call",
        "id": "s",
        "tool": "submit_answer",
        "params": {"answer": True},
    }
    image = {
        "version": 2,
        "type": "tool_call",
        "id": "i",
        "tool": "read_generated_image",
        "params": {"path": "x.png"},
    }
    with pytest.raises(AdapterRunError, match="pre_image_policy_violation"):
        controller._tool_reply(submit, encouraged, Event())
    assert controller._tool_reply(image, forbidden, Event())["error"] == "visualization_forbidden"


def _minimal_start() -> dict[str, object]:
    return {
        "version": 2,
        "type": "run_start",
        "id": "run",
        "prompt": "offline",
        "budget": {"maxTurns": 1, "maxToolCalls": 1, "timeoutMs": 1000},
        "config": {
            "promptMode": "visualization_forbidden",
            "answerType": "boolean",
            "modelId": "gpt-5.6-luna",
            "thinkingLevel": "medium",
            "implementationDigests": {
                "adapter": "adapter@sha256:" + "a" * 64,
                "scorer": "scorer@sha256:" + "b" * 64,
                "protocol": "protocol@sha256:" + "c" * 64,
            },
        },
    }


def test_controller_rejects_premature_successful_completion(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    process = _Process(
        [
            {"version": 2, "type": "run_started", "id": "run", "tools": list(TOOLS)},
            {
                "version": 2,
                "type": "run_complete",
                "id": "run",
                "ok": True,
                "reason": "submitted",
                "session_evidence": _session_evidence(),
            },
        ]
    )
    monkeypatch.setattr(subprocess, "Popen", lambda *args, **kwargs: process)
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-forbidden",
    )
    with pytest.raises(AdapterRunError, match="without_answer"):
        AdapterController(
            _adapter_command(tmp_path), tmp_path / "auth", tmp_path / "transcript"
        ).run("run", broker, _minimal_start(), *_operation_pair())


def test_transcript_volume_does_not_exhaust_lifecycle_audit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    frames: list[dict[str, object]] = [
        {"version": 2, "type": "run_started", "id": "run", "tools": list(TOOLS)},
        *[
            {"version": 2, "type": "transcript", "event": "message_update", "delta": "x"}
            for _ in range(2049)
        ],
        {
            "version": 2,
            "type": "tool_call",
            "id": "tool-1",
            "tool": "submit_answer",
            "params": {"answer": True},
        },
        {
            "version": 2,
            "type": "run_complete",
            "id": "run",
            "ok": True,
            "reason": "submitted",
            "session_evidence": _session_evidence(),
        },
    ]
    monkeypatch.setattr(subprocess, "Popen", lambda *args, **kwargs: _Process(frames))
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-forbidden",
    )

    terminal = AdapterController(
        _adapter_command(tmp_path), tmp_path / "auth", tmp_path / "transcript"
    ).run("run", broker, _minimal_start(), *_operation_pair())

    assert terminal.ok
    _, records = _audit(tmp_path)
    assert all(record["frame_type"] != "transcript" for record in records)
    assert any(record["frame_type"] == "run_complete" for record in records)


def test_controller_accepts_exhausted_budget_as_failed_terminal(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    process = _Process(
        [
            {"version": 2, "type": "run_started", "id": "run", "tools": list(TOOLS)},
            {"version": 2, "type": "transcript", "event": "turn_end"},
            {
                "version": 2,
                "type": "run_complete",
                "id": "run",
                "ok": False,
                "reason": "max_turns",
                "error": "private",
                "session_evidence": _session_evidence(),
            },
        ]
    )
    monkeypatch.setattr(subprocess, "Popen", lambda *args, **kwargs: process)
    broker = CaseBroker(
        "case",
        _Case(tmp_path),
        AnswerTransaction("instance", AnswerType.BOOLEAN),
        "visualization-forbidden",
    )
    with pytest.raises(AdapterRunError, match="adapter_run_failed"):
        AdapterController(
            _adapter_command(tmp_path), tmp_path / "auth", tmp_path / "transcript"
        ).run("run", broker, _minimal_start(), *_operation_pair())
    audit_bytes, audit_records = _audit(tmp_path)
    assert not (tmp_path / "adapter.transcript.ndjson").exists()
    assert b"private" not in audit_bytes
    assert any(record.get("terminal_category") == "max_turns" for record in audit_records)
